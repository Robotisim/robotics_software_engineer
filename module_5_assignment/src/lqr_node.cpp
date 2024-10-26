#include <angles/angles.h>
#include <cmath>
#include <lqr_node.hpp>
#include <rclcpp/logging.hpp>
#include <tuple>
#include <vector>

input input_old = input(0, 0);

LqrNode::LqrNode()
    : Node("LqrNode"), dt_(0.03), tolerance(0.020), end_controller(false),
      max_linear_velocity(0.05), max_angular_velocity(M_PI / 2),
      current_waypoint(0), odom_received_(false) // Ensure current_waypoint is initialized to 0
{
  robot_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&LqrNode::robotPoseCallback, this, std::placeholders::_1));

  control_input_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  control_loop_timer_ = this->create_wall_timer(std::chrono::milliseconds(30),
                                                std::bind(&LqrNode::controlLoopCallback, this));
                                                
  goal_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("goal_markers", 10);
  path_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("path_markers", 10);

  Q_ << 1, 0, 0, 0, 1, 0, 0, 0, 1;

  R_ << 0.25, 0, 0, 0.25;
  lqr_ = std::make_unique<LQR>(Q_, R_, 100);

  waypoints_ = {State(1, 1, M_PI / 4), State(2, 7, M_PI / 2),
                State(3, 9, M_PI / 5), State(4, 6, 3 * M_PI / 2),
                State(-1, 0, 0), State(-3,0,2* M_PI/2),
                State(-5, 0, 4 * M_PI/2), State(-4,0,-4 * M_PI/4)};
  actual_state_ = State(0, 0, 0);

  optimiseHeading(waypoints_);
}

void LqrNode::robotPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  actual_state_ =
      State(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
  odom_received_ = true;

  // Update the path markers
  updatePathMarkers();
}

void LqrNode::updatePathMarkers()
{
  visualization_msgs::msg::MarkerArray marker_array;
  int id = 0;
  for (const auto &state : waypoints_)
  {
    visualization_msgs::msg::Marker path_marker;
    path_marker.header.frame_id = "map";  // Ensure this frame exists
    path_marker.header.stamp = this->now();
    path_marker.ns = "path";
    path_marker.id = id++;
    path_marker.type = visualization_msgs::msg::Marker::SPHERE;  // Use SPHERE
    path_marker.action = visualization_msgs::msg::Marker::ADD;
    
    // Set marker position
    path_marker.pose.position.x = state.x;
    path_marker.pose.position.y = state.y;
    path_marker.pose.position.z = 0.0;
    
    // Set orientation to identity quaternion
    path_marker.pose.orientation.x = 0.0;
    path_marker.pose.orientation.y = 0.0;
    path_marker.pose.orientation.z = 0.0;
    path_marker.pose.orientation.w = 1.0;

    // Set the scale (non-zero)
    path_marker.scale.x = 0.2;
    path_marker.scale.y = 0.2;
    path_marker.scale.z = 0.2;

    // Set color
    path_marker.color.r = 0.0;
    path_marker.color.g = 1.0;  // Green color
    path_marker.color.b = 0.0;
    path_marker.color.a = 1.0;  // Fully opaque

    // Push the marker to the array
    marker_array.markers.push_back(path_marker);
  }
  // Publish the marker array
  path_marker_pub_->publish(marker_array);
}


void LqrNode::publishVelocity(double v, double w)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = v;
  msg.angular.z = w;
  RCLCPP_INFO(rclcpp::get_logger("LQR"), "Publishing control input: v=%f, w=%f",
              v, w);
  control_input_ = input(v, w);
  input_old = input(v, w);
  control_input_pub_->publish(msg);
}

void LqrNode::optimiseHeading(std::vector<State> &waypoints)
{
  for (size_t i = 0; i < waypoints.size() - 1; ++i)
  {
    double dx = waypoints[i + 1].x - waypoints[i].x;
    double dy = waypoints[i + 1].y - waypoints[i].y;
    waypoints[i].theta = std::atan2(dy, dx);
  }
  waypoints.back().theta = waypoints[waypoints.size() - 2].theta;
}

void LqrNode::controlLoopCallback()
{
  if (!odom_received_)
  {
    RCLCPP_INFO(rclcpp::get_logger("LQR"), "Waiting for odometry message...");
    return;
  }
  if (end_controller)
  {
    RCLCPP_INFO(rclcpp::get_logger("LQR"), "Goal reached!");
    control_loop_timer_->cancel();
    return;
  }

  State desired_state = waypoints_[current_waypoint];
  Eigen::Vector3d x_actual(actual_state_.x, actual_state_.y,
                           actual_state_.theta);
  Eigen::Vector3d x_desired(desired_state.x, desired_state.y,
                            desired_state.theta);
  state_error_ = x_actual - x_desired;

  if (current_waypoint == 2)
  {
    waypoints_[current_waypoint + 1] = State(-1, 3, M_PI);
  }

  RCLCPP_INFO(rclcpp::get_logger("LQR"), "Current Waypoint:=%zu ", current_waypoint); // Change %d to %zu
  RCLCPP_INFO(rclcpp::get_logger("LQR"), "Actual state: x=%f, y=%f, theta=%f", x_actual(0), x_actual(1), x_actual(2));
  RCLCPP_INFO(rclcpp::get_logger("LQR"), "Desired state: x=%f, y=%f, theta=%f", x_desired(0), x_desired(1), x_desired(2));
  RCLCPP_INFO(rclcpp::get_logger("LQR"), "State error: x=%f, y=%f, theta=%f", state_error_(0), state_error_(1), state_error_(2));
  RCLCPP_INFO(rclcpp::get_logger("LQR"), "Current goal: x=%f, y=%f, theta=%f",
              waypoints_[current_waypoint].x, waypoints_[current_waypoint].y, waypoints_[current_waypoint].theta);

  auto A = lqr_->getA(actual_state_.theta, control_input_.v, dt_);
  auto B = lqr_->getB(actual_state_.theta, dt_);
  lqr_->updateMatrices(A, B);
  lqr_->computeRiccati(B, A);

  auto u = lqr_->computeOptimalInput(state_error_);

  Eigen::EigenSolver<Eigen::MatrixXd> solver(B * lqr_->K_ + A);
  auto eigenValues = solver.eigenvalues().real();
  RCLCPP_INFO(rclcpp::get_logger("LQR"), "Eigenvalues: %f, %f, %f",
              eigenValues(0), eigenValues(1), eigenValues(2));

  publishVelocity(
      std::clamp(u(0), -max_linear_velocity, max_linear_velocity),
      std::clamp(u(1), -max_angular_velocity, max_angular_velocity));

  if (state_error_.norm() < tolerance)
  {
    RCLCPP_INFO(rclcpp::get_logger("LQR"), "Waypoint reached!");
    current_waypoint++;
    if (current_waypoint >= waypoints_.size())
    {
      end_controller = true;
      publishVelocity(0.0, 0.0);
    }
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto controller = std::make_shared<LqrNode>();
  rclcpp::spin(controller);
  rclcpp::shutdown();
  return 0;
}
