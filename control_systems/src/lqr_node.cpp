#include <angles/angles.h>
#include <cmath>
#include <lqr_node.hpp>
#include <rclcpp/logging.hpp>
#include <tuple>
#include <vector>

input input_old = input(0, 0);

LqrNode::LqrNode()
    : Node("LqrNode"), dt_(0.03), tolerance(0.8), end_controller(false),
      max_linear_velocity(0.8), max_angular_velocity(M_PI / 2),
      current_waypoint(0), odom_received_(false) {

  robot_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10,
      std::bind(&LqrNode::robotPoseCallback, this, std::placeholders::_1));
  control_input_pub_ =this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  control_loop_timer_ = this->create_wall_timer(std::chrono::milliseconds(30),
                        std::bind(&LqrNode::controlLoopCallback, this));

  Q_ << 0.8, 0, 0, 0, 0.8, 0, 0, 0, 0.8;
  R_ << 0.8, 0, 0, 0.8;
  lqr_ = std::make_unique<LQR>(Q_, R_, 100);

  waypoints_ = {State(1, 1, M_PI / 4), State(2, 2, M_PI / 2),
                State(3, 3, M_PI),     State(4, 4, 3 * M_PI / 2),
                // State(-1, 4, M_PI),    State(-2, 3, -M_PI / 2),
                // State(-3, 2, M_PI),    State(-3, 1, M_PI / 2),
                State(0, 0, 0)};
  actual_state_ = State(0, 0, 0);

  optimiseHeading(waypoints_);

}

void LqrNode::robotPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {

 tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  actual_state_ =
      State(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
  odom_received_ = true;

}

void LqrNode::publishVelocity(double v, double w) {

  geometry_msgs::msg::Twist msg;
  msg.linear.x = v;
  msg.angular.z = w;
  RCLCPP_INFO(rclcpp::get_logger("LQR"), "Publishing control input: v=%f, w=%f",
              v, w);
  control_input_ = input(v, w);
  input_old = input(v, w);
  control_input_pub_->publish(msg);

}

void LqrNode::optimiseHeading(std::vector<State> &waypoints) {

  for (size_t i = 0; i < waypoints.size() - 1; ++i) {
    double dx = waypoints[i + 1].x - waypoints[i].x;
    double dy = waypoints[i + 1].y - waypoints[i].y;
    waypoints[i].theta = std::atan2(dy, dx);
  }
  waypoints.back().theta = waypoints[waypoints.size() - 2].theta;

}

void LqrNode::controlLoopCallback() {

 if (!odom_received_) {
    RCLCPP_INFO(rclcpp::get_logger("LQR"), "Waiting for odometry message...");
    return;
  }
  if (end_controller) {
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

  if (current_waypoint == 2) {
    waypoints_[current_waypoint + 1] = State(-1, 3, M_PI);
  }

  RCLCPP_INFO(rclcpp::get_logger("LQR"), "Current Waypoint:=%d ",current_waypoint);
  RCLCPP_INFO(rclcpp::get_logger("LQR"), "Actual state: x=%f, y=%f, theta=%f",x_actual(0), x_actual(1), x_actual(2));
  RCLCPP_INFO(rclcpp::get_logger("LQR"), "Desired state: x=%f, y=%f, theta=%f",x_desired(0), x_desired(1), x_desired(2));
  RCLCPP_INFO(rclcpp::get_logger("LQR"), "State error: x=%f, y=%f, theta=%f",state_error_(0), state_error_(1), state_error_(2));
  RCLCPP_INFO(rclcpp::get_logger("LQR"), "Current goal: x=%f, y=%f, theta=%f",
                waypoints_[current_waypoint].x, waypoints_[current_waypoint].y,waypoints_[current_waypoint].theta);

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

  if (state_error_.norm() < tolerance) {
    RCLCPP_INFO(rclcpp::get_logger("LQR"), "Waypoint reached!");
    current_waypoint++;
    if (current_waypoint >= waypoints_.size()) {
      end_controller = true;
      publishVelocity(0.0, 0.0);
    }
  }

}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto controller = std::make_shared<LqrNode>();
  rclcpp::spin(controller);
  rclcpp::shutdown();
  return 0;
}
