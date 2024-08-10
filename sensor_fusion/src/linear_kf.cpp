/*
 * This code demonstrates a basic implementation of a Kalman Filter for sensor fusion in a ROS 2 environment.
 * Specific to sensor fusion:
 * - State Vector: Contains the position (x, y) of the robot.
 * - Measurement Vector: Obtained from the odometry sensor.
 * - Prediction Step: Uses control inputs (velocity commands) to predict the next state.
 * - Update Step: Adjusts the predicted state based on the sensor measurements.
 * - Process Noise: Represents the uncertainty in the model's prediction.
 * - Measurement Noise: Represents the uncertainty in the sensor measurements.
 */

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <Eigen/Dense>
#include <chrono>
#include <functional>
#include <random>


using namespace std::chrono_literals;

double generateGaussianNoise(double mean, double stddev) {
    static std::random_device rd;
    static std::mt19937 generator(rd());
    std::normal_distribution<double> distribution(mean, stddev);
    return distribution(generator);
}


class SimpleKalmanFilter2D
{
public:
  SimpleKalmanFilter2D(double process_noise, double measurement_noise, double estimation_error, double initial_x, double initial_y)
  {
    Q = process_noise;
    R = measurement_noise;
    P.setIdentity();
    P *= estimation_error;
    X << initial_x, initial_y;
    F.setIdentity();
    H.setIdentity();
  }

  void predict(double control_vx, double control_vy, double dt)
  {
    double noisy_vx=control_vx + generateGaussianNoise(0,Q);
    double noisy_vy=control_vy + generateGaussianNoise(0,Q);
    // State prediction
    X(0) = X(0) + noisy_vx * dt;
    X(1) = X(1) + noisy_vy * dt;

    // Covariance prediction
    P = F * P * F.transpose() + Q * Eigen::Matrix2d::Identity();
  }

  void update(double measured_x, double measured_y)
  {
    Eigen::Vector2d Z;
    double noisy_measured_x = measured_x + generateGaussianNoise(0,R);
    double noisy_measured_y = measured_y + generateGaussianNoise(0,R);
    Z << noisy_measured_x, noisy_measured_y;

    Eigen::Vector2d Y = Z - H * X; // Measurement residual
    Eigen::Matrix2d S = H * P * H.transpose() + R * Eigen::Matrix2d::Identity(); // Residual covariance
    Eigen::Matrix2d K = P * H.transpose() * S.inverse(); // Kalman gain

    // State update
    X = X + K * Y;
    P = (Eigen::Matrix2d::Identity() - K * H) * P; // Covariance update
  }

  double getStateX() { return X(0); }
  double getStateY() { return X(1); }

private:
  Eigen::Matrix2d P; // Estimation error covariance
  Eigen::Matrix2d F; // State transition matrix
  Eigen::Matrix2d H; // Measurement matrix
  double Q; // Process noise covariance
  double R; // Measurement noise covariance
  Eigen::Vector2d X; // State
};

class Tb3OdomNode : public rclcpp::Node
{
public:
  Tb3OdomNode() : Node("tb3_odom_node"), kf(0.1, 0.1, 1.0, 0.0, 0.0), control_vx(0.0), control_vy(0.0)
  {
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&Tb3OdomNode::odom_callback, this, std::placeholders::_1));

    cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&Tb3OdomNode::cmd_vel_callback, this, std::placeholders::_1));

    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

    timer_ = this->create_wall_timer(100ms, std::bind(&Tb3OdomNode::timer_callback, this));
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    double measured_x = msg->pose.pose.position.x;
    double measured_y = msg->pose.pose.position.y;

    // Update the Kalman filter with the measurement
    kf.update(measured_x, measured_y);
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    control_vx = msg->linear.x;
    control_vy = msg->linear.y;
  }

  void timer_callback()
  {
    double dt = 0.1; // Time step, in seconds
    kf.predict(control_vx, control_vy, dt);

    // Get the filtered state
    double filtered_x = kf.getStateX();
    double filtered_y = kf.getStateY();

    // Publish the filtered position as a marker
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "odom";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "tb3_odom";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = filtered_x;
    marker.pose.position.y = filtered_y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    marker_publisher_->publish(marker);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  SimpleKalmanFilter2D kf;
  double control_vx;
  double control_vy;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Tb3OdomNode>());
  rclcpp::shutdown();
  return 0;
}
