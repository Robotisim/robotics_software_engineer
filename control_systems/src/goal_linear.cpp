// This code implements a ROS2 node for reaching a linear goal position.
// It subscribes to odometry messages and publishes velocity commands.
// Author: Robotisim

#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

class GoalLinear : public rclcpp::Node {
public:
  GoalLinear() : Node("Goal_Linear") {
    this->declare_parameter<double>("set_point", 0.0);
    this->declare_parameter<double>("kp", 0.5);
    _publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    _subscription = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&GoalLinear::odomCallback, this, std::placeholders::_1));
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr tb3OdomMsg) {
    auto message = geometry_msgs::msg::Twist();
    double goalX = this->get_parameter("set_point").as_double();
    double kp = this->get_parameter("kp").as_double();
    double currentX = tb3OdomMsg->pose.pose.position.x;

    _error = kp * (goalX - currentX);

    if (std::fabs(_error) < _THRESHOLD) {
      message.linear.x = 0.0;
      RCLCPP_INFO(this->get_logger(), "Reached Goal");
    } else {
      message.linear.x = _error;
      RCLCPP_INFO(this->get_logger(), "Error = %f", _error);
    }
    _publisher->publish(message);
  }
  double _error = 0;
  const double _THRESHOLD = 0.01;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisher;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _subscription;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalLinear>());
  rclcpp::shutdown();
  return 0;
}
