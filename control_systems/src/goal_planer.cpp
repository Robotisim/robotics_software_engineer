// This code implements a ROS2 node for navigating towards a goal position.
// It calculates the distance and angle error to the goal and publishes velocity commands.
// Author: Robotisim

#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <rclcpp/rclcpp.hpp>

class GoalPlanner : public rclcpp::Node {
public:
  GoalPlanner() : Node("Goal_Planner") {
    this->declare_parameter<double>("set_point_x", 5.0);
    this->declare_parameter<double>("set_point_y", 3.0);
    this->declare_parameter<double>("kp_angle", 0.5);
    this->declare_parameter<double>("kp_distance", 0.5);
    _publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    _subscription = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10,
        std::bind(&GoalPlanner::odomCallback, this, std::placeholders::_1));
  }

private:
  double getYawFromQuaternion(const geometry_msgs::msg::Quaternion &quat) {
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr tb3OdomMsg) {
    auto cmdMsg = geometry_msgs::msg::Twist();
    double goalX = this->get_parameter("set_point_x").as_double();
    double goalY = this->get_parameter("set_point_y").as_double();
    double kpAngle = this->get_parameter("kp_angle").as_double();
    double kpDistance = this->get_parameter("kp_distance").as_double();

    double robotX = tb3OdomMsg->pose.pose.position.x;
    double robotY = tb3OdomMsg->pose.pose.position.y;
    double yaw = getYawFromQuaternion(tb3OdomMsg->pose.pose.orientation);

    double errorX = goalX - robotX;
    double errorY = goalY - robotY;

    double errorInDistance = sqrt(pow(errorX, 2) + pow(errorY, 2));
    double errorInAngle = atan2(errorY, errorX) - yaw;
    // Normalize angle to [-pi, pi]
    errorInAngle = atan2(sin(errorInAngle), cos(errorInAngle));

    RCLCPP_INFO(this->get_logger(), "E_D : %f E_A : %f", errorInDistance, errorInAngle);

    // Clamp the errors to avoid excessive values
    errorInDistance = std::max(0.0, std::min(errorInDistance, 1.0));
    errorInAngle = std::max(-1.0, std::min(errorInAngle, 1.0));

    if (errorInDistance > 0.1) {
      cmdMsg.linear.x = kpDistance * errorInDistance;
      cmdMsg.angular.z = kpAngle * errorInAngle;
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal Reached");
      cmdMsg.linear.x = 0.0;
      cmdMsg.angular.z = 0.0;
    }

    _publisher->publish(cmdMsg);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisher;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _subscription;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalPlanner>());
  rclcpp::shutdown();
  return 0;
}
