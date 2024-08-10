#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <rclcpp/rclcpp.hpp>

class GoalPlaner : public rclcpp::Node {
public:
  GoalPlaner() : Node("Goal_Planer") {
    this->declare_parameter<double>("set_point_x", 5.0);
    this->declare_parameter<double>("set_point_y", 3.0);
    this->declare_parameter<double>("kp_angle", 0.5);
    this->declare_parameter<double>("kp_distance", 0.5);
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10,
        std::bind(&GoalPlaner::odom_callback, this, std::placeholders::_1));
  }

private:
  double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion &quat) {
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr tb3_odom_msg) {
    auto cmd_msg = geometry_msgs::msg::Twist();
    double goal_x = this->get_parameter("set_point_x").as_double();
    double goal_y = this->get_parameter("set_point_y").as_double();
    double Kp_angle = this->get_parameter("kp_angle").as_double();
    double Kp_distance = this->get_parameter("kp_distance").as_double();

    double robot_x = tb3_odom_msg->pose.pose.position.x;
    double robot_y = tb3_odom_msg->pose.pose.position.y;
    double yaw = get_yaw_from_quaternion(tb3_odom_msg->pose.pose.orientation);

    // RCLCPP_INFO(this->get_logger(), "X : %f Y : %f Ya : %f", robot_x,robot_y,yaw);

    float error_x= goal_x - robot_x;
    float error_y = goal_y - robot_y;

    double error_in_distance= sqrt(pow(error_x,2) + pow(error_y,2));
    double error_in_angle= atan2(error_y,error_x) - yaw;
    error_in_angle = atan2(sin(error_in_angle), cos(error_in_angle));



    RCLCPP_INFO(this->get_logger(), "E_D : %f E_A : %f", error_in_distance,error_in_angle );

    error_in_distance = std::max(0.0, std::min(error_in_distance, 1.0));
    error_in_angle    = std::max(-1.0, std::min(error_in_angle, 1.0));

    if(error_in_distance > 0.1){
        cmd_msg.linear.x =Kp_distance*error_in_distance;
        cmd_msg.angular.z=Kp_angle*error_in_angle;
    }else{
        RCLCPP_INFO(this->get_logger(), "Goal Reached");
        cmd_msg.linear.x =0.0;
        cmd_msg.angular.z=0.0;

    }



    publisher_->publish(cmd_msg);



  }





  double error = 0;
  const double THRESHOLD = 0.01;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalPlaner>());
  rclcpp::shutdown();
  return 0;
}