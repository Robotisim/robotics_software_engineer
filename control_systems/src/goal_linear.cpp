#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

class GoalLinear : public rclcpp::Node
{
public:
  GoalLinear()
  : Node("Goal_Linear")
  {
    this->declare_parameter<double>("set_point", 0.0);
    this->declare_parameter<double>("kp", 0.5);
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&GoalLinear::odom_callback, this, std::placeholders::_1));
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr tb3_odom_msg)
  {
    auto message = geometry_msgs::msg::Twist();
    double goal_x = this->get_parameter("set_point").as_double();
    double Kp = this->get_parameter("kp").as_double();
    double current_x = tb3_odom_msg->pose.pose.position.x;

    error = Kp * (goal_x - current_x);

    if(std::fabs(error) < THRESHOLD){
        message.linear.x= 0.0;
        RCLCPP_INFO(this->get_logger(), "Reached Goal");
    }else {
        message.linear.x=error;
        RCLCPP_INFO(this->get_logger(), "Error = %f", error);
    }
    publisher_->publish(message);



  }
  double error=0;
  const double THRESHOLD= 0.01;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalLinear>());
  rclcpp::shutdown();
  return 0;
}