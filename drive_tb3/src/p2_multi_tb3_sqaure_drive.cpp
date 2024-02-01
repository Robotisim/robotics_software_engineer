/**
 * Author: Muhammad Luqman
 * Organization: Robotisim
 *
 * This ROS2 node, "turtlesim_driving", makes a Turtlebot3 robot move in a square pattern.
 *
 * The node publishes geometry_msgs/Twist messages to a specified command velocity topic
 * to control the robot's motion. The robot moves straight for a certain duration, then turns
 * for a certain duration, and repeats these steps to complete the square pattern.
 */
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class drive_turtlesim : public rclcpp::Node
{
  public:
    drive_turtlesim()
    : Node("turtlesim_driving"), count_(0)
    {
      this->declare_parameter<std::string>("cmd_vel_topic","/turtle1/cmd_vel");
      std::string cmd_vel_topic_var = this->get_parameter("cmd_vel_topic").as_string();


      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_var, 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&drive_turtlesim::timer_callback, this));
    }

  private:
    double linear_velocity = 0.5;
    double angular_velocity = 1.2;
    double side_length = 2.5;

    void timer_callback()
    {

      // Calculate travel time and turn time
      double travel_time = side_length / linear_velocity;
      double turn_time = 2 + (M_PI / (2.0 * angular_velocity) );

      // Create the Twist message
      auto message = geometry_msgs::msg::Twist();

      // Move robot straight
      message.linear.x = linear_velocity;
      message.angular.z = 0.0;
      publisher_->publish(message);
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(travel_time)));

      // Stop robot and make a turn
      message.angular.z = angular_velocity;
      message.linear.x = angular_velocity;
      publisher_->publish(message);
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(turn_time)));

    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drive_turtlesim>());
  rclcpp::shutdown();
  return 0;
}