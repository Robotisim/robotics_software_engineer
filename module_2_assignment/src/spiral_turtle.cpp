#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
      : Node("minimal_publisher"), count_(0)
  {
    /* we will pass the parameter first*/
    this->declare_parameter<std::string>("cmd_vel_topic", "/turtle1/cmd_vel");
    std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = geometry_msgs::msg::Twist();
    static double a = 0.5; // Growth factor of the spiral

    // Adjust the initial theta value to control starting point
    static double theta = 0.1; // Adjust this value for spiral starting point

    int16_t new_radius = std::exp(a * theta);
    message.linear.x = new_radius * std::cos(theta);
    message.linear.y = new_radius * std::sin(theta);
    message.angular.z = 6; // we can change the angular speed as we desire.

    // Increment theta after calculations for proper spiral generation
    theta += 0.1;

    RCLCPP_INFO(this->get_logger(), "Drive the turtle.");
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
