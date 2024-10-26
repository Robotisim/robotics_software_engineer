#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class MotionController : public rclcpp::Node
{
public:
  MotionController()
      : Node("motion_controller"), count_(0)
  {
    // Declare parameter for motion type with default value
    this->declare_parameter<std::string>("motion_type", "circle");
    motion_type_ = this->get_parameter("motion_type").as_string();

    // Declare common parameter for both motions
    this->declare_parameter<double>("angular_z", 6.0);
    angular_z_ = this->get_parameter("angular_z").as_double();

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MotionController::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = geometry_msgs::msg::Twist();

    // Check for valid motion type
    if (motion_type_ == "circle")
    {
      // Set values for circular motion
      this->declare_parameter<double>("radius", 1.0);
      double radius = this->get_parameter("radius").as_double();
      message.linear.x = radius;
    }
    else if (motion_type_ == "spiral")
    {
      // Set values for spiral motion
      this->declare_parameter<double>("a", 0.5);     // Growth factor
      this->declare_parameter<double>("theta", 0.1); // Starting theta
      double a = this->get_parameter("a").as_double();
      double theta = this->get_parameter("theta").as_double();
      int16_t new_radius = std::exp(a * theta);
      message.linear.x = new_radius * std::cos(theta);
      message.linear.y = new_radius * std::sin(theta);
      theta += 0.1; // Adjust increment for desired spiral path
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid motion type. Choose either 'circular' or 'spiral'. Thank you!");
      return;
    }

    message.angular.z = angular_z_;

    RCLCPP_INFO(this->get_logger(), "Driving turtle in %s motion.", motion_type_.c_str());
    publisher_->publish(message);
  }

  std::string motion_type_;
  double angular_z_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionController>());
  rclcpp::shutdown();
  return 0;
}
