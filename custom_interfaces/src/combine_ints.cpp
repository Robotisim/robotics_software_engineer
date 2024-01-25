#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/two_ints.hpp"
using namespace std::chrono_literals;

class SimplePublisher : public rclcpp::Node
{
public:
  SimplePublisher()
  : Node("simple_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<custom_interfaces::msg::TwoInts>("two_ints_topic", 10);
    timer_ = this->create_wall_timer(1s, std::bind(&SimplePublisher::timerCallback, this));
  }

private:
  void timerCallback()
  {
    auto message = custom_interfaces::msg::TwoInts();
    message.first_int = count_;
    message.second_int = -count_;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%d', '%d'", message.first_int, message.second_int);
    publisher_->publish(message);
    count_++;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<custom_interfaces::msg::TwoInts>::SharedPtr publisher_;
  int count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimplePublisher>());
  rclcpp::shutdown();
  return 0;
}
