#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::Int16>(
      "int_pub", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::Int16 & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg.data);
    }
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}