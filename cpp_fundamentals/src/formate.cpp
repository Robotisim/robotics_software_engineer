#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;
class PublisherNode : public rclcpp::Node {
public:
  PublisherNode() : Node("pub"), cnt(0) {
    pub = this->create_publisher<std_msgs::msg::String>("tpc", 10);
    tmr_ = this->create_wall_timer(500ms, std::bind(&Pub::cb, this));
  }

private:
  void cb() {
    auto msg = std_msgs::msg::String();
    msg.data = "Hello, world! " + std::to_string(cnt++);
    RCLCPP_INFO(this->get_logger(), "Pub: '%s'", msg.data.c_str());
    pub->publish(msg);
  }
  rclcpp::TimerBase::SharedPtr _tmr;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub;
  size_t _cnt;
};
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PublisherNode>());
  rclcpp::shutdown();
  return 0;
}
