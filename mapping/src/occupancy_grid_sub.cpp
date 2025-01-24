#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

using std::placeholders::_1;

class MapSubscriber : public rclcpp::Node
{
public:
  MapSubscriber()
  : Node("map_subscriber")
  {
    // MAIN ISSUES SOLVE WITH SPECIFIC -> QoS profile to match map_server's settings
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default)).reliable().transient_local();

    subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", qos, std::bind(&MapSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) const
  {
   
    RCLCPP_INFO(this->get_logger(), "Map received: width=%u, height=%u", msg->info.width, msg->info.height);

	// Checking occupancy data

    int8_t max_value = -1;
    int zero_count = 0;
    int non_zero_count = 0;

    for (size_t i = 0; i < msg->data.size(); ++i) {
      int8_t value = msg->data[i];
      
      if (value == 0) {
        zero_count++;
      } else if (value > 0) {
        non_zero_count++;
        if (value > max_value) {
          max_value = value;
        }
      }
    }

    RCLCPP_INFO(this->get_logger(), "Max value in map: %d", max_value);
    RCLCPP_INFO(this->get_logger(), "Zero values: %d, Non-zero values: %d", zero_count, non_zero_count);
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapSubscriber>());
  rclcpp::shutdown();
  return 0;
}
