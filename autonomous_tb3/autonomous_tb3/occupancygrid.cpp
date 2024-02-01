#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <vector>
#include <memory>

using namespace std::chrono_literals;

class MapPublisher : public rclcpp::Node {
public:
  MapPublisher() : Node("map_publisher") {
    publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&MapPublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    auto map_msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();

    // Set the frame ID and timestamp
    map_msg->header.frame_id = "map";
    map_msg->header.stamp = this->get_clock()->now();

    // Set the map metadata
    map_msg->info.resolution = 1.0; // 1 meter per grid cell
    map_msg->info.width = 61;
    map_msg->info.height = 61;
    map_msg->info.origin.position.x = 0.0;
    map_msg->info.origin.position.y = 0.0;
    map_msg->info.origin.position.z = 0.0;
    map_msg->info.origin.orientation.w = 1.0;

    // Initialize the map data
    std::vector<int8_t> data(map_msg->info.width * map_msg->info.height, 0);

    // Add obstacles to the map
    add_obstacles(data);

    // Copy the data into the message
    map_msg->data = data;

    // Publish the map
    publisher_->publish(std::move(map_msg));
    RCLCPP_INFO(this->get_logger(), "Published map");
  }

  void add_obstacles(std::vector<int8_t> &data) {
    int size = 61; // Size of the map
    // Add the outer walls
    for (int i = 0; i < size; i++) {
      data[i] = 100; // Bottom
      data[i * size] = 100; // Left
      data[(size - 1) * size + i] = 100; // Top
      data[i * size + (size - 1)] = 100; // Right
    }

    // Add internal walls
    for (int i = 0; i <= 40; i++) {
      data[15 + i * size] = 100; // Vertical wall from the bottom
    }

    for (int i = 20; i <= 60; i++) {
      data[45 + i * size] = 100; // Vertical wall from the top
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapPublisher>());
  rclcpp::shutdown();
  return 0;
}
