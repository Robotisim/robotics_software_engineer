#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
using namespace std::chrono_literals;

class OccupancyGridPub : public rclcpp::Node
{
public:
  OccupancyGridPub()
  : Node("Occupancy_Grid_Pub")
  {
    publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&OccupancyGridPub::OG_callback, this));
  }

private:
  void OG_callback()
  {
    auto occupancy_grid_message = nav_msgs::msg::OccupancyGrid();
    occupancy_grid_message.header = std_msgs::msg::Header();
    occupancy_grid_message.header.stamp = this->now();
    occupancy_grid_message.header.frame_id = "map_frame";

    occupancy_grid_message.info.resolution=1.0;
    occupancy_grid_message.info.width=3;
    occupancy_grid_message.info.height=3;

    std::vector<int8_t> data = {100, 0, -1, 0, 100, 0, 0, 0, -1};
    occupancy_grid_message.data = data;




    publisher_->publish(occupancy_grid_message);

  }

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancyGridPub>());
  rclcpp::shutdown();
  return 0;
}
