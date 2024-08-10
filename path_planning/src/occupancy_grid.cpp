#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"


using namespace std::chrono_literals;
class OccupancyGrid_Publisher : public rclcpp::Node
{
public:
  OccupancyGrid_Publisher()
  : Node("occupancy_grid_publisher")
  {
    RCLCPP_INFO(this->get_logger(), "Publishing Occupancy Grid");

    og_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("occupancy_grid", 10);
    og_timer =
      this->create_wall_timer(500ms, std::bind(&OccupancyGrid_Publisher::og_callback, this));
  }

private:
  void og_callback()
  {

    auto occupancy_grid_msg = nav_msgs::msg::OccupancyGrid();
    int grid_width = 10, grid_height = 10;
    int area = grid_height * grid_width;
    std::vector<signed char> og_array(area, 0);   // Initialize all cells as free

    auto add_line = [&](int row) {
        for (int x = 2; x < grid_width - 2; ++x) {   // Start from 2 and end 2 cells before the edge
          og_array[row * grid_width + x] = 100;     // Set cells to occupied
        }
      };

    // Adding lines on the 2nd, 6th, and 9th rows
    // if (grid_height > 1) add_line(1);
    if (grid_height > 5) {add_line(5);}
    // if (grid_height > 8) add_line(8);

    // og_array[99]=100; // finding starting point
    occupancy_grid_msg.header.stamp = rclcpp::Clock().now();
    occupancy_grid_msg.header.frame_id = "map_frame";

    occupancy_grid_msg.info.resolution = 1;

    occupancy_grid_msg.info.width = grid_width;
    occupancy_grid_msg.info.height = grid_height;

    occupancy_grid_msg.info.origin.position.x = -5.0;
    occupancy_grid_msg.info.origin.position.y = -5.0;
    occupancy_grid_msg.info.origin.position.z = 0.0;
    occupancy_grid_msg.info.origin.orientation.x = 0.0;
    occupancy_grid_msg.info.origin.orientation.y = 0.0;
    occupancy_grid_msg.info.origin.orientation.z = 0.0;
    occupancy_grid_msg.info.origin.orientation.w = 1.0;
    occupancy_grid_msg.data = og_array;

    og_pub->publish(occupancy_grid_msg);
  }


  rclcpp::TimerBase::SharedPtr og_timer;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr og_pub;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancyGrid_Publisher>());
  rclcpp::shutdown();
  return 0;
}