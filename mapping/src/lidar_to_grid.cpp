#include <chrono>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/header.hpp>
using namespace std::chrono_literals;

class LidarSub : public rclcpp::Node {
public:
	LidarSub() : Node("Lidar_Sub") {
		subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
		    "/scan", 10, std::bind(&LidarSub::Lidar_callback, this, std::placeholders::_1));
		publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
	}

private:

	void Lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
		RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->ranges[0]);

		// Create a new occupancy grid message
		auto map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
		map->header.frame_id = "map";
		map->header.stamp = this->now();
		msg->header = std_msgs::msg::Header();
		map->info.resolution = 1;
		map->info.width = 20;
		map->info.height = 20;
		map->info.origin.position.x = -10.0;
		map->info.origin.position.y = -10.0;
		map->info.origin.position.z = 0.0;
		map->data.resize(map->info.width * map->info.height);


        // Fill the map with the laser scan data
		for (int i = 0; i < msg->ranges.size(); i++) {
			double angle = msg->angle_min + i * msg->angle_increment;
			if (msg->ranges[i] < msg->range_min || msg->ranges[i] > msg->range_max) {
				continue; // Skip invalid range
			}
			double x = msg->ranges[i] * cos(angle);
			double y = msg->ranges[i] * sin(angle);
			int cell_x = (x - map->info.origin.position.x) / map->info.resolution;
			int cell_y = (y - map->info.origin.position.y) / map->info.resolution;

			RCLCPP_INFO(this->get_logger(), "x: '%d', y: '%d'", cell_x, cell_y);
			if (cell_x >= 0 && cell_x < map->info.width && cell_y >= 0 && cell_y < map->info.height) {
				map->data[cell_y * map->info.width + cell_x] = 100;
			}
		}

		// Publish the map
		publisher_->publish(*map);
	}

	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
	rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)

{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<LidarSub>());
	rclcpp::shutdown();
	return 0;
}