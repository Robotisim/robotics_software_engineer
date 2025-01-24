#include <chrono>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;

class LidarSub : public rclcpp::Node {
public:
	LidarSub() : Node("Lidar_Sub") {
		subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
		    "/scan", 10, std::bind(&LidarSub::Lidar_callback, this, std::placeholders::_1));
		publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
		// timer_ = this->create_wall_timer(500ms, std::bind(&LidarSub::OG_callback, this));

		marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker_array", 10);

		odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
		    "/odom", 10, std::bind(&LidarSub::odom_callback, this, std::placeholders::_1));
	}

private:
	void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
		RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->pose.pose.position.x);
		current_x = msg->pose.pose.position.x;
		current_y = msg->pose.pose.position.y;
		tf2::Quaternion q(msg->pose.pose.orientation.x,
		                  msg->pose.pose.orientation.y,
		                  msg->pose.pose.orientation.z,
		                  msg->pose.pose.orientation.w);
		tf2::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		normalise_angle(yaw);
		heading = yaw;

		auto marker = std::make_shared<visualization_msgs::msg::Marker>();
		marker->header.frame_id = "map";
		marker->header.stamp = this->now();
		marker->ns = "bot";
		marker->id = 0;
		marker->type = visualization_msgs::msg::Marker::ARROW;
		marker->action = visualization_msgs::msg::Marker::ADD;
		marker->pose.position.x = current_x;
		marker->pose.position.y = current_y;
		marker->pose.position.z = 0.0; // Replace with bot's z position
		marker->pose.orientation.x = q.x();
		marker->pose.orientation.y = q.y();
		marker->pose.orientation.z = q.z();
		marker->pose.orientation.w = q.w();
		marker->scale.x = 1.0; // Length of the arrow
		marker->scale.y = 0.1; // Width of the arrow
		marker->scale.z = 0.0; // Height of the arrow (0 for 2D)
		marker->color.a = 1.0; // Don't forget to set alpha!
		marker->color.r = 0.0;
		marker->color.g = 1.0;
		marker->color.b = 0.0;

		marker_publisher_->publish(*marker);
	}

	void normalise_angle(double& yaw) {
		if (previous_yaw - yaw > M_PI) {
			full_rotations++;
		} else if (yaw - previous_yaw > M_PI) {
			full_rotations--;
		}
		previous_yaw = yaw;
		yaw += full_rotations * 2.0 * M_PI; // Add 2π for each full rotation
	}

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
			double x = msg->ranges[i] * cos(angle + heading) + current_x;
			double y = msg->ranges[i] * sin(angle + heading) + current_y;
			int cell_x = (x - map->info.origin.position.x) / map->info.resolution;
			int cell_y = (y - map->info.origin.position.y) / map->info.resolution;

			// TODO Also incorporate the robot's position in the map

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
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
	double current_x;
	double current_y;
	double heading;
	double previous_yaw = 0.0;
	int full_rotations = 0;
};

int main(int argc, char** argv)

{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<LidarSub>());
	rclcpp::shutdown();
	return 0;
}