#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

using std::placeholders::_1;

class MapSubscriber : public rclcpp::Node
{
public:
  MapSubscriber()
  : Node("map_subscriber")
  {
    // QoS profile to match map_server's settings
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default)).reliable().transient_local();

    subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", qos, std::bind(&MapSubscriber::map_callback, this, _1));
    
    goal_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, std::bind(&MapSubscriber::goal_callback, this, _1));

    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);

    map_resolution_ = 0.05;  // Example map resolution
    map_origin_x_ = -6.86;    // Example map origin x
    map_origin_y_ = -5.41;    // Example map origin y
  }

private:
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) const
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

  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Goal received Rviz Cord: x=%.2f, y=%.2f", msg->pose.position.x, msg->pose.position.y);

    // Convert goal coordinates to map indices
    int map_index_x = static_cast<int>((msg->pose.position.x - map_origin_x_) / map_resolution_);
    int map_index_y = static_cast<int>((msg->pose.position.y - map_origin_y_) / map_resolution_);

    RCLCPP_INFO(this->get_logger(), "Goal indices on Map Cord: x=%d, y=%d", map_index_x, map_index_y);

    // Create and publish a marker at the goal position
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "goal_marker";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = msg->pose.position.x;
    marker.pose.position.y = msg->pose.position.y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;  // Ball radius
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    marker_publisher_->publish(marker);
    RCLCPP_INFO(this->get_logger(), "Published marker at goal position.");
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;

  double map_resolution_;
  double map_origin_x_;
  double map_origin_y_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapSubscriber>());
  rclcpp::shutdown();
  return 0;
}
