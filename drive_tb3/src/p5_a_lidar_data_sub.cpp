/**
 * @file p5_a_lidar_data_sub.cpp
 * @author Muhammad Luqman
 * @brief This ROS2 node receives LaserScan messages, calculates the distances to the closest obstacles
 *  in the front, right, and left directions, and publishes Twist messages to control robot motion.
 * @organization Robotisim
 */
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>

class lidar_subscriber : public rclcpp::Node
{
public:
    lidar_subscriber()
    : Node("lidar_subscriber")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/maze_solver/cmd_vel", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/maze_solver/scan", 10, std::bind(&lidar_subscriber::lidar_callback, this, std::placeholders::_1));
    }

private:
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr lidar_msg)
    {
        // Group obstacles
        float right_obstacle = *std::min_element(lidar_msg->ranges.begin() + 260, lidar_msg->ranges.begin() + 280);
        float front_obstacle = *std::min_element(lidar_msg->ranges.begin() + 340, lidar_msg->ranges.begin() + 360);
        float left_obstacle= *std::min_element(lidar_msg->ranges.begin() + 80, lidar_msg->ranges.begin() + 100);
        // Publish distances to the obstacles
        RCLCPP_INFO(this->get_logger(), "Front: %f, Right: %f, Left: %f,", front_obstacle, right_obstacle, left_obstacle);


    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<lidar_subscriber>());
    rclcpp::shutdown();
    return 0;
}
