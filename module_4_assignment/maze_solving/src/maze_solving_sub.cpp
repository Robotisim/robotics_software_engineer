#include <memory>
#include <algorithm> // This helps to use std::min_element
#include <cmath>
#include <iostream>

/*
Why include <memory>?
    Explanation: The <memory> header is included because it provides facilities for managing dynamic memory in C++. Specifically, it includes smart pointers like std::shared_ptr and std::make_shared which are used in your code.
    What happens if we don't include it?: If you don't include <memory>, the compiler will not recognize std::make_shared and std::shared_ptr, resulting in a compilation error.
*/

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

// Define the RobotState enum
enum class RobotState
{
    MOVING_STRAIGHT,
    TURNING_RIGHT,
    TURNING_LEFT,
    MOVING_BACK,
    OUT_OF_MAZE
};

class MazeSolving : public rclcpp::Node
{
public:
    MazeSolving() : Node("maze_solving"), state_(RobotState::MOVING_STRAIGHT)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&MazeSolving::lidarCallback, this, std::placeholders::_1));
    }

private:
    const float forward_segment_threshold = 1.5; // Configurable threshold

    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr lidar_msg)
    {
        analyzeObstacle(lidar_msg);
        determineState();
        publishVelocity();
    }

    void analyzeObstacle(const sensor_msgs::msg::LaserScan::SharedPtr lidar_msg)
    {
        front_segment = *std::min_element(lidar_msg->ranges.begin() + 340, lidar_msg->ranges.begin() + 360);
        forward_segment = *std::min_element(lidar_msg->ranges.begin() + 0, lidar_msg->ranges.begin() + 20);
        left_segment = *std::min_element(lidar_msg->ranges.begin() + 60, lidar_msg->ranges.begin() + 105);
        right_segment = *std::min_element(lidar_msg->ranges.begin() + 250, lidar_msg->ranges.begin() + 310);
        back_segment = *std::min_element(lidar_msg->ranges.begin() + 170, lidar_msg->ranges.begin() + 200);

        RCLCPP_INFO(this->get_logger(),
                    "Front Segment: '%f', Forward Segment: '%f', Left Segment: '%f', Right Segment: '%f', Back Segment: '%f'",
                    front_segment,
                    forward_segment,
                    left_segment,
                    right_segment,
                    back_segment);
        if (front_segment == std::numeric_limits<float>::infinity()&&
        right_segment == std::numeric_limits<float>::infinity()&&
        left_segment == std::numeric_limits<float>::infinity()){
            state_ = RobotState::OUT_OF_MAZE;
        }
    }

    void determineState()
    {
        switch (state_)
        {
        case RobotState::MOVING_STRAIGHT:
            if (front_segment <= forward_segment_threshold)
            {
                if ((right_segment > left_segment) || (std::isinf(right_segment) && std::isinf(left_segment)))
                {
                    state_ = RobotState::TURNING_RIGHT;
                }
                else if (right_segment < left_segment)
                {
                    state_ = RobotState::TURNING_LEFT;
                }
                else
                {
                    state_ = RobotState::MOVING_BACK;
                }
            }
            break;

        case RobotState::TURNING_LEFT:
            state_ = RobotState::MOVING_STRAIGHT;
            break;

        case RobotState::TURNING_RIGHT:
            state_ = RobotState::MOVING_STRAIGHT;
            break;

        case RobotState::MOVING_BACK:
            state_ = RobotState::MOVING_STRAIGHT;
            break;

        case RobotState::OUT_OF_MAZE:
            break;
        }
    }

    void publishVelocity()
    {
        geometry_msgs::msg::Twist command;

        switch (state_)
        {
        case RobotState::MOVING_STRAIGHT:
            command.linear.x = 0.3;
            command.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "Moving Straight");
            break;

        case RobotState::TURNING_LEFT:
            command.linear.x = 0.0;
            command.angular.z = 2.0;
            RCLCPP_INFO(this->get_logger(), "Turning Left");
            break;

        case RobotState::TURNING_RIGHT:
            command.linear.x = 0.0;
            command.angular.z = -2.0;
            RCLCPP_INFO(this->get_logger(), "Turning Right");
            break;

        case RobotState::MOVING_BACK:
            command.linear.x = -0.5;
            command.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "Moving Back");
            break;

        case RobotState::OUT_OF_MAZE:
            command.linear.x = 0.0;
            command.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "Out of Maze");
            break;
        }

        publisher_->publish(command);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    mutable RobotState state_; // Mutable to allow modification in const method

    // These variables should be class members so they can be used in different methods
    float front_segment;
    float forward_segment;
    float left_segment;
    float right_segment;
    float back_segment;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MazeSolving>());
    rclcpp::shutdown();
    return 0;
}
