/**
 * @file p5_b_maze_solving.cpp
 * @author Muhammad Luqman
 * @brief This ROS2 node receives LaserScan messages, determines the robot's state based on the
 *  distances to the closest obstacles, and publishes Twist messages to control robot motion accordingly.
 * @organization Robotisim
 */

#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>


enum class RobotState{
    MOVING_STRAIGHT,
    TURNING_LEFT,
    TURNING_RIGHT,
    OUT_OF_MAZE
};



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
        // Determine robot's state and control robot motion accordingly
        float right_obstacle = *std::min_element(lidar_msg->ranges.begin() + 260, lidar_msg->ranges.begin() + 280);
        float front_obstacle = *std::min_element(lidar_msg->ranges.begin() + 340, lidar_msg->ranges.begin() + 360);
        float left_obstacle= *std::min_element(lidar_msg->ranges.begin() + 80, lidar_msg->ranges.begin() + 100);
        RCLCPP_INFO(this->get_logger(), "Front: %f, Right: %f, Left: %f,", front_obstacle, right_obstacle, left_obstacle);

        if(front_obstacle == std::numeric_limits<float>::infinity() && right_obstacle == std::numeric_limits<float>::infinity() &&
        left_obstacle == std::numeric_limits<float>::infinity() ){
            state_ = RobotState::OUT_OF_MAZE;
        }

        switch (state_)
        {
        case RobotState::MOVING_STRAIGHT:
            if(front_obstacle < front_threshold){
                // take a turn
                if(left_obstacle < right_obstacle){
                    state_=RobotState::TURNING_RIGHT;
                }else{
                    state_=RobotState::TURNING_LEFT;
                }
            }
            break;
        case RobotState::TURNING_LEFT:
            if(front_obstacle > front_threshold){
                state_ = RobotState::MOVING_STRAIGHT;
            }
            break;
        case RobotState::TURNING_RIGHT:
            if(front_obstacle > front_threshold){
                state_ = RobotState::MOVING_STRAIGHT;
            }
            break;

        }

        switch (state_)
        {
            case RobotState::MOVING_STRAIGHT:
                command.linear.x = linear_vel;
                command.angular.z = 0.0;
                RCLCPP_INFO(this->get_logger(), "Moving straight");
                break;

            case RobotState::TURNING_LEFT:
                command.linear.x = 0.0;
                command.angular.z = angular_vel;
                RCLCPP_INFO(this->get_logger(), "Turning left");
                break;

            case RobotState::TURNING_RIGHT:
                command.linear.x = 0.0;
                command.angular.z = -angular_vel;
                RCLCPP_INFO(this->get_logger(), "Turning right");
                break;

            case RobotState::OUT_OF_MAZE:
                command.linear.x = 0.0;
                command.angular.z = 0.0;
                RCLCPP_INFO(this->get_logger(), "Out of maze, stopping");
                break;
        }

        publisher_->publish(command);

    }
    double front_threshold = 1.5;
    double angular_vel = 0.8;
    double linear_vel = 0.6;
    geometry_msgs::msg::Twist command;
    RobotState state_;
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
