/**
 * @file p3_d_xy_goal.cpp
 * @author Muhammad Luqman
 * @brief This ROS2 node calculates the error between the current and goal positions in the x and y directions,
 * and the error in the yaw angle, and publishes Twist messages to control robot motion accordingly.
 * @organization Robotisim
 */
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <cmath>

class Linear_goal_node : public rclcpp::Node
{
public:
    Linear_goal_node()
    : Node("linear_goal_node")
    {
        this->declare_parameter<double>("goal_x", 2.0);
        this->declare_parameter<double>("goal_y", 3.0);
        this->declare_parameter<double>("kp", 5.0);


        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&Linear_goal_node::odom_callback, this, std::placeholders::_1));
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr tb3_odom_msg)
    {
        // Defineing all variables
        auto cmd_msg = geometry_msgs::msg::Twist();
        goal_x = this->get_parameter("goal_x").as_double(); // Set your goal x
        goal_y = this->get_parameter("goal_y").as_double();  // Set your goal y
        Kp_     = this->get_parameter("kp").as_double();      // Set the proportional gain for control
        // Get Robot Pose
        float robot_pos_x = tb3_odom_msg->pose.pose.position.x;
        float robot_pos_y = tb3_odom_msg->pose.pose.position.y;

        // Convert Quaternions to Euler
        double roll,pitch,yaw;
        tf2::Quaternion q(
            tb3_odom_msg->pose.pose.orientation.x,
            tb3_odom_msg->pose.pose.orientation.y,
            tb3_odom_msg->pose.pose.orientation.z,
            tb3_odom_msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll,pitch,yaw);

        // Calcualte Errors
        float error_x = goal_x - robot_pos_x;
        float error_y = goal_y - robot_pos_y;

        // Equations for Go TO Goal Behaviour
        float distance_to_goal = sqrt( pow(error_x,2) + pow(error_y,2));
        float angle_to_goal = atan2(error_y,error_x);
        float error_in_angle = angle_to_goal - yaw ;


        // Logging
        RCLCPP_INFO(this->get_logger(), "DG : %f AG : %f EA : %f", distance_to_goal,angle_to_goal , error_in_angle);

        // Robot Motion Commands

        if(abs(error_in_angle)>0.1){ // Not facing the goal
            cmd_msg.linear.x =0.0;
            cmd_msg.angular.z =Kp_*(error_in_angle);
            RCLCPP_INFO(this->get_logger(), "Case # 1 Rotating towards the Goal");

        }else{ // If we are looking at the goal

            if(distance_to_goal > 0.1){ //if not reached the goal
                cmd_msg.linear.x =Kp_*(distance_to_goal);
                cmd_msg.angular.z =0.0;
                RCLCPP_INFO(this->get_logger(), "Case # 2 Moving Towards the Goal");

            }else{ // we have reached the goal
                cmd_msg.linear.x =0.0;
                cmd_msg.angular.z =0.0;
                RCLCPP_INFO(this->get_logger(), "Case # 3 Reached the Goal");

            }

        }


        publisher_->publish(cmd_msg);

    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    float goal_x;
    float goal_y;
    float Kp_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Linear_goal_node>());
    rclcpp::shutdown();
    return 0;
}
