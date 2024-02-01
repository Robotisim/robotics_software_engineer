/**
 * @file p3_c_linear_goal.cpp
 * @author Muhammad Luqman
 * @brief This ROS2 node calculates the error between the current and goal positions in the x-direction
 * and publishes Twist messages to control robot motion accordingly.
 * @organization Robotisim
 */


#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

class Linear_goal_node : public rclcpp::Node
{
public:
    Linear_goal_node()
    : Node("linear_goal_node")
    {
        this->declare_parameter<double>("goal_x", 1.0);
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&Linear_goal_node::odom_callback, this, std::placeholders::_1));
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr tb3_odom_msg)
    {
        auto message = geometry_msgs::msg::Twist();
        double goal_x = this->get_parameter("goal_x").as_double();
        double current_x = tb3_odom_msg->pose.pose.position.x;
        double Kp = 2.0;
        // Publish error and control robot motion
        double error = Kp*(goal_x - current_x) ;


        // if(error < 0.005){
        //     RCLCPP_INFO(this->get_logger(), "Goal Reached");

        //     message.linear.x =0.0;
        //     publisher_->publish(message);
        // }else{
        // RCLCPP_INFO(this->get_logger(), "Error : %f", error);
        //     message.linear.x =error;
        //     publisher_->publish(message);
        // }

        // Understanding Proportional Controller
        RCLCPP_INFO(this->get_logger(), "Error : %f", error);
        message.linear.x =error;
        publisher_->publish(message);

    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;

};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Linear_goal_node>());
    rclcpp::shutdown();
    return 0;
}
