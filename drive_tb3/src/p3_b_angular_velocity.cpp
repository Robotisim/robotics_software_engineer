/**
 * @file p3_b_angular_velocity.cpp
 * @author Muhammad Luqman
 * @brief This ROS2 node calculates the angular velocity for a robot moving in a circle
 * of a specified radius and publishes Twist messages for robot motion.
 * @organization Robotisim
 */

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class Radius_motion : public rclcpp::Node
{
public:
    Radius_motion()
    : Node("Radius_motion_node")
    {
        this->declare_parameter<double>("linear_velocity", 1.1);
        this->declare_parameter<double>("radius", 1.0);

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&Radius_motion::timer_callback, this));
    }

    void timer_callback()
    {
        double linear_velocity =this->get_parameter("linear_velocity").as_double();
        double radius = this->get_parameter("radius").as_double();
        RCLCPP_INFO(this->get_logger(), "VEL_Linear: %f - Radius: %f", linear_velocity,radius);
        // Calculate angular velocity and create the Twist message
        auto message = geometry_msgs::msg::Twist();
        message.linear.x =linear_velocity;
        message.angular.z = linear_velocity/radius;
        publisher_->publish(message);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Radius_motion>());
    rclcpp::shutdown();
    return 0;
}
