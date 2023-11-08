/**
 * @file p3_a_acceleration.cpp
 * @author Muhammad Luqman
 * @brief This ROS2 node calculates and outputs the acceleration of a robot and publishes Twist messages for robot motion.
 * @organization Robotisim
 */
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class Acceleration_node : public rclcpp::Node
{
public:
    Acceleration_node()
    : Node("acceleration_cal_node"), last_velocity_(0.0), last_time_(this->now())
    {
        this->declare_parameter<double>("linear_velocity", 2.0);

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&Acceleration_node::timer_callback, this));
    }

    void timer_callback()
    {
        auto current_time = this->now();
        auto current_velocity = this->get_parameter("linear_velocity").as_double();

        // calculate acceleration
        double dt = (current_time - last_time_).seconds();
        double acceleration = (current_velocity - last_velocity_) / dt;

        RCLCPP_INFO(this->get_logger(), "Acceleration: %f m/s^2", acceleration);

        // update last velocity and time
        last_velocity_ = current_velocity;
        last_time_ = current_time;
        // Create and publish the Twist message for robot motion
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = current_velocity;
        publisher_->publish(message);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    double last_velocity_;
    rclcpp::Time last_time_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Acceleration_node>());
    rclcpp::shutdown();
    return 0;
}
