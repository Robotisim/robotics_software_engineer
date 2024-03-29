// Purpose:
// - Control a robot by sending velocity commands at regular intervals
// - Demonstrate the use of ROS2 publishers within a class
// Author: Robotisim

#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class RobotDriver : public rclcpp::Node {
public:
    // Constructor: sets up the robot driver node
    RobotDriver() : Node("robot_driver"), _count(0) {
        this->declare_parameter<std::string>("cmd_vel_topic", "/turtle1/cmd_vel");

        auto cmdVelTopic = this->get_parameter("cmd_vel_topic").as_string();

        _publisher = this->create_publisher<geometry_msgs::msg::Twist>(cmdVelTopic, 10);
        _timer = this->create_wall_timer(500ms, std::bind(&RobotDriver::timerCallback, this));
    }

private:
    // Timer callback: sends velocity commands
    void timerCallback() {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 0.5; // Set linear velocity
        message.angular.z = 0.4; // Set angular velocity
        RCLCPP_INFO(this->get_logger(), "Driving Turtle");
        _publisher->publish(message);
    }

    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisher;
    size_t _count;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotDriver>());
    rclcpp::shutdown();
    return 0;
}
