#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class DroneController : public rclcpp::Node
{
public:
    DroneController()
    : Node("drone_controller")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/drone/cmd_vel", 10);

        auto timer_callback =
        [this]() -> void {
            auto message = geometry_msgs::msg::Twist();

            // Forward
            message.linear.x = 1.0;
            publisher_->publish(message);
            rclcpp::sleep_for(1s);

            // Left
            message.linear.x = 0.0;
            message.angular.z = 1.0;
            publisher_->publish(message);
            rclcpp::sleep_for(1s);

            // Right
            message.angular.z = -1.0;
            publisher_->publish(message);
            rclcpp::sleep_for(1s);

            // Back
            message.angular.z = 0.0;
            message.linear.x = -1.0;
            publisher_->publish(message);
            rclcpp::sleep_for(1s);

            // Stop
            message.linear.x = 0.0;
            publisher_->publish(message);
        };
        timer_ = this->create_wall_timer(5000ms, timer_callback);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneController>());
    rclcpp::shutdown();
    return 0;
}
