// This code defines a ROS2 node that publishes velocities to wheels using a Float64MultiArray message.
// It demonstrates periodic message publishing with a wall timer.
// Author: Robotisim

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class Float64MultiArrayPublisher : public rclcpp::Node {
public:
    Float64MultiArrayPublisher()
        : Node("float64_multi_array_publisher") {
        _publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("/wheels_velocity_controller/commands", 10);
        _timer = this->create_wall_timer(
            500ms, std::bind(&Float64MultiArrayPublisher::publishMessage, this));
    }

private:
    void publishMessage() {
        std_msgs::msg::Float64MultiArray message;
        message.data = {1.5, 1.5}; // Set both wheel velocities to 1.5 m/s

        RCLCPP_INFO(this->get_logger(), "Publishing: '%f, %f'",
                    message.data[0], message.data[1]);
        _publisher->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timer;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Float64MultiArrayPublisher>());
    rclcpp::shutdown();
    return 0;
}
