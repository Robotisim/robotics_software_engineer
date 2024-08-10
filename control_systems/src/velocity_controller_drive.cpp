#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class Float64MultiArrayPublisher : public rclcpp::Node
{
public:
    Float64MultiArrayPublisher()
        : Node("float64_multi_array_publisher")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/wheels_velocity_controller/commands", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&Float64MultiArrayPublisher::publish_message, this));
    }

private:
    void publish_message()
    {
        std_msgs::msg::Float64MultiArray message;
        message.data = {1.5, 1.5};

        RCLCPP_INFO(this->get_logger(), "Publishing: '%f, %f'",
                    message.data[0], message.data[1]);
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Float64MultiArrayPublisher>());
    rclcpp::shutdown();
    return 0;
}
