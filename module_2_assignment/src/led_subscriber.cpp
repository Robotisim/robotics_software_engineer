#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

class LedSubscriber : public rclcpp::Node {
public:
    LedSubscriber() : Node("led_subscriber") {
        subscription_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "led_states", 10, std::bind(&LedSubscriber::led_callback, this, std::placeholders::_1));
    }

private:
    void led_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
        if (msg->data.size() != 3) {
            RCLCPP_ERROR(this->get_logger(), "Received incorrect number of LED states!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Received LED states: [%d, %d, %d]",
                    msg->data[0], msg->data[1], msg->data[2]);
    }

    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LedSubscriber>());
    rclcpp::shutdown();
    return 0;
}
