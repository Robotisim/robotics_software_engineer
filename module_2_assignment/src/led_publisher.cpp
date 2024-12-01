#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

class LedPublisher : public rclcpp::Node {
public:
    LedPublisher() : Node("led_publisher"), state_{0, 1, 0} {
        publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("led_states", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&LedPublisher::publish_led_states, this));
    }

private:
    void publish_led_states() {
        auto message = std_msgs::msg::UInt8MultiArray();
        message.data = state_;
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Published LED states: [%d, %d, %d]",
                    state_[0], state_[1], state_[2]);
        // Toggle the states
        for (auto &s : state_) {
            s = (s == 0) ? 1 : 0;
        }
    }

    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<uint8_t> state_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LedPublisher>());
    rclcpp::shutdown();
    return 0;
}
