#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "custom_interfaces/msg/camera_lidar.hpp"

class CameraLidarPublisher : public rclcpp::Node {
public:
    CameraLidarPublisher() : Node("camera_lidar_publisher") {

        publisher_ = this->create_publisher<custom_interfaces::msg::CameraLidar>("camera_lidar_topic", 10);


        camera_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera_topic", 10, std::bind(&CameraLidarPublisher::camera_callback, this, std::placeholders::_1));

        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "lidar_topic", 10, std::bind(&CameraLidarPublisher::lidar_callback, this, std::placeholders::_1));
    }

private:
    void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        camera_data_ = *msg;
        publish_combined_message();
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        lidar_data_ = *msg;
        publish_combined_message();
    }

    void publish_combined_message() {
        auto combined_msg = custom_interfaces::msg::CameraLidar();
        combined_msg.camera_data = camera_data_;
        combined_msg.lidar_data = lidar_data_;
        publisher_->publish(combined_msg);
    }

    rclcpp::Publisher<custom_interfaces::msg::CameraLidar>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    sensor_msgs::msg::Image camera_data_;
    sensor_msgs::msg::LaserScan lidar_data_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraLidarPublisher>());
    rclcpp::shutdown();
    return 0;
}
