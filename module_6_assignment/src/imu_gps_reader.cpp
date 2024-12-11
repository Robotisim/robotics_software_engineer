#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp" 


class IMUGPSReader : public rclcpp::Node
{
public:
    IMUGPSReader()
        : Node("imu_gps_reader")
    {
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu", 10, std::bind(&IMUGPSReader::imu_callback, this, std::placeholders::_1));
        
        gps_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "gps", 10, std::bind(&IMUGPSReader::gps_callback, this, std::placeholders::_1));
        
        // Publisher for fused data (e.g., velocity, position, etc.)
        sensor_fusion_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "sensor_fusion_data", 10);
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        latest_imu_ = msg;

        // Log IMU data
        RCLCPP_INFO(this->get_logger(), "Received IMU data: Orientation x=%.2f, y=%.2f, z=%.2f, w=%.2f",
                    latest_imu_->orientation.x,
                    latest_imu_->orientation.y,
                    latest_imu_->orientation.z,
                    latest_imu_->orientation.w);
        
        // Perform IMU-related computations or sensor fusion here (if needed)
    }

    void gps_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        latest_gps_ = msg;

        // Log GPS data
        RCLCPP_INFO(this->get_logger(), "Received GPS data: Position x=%.2f, y=%.2f, z=%.2f",
                    latest_gps_->pose.pose.position.x,
                    latest_gps_->pose.pose.position.y,
                    latest_gps_->pose.pose.position.z);

        // Fuse IMU and GPS data here (you might need a Kalman filter, for example)
        // Example: create a fused message for publishing
        geometry_msgs::msg::Twist fused_msg;
        fused_msg.linear.x = latest_gps_->pose.pose.position.x;  // Example: using GPS position
        fused_msg.linear.y = latest_gps_->pose.pose.position.y;
        fused_msg.angular.z = latest_imu_->orientation.z;  // Example: using IMU orientation
        
        // Publish fused data
        sensor_fusion_publisher_->publish(fused_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr sensor_fusion_publisher_;

    sensor_msgs::msg::Imu::SharedPtr latest_imu_;
    nav_msgs::msg::Odometry::SharedPtr latest_gps_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IMUGPSReader>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
