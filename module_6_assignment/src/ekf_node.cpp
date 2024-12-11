#include "ekf_lib.hpp"
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>  // Correct ROS 2 header
#include <nav_msgs/msg/odometry.hpp>

class EKFNode : public rclcpp::Node
{
public:
    EKFNode() : Node("ekf_node")
    {
        this->declare_parameter<std::string>("imu_topic", "/imu");
        this->declare_parameter<std::string>("gps_topic", "/odom");
        this->declare_parameter<std::string>("output_topic", "/fused_odom");
        this->declare_parameter<double>("dt", 0.1);

        imu_topic_ = this->get_parameter("imu_topic").as_string();
        gps_topic_ = this->get_parameter("gps_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        dt_ = this->get_parameter("dt").as_double();

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, 10, std::bind(&EKFNode::imu_callback, this, std::placeholders::_1));
        gps_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            gps_topic_, 10, std::bind(&EKFNode::gps_callback, this, std::placeholders::_1));

        fused_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(output_topic_, 10);

        initialize_ekf();
        RCLCPP_INFO(this->get_logger(), "EKF Node Initialized");
    }

private:
    void initialize_ekf()
    {
        // State vector [x, y, vx, vy, theta]
        Eigen::VectorXd x(5);
        x << 0, 0, 0, 0, 0;

        // Covariance matrix (initial uncertainty)
        Eigen::MatrixXd P = Eigen::MatrixXd::Identity(5, 5);

        // State transition matrix (F)
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(5, 5);

        // Measurement matrix (H) - Adjusted for 5D measurement
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(5, 5);

        // Measurement noise covariance (R) - Adjusted for 5D measurement
        Eigen::MatrixXd R = Eigen::MatrixXd::Identity(5, 5);

        // Process noise covariance (Q)
        Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(5, 5) * 0.1;

        ekf_.initialize(x, P, F, H, R, Q);
        ekf_.dt = dt_; // Set time step in EKF
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Extract IMU data (linear acceleration and angular velocity)
        imu_data_ << msg->linear_acceleration.x, msg->angular_velocity.z;
        imu_received_ = true;
    }

    void gps_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!imu_received_)
        {
            RCLCPP_WARN(this->get_logger(), "IMU data not received yet, skipping GPS callback.");
            return;
        }

        // Extract GPS data (position x, y)
        gps_data_ << msg->pose.pose.position.x, msg->pose.pose.position.y;

        // Prepare 5D measurement vector (position + velocity + orientation)
        Eigen::VectorXd imu_measurement(3); // Assuming IMU gives [vx, vy, theta]
        imu_measurement << imu_data_[0], imu_data_[1], imu_data_[2];

        // Construct the full 5D measurement vector
        Eigen::VectorXd measurement(5);
        measurement << gps_data_, imu_measurement;

        // Predict and update the EKF with the 5D measurement
        ekf_.predict();
        ekf_.update(measurement);

        // Prepare and publish the fused state
        nav_msgs::msg::Odometry fused_msg;
        fused_msg.header.stamp = this->now();
        fused_msg.header.frame_id = "map";

        // Publish the fused state (position, velocity, orientation)
        fused_msg.pose.pose.position.x = ekf_.x_(0);  // x position
        fused_msg.pose.pose.position.y = ekf_.x_(1);  // y position
        fused_msg.twist.twist.linear.x = ekf_.x_(2);  // x velocity
        fused_msg.twist.twist.angular.z = ekf_.x_(4); // orientation (theta)

        fused_pub_->publish(fused_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_pub_;

    std::string imu_topic_;
    std::string gps_topic_;
    std::string output_topic_;
    double dt_;

    Eigen::VectorXd imu_data_{2}; // IMU data (acceleration, angular velocity)
    Eigen::VectorXd gps_data_{2}; // GPS data (position x, y)
    bool imu_received_ = false;

    ExtendedKalmanFilter ekf_; // EKF instance
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EKFNode>());
    rclcpp::shutdown();
    return 0;
}
