#include "ekf_lib.hpp"
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

typedef Eigen::Matrix<double, 2, 1> Vector2d;
typedef Eigen::Matrix<double, 4, 1> Vector4d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;
typedef Eigen::Matrix<double, 4, 5> Matrix45d;
typedef Eigen::Matrix<double, 5, 1> Vector5d;
typedef Eigen::Matrix<double, 5, 5> Matrix5d;
typedef Eigen::Matrix<double, 2, 5> Matrix25d;

class ExtendedKalmanFilter_Node : public rclcpp::Node {
public:
    ExtendedKalmanFilter_Node() : Node("ekf_node") {
        setMatrices();

        markerArraySub = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "kitti/marker_array", 10,
            std::bind(&ExtendedKalmanFilter_Node::markerArrayCallback, this, std::placeholders::_1));

        // Create a subscriber for sensor_msgs::msg::Imu
        imuSub = this->create_subscription<sensor_msgs::msg::Imu>(
            "kitti/imu", 10, std::bind(&ExtendedKalmanFilter_Node::imuCallback, this, std::placeholders::_1));
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        imu_msg = msg;
    }

    void markerArrayCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        marker_array_msg = msg;
        estimation();
    }

    void estimation() {
        RCLCPP_INFO(this->get_logger(), "--------> Starting EKF Iterations ");

        ekf.predict();
        RCLCPP_INFO(this->get_logger(), "Prediction step completed. State and covariance matrices have been predicted.");

        measurements << marker_array_msg->markers[0].pose.position.x, marker_array_msg->markers[0].pose.position.y,
                       imu_msg->linear_acceleration.x, imu_msg->angular_velocity.z;
        RCLCPP_INFO(this->get_logger(), "Measurements obtained from sensors: x = %f, y = %f, ax = %f, az = %f",
                    measurements[0], measurements[1], measurements[2], measurements[3]);

        std::vector<double> cov_values = {cov_gps, cov_gps, cov_imu, cov_imu};
        ekf.updateR(cov_values);
        ekf.update(measurements);
        RCLCPP_INFO(this->get_logger(), "Update step completed. State and covariance matrices have been corrected.");
    }

    void setMatrices() {
        P_in << 1, 0, 0, 0, 0,
                0, 1, 0, 0, 0,
                0, 0, 1, 0, 0,
                0, 0, 0, 1, 0,
                0, 0, 0, 0, 1;
        // State Transition Matrix
        F_in << 1, 0, 0, 0, 0,
                0, 1, 0, 0, 0,
                0, 0, 1, 0, 0,
                0, 0, 0, 1, 0,
                0, 0, 0, 0, 1;
        // Measurement Matrix
        H_in.setZero();
        // Measurement Noise Covariance Matrix
        R_in.setZero();
        // Process Noise Covariance Matrix
        Q_in << 0.1, 0, 0, 0, 0,
                0, 0.1, 0, 0, 0,
                0, 0, 0.1, 0, 0,
                0, 0, 0, 0.1, 0,
                0, 0, 0, 0, 0.1;

        ekf.dt = 0.1;

        ekf.x_pred_ = x_in;
        ekf.z_pred_ = Vector4d::Zero();

        ekf.initialize(x_in, P_in, F_in, H_in, R_in, Q_in);
    }

    Vector5d x_in;
    Matrix5d P_in;
    Matrix5d F_in;
    Matrix45d H_in;
    Matrix4d R_in;
    Matrix5d Q_in;

    Vector4d measurements;

    // The following values are the covariances for the GPS and IMU sensors taken from the KITTI data.
    double cov_imu = 0.012727922061358;
    double cov_gps = 0.028452340807104;

    ExtendedKalmanFilter ekf;
    sensor_msgs::msg::Imu::SharedPtr imu_msg;
    visualization_msgs::msg::MarkerArray::SharedPtr marker_array_msg;

    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr markerArraySub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ExtendedKalmanFilter_Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}