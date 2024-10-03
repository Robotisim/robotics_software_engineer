#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class CameraSubscriber : public rclcpp::Node {
public:
  CameraSubscriber()
  : Node("camera_subscriber_node"), _angularVel(0.3), _integral(0.0) {
    this->declare_parameter<int>("lower_threshold", 200);
    this->declare_parameter<int>("upper_threshold", 250);
    this->declare_parameter<double>("Kp", 0.01);
    this->declare_parameter<double>("Ki", 0.001);

    _publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    _subscription = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10,
      std::bind(&CameraSubscriber::cameraCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "\n------ Node Started -----\n");
  }

private:
  void cameraCallback(const sensor_msgs::msg::Image::SharedPtr cameraMsg) {
    auto velocityMsg = geometry_msgs::msg::Twist();
    cv_bridge::CvImagePtr cvPtr;
    cvPtr = cv_bridge::toCvCopy(cameraMsg, "bgr8");
    cv::Mat grayImage, cannyImage;
    cv::cvtColor(cvPtr->image, grayImage, cv::COLOR_BGR2GRAY);

    int upperThreshold = this->get_parameter("upper_threshold").as_int();
    int lowerThreshold = this->get_parameter("lower_threshold").as_int();
    cv::Canny(grayImage, cannyImage, lowerThreshold, upperThreshold);

    // Process Canny image to find the line's midpoint
    int row = 150, column = 0;
    cv::Mat roi = cannyImage(cv::Range(row, row + 240), cv::Range(column, column + 640));

    std::vector<int> edge;
    for (int i = 0; i < 640; ++i) {
      if (roi.at<uchar>(160, i) == 255) {
        edge.push_back(i);
      }
    }

    if (!edge.empty()) {
      int midArea = edge.back() - edge.front();
      int midPoint = edge.front() + midArea / 2;
      int robotMidPoint = 640 / 2;

      // Calculate error and adjust robot's direction using PI controller
      double error = robotMidPoint - midPoint;

      // Proportional term
      double Kp = this->get_parameter("Kp").as_double();
      double proportional = Kp * error;

      // Integral term (accumulating error)
      double Ki = this->get_parameter("Ki").as_double();
      _integral += error;
      double integral = Ki * _integral;

      // Total control (angular velocity adjustment)
      double control = proportional + integral;

      velocityMsg.linear.x = 0.1; // Forward velocity remains constant
      velocityMsg.angular.z = control;

      _publisher->publish(velocityMsg);

      // Visualization
      cv::circle(roi, cv::Point(midPoint, 160), 2, cv::Scalar(255, 255, 255), -1);
      cv::circle(roi, cv::Point(robotMidPoint, 160), 5, cv::Scalar(255, 255, 255), -1);
      cv::imshow("Image", roi);
      cv::waitKey(1);
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisher;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _subscription;
  double _angularVel;
  double _integral;  // Integral accumulator
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraSubscriber>());
  rclcpp::shutdown();
  return 0;
}
