/**
 * @file p5_c_camera_data_sub.cpp
 * @author Muhammad Luqman
 * @brief This ROS2 node receives Image messages, converts the images to grayscale, and displays the images.
 * @organization Robotisim
 */
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class camera_subscriber : public rclcpp::Node
{
public:
    camera_subscriber()
    : Node("camera_subscriber_node")
    {
        this->declare_parameter<int>("lower_threshold",200);
        this->declare_parameter<int>("upper_threshold",250);

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&camera_subscriber::camera_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "------ Node Started -----");
    }



private:
        int count_1_edge=0;int count_2_edge=0; int count_3_edge=0; int count_4_edge=0;

    void camera_callback(const sensor_msgs::msg::Image::SharedPtr camera_msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(camera_msg,"bgr8");
        cv::Mat gray_image, canny_image;
        cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);

        cv::imshow("Image",gray_image);
        cv::waitKey(1);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char ** argv)
{


    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<camera_subscriber>());
    rclcpp::shutdown();
    return 0;
}
