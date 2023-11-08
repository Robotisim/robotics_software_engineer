/**
 * @file p5_d_line_following.cpp
 * @author Muhammad Luqman
 * @brief This ROS2 node receives Image messages, applies the Canny edge detection algorithm to the images,
 * finds the midpoint of the detected line, calculates the error between the midpoint and the center of the image,
 * and publishes Twist messages to control robot motion.
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
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&camera_subscriber::camera_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "\n------ Node Started -----\n");
    }



private:
        int count_1_edge=0;int count_2_edge=0; int count_3_edge=0; int count_4_edge=0;

    void camera_callback(const sensor_msgs::msg::Image::SharedPtr camera_msg)
    {
        auto velocity_msg = geometry_msgs::msg::Twist();
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(camera_msg,"bgr8");
        cv::Mat gray_image, canny_image;
        cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);

        int upper_threshold= this->get_parameter("upper_threshold").as_int();
        int lower_threshold= this->get_parameter("lower_threshold").as_int();
        cv::Canny(gray_image,canny_image,lower_threshold,upper_threshold);

        int row=150,column=0;
        // Apply Canny edge detection and find midpoint of the line
        cv::Mat roi = canny_image(cv::Range(row,row+240) , cv::Range(column , column+640));


        std::vector<int> edge;
        for(int i=0;i<640;++i){
            if(roi.at<uchar>(160,i)==255){
                edge.push_back(i);
            }
        }

        // Finding Mid point
        int mid_area = edge[1] - edge[0];
        int mid_point = edge[0] + mid_area/2;
        int robot_mid_point = 640/2;

        cv::circle(roi,cv::Point(mid_point,160),2,cv::Scalar(255,255,255),-1);
        cv::circle(roi,cv::Point(robot_mid_point,160),5,cv::Scalar(255,255,255),-1);

        // Control Algorithms
        velocity_msg.linear.x=0.1;
        double error = robot_mid_point - mid_point;
        if(error<0){
           RCLCPP_INFO(this->get_logger(), "Turn Right");
           velocity_msg.angular.z=-angular_vel;
        }
        else {
            RCLCPP_INFO(this->get_logger(), "Turn Left");
            velocity_msg.angular.z=angular_vel;
        }

        publisher_->publish(velocity_msg);

        cv::imshow("Image",roi);
        cv::waitKey(1);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    double angular_vel=0.3;
};


int main(int argc, char ** argv)
{


    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<camera_subscriber>());
    rclcpp::shutdown();
    return 0;
}
