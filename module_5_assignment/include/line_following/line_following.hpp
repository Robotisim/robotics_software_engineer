#ifndef LINE_FOLLOWING_HPP
#define LINE_FOLLOWING_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "pid_lib/pid_lib.hpp"
#include <vector>

class LineFollowing : public rclcpp::Node
{
public:
    LineFollowing();  // Constructor
    ~LineFollowing(); // Destructor

private:
    void image_callback(const std::shared_ptr<sensor_msgs::msg::Image> camera_msg);
    void timer_callback();

    // Publishers and subscribers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisher;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _subscriber_camera_image_raw;
    rclcpp::TimerBase::SharedPtr _timer;

    // PID controllers
    std::unique_ptr<PID> _angular_pid;
    std::unique_ptr<PID> _linear_pid;

    // Internal parameters and state
    bool _bool_update = false;
    double _dt = 0.1;
    double _anglr_vlcty;
    int _cntrl1, _cntrl2, _cntrl3, _cntrl4;
};

#endif // LINE_FOLLOWING_HPP
