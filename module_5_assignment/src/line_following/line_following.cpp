#include "line_following/line_following.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace std;

LineFollowing::LineFollowing() : Node("line_following"), _cntrl1(0), _cntrl2(0), _cntrl3(0), _cntrl4(0), _anglr_vlcty(0.21)
{
    const double angular_Z = 0.5;    // Maximum angular velocity
    const double Kp_Angular = 0.01;  // Angular PID Kp
    const double Ki_Angular = 0.01;  // Angular PID Ki
    const double Kd_Angular = 0.00;  // Angular PID Kd

    const double linear_X = 0.22;    // Maximum linear velocity
    const double Kp_Linear = 0.1;    // Linear PID Kp
    const double Ki_Linear = 0.01;   // Linear PID Ki
    const double Kd_Linear = 0.0;    // Linear PID Kd

    // Declare parameters
    this->declare_parameter<int>("low_threshold", 200);
    this->declare_parameter<int>("upper_threshold", 250);

    this->declare_parameter<double>("Kp_angular", Kp_Angular);
    this->declare_parameter<double>("Ki_angular", Ki_Angular);
    this->declare_parameter<double>("Kd_angular", Kd_Angular);

    this->declare_parameter<double>("linear_sp", linear_X);
    this->declare_parameter<double>("Kp_linear", Kp_Linear);
    this->declare_parameter<double>("Ki_linear", Ki_Linear);
    this->declare_parameter<double>("Kd_linear", Kd_Linear);

    // Initialize publishers and subscribers
    _publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    _subscriber_camera_image_raw = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10, std::bind(&LineFollowing::image_callback, this, _1));
    _timer = this->create_wall_timer(100ms, std::bind(&LineFollowing::timer_callback, this));

    // Initialize PID controllers for angular and linear velocities
    _angular_pid = std::make_unique<PID>(Kp_Angular, Ki_Angular, Kd_Angular, -angular_Z, angular_Z);
    _linear_pid = std::make_unique<PID>(Kp_Linear, Ki_Linear, Kd_Linear, -linear_X, linear_X);
}

void LineFollowing::timer_callback()
{
    _bool_update = true;
}

void LineFollowing::image_callback(const std::shared_ptr<sensor_msgs::msg::Image> camera_msg)
{
    static vector<int> prev_edge;
    int low_threshold_ = this->get_parameter("low_threshold").as_int();
    int upper_threshold_ = this->get_parameter("upper_threshold").as_int();

    // Update angular PID parameters
    _angular_pid->setKp(this->get_parameter("Kp_angular").as_double());
    _angular_pid->setKi(this->get_parameter("Ki_angular").as_double());
    _angular_pid->setKd(this->get_parameter("Kd_angular").as_double());

    // Update linear PID parameters
    double linear_sp = this->get_parameter("linear_sp").as_double();
    _linear_pid->setKp(this->get_parameter("Kp_linear").as_double());
    _linear_pid->setKi(this->get_parameter("Ki_linear").as_double());
    _linear_pid->setKd(this->get_parameter("Kd_linear").as_double());

    // Convert image to OpenCV format and apply Canny edge detection
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(camera_msg, "bgr8");

    cv::Mat gray_img, canny_img;
    cv::cvtColor(cv_ptr->image, gray_img, cv::COLOR_BGR2GRAY);
    cv::Canny(gray_img, canny_img, low_threshold_, upper_threshold_);

    // Extract region of interest (ROI)
    int row = 150;
    cv::Mat roi = canny_img(cv::Range(row, row + 240), cv::Range(0, 640));

    // Detect edges
    vector<int> edge;
    for (int i = 0; i < 640; i++) {
        if (roi.at<uchar>(160, i) == 255) {
            edge.push_back(i);
        }
    }

    switch (edge.size()) {
    case 1:
        _cntrl1 += 1;
        break;
    case 2:
        _cntrl2 += 1;
        break;
    case 3:
        _cntrl3 += 1;
        break;
    case 4:
        _cntrl4 += 1;
        break;
    default:
        break;
    }

    if (edge.empty()) {
        edge = prev_edge;
    }

    // Compute midpoints
    int mid_area = abs(edge[1] - edge[0]);
    int mid_point = edge[0] + mid_area / 2;
    int robot_midpoint = 640 / 2;

    // Visualize midpoints on the image
    cv::circle(roi, cv::Point(mid_point, 160), 2, cv::Scalar(255, 255, 255), -1);
    cv::circle(roi, cv::Point(robot_midpoint, 160), 5, cv::Scalar(255, 255, 255), -1);

    cv::imshow("Canny Img", roi);
    cv::waitKey(1);

    // Publish velocities using PID control
    geometry_msgs::msg::Twist velocity_msg_;
    if (_bool_update) {
        double angular_speed = -_angular_pid->stepPID(static_cast<double>(robot_midpoint), static_cast<double>(mid_point), 0.01);
        double linear_speed = -_linear_pid->stepPID(velocity_msg_.linear.x, linear_sp, _dt) / (1 + velocity_msg_.angular.z);

        velocity_msg_.angular.z = angular_speed / (1 + 1.5 * abs(linear_speed));
        velocity_msg_.linear.x = linear_speed / (1 + 1.5 * abs(velocity_msg_.angular.z));

        RCLCPP_INFO(this->get_logger(), "Linear: %.3f / Angular: %.3f", velocity_msg_.linear.x, velocity_msg_.angular.z);
        _publisher->publish(velocity_msg_);
        _bool_update = false;
    }

    if (!edge.empty()) {
        prev_edge = edge;
    }
}

LineFollowing::~LineFollowing()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down Line Following node");
}
