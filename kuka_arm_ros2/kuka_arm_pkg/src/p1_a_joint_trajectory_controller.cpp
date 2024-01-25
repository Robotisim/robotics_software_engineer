#include "rclcpp/rclcpp.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

class TrajectoryPublisher : public rclcpp::Node
{
public:
    TrajectoryPublisher()
    : Node("trajectory_publisher_node")
    {
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/kuka_arm_controller/joint_trajectory", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&TrajectoryPublisher::timer_callback, this));
        joints_ = {"joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"};
        goal_positions_ = {2.5,1.5,-2.5,0.5,0.3,0.1};
    }

private:
    void timer_callback()
    {
        auto message = trajectory_msgs::msg::JointTrajectory();
        message.joint_names = joints_;
        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
        point.positions = goal_positions_;
        point.time_from_start.sec = 2;
        message.points.push_back(point);
        publisher_->publish(message);
    }
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::string> joints_;
    std::vector<double> goal_positions_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryPublisher>());
    rclcpp::shutdown();
    return 0;
}
