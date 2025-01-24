#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>
#include <cmath>

using NavigateToPose = nav2_msgs::action::NavigateToPose;

class Nav2InitializeAndSendGoal : public rclcpp::Node
{
public:
    Nav2InitializeAndSendGoal() : Node("nav2_initialize_and_send_goal")
{
    initial_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
    client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");

    set_initial_pose();
    send_goal();
}
private:
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;

    void set_initial_pose()
    {
        auto initial_pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        initial_pose_msg.header.frame_id = "map";
        initial_pose_msg.header.stamp = this->get_clock()->now();

        initial_pose_msg.pose.pose.position.x = -6.78;
        initial_pose_msg.pose.pose.position.y = 3.530;
        initial_pose_msg.pose.pose.orientation.z = std::sin(0.0 / 2.0);
        initial_pose_msg.pose.pose.orientation.w = std::cos(0.0 / 2.0);

        initial_pose_msg.pose.covariance[0] = 0.25;
        initial_pose_msg.pose.covariance[7] = 0.25;
        initial_pose_msg.pose.covariance[35] = 0.0685;

        for (int i = 0; i < 5; i++)
        {
            RCLCPP_INFO(this->get_logger(), "Publishing initial pose");
            initial_pose_publisher_->publish(initial_pose_msg);
            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
        rclcpp::sleep_for(std::chrono::seconds(2));

    }

    void send_goal()
    {
        if (!client_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available");
            return;
        }
      auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->get_clock()->now();
        goal_msg.pose.pose.position.x = -5.60;
        goal_msg.pose.pose.position.y = -3.980;
        goal_msg.pose.pose.orientation.z = std::sin(0.0 / 2.0);
        goal_msg.pose.pose.orientation.w = std::cos(0.0 / 2.0);

        auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        options.result_callback = [this](const auto &result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
                RCLCPP_INFO(this->get_logger(), "Goal reached");
            else
                RCLCPP_ERROR(this->get_logger(), "Failed to reach goal");
        };

        client_->async_send_goal(goal_msg, options);


    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Nav2InitializeAndSendGoal>());
    rclcpp::shutdown();
    return 0;
}
