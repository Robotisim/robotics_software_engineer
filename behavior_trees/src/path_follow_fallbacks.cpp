#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "turtlesim/msg/pose.hpp"

// ... (include other necessary headers)
turtlesim::msg::Pose current_pose;
using namespace std::chrono_literals;
using namespace BT;

class OrientTowardsGoal : public SyncActionNode {
public:
    OrientTowardsGoal(const std::string& name, const NodeConfiguration& config)
        : SyncActionNode(name, config), node_(rclcpp::Node::make_shared("orient_towards_goal_node")) {
        publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

    }

    NodeStatus tick() override {
        auto goal_x = getInput<double>("goal_x").value();
        auto goal_y = getInput<double>("goal_y").value();

        // Calculate the desired orientation (angle to goal)
        double desired_angle = std::atan2(goal_y - current_pose.y, goal_x - current_pose.x);

        // Calculate the difference between current orientation and the desired
        double angle_diff = desired_angle - current_pose.theta;

        // Normalize the angle
        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

        // Set angular velocity based on the angle difference
        geometry_msgs::msg::Twist msg;
        msg.angular.z = angle_diff; // Consider using a PID controller for smoother control

        publisher_->publish(msg);

        // Check if the orientation is approximately correct
        if (std::abs(angle_diff) < orientation_tolerance) { // orientation_tolerance is a predefined threshold
            // RCLCPP_INFO(node_->get_logger(), "Orientation fixed to Goal [%f, %f]", goal_x, goal_y);

            return NodeStatus::SUCCESS;
        }

        return NodeStatus::FAILURE;
    }

    static PortsList providedPorts() {
        return { InputPort<double>("goal_x"), InputPort<double>("goal_y") };
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    const double orientation_tolerance = 0.1; // Adjust as necessary
};



class MoveTowardsGoal : public SyncActionNode {
public:
    MoveTowardsGoal(const std::string& name, const NodeConfiguration& config)
        : SyncActionNode(name, config), node_(rclcpp::Node::make_shared("move_towards_goal_node")) {
        publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    }

    NodeStatus tick() override {
        auto goal_x = getInput<double>("goal_x").value();
        auto goal_y = getInput<double>("goal_y").value();

        double distance = std::sqrt(std::pow(current_pose.x - goal_x, 2) + std::pow(current_pose.y - goal_y, 2));
        // RCLCPP_INFO(node_->get_logger(), "Moving to current goal: [%f, %f]", goal_x, goal_y);

        if (distance < goal_tolerance) {
            // RCLCPP_INFO(node_->get_logger(), "Reached Goal [%f, %f]", goal_x, goal_y);

            return NodeStatus::SUCCESS;
        }

        // Calculate the desired orientation (angle to goal)
        double desired_angle = std::atan2(goal_y - current_pose.y, goal_x - current_pose.x);

        // Calculate the difference between current orientation and the desired
        double angle_diff = desired_angle - current_pose.theta;
        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

        // Move towards the goal if oriented correctly
        if (std::abs(angle_diff) < orientation_tolerance) {
            geometry_msgs::msg::Twist msg;
            msg.linear.x = linear_speed; // Set linear speed
            publisher_->publish(msg);
        }

        return NodeStatus::FAILURE;
    }

    static PortsList providedPorts() {
        return { InputPort<double>("goal_x"), InputPort<double>("goal_y") };
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    const double goal_tolerance = 0.3; // Adjust as necessary
    const double orientation_tolerance = 0.1; // Adjust as necessary
    const double linear_speed = 0.5; // Adjust as necessary
};


class CheckGoalReached : public ConditionNode {
public:
    CheckGoalReached(const std::string& name, const NodeConfiguration& config)
        : ConditionNode(name, config) {}

    NodeStatus tick() override {
        auto goal_x = getInput<double>("goal_x").value();
        auto goal_y = getInput<double>("goal_y").value();

        double distance = std::sqrt(std::pow(current_pose.x - goal_x, 2) + std::pow(current_pose.y - goal_y, 2));
        float goal_tolerance = 0.1;
        if (distance < goal_tolerance) { // goal_tolerance is a predefined threshold
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    static PortsList providedPorts() {
        return { InputPort<double>("goal_x"), InputPort<double>("goal_y") };
    }
};



void poseCallback(const turtlesim::msg::Pose::SharedPtr pose) {
    current_pose = *pose;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("go_to_goal_node");

    // Subscribe to the pose topic
    auto pose_subscriber = node->create_subscription<turtlesim::msg::Pose>(
        "/turtle1/pose", 10, poseCallback);

    // Behavior Tree setup
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<OrientTowardsGoal>("OrientTowardsGoal");
    factory.registerNodeType<MoveTowardsGoal>("MoveTowardsGoal");
    factory.registerNodeType<CheckGoalReached>("CheckGoalReached");

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("behavior_trees");
    std::string bt_xml_file = package_share_directory + "/trees/path_follow_fallbacks_bt.xml";
    auto tree = factory.createTreeFromFile(bt_xml_file);

    while (rclcpp::ok()) {
        tree.tickOnce();
        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}
