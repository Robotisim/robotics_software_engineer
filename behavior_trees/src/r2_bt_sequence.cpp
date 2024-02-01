#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <chrono>
#include <iostream>
#include <memory>

using namespace std::chrono_literals;

class bt_tbsim_driver : public rclcpp::Node {
public:
    bt_tbsim_driver(const std::string& name) : Node(name) {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    }

    BT::NodeStatus checkBattery() {
        // Placeholder for actual battery check logic
        RCLCPP_INFO(this->get_logger(), "Checking battery status");
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus driveStraight() {
        RCLCPP_INFO(this->get_logger(), "Driving Straight");
        geometry_msgs::msg::Twist msg;
        msg.linear.x = 0.5;
        publisher_->publish(msg);
        rclcpp::sleep_for(1s);
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus turnRight() {
        RCLCPP_INFO(this->get_logger(), "Turning Right");
        geometry_msgs::msg::Twist msg;
        msg.angular.z = 1.0;
        publisher_->publish(msg);
        rclcpp::sleep_for(1s);
        return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus TangentMove() {
        RCLCPP_INFO(this->get_logger(), "Move tangent");
        geometry_msgs::msg::Twist msg;
        msg.linear.x = 0.5;
        msg.angular.z = -1.0;
        publisher_->publish(msg);
        rclcpp::sleep_for(1s);
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bt_tbsim_driver>("bt_actions_node");

    BT::BehaviorTreeFactory factory;

    // Register the action nodes
    factory.registerSimpleCondition("CheckBattery",
    std::bind(&bt_tbsim_driver::checkBattery, node));

    factory.registerSimpleAction("DriveStraight",
    std::bind(&bt_tbsim_driver::driveStraight, node));

    factory.registerSimpleAction("TurnRight",
    std::bind(&bt_tbsim_driver::turnRight, node));

    factory.registerSimpleAction("TangentMove",
    std::bind(&bt_tbsim_driver::TangentMove, node));

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("behavior_trees");
    std::string bt_xml_file = package_share_directory + "/trees/r2_bt_sequence_tree.xml";

    auto tree = factory.createTreeFromFile(bt_xml_file);

    std::cout << "Starting Behavior Tree" << std::endl;
    while (rclcpp::ok()) {
        tree.tickOnce();
        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}
