#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <iostream>

using namespace std::chrono_literals;
using namespace BT;

class DriveStraight : public SyncActionNode {
public:
    DriveStraight(const std::string& name, const NodeConfiguration& config)
        : SyncActionNode(name, config) {
        node_ = rclcpp::Node::make_shared("drive_straight_node");
        publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    }

    NodeStatus tick() override {
        RCLCPP_INFO(node_->get_logger(), "Driving Straight");
        geometry_msgs::msg::Twist msg;
        msg.linear.x = 1.0;
        publisher_->publish(msg);
        rclcpp::sleep_for(1s);
        return NodeStatus::SUCCESS;
    }

    static PortsList providedPorts() {
        return {};
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

class TurnRight : public SyncActionNode {
public:
    TurnRight(const std::string& name, const NodeConfiguration& config)
        : SyncActionNode(name, config) {
        node_ = rclcpp::Node::make_shared("turn_right_node");
        publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    }

    NodeStatus tick() override {
        RCLCPP_INFO(node_->get_logger(), "Turning Right");
        geometry_msgs::msg::Twist msg;
        msg.angular.z = -1.0;
        publisher_->publish(msg);
        rclcpp::sleep_for(1s);
        return NodeStatus::SUCCESS;
    }

    static PortsList providedPorts() {
        return {};
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<DriveStraight>("DriveStraight");
    factory.registerNodeType<TurnRight>("TurnRight");

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("behavior_trees");
    std::string bt_xml_file = package_share_directory + "/trees/sequence_nodes.xml";

    auto tree = factory.createTreeFromFile(bt_xml_file);

    std::cout << "Starting Behavior Tree" << std::endl;
    while (rclcpp::ok()) {
        tree.tickOnce();
        rclcpp::spin_some(rclcpp::Node::make_shared("bt_node"));
    }

    rclcpp::shutdown();
    return 0;
}
