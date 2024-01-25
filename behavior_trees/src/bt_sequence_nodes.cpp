#include "behaviortree_cpp/bt_factory.h"
#include <iostream>


BT::NodeStatus batteryCharging() {
  std::cout << "Battery is charging" << std::endl;
  return BT::NodeStatus::SUCCESS;
  }


BT::NodeStatus batteryTemp() {
  std::cout << "Checking battery temperature" << std::endl;
  // Insert condition to simulate temperature check
  bool temp_ok = false; // Assume temperature is okay
  return temp_ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus batterySupplying() {
  std::cout << "Battery supplying power" << std::endl;
  return BT::NodeStatus::FAILURE;
}

int main(){
  BT::BehaviorTreeFactory factory;

  // Register functions as nodes
  factory.registerSimpleCondition("BatteryCharging",std::bind(batteryCharging));
  factory.registerSimpleCondition("BatteryTemp", std::bind(batteryTemp));
  factory.registerSimpleAction("BatterySupplying", std::bind(batterySupplying));

  // Create a simple sequence tree
  auto tree = factory.createTreeFromFile("/home/luqman/ros2_ws/src/behavior_trees/trees/bt_sequence_nodes_tree.xml");

  // Execute the tree
  std::cout << "Starting Behavior Tree" << std::endl;
  BT::NodeStatus status = tree.tickOnce();
  if (status == BT::NodeStatus::SUCCESS) {
    std::cout << "All operations successful" << std::endl;
  } else {
    std::cout << "There was a failure in the operations" << std::endl;
  }

  return 0;
}
