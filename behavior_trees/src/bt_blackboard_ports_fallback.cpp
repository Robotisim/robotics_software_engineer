#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include <iostream>
#include <string>

using namespace BT;
class MoveForward : public SyncActionNode {
public:
  MoveForward(const std::string &name, const NodeConfiguration &config)
      : SyncActionNode(name, config) {}

  static PortsList providedPorts() {
    return {InputPort<int>("sensor_distance")};
    }
  NodeStatus tick() override {
        auto distance_result = getInput<int>("sensor_distance");
        std::cout << "Moving forward " << distance_result.value()
                  << " cm." << std::endl;

        return NodeStatus::SUCCESS;
    }

};

class TurnLeft : public SyncActionNode {
public:
    TurnLeft(const std::string &name, const NodeConfiguration &config)
        : SyncActionNode(name, config) {}
    static PortsList providedPorts() {
        return {}; // No ports are used, but the method is still required
    }
    NodeStatus tick() override {
        std::cout << "Turning left." << std::endl;
        return NodeStatus::SUCCESS;
    }
};

class CheckDistance : public SyncActionNode {
public:
    CheckDistance(const std::string &name, const NodeConfiguration &config)
        : SyncActionNode(name, config) {}

    static PortsList providedPorts() {
        return { OutputPort<int>("sensor_distance") };
    }

    NodeStatus tick() override {
        std::cout << "Reading Distance Sensor" << std::endl;

        int distance = readSensorDistance();
        setOutput("sensor_distance", distance);
        if (distance < 15) {
            std::cout << "Obstacle detected! Distance is " << distance << " cm." << std::endl;
            return NodeStatus::FAILURE;
        } else {
            std::cout << "Area Clear" << std::endl;
            return NodeStatus::SUCCESS;
        }
    }

private:
    int readSensorDistance() {
        return 16;
    }
};


int main() {
  BehaviorTreeFactory factory;

    factory.registerNodeType<TurnLeft>("TurnLeft");
    factory.registerNodeType<CheckDistance>("CheckDistance");
  factory.registerNodeType<MoveForward>("MoveForward");

  auto tree =factory.createTreeFromFile("/home/luqman/ros2_ws/src/behavior_trees/"
                                 "trees/bt_blackboard_ports_fallback_tree.xml");
  tree.tickWhileRunning();
  return 0;
}
