#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include <iostream>
#include <string>

using namespace BT;

class ExtinguishFire : public SyncActionNode {
public:
    ExtinguishFire(const std::string& name, const NodeConfiguration& config)
        : SyncActionNode(name, config) {}

    static PortsList providedPorts() {
        return { InputPort<std::string>("area") };
    }

    NodeStatus tick() override {
        auto area = getInput<std::string>("area");
        if (!area) {
            throw RuntimeError("missing required input [area]: ", area.error());
        }
        std::cout << "Extinguishing fire in area: " << area.value() << std::endl;
        // Simulate fire extinguishing logic
        return NodeStatus::SUCCESS;
    }
};

class SendToAmbulance : public SyncActionNode {
public:
    SendToAmbulance(const std::string& name, const NodeConfiguration& config)
        : SyncActionNode(name, config) {}

    static PortsList providedPorts() {
        return { InputPort<std::string>("emergency_message") };
    }

    NodeStatus tick() override {
        auto message = getInput<std::string>("emergency_message");
        if (!message) {
            throw RuntimeError("missing required input [emergency_message]: ", message.error());
        }
        std::cout << "Sending message to ambulance: " << message.value() << std::endl;
        // Simulate sending message logic
        return NodeStatus::SUCCESS;
    }
};

int main() {
    BehaviorTreeFactory factory;
    factory.registerNodeType<ExtinguishFire>("ExtinguishFire");
    factory.registerNodeType<SendToAmbulance>("SendToAmbulance");

    auto tree = factory.createTreeFromFile("../trees/drone_fire_extinguish_bt.xml");
    tree.tickWhileRunning();
    return 0;
}


