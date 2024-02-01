/**
 * @file p6_b_sdf_spawner.cpp
 * @author Muhammad Luqman
 * @brief This ROS2 node reads a file into a string and sends a service request to spawn an entity in a simulation.
 * @organization Robotisim
 */
#include "rclcpp/rclcpp.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"

#include <fstream>
#include <streambuf>
#include <string>
#include <iostream>
#include <chrono>

// Function to read a file into a string
std::string readFile(const std::string& filePath) {
    std::ifstream fileStream(filePath);
    return std::string((std::istreambuf_iterator<char>(fileStream)),
                        std::istreambuf_iterator<char>());
}

int main(int argc, char * argv[]) {
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Check for correct number of arguments
    if(argc < 3){
        std::cout << "Provide path to sdf and a name for the entity" << std::endl;
        return -1;
    }

    // Create node and client
    auto node = rclcpp::Node::make_shared("Spawning_Node");
    auto client = node->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");

    // Wait for service
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(node->get_logger(), "Waiting for service to appear...");
    }
    RCLCPP_INFO(node->get_logger(), "Connected to spawner");

    // Create and populate request
    auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
    request->name = argv[2];
    request->xml = readFile(argv[1]);
    if(argc > 4){
        request->initial_pose.position.x = std::stof(argv[3]);
        request->initial_pose.position.y = std::stof(argv[4]);
    }

    // Send request
    RCLCPP_INFO(node->get_logger(), "Sending service request to `/spawn_entity`");
    auto result_future = client->async_send_request(request);

    // Handle response
    if (rclcpp::spin_until_future_complete(node, result_future) != rclcpp::FutureReturnCode::TIMEOUT)
    {
        RCLCPP_INFO(node->get_logger(), "Response: %s", result_future.get()->status_message.c_str());
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to call service /spawn_entity");
    }

    // Shutdown
    RCLCPP_INFO(node->get_logger(), "Done! Shutting down node.");
    rclcpp::shutdown();

    return 0;
}
