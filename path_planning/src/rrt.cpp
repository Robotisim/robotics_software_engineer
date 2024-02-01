#include "path_planning/rrt.h"
#include <cmath>
#include <limits>
#include <algorithm>
#include <opencv2/opencv.hpp>

// Node constructor
Node::Node(float x_coord, float y_coord) : x(x_coord), y(y_coord), parent(nullptr), cost(0.0) {}

// RRT constructor
RRT::RRT(Node* start_node, Node* goal_node, std::vector<std::vector<float>> obstacle_list, std::vector<float> sampling_area,
         float expand_distance, float path_resolution, int goal_sample_rate, int max_iterations)
    : start(start_node), goal(goal_node), obstacles(obstacle_list), sampling_area(sampling_area),
      expand_distance(expand_distance), path_resolution(path_resolution), goal_sample_rate(goal_sample_rate),
      max_iterations(max_iterations), gen(rd()), goal_distribution(0, 100),
      area_distribution(sampling_area[0], sampling_area[1]) {
}

// RRT path planning
std::vector<Node*> RRT::planPath() {
    nodes.push_back(start);
    for (int i = 0; i < max_iterations; i++) {
        Node* rnd_node = getRandomNode();
        Node* nearest_node = getClosestNode(rnd_node);
        Node* new_node = createNewNode(nearest_node, rnd_node, expand_distance);

        if (new_node && isCollisionFree(new_node)) {
            nodes.push_back(new_node);
            if (isNodeCloseToGoal(new_node)) {
                return generateFinalPath(new_node);
            }
        }
    }
    // Return an empty path if no path found
    return std::vector<Node*>();
}

// Get random node
Node* RRT::getRandomNode() {
    if (goal_distribution(gen) > goal_sample_rate) {
        return new Node(area_distribution(gen), area_distribution(gen));
    }
    return new Node(goal->x, goal->y); // Biasing towards the goal
}

// Get closest node
Node* RRT::getClosestNode(Node* rnd_node) {
    Node* closest = nullptr;
    float min_dist = std::numeric_limits<float>::max();
    for (Node* node : nodes) {
        float dist = std::sqrt(std::pow(node->x - rnd_node->x, 2) + std::pow(node->y - rnd_node->y, 2));
        if (dist < min_dist) {
            min_dist = dist;
            closest = node;
        }
    }
    return closest;
}

// Create new node
Node* RRT::createNewNode(Node* nearest, Node* rnd_node, float max_extend_length) {
    // Calculate angle and distance from nearest to rnd_node
    std::vector<float> dist_and_angle = calculateDistanceAndAngle(nearest, rnd_node);
    float distance = dist_and_angle[0], angle = dist_and_angle[1];
    float new_distance = std::min(distance, max_extend_length);

    // Create a new node in the direction of rnd_node from nearest node
    Node* new_node = new Node(nearest->x + new_distance * cos(angle), nearest->y + new_distance * sin(angle));
    new_node->parent = nearest;
    return new_node;
}

// Check if the node is collision free
bool RRT::isCollisionFree(Node* node) {
    // Check if node is inside any of the obstacles
    for (auto& obstacle : obstacles) {
        float dist_to_obstacle = std::sqrt(std::pow(obstacle[0] - node->x, 2) + std::pow(obstacle[1] - node->y, 2));
        if (dist_to_obstacle <= obstacle[2]) {
            return false;
        }
    }
    return true;
}

// Check if node is close to the goal
bool RRT::isNodeCloseToGoal(Node* node) {
    float distance_to_goal = std::sqrt(std::pow(goal->x - node->x, 2) + std::pow(goal->y - node->y, 2));
    return distance_to_goal <= expand_distance;
}

// Generate final path from start to goal
std::vector<Node*> RRT::generateFinalPath(Node* last_node) {
    std::vector<Node*> path;
    Node* current = last_node;
    while (current != nullptr) {
        path.push_back(current);
        current = current->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

// Calculate distance and angle between two nodes
std::vector<float> RRT::calculateDistanceAndAngle(Node* from_node, Node* to_node) {
    float dx = to_node->x - from_node->x;
    float dy = to_node->y - from_node->y;
    float distance = std::sqrt(dx * dx + dy * dy);
    float angle = std::atan2(dy, dx);
    return std::vector<float>{distance, angle};
}
