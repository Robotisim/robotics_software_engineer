#include "algo_rrt.hpp"
#include <algorithm>
#include <cmath>
#include <random>
#include <limits>

RRT_Planner::RRT_Planner() {

}

void RRT_Planner::setDomain(std::vector<int>& domain) {
    std::fill(DOMAIN.begin(), DOMAIN.end(), 0);
    std::copy(domain.begin(), domain.end(), DOMAIN.begin());
}


NodeRrt RRT_Planner::generateRandomNode(std::mt19937& gen) {
    std::uniform_int_distribution<int> distribution(0, GRID_WIDTH * GRID_HEIGHT - 1);
    int random_index = distribution(gen);

    return indexToCoordinate(random_index);
}

NodeRrt RRT_Planner::findNearestNode(std::vector<NodeRrt> const& nodes, NodeRrt const& randomNode) {
    NodeRrt nearestNode = nodes[0];
    double minDistance = std::numeric_limits<double>::max();

    for (auto const& node : nodes) {
        double distance = NodeRrt::heuristics(node, randomNode);
        if (distance < minDistance) {
            minDistance = distance;
            nearestNode = node;
        }
    }

    return nearestNode;
}

NodeRrt RRT_Planner::findNewConfig(NodeRrt const& nearestNode, NodeRrt const& randomNode) {
    int dx = randomNode.getX() - nearestNode.getX();
    int dy = randomNode.getY() - nearestNode.getY();

    float distance = NodeRrt::heuristicsEuclid(nearestNode, randomNode);

    float scaledDx = (distance > 0) ? (dx * STEP_SIZE) / distance : dx;
    float scaledDy = (distance > 0) ? (dy * STEP_SIZE) / distance : dy;

    int x = nearestNode.getX() + static_cast<int>(scaledDx);
    int y = nearestNode.getY() + static_cast<int>(scaledDy);

    return NodeRrt(x, y);
}

bool RRT_Planner::isObstacle(NodeRrt const& nearestNode, NodeRrt const& newNode) {
    if (newNode.getX() < 0 || newNode.getX() >= GRID_WIDTH || newNode.getY() < 0 || newNode.getY() >= GRID_HEIGHT) {
        return true;
    }

    if (DOMAIN[newNode.getY() * GRID_WIDTH + newNode.getX()] == 100) {
        return true;
    }

    int dx = newNode.getX() - nearestNode.getX();
    int dy = newNode.getY() - nearestNode.getY();
    int steps = std::max(std::abs(dx), std::abs(dy));

    for (int i = 0; i < steps; i++) {
        float x = nearestNode.getX() + (i * dx) / steps;
        float y = nearestNode.getY() + (i * dy) / steps;

        if (DOMAIN[static_cast<int>(y) * GRID_WIDTH + static_cast<int>(x)] == 100) {
            return true;
        }
    }

    return false;
}



bool RRT_Planner::isGoalFound(NodeRrt& newNode, NodeRrt const& goal) {
    return newNode == goal;
}

NodeRrt RRT_Planner::indexToCoordinate(int index) {
    int x = index % GRID_WIDTH;
    int y = index / GRID_WIDTH;

    return NodeRrt(x, y);
}



std::vector<NodeRrt> RRT_Planner::planPath(NodeRrt const& start, NodeRrt const& goal) {
    nodes.push_back(start);
    std::mt19937 gen(SEED);

    for (int i = 0; i < MAX_ITERATIONS; i++) {
        NodeRrt randomNode = generateRandomNode(gen);
        NodeRrt nearestNode = findNearestNode(nodes, randomNode);
        if (nearestNode == randomNode) {
            continue;
        }

        NodeRrt newNode = findNewConfig(nearestNode, randomNode);

        if (isObstacle(nearestNode, newNode)) {
            continue;
        }

        newNode.setParent(std::make_shared<NodeRrt>(nearestNode));
        nodes.push_back(newNode);

        if (isGoalFound(newNode, goal)) {
            path.push_back(newNode);
            auto parent = *newNode.getParent();

            while (&parent != &start) {
                path.push_back(parent);
                if (parent.getParent() == nullptr) {
                    break;
                }

                parent = *parent.getParent();
            }
            std::reverse(path.begin(), path.end());
            break;
        }
    }

    return path;
}