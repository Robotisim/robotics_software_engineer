#ifndef RRT_PLANNER_HPP
#define RRT_PLANNER_HPP

#include "algo_rrt_node.hpp"
#include <array>
#include <memory>
#include <random>
#include <vector>

class RRT_Planner {
public:
    RRT_Planner();
    NodeRrt START = NodeRrt(0, 0);
    NodeRrt GOAL = NodeRrt(20, 20);

    void setDomain(std::vector<int>& domain);
    static NodeRrt indexToCoordinate(int index);

    NodeRrt generateRandomNode(std::mt19937& gen);
    bool isObstacle(NodeRrt const& nearest_node, NodeRrt const& new_node);

    bool isGoalFound(NodeRrt& new_node, NodeRrt const& goal);
    NodeRrt findNearestNode(std::vector<NodeRrt> const& nodes, NodeRrt const& random_node);
    NodeRrt findNewConfig(NodeRrt const& nearest_node, NodeRrt const& random_node);

    std::vector<NodeRrt> planPath(NodeRrt const& start, NodeRrt const& goal);

private:


    void Occupany_Callback();
    void publishPath(std::vector<NodeRrt> const& path);

    int const MAX_ITERATIONS = 1000000;
    int const SEED = 0;

    static constexpr int GRID_WIDTH = 20;
    static constexpr int GRID_HEIGHT = 20;
    static constexpr float STEP_SIZE = 2.0F;

    std::array<int, GRID_WIDTH * GRID_HEIGHT> DOMAIN{};
    std::vector<NodeRrt> nodes;
    std::vector<NodeRrt> path;



};

#endif // RRT_PLANNER_HPP
