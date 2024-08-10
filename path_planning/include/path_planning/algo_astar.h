#ifndef ALGORITHM_ASTAR_H
#define ALGORITHM_ASTAR_H

#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <vector>
#include <iostream>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

class NodeAstar {
public:
    int x, y;
    float g_cost, h_cost, f_cost;
    std::shared_ptr<NodeAstar> parent;

    NodeAstar(int x, int y, std::shared_ptr<NodeAstar> parent = nullptr);

    void set_gcost(float cost);
    void set_hcost(float cost);
};

struct compare_node {
    bool operator()(const std::shared_ptr<NodeAstar>& a, const std::shared_ptr<NodeAstar>& b) const;
};

float heuristic(int x1, int y1, int x2, int y2);

std::pair<int, int> indexToCoordinates(int index, int width);

std::vector<geometry_msgs::msg::PoseStamped> astar_search(const nav_msgs::msg::OccupancyGrid& grid);

#endif // ALGORITHM_ASTAR_H