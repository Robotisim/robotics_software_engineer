#include "path_planning/Dijkstra_node.h"
#include <limits>
#include <vector>
#include <queue>
#include <opencv2/opencv.hpp>

// Constructor implementation for Dijkstra_node
Dijkstra_node::Dijkstra_node(int x, int y, std::shared_ptr<Dijkstra_node> parent)
: x(x), y(y), g_cost(std::numeric_limits<float>::max()), parent(parent) {}

// Set the G cost for a node
void Dijkstra_node::setGCost(float cost) {
    g_cost = cost;
}

// Comparator for the priority queue in Dijkstra's algorithm
bool CompareNodeDijkstra::operator()(const std::shared_ptr<Dijkstra_node>& a, const std::shared_ptr<Dijkstra_node>& b) {
    return a->g_cost > b->g_cost;
}

// Dijkstra's Algorithm
void dijkstra_algorithm(cv::Mat &map, Dijkstra_node &start, Dijkstra_node &goal) {
    std::priority_queue<std::shared_ptr<Dijkstra_node>, std::vector<std::shared_ptr<Dijkstra_node>>, CompareNodeDijkstra> open_list;
    std::vector<std::vector<float>> cost_so_far(map.rows, std::vector<float>(map.cols, std::numeric_limits<float>::max()));

    auto start_node = std::make_shared<Dijkstra_node>(start.x, start.y);
    start_node->setGCost(0);
    open_list.push(start_node);
    cost_so_far[start.x][start.y] = 0;

    std::vector<std::pair<int, int>> directions = {
        {0, 1}, {1, 0}, {0, -1}, {-1, 0},
        {1, 1}, {-1, -1}, {1, -1}, {-1, 1}
    };

    while (!open_list.empty()) {
        auto current = open_list.top();
        open_list.pop();

        if (current->x == goal.x && current->y == goal.y) {
            // Path reconstruction
            while (current != nullptr) {
                cv::circle(map, cv::Point(current->x, current->y), 3, cv::Scalar(255, 0, 0), -1);
                current = current->parent;
            }
            break;
        }

        for (const auto& dir : directions) {
            int new_x = current->x + dir.first;
            int new_y = current->y + dir.second;

            if (new_x >= 0 && new_x < map.cols && new_y >= 0 && new_y < map.rows) {
                float new_cost = current->g_cost + ((dir.first == 0 || dir.second == 0) ? 1 : std::sqrt(2));
                if (new_cost < cost_so_far[new_x][new_y]) {
                    cost_so_far[new_x][new_y] = new_cost;
                    auto neighbor = std::make_shared<Dijkstra_node>(new_x, new_y, current);
                    neighbor->setGCost(new_cost);
                    open_list.push(neighbor);
                }
            }
        }
    }
}

// Visualize function remains the same as provided in Astar.cpp
void visualize_dijikstra(const cv::Mat &img) {
    cv::imshow("Pathfinding Visualization", img);
    cv::waitKey(0);
}
