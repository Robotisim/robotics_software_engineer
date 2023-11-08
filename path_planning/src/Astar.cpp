// AStar.cpp
#include "path_planning/Astar_node.h"
#include <limits>
#include <cmath>
#include <opencv2/opencv.hpp>

// Node class constructors and methods
Astar_node::Astar_node(int x, int y, std::shared_ptr<Astar_node> parent)
: x(x), y(y), g_cost(0), h_cost(0), f_cost(0), parent(parent) {}

void Astar_node::setGCost(float cost) {
  g_cost = cost;
  f_cost = g_cost + h_cost;
}

void Astar_node::setHCost(float cost) {
  h_cost = cost;
  f_cost = g_cost + h_cost;
}

// Comparator for nodes
bool CompareNode::operator()(const std::shared_ptr<Astar_node>& a, const std::shared_ptr<Astar_node>& b) {
  return a->f_cost > b->f_cost;
}

// Heuristic function
float heuristic(int x1, int y1, int x2, int y2) {
  return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}

// A* Algorithm implementation
void a_star_algorithm(cv::Mat &map, Astar_node &start, Astar_node &goal) {
  std::priority_queue<std::shared_ptr<Astar_node>, std::vector<std::shared_ptr<Astar_node>>, CompareNode> open_list;
  std::vector<std::vector<float>> cost_so_far(map.rows, std::vector<float>(map.cols, std::numeric_limits<float>::max()));
  std::vector<std::vector<bool>> closed_list(map.rows, std::vector<bool>(map.cols, false));

  auto start_node = std::make_shared<Astar_node>(start.x, start.y);
  start_node->setGCost(0);
  start_node->setHCost(heuristic(start.x, start.y, goal.x, goal.y));
  open_list.push(start_node);
  cost_so_far[start.x][start.y] = 0;

  std::vector<std::pair<int, int>> directions = {
    {0, 1}, {1, 0}, {0, -1}, {-1, 0},
    {1, 1}, {-1, 1}, {1, -1}, {-1, -1}
  };

  while (!open_list.empty()) {
    auto current = open_list.top();
    open_list.pop();

    if (current->x == goal.x && current->y == goal.y) {
      // Path reconstruction
      auto path_node = current;
      while (path_node->parent != nullptr) {
        cv::line(map, cv::Point(path_node->x, path_node->y), cv::Point(path_node->parent->x, path_node->parent->y), cv::Scalar(0, 255, 0), 2);
        path_node = path_node->parent;
      }
      return;
    }

    closed_list[current->x][current->y] = true;

    // Neighbor exploration
    for (auto dir : directions) {
      int new_x = current->x + dir.first;
      int new_y = current->y + dir.second;

      float new_cost = current->g_cost + ((dir.first == 0 || dir.second == 0) ? 1.0 : std::sqrt(2));

      if (new_x >= 0 && new_x < map.cols && new_y >= 0 && new_y < map.rows && !closed_list[new_x][new_y]) {
        auto neighbor = std::make_shared<Astar_node>(new_x, new_y, current);

        if (new_cost < cost_so_far[new_x][new_y]) {
          neighbor->setGCost(new_cost);
          neighbor->setHCost(heuristic(neighbor->x, neighbor->y, goal.x, goal.y));
          open_list.push(neighbor);
          cost_so_far[new_x][new_y] = new_cost;
        }
      }
    }
  }
}


// Visualize function
void visualize_astar(const cv::Mat &img) {
  cv::imshow("A* Pathfinding", img);
  cv::waitKey(0);
}
