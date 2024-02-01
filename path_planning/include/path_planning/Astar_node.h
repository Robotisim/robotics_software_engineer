// Node.h
#ifndef NODE_H
#define NODE_H

#include <memory>
#include <vector>
#include <queue>
#include <opencv2/opencv.hpp>

class Astar_node {
public:
  int x, y;
  float g_cost, h_cost, f_cost;
  std::shared_ptr<Astar_node> parent;

  Astar_node(int x, int y, std::shared_ptr<Astar_node> parent = nullptr);
  void setGCost(float cost);
  void setHCost(float cost);
};

// Comparator for nodes in the priority queue
struct CompareNode {
  bool operator()(const std::shared_ptr<Astar_node>& a, const std::shared_ptr<Astar_node>& b);
};

// A* Algorithm function declarations
void a_star_algorithm(cv::Mat &map, Astar_node &start, Astar_node &goal);
float heuristic(int x1, int y1, int x2, int y2);
void visualize_astar(const cv::Mat &img);

#endif // NODE_H
