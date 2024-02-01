#ifndef DIJKSTRA_NODE_H
#define DIJKSTRA_NODE_H

#include <memory>
#include <vector>
#include <queue>
#include <opencv2/opencv.hpp>

class Dijkstra_node {
public:
  int x, y;
  float g_cost;
  std::shared_ptr<Dijkstra_node> parent;

  Dijkstra_node(int x, int y, std::shared_ptr<Dijkstra_node> parent = nullptr);
  void setGCost(float cost);
};

// Comparator for nodes in the priority queue
struct CompareNodeDijkstra {
  bool operator()(const std::shared_ptr<Dijkstra_node>& a, const std::shared_ptr<Dijkstra_node>& b);
};

// Dijkstra Algorithm function declarations
void dijkstra_algorithm(cv::Mat &map, Dijkstra_node &start, Dijkstra_node &goal);
void visualize_dijikstra(const cv::Mat &img);

#endif // DIJKSTRA_NODE_H
