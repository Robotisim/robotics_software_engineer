// main.cpp
#include "path_planning/Astar_node.h"
#include "path_planning/Dijkstra_node.h"

int main() {
  int map_width = 700, map_height = 700;
  cv::Mat map(map_height, map_width, CV_8UC3, cv::Scalar(255, 255, 255));

  // Astar_node astar_start(600, 100), astar_goal(100,650);
  // cv::circle(map, cv::Point(astar_start.x, astar_start.y), 10, cv::Scalar(0, 0, 255), -1); // Start point
  // cv::circle(map, cv::Point(astar_goal.x, astar_goal.y), 10, cv::Scalar(0, 0, 0), -1); // Goal point

  // // Run A* algorithm
  // a_star_algorithm(map, astar_start, astar_goal);
  // visualize_astar(map);

  // Clear the map for Dijkstra visualization
  Dijkstra_node dijkstra_start(0, 0), dijkstra_goal(640,650);

  map = cv::Scalar(255, 255, 255);
  cv::circle(map, cv::Point(dijkstra_start.x, dijkstra_start.y), 10, cv::Scalar(0, 0, 255), -1); // Start point
  cv::circle(map, cv::Point(dijkstra_goal.x, dijkstra_goal.y), 10, cv::Scalar(0, 0, 0), -1); // Goal point

  // Run Dijkstra's algorithm
  dijkstra_algorithm(map, dijkstra_start, dijkstra_goal);
  visualize_dijikstra(map);

  return 0;
}
