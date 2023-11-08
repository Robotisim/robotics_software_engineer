#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

// Define the Node class
class Node {
public:
  int x, y; // Node position - more efficient than a struct.
  float cost; // Total cost so far for the node.
  Node *parent; // Pointer to parent node for path generation.

  Node(int x, int y, Node *p = nullptr) : x(x), y(y), cost(0), parent(p) {}
};

// Function declarations
void visualize(const cv::Mat &img);



//***************** B addition
Node convert_to_grid(float x, float y, float resolution) {
  return Node(static_cast<int>(x / resolution), static_cast<int>(y / resolution));
}
//*****************
int main() {
  // Define the map size and create a blank image
  int map_width = 300;
  int map_height = 300;
  cv::Mat map(map_height, map_width, CV_8UC3, cv::Scalar(255, 255, 255));

  // Visualize the starting point
  Node start(10, 10); // Starting at (10,10)
  cv::circle(map, cv::Point(start.x, start.y), 2, cv::Scalar(255, 0, 0), -1);

  //***************** B addition
  float resolution = 1.0; // Grid resolution is 1 unit
  Node start_point = convert_to_grid(10.0, 10.0, resolution); // Start in grid coordinates
  Node goal_point = convert_to_grid(90.0, 90.0, resolution); // Goal in grid coordinates
  cv::circle(map, cv::Point(goal_point.x, goal_point.y), 2, cv::Scalar(0, 0, 255), -1);
//*****************
  visualize(map);

  return 0;
}

// Function definitions
void visualize(const cv::Mat &img) {
  cv::imshow("A* Pathfinding", img);
  cv::waitKey(0); // Wait indefinitely for a keypress
}
