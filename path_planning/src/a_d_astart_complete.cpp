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

//***************** C addition
void a_star_algorithm(cv::Mat &map, Node &start, Node &goal) {
  // Initialize open and closed lists
  std::vector<Node*> open_list;
  std::vector<Node*> closed_list;

  // Add the start node to the open list
  open_list.push_back(&start);

  // Main loop of the A* algorithm
  while (!open_list.empty()) {
    // Placeholder for algorithm steps
    std::cout << "Running A* loop..." << std::endl;

//***************** D addition
    Node *current = open_list.front();
    open_list.erase(open_list.begin()); // Remove current from open list
    closed_list.push_back(current);
  // For each of the node's neighbours...
    // (Here you would have logic for navigating the neighbours, calculating costs, and adding them to the open list if appropriate)

    // Check if we have reached the goal

    if (current->x == goal.x && current->y == goal.y) {
      // Reconstruct path and visualize
      while (current != nullptr) {
        cv::circle(map, cv::Point(current->x, current->y), 2, cv::Scalar(0, 255, 0), -1);
        current = current->parent;
      }
      break;
    }
  }//*****************

  // Visualization (to be implemented)
}//*****************


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
  //***************** C addition
  a_star_algorithm(map, start_point, goal_point);
  //*****************

  visualize(map);

  return 0;
}

// Function definitions
void visualize(const cv::Mat &img) {
  cv::imshow("A* Pathfinding", img);
  cv::waitKey(0); // Wait indefinitely for a keypress
}
