#include<iostream>
#include<vector>
#include "path_planning/rrt.h"
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>

using namespace std;

int main() {
  vector<vector<float>> obstacles{
    {5, 5, 1},
    {3, 6, 2},
    {3, 8, 2},
    {3, 10, 2},
    {7, 5, 2},
    {9, 5, 2}
  };

  Node* start_point = new Node(0.0, 0.0);
  Node* goal_point = new Node(15.0, 3.0);
  vector<float> sampling_area{-2, 15};

  RRT rrt_planner(start_point, goal_point, obstacles, sampling_area, 0.5, 1.0, 5, 500);
  vector<Node*> path = rrt_planner.planPath();

  // Visualization parameters
  const cv::Scalar obstacleColor(0, 0, 0); // Color for the obstacles: Black
  const cv::Scalar startColor(0, 255, 0); // Color for the start point: Green
  const cv::Scalar goalColor(0, 0, 255); // Color for the goal point: Red
  const cv::Scalar pathColor(255, 0, 0); // Color for the path: Blue

  // Set the window dimensions to match the desired display size
  int window_width = 800;  // width of the window
  int window_height = 600; // height of the window

  // Create a map with the window dimensions for direct display without resizing
  cv::Mat map(window_height, window_width, CV_8UC3, cv::Scalar(255, 255, 255));

  // Determine the scaling factors for width and height separately
  float scale_width = static_cast<float>(window_width) / (sampling_area[1] - sampling_area[0]);
  float scale_height = static_cast<float>(window_height) / (sampling_area[1] - sampling_area[0]);

  // Use the smaller of the two scaling factors to ensure everything fits in the window
  float scale = std::min(scale_width, scale_height);

  // Adjust the offset based on the new scale
  int offset = static_cast<int>(scale);  // This can be adjusted as needed

  cv::namedWindow("RRT Path Planning", cv::WINDOW_NORMAL); // Create a window that can be resized

  // Draw obstacles
  for (const auto& obstacle : obstacles) {
    cv::Point center(static_cast<int>((obstacle[0] - sampling_area[0]) * scale + offset / 2),
                     static_cast<int>((obstacle[1] - sampling_area[0]) * scale + offset / 2));
    cv::circle(map, center, static_cast<int>(obstacle[2] * scale), obstacleColor, -1);
  }

  // Draw start and goal points
  const int radius = 3; // Radius of the points, adjusted for scale
  cv::circle(map, cv::Point(static_cast<int>((start_point->x - sampling_area[0]) * scale + offset / 2),
                            static_cast<int>((start_point->y - sampling_area[0]) * scale + offset / 2)),
             static_cast<int>(radius * scale), startColor, -1);

  // Draw path
  for (size_t i = 1; i < path.size(); i++) {
    Node* from_node = path[i - 1];
    Node* to_node = path[i];
    cv::line(map, cv::Point(static_cast<int>((from_node->x - sampling_area[0]) * scale + offset / 2),
                            static_cast<int>((from_node->y - sampling_area[0]) * scale + offset / 2)),
             cv::Point(static_cast<int>((to_node->x - sampling_area[0]) * scale + offset / 2),
                       static_cast<int>((to_node->y - sampling_area[0]) * scale + offset / 2)),
             pathColor, static_cast<int>(scale), cv::LINE_AA);
  }

  // Display the result
  cv::imshow("RRT Path Planning", map);
  cv::waitKey(0);

  // Cleanup
  for (Node* node : path) {
    delete node;
  }

  return 0;
}
