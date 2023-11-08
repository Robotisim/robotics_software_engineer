/**
 * Purpose: Implement A* Pathfinding Algorithm using OpenCV to visualize the path.
 * Flow:
 *    - Define a map and nodes
 *    - Run A* algorithm to find the shortest path
 *    - Visualize the path using OpenCV
 *
 * Functions:
 *    - heuristic: Calculate heuristic based on Euclidean distance
 *    - a_star_algorithm: Main function to execute the A* pathfinding
 *    - visualize: Display the map and path using OpenCV window
 *
 * Author: Luqman
 * Date: 2023-11-04
 */

#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <opencv2/opencv.hpp>

class Node {
public:
  int x, y;
  float g_cost, h_cost, f_cost;
  std::shared_ptr<Node> parent;

  Node(int x, int y, std::shared_ptr<Node> parent = nullptr)
  : x(x), y(y), g_cost(0), h_cost(0), f_cost(0), parent(parent) {}

  void setGCost(float cost) {
    g_cost = cost;
    f_cost = g_cost + h_cost;
  }

  void setHCost(float cost) {
    h_cost = cost;
    f_cost = g_cost + h_cost;
  }
};

bool enable_real_time_visualization = false;
bool add_obstacles = true;

struct CompareNode {
  bool operator()(const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) {
    return a->f_cost > b->f_cost;
  }
};

// Calculate the heuristic value using Euclidean distance
float heuristic(int x1, int y1, int x2, int y2) {
    return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}

bool isObstacle(const cv::Mat& map, int x, int y) {
    cv::Vec3b color = map.at<cv::Vec3b>(y, x);
    bool obstacle = (color[0] == 0 && color[1] == 0 && color[2] == 0);
    if (obstacle) {
        std::cout << "Obstacle detected at (" << x << ", " << y << ")" << std::endl;
    }
    return obstacle;
}





// A* Algorithm implementation to find the shortest path from start to goal
void a_star_algorithm(cv::Mat &map, Node &start, Node &goal) {
  std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, CompareNode> open_list;
  std::vector<std::vector<float>> cost_so_far(map.rows, std::vector<float>(map.cols, std::numeric_limits<float>::max()));
  std::vector<std::vector<bool>> closed_list(map.rows, std::vector<bool>(map.cols, false));

  auto start_node = std::make_shared<Node>(start.x, start.y);
  start_node->setGCost(0);
  start_node->setHCost(heuristic(start.x, start.y, goal.x, goal.y));
  open_list.push(start_node);
  cost_so_far[start.x][start.y] = 0;

  std::vector<std::pair<int, int>> directions = {
    {0, 1}, {1, 0}, {0, -1}, {-1, 0},
    {1, 1}, {-1, 1}, {1, -1}, {-1, -1}
  };



//*** To Visualize in runtime for path funding
if (enable_real_time_visualization) {
  cv::namedWindow("A* Pathfinding Live", cv::WINDOW_NORMAL);
}

//*** To Visualize in runtime for path funding

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

    //*** To Visualize in runtime for path funding
    if (enable_real_time_visualization) {
    // Visualize the current node
      cv::circle(map, cv::Point(current->x, current->y), 2, cv::Scalar(255, 0, 0), -1);

      // Update the visualization
      cv::imshow("A* Pathfinding Live", map);
      cv::waitKey(1); // Small delay to visualize the process
    }
    //********* To Visualize in runtime for path funding



    // Neighbor exploration
  for (auto dir : directions) {
      int new_x = current->x + dir.first;
      int new_y = current->y + dir.second;

      float new_cost = current->g_cost + ((dir.first == 0 || dir.second == 0) ? 1.0 : std::sqrt(2));

      // Check if the new position is valid, not an obstacle, and not already in the closed list
    if (new_x >= 0 && new_x < map.rows && new_y >= 0 && new_y < map.cols && !closed_list[new_x][new_y] && !isObstacle(map, new_x, new_y)) {
          auto neighbor = std::make_shared<Node>(new_x, new_y, current);

        //*** To Visualize in runtime for path funding
        cv::circle(map, cv::Point(neighbor->x, neighbor->y), 2, cv::Scalar(0, 255, 0), -1);
        //*** To Visualize in runtime for path funding


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

// Display the map and the found path using OpenCV
void visualize(const cv::Mat &img) {
  cv::imshow("A* Pathfinding", img);
  cv::waitKey(0);
}

int main() {
  int map_width = 700, map_height = 700;
  cv::Mat map(map_height, map_width, CV_8UC3, cv::Scalar(255, 255, 255));

  Node start(100, 100), goal(400,650);

  cv::circle(map, cv::Point(start.x, start.y), 10, cv::Scalar(0, 0, 255), -1); // Red circle for start
  cv::circle(map, cv::Point(goal.x, goal.y), 10, cv::Scalar(0, 0, 0), -1);

    if (add_obstacles) {
    cv::rectangle(map, cv::Point(250, 250), cv::Point(450, 450), cv::Scalar(0, 0, 0), -1);
  }

  cv::imshow("Map with Obstacles", map);
  cv::waitKey(0); // Wait for a key press to continue


  a_star_algorithm(map, start, goal);

  if (!enable_real_time_visualization) {
    visualize(map);
  }

  return 0;
}
