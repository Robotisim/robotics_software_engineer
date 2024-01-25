#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

class Astar_Node {
public:
  int x, y;
  float g_cost, h_cost, f_cost;
  std::shared_ptr<Astar_Node> parent;

  Astar_Node(int x, int y, std::shared_ptr<Astar_Node> parent = nullptr)
      : x(x), y(y), g_cost(0), h_cost(0), f_cost(0), parent(parent) {}

  void set_gcost(float cost) {
    g_cost = cost;
    f_cost = g_cost + h_cost;
  }

  void set_hcost(float cost) {
    h_cost = cost;
    f_cost = g_cost + h_cost;
  }
};
struct compare_node {
  bool operator()(const std::shared_ptr<Astar_Node> &a,
                  const std::shared_ptr<Astar_Node> &b) {
    return a->f_cost > b->f_cost;
  }
};

float heuristic(int x1, int y1, int x2, int y2) {
  return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}

class AstarNode : public rclcpp::Node {
public:
  AstarNode() : Node("astar_node") {
    RCLCPP_INFO(this->get_logger(), "Performing Astar Search");

    occupancy_grid_subscriber_ =
        this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "occupancy_grid", 10,
            std::bind(&AstarNode::occupancyGridCallback, this,
                      std::placeholders::_1));

    // Publishers for the path
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);
  }

private:
  std::pair<int, int> indexToCoordinates(int index, int width) {
    if (index < 0 || index >= width * width) {
      RCLCPP_ERROR(this->get_logger(), "Index out of bounds");
      return {-1, -1}; // Return an invalid coordinate
    }
    int y = index / width;
    int x = index % width;
    return {x, y};
  }
  void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid &grid) {
    int grid_width = grid.info.width;
    int grid_height = grid.info.height;
    const auto &grid_data = grid.data;

    // Count free and occupied cells
    int free_cells = 0, occupied_cells = 0;
    for (auto cell : grid_data) {
      if (cell == 0)
        free_cells++;
      else if (cell == 100)
        occupied_cells++;
    }
    RCLCPP_DEBUG(this->get_logger(), "Free cells: %d, Occupied cells: %d",
                 free_cells, occupied_cells);

    // Convert indices to coordinates and set start and goal points
    auto start_coords = indexToCoordinates(0, grid_width);
    auto goal_coords = indexToCoordinates(84, grid_width);

    RCLCPP_DEBUG(this->get_logger(), "Raw start coords: (%d, %d), Raw goal coords: (% d, % d) ",
                   start_coords.first,start_coords.second, goal_coords.first, goal_coords.second);

    // Use class member variables directly
    this->start_point.x = static_cast<float>(start_coords.first);
    this->start_point.y = static_cast<float>(start_coords.second);
    this->goal_point.x = static_cast<float>(goal_coords.first);
    this->goal_point.y = static_cast<float>(goal_coords.second);
    // Print start and goal points
    RCLCPP_DEBUG(this->get_logger(),
                 "Starting point: (%f, %f) - Goal point: (%f, %f)",
                 this->start_point.x, this->start_point.y, this->goal_point.x,
                 this->goal_point.y);
    RCLCPP_DEBUG(this->get_logger(), "Map Width: %d, Map Height: %d",
                 grid_width, grid_height);



    // Astar Implementation Started
    std::priority_queue<std::shared_ptr<Astar_Node>,
                        std::vector<std::shared_ptr<Astar_Node>>, compare_node>
        open_list;

    std::vector<std::vector<float>> cost_so_far(
        grid_height,
        std::vector<float>(grid_width, std::numeric_limits<float>::max()));
    std::vector<std::vector<bool>> closed_list(
        grid_height, std::vector<bool>(grid_width, false));

    auto start_node = std::make_shared<Astar_Node>(
        static_cast<int>(start_point.x), static_cast<int>(start_point.y));
    start_node->set_gcost(0);
    start_node->set_hcost(heuristic(start_node->x, start_node->y,
                                    static_cast<int>(goal_point.x),
                                    static_cast<int>(goal_point.y)));

    open_list.push(start_node);
    cost_so_far[start_point.x][start_point.y] = 0;

    std::vector<std::pair<int, int>> directions = {
        {0, 1}, {1, 0}, {0, -1}, {-1, 0}, {1, 1}, {-1, 1}, {1, -1}, {-1, -1}};

    while (!open_list.empty()) {
      auto current_node = open_list.top();
      open_list.pop();

      RCLCPP_DEBUG(
          this->get_logger(), "Checking node at (%d, %d) against goal (%d, %d)",
          current_node->x, current_node->y, goal_point.x, goal_point.y);
      // Check if goal is reached

      if (current_node->x == goal_point.x && current_node->y == goal_point.y) {
        std::vector<geometry_msgs::msg::PoseStamped> path_reversed;

        auto path_node = current_node;
        int node_count = 0; // Initialize a counter for the number of nodes

        while (path_node != nullptr) {
          node_count++;
          geometry_msgs::msg::PoseStamped pose;
          pose.header.frame_id = grid.header.frame_id;
          pose.header.stamp = grid.header.stamp;
          pose.pose.position.x = path_node->x - 5.0;
          pose.pose.position.y = path_node->y - 5.0;
          pose.pose.position.z = 0;      // Assuming a 2D plane
          pose.pose.orientation.w = 1.0; // No rotation

          path_reversed.push_back(pose);
          path_node = path_node->parent;
        }
        RCLCPP_INFO(this->get_logger(),
                    "Found goal path. Total number of nodes in the path: %d",
                    node_count);

        // Reverse the path to get it from start to goal
        std::vector<geometry_msgs::msg::PoseStamped> path;
        for (auto it = path_reversed.rbegin(); it != path_reversed.rend();
             ++it) {
          path.push_back(*it);
        }
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id =
            grid.header.frame_id; // Same frame as the occupancy grid
        path_msg.header.stamp = grid.header.stamp;
        path_msg.poses = path;

        path_publisher_->publish(path_msg);

        break;
      }

      closed_list[current_node->x][current_node->y] = true;

      // Explore neighbors
      for (auto dir : directions) {
        int new_x = current_node->x + dir.first;
        int new_y = current_node->y + dir.second;

        // Check if new node is within the grid and not an obstacle
        if (new_x >= 0 && new_x < grid_width && new_y >= 0 &&
            new_y < grid_height) {
          int new_index = new_y * grid_width + new_x;
          if (grid_data[new_index] == 100) { // Check if the cell is an obstacle
            continue;
          }

          float new_cost =
              current_node->g_cost +
              ((dir.first == 0 || dir.second == 0) ? 1.0 : std::sqrt(2));
          float estimated_total_cost =
              new_cost + heuristic(new_x, new_y, goal_point.x, goal_point.y);
          RCLCPP_DEBUG(this->get_logger(),
                       "Considering neighbor at (%d, %d) with g_cost = %f, h_cost = %f, total travel cost = %f",
                       new_x, new_y, new_cost,
                       heuristic(new_x, new_y, goal_point.x, goal_point.y),
                       estimated_total_cost);

          if (new_x >= 0 && new_x < grid_width && new_y >= 0 &&
              new_y < grid_height && !closed_list[new_x][new_y]) {
            auto neighbor =
                std::make_shared<Astar_Node>(new_x, new_y, current_node);

            if (new_cost < cost_so_far[new_x][new_y]) {
              neighbor->set_gcost(new_cost);
              neighbor->set_hcost(heuristic(neighbor->x, neighbor->y,
                                            goal_point.x, goal_point.y));
              open_list.push(neighbor);
              cost_so_far[new_x][new_y] = new_cost;

              RCLCPP_DEBUG(this->get_logger(),
                           "---> Moving to neighbor at (%d, %d) with new g_cost: %f",
                           new_x, new_y, new_cost);
            }
          }
        }
      }
    }
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
      occupancy_grid_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  geometry_msgs::msg::Point start_point;
  geometry_msgs::msg::Point goal_point;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AstarNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
