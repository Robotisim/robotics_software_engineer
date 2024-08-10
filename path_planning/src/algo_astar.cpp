#include "algo_astar.h"


NodeAstar::NodeAstar(int x, int y, std::shared_ptr<NodeAstar> parent)
    : x(x), y(y), g_cost(0), h_cost(0), f_cost(0), parent(parent) {}

void NodeAstar::set_gcost(float cost) {
    g_cost = cost;
    f_cost = g_cost + h_cost;
}

void NodeAstar::set_hcost(float cost) {
    h_cost = cost;
    f_cost = g_cost + h_cost;
}

bool compare_node::operator()(const std::shared_ptr<NodeAstar>& a, const std::shared_ptr<NodeAstar>& b) const {
    return a->f_cost > b->f_cost;
}

float heuristic(int x1, int y1, int x2, int y2) {
    return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}




std::pair<int, int> indexToCoordinates(int index, int width) {
  if (index < 0 || index >= width * width) {
    std::cout << "Index out of bounds" << std::endl;
    return {-1, -1};
  }

  int y = index / width;
  int x = index % width;

  return {x, y};
}

std::vector<geometry_msgs::msg::PoseStamped>
astar_search(const nav_msgs::msg::OccupancyGrid &grid) {

  std::cout << "Started Astar Algorithm" << std::endl;
  std::vector<geometry_msgs::msg::PoseStamped> path;
  int GRID_HEGHT = grid.info.height;
  int GRID_WIDTH = grid.info.width;
  const auto &grid_data = grid.data;

  geometry_msgs::msg::Point start_point;
  geometry_msgs::msg::Point goal_point;

  int free_cells = 0, occupied_cells = 0;

  for (auto cell : grid_data) {
    if (cell == 0) {
      free_cells++;
    } else if (cell == 100) {
      occupied_cells++;
    }
  }

  std::cout << "Free cells: " << free_cells
            << ", Occupied cells: " << occupied_cells << std::endl;

  // convert indexes into xy
  auto start_coordinates = indexToCoordinates(0, GRID_WIDTH);
  auto goal_coordinates = indexToCoordinates(68, GRID_WIDTH);

  start_point.x = static_cast<float>(start_coordinates.first);
  start_point.y = static_cast<float>(start_coordinates.second);

  goal_point.x = static_cast<float>(goal_coordinates.first);
  goal_point.y = static_cast<float>(goal_coordinates.second);

  // Print start and goal points
  std::cout << "Starting point: (" << start_point.x << ", " << start_point.y
            << ") - Goal point: (" << goal_point.x << ", " << goal_point.y
            << ")" << std::endl;
  std::cout << "Map Width: " << GRID_WIDTH << ", Map Height: " << GRID_HEGHT
            << std::endl;


  std::priority_queue<std::shared_ptr<NodeAstar>,std::vector<std::shared_ptr<NodeAstar>>, compare_node>open_list;
  std::vector<std::vector<float>> cost_so_far(GRID_HEGHT,std::vector<float>(GRID_WIDTH, std::numeric_limits<float>::max()));
  std::vector<std::vector<bool>> closed_list(GRID_HEGHT, std::vector<bool>(GRID_WIDTH, false));
  auto start_node = std::make_shared<NodeAstar>(static_cast<int>(start_point.x), static_cast<int>(start_point.y));

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

    std::cout << "Checking node at (" << current_node->x << ", "
              << current_node->y << ") against goal (" << goal_point.x << ", "
              << goal_point.y << ")" << std::endl;

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

      std::cout << "Found goal path. Total number of nodes in the path: "
                << node_count << std::endl;

      // Reverse the path to get it from start to goal
      std::vector<geometry_msgs::msg::PoseStamped> path;
      for (auto it = path_reversed.rbegin(); it != path_reversed.rend(); ++it) {
        path.push_back(*it);
      }
      return path;

      break;
    }

    closed_list[current_node->x][current_node->y] = true;


    // Explore neighbors
    for (auto dir : directions) {
      int new_x = current_node->x + dir.first;
      int new_y = current_node->y + dir.second;
      std::cout << "Considering neighbor at (" << new_x << ", " << new_y << ")"
                << std::endl;

      // Check if new node is within the grid and not an obstacle
      if (new_x >= 0 && new_x < GRID_WIDTH && new_y >= 0 &&
          new_y < GRID_HEGHT) {
        int new_index = new_y * GRID_WIDTH + new_x;
        if (grid_data[new_index] == 100) { // Check if the cell is an obstacle
          continue;
        }

        float new_cost =
            current_node->g_cost +
            ((dir.first == 0 || dir.second == 0) ? 1.0 : std::sqrt(2));

        if (!closed_list[new_x][new_y]) {
          auto neighbor =
              std::make_shared<NodeAstar>(new_x, new_y, current_node);

          if (new_cost < cost_so_far[new_x][new_y]) {
            neighbor->set_gcost(new_cost);
            neighbor->set_hcost(heuristic(neighbor->x, neighbor->y,
                                          goal_point.x, goal_point.y));
            open_list.push(neighbor);
            cost_so_far[new_x][new_y] = new_cost;
          }
        }
      }
    }

  }


}
