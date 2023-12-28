#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

// Node Class
class Node{
  public :
    int x,y;
    float g_cost,h_cost,f_cost;

    std::shared_ptr<Node> parent;

    Node(int x,int y , std::shared_ptr<Node> parent=nullptr) : x(x),y(y),g_cost(0),h_cost(0),f_cost(0),parent(parent){}

    void set_gcost(float cost){
      g_cost=cost;
      f_cost = g_cost + h_cost;

    }

    void set_hcost(float cost){
      h_cost = cost;
      f_cost = g_cost+h_cost;
    }
};

struct compare_node{
  bool operator()(const std::shared_ptr<Node>& a,std::shared_ptr<Node>& b){
    return a->f_cost > b->f_cost;
  }

};

float heuristic(int x1,int y1 , int x2,int y2 ){
  return std::sqrt(std::pow(x1-x2,2) + std::pow(y1-y2,2));
}


void a_start_algo(cv::Mat &map , Node &start_point , Node &goal_point){


  std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, compare_node> open_list;
  std::vector<std::vector<float>> cost_so_far(map.rows, std::vector<float>(map.cols, std::numeric_limits<float>::max()));
  std::vector<std::vector<bool>> closed_list(map.rows, std::vector<bool>(map.cols, false));

  auto start_node = std::make_shared<Node>(start_point.x, start_point.y);
  start_node->set_gcost(0);
  start_node->set_hcost(heuristic(start_point.x, start_point.y, goal_point.x, goal_point.y));
  open_list.push(start_node);
  cost_so_far[start_point.x][start_point.y] = 0;

  std::vector<std::pair<int, int>> directions = {
    {0, 1}, {1, 0}, {0, -1}, {-1, 0},
    {1, 1}, {-1, 1}, {1, -1}, {-1, -1}
  };

  while (!open_list.empty()) {
    auto current_node = open_list.top();
    open_list.pop();

    if (current_node->x == goal_point.x && current_node->y == goal_point.y) {
      // Path reconstruction
      auto path_node = current_node;
      while (path_node->parent != nullptr) {
        cv::line(map, cv::Point(path_node->x, path_node->y), cv::Point(path_node->parent->x, path_node->parent->y), cv::Scalar(0, 255, 0), 2);
        path_node = path_node->parent;
      }
      return;
    }

    closed_list[current_node->x][current_node->y] = true;

    // Neighbor exploration
    for (auto dir : directions) {
      int new_x = current_node->x + dir.first;
      int new_y = current_node->y + dir.second;

      float new_cost = current_node->g_cost + ((dir.first == 0 || dir.second == 0) ? 1.0 : std::sqrt(2));

      if (new_x >= 0 && new_x < map.cols && new_y >= 0 && new_y < map.rows && !closed_list[new_x][new_y]) {
        auto neighbor = std::make_shared<Node>(new_x, new_y, current_node);

        if (new_cost < cost_so_far[new_x][new_y]) {
          neighbor->set_gcost(new_cost);
          neighbor->set_hcost(heuristic(neighbor->x, neighbor->y, goal_point.x, goal_point.y));
          open_list.push(neighbor);
          cost_so_far[new_x][new_y] = new_cost;
        }
      }
    }
  }


}



void Visualization(const cv::Mat &img){
  cv::imshow("Map",img);
  cv::waitKey(0);
}

int main() {

  int map_width = 800;
  int map_height = 800;
  int grid_size=10;
  cv::Scalar grid_color(0,0,0);
  cv::Mat map(map_height,map_width, CV_8UC3 , cv::Scalar(255,255,255));

  // Define Node
  Node starting_point(700,750);
  Node goal_point(100,250);
  cv::circle(map , cv::Point(starting_point.x,starting_point.y),5,cv::Scalar(255,0,0),-1);
  cv::circle(map , cv::Point(goal_point.x,goal_point.y),5,cv::Scalar(0,255,0),-1);

  // Grid line drawing
  for (int x = 0; x <= map_width; x += grid_size) {
    cv::line(map, cv::Point(x, 0), cv::Point(x, map_height), grid_color);
  }

   for (int y = 0; y <= map_height; y += grid_size) {
    cv::line(map, cv::Point(0, y), cv::Point(map_width, y), grid_color);
  }


  a_start_algo(map,starting_point , goal_point);


  Visualization(map);

  return 0;
}

