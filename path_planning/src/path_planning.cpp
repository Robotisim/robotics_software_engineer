#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include "algo_gridsweep.h"
#include "algo_astar.h"
#include "algo_rrt.hpp"

class PathPlanning : public rclcpp::Node {
public:
  PathPlanning() : Node("path_planning_node"),
                   use_gridsweep_(false),
                   use_astar_(true),
                   use_rrt_(false) {  // Set your algorithm flags here
    RCLCPP_INFO(this->get_logger(), "Performing Path Planning Search");

    occupancy_grid_subscriber_ =
        this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "occupancy_grid", 10,
            std::bind(&PathPlanning::occupancyGridCallback, this,
                      std::placeholders::_1));

    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);
  }

private:
  void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid &grid) {
    RCLCPP_INFO(this->get_logger(), "Received occupancy grid with dimensions: %d x %d", grid.info.width, grid.info.height);

    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = grid.header.frame_id;
    path_msg.header.stamp = grid.header.stamp;

    if (use_gridsweep_) {
      path_msg.poses = grid_sweep(grid);
      RCLCPP_INFO(this->get_logger(), "GridSweep planner executed");
    } else if (use_astar_) {
      path_msg.poses = astar_search(grid);
      RCLCPP_INFO(this->get_logger(), "A* planner executed");
    } else if (use_rrt_) {
      RRT_Planner planner;
      std::vector<int> domain(grid.data.begin(), grid.data.end());
      planner.setDomain(domain);

      NodeRrt start(0, 0);
      NodeRrt goal(4, 5);
      std::vector<NodeRrt> path = planner.planPath(start, goal);

      if (path.empty()) {
        RCLCPP_WARN(this->get_logger(), "No path found by RRT planner");
      } else {
        for (const auto &node : path) {
          geometry_msgs::msg::PoseStamped pose;
          pose.pose.position.x = node.getX() - 5;
          pose.pose.position.y = node.getY() - 5;
          pose.pose.position.z = 0.0;
          path_msg.poses.push_back(pose);
        }
      }
      RCLCPP_INFO(this->get_logger(), "RRT planner executed");
    }

    path_publisher_->publish(path_msg);
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

  bool use_gridsweep_;
  bool use_astar_;
  bool use_rrt_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathPlanning>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
