#include "nav2_planner_interface.hpp"

#include <memory>
#include <string>
#include <vector>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_astar_planner {

	AstarPlanner::AstarPlanner() : tf_(nullptr), costmap_(nullptr) {
	}

	AstarPlanner::~AstarPlanner() {
		RCLCPP_INFO(logger_, "Destroying plugin %s of type AstarPlanner", name_.c_str());
	}

	void AstarPlanner::configure(rclcpp_lifecycle::LifecycleNode::WeakPtr const& parent,
	                             std::string name,
	                             std::shared_ptr<tf2_ros::Buffer> tf,
	                             std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
		tf_ = tf;
		name_ = name;
		costmap_ = costmap_ros->getCostmap();
		global_frame_ = costmap_ros->getGlobalFrameID();

		node_ = parent;
		auto node = parent.lock();
		clock_ = node->get_clock();
		logger_ = node->get_logger();

		RCLCPP_INFO(logger_, "Configuring plugin %s of type AstarPlanner", name_.c_str());
	}

	void AstarPlanner::activate() {
		RCLCPP_INFO(logger_, "Activating plugin %s of type AstarPlanner", name_.c_str());
		auto node = node_.lock();
	}

	void AstarPlanner::deactivate() {
		RCLCPP_INFO(logger_, "Deactivating plugin %s of type AstarPlanner", name_.c_str());
		auto node = node_.lock();
	}

	void AstarPlanner::cleanup() {
		RCLCPP_INFO(logger_, "Cleaning up plugin %s of type AstarPlanner", name_.c_str());
	}

	nav_msgs::msg::OccupancyGrid AstarPlanner::costmapToOccupancyGrid() {
		nav_msgs::msg::OccupancyGrid grid;
		grid.info.width = costmap_->getSizeInCellsX();
		grid.info.height = costmap_->getSizeInCellsY();
		grid.info.resolution = costmap_->getResolution();
		grid.info.origin.position.x = costmap_->getOriginX();
		grid.info.origin.position.y = costmap_->getOriginY();
		grid.data.resize(grid.info.width * grid.info.height);
		std::transform(costmap_->getCharMap(),
		               costmap_->getCharMap() + grid.data.size(),
		               grid.data.begin(),
		               [](unsigned char cost) -> int8_t {
			               if (cost == nav2_costmap_2d::NO_INFORMATION) {
				               return -1; // Unknown
			               } else if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE or
			                          cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
				               return 100; // Occupied
			               } else {
				               return 0; // Free
			               }
		               });
		return grid;
	}

	nav_msgs::msg::Path AstarPlanner::createPlan(geometry_msgs::msg::PoseStamped const& start,
	                                             geometry_msgs::msg::PoseStamped const& goal) {
		unsigned int mx_start, my_start, mx_goal, my_goal;
		if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx_start, my_start)) {
			RCLCPP_WARN(logger_,
			            "Cannot create a plan: the robot's start position is off the global"
			            " costmap. Planning will always fail, are you sure"
			            " the robot has been properly localized?");
			return nav_msgs::msg::Path();
		}

		if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal)) {
			RCLCPP_WARN(logger_,
			            "Cannot create a plan: the goal is off the global costmap."
			            " Planning will always fail to this goal.");
			return nav_msgs::msg::Path();
		}

		nav_msgs::msg::OccupancyGrid grid = costmapToOccupancyGrid();

		auto astar_path = astar_search(grid, mx_start, my_start, mx_goal, my_goal);
		if (!astar_path.empty()) {
			nav_msgs::msg::Path path;
			path.header.stamp = clock_->now();
			path.header.frame_id = global_frame_;
			path.poses = astar_path;
			return path;
		} else {
			RCLCPP_WARN(logger_, "Could not create a plan");
			return nav_msgs::msg::Path();
		}
	}

} // namespace nav2_astar_planner

PLUGINLIB_EXPORT_CLASS(nav2_astar_planner::AstarPlanner, nav2_core::GlobalPlanner)