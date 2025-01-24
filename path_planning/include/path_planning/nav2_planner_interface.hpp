#ifndef ASTAR_PLANNER_HPP
#define ASTAR_PLANNER_HPP

#include <memory>
#include <string>
#include <vector>

#include "algo_astar_plugin.h"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_astar_planner {

	class AstarPlanner : public nav2_core::GlobalPlanner {
	public:
		AstarPlanner();
		~AstarPlanner();

        void configure(rclcpp_lifecycle::LifecycleNode::WeakPtr const& parent,
		               std::string name,
		               std::shared_ptr<tf2_ros::Buffer> tf,
		               std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

		void cleanup() override;
		void activate() override;
		void deactivate() override;

		nav_msgs::msg::Path createPlan(geometry_msgs::msg::PoseStamped const& start,
		                               geometry_msgs::msg::PoseStamped const& goal) override;


	protected:
		rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
		rclcpp::Clock::SharedPtr clock_;

	    rclcpp::Logger logger_{rclcpp::get_logger("AstarPlanner")};
		std::shared_ptr<tf2_ros::Buffer> tf_;

		nav2_costmap_2d::Costmap2D* costmap_;
		std::string global_frame_;
		std::string name_;

		nav_msgs::msg::OccupancyGrid costmapToOccupancyGrid();

	};

}

#endif