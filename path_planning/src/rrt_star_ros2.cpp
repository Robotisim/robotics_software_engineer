#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <random>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

class RRT_STAR_Node {
public:
	RRT_STAR_Node(int x, int y, std::shared_ptr<RRT_STAR_Node> parent = nullptr)
	    : x(x), y(y), cost(0.0f), parent(parent) {
	}

	void set_cost(float cost) {
		this->cost = cost;
	}

	void set_parent(std::shared_ptr<RRT_STAR_Node> parent) {
		this->parent = parent;
	}

	int get_x() {
		return x;
	}

	int get_y() {
		return y;
	}

	float get_cost() {
		return cost;
	}

	std::shared_ptr<RRT_STAR_Node> get_parent() {
		return parent;
	}

	bool operator==(RRT_STAR_Node const& node) {
		return (x == node.x && y == node.y);
	}

	bool operator!=(RRT_STAR_Node const& node) {
		return (x != node.x || y != node.y);
	}

	bool operator<(RRT_STAR_Node const& node) {
		return (cost < node.cost);
	}

	bool operator>(RRT_STAR_Node const& node) {
		return (cost > node.cost);
	}

private:
	int x, y;
	float cost;
	std::shared_ptr<RRT_STAR_Node> parent;
};

class RRT_STAR_planner : public rclcpp::Node {
public:
	RRT_STAR_planner() : Node("rrt_star_planner"), generator_(std::random_device{}()) {
		RCLCPP_INFO(this->get_logger(), "Performing RRT* Search");

		occupancy_grid_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
		    "occupancy_grid", 10, std::bind(&RRT_STAR_planner::occupancyGridCallback, this, std::placeholders::_1));

		// Publishers for the path
		path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);
	}

private:
	std::default_random_engine generator_;

	// Create a start and goal node
	std::shared_ptr<RRT_STAR_Node> start_node = std::make_shared<RRT_STAR_Node>(0, 0);
	std::shared_ptr<RRT_STAR_Node> goal_node = std::make_shared<RRT_STAR_Node>(9, 9);

	// Create a vector to store the nodes
	std::vector<std::shared_ptr<RRT_STAR_Node>> nodes;

	// Create a vector to store the path
	std::vector<std::shared_ptr<RRT_STAR_Node>> path;

	// Create a vector to store the nearest node
	std::shared_ptr<RRT_STAR_Node> nearest_node;

	// Create a vector to store the new node
	std::shared_ptr<RRT_STAR_Node> new_node;

	// Create a vector to store the best parent
	std::shared_ptr<RRT_STAR_Node> best_parent;

	// Create a vector to store the radius
	float radius = 5.0;

	// Create a vector to store the step size
	float step_size = 2.0;

	// Create a vector to store the max iterations
	int max_iterations = 1000;

	// Create a vector to store the min distance
	float min_distance = std::numeric_limits<float>::max();

	// Create a vector to store the min cost
	float min_cost = std::numeric_limits<float>::max();

	bool is_goal_reached = false;

	// Create a vector to store the min node
	std::shared_ptr<RRT_STAR_Node> min_node;

	std::pair<int, int> indexToCoordinates(int index, int width) {
		if (index < 0 || index >= width * width) {
			RCLCPP_ERROR(this->get_logger(), "Index out of bounds");
			return {-1, -1}; // Return an invalid coordinate
		}
		int y = index / width;
		int x = index % width;
		return {x, y};
	}

	std::pair<int, int> sampleRandomNode(nav_msgs::msg::OccupancyGrid const& grid) {
		std::uniform_int_distribution<int> distribution(0, grid.info.width * grid.info.height - 1);
		int index = distribution(generator_);
		RCLCPP_DEBUG(this->get_logger(), "Sampled index: %d", index);
		return indexToCoordinates(index, grid.info.width);
	}

	std::shared_ptr<RRT_STAR_Node> nearestNode(std::pair<int, int> const& sample) {
		float min_distance = std::numeric_limits<float>::max();
		std::shared_ptr<RRT_STAR_Node> nearest_node;

		for (auto const& node : nodes) {
			float distance =
			    std::sqrt(std::pow(node->get_x() - sample.first, 2) + std::pow(node->get_y() - sample.second, 2));
			if (distance < min_distance) {
				min_distance = distance;
				nearest_node = node;
			}
		}

		return nearest_node;
	}

	std::shared_ptr<RRT_STAR_Node> newConfig(std::shared_ptr<RRT_STAR_Node> const& nearest_node,
	                                         std::pair<int, int> const& sample) {
		float distance = std::sqrt(std::pow(nearest_node->get_x() - sample.first, 2) +
		                           std::pow(nearest_node->get_y() - sample.second, 2));
		float ratio = step_size / distance;
		int x = static_cast<int>(nearest_node->get_x() + ratio * (sample.first - nearest_node->get_x()));
		int y = static_cast<int>(nearest_node->get_y() + ratio * (sample.second - nearest_node->get_y()));
		return std::make_shared<RRT_STAR_Node>(x, y);
	}

	std::shared_ptr<RRT_STAR_Node> bestParent(std::shared_ptr<RRT_STAR_Node> const& new_node,
	                                          nav_msgs::msg::OccupancyGrid const& grid) {
		float best_cost = std::numeric_limits<float>::max();
		RCLCPP_DEBUG(this->get_logger(), "Best cost: %f", best_cost);
		std::shared_ptr<RRT_STAR_Node> best_parent;

		for (auto const& node : nodes) {
			RCLCPP_DEBUG(this->get_logger(), "Node point: (%d, %d)", node->get_x(), node->get_y());
			if (obstacleFree(node->get_x(), node->get_y(), new_node->get_x(), new_node->get_y(), grid)) {
				float distance =
				    std::sqrt(std::pow(node->get_x() - new_node->get_x(), 2) + std::pow(node->get_y() - new_node->get_y(), 2));
				RCLCPP_DEBUG(this->get_logger(), "Distance: %f", distance);

				if (distance < radius) {
					RCLCPP_DEBUG(this->get_logger(), "radius is %f", radius);
					RCLCPP_DEBUG(this->get_logger(), "cost of node: %f", node->get_cost());
					float new_cost = node->get_cost() + distance;
					if (new_cost < best_cost) {
						best_cost = new_cost;
						best_parent = node;
					}
				}
			}
		}

		return best_parent;
	}

	void rewire(std::shared_ptr<RRT_STAR_Node> const& new_node) {
		for (auto const& node : nodes) {
			float distance =
			    std::sqrt(std::pow(node->get_x() - new_node->get_x(), 2) + std::pow(node->get_y() - new_node->get_y(), 2));
			if (distance < radius) {
				float new_cost = new_node->get_cost() + distance;
				if (new_cost < node->get_cost()) {
					node->set_cost(new_cost);
					node->set_parent(new_node);
				}
			}
		}
	}

	bool isGoalReached(std::shared_ptr<RRT_STAR_Node> const& new_node, std::shared_ptr<RRT_STAR_Node> const& goal_node) {
		float distance = std::sqrt(std::pow(new_node->get_x() - goal_node->get_x(), 2) +
		                           std::pow(new_node->get_y() - goal_node->get_y(), 2));
		is_goal_reached = distance < 2.0f;
		return distance < 2.0f;
	}

	std::vector<std::shared_ptr<RRT_STAR_Node>> getPath(std::shared_ptr<RRT_STAR_Node> const& start_node,
	                                                    std::shared_ptr<RRT_STAR_Node> const& goal_node) {
		std::vector<std::shared_ptr<RRT_STAR_Node>> path;
		std::shared_ptr<RRT_STAR_Node> current_node = goal_node;

		while (current_node != start_node) {
			RCLCPP_INFO(this->get_logger(),
			            "Node point: (%d, %d) has parent(%d, %d)",
			            current_node->get_x(),
			            current_node->get_y(),
			            current_node->get_parent()->get_x(),
			            current_node->get_parent()->get_y());
			path.push_back(current_node);
			current_node = current_node->get_parent();
		}

		path.push_back(start_node);
		std::reverse(path.begin(), path.end());
		RCLCPP_INFO(this->get_logger(), "Path size: %zu", path.size());
		return path;
	}

	void publishPath(std::vector<std::shared_ptr<RRT_STAR_Node>> const& path, nav_msgs::msg::OccupancyGrid const& grid) {
		std::vector<geometry_msgs::msg::PoseStamped> path_reversed;
		nav_msgs::msg::Path path_msg;
		int node_count = 0;
		path_msg.header.frame_id = grid.header.frame_id;
		path_msg.header.stamp = grid.header.stamp;
		// RCLCPP_INFO(this->get_logger(),
		//             "Goal point has parent(%d, %d)",
		//             goal_node->get_parent()->get_x(),
		//             goal_node->get_parent()->get_y());

		for (auto const& node : path) {
			node_count++;
			geometry_msgs::msg::PoseStamped pose;
			pose.header.frame_id = grid.header.frame_id;
			pose.header.stamp = grid.header.stamp;
			pose.pose.position.x = node->get_x() - 5.0;
			pose.pose.position.y = node->get_y() - 5.0;
			pose.pose.position.z = 0;
			pose.pose.orientation.w = 1.0;
			path_reversed.push_back(pose);
		}

		RCLCPP_INFO(this->get_logger(), "Found goal path. Total number of nodes in the path: %d", node_count);

		path_publisher_->publish(path_msg);
	}

	void occupancyGridCallback(nav_msgs::msg::OccupancyGrid const& grid) {

		auto const& grid_data = grid.data;
		int grid_width = grid.info.width;

		// Count free and occupied cells
		int free_cells = 0, occupied_cells = 0;
		for (auto cell : grid_data) {
			if (cell == 0)
				free_cells++;
			else if (cell == 100)
				occupied_cells++;
		}
		auto start_coords = indexToCoordinates(0, grid_width);
		auto goal_coords = indexToCoordinates(89, grid_width);

		this->start_point.x = static_cast<float>(start_coords.first);
		this->start_point.y = static_cast<float>(start_coords.second);
		this->goal_point.x = static_cast<float>(goal_coords.first);
		this->goal_point.y = static_cast<float>(goal_coords.second);

		start_node = std::make_shared<RRT_STAR_Node>(start_coords.first, start_coords.second);
		goal_node = std::make_shared<RRT_STAR_Node>(goal_coords.first, goal_coords.second);

		nodes.push_back(start_node);
		// nodes.push_back(goal_node);
		RCLCPP_INFO(this->get_logger(),
		            "Starting point: (%f, %f) - Goal point: (%f, %f)",
		            this->start_point.x,
		            this->start_point.y,
		            this->goal_point.x,
		            this->goal_point.y);

		for (int i = 0; i < max_iterations; ++i) {

			std::pair<int, int> sample = sampleRandomNode(grid);

			RCLCPP_DEBUG(this->get_logger(), "Sampled point: (%d, %d)", sample.first, sample.second);
			std::shared_ptr<RRT_STAR_Node> nearest_node = nearestNode(sample);
			if (nearest_node->get_x() == sample.first && nearest_node->get_y() == sample.second) {
				continue;
			}

			RCLCPP_DEBUG(this->get_logger(), "Nearest node: (%d, %d)", nearest_node->get_x(), nearest_node->get_y());
			std::shared_ptr<RRT_STAR_Node> new_node = newConfig(nearest_node, sample);

			RCLCPP_DEBUG(this->get_logger(), "New node: (%d, %d)", new_node->get_x(), new_node->get_y());

			if (!obstacleFree(nearest_node->get_x(), nearest_node->get_y(), new_node->get_x(), new_node->get_y(), grid)) {
				RCLCPP_DEBUG(this->get_logger(), "Obstacle detected between nearest and new node");
				continue;
			}

			std::shared_ptr<RRT_STAR_Node> best_parent = bestParent(new_node, grid);
			new_node->set_cost(best_parent->get_cost() + std::sqrt(std::pow(new_node->get_x() - best_parent->get_x(), 2) +
			                                                       std::pow(new_node->get_y() - best_parent->get_y(), 2)));

			nodes.push_back(new_node);
			RCLCPP_DEBUG(this->get_logger(), "New node cost: %f", new_node->get_cost());

			new_node->set_parent(best_parent);
			RCLCPP_DEBUG(this->get_logger(), "New node parent: (%d, %d)", best_parent->get_x(), best_parent->get_y());
			rewire(new_node);
			RCLCPP_DEBUG(this->get_logger(), "Rewired new node: (%d, %d)", new_node->get_x(), new_node->get_y());

			if (isGoalReached(new_node, goal_node)) {
				RCLCPP_INFO(this->get_logger(),
				            "Node point: (%d, %d) - Goal point: (%d, %d)",
				            new_node->get_x(),
				            new_node->get_y(),
				            goal_node->get_x(),
				            goal_node->get_y());
				path = getPath(start_node, new_node);
				publishPath(path, grid);

				// std::vector<geometry_msgs::msg::PoseStamped> path_reversed;
				// auto path_node = new_node;
				// RCLCPP_INFO(this->get_logger(),
				//             "Goal point has parent(%d, %d)",
				//             goal_node->get_parent()->get_x(),
				//             goal_node->get_parent()->get_y());
				// RCLCPP_INFO(this->get_logger(),
				//             "Node point: (%d, %d) has parent(%d, %d)",
				//             new_node->get_x(),
				//             new_node->get_y(),
				//             new_node->get_parent()->get_x(),
				//             new_node->get_parent()->get_y());
				// int node_count = 0;
				// while (path_node != nullptr) {
				// 	// RCLCPP_DEBUG(this->get_logger(),
				// 	//              "Node at (%d, %d) has parent (%d,%d)",
				// 	//              path_node->x,
				// 	//              path_node->y,
				// 	//              path_node->get_parent()->x,
				// 	//              path_node->get_parent()->y);
				// 	node_count++;
				// 	geometry_msgs::msg::PoseStamped pose;
				// 	pose.header.frame_id = grid.header.frame_id;
				// 	pose.header.stamp = grid.header.stamp;
				// 	pose.pose.position.x = path_node->get_x() - 5.0;
				// 	pose.pose.position.y = path_node->get_y() - 5.0;
				// 	pose.pose.position.z = 0;
				// 	pose.pose.orientation.w = 1.0;
				// 	path_reversed.push_back(pose);
				// 	path_node = path_node->get_parent();
				// }
				// RCLCPP_INFO(this->get_logger(), "Found goal path. Total number of nodes in the path: %d", node_count);
				// std::vector<geometry_msgs::msg::PoseStamped> path;
				// for (auto it = path_reversed.rbegin(); it != path_reversed.rend(); ++it) {
				// 	path.push_back(*it);
				// }
				// nav_msgs::msg::Path path_msg;
				// path_msg.header.frame_id = grid.header.frame_id;
				// path_msg.header.stamp = grid.header.stamp;
				// path_msg.poses = path;
				// path_publisher_->publish(path_msg);
				// break;
			}
		}
	}

	bool obstacleFree(int x1, int y1, int x2, int y2, nav_msgs::msg::OccupancyGrid const& grid) {

		int grid_width = grid.info.width;
		int grid_height = grid.info.height;
		auto const& grid_data = grid.data;

		// Check if the line between the two points is obstacle free
		int dx = std::abs(x2 - x1);
		int dy = std::abs(y2 - y1);
		int x = x1;
		int y = y1;
		int n = 1 + dx + dy;
		int x_inc = (x2 > x1) ? 1 : -1;
		int y_inc = (y2 > y1) ? 1 : -1;
		int error = dx - dy;
		dx *= 2;
		dy *= 2;

		for (; n > 0; --n) {
			RCLCPP_DEBUG(this->get_logger(), "Checking point (%d, %d) with value %d", x, y, grid_data[x + y * grid_width]);
			if (x < 0 || x >= grid_width || y < 0 || y >= grid_height) {
				RCLCPP_DEBUG(this->get_logger(), "Point (%d, %d) out of bounds", x, y);

				return false;
			}
			if (grid_data[x + y * grid_width] != 0) {
				RCLCPP_DEBUG(this->get_logger(), "Collision at point (%d, %d)", x, y);
				return false;
			}
			if (error > 0) {
				x += x_inc;
				error -= dy;
			} else {
				y += y_inc;
				error += dx;
			}
		}
		return true;
	}

	// bool obstacleFree(int x1, int y1, int x2, int y2, nav_msgs::msg::OccupancyGrid const& grid) {
	// 	int grid_width = grid.info.width;
	// 	int grid_height = grid.info.height;
	// 	auto const& grid_data = grid.data;

	// 	int num_samples = 10; // Number of points to sample along the line

	// 	for (int i = 0; i <= num_samples; ++i) {
	// 		float alpha = static_cast<float>(i) / num_samples;
	// 		int x = static_cast<int>(x1 + alpha * (x2 - x1));
	// 		int y = static_cast<int>(y1 + alpha * (y2 - y1));

	// 		RCLCPP_DEBUG(this->get_logger(), "Sampled point for obstacle: (%d, %d)", x, y);

	// 		if (x < 0 || x >= grid_width || y < 0 || y >= grid_height) {
	// 			return false; // Point is out of bounds
	// 		}

	// 		if (grid_data[x + y * grid_width] != 0) {
	// 			return false; // Collision at this point
	// 		}
	// 	}

	// 	return true; // No collision detected
	// }

	rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_subscriber_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
	geometry_msgs::msg::Point start_point;
	geometry_msgs::msg::Point goal_point;
};

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RRT_STAR_planner>());
	rclcpp::shutdown();
	return 0;
}