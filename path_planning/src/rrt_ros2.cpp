#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <random>
#include <rclcpp/logging.hpp>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

class RRT_Node {
public:
	int x, y;

	std::shared_ptr<RRT_Node> parent;

	RRT_Node(int x, int y, std::shared_ptr<RRT_Node> parent = nullptr) : x(x), y(y), parent(parent) {
	}

	void set_parent(std::shared_ptr<RRT_Node> new_parent) {
		parent = new_parent;
	}

	std::shared_ptr<RRT_Node> get_parent() {
		return parent;
	}
};

float heuristic(int x1, int y1, int x2, int y2) {
	return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}

class RRTNode : public rclcpp::Node {
public:
	RRTNode() : Node("rrt_node"), generator_(std::random_device{}()) {
		RCLCPP_INFO(this->get_logger(), "Performing RRT Search");

		occupancy_grid_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
		    "occupancy_grid", 10, std::bind(&RRTNode::occupancyGridCallback, this, std::placeholders::_1));

		// Publishers for the path
		path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);
	}

private:
	std::default_random_engine generator_;

	std::pair<int, int> indexToCoordinates(int index, int width) {
		if (index < 0 || index >= width * width) {
			RCLCPP_ERROR(this->get_logger(), "Index out of bounds");
			return {-1, -1}; // Return an invalid coordinate
		}
		int y = index / width;
		int x = index % width;
		return {x, y};
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

	void extendRandomNode(std::shared_ptr<RRT_Node>& nearest_node,
	                      std::shared_ptr<RRT_Node>& random_node,
	                      float step_size,
	                      std::vector<std::shared_ptr<RRT_Node>>& tree_,
	                      nav_msgs::msg::OccupancyGrid const& grid_) {

		float distance = heuristic(nearest_node->x, nearest_node->y, random_node->x, random_node->y);

		if (distance > step_size) {
			float dx = (random_node->x - nearest_node->x) / distance;
			float dy = (random_node->y - nearest_node->y) / distance;
			int new_x = static_cast<int>(nearest_node->x + step_size * dx);
			int new_y = static_cast<int>(nearest_node->y + step_size * dy);
			auto new_node = std::make_shared<RRT_Node>(new_x, new_y);
			if (obstacleFree(nearest_node->x, nearest_node->y, new_x, new_y, grid_)) {
				new_node->set_parent(nearest_node);
				tree_.push_back(new_node);
				RCLCPP_DEBUG(this->get_logger(), "Added node at (%d, %d) to the tree", new_node->x, new_node->y);
				RCLCPP_DEBUG(this->get_logger(),
				             "Added node at (%d, %d) to the tree with parent (%d, %d)",
				             new_node->x,
				             new_node->y,
				             new_node->get_parent()->x,
				             new_node->get_parent()->y);
				random_node = new_node;
			}
		} else if (distance <= step_size && distance > 0.0f) {
			if (obstacleFree(nearest_node->x, nearest_node->y, random_node->x, random_node->y, grid_)) {
				random_node->set_parent(nearest_node);
				tree_.push_back(random_node);
				RCLCPP_DEBUG(this->get_logger(), "Added node at (%d, %d) to the tree", random_node->x, random_node->y);
				RCLCPP_DEBUG(this->get_logger(),
				             "Added node at (%d, %d) to the tree with parent (%d, %d)",
				             random_node->x,
				             random_node->y,
				             random_node->get_parent()->x,
				             random_node->get_parent()->y);
			}
		}
	}

	void occupancyGridCallback(nav_msgs::msg::OccupancyGrid const& grid) {
		int grid_width = grid.info.width;
		int grid_height = grid.info.height;
		auto const& grid_data = grid.data;

		// Count free and occupied cells
		int free_cells = 0, occupied_cells = 0;
		for (auto cell : grid_data) {
			if (cell == 0)
				free_cells++;
			else if (cell == 100)
				occupied_cells++;
		}

		// RCLCPP_INFO(this->get_logger(), "Free cells: %d, Occupied cells: %d",
		//             free_cells, occupied_cells);

		// Convert indices to coordinates and set start and goal points
		auto start_coords = indexToCoordinates(0, grid_width);
		auto goal_coords = indexToCoordinates(89, grid_width);

		//  RCLCPP_INFO(this->get_logger(), "Raw start coords: (%d, %d), Raw goal
		//  coords: (%d, %d)",
		//           start_coords.first, start_coords.second, goal_coords.first,
		//           goal_coords.second);

		// Use class member variables directly
		this->start_point.x = static_cast<float>(start_coords.first);
		this->start_point.y = static_cast<float>(start_coords.second);
		this->goal_point.x = static_cast<float>(goal_coords.first);
		this->goal_point.y = static_cast<float>(goal_coords.second);
		// Print start and goal points
		RCLCPP_INFO(this->get_logger(),
		            "Starting point: (%f, %f) - Goal point: (%f, %f)",
		            this->start_point.x,
		            this->start_point.y,
		            this->goal_point.x,
		            this->goal_point.y);
		// RCLCPP_INFO(this->get_logger(), "Map Width: %d, Map Height: %d",
		// grid_width, grid_height);
		std::vector<std::shared_ptr<RRT_Node>> tree;
		tree.push_back(std::make_shared<RRT_Node>(static_cast<int>(start_point.x), static_cast<int>(start_point.y)));

		std::uniform_int_distribution<int> distribution(0, grid_width * grid_height - 1);

		int max_iterations = 1000;
		float step_size = 3.0f;

		for (int i = 0; i < max_iterations; i++) {
			int random_index = distribution(generator_);
			auto random_coords = indexToCoordinates(random_index, grid_width);
			auto random_node = std::make_shared<RRT_Node>(random_coords.first, random_coords.second);
			auto nearest_node = tree[0];
			float min_distance = std::numeric_limits<float>::max();

			for (auto node : tree) {
				float distance = heuristic(node->x, node->y, random_node->x, random_node->y);
				RCLCPP_DEBUG(this->get_logger(),
				             "Checking existent node at (%d, %d) against random node (%d, %d)",
				             node->x,
				             node->y,
				             random_node->x,
				             random_node->y);
				if (distance < min_distance) {

					min_distance = distance;
					nearest_node = node;

					RCLCPP_DEBUG(this->get_logger(), "nearest node is (%d, %d)", nearest_node->x, nearest_node->y);
				}
			}

			extendRandomNode(nearest_node, random_node, step_size, tree, grid);

			// Add the random node to the tree
			// RCLCPP_DEBUG(this->get_logger(), "Added node at (%d, %d) to the tree",
			//              random_node->x, random_node->y);
			// Check if the random node is close to the goal

			if (heuristic(random_node->x, random_node->y, goal_point.x, goal_point.y) < 1.0 &&
			    random_node->get_parent() != nullptr) {
				RCLCPP_DEBUG(this->get_logger(),
				             "Close to node (%d, %d) is goal node (%d, %d)",
				             random_node->x,
				             random_node->y,
				             static_cast<int>(goal_point.x),
				             static_cast<int>(goal_point.y));

				std::vector<geometry_msgs::msg::PoseStamped> path_reversed;
				auto path_node = random_node;
				int node_count = 0;
				while (path_node != nullptr && path_node->get_parent() != nullptr) {
					RCLCPP_DEBUG(this->get_logger(),
					             "Node at (%d, %d) has parent (%d,%d)",
					             path_node->x,
					             path_node->y,
					             path_node->get_parent()->x,
					             path_node->get_parent()->y);
					node_count++;
					geometry_msgs::msg::PoseStamped pose;
					pose.header.frame_id = grid.header.frame_id;
					pose.header.stamp = grid.header.stamp;
					pose.pose.position.x = path_node->x - 5.0;
					pose.pose.position.y = path_node->y - 5.0;
					pose.pose.position.z = 0;
					pose.pose.orientation.w = 1.0;
					path_reversed.push_back(pose);
					path_node = path_node->parent;
				}
				RCLCPP_INFO(this->get_logger(), "Found goal path. Total number of nodes in the path: %d", node_count);
				std::vector<geometry_msgs::msg::PoseStamped> path;
				for (auto it = path_reversed.rbegin(); it != path_reversed.rend(); ++it) {
					path.push_back(*it);
				}
				nav_msgs::msg::Path path_msg;
				path_msg.header.frame_id = grid.header.frame_id;
				path_msg.header.stamp = grid.header.stamp;
				path_msg.poses = path;
				path_publisher_->publish(path_msg);
				break;
			}
		}
	}

	rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_subscriber_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
	geometry_msgs::msg::Point start_point;
	geometry_msgs::msg::Point goal_point;
};

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<RRTNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
