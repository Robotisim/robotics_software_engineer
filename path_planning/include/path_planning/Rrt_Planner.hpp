#ifndef RRT_PLANNER_HPP
#define RRT_PLANNER_HPP

#include <Node.hpp>
#include <array>
#include <memory>
#include <random>
#include <vector>

namespace PathPlanning {
	class RRT_Planner {
	public:
		RRT_Planner();
		Node indexToCoordinate(int index);
		Node findNearestNode(std::vector<Node> const& nodes, Node const& random_node);
		Node findNewConfig(Node const& nearest_node, Node const& random_node);
		bool isObstacle(Node const& nearest_node, Node const& new_node);
		std::vector<Node> planPath(Node const& start, Node const& goal);
		void setDomain(std::vector<int>& domain);
		Node START = Node(0, 0);
		Node GOAL = Node(8, 8);
		void setStepSize(float step_size);
		void setGridHeight(int height);
		void setGridWidth(int width);

	private:
		Node generateRandomNode(std::mt19937& gen);

		bool isGoalFound(Node& new_node, Node const& goal);

		void publishPath(std::vector<Node> const& path);

		int const MAX_ITERATIONS = 1000;
		int const SEED = 0;

		int GRID_WIDTH = 10;
		int GRID_HEIGHT = 10;
		float STEP_SIZE = 2.0F;

		std::array<int, 10 * 10> DOMAIN{};
		std::vector<Node> nodes;
		std::vector<Node> path;
	};

} // namespace PathPlanning

#endif // RRT_PLANNER_HPP