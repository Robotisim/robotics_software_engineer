#ifndef NODE_HPP
#define NODE_HPP
#include <memory>

namespace PathPlanning {
	class Node {
	public:
		Node(int x, int y, std::shared_ptr<Node> parent, float cost);
		Node(int x, int y);
		void setParent(std::shared_ptr<Node> parent);
		auto getParent() -> std::shared_ptr<Node>;
		void setCost(float cost);
		auto getCost() const -> float;
		auto getX() const -> int;
		auto getY() const -> int;
		auto operator==(Node const& node) -> bool;
		Node();
		static auto heuristics(Node const& node_1, Node const& node_2) -> float;
		static float heuristicsEuclid(Node const& node_1, Node const& node_2);

	private:
		int x, y;
		float cost = 0.0F;
		std::shared_ptr<Node> parent = nullptr;
	};
} // namespace PathPlanning

#endif // NODE_HPP