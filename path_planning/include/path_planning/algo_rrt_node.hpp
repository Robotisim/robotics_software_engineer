#ifndef RRT_NODE_HPP
#define RRT_NODE_HPP

#include <memory>

class NodeRrt {
public:


		NodeRrt(int x, int y, std::shared_ptr<NodeRrt> parent, float cost);
		NodeRrt(int x, int y);

		void setParent(std::shared_ptr<NodeRrt> parent);
        void setCost(float cost);

		auto getParent() -> std::shared_ptr<NodeRrt>;
		auto getCost() const -> float;
		auto getX() const -> int;
		auto getY() const -> int;
    	bool operator==(const NodeRrt& NodeRrt) const;
		NodeRrt();
		static auto heuristics(NodeRrt const& node_1, NodeRrt const& node_2) -> float;
		static float heuristicsEuclid(NodeRrt const& node_1, NodeRrt const& node_2);


private:

		int x, y;
		float cost = 0.0F;
		std::shared_ptr<NodeRrt> parent = nullptr;



};

#endif // RRT_NODE_HPP
