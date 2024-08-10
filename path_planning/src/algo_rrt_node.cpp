#include <algo_rrt_node.hpp>
#include <cmath>
#include <memory>
#include <utility>


NodeRrt::NodeRrt(int x, int y, std::shared_ptr<NodeRrt> parent, float cost)
    : x(x), y(y), cost(cost), parent(std::move(parent)) {}

NodeRrt::NodeRrt(int x, int y) : x(x), y(y) {}

void NodeRrt::setCost(float cost) { this->cost = cost; }
void NodeRrt::setParent(std::shared_ptr<NodeRrt> parent) {
  this->parent = std::move(parent);
}


auto NodeRrt::getParent() -> std::shared_ptr<NodeRrt> { return parent; }
auto NodeRrt::getCost() const -> float { return cost; }
auto NodeRrt::getX() const -> int { return x; }
auto NodeRrt::getY() const -> int { return y; }



bool NodeRrt::operator==(const NodeRrt& node) const {
    return (x == node.x && y == node.y);
}


auto NodeRrt::heuristics(NodeRrt const &node_1, NodeRrt const &node_2) -> float {
  return std::abs(static_cast<float>(node_1.getX() - node_2.getX())) +
         std::abs(static_cast<float>(node_1.getY() - node_2.getY()));
}

float NodeRrt::heuristicsEuclid(NodeRrt const &node_1, NodeRrt const &node_2) {
  return std::sqrt(std::pow(node_1.getX() - node_2.getX(), 2) +
                   std::pow(node_1.getY() - node_2.getY(), 2));
}

