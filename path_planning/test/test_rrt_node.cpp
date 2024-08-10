#include <gtest/gtest.h>
#include "algo_rrt_node.hpp"


TEST(NodeTest, DefaultConstructor) {
    NodeRrt node(1, 2);
    EXPECT_EQ(node.getX(), 1);
    EXPECT_EQ(node.getY(), 2);
    EXPECT_EQ(node.getCost(), 0.0f);
    EXPECT_EQ(node.getParent(), nullptr);
}


TEST(NodeTest, ParameterizedConstructor) {
    auto parent = std::make_shared<NodeRrt>(0, 0);
    NodeRrt node(1, 2, parent, 1.5f);
    EXPECT_EQ(node.getX(), 1);
    EXPECT_EQ(node.getY(), 2);
    EXPECT_EQ(node.getCost(), 1.5f);
    EXPECT_EQ(node.getParent(), parent);
}



TEST(NodeTest, SetParent) {
    auto parent = std::make_shared<NodeRrt>(0, 0);
    NodeRrt node(1, 2);
    node.setParent(parent);
    EXPECT_EQ(node.getParent(), parent);
}

TEST(NodeTest, SetCost) {
    NodeRrt node(1, 2);
    node.setCost(2.5f);
    EXPECT_EQ(node.getCost(), 2.5f);
}

TEST(NodeTest, EqualityOperator) {
    NodeRrt node1(1, 2);
    NodeRrt node2(1, 2);
    NodeRrt node3(2, 3);
    EXPECT_TRUE(node1 == node2);
    EXPECT_FALSE(node1 == node3);
}

TEST(NodeTest, Heuristics) {
    NodeRrt node1(0, 0);
    NodeRrt node2(3, 4);
    EXPECT_EQ(NodeRrt::heuristics(node1, node2), 7);
    EXPECT_NEAR(NodeRrt::heuristicsEuclid(node1, node2), 5.2, 0.21);
}


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
