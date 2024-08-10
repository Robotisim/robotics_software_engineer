#include "gtest/gtest.h"
#include "algo_rrt.hpp"
#include <random>


TEST(RRTPlannerTest, GenerateRandomNode) {
    RRT_Planner planner;
    std::mt19937 gen(0);
    NodeRrt randomNode = planner.generateRandomNode(gen);
    EXPECT_GE(randomNode.getX(), 0);
    EXPECT_LT(randomNode.getX(), 20);
    EXPECT_GE(randomNode.getY(), 0);
    EXPECT_LT(randomNode.getY(), 20);
}

TEST(RRTPlannerTest, FindNearestNode) {
    RRT_Planner planner;
    NodeRrt start(0, 0);
    NodeRrt node1(5, 5);
    NodeRrt node2(10, 10);
    std::vector<NodeRrt> nodes = { start, node1, node2 };
    NodeRrt randomNode(6, 6);
    NodeRrt nearestNode = planner.findNearestNode(nodes, randomNode);
    EXPECT_EQ(nearestNode, node1);
}

TEST(RRTPlannerTest, FindNewConfig) {
    RRT_Planner planner;
    NodeRrt nearestNode(0, 0);
    NodeRrt randomNode(3, 4);
    NodeRrt newNode = planner.findNewConfig(nearestNode, randomNode);
    EXPECT_NEAR(newNode.getX(), 1.2, 0.7);
    EXPECT_NEAR(newNode.getY(), 1.6, 0.7);
}

TEST(RRTPlannerTest, IsObstacle) {
    RRT_Planner planner;
    std::vector<int> domain(20 * 20, 0);
    planner.setDomain(domain);
    NodeRrt nearestNode(0, 0);
    NodeRrt newNode(3, 4);
    EXPECT_FALSE(planner.isObstacle(nearestNode, newNode));

    domain[4 * 20 + 3] = 100; // Set (3, 4) as an obstacle
    planner.setDomain(domain);
    EXPECT_TRUE(planner.isObstacle(nearestNode, newNode));
}

TEST(RRTPlannerTest, IsGoalFound) {
    RRT_Planner planner;
    NodeRrt newNode(3, 4);
    NodeRrt goal(3, 4);
    EXPECT_TRUE(planner.isGoalFound(newNode, goal));

    NodeRrt otherNode(5, 5);
    EXPECT_FALSE(planner.isGoalFound(otherNode, goal));
}
void printPath(const std::vector<NodeRrt>& path) {
    for (const auto& node : path) {
        std::cout << "(" << node.getX() << ", " << node.getY() << ") -> ";
    }
    std::cout << "Goal" << std::endl;
}

TEST(RRTPlannerTest, PlanPath) {
    RRT_Planner planner;
    std::vector<int> domain(20 * 20, 0);
    planner.setDomain(domain);
    NodeRrt start(0, 0);
    NodeRrt goal(3, 5);
    std::vector<NodeRrt> path = planner.planPath(start, goal);

    ASSERT_FALSE(path.empty());
    EXPECT_EQ(path.front(), start);
    EXPECT_EQ(path.back(), goal);

    std::cout << "Path size: " << path.size() << std::endl;
    printPath(path);
}

TEST(RRTPlannerTest, PathValidity) {
    RRT_Planner planner;
    std::vector<int> domain(20 * 20, 0);
    domain[2 * 20 + 2] = 100; // Set (2, 2) as an obstacle
    planner.setDomain(domain);
    NodeRrt start(0, 0);
    NodeRrt goal(3, 5);
    std::vector<NodeRrt> path = planner.planPath(start, goal);

    ASSERT_FALSE(path.empty());

    for (const auto& node : path) {
        EXPECT_FALSE(planner.isObstacle(node, node));
    }
}

TEST(RRTPlannerTest, NoPathFound) {
    RRT_Planner planner;
    std::vector<int> domain(20 * 20, 100); // Fill domain with obstacles to make pathfinding impossible
    planner.setDomain(domain);
    NodeRrt start(0, 0);
    NodeRrt goal(19, 19);
    std::vector<NodeRrt> path = planner.planPath(start, goal);

    EXPECT_TRUE(path.empty());
}




TEST(RRTPlannerTest, CorrectParentAssignment) {
    RRT_Planner planner;
    std::vector<int> domain(20 * 20, 0);
    planner.setDomain(domain);
    NodeRrt start(0, 0);
    NodeRrt goal(3, 5);
    std::vector<NodeRrt> path = planner.planPath(start, goal);

    ASSERT_FALSE(path.empty());

    for (size_t i = 1; i < path.size(); ++i) {
        EXPECT_EQ(*path[i].getParent(), path[i - 1]);
    }
}



int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}