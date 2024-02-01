#ifndef RRT_H
#define RRT_H

#include<iostream>
#include<limits>
#include<vector>
#include<random>
#include<opencv2/opencv.hpp>

class Node{
public:
  float x, y;
  std::vector<float> path_x, path_y;
  Node* parent;
  float cost;
  Node(float x_coord, float y_coord);
};

class RRT{
public:
  Node* getRandomNode(); // Function declaration
  bool isNodeCloseToGoal(Node* node);
  std::vector<Node*> generateFinalPath(Node* last_node);


  RRT(Node* start_node, Node* goal_node, std::vector<std::vector<float>> obstacle_list,
      std::vector<float> sampling_area, float expand_distance=1.0,
      float path_resolution=1.0, int goal_sample_rate=5, int max_iterations=500);

  std::vector<Node*> planPath();

private:
    Node* getClosestNode(Node* rnd_node);
    bool isCollisionFree(Node* node);
    Node* createNewNode(Node* nearest, Node* random_node, float max_extend_length);
    static std::vector<float> calculateDistanceAndAngle(Node* from_node, Node* to_node);

    Node* start;
    Node* goal;
    std::vector<std::vector<float>> obstacles;
    std::vector<float> sampling_area;
    float expand_distance;
    float path_resolution;
    int goal_sample_rate;
    int max_iterations;
    std::vector<Node*> nodes;

    std::random_device rd;
    std::mt19937 gen;
    std::uniform_int_distribution<int> goal_distribution;
    std::uniform_real_distribution<float> area_distribution;
    };

#endif // RRT_H
