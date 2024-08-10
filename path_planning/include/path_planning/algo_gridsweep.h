
#ifndef ALGO_GRIDSWEEP_H
#define ALGO_GRIDSWEEP_H

#include <nav_msgs/msg/occupancy_grid.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <vector>
#include <iostream>
std::vector<geometry_msgs::msg::PoseStamped> grid_sweep(const nav_msgs::msg::OccupancyGrid& grid);

#endif // ALGO_GRIDSWEEP_H
