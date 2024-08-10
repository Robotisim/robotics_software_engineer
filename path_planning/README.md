## Path Planning Package

The `path_planning` package contains comprehensive set of algorithms and utilities for implementing various path planning strategies in robotic applications. This package includes refined implementations of A* (A-star), Grid Sweep, and Rapidly-exploring Random Tree (RRT) algorithms, all tightly integrated with ROS 2 for seamless deployment and visualization

#### Source Files and Headers
**Note**: File names might have changed since the initial lectures, as this represents the final form of the package as it evolved by the last lecture.


1. **path_planning.cpp** (Improved Version)
   - **Purpose:** Acts as the central node integrating all path planning algorithms (A*, RRT, Grid Sweep) and provides a unified interface for executing them based on the scenario.
   - **Key Functions:**
     - `void PathPlanning::choose_algorithm()` - Allows selection of the most appropriate path planning algorithm based on the current environment and task.
     - `void PathPlanning::execute_algorithm()` - Executes the selected path planning algorithm and outputs the resulting path.
     - **Improvements:** Enhanced logic for better decision-making when choosing the path planning algorithm, optimizing the execution flow, and integrating more robust error handling mechanisms.

#### Launch Files

1. **path_plan_rviz.launch.py**
   - **Purpose:** Launches the occupancy grid and path planning nodes along with RViz for visualization.
   - **Nodes Launched:**
     - `occupancy_grid_node` - Manages the occupancy grid for path planning.
     - `path_planning_node` - Executes the chosen path planning algorithm.
     - `rviz2` - Visualizes the environment, occupancy grid, and planned path.
   - **Command to run:**
   ```sh
   ros2 launch path_planning path_plan_rviz.launch.py
   ```

### Algorithms Implemented

1. **A* (A-star)**
   - **Description:** A grid-based algorithm that efficiently finds the shortest path using heuristic search techniques.
   - **Use Case:** Best suited for structured environments with well-defined grids, such as indoor navigation.

2. **Grid Sweep**
   - **Description:** Ensures complete coverage of a grid by systematically sweeping through all areas, ensuring no spot is left unvisited.
   - **Use Case:** Ideal for applications requiring full coverage, such as vacuum cleaning robots or agricultural drones.

3. **RRT (Rapidly-exploring Random Tree)**
   - **Description:** A versatile algorithm that quickly explores complex, high-dimensional spaces, making it suitable for environments where grid-based methods are impractical.
   - **Use Case:** Best for unstructured environments where traditional methods struggle, such as outdoor or dynamic environments.

### CMakeLists Information

The `CMakeLists.txt` file is configured to build and link all components of the `path_planning` package:

- **Executables:**
  - `occupancy_grid_node`
  - `path_planning_node`

- **Libraries:**
  - `algo_astar`
  - `algo_grid_sweep`
  - `algo_rrt`

- **Tests:**
  - GTest-based unit  and integeration tests for RRT implementations, ensuring algorithm robustness and reliability.

### Building and Running

To build the package:

```sh
colcon build --packages-select path_planning
```

To source the setup file:

```sh
source install/setup.bash
```

To run the tests:

```sh
colcon test --packages-select path_planning
```

### Example Commands

- **Path Planning with RViz:**
  ```sh
  ros2 launch path_planning path_plan_rviz.launch.py
  ```

### Test your Learnings with Assignments
- [Assignment](https://github.com/Robotisim/robotics_software_engineer/tree/assignments/module_8_assignment)