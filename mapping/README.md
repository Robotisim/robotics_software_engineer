## SLAM and Mapping Package

This package is dedicated to teaching and implementing various SLAM (Simultaneous Localization and Mapping) and mapping techniques. The package includes tools and nodes for generating and using maps in both 2D and 3D environments, utilizing TurtleBot3 and other robots in ROS 2 simulations.
#### Source Files

1. **lidar_to_grid.cpp** - Converts LiDAR scan data to an occupancy grid format for mapping purposes.

2. **occupancy_grid_pub.cpp** - Publishes an occupancy grid map based on the robot's sensor inputs.

#### Launch Files
**Note**: File names might have changed since the initial lectures, as this represents the final form of the package as it evolved by the last lecture.


1. **maze_tb3_bringup.launch.py** - Initializes the TurtleBot3 in a simulated maze environment and sets up Gazebo, RViz, and the robot's sensors.

   **Purpose:**
   - To set up a complete simulation environment for TurtleBot3 in a maze world.

   **Command to run:**
   ```sh
   ros2 launch mapping maze_tb3_bringup.launch.py
   ```
2. **map_loading_2d.launch.py** - Launches the map server and RViz for visualizing a pre-built 2D map.

   **Purpose:** - To load and visualize a 2D occupancy grid map using the map server in ROS 2.

   **Command to run:**
   ```sh
   ros2 launch mapping map_loading_2d.launch.py
   ```

3. **maze_mapping_2d.launch.py** - Launches the environment and SLAM process in a 2D maze world using TurtleBot3 and the `slam_toolbox` package.

   **Purpose:**
   - To perform 2D SLAM in a maze environment using TurtleBot3.

   **Command to run:**
   ```sh
   ros2 launch mapping maze_mapping_2d.launch.py
   ```

4. **maze_mapping_3d.launch.py** - Sets up the TurtleBot3 in a 3D maze environment for SLAM using depth camera data and RTAB-Map.

   **Purpose:**
   - To perform 3D SLAM in a maze environment with a depth camera on TurtleBot3.

   **Command to run:**
   ```sh
   ros2 launch mapping maze_mapping_3d.launch.py
   ```

### SLAM Algorithms and Tools

1. **Occupancy Grid Mapping**
   - A technique that builds a 2D map by converting sensor data (like LiDAR) into a grid where each cell represents the probability of being occupied.

2. **RTAB-Map**
   - A real-time appearance-based mapping tool that can perform SLAM in 3D environments using RGB-D or stereo cameras.

3. **SLAM Toolbox**
   - A collection of tools for 2D SLAM, useful for real-time localization and mapping with 2D sensors like LiDAR.

### Commands to Run Launch Files
- **Bring up TurtleBot3 in a Maze Environment:**
  ```sh
  ros2 launch mapping maze_tb3_bringup.launch.py
  ```

- **Load and Visualize a 2D Map:**
  ```sh
  ros2 launch mapping map_loading_2d.launch.py
  ```

- **Perform 2D SLAM in a Maze:**
  ```sh
  ros2 launch mapping maze_mapping_2d.launch.py
  ## Next terminal run driving node
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  ```

- **Perform 3D SLAM in a Maze:**
  ```sh
  ## Requires Rtabmapping and change in turtlebot3 camera to depth
  ros2 launch mapping maze_mapping_3d.launch.py
  ```

### Test your Learnings with Assignments
- [Assignment](https://github.com/Robotisim/robotics_software_engineer/tree/assignments/module_7_assignment)

### CMakeLists Information

The CMakeLists.txt file for the `mapping` package includes the necessary configurations for building the package. Key components include:

- **Executables:**
  - `lidar_to_grid`
  - `occupancy_grid_pub`

Dependencies include `rclcpp`, `sensor_msgs`, `nav_msgs`, and `geometry_msgs`.
