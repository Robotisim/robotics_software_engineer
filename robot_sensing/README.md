## Introduction

The `robot_sensing` package is designed to simulate and control robots equipped with various sensors, particularly in environments like mazes and line-following tracks. This package provides launch files and nodes for maze solving using LIDAR and line following using a camera sensor. It serves as an educational tool for understanding how sensors can be used to guide robots in complex environments using ROS 2.

## How to Run

**System Requirements**: ROS 2 Humble on Ubuntu 22.04 (tested).

To build the package:

```sh
colcon build --packages-select robot_sensing
```

To source the setup file:

```sh
source install/setup.bash
```

### Running Simulations

- **Maze Solving with LIDAR:**
  ```sh
  ros2 launch robot_sensing lidar_maze_solving.launch.py
  ```

- **Line Following with Camera:**
  ```sh
  ros2 launch robot_sensing camera_line_following.launch.py
  ```

- **Custom Sensor Setup in Gazebo:**
  ```sh
  ros2 launch robot_sensing custom_sensors.launch.py
  ```

**Note**: File names might have changed since the initial lectures, as this represents the final form of the package as it evolved by the last lecture.

## Source Files and Launch Files

### Source Files

1. **line_following_with_camera.cpp**
   - **Purpose:** Implements the logic for a robot to follow a line using input from a camera sensor. The algorithm processes the camera feed to detect the line and generate velocity commands to keep the robot on track.
   - **Algorithm:** Uses computer vision techniques to identify the line and a proportional controller to adjust the robot's steering based on the line's position relative to the robot.

2. **maze_solving_with_lidar.cpp**
   - **Purpose:** Implements a maze-solving algorithm that uses LIDAR data to navigate through a maze. The robot scans its environment and makes decisions based on detected obstacles.
   - **Algorithm:** The robot uses a wall-following algorithm, combined with LIDAR data, to systematically explore and find the exit of the maze.

### Launch Files

1. **camera_line_following.launch.py**
   - **Purpose:** Launches a simulation environment for line following using a camera in Gazebo.
   - **Nodes Launched:**
     - `robot_state_publisher` - Publishes the robot's URDF to the TF tree.
     - `gazebo_ros` - Spawns the robot in a Gazebo world configured for line following.
     - `line_following` - The node that controls the robot to follow the line based on camera input.

2. **lidar_maze_solving.launch.py**
   - **Purpose:** Launches a maze-solving simulation using LIDAR data in Gazebo.
   - **Nodes Launched:**
     - `robot_state_publisher` - Publishes the robot's URDF to the TF tree.
     - `gazebo_ros` - Spawns the robot in a Gazebo world configured as a maze.
     - `maze_solver` - The node that drives the robot through the maze using LIDAR input.

3. **custom_sensors.launch.py**
   - **Purpose:** Launches a custom sensor setup in Gazebo, allowing for the simulation of different sensors on the robot.
   - **Nodes Launched:**
     - `robot_state_publisher` - Publishes the robot's URDF to the TF tree.
     - `joint_state_publisher` - Publishes joint states for the robot in Gazebo.
     - `gazebo_ros` - Spawns the robot in Gazebo with custom sensor configurations.

### Package Dependencies

The `CMakeLists.txt` file specifies the necessary dependencies for building and running the `robot_sensing` package:

- **Dependencies:**
  - `rclcpp` - ROS 2 C++ API for creating nodes, publishers, and subscribers.
  - `sensor_msgs` - ROS 2 message types for handling sensor data.
  - `geometry_msgs` - ROS 2 message types for geometry data, including velocity commands for the robot.
  - `gazebo_ros` - Integration with the Gazebo simulator for launching and controlling the robot in a simulated environment.

### Test your Learnings with Assignments
- [Assignment](https://github.com/Robotisim/robotics_software_engineer/tree/assignments/module_4_assignment)