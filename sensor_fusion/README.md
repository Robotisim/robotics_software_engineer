## Introduction

The `sensor_fusion` package is a comprehensive tool designed for teaching and implementing sensor fusion techniques in robotics. The package includes practical implementations of 1D and 2D Kalman Filters as well as the Extended Kalman Filter (EKF). These algorithms are crucial for combining data from various sensors to accurately estimate the state of a robot, which is essential in applications such as localization, navigation, and control.

## How to Run

**System Requirements**: ROS 2 Humble on Ubuntu 22.04 (tested).

To build the package:

```sh
colcon build --packages-select sensor_fusion
```

To source the setup file:

```sh
source install/setup.bash
```

### Running Simulations

- **EKF with KITTI Dataset:**
  ```sh
  ros2 launch sensor_fusion kitti_data_run.launch.py
  ```
  - **Download KITTI Dataset:**
      ```sh
      # Navigate to the directory containing the bash script (raw_data_downloader.sh)
      chmod +x raw_data_downloader.sh
      ./raw_data_downloader.sh
      ```
  - **Arrange KITTI Data in your workspace:**
      Ensure your workspace has the following structure:
      ```
      ros2_ws
         ├── data
         └── 2011_09_26
             ├── 2011_09_26_drive_0001_sync
             │   ├── image_00
             │   │   ├── data ...
      ```
  - **Run KITTI Data Sample:**
      ```sh
      ros2 run ros2_kitti_publishers kitti_publishers
      ```
  - **Run IMU and GPS Data Fusion Node:**
      ```sh
      ros2 run sensor_fusion ekf_imu_gps
      ```

- **Kalman Filter with TurtleBot3 in Gazebo:**
  ```sh
  ros2 launch sensor_fusion linear_kf.launch.py
  # In a new terminal, run:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  ```

- **EKF with TurtleBot3 IMU and Odometry:**
  ```sh
  ros2 launch sensor_fusion ekf_tb3_imu_odom.launch.py
  ```

**Note**: File names might have changed since the initial lectures, as this represents the final form of the package as it evolved by the last lecture.

### Test your Learnings with Assignments
- [Assignment](https://github.com/Robotisim/robotics_software_engineer/tree/assignments/module_6_assignment)


### Source Files

1. **ekf_lib.cpp / ekf_lib.hpp**
   - **Purpose:** Implements the core logic for the Extended Kalman Filter (EKF) algorithm, allowing for sensor fusion in non-linear systems.
   - **Algorithm:** The EKF is used to estimate the state of a robot by fusing data from multiple sensors, such as IMU and GPS.

2. **ekf_node.cpp**
   - **Purpose:** A ROS 2 node that uses the `ekf_lib` to perform sensor fusion with the EKF. This node is designed to integrate and process sensor data in real-time, producing accurate state estimates.

3. **linear_kf.cpp**
   - **Purpose:** Implements a node for 1D or 2D Kalman Filter algorithms, suitable for linear state estimation tasks in robotics.

### Launch Files

1. **ekf_tb3_imu_odom.launch.py**
   - **Purpose:** Launches the EKF node for fusing IMU and odometry data from a TurtleBot3 robot. This configuration is useful for accurate state estimation in environments where both IMU and odometry data are available.
   - **Nodes Launched:**
     - `ekf_filter_node` - Performs sensor fusion using the EKF algorithm.

2. **kitti_data_run.launch.py**
   - **Purpose:** Sets up the environment for running sensor fusion with data from the KITTI dataset, using RViz for visualization and the EKF node for processing.
   - **Nodes Launched:**
     - `rviz2` - Visualizes the data and state estimates.
     - `kitti_publishers` - Publishes KITTI dataset data to the ROS 2 environment.
     - `ekf_node` - Runs the EKF algorithm on the KITTI data.

3. **linear_kf.launch.py**
   - **Purpose:** Launches the Kalman Filter node in a TurtleBot3 simulation environment within Gazebo, along with RViz for visualization. This setup is ideal for understanding how linear Kalman Filters operate in a simulated robotic environment.
   - **Nodes Launched:**
     - `linear_kalman_filter` - Runs the linear Kalman Filter for state estimation.
     - `rviz2` - Provides visualization for the sensor fusion process.

## Package Dependencies

The `CMakeLists.txt` file specifies the necessary dependencies for building and running the `sensor_fusion` package:

- **Dependencies:**
  - `rclcpp` - ROS 2 C++ API for creating nodes, publishers, and subscribers.
  - `std_msgs` - Standard ROS 2 message types, used in the publisher and subscriber nodes.
  - `sensor_msgs` - ROS 2 message types for handling sensor data.
  - `nav_msgs` - ROS 2 message types for navigation data, such as odometry.
  - `visualization_msgs` - ROS 2 message types for visualization, such as markers in RViz.
  - `Eigen3` - A C++ template library for linear algebra, used in the EKF and Kalman Filter implementations.
  - `gazebo_ros` - Integration with the Gazebo simulator for launching and controlling the robot in a simulated environment.

This package is an excellent resource for learning and applying sensor fusion techniques in robotics. It offers practical examples and configurations that can be directly used or adapted for real-world applications.