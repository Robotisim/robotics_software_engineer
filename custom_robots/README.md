## Custom Robots Package

The `custom_robots` package is designed for simulating custom robotic models within the ROS 2 environment. This package includes a differential drive robot model, which can be launched in both RViz and Gazebo for visualization and simulation purposes. It is an ideal starting point for anyone looking to create, simulate, and visualize custom robot models in a controlled environment.

#### Source Files
`Note`: File names might have changed since the initial lectures, as this represents the final form of the package as it evolved by the last lecture.
1. **static_transform.cpp**
   - **Purpose:** Implements a ROS 2 node for broadcasting static transforms between frames, essential for defining the robot's coordinate frames relative to each other.
   - **Key Functions:**
     - `void StaticTransform::broadcast()` - Publishes static transforms to the `/tf_static` topic.

#### URDF Files

1. **diff_drive.urdf**
   - **Purpose:** Defines the URDF (Unified Robot Description Format) model for the differential drive robot. This file contains the kinematic and dynamic properties of the robot, including links, joints, and sensors.
   - **Key Elements:**
     - `<link>` - Defines the physical parts of the robot, such as the chassis and wheels.
     - `<joint>` - Specifies the connections between different links, including the type of joint (e.g., revolute for wheels).

#### Launch Files

1. **rviz.launch.py**
   - **Purpose:** Launches a visualization environment for the differential drive robot using RViz2. This setup is ideal for testing and debugging the robot's URDF model.
   - **Nodes Launched:**
     - `robot_state_publisher` - Publishes the robot's state to the TF tree.
     - `joint_state_publisher_gui` - Allows interactive configuration of joint states.
     - `rviz2` - Provides a customizable visualization of the robot model.
   - **Command to run:**
   ```sh
   ros2 launch custom_robots rviz.launch.py
   ```

2. **gazebo.launch.py**
   - **Purpose:** Launches a Gazebo simulation environment with the differential drive robot. The robot is spawned in Gazebo using its URDF description, allowing for dynamic simulation and interaction.
   - **Nodes Launched:**
     - `robot_state_publisher` - Publishes the robot's state for use in simulation.
     - `joint_state_publisher` - Manages the robot's joint states for accurate simulation.
     - `gazebo_ros` - Spawns the robot in the Gazebo environment.
   - **Command to run:**
   ```sh
   ros2 launch custom_robots gazebo.launch.py
   ```

### CMakeLists Information

The `CMakeLists.txt` file is configured to build the `static_transform` node and install the necessary files:

- **Executables:**
  - `static_transform` - A ROS 2 node for broadcasting static transforms.

- **Installation:**
  - Installs the built executables and necessary directories (launch, URDF, config) into the appropriate locations within the ROS 2 workspace.

### Building and Running

To build the package:

```sh
colcon build --packages-select custom_robots
```

To source the setup file:

```sh
source install/setup.bash
```

### Example Commands

- **Visualize the Robot in RViz:**
  ```sh
  ros2 launch custom_robots rviz.launch.py
  ```

- **Simulate the Robot in Gazebo:**
  ```sh
  ros2 launch custom_robots gazebo.launch.py
  ```
### Test your Learnings with Assignments
- [Assignment](https://github.com/Robotisim/robotics_software_engineer/tree/assignments/module_3_assignment)