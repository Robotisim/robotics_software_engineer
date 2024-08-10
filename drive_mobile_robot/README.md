## Introduction

The `drive_mobile_robot` package is designed to control mobile robots, particularly in simulated environments using ROS 2. This package includes nodes for driving a TurtleBot3 robot, as well as controlling turtles in a Turtlesim environment. It serves as an example of basic robot control in ROS 2, demonstrating how to publish and subscribe to command velocity topics to drive robots in different simulated environments.

## How to Run

**System Requirements**: ROS 2 Humble on Ubuntu 22.04 (tested).

To build the package:

```sh
colcon build --packages-select drive_mobile_robot
```

To source the setup file:

```sh
source install/setup.bash
```

### Running in Turtlesim

- **Launch a single TurtleBot3 driver in a Turtlesim environment:**
  ```sh
  ros2 launch drive_mobile_robot turtletbot_drive.launch.py
  ```

- **Launch Turtlesim with multiple turtles and control them:**
  ```sh
  ros2 launch drive_mobile_robot multi_turtle_drive.launch.py
  ```

### Running Multiple Nodes

- **Launch multiple nodes for controlling Turtlesim:**
  ```sh
  ros2 launch drive_mobile_robot multi_nodes.launch.py
  ```

**Note**: File names might have changed since the initial lectures, as this represents the final form of the package as it evolved by the last lecture.

### Test your Learnings with Assignments
- [Assignment](https://github.com/Robotisim/robotics_software_engineer/tree/assignments/module_2_assignment)

### Source Files

1. **drive_turtle.cpp**
   - **Purpose:** Implements a basic robot driver node for controlling a TurtleBot3 or a turtle in Turtlesim. This node subscribes to velocity commands (`/cmd_vel`) and drives the robot accordingly.
   - **Algorithm:** The node uses a simple control loop to listen to incoming velocity commands and applies them to the robot's actuators.

2. **publisher.cpp**
   - **Purpose:** Implements a ROS 2 node that publishes custom messages, which could be used to send control commands or status updates.

3. **subscriber.cpp**
   - **Purpose:** Implements a ROS 2 node that subscribes to specific topics, processing incoming messages such as velocity commands for the robot.

### Launch Files

1. **turtletbot_drive.launch.py**
   - **Purpose:** Launches a robot driver node for controlling the TurtleBot3 in a ROS 2 environment.
   - **Nodes Launched:**
     - `robot_driver` - A node that subscribes to `/cmd_vel` for velocity commands and drives the TurtleBot3.

2. **multi_nodes.launch.py**
   - **Purpose:** Starts the Turtlesim node and a custom turtle driver node to control a turtle within the Turtlesim environment.
   - **Nodes Launched:**
     - `turtlesim` - The Turtlesim node for simulation.
     - `turtle_driver` - The custom driver node to control the turtle.

3. **multi_turtle_drive.launch.py**
   - **Purpose:** Launches a Turtlesim environment, spawns a second turtle, and controls both turtles using separate nodes.
   - **Nodes Launched:**
     - `turtlesim` - The Turtlesim node for simulation.
     - `turtle_driver` - Drives the original turtle.
     - `turtle_driver2` - Drives the second spawned turtle.

## Package Dependencies

The `CMakeLists.txt` file specifies the necessary dependencies for building and running the `drive_mobile_robot` package:

- **Dependencies:**
  - `rclcpp` - ROS 2 C++ API for creating nodes, publishers, and subscribers.
  - `std_msgs` - Standard ROS 2 message types, used in the publisher and subscriber nodes.
  - `geometry_msgs` - ROS 2 message types for geometry data, including velocity commands for the robot.

These dependencies ensure that the nodes can publish and subscribe to the necessary topics for controlling the robots within the simulation environment.

