## Control Systems Package

This package is part of the robotics curriculum of RSE aimed at teaching various control algorithms using ROS 2 and TurtleBot3 simulations. The package includes implementations of different control algorithms, including a proportional controller.

### Overview

The `control_systems` package contains the following key files and functionalities:
### Algorithms Taught

1. **Proportional Control**
   - A basic feedback control algorithm that adjusts the control input proportionally to the error.

2. **Goal Planning**
   - Planning a trajectory to achieve a goal position in a 2D space.

3. **Linear Quadratic Regulator (LQR)**
   - An optimal control algorithm that minimizes a cost function representing the state and control effort.


### Commands to Run on your system

- To build the package:
  ```sh
  colcon build --packages-select control_systems
  ```

- To source the setup file:
  ```sh
  source install/setup.bash
  ```

- To run the goal planner:
  ```sh
  ros2 launch control_systems goal_planer.launch.py
  ```

- To run the velocity controller:
  ```sh
  ros2 launch control_systems velocity_controller.launch.py
  ```
- To run the Multi Goal Following using LQR
  ```sh
  ros2 launch  control_systems lqr_multi_goals.launch.py
  ```
### Test your Learnings with Assignments
- [Assignment](https://github.com/Robotisim/robotics_software_engineer/tree/assignments/module_5_assignment)

#### Source Files
**Note**: File names might have changed since the initial lectures, as this represents the final form of the package as it evolved by the last lecture.

1. **goal_linear.cpp**
   - Implements the logic for achieving a linear goal.

2. **goal_planer.cpp**
   - Implements the logic for a goal planner in a 2D space.

3. **lqr_lib.cpp**
   - Contains the library for the Linear Quadratic Regulator (LQR) algorithm.

4. **lqr_node.cpp**
   - Implements the ROS node for executing the LQR control algorithm.

5. **velocity_controller_drive.cpp**
   - Implements a velocity controller using *ros2_control* for driving the robot.

Dependencies are defined and linked, including `rclcpp`, `tf2_ros`, `geometry_msgs`, `nav_msgs`, `angles`, and `Eigen3`.


