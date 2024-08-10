## C++ Fundamentals Package

The `cpp_fundamentals` package is designed to provide a comprehensive introduction to essential C++ programming concepts, particularly in the context of robotics. This package includes various example programs that cover classes, functions, pointers, templates, namespaces, and more, all applied to robotics scenarios. The package also includes a small project on line following using C++.

#### Source Files
`Note`: File names might have changed since the initial lectures, as this represents the final form of the package as it evolved by the last lecture.
1. **class_robot.cpp**
   - **Purpose:** Demonstrates the use of C++ classes by modeling a robot with various attributes and behaviors encapsulated within a class.

2. **functions_sensor_reading.cpp**
   - **Purpose:** Illustrates the use of functions in C++ by simulating sensor readings and processing them in a robotic context.

3. **lib_line_following.cpp / lib_line_following.h**
   - **Purpose:** Implements a line-following algorithm using C++ libraries and demonstrates how to organize code using header and source files.
   - **Key Functions:**
     - `void LineFollowing::follow_line()` - Core algorithm for following a line using sensor inputs.

4. **lib_main.cpp**
   - **Purpose:** The main entry point for the line-following robot application, integrating various functions and classes.

5. **namespaces_sensor_processing.cpp**
   - **Purpose:** Explains the use of namespaces in C++ by organizing sensor processing functions within a namespace, avoiding naming conflicts.

6. **pointers_robot_location.cpp**
   - **Purpose:** Demonstrates the use of pointers in C++, particularly in tracking and updating the robot's location.

7. **ros2_node.cpp**
   - **Purpose:** Shows how to integrate C++ code with ROS 2 by creating a simple ROS 2 node that interfaces with a robot.

8. **template_robot.cpp**
   - **Purpose:** Demonstrates the use of C++ templates by creating a generic robot class that can handle various data types and operations.

### CMakeLists Information

The `CMakeLists.txt` file is configured to build multiple executables, each corresponding to a specific example in the `cpp_fundamentals` package. It links necessary libraries such as Eigen3, OpenCV, and Boost, which are commonly used in robotics:

- **Executables:**
  - `lib_lf` - Demonstrates the line-following algorithm.
  - `class_r` - Example using C++ classes.
  - `function_sr` - Example focusing on functions and sensor readings.
  - `namespaces_rp` - Example using namespaces for organizing code.
  - `pointer_r` - Example demonstrating pointers in C++.
  - `template_r` - Example using templates for generic programming.

### Building and Running

To build the package:

```sh
colcon build --packages-select cpp_fundamentals
```

To source the setup file:

```sh
source install/setup.bash
```
### Test your Learnings with Assignments
- [Assignment](https://github.com/Robotisim/robotics_software_engineer/tree/assignments/module_1_assignment)

### Example Commands

- **Run the Line Following Algorithm:**
  ```sh
  ./lib_lf
  ```

- **Run the Class Example:**
  ```sh
  ./class_r
  ```

- **Run the Function Example:**
  ```sh
  ./function_sr
  ```

- **Run the Namespace Example:**
  ```sh
  ./namespaces_rp
  ```

- **Run the Pointer Example:**
  ```sh
  ./pointer_r
  ```

- **Run the Template Example:**
  ```sh
  ./template_r
  ```
