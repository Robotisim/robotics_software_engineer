# Robotics Software Engineer Learning Path Assignments

This repository contains the assignments for the Robotics Software Engineer learning path. Each assignment corresponds to a module from the [Robotics Software Engineer course](https://github.com/Robotisim/robotics_software_engineer), designed to reinforce the concepts and skills covered in the lectures.

## Assignment Structure

The assignments are organized into separate modules, each focusing on a specific aspect of robotics software development. Below is a brief description of each module and a link to the corresponding assignment package:

### Module 1: Introduction to OOP Concepts in C++ for Robotics

- **Description:** This module introduces basic Object-Oriented Programming (OOP) concepts using C++ within a robotics context. You will develop a Robot class, simulate sensor readings, and create a simple sensor library.
- **Assignment:** [Module 1 Assignment](module_1_assignment/Readme.md)

### Module 2: Developing Custom ROS 2 Nodes and Launch Files

- **Description:** This module focuses on developing custom ROS 2 nodes and utilizing launch files to manage node execution. You will create nodes for Turtlesim and explore the use of ROS 2 services and parameters.
- **Assignment:** [Module 2 Assignment](module_2_assignment/Readme.md)

### Module 3: Concepts of URDF and Robot Creation

- **Description:** In this module, you will learn how to create and manipulate robot models using URDF (Unified Robot Description Format). The tasks include defining transform trees, adding joints, and building a mobile manipulator.
- **Assignment:** [Module 3 Assignment](module_3_assignment/Readme.md)

### Module 4: Sensor Data Manipulation for Robot Control

- **Description:** This module teaches how to use sensor data for controlling robots. You will refactor a line-following project, design software flow for maze solving, and implement a maze environment for TurtleBot3.
- **Assignment:** [Module 4 Assignment](module_4_assessment/module_4_assignment/Readme.md)

### Module 5: Enhancing Robot Control for Smooth Motion

- **Description:** The focus of this module is on improving robot control to achieve smooth motion. You will enhance a camera-based line-following system, optimize TurtleBot3's path to minimize energy consumption, and add a position controller to a robotic arm URDF.
- **Assignment:** [Module 5 Assignment](module_5_assignment/Readme.md)

### Module 6: IMU and GPS Sensor Fusion for TurtleBot3

- **Description:** This module covers the implementation and analysis of sensor fusion using an Extended Kalman Filter (EKF) to fuse IMU and GPS data for TurtleBot3. You will visualize the fused data and experiment with different noise covariance matrices.
- **Assignment:** [Module 6 Assignment](module_6_assignment/Readme.md)

### Module 7: ROS 2 Mapping with SLAM

- **Description:** This module focuses on mapping with ROS 2, where you will create a 2D LIDAR-based map, explore the required inputs and outputs for 2D and 3D mapping, and explain the mapping algorithm (Gmapping) in your own words.
- **Assignment:** [Module 7 Assignment](module_7_assignment/Readme.md)

### Module 8: Path Planning with A* and RRT

- **Description:** In this module, you will compare A* and RRT path planning algorithms, improve RRT to RRT*, and write unit tests for RRT* based on the maze created in Module 7.
- **Assignment:** [Module 8 Assignment](module_8_assignment/Readme.md)

## Getting Started

To begin working on these assignments, clone this repository and navigate to the relevant module's package. Each module contains its own `CMakeLists.txt` and `package.xml` for building the package in a ROS 2 workspace.

```bash
git clone https://github.com/Robotisim/robotics_software_engineer.git -b assignments
cd robotics_software_engineer
```

## Learning Path

These assignments are designed to complement the [Robotics Software Engineer course](https://github.com/Robotisim/robotics_software_engineer), providing hands-on experience with the concepts discussed in the lectures. Completing these assignments will help solidify your understanding of robotics software engineering and prepare you for more advanced topics.

## License
This work is licensed under a Creative Commons Attribution-NonCommercial 4.0 International License. This license allows others to remix, tweak, and build upon the work non-commercially, as long as they credit the owner (robotisim) and license their new creations under the identical terms.

![Creative Commons License](https://i.creativecommons.org/l/by-nc/4.0/88x31.png)

This license is acceptable for Free Cultural Works.

For more information, please visit [Creative Commons License](http://creativecommons.org/licenses/by-nc/4.0/).

---

© 2024 robotisim. All rights reserved.
