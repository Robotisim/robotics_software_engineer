# These assignment-Tasks are not yet Finilized

## Assignments for Module #8 : Behaviour Trees
- Create all files in *module_8_assignment* package

### Assignment 1: Behavior Tree for Object-Detection-Triggered Navigation
Objective
Develop a Behavior Tree in ROS2 that controls a robot's navigation actions based on real-time image processing results. The robot should move to a predefined goal when a ball is detected and return to its starting position if a square is detected.

**Tasks**
Image Processing Node Setup

Implement a ROS2 node that subscribes to the robot's camera topic.
Use OpenCV or another image processing library to detect balls and squares in the camera feed. You can apply color thresholding, contour detection, or shape recognition algorithms.
Publish detection results (type and location of detected object) on a custom topic.
Behavior Tree Creation

Design a Behavior Tree that listens to the image processing node's output.
Include conditions/actions in the tree for:
Moving to a predefined goal (x, y coordinates) when a ball is detected.
Returning to the starting position when a square is detected.
Utilize the Navigation2 package for movement actions, configuring it as necessary for your robot and environment.
Integration and Testing

Integrate the Behavior Tree with the image processing node in a ROS2 simulation environment.
Test the system in a virtual environment where both a ball and a square can be presented to the robot's camera in various scenarios.
Must use backports.