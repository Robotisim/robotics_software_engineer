"""
Author: Muhammad Luqman
Organization: Robotisim

This file launches two included launch files from the 'drive_turtle' package to initiate a simulation and start robot controllers.

Packages used:
- turtlesim
"""

#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ##### File Paths are stored
    # This line gets the path to the drive_turtle package.
    pkg_path = get_package_share_directory('drive_turtle')

    ##### Include the launch file to start the simulation
    # This line includes the launch file p1_b_multi_robot_single_sim.launch.py from the drive_turtle package.
    bring_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'p1_b_multi_robot_single_sim.launch.py')
        ),
    )

    ##### Include the launch file to start the robot controllers
    # This line includes the launch file p1_c_multi_robot_controller.launch.py from the drive_turtle package.
    bring_robots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'p1_c_multi_robot_controller.launch.py')
    ),
    )

    ld = LaunchDescription()

    # Add the IncludeLaunchDescription actions to the LaunchDescription
    ld.add_action(bring_sim)
    ld.add_action(bring_robots)

    # Return the LaunchDescription
    return ld
