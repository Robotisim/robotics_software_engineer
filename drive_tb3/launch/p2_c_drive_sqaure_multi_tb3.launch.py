"""
Author: Muhammad Luqman
Organization: Robotisim

This launch file includes another launch file to spawn multiple TurtleBot3 models and starts four nodes each driving a
 distinct TurtleBot3 model.

Arguments used:
- world: Full path to the Gazebo world file to load.
- x_pos: The x-coordinate where the TurtleBot3 model should be spawned in the Gazebo world.
- y_pos: The y-coordinate where the TurtleBot3 model should be spawned in the Gazebo world.
- yaw_rot: The yaw rotation of the TurtleBot3 model when it is spawned.
- robot_name: The name of the TurtleBot3 model to be spawned.
- robot_ns: The namespace into which the TurtleBot3 model should be spawned.

Packages used:
- gazebo_ros: This package provides nodes and plugins necessary to run, interface and interact with Gazebo simulations from ROS2. In this launch file, it is used to start the Gazebo server and client.
- turtlebot3_gazebo: This package contains the Gazebo simulation environments for TurtleBot3. In this launch file, it is used to retrieve the path of the Gazebo world file.
- drive_tb3: This is a custom package that is likely to contain launch files and nodes necessary to control the TurtleBot3 models. In this launch file, it is used to spawn the TurtleBot3 models at specified locations in the Gazebo simulation.

"""

#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import  Node


def generate_launch_description():
    ##### File Paths are stored
    # This section retrieves the path of the drive_tb3 package.
    pkg_tb3_drive = get_package_share_directory('drive_tb3')

    ##### Include the launch file to spawn multiple TurtleBot3 models
    # This section includes the p2_b_multi_tb3.launch.py launch file from the drive_tb3 package to spawn multiple TurtleBot3 models.
    spawn_multi_tb3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_drive, 'launch', 'p2_b_multi_tb3.launch.py')
        ),
    )

    ##### Start the node to drive the first TurtleBot3 model
    # These sections start four nodes to drive each of the TurtleBot3 models. The cmd_vel_topic parameter is set to the command velocity topic of each robot.
    drive_tb3_a = Node(
        package='drive_tb3',
        executable='p2_multi_tb3_sqaure_drive',
        name='drive_tb3_a',
        parameters=[
            {'cmd_vel_topic': '/robot_a/cmd_vel'},
        ]
    )

    drive_tb3_b = Node(
        package='drive_tb3',
        executable='p2_multi_tb3_sqaure_drive',
        name='drive_tb3_b',
        parameters=[
            {'cmd_vel_topic': '/robot_b/cmd_vel'},
        ]
    )

    drive_tb3_c = Node(
        package='drive_tb3',
        executable='p2_multi_tb3_sqaure_drive',
        name='drive_tb3_c',
        parameters=[
            {'cmd_vel_topic': '/robot_c/cmd_vel'},
        ]
    )

    drive_tb3_d = Node(
        package='drive_tb3',
        executable='p2_multi_tb3_sqaure_drive',
        name='drive_tb3_d',
        parameters=[
            {'cmd_vel_topic': '/robot_d/cmd_vel'},
        ]
    )

    ld = LaunchDescription()

    ##### Add the commands to the launch description
    # This section adds all the IncludeLaunchDescription and Node actions to the launch description.
    ld.add_action(spawn_multi_tb3)
    ld.add_action(drive_tb3_a)
    ld.add_action(drive_tb3_b)
    ld.add_action(drive_tb3_c)
    ld.add_action(drive_tb3_d)

    # Return the LaunchDescription
    return ld
