"""
Author: Muhammad Luqman
Organization: Robotisim

This launch file initiates a Gazebo simulation, spawning a TurtleBot3 model at specified coordinates
using gazebo_ros and turtlebot3_gazebo packages.

Packages used:

- gazebo_ros
- turtlebot3_gazebo

Arguments used:

- x_pos: The x-coordinate where the TurtleBot3 model should be spawned in the Gazebo world.
- y_pos: The y-coordinate where the TurtleBot3 model should be spawned in the Gazebo world.
- yaw_rot: The yaw rotation of the TurtleBot3 model when it is spawned.
- robot_name: The name of the TurtleBot3 model to be spawned.
- robot_ns: The namespace into which the TurtleBot3 model should be spawned.
"""

#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import  LaunchConfiguration
from launch_ros.actions import  Node


def generate_launch_description():
    ##### File Paths are stored
    # This section retrieves the path of the TurtleBot3 model based on the environment variable 'TURTLEBOT3_MODEL'.
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf'
    )

    ##### Define arguments - parameters
    # These lines define the parameters for the TurtleBot3 model's spawn location and its namespace.
    x_pos = LaunchConfiguration('x_pos',default='0.0')
    y_pos = LaunchConfiguration('y_pos',default='0.0')
    yaw_rot = LaunchConfiguration('yaw_rot',default='0.0')
    robot_name = LaunchConfiguration('robot_name',default='waffle_pi')
    robot_ns = LaunchConfiguration('robot_ns',default='')

    ##### Spawn the TurtleBot3 model at the specified location
    # This section spawns the TurtleBot3 model at the specified location in the Gazebo simulation.
    start_gazebo_ros_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name,
            '-file', urdf_path,
            '-x', x_pos,
            '-y', y_pos,
            '-z', '0.01',
            '-Y',yaw_rot,
            '-robot_namespace',robot_ns
        ],
        output='screen',
    )

    ld = LaunchDescription()

    ##### Add the commands to the launch description
    # This line adds the spawn command to the launch description.
    ld.add_action(start_gazebo_ros_spawner_cmd)

    # Return the LaunchDescription
    return ld
