#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription , ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro

def generate_launch_description():

    drone_pkg = get_package_share_directory('sjtu_drone_bringup')
    tb3_pkg = get_package_share_directory('turtlebot3_gazebo')




    drone_bringup= IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(drone_pkg, 'launch', 'sjtu_drone_bringup.launch.py')
            ),

        )

    tb3_bringup= IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_pkg, 'launch', 'spawn_turtlebot3.launch.py')
            ),

        )

    takeoff_cmd= ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '/drone/takeoff', 'std_msgs/msg/Empty', '{}','--once'],
            shell=True,
            output="screen"
        )


    follow_tb3_vision= Node(
            package="drive_drone",
            executable="p1_b_tb3_follower",
            output="screen"
        )
    drive_tb3= Node(
            package="teleop_twist_keyboard",
            executable="teleop_twist_keyboard",
            output="screen",
            prefix="xterm -e"
        )

    nodes_to_run = [
        drone_bringup,
        takeoff_cmd,
        follow_tb3_vision,
        tb3_bringup,
        drive_tb3


    ]

    return LaunchDescription(nodes_to_run)