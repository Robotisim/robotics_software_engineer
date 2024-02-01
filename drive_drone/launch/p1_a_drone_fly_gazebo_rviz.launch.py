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





    drone_bringup= IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(drone_pkg, 'launch', 'sjtu_drone_bringup.launch.py')
            ),

        )

    takeoff_cmd= ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '/drone/takeoff', 'std_msgs/msg/Empty', '{}','--once'],
            shell=True,
            output="screen"
        )

    drive_drone= Node(
            package="drive_drone",
            executable="p1_a_drive_node",
            output="screen"
        )

    nodes_to_run = [
        drone_bringup,
        takeoff_cmd,
        drive_drone


    ]

    return LaunchDescription(nodes_to_run)