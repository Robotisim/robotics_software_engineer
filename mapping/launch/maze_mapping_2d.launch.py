#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    env_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join( get_package_share_directory('mapping'), 'launch', 'maze_tb3_bringup.launch.py')
        ),
    )


    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join( get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
    )


    ld = LaunchDescription()

    ld.add_action(env_bringup)
    ld.add_action(mapping)
    return ld