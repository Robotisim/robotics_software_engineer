#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    sim_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('custom_robots'), 'launch', 'gazebo.launch.py')
        )
    )

    goal_planner_node = Node(
        package='control_systems',
        executable='lqr_node',
        name='multi_goal_follow'
    )

    return LaunchDescription([
        sim_bringup,
        goal_planner_node
    ])
