#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to the TurtleBot3 Gazebo launch file
    tb3_gazebo_launch_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    tb3_gazebo_launch_file = os.path.join(tb3_gazebo_launch_dir, 'empty_world.launch.py')

    # Node for the goal planner
    goal_planner_node = Node(
        package='control_systems',
        executable='lqr_node',
        name='multi_goal_follow'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tb3_gazebo_launch_file)
        ),
        goal_planner_node
    ])
