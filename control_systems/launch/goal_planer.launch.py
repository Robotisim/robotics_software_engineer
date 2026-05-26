#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    start_rqt = LaunchConfiguration('start_rqt')

    sim_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('custom_robots'), 'launch', 'gazebo.launch.py')
        )
    )

    goal_planner_node = Node(
        package='control_systems',
        executable='goal_planer',
        name='goal_planer'
    )

    rqt_gui_command = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_gui', 'rqt_gui'],
        output='screen',
        condition=IfCondition(start_rqt)
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_rqt', default_value='false'),
        sim_bringup,
        goal_planner_node,
        rqt_gui_command
    ])
