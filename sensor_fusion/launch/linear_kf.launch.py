#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Paths
    turtlebot3_gazebo_path = get_package_share_directory('turtlebot3_gazebo')
    rviz_config_path = os.path.join(get_package_share_directory('sensor_fusion'), 'config', 'sensor_fusion.rviz')

    return LaunchDescription([
        # Launch Gazebo with empty world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot3_gazebo_path, 'launch', 'empty_world.launch.py')
            )
        ),
        # Launch RViz2 with a specific configuration file
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        ),
        # Launch a_x_estimation node
        Node(
            package='sensor_fusion',
            executable='linear_kf',
            name='linear_kalman_filter',
            output='screen'
        )
    ])
