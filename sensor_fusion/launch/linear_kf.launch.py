#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    rviz_config_path = os.path.join(get_package_share_directory('sensor_fusion'), 'config', 'sensor_fusion.rviz')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('custom_robots'), 'launch', 'gazebo.launch.py')
            )
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        ),
        Node(
            package='sensor_fusion',
            executable='linear_kf',
            name='linear_kalman_filter',
            output='screen'
        )
    ])
