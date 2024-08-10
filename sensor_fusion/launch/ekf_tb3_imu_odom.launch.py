#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    pkg_share_dir = get_package_share_directory("sensor_fusion")
    parameters_file_path = os.path.join(pkg_share_dir,"config", "ekf_tb3_imu_odom.yaml")
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[parameters_file_path],
        )
    ])
