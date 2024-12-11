#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory for module_6_assignment
    pkg_share = get_package_share_directory('module_6_assignment')
    
    # Path to the EKF parameter file
    parameters_file_path = os.path.join(pkg_share, 'config', 'ekf_params.yaml')
    
    return LaunchDescription([
        # Log information that the launch file is being loaded
        LogInfo(msg="Starting EKF Fusion Launch File..."),
        
        # Launch the EKF node (using your custom EKF node)
        Node(
            package='module_6_assignment',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[parameters_file_path],
            remappings=[('/imu', '/imu'),
                        ('/gps', '/gps'),
                        ('/fused_odom', '/odom')]  # Assuming the EKF node publishes to /odom
        ),
            # Launch rviz2 node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
             output='screen',
    )
    ])
