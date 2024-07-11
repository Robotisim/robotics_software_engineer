# Purpose:
# - Launch a ROS2 robot simulation environment using Gazebo
# - Publish robot state and joint states from URDF file
# Author: Robotisim

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import ExecuteProcess

def generate_launch_description():
    pkgPath = get_package_share_directory('robot_sensing')

    urdfFile = os.path.join(pkgPath, 'urdf', 'diff_drive.urdf')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdfFile]),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            arguments=[urdfFile]),

        # Gazebo related stuff required to launch the robot in simulation
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='robot_spawner',
            output='screen',
            arguments=["-topic", "/robot_description", "-entity", "diff_drive_bot"])
    ])
