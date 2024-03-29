"""
Purpose: Launches a differential drive robot in Gazebo, spawns the robot model, and starts the necessary controllers.
Author: Robotisim
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import ExecuteProcess

def generate_launch_description():
    pkgPath = get_package_share_directory('control_systems')
    urdfFile = os.path.join(pkgPath, 'urdf', 'diff_drive_velocity_controller.urdf')

    return LaunchDescription([
        # Publishes state information of the robot based on URDF.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdfFile]),

        # Launch Gazebo with the ROS factory plugin to spawn entities.
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'),

        # Spawns the robot entity in Gazebo based on the URDF provided.
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='robot_spawner',
            output='screen',
            arguments=["-topic", "/robot_description", "-entity", "diff_drive_bot"]),

        # Starts the joint state broadcaster to publish joint states.
        Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=["joint_state_broadcaster"]),

        # Starts the controller for the wheels' velocity.
        Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=["wheels_velocity_controller"]),
    ])
