from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import  ExecuteProcess

def generate_launch_description():
    pkg_path = get_package_share_directory('control_systems')

    urdf_file = os.path.join(pkg_path, 'urdf','diff_drive_velocity_controller.urdf')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf_file]),

    #  Gazebo related stuff required to launch the robot in simulation
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='robot_spawner',
            output='screen',
            arguments=["-topic", "/robot_description", "-entity", "diff_drive_bot"]),
        Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=["joint_state_broadcaster"]),

        Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=["wheels_velocity_controller"]),



    ])