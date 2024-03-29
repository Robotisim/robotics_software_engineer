# This script generates a ROS 2 launch description for a differential drive robot simulation.
# It utilizes the robot_state_publisher and joint_state_publisher to load and publish the robot's URDF.
# Additionally, it launches a Gazebo simulation environment and spawns the robot within Gazebo.
# The URDF model is specified in the 'custom_robots' package under 'urdf/diff_drive.urdf'.
# Author: Robotisim

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Get the path to the 'custom_robots' package
    pkgPath = get_package_share_directory('custom_robots')

    # Specify the location of the URDF file within the package
    urdfFile = os.path.join(pkgPath, 'urdf', 'diff_drive.urdf')

    return LaunchDescription([
        # Node to publish the state of the robot to TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdfFile]),
        # Node to publish the joint states of the robot, helping in visualizing the robot in RViz
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            arguments=[urdfFile]),
        # Launch Gazebo with the ROS plugins required for simulation
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'),
        # Spawn the robot entity in Gazebo simulation from the URDF topic
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='robot_spawner',
            output='screen',
            arguments=["-topic", "/robot_description", "-entity", "diff_drive_bot"])
    ])
