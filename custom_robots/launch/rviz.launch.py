# Launches a visualization environment for a differential drive robot using RViz2 in ROS 2.
# It loads and publishes the robot's URDF file to the robot_state_publisher,
# and uses joint_state_publisher_gui for interactive joint state configuration.
# The RViz2 configuration is loaded from a specified file to customize the visualization.
# This setup is ideal for visual testing and debugging of robot models.
# Author: Robotisim

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the 'custom_robots' package
    pkgPath = get_package_share_directory('custom_robots')
    # Define the path to the RViz2 configuration file
    rvizConfigFilePath = os.path.join(pkgPath, 'config', 'urdf_view.rviz')
    # Define the path to the URDF file
    urdfFile = os.path.join(pkgPath, 'urdf', 'diff_drive.urdf')

    return LaunchDescription([
        # Node to publish the robot's URDF to the robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdfFile]),
        # Node for interactive joint state publishing using a GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher',
            arguments=[urdfFile]),
        # Node to launch RViz2 with the specified configuration file
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rvizConfigFilePath]),
    ])
