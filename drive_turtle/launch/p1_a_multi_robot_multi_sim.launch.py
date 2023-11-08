"""
Author: Muhammad Luqman
Organization: Robotisim

This launch file is used to start two instances of the turtlesim_node, a simple simulator for ROS2.
These two nodes, turtlesim1 and turtlesim2, are placed in different namespaces, robot1 and robot2 respectively,
which allows them to operate independently within the ROS2 system.

Packages used:
- turtlesim
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ######### Running Turtlesim Node
    # Nodes for two turtlesim simulations
    turtlesim1 = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim1',
        namespace='robot1'
    )

    turtlesim2 = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim2',
        namespace='robot2'
    )

    return LaunchDescription([
        turtlesim1,
        turtlesim2,
    ])
