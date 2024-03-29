# This ROS 2 launch file starts up the Turtlesim node for simulation and a custom turtle driver node for controlling the turtle.
# The Turtlesim node provides a graphical interface for visualizing the turtle's movements.
# The turtle_driver node, part of the drive_mobile_robot package, issues commands to control the turtle's movement.
# Author: Robotisim

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define the Turtlesim node
    turtlesim = Node(
        package='turtlesim',  # Name of the package where the turtlesim_node is located
        executable='turtlesim_node',  # Name of the executable to run
        name='turtlesim'  # Name assigned to the node
    )

    # Define the turtle driver node
    turtle_driver = Node(
        package='drive_mobile_robot',  # Name of the custom package containing the driving logic
        executable='drive_turtle',  # Name of the executable to run
        name='turtle_driver'  # Name assigned to the node
    )

    # Return the LaunchDescription object, which includes both nodes
    return LaunchDescription([
        turtlesim,
        turtle_driver,
    ])
