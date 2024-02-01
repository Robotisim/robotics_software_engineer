"""
Author: Muhammad Luqman
Organization: Robotisim

This launch file starts two controllers for two turtles. Each controller drives its turtle at a different speed.
Packages used:
- turtlesim
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    ##### Controller for turtle1
    # This node runs the p1_multi_turtle_controller executable from the drive_turtle package.
    # It controls the turtle named 'turtle1', driving it at a speed of 2.0.
    turtle1_controller = Node(
        package='drive_turtle',
        executable='p1_multi_turtle_controller',
        name='turtle1_controller',
        parameters=[
            {'cmd_vel_topic': '/turtle1/cmd_vel'},
            {'linear_velocity': 2.5}
        ]
    )

    ##### Controller for turtle2
    # This node runs the p1_multi_turtle_controller executable from the drive_turtle package.
    # It controls the turtle named 'turtle2', driving it at a speed of 1.5.
    turtle2_controller = Node(
        package='drive_turtle',
        executable='p1_multi_turtle_controller',
        name='turtle2_controller',
        parameters=[
            {'cmd_vel_topic': '/turtle2/cmd_vel'},
            {'linear_velocity': 1.5}
        ]
    )

    # Add the controllers to the LaunchDescription
    ld.add_action(turtle1_controller)
    ld.add_action(turtle2_controller)

    # Return the LaunchDescription
    return ld
