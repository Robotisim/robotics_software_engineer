"""
Author: Muhammad Luqman
Organization: Robotisim

This launch file starts a turtlesim_node, kills the first turtle, and then spawns two new turtles at different locations.

Packages used:
- turtlesim
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import TimerAction

def generate_launch_description():
    # Create a new LaunchDescription
    ld = LaunchDescription()

    ##### Starting the turtlesim_node
    # This node starts the turtlesim simulation.
    bring_sim = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim'
    )

    ##### Killing the first turtle
    # This command calls the /kill service to remove the first turtle from the simulation.
    kill_first = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/kill', 'turtlesim/srv/Kill', "\"{name: 'turtle1'}\"",],
        name='kill_turtle1',
        shell=True
    )

    ##### Spawning the first turtle
    # This command calls the /spawn service to create a new turtle at coordinates (3, 1) after a delay of 2 seconds.
    spawn_turtle_1 = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', "\"{x: 3.0, y: 1.0, theta: 1.57, name: 'turtle1'}\"",],
        name='spawn_turtle1',
        shell=True
    )

    spawn_turtle1_delayed = TimerAction(
            period=2.0,
            actions=[spawn_turtle_1],
        )

    ##### Spawning the second turtle
    # This command calls the /spawn service to create a second turtle at coordinates (2, 1).
    spawn_turtle_2 = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', "\"{x: 2.0, y: 1.0, theta: 1.57, name: 'turtle2'}\"",],
        name='spawn_turtle2',
        shell=True
    )

    # Add the nodes and actions to the LaunchDescription
    ld.add_action(bring_sim)
    ld.add_action(kill_first)
    ld.add_action(spawn_turtle1_delayed)
    ld.add_action(spawn_turtle_2)

    # Return the LaunchDescription
    return ld
