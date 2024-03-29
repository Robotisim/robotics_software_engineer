# Launches a Turtlesim environment, spawns a second turtle, and controls both turtles.
# - Uses the turtlesim package to launch a basic turtle simulator.
# - Spawns a second turtle in the simulator.
# - Drives the original and the second turtle with separate nodes.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Node configuration for launching the Turtlesim node
    turtlesimNode = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
    )

    # ExecuteProcess used to spawn a second turtle in the Turtlesim environment
    spawnTurtle2 = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', "\"{x: 2.0, y: 1.0, theta: 1.57, name: 'turtle2'}\""],
        name='spawn_turtle2',
        shell=True
    )

    # Node configuration for driving the original turtle
    turtleDriver = Node(
        package='drive_mobile_robot',
        executable='drive_turtle',
        name='turtle_driver',
        # parameters=[{'cmd_vel_topic': '/turtle1/cmd_vel'}]
    )

    # Node configuration for driving the second spawned turtle
    turtleDriver2 = Node(
        package='drive_mobile_robot',
        executable='drive_turtle',
        name='turtle_driver2',
        parameters=[{'cmd_vel_topic': '/turtle2/cmd_vel'}]
    )

    return LaunchDescription([
        turtlesimNode,
        spawnTurtle2,
        turtleDriver,
        turtleDriver2,
    ])
