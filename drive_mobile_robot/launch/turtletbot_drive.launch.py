# Launches a robot driver node for controlling the TurtleBot3 in a ROS 2 environment.
# - Utilizes the `drive_mobile_robot` package for robot control logic.
# - Configures the robot driver node to subscribe to `/cmd_vel` for velocity commands.
# Author: Robotisim

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():

    turtlebot3_driver=Node(
        package='drive_mobile_robot',
        executable='drive_turtle',
        name='robot_driver',
        parameters=[
            {'cmd_vel_topic': '/cmd_vel'},
        ]
    )

    return LaunchDescription([
        turtlebot3_driver,

    ])
