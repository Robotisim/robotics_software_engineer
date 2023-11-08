"""
Author: Muhammad Luqman
Organization: Robotisim

This launch file includes the 'empty_world.launch.py' file to spawn a TurtleBot3 model in an empty world, and starts two nodes:
1. The 'rqt_reconfigure' node, which provides a GUI for dynamically reconfiguring parameters.
2. The 'p3_d_xy_goal' node from the 'drive_tb3' package, which drives the TurtleBot3 model.



Packages used:
- turtlebot3_gazebo: This package is used to spawn a TurtleBot3 model in an empty world. The launch file empty_world.launch.py is specifically included from this package.
- rqt_reconfigure: This package provides a node that is used to set up a GUI for dynamic reconfiguration of parameters.
- drive_tb3: This is a custom package likely containing nodes for controlling the TurtleBot3. The p3_d_xy_goal node from this package is used to drive the TurtleBot3 model.
"""

#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    ##### File Paths are stored
    # This section retrieves the path of the turtlebot3_gazebo package.
    pkg_tb3_drive = get_package_share_directory('turtlebot3_gazebo')

    ##### Include the launch file to spawn TurtleBot3 model in an empty world
    # This section includes the empty_world.launch.py launch file from the turtlebot3_gazebo package to spawn a TurtleBot3 model in an empty world.
    spawn_tb3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_drive, 'launch', 'empty_world.launch.py')
        ),
    )

    ##### Start the rqt_reconfigure node
    # This section starts the rqt_reconfigure node, which provides a GUI for dynamically reconfiguring parameters.
    reconfigure = Node(
        package='rqt_reconfigure',
        executable='rqt_reconfigure',
        name='parameter_reconfig'
    )

    ##### Start the node to drive the TurtleBot3 model
    # This section starts the p3_d_xy_goal node from the drive_tb3 package to drive the TurtleBot3 model.
    drive_tb3_b = Node(
        package='drive_tb3',
        executable='p3_d_xy_goal',
        name='custom_node'
    )

    ld = LaunchDescription()

    ##### Add the commands to the launch description
    # This section adds all the IncludeLaunchDescription and Node actions to the launch description.
    ld.add_action(spawn_tb3)
    ld.add_action(reconfigure)
    ld.add_action(drive_tb3_b)

    # Return the LaunchDescription
    return ld
