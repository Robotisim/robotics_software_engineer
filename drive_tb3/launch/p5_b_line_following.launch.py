"""
Author: Muhammad Luqman
Organization: Robotisim

This launch file starts a Gazebo simulation with a specified world, spawns a line track model and a TurtleBot3 model
at specified locations, and starts a node for line following.

Arguments used:
- 'world': Path to the Gazebo world file to load
- 'x_pos': x-coordinate position to spawn the TurtleBot3 model
- 'y_pos': y-coordinate position to spawn the TurtleBot3 model
- 'yaw_rot': yaw rotation to spawn the TurtleBot3 model
- 'robot_name': Name of the TurtleBot3 model
- 'maze_path': Path to the maze model file (in SDF format)

Packages used:
- drive_tb3: This custom package is used to specify the line following world file for Gazebo, spawn the TurtleBot3 model and line track model at specific locations, and start the line following node (p5_d_line_following).
- gazebo_ros: This package is used to launch Gazebo with the specified world file. The launch files gzserver.launch.py and gzclient.launch.py are included from this package.

"""

#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    ############# Paths Setting
    # This section retrieves the paths of the drive_tb3 and gazebo_ros packages, sets the path of the line track model and the world file.
    pkg_tb3_drive = get_package_share_directory('drive_tb3')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    maze_path = os.path.join(pkg_tb3_drive,'models','line_track','model.sdf')

    world = os.path.join(
        pkg_tb3_drive,
        'worlds',
        'line_following.world'
    )

    ############# Gazebo Definitions
    # These sections start the Gazebo server and client with the specified world.
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    ############# Spawning Robot
    # This section spawns a TurtleBot3 model at a specified location using the p2_a_spawn_tb3.launch.py launch file from the drive_tb3 package.
    robot_spawner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_drive, 'launch', 'p2_a_spawn_tb3.launch.py')
        ),
        launch_arguments={
            'x_pos': '2.16',
            'y_pos': '-1.04',
            'yaw_rot': '3.11',
            'robot_name' :'turtlebot_3',
        }.items()
    )

    ############# Lidar Processing Node
    # This section starts the p5_d_line_following node from the drive_tb3 package for line following.
    follow_line_tb3 = Node(
        package='drive_tb3',
        executable='p5_d_line_following',
        name='Line_following'
    )

    ############# Spawning Maze Model
    # This section starts the p6_b_sdf_spawner node from the drive_tb3 package to spawn the line track model.
    maze_spawner = Node(
        package='drive_tb3',
        executable='p6_b_sdf_spawner',
        name='maze_model',
        arguments=[maze_path,"Maze","0.0","0.0"]
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(maze_spawner)
    ld.add_action(robot_spawner)
    ld.add_action(follow_line_tb3)

    # Return the LaunchDescription
    return ld
