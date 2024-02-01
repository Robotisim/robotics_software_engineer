"""
Author: Muhammad Luqman
Organization: Robotisim

This launch file initiates a Gazebo simulation in a specified world and spawns four TurtleBot3 models at distinct locations.

Arguments used:
- world: Full path to the Gazebo world file to load.
- x_pos: The x-coordinate where the TurtleBot3 model should be spawned in the Gazebo world.
- y_pos: The y-coordinate where the TurtleBot3 model should be spawned in the Gazebo world.
- yaw_rot: The yaw rotation of the TurtleBot3 model when it is spawned.
- robot_name: The name of the TurtleBot3 model to be spawned.
- robot_ns: The namespace into which the TurtleBot3 model should be spawned.

Packages used:
- gazebo_ros: This package provides nodes and plugins necessary to run, interface and interact with Gazebo simulations from ROS2. In this launch file, it is used to start the Gazebo server and client.
- turtlebot3_gazebo: This package contains the Gazebo simulation environments for TurtleBot3. In this launch file, it is used to retrieve the path of the Gazebo world file.
- drive_tb3: This is a custom package that is likely to contain launch files and nodes necessary to control the TurtleBot3 models. In this launch file, it is used to spawn the TurtleBot3 models at specified locations in the Gazebo simulation.





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
    # This section retrieves the path of the gazebo_ros and drive_tb3 packages.
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tb3_drive = get_package_share_directory('drive_tb3')

    #### Gazebo Launching and importing
    # This section retrieves the path to the desired Gazebo world file.
    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_dqn_stage1.world'
    )

    ##### Start the Gazebo server with the specified world
    # This section starts the Gazebo server with the specified world.
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    ##### Start the Gazebo client
    # This section starts the Gazebo client.
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    ##### Robot Spawning
    # These sections spawn four TurtleBot3 models at specified locations using the p2_a_spawn_tb3.launch.py launch file from the drive_tb3 package.
    robot_spawner1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_drive, 'launch', 'p2_a_spawn_tb3.launch.py')
        ),
        launch_arguments={
            'x_pos' :'2.0' ,
            'y_pos' :'-2.0',
            'yaw_rot' :'1.57',
            'robot_name' :'tb_1',
            'robot_ns'  : 'robot_a'
        }.items()
    )

    robot_spawner2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_drive, 'launch', 'p2_a_spawn_tb3.launch.py')
        ),
        launch_arguments={
            'x_pos' :'2.0' ,
            'y_pos' :'2.0',
            'yaw_rot' :'3.14',
            'robot_name' :'tb_2',
            'robot_ns'  : 'robot_b'
        }.items()
    )

    robot_spawner3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_drive, 'launch', 'p2_a_spawn_tb3.launch.py')
        ),
        launch_arguments={
            'x_pos' :'-2.0' ,
            'y_pos' :'2.0',
            'yaw_rot' :'-1.57',
            'robot_name' :'tb_3',
            'robot_ns'  : 'robot_c'
        }.items()
    )

    robot_spawner4 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_drive, 'launch', 'p2_a_spawn_tb3.launch.py')
        ),
        launch_arguments={
            'x_pos' :'-2.0' ,
            'y_pos' :'-2.0',
            'yaw_rot' :'0.0',
            'robot_name' :'tb_4',
            'robot_ns'  : 'robot_d'
        }.items()
    )

    ld = LaunchDescription()

    ##### Add the commands to the launch description
    # This section adds all the IncludeLaunchDescription actions to the launch description.
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_spawner1)
    # ld.add_action(robot_spawner2)
    # ld.add_action(robot_spawner3)
    # ld.add_action(robot_spawner4)

    # Return the LaunchDescription
    return ld
