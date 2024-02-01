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
    robot_spawner= IncludeLaunchDescription(
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


    ld = LaunchDescription()

    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_spawner)

    return ld
