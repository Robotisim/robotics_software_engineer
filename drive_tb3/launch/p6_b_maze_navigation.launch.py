"""
Author: Darby Lim
Modified by: Muhammad Luqman
Organization: Robotisim

This launch file starts a Gazebo simulation, spawns a TurtleBot3 model and a maze, and then starts the navigation2 functionality along with Rviz2 for visualization.

Packages used:
- gazebo_ros
- drive_tb3
- turtlebot3_gazebo
- nav2_bringup
- rviz2

Arguments used:
- map: Full path to map file to load
- params_file: Full path to param file to load
- use_sim_time: Use simulation (Gazebo) clock if true
"""

#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']


def generate_launch_description():
    ############# Paths Setting
    # This section sets the paths of the launch file directory, maze model, Rviz configuration file, and navigation2 bringup.
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tb3_drive = get_package_share_directory('drive_tb3')
    maze_path = os.path.join(pkg_tb3_drive,'models','maze','model.sdf')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')

    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('drive_tb3'),
            'map',
            'maze_map.yaml'))

    param_file_name = TURTLEBOT3_MODEL + '.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('drive_tb3'),
            'config',
            param_file_name))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('drive_tb3'),
        'config',
        'nav2.rviz')

    return LaunchDescription([
        # Arguments
        # This section declares the launch arguments.
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # Launch Files included
        # These sections start the Gazebo server and client and the robot_state_publisher node.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            )
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': 'True'}.items()
        ),

        # Spawning Robot
        # This section spawns a TurtleBot3 model at a specified location.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_tb3_drive, 'launch', 'p2_a_spawn_tb3.launch.py')
            ),
            launch_arguments={
                'x_pos': '-6.1',
                'y_pos': '-7.1',
                'yaw_rot': '1.57',
                'robot_name' :'turtlebot_3',
            }.items()
        ),

        # Integerating Nav2 Stack
        # This section starts the bringup_launch.py launch file from the nav2_bringup package for navigation2 functionality.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),

        # Spawn Maze
        # This section spawns a maze.
        Node(
            package='drive_tb3',
            executable='p6_b_sdf_spawner',
            name='maze_model',
            arguments=[maze_path,"Maze","0.0","0.0"]
        ),

        # Launch Rviz2 with Configuration
        # This section starts the Rviz2 node for visualization.
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
