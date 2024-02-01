"""
Author: Muhammad Luqman
Organization: Robotisim

This launch file starts a Gazebo simulation, spawns a TurtleBot3 model and a maze,
then starts the mapping functionality and Rviz2 for visualization.


Arguments used:

- 'use_sim_time': Use simulation (Gazebo) clock if true
- 'x_pos': x-coordinate position to spawn the TurtleBot3 model
- 'y_pos': y-coordinate position to spawn the TurtleBot3 model
- 'yaw_rot': yaw rotation to spawn the TurtleBot3 model
- 'robot_name': Name of the TurtleBot3 model
- 'maze_path': Path to the maze model file (in SDF format)
- 'rviz_config': Path to the Rviz configuration file

Packages used:
- turtlebot3_gazebo: This package is used to specify the directory for the launch files and to start the robot_state_publisher node.
- drive_tb3: This custom package is used to spawn the TurtleBot3 model and the maze.
- gazebo_ros: This package is used to launch Gazebo. The launch files gzserver.launch.py and gzclient.launch.py are included from this package.
- slam_toolbox: This package is used to start the mapping functionality via the online_async_launch.py launch file.
- rviz2: This package is used to start the Rviz2 node for visualization.





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
    # This section sets the paths of the launch file directory, maze model, and Rviz configuration file.
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_tb3_drive = get_package_share_directory('drive_tb3')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    maze_path = os.path.join(pkg_tb3_drive,'models','maze','model.sdf')
    config_dir = os.path.join(get_package_share_directory('turtlebot3_navigation2'),'config')
    rviz_config= os.path.join(config_dir,'tb3_navigation2.rviz')

    ############# Gazebo Definitions
    # These sections start the Gazebo server and client.
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
    )
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    ############# Spawning Robot and Maze
    # This section starts the robot_state_publisher node, spawns a TurtleBot3 model at a specified location, and spawns a maze.
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': 'True'}.items()
    )
    robot_spawner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_drive, 'launch', 'p2_a_spawn_tb3.launch.py')
        ),
        launch_arguments={
            'x_pos': '-6.1',
            'y_pos': '-7.1',
            'yaw_rot': '1.57',
            'robot_name' :'turtlebot_3',
        }.items()
    )
    maze_spawner = Node(
        package='drive_tb3',
        executable='p6_b_sdf_spawner',
        name='maze_model',
        arguments=[maze_path,"Maze","0.0","0.0"]
    )

    ############# Mapping Functionality
    # This section starts the online_async_launch.py launch file from the slam_toolbox package for mapping.
    maze_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'),'launch', 'online_async_launch.py')
        ),
    )

    ############ Rviz Configuration
    # This section starts the Rviz2 node for visualization.
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # Add 2 more Nodes for spawning robot with navigation package

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_spawner)
    ld.add_action(maze_spawner)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(maze_mapping)
    ld.add_action(rviz)

    # Return the LaunchDescription
    return ld
