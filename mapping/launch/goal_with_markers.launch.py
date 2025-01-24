#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = os.path.join(get_package_share_directory('mapping'))
    map_yaml_file = os.path.join(pkg_path, 'map', 'map.yaml')
    rviz_config = os.path.join(pkg_path, 'config', 'map_loading_goal.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],

    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_file}]
    )


    set_map_server_to_configure = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/map_server', 'configure'],
        output='screen'
    )

    # Command to activate the map server, wrapped in a TimerAction for delay
    set_map_server_to_activate = TimerAction(
        period=3.0,  # Delay in seconds
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/map_server', 'activate'],
                output='screen'
            )
        ]
    )


    ld = LaunchDescription()
    ld.add_action(map_server)
    ld.add_action(rviz)
    ld.add_action(set_map_server_to_configure)
    ld.add_action(set_map_server_to_activate)


    return ld
