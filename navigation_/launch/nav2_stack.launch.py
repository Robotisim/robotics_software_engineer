#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    mapping_pkg_dir = os.path.join(get_package_share_directory('mapping'))
    navigation_pkg_dir=get_package_share_directory('navigation_')
    map_file = os.path.join(mapping_pkg_dir,'map','maze_2d.yaml')
    params_file = os.path.join(navigation_pkg_dir,'config','nav2_params.yaml')
    rviz_config_file = os.path.join(navigation_pkg_dir,'config','nav2.rviz')

    env_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join( mapping_pkg_dir, 'launch', 'maze_tb3_bringup.launch.py')
        ),
    )


    navigation = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'), '/launch/bringup_launch.py']),
                launch_arguments={
                    'map': map_file,
                    'params_file': params_file
    }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=["-d", rviz_config_file],
        name='rviz'
    )

    ld = LaunchDescription()

    ld.add_action(env_bringup)
    ld.add_action(navigation)
    ld.add_action(rviz)

    return ld