#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory("path_planning")
    rviz_config_file_path = os.path.join(
        pkg_path, "config", "search_path_planning.rviz"
    )
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file_path],
    )

    Occupancy_grid = Node(
        package="path_planning",
        executable="occupancy_grid_node",
        output="screen",
    )

    path_planning_node = Node(
        package="path_planning",
        executable="path_planning_node",
        output="screen",
    )

    ld = LaunchDescription()

    ld.add_action(Occupancy_grid)
    ld.add_action(path_planning_node)
    ld.add_action(rviz2)

    return ld
