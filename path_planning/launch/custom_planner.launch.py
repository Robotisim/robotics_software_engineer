#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    mapping_pkg_path = os.path.join(get_package_share_directory('mapping'))
    map_yaml_file = os.path.join(mapping_pkg_path, 'map', 'map.yaml')
    rviz_config_file = os.path.join(get_package_share_directory('navigation_'), 'config', 'nav2_steps.rviz')
    planner_params_file = os.path.join(get_package_share_directory('path_planning'), "config", "planner_params.yaml"
    )

    nodes = [
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=["-d", rviz_config_file],
            name='rviz'
        ),

       Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml_file}]
        ),
        Node(
            package="nav2_costmap_2d",
            executable="nav2_costmap_2d",
            name="costmap",
            output="screen"
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf_base_link_to_map",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            output="screen"
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_pub_odom_to_base_link",
            arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
            output="screen"
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[planner_params_file],
        )



    ]

    return LaunchDescription(nodes)
