#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import  Node


def generate_launch_description():
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        output='screen',
    )

    behaviour_tree_node = Node(
        package='behavior_trees',
        executable='b_r2_bt_go_to_goal',
        output='screen',
    )



    ld = LaunchDescription()

    ld.add_action(turtlesim_node)
    ld.add_action(behaviour_tree_node)

    return ld
