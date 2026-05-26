#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    world = os.path.join(
        get_package_share_directory('mapping'),
        'worlds',
        'maze_map.world'
    )

    sim_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_sensing'), 'launch', 'custom_sensors.launch.py')
        ),
        launch_arguments={'world': world, 'x_pose': '-6.794486', 'y_pose': '3.618'}.items()
    )

    ld = LaunchDescription()

    ld.add_action(sim_bringup)
    return ld
