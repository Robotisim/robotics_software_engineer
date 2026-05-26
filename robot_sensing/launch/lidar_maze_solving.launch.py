#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Joep Tool

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    world = os.path.join(
        get_package_share_directory('robot_sensing'),
        'worlds',
        'maze.world'
    )

    sim_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_sensing'), 'launch', 'custom_sensors.launch.py')
        ),
        launch_arguments={'world': world, 'x_pose': '-8.980152', 'y_pose': '7.135622'}.items()
    )

    maze_solver = Node(
        package = 'robot_sensing',
        name = 'maze_solver',
        executable ='mazeSolving',

    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(sim_bringup)
    ld.add_action(maze_solver)

    return ld
