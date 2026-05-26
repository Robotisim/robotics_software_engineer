# This script generates a ROS 2 launch description for a differential drive robot simulation.
# It utilizes the robot_state_publisher and joint_state_publisher to load and publish the robot's URDF.
# Additionally, it launches a Gazebo simulation environment and spawns the robot within Gazebo.
# The URDF model is specified in the 'custom_robots' package under 'urdf/diff_drive.urdf'.
# Author: Robotisim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkgPath = get_package_share_directory('custom_robots')
    urdfFile = os.path.join(pkgPath, 'urdf', 'diff_drive.urdf')

    with open(urdfFile, 'r') as urdf:
        robot_description = {'robot_description': urdf.read()}

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': '-r -s empty.sdf'}.items()
    )

    return LaunchDescription([
        gazebo,

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': True}]),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[robot_description, {'use_sim_time': True}]),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gazebo_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
            ],
            output='screen'),

        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    name='robot_spawner',
                    output='screen',
                    arguments=[
                        '-name', 'diff_drive_bot',
                        '-topic', '/robot_description',
                        '-x', '0.0',
                        '-y', '0.0',
                        '-z', '0.2'
                    ])
            ])
    ])
