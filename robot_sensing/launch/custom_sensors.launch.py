# Purpose:
# - Launch a ROS2 robot simulation environment using Gazebo
# - Publish robot state and joint states from URDF file
# Author: Robotisim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkgPath = get_package_share_directory('robot_sensing')

    urdfFile = os.path.join(pkgPath, 'urdf', 'diff_drive.urdf')
    default_world = os.path.join(pkgPath, 'worlds', 'maze.world')

    world = LaunchConfiguration('world')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')

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
        launch_arguments={'gz_args': ['-r -s ', world]}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('z_pose', default_value='0.2'),

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
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'
            ],
            output='screen',
        ),

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
                        '-x', x_pose,
                        '-y', y_pose,
                        '-z', z_pose
                    ])
            ])
    ])
