from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_path = get_package_share_directory('control_systems')

    urdf_file = os.path.join(pkg_path, 'urdf','diff_drive_velocity_controller.urdf')
    controller_config = os.path.join(pkg_path, 'config', 'diffbot_controller.yaml')

    with open(urdf_file, 'r') as urdf:
        robot_description_content = urdf.read().replace('__CONTROLLER_CONFIG__', controller_config)

    robot_description = {'robot_description': robot_description_content}

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
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen'),

        Node(
            package='ros_gz_sim',
            executable='create',
            name='robot_spawner',
            output='screen',
            arguments=[
                '-name', 'diff_drive_velocity_bot',
                '-topic', '/robot_description',
                '-z', '0.2'
            ]),

        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    output='screen',
                    arguments=['joint_state_broadcaster']),

                Node(
                    package='controller_manager',
                    executable='spawner',
                    output='screen',
                    arguments=['wheels_velocity_controller']),

                Node(
                    package='control_systems',
                    executable='velocity_controller_drive',
                    output='screen')
            ]),

    ])
