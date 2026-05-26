import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory('slambot_control'),
        'urdf',
        'slambot_mock.urdf'
    )

    with open(urdf_path, 'r') as infp:
        robot_description_content = infp.read()

    robot_description = {'robot_description': robot_description_content}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
    )

    robot_controllers = PathJoinSubstitution([
        FindPackageShare('slambot_control'),
        'config',
        'slambot_controller.yaml',
    ])

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_controllers],
        output='both',
        remappings=[
            ('~/robot_description', '/robot_description'),
            ('/diff_drive_controller/odom', '/odom'),
            ('/diff_drive_controller/cmd_vel_unstamped', '/cmd_vel'),
        ],
    )

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen',
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher_node,
        control_node,
        joint_state_spawner,
        diff_drive_spawner,
    ])
