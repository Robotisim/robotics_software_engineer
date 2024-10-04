from launch import LaunchDescription
from launch_ros.actions import Node

# Import for getting package share directory (assuming you're using ament_package)
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    """Generates launch description for robot state publisher, joint state publisher and rviz2."""

    # Get path to the URDF file
    pkg_path = get_package_share_directory('custom_robots')
    urdf_file = os.path.join(pkg_path, "urdf", "ackerman_drive.urdf")

    # Check if URDF file exists (optional for error handling)
    #if not os.path.isfile(urdf_file):
    #    raise RuntimeError(f"URDF file not found: {urdf_file}")

    # Launch robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[urdf_file],
    )

    # Launch joint state publisher node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[urdf_file],
    )

    # Launch rviz2 node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    # Return launch description
    return LaunchDescription([robot_state_publisher_node,rviz2_node,joint_state_publisher_node])

