from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get paths to package directories
    turtlebot3_description_path = get_package_share_directory('turtlebot3_description')
    turtlebot3_urdf_file = os.path.join(turtlebot3_description_path, 'urdf', 'turtlebot3_burger.urdf')

    # Check if URDF file exists
    if not os.path.isfile(turtlebot3_urdf_file):
        raise RuntimeError(f"URDF file not found: {turtlebot3_urdf_file}")

    # Include the empty world launch file
    gazebo_launch_file_path = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch', 'empty_world.launch.py')
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file_path)
    )

    # Launch robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(turtlebot3_urdf_file).read()}],
    )

    # Launch joint state publisher node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'robot_description': open(turtlebot3_urdf_file).read()}],
    )

    # Launch rviz2 node with a configuration file (optional)
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('module_5_assignment'), 'rviz', 'default_config.rviz')]
    )

    # Launch your LQR node
    lqr_node = Node(
        package='module_5_assignment',
        executable='lqr_nodetst',
        name='lqrNode',
        output='screen',
    )

    # Return launch description
    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz2_node,
        lqr_node,
    ])
