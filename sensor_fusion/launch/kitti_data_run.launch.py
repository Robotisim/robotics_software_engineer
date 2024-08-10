from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    config_file_path = os.path.join(
        get_package_share_directory("sensor_fusion"), "config", "kitti_viz.rviz"
    )

    return LaunchDescription(
        [
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", config_file_path],
            ),
            Node(
                package="ros2_kitti_publishers",
                executable="kitti_publishers",
                output="screen",
            ),
            Node(
                package="sensor_fusion",
                executable="ekf_node",
                output="screen",
            ),
        ]
    )
