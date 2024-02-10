from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    config_file_path = os.path.join(get_package_share_directory('point_cloud_processing'),'rviz','kitti_viz.rviz')

    return LaunchDescription([
        Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['-d', config_file_path]
            ),
    ])