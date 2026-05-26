from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    bring_environment = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mapping'), 'launch', 'maze_tb3_bringup.launch.py')
        )
    )


    rviz_bringup = Node(

        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('slam'), 'config', 'view_map.rviz')]
    )

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )



    ########## Launch Description ##########
    ld = LaunchDescription()

    ld.add_action(bring_environment)
    ld.add_action(rviz_bringup)
    ld.add_action(mapping)

    return ld
