import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    This launch file initializes:
    1. Environment (Gazebo World)
    2. Robot (Spawn a Mobile Robot)
    3. 3D RTAB Mapping System (RTAB-Map)
    4. RViz for Visualization
    """

    bringup_environment = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_sensing'), 'launch', 'custom_sensors.launch.py')
        )
    )

    # ------------------------------
    # 3️⃣ RTAB-Map 3D SLAM Configuration
    # ------------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')

    rtab_parameters ={
          'frame_id':'simple_drone/base_footprint',
          'odom_frame_id': 'simple_drone/odom',
          'use_sim_time':use_sim_time,
          'subscribe_depth':True,
          'use_action_for_goal':True,
          'qos_image':qos,
          'qos_imu':qos,
        #   'Reg/Force3DoF':'true',
        #   'Optimizer/GravitySigma':'0' # Disable imu constraints (we are already in 2D)
    }

    remappings=[
        ('/odom', 'simple_drone/odom'),
        ('rgb/image', '/simple_drone/front/image_raw'),
        ('rgb/camera_info', '/simple_drone/front/camera_info'),
        ('depth/image', '/simple_drone/front/depth/image_raw')
    ]

    # ------------------------------
    # 4️⃣ RViz Visualization
    # ------------------------------
    rviz_config_path = os.path.join(get_package_share_directory('slam'), 'config', '3d_mapping.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    # ------------------------------
    # 5️⃣ RTAB-Map 3D SLAM Nodes
    # ------------------------------

    # **Mapping Mode (SLAM)**
    rtabmap_slam_node = Node(
        condition=UnlessCondition(localization),
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[rtab_parameters],
        remappings=remappings,
        arguments=['-d']  # This will delete the previous database (~/.ros/rtabmap.db)
    )

    # **Localization Mode**
    rtabmap_localization_node = Node(
        condition=IfCondition(localization),
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[rtab_parameters, {
            'Mem/IncrementalMemory': 'False',
            'Mem/InitWMWithAllNodes': 'True'
        }],
        remappings=remappings
    )

    # **RTAB-Map Visualization**
    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        output='screen',
        parameters=[rtab_parameters],
        remappings=remappings
    )

    # ------------------------------
    # 6️⃣ Return Launch Description
    # ------------------------------
    return LaunchDescription([
        # Declare Launch Arguments
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('qos', default_value='2', description='QoS used for input sensor topics'),
        DeclareLaunchArgument('localization', default_value='false', description='Launch in localization mode.'),

        # Include Gazebo World & Robot
        bringup_environment,
        # Include RViz & RTAB-Map Nodes
        rviz_node,
        rtabmap_slam_node,
        rtabmap_localization_node,
        rtabmap_viz_node
    ])
