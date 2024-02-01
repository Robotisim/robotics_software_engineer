from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    # Path to Robot's Xacro File
    pkg_path= get_package_share_directory("kuka_arm_pkg")
    xacro_file= os.path.join(pkg_path,'urdf','kr210.urdf.xacro')
    rviz_config = os.path.join(
        get_package_share_directory("kuka_arm_pkg"), "config" ,"kuka_arm.rviz"
    )

    #Processing Xacro File
    xacro_parser=xacro.parse(open(xacro_file))
    xacro.process_doc(xacro_parser)

    #Feeding URDF to ROS
    parameters = {'robot_description': xacro_parser.toxml()}


    #Ros Packages
    joint_state_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",

    )
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[parameters]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
    )

    #Running all definations
    nodes_to_run = [
        joint_state_publisher,
        robot_state_publisher_node,
        rviz_node
    ]


    return LaunchDescription(nodes_to_run)