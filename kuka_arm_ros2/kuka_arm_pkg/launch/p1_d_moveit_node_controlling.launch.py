from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config_node = (
        MoveItConfigsBuilder("kuka_arm_moveit_ws",package_name="kuka_arm_moveit")
        .robot_description(file_path="config/kr210_arm.urdf.xacro")
        .robot_description_semantic(file_path="config/kr210_arm.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )
    # Moveit action server
    move_group_node = Node(
        package="kuka_arm_pkg",
        executable="p1_b_moveit_drive",
        output="screen",
        parameters=[moveit_config_node.to_dict()],
    )
    return LaunchDescription([move_group_node])