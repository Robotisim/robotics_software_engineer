from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='led_control',
            executable='led_publisher',
            name='led_publisher',
        ),
        Node(
            package='led_control',
            executable='led_subscriber',
            name='led_subscriber',
        ),
    ])
