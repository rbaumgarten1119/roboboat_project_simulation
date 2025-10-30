from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navigation_pkg',
            executable='navigator_node',
            name='navigator_node',
            output='screen'
        )
    ])
