import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('my_lidar_processor')
    params_file = os.path.join(pkg_share, 'config', 'processor_params.yaml')

    return LaunchDescription([
        Node(
            package='my_lidar_processor',
            executable='lidar_processor_3d_node',
            name='lidar_processor_3d_node',
            output='screen',
            parameters=[params_file],
            remappings=[('input_pointcloud', 'velodyne_points'), ('output_pointcloud', 'velodyne_points_filtered')]
        )
    ])
