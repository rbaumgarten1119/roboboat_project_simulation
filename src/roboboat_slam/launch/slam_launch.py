from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('roboboat_slam')
    params_dir = os.path.join(pkg_share, 'params')

    ekf_params = os.path.join(params_dir, 'roboboat_localization.yaml')
    slam_params = os.path.join(params_dir, 'slam_toolbox_params.yaml')

    ld = LaunchDescription()

    # Thruster dead-reckoning -> /odom
    ld.add_action(Node(
        package='roboboat_slam',
        executable='thruster_to_odom',
        name='thruster_to_odom',
        output='screen',
        parameters=[{'use_sim_time': True}]
    ))

    # EKF (robot_localization)
    ld.add_action(Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params, {'use_sim_time': True}]
    ))

    # PointCloud2 → LaserScan converter
    ld.add_action(Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pcl_to_scan',
        output='screen',
        remappings=[('/cloud_in','/lidar/points'), ('/scan','/scan')],
        parameters=[{'use_sim_time': True}]
    ))

    # slam_toolbox (mapping)
    ld.add_action(Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': True}]
    ))

    return ld
