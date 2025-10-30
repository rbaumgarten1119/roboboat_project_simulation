#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

def generate_launch_description():
    # Declare launch arguments
    lidar_topic_arg = DeclareLaunchArgument(
        'lidar_topic',
        default_value='/lidar',
        description='LIDAR topic name in ROS2'
    )
    
    output_file_arg = DeclareLaunchArgument(
        'output_file',
        default_value='lidar_3d_data.csv',
        description='Output CSV file path'
    )
    
    log_frequency_arg = DeclareLaunchArgument(
        'log_frequency',
        default_value='10',
        description='Logging frequency in Hz'
    )
    
    max_points_arg = DeclareLaunchArgument(
        'max_points_per_scan',
        default_value='10000',
        description='Maximum points to log per scan'
    )

    gazebo_topic_arg = DeclareLaunchArgument(
        'gazebo_topic',
        default_value='/lidar',
        description='Gazebo internal LIDAR topic name'
    )

    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='true',
        description='Enable debug output'
    )

    # Debug: List Gazebo topics
    list_gz_topics = ExecuteProcess(
        cmd=['gz', 'topic', '-l'],
        output='screen',
        name='list_gazebo_topics',
        condition=IfCondition(LaunchConfiguration('debug_mode'))
    )

    # ROS-Gazebo bridge for 3D LIDAR data
    # Try common LIDAR message type variations
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/lidar@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            # Alternative message types to try:
            # '/lidar@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloud',
            # '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
        ],
        output='screen',
        name='lidar_bridge',
        parameters=[{
            'use_sim_time': True
        }]
    )

    # Debug: Check ROS2 topics after bridge starts
    check_ros_topics = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'topic', 'list'],
                output='screen',
                name='list_ros_topics',
                condition=IfCondition(LaunchConfiguration('debug_mode'))
            )
        ]
    )

    # Debug: Test topic data
    test_topic_data = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['bash', '-c', 'timeout 3s ros2 topic echo /lidar --no-arr | head -20'],
                output='screen',
                name='test_lidar_data',
                condition=IfCondition(LaunchConfiguration('debug_mode'))
            )
        ]
    )
    
    # 3D LIDAR CSV Logger Node - start after bridge has time to initialize
    lidar_logger_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='lidar_csv_logger',
                executable='lidar_csv_logger_node',
                name='lidar_csv_logger',
                output='screen',
                parameters=[{
                    'output_file': LaunchConfiguration('output_file'),
                    'lidar_topic': LaunchConfiguration('lidar_topic'),
                    'log_frequency': LaunchConfiguration('log_frequency'),
                    'max_points_per_scan': LaunchConfiguration('max_points_per_scan'),
                    'use_sim_time': True
                }],
                # Add remapping if needed
                # remappings=[
                #     ('/lidar', '/lidar/points'),  # Example remapping
                # ]
            )
        ]
    )

    return LaunchDescription([
        lidar_topic_arg,
        output_file_arg,
        log_frequency_arg,
        max_points_arg,
        gazebo_topic_arg,
        debug_mode_arg,
        list_gz_topics,
        bridge_node,
        check_ros_topics,
        test_topic_data,
        lidar_logger_node
    ])
