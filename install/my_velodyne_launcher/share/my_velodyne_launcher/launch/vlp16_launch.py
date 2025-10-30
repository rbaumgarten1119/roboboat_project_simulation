#edit made 9/13 14:42 - added converter node, frankly unsure if the node works, but wanted to make progress in branch
# Useful documentation - https://docs.ros.org/en/rolling/p/velodyne_pointcloud/
#Slightly less useful documentation - https://wiki.ros.org/velodyne_pointcloud
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # This is the launch description for the Velodyne driver node.
    # It's configured for a VLP-16 sensor.

    velodyne_driver_node = Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        name='velodyne_driver_node',
        output='screen',
        parameters=[
            {'device_ip': '192.168.1.201'}, # The IP of your sensor
            {'port': 2368},                 # The port the sensor is publishing on
            {'frame_id': 'velodyne_link'},  # The frame ID for the LiDAR data
            {'model': 'VLP16'},             # The model of your sensor
            {'gps_time': False},            # Use host time, not GPS time
            {'rpm': 600.0},                 # The sensor's rotational speed
            {'cut_angle': -0.01}            # Use the default packet rate logic
            # To use a PCAP file for testing, comment out the lines above
            # and uncomment the lines below.
            # {'pcap': '/path/to/your/pcap_file.pcap'},
            # {'read_once': True}
        ]
    )

    convert_node = Node(
        package='velodyne_pointcloud', #This should come from an import statement somewhere, I need to investigate later... same with those below
        executable='velodyne_convert_node',
        name='velodyne_convert_node',
        parameters=[{'calibration':""}, #can make a calibration file
                    {'min_range':0.0},
                    {'max_range':200.0},
                    {'view_direction':0},
                    {'view_width':3.14},
                    {'organize_cloud':True}] 
    )
    #for calibration file, I saw once that it was a .yaml file you can configure - from documentation - "There are a set of default calibration files to start with in the “params” subdirectory in this package" Where package is velodyne_pointcloud
    #for min range - I set at zero for the moment but if we get too many points too close (i.e. on the boat) we can edit this
    #for max range - I have no clue what units we are in but I put 200, edit if there are issues
    #for view_direction - from documentaion - Must be between -Pi and Pi, where 0 is straight ahead from the device. Defaults to 0.0.
    #for view_width - from 0 to 2pi, I set it at 3.14 and it will only make a 180 degree pointcloud - reduces the load on cpu for testing
    #for organize_cloud - Whether to organize the cloud by ring (True), or to use the order as it comes directly from the driver (False). Defaults to True.
    #for target_frame, check first documentation link, I cant explain
    #fixed frame and target frame are two other parameters we can pass, but I dont think we need to limit GPU/CPU that much...


    return LaunchDescription([
        velodyne_driver_node,
        convert_node
    ])