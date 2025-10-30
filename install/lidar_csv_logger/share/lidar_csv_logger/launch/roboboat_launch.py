from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Adjust this path to where your roboboat.sdf is stored
    roboboat_sdf = os.path.join(
        os.getenv("HOME"), "ros2_ws", "src", "lidar_csv_logger", "worlds", "roboboatTest.sdf"
    )

    return LaunchDescription([
        # Spawn roboboat model
        ExecuteProcess(
            cmd=["gz", "sim", "-r", "-v4", roboboat_sdf],
            output="screen"
        ),
            # **Here** add the clock bridge
        ExecuteProcess(
            cmd=[
              "ros2", "run", "ros_gz_bridge", "parameter_bridge",
              "/world/empty/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock",
              "--ros-args",
              "-r", "/world/empty/clock:=/clock",
              "-p", "qos.reliability:=reliable",
              "-p", "qos.durability:=transient_local"
            ],
            output="screen"
        ),

        # Bridge lidar points (3D point cloud)
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_bridge", "parameter_bridge",
                "/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked"
            ],
            output="screen"
        ),

        # Bridge IMU sensor
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_bridge", "parameter_bridge",
                "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU"
            ],
            output="screen"
        ),
        
        # REST OF PROCESSES ARE TEST FOR CAMERA. IF EXPERIENCING PROBLEMS REMOVE THE ONES BELOW THIS LINE
        
        # Bridge the RGB color image
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_bridge", "parameter_bridge",
                "/rs_front/image@sensor_msgs/msg/Image@gz.msgs.Image"
            ],
            output="screen"
        ),
        # Bridge the depth image
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_bridge", "parameter_bridge",
                "/rs_front/depth_image@sensor_msgs/msg/Image@gz.msgs.Image"
            ],
            output="screen"
        ),
        # Bridge the point cloud
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_bridge", "parameter_bridge",
                "/rs_front/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked"
            ],
            output="screen"
        ),

        # Bridge GPS sensor
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_bridge", "parameter_bridge",
                "/gps/fix@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat"
            ],
            output="screen"
        ),
        # Thruster command bridges (NEW)
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_bridge", "parameter_bridge",
                "/roboboatmodel/roboboat/joint/left_thruster_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double",
                "/roboboatmodel/roboboat/joint/right_thruster_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double"
            ],
            output="screen"
        ),
    ])
