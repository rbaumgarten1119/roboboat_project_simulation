#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu, Image, NavSatFix
from sensor_msgs_py import point_cloud2 as pc2
from cv_bridge import CvBridge
import csv
import os
import cv2
import numpy as np
from datetime import datetime

class RoboboatLogger(Node):
    def __init__(self):
        super().__init__('roboboat_logger')

        # Bridge for image conversion
        self.bridge = CvBridge()

        # Create log directory
        self.data_dir = os.path.join(os.getcwd(), "roboboat_logs")
        os.makedirs(self.data_dir, exist_ok=True)

        # LiDAR CSV
        self.csv_file = os.path.join(
            self.data_dir, f"lidar_{self.timestamp()}.csv")
        with open(self.csv_file, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["time_sec", "x", "y", "z"])

        # IMU CSV
        self.imu_file = os.path.join(
            self.data_dir, f"imu_{self.timestamp()}.csv")
        with open(self.imu_file, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "time_sec",
                "orientation_x", "orientation_y", "orientation_z", "orientation_w",
                "angular_velocity_x", "angular_velocity_y", "angular_velocity_z",
                "linear_acceleration_x", "linear_acceleration_y", "linear_acceleration_z"
            ])

        # GPS CSV
        self.gps_file = os.path.join(
            self.data_dir, f"gps_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
        with open(self.gps_file, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "time_sec", "latitude", "longitude", "altitude",
                "cov_type", "cov_xx", "cov_xy", "cov_xz", "cov_yy", "cov_yz", "cov_zz"
            ])

        # Log filenames
        self.get_logger().info(f"Logging LiDAR to {self.csv_file}")
        self.get_logger().info(f"Logging IMU to {self.imu_file}")
        self.get_logger().info(f"Logging GPS to {self.gps_file}")

        # Camera video settings
        self.image_width = 640
        self.image_height = 360
        self.fps = 30

        # RGB Video Writer
        rgb_filename = os.path.join(
            self.data_dir, f"camera_rgb_{self.timestamp()}.avi")
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self.rgb_video_writer = cv2.VideoWriter(
            rgb_filename, fourcc, self.fps, (self.image_width, self.image_height))
        self.get_logger().info(f"Recording RGB video to {rgb_filename}")

        # Depth Video Writer
        depth_filename = os.path.join(
            self.data_dir, f"camera_depth_{self.timestamp()}.avi")
        self.depth_video_writer = cv2.VideoWriter(
            depth_filename, fourcc, self.fps, (self.image_width, self.image_height), False)
        self.get_logger().info(f"Recording Depth video to {depth_filename}")

        # ROS2 Topic Subscribers
        self.create_subscription(PointCloud2, '/lidar/points', self.lidar_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Image, '/rs_front/image', self.image_callback, 10)
        self.create_subscription(Image, '/rs_front/depth_image', self.depth_callback, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)

    def timestamp(self):
        return datetime.now().strftime('%Y%m%d_%H%M%S')

    def lidar_callback(self, msg: PointCloud2):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        cloud_filename = os.path.join(
            self.data_dir, f"pointcloud_{self.timestamp()}.pcd")
        with open(cloud_filename, "w") as f:
            f.write("# .PCD v0.7 - Point Cloud Data file format\n")
            f.write("VERSION 0.7\n")
            f.write("FIELDS x y z\n")
            f.write("SIZE 4 4 4\n")
            f.write("TYPE F F F\n")
            f.write("COUNT 1 1 1\n")
            f.write(f"WIDTH {msg.width}\n")
            f.write(f"HEIGHT {msg.height}\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {msg.width * msg.height}\n")
            f.write("DATA ascii\n")
            for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                f.write(f"{p[0]} {p[1]} {p[2]}\n")

        with open(self.csv_file, "a", newline="") as f:
            writer = csv.writer(f)
            for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                writer.writerow([stamp, p[0], p[1], p[2]])

    def imu_callback(self, msg: Imu):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with open(self.imu_file, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                stamp,
                msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w,
                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
            ])

    def gps_callback(self, msg: NavSatFix):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with open(self.gps_file, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                stamp,
                msg.latitude,
                msg.longitude,
                msg.altitude,
                msg.position_covariance_type,
                *msg.position_covariance
            ])

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if (cv_image.shape[1], cv_image.shape[0]) != (self.image_width, self.image_height):
                cv_image = cv2.resize(cv_image, (self.image_width, self.image_height))
            self.rgb_video_writer.write(cv_image)
        except Exception as e:
            self.get_logger().error(f"Failed to write RGB frame: {e}")

    def depth_callback(self, msg: Image):
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            max_depth = 10.0
            cv_depth = np.clip(cv_depth, 0, max_depth)
            normalized = (cv_depth / max_depth * 255).astype(np.uint8)

            if (normalized.shape[1], normalized.shape[0]) != (self.image_width, self.image_height):
                normalized = cv2.resize(normalized, (self.image_width, self.image_height))

            self.depth_video_writer.write(normalized)
        except Exception as e:
            self.get_logger().error(f"Failed to write depth frame: {e}")

    def destroy_node(self):
        if self.rgb_video_writer is not None:
            self.rgb_video_writer.release()
        if self.depth_video_writer is not None:
            self.depth_video_writer.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RoboboatLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
