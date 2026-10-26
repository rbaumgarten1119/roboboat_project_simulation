#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist, Vector3
import tf_transformations


def _diag36(x, y, z, rr, pp, yy):
    """Build a 6x6 diag (flattened 36) for pose/twist covariance."""
    cov = [0.0] * 36
    cov[0]  = x   # xx
    cov[7]  = y   # yy
    cov[14] = z   # zz
    cov[21] = rr  # rr
    cov[28] = pp  # pp
    cov[35] = yy  # yy
    return cov


class ThrusterToOdom(Node):
    def __init__(self):
        super().__init__('thruster_to_odom')

        # -----------------------------
        # Parameters (override in launch/YAML if needed)
        # -----------------------------
        self.declare_parameter('left_topic',  '/roboboatmodel/roboboat/joint/back_left_thruster_joint/cmd_thrust')
        self.declare_parameter('right_topic', '/roboboatmodel/roboboat/joint/back_right_thruster_joint/cmd_thrust')
        self.declare_parameter('imu_topic',   '/imu')

        self.declare_parameter('thrust_to_speed', 0.2)  # m/s per unit thrust (tune)
        self.declare_parameter('wheel_base',      0.6)  # meters between thrusters (tune)
        self.declare_parameter('publish_rate',    20.0) # Hz

        # Reasonable covariance defaults (tune as you validate)
        self.declare_parameter('pose_cov_xy',     0.05)
        self.declare_parameter('pose_cov_z',      0.10)
        self.declare_parameter('pose_cov_rp',     0.10)
        self.declare_parameter('pose_cov_yaw',    0.20)

        self.declare_parameter('twist_cov_vxy',   0.02)
        self.declare_parameter('twist_cov_vz',    0.05)
        self.declare_parameter('twist_cov_wrp',   0.05)
        self.declare_parameter('twist_cov_wz',    0.10)

        # Read params
        self.left_topic       = self.get_parameter('left_topic').get_parameter_value().string_value
        self.right_topic      = self.get_parameter('right_topic').get_parameter_value().string_value
        self.imu_topic        = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.thrust_to_speed  = float(self.get_parameter('thrust_to_speed').value)
        self.wheel_base       = float(self.get_parameter('wheel_base').value)
        publish_rate          = float(self.get_parameter('publish_rate').value)

        # Covs
        pc_xy  = float(self.get_parameter('pose_cov_xy').value)
        pc_z   = float(self.get_parameter('pose_cov_z').value)
        pc_rp  = float(self.get_parameter('pose_cov_rp').value)
        pc_yaw = float(self.get_parameter('pose_cov_yaw').value)

        tc_vxy = float(self.get_parameter('twist_cov_vxy').value)
        tc_vz  = float(self.get_parameter('twist_cov_vz').value)
        tc_wrp = float(self.get_parameter('twist_cov_wrp').value)
        tc_wz  = float(self.get_parameter('twist_cov_wz').value)

        self.pose_cov  = _diag36(pc_xy, pc_xy, pc_z, pc_rp, pc_rp, pc_yaw)
        self.twist_cov = _diag36(tc_vxy, tc_vxy, tc_vz, tc_wrp, tc_wrp, tc_wz)

        # -----------------------------
        # State
        # -----------------------------
        self.left = 0.0
        self.right = 0.0
        self.imu_yaw_rate = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = self.get_clock().now()
        self.yaw_rate_lpf = 0.0  # simple low-pass on yaw rate

        # -----------------------------
        # QoS and I/O
        # -----------------------------
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.create_subscription(Float64, self.left_topic,  self.left_cb,  10)
        self.create_subscription(Float64, self.right_topic, self.right_cb, 10)
        self.create_subscription(Imu,     self.imu_topic,   self.imu_cb,   sensor_qos)

        self.timer = self.create_timer(1.0 / max(1e-3, publish_rate), self.timer_cb)

        self.get_logger().info(f"thruster_to_odom started. left='{self.left_topic}', right='{self.right_topic}', imu='{self.imu_topic}'")

    # --------- Callbacks ---------
    def left_cb(self, msg: Float64):
        self.left = float(msg.data)

    def right_cb(self, msg: Float64):
        self.right = float(msg.data)

    def imu_cb(self, msg: Imu):
        # Only use z angular velocity (assuming IMU frame aligns with base_link axes)
        self.imu_yaw_rate = float(msg.angular_velocity.z)

    # --------- Main update ---------
    def timer_cb(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0 or dt > 1.0:
            # Guard against time jumps / pauses
            self.last_time = now
            return

        # Forward speed from average thrust
        forward_speed = (self.left + self.right) * 0.5 * self.thrust_to_speed

        # Differential yaw rate from thrust difference
        yaw_rate_thr = 0.0
        if self.wheel_base > 1e-6:
            yaw_rate_thr = (self.right - self.left) / self.wheel_base * 0.5

        # Fuse IMU yaw rate with thruster-derived rate (simple complementary blend)
        yaw_rate = 0.7 * self.imu_yaw_rate + 0.3 * yaw_rate_thr

        # Low-pass yaw rate to avoid spikes
        alpha = 0.2
        self.yaw_rate_lpf = (1.0 - alpha) * self.yaw_rate_lpf + alpha * yaw_rate
        yaw_rate = self.yaw_rate_lpf

        # Integrate pose (planar)
        self.x   += forward_speed * math.cos(self.yaw) * dt
        self.y   += forward_speed * math.sin(self.yaw) * dt
        self.yaw += yaw_rate * dt

        # Normalize yaw to [-pi, pi]
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # Compose odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        qx, qy, qz, qw = tf_transformations.quaternion_from_euler(0.0, 0.0, self.yaw)
        odom.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        odom.twist.twist = Twist(
            linear=Vector3(x=forward_speed, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=yaw_rate)
        )

        # Non-zero covariances so EKF stays numerically healthy
        odom.pose.covariance  = self.pose_cov
        odom.twist.covariance = self.twist_cov

        # Basic sanity check (avoid publishing NaNs)
        if not all(map(math.isfinite, [
            self.x, self.y, self.yaw, forward_speed, yaw_rate, qx, qy, qz, qw
        ])):
            self.get_logger().warn("Non-finite value detected; skipping odom publish this tick.")
            self.last_time = now
            return

        self.pub.publish(odom)
        self.last_time = now


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
