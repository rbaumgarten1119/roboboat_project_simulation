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


def diag36(x, y, z, rr, pp, yy):
    cov = [0.0] * 36
    cov[0] = x
    cov[7] = y
    cov[14] = z
    cov[21] = rr
    cov[28] = pp
    cov[35] = yy
    return cov


class ThrusterToOdom(Node):
    def __init__(self):
        super().__init__('thruster_to_odom')

        # PARAMETERS
        self.declare_parameter('front_left_topic',  '/roboboatmodel/roboboat/joint/front_left_thruster_joint/cmd_thrust')
        self.declare_parameter('front_right_topic', '/roboboatmodel/roboboat/joint/front_right_thruster_joint/cmd_thrust')
        self.declare_parameter('back_left_topic',   '/roboboatmodel/roboboat/joint/back_left_thruster_joint/cmd_thrust')
        self.declare_parameter('back_right_topic',  '/roboboatmodel/roboboat/joint/back_right_thruster_joint/cmd_thrust')
        self.declare_parameter('imu_topic',         '/imu')

        self.declare_parameter('thrust_to_speed', 0.2)  # forward gain
        self.declare_parameter('yaw_gain',        0.4)  # turning gain
        self.declare_parameter('publish_rate',    20.0)

        # READ PARAMS
        self.front_left_topic  = self.get_parameter('front_left_topic').value
        self.front_right_topic = self.get_parameter('front_right_topic').value
        self.back_left_topic   = self.get_parameter('back_left_topic').value
        self.back_right_topic  = self.get_parameter('back_right_topic').value
        self.imu_topic         = self.get_parameter('imu_topic').value

        self.thrust_to_speed = float(self.get_parameter('thrust_to_speed').value)
        self.yaw_gain        = float(self.get_parameter('yaw_gain').value)
        publish_rate         = float(self.get_parameter('publish_rate').value)

        # STATE
        self.front_left = 0.0
        self.front_right = 0.0
        self.back_left = 0.0
        self.back_right = 0.0

        self.imu_yaw_rate = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = self.get_clock().now()
        self.yaw_rate_lpf = 0.0

        # SUBSCRIBERS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.create_subscription(Float64, self.front_left_topic,  self.cb_fl, 10)
        self.create_subscription(Float64, self.front_right_topic, self.cb_fr, 10)
        self.create_subscription(Float64, self.back_left_topic,   self.cb_bl, 10)
        self.create_subscription(Float64, self.back_right_topic,  self.cb_br, 10)
        self.create_subscription(Imu,     self.imu_topic,         self.cb_imu, qos)

        self.timer = self.create_timer(1.0 / publish_rate, self.update)

        self.get_logger().info("4-THRUSTER ODOM RUNNING")

    # CALLBACKS
    def cb_fl(self, msg): self.front_left = float(msg.data)
    def cb_fr(self, msg): self.front_right = float(msg.data)
    def cb_bl(self, msg): self.back_left = float(msg.data)
    def cb_br(self, msg): self.back_right = float(msg.data)

    def cb_imu(self, msg):
        self.imu_yaw_rate = float(msg.angular_velocity.z)

    # MAIN UPDATE
    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0 or dt > 1.0:
            self.last_time = now
            return

        # FORWARD SPEED from back thrusters ONLY
        forward_speed = (self.back_left + self.back_right) * 0.5 * self.thrust_to_speed

        # TURNING from front thrusters ONLY
        yaw_rate_thr = (self.front_right - self.front_left) * self.yaw_gain

        # IMU BLEND
        yaw_rate = 0.7 * self.imu_yaw_rate + 0.3 * yaw_rate_thr

        # LPF
        alpha = 0.2
        self.yaw_rate_lpf = (1 - alpha) * self.yaw_rate_lpf + alpha * yaw_rate
        yaw_rate = self.yaw_rate_lpf

        # INTEGRATE
        self.x   += forward_speed * math.cos(self.yaw) * dt
        self.y   += forward_speed * math.sin(self.yaw) * dt
        self.yaw += yaw_rate * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # ODOM MSG
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = 0.0

        q = tf_transformations.quaternion_from_euler(0.0, 0.0, self.yaw)
        odom.pose.pose.orientation = Quaternion(x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))

        # Make sure values are finite before sending to C
        if not all(map(math.isfinite, [forward_speed, yaw_rate])):
            self.get_logger().warn(
                f"Non-finite twist: forward_speed={forward_speed}, yaw_rate={yaw_rate}, skipping publish"
            )
            self.last_time = now
            return

        odom.twist.twist = Twist(
            linear=Vector3(x=float(forward_speed), y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=float(yaw_rate))
        )

        self.pub.publish(odom)
        self.last_time = now

def main(args=None):
    rclpy.init(args=args)
    node = ThrusterToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
