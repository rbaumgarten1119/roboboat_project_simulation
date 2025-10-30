import math
import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, NavSatFix
from std_msgs.msg import Float64
import sensor_msgs_py.point_cloud2 as pc2


class LidarGapGPSNavigator(Node):
    """Navigator that uses LiDAR gap detection for steering
    and GPS for approximate position logging.
    """

    def __init__(self):
        super().__init__('lidar_gap_gps_navigator')

        # === Thruster publishers ===
        self.left_thruster_pub = self.create_publisher(
            Float64,
            '/roboboatmodel/roboboat/joint/left_thruster_joint/cmd_thrust',
            10
        )
        self.right_thruster_pub = self.create_publisher(
            Float64,
            '/roboboatmodel/roboboat/joint/right_thruster_joint/cmd_thrust',
            10
        )

        # === Subscribers ===
        self.create_subscription(PointCloud2, '/lidar/points', self.lidar_callback, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)

        # === Control parameters ===
        self.safe_distance = 2.5     # meters
        self.max_thrust = 1.0
        self.turn_gain = 0.02
        self.forward_gain = 0.8

        # === State ===
        self.position = None
        self.ref_lat = None
        self.ref_lon = None
        self.cos_ref_lat = None

        self.get_logger().info(
            "Lidar + GPS navigator started (ROS 2 Jazzy / Gazebo Harmonic)."
        )

    # --------------------------------------------------
    # GPS callback (robust for Harmonic's near-zero drift)
    # --------------------------------------------------
    def gps_callback(self, msg: NavSatFix):
        lat = msg.latitude
        lon = msg.longitude
        status = msg.status.status

        # Ignore invalid or uninitialized fixes
        if status < 0:
            self.get_logger().warn("GPS: no valid fix yet (status < 0). Waiting…")
            return

        # Ignore clearly bogus coordinates near (0, 0)
        if abs(lat) < 1.0 and abs(lon) < 1.0:
            self.get_logger().warn(
                f"GPS: skipping unrealistic fix (lat={lat:.6e}, lon={lon:.6e})"
            )
            return

        # First valid fix → lock reference
        if self.ref_lat is None:
            self.ref_lat = lat
            self.ref_lon = lon
            self.cos_ref_lat = math.cos(math.radians(self.ref_lat))
            self.get_logger().info(
                f"GPS reference locked at lat={self.ref_lat:.8f}, lon={self.ref_lon:.8f}"
            )

        # Convert WGS84 (deg) → local ENU (m)
        d_lat = lat - self.ref_lat
        d_lon = lon - self.ref_lon
        meters_per_deg_lat = 111320.0
        meters_per_deg_lon = 111320.0 * self.cos_ref_lat

        north_m = d_lat * meters_per_deg_lat
        east_m = d_lon * meters_per_deg_lon

        self.position = {'x': east_m, 'y': north_m}

    # --------------------------------------------------
    # LiDAR callback: find biggest gap and drive through
    # --------------------------------------------------
    def lidar_callback(self, msg: PointCloud2):
        """Process LiDAR point cloud and command thrusters toward the largest open gap."""
        try:
            # read_points() already returns a generator of dict-like records
            gen = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
            pts_list = list(gen)
        except Exception as e:
            self.get_logger().error(f"LiDAR read_points() failed: {e}")
            self.stop_boat()
            return

        if not pts_list:
            self.get_logger().warn("LiDAR: empty cloud; stopping.")
            self.stop_boat()
            return

        # Convert structured array -> plain Nx3 float array
        # (each element is a tuple (x, y, z))
        try:
            pts = np.array([[p[0], p[1], p[2]] for p in pts_list], dtype=np.float32)
        except Exception as e:
            self.get_logger().error(f"LiDAR conversion failed: {e}")
            self.stop_boat()
            return

        # --- Continue with your existing gap logic ---
        x, y = pts[:, 0], pts[:, 1]
        angles = np.arctan2(y, x)
        dists = np.sqrt(x**2 + y**2)

        if len(dists) == 0:
            self.get_logger().warn("LiDAR: all NaNs or zero ranges; stopping.")
            self.stop_boat()
            return

        order = np.argsort(angles)
        angles, dists = angles[order], dists[order]

        free_mask = dists > self.safe_distance
        free_idx = np.where(free_mask)[0]
        if len(free_idx) == 0:
            self.get_logger().warn("LiDAR: no safe gap; stopping.")
            self.stop_boat()
            return

        gaps, start = [], free_idx[0]
        for i in range(1, len(free_idx)):
            if free_idx[i] != free_idx[i - 1] + 1:
                gaps.append((start, free_idx[i - 1]))
                start = free_idx[i]
        gaps.append((start, free_idx[-1]))

        gap_start, gap_end = max(gaps, key=lambda g: angles[g[1]] - angles[g[0]])
        mid_angle = 0.5 * (angles[gap_start] + angles[gap_end])

        turn = -self.turn_gain * mid_angle
        forward = self.forward_gain
        left = np.clip(forward + turn, -self.max_thrust, self.max_thrust)
        right = np.clip(forward - turn, -self.max_thrust, self.max_thrust)

        self.left_thruster_pub.publish(Float64(data=float(left)))
        self.right_thruster_pub.publish(Float64(data=float(right)))

        if self.position:
            self.get_logger().info(
                f"Pos (E,N)=({self.position['x']:.2f},{self.position['y']:.2f}) m | "
                f"Gap {math.degrees(mid_angle):.1f}° | L={left:.2f} R={right:.2f}"
            )
        else:
            self.get_logger().info(
                f"GPS pending | Gap {math.degrees(mid_angle):.1f}° | L={left:.2f} R={right:.2f}"
            )


    # --------------------------------------------------
    def stop_boat(self):
        self.left_thruster_pub.publish(Float64(data=0.0))
        self.right_thruster_pub.publish(Float64(data=0.0))


def main(args=None):
    rclpy.init(args=args)
    node = LidarGapGPSNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
