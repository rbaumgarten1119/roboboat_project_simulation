#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


def force_to_rpm(force):
    F = abs(force)

    # cubic polynomial coefficients (18 V data)
    a = 26.64039342
    b = -333.18440480
    c = 1632.66667909
    d = 305.73021485

    rpm = a*F**3 + b*F**2 + c*F + d

    # preserve sign for reverse thrust
    return rpm if force >= 0 else -rpm


def rpm_to_radsec(rpm):
    """Convert RPM → rad/s."""
    return (rpm * 2 * math.pi) / 60.0


class ThrusterCommander(Node):
    def __init__(self):
        super().__init__("thruster_commander")

        # Thruster topics
        self.front_left_pub = self.create_publisher(
            Float64,
            "/roboboatmodel/roboboat/joint/front_left_thruster_joint/cmd_thrust",
            10)

        self.front_right_pub = self.create_publisher(
            Float64,
            "/roboboatmodel/roboboat/joint/front_right_thruster_joint/cmd_thrust",
            10)

        self.back_left_pub = self.create_publisher(
            Float64,
            "/roboboatmodel/roboboat/joint/back_left_thruster_joint/cmd_thrust",
            10)

        self.back_right_pub = self.create_publisher(
            Float64,
            "/roboboatmodel/roboboat/joint/back_right_thruster_joint/cmd_thrust",
            10)

    def compute_and_send(self, x, y, theta):

        # 1. magnitude
        m = math.sqrt(x*x + y*y)

        # 2. vector components
        x_comp = (m * math.cos(theta)) / 2.0
        y_comp = (m * math.sin(theta)) / 2.0

        # forces to each thruster
        F_FL = x_comp
        F_FR = x_comp
        F_BL = y_comp
        F_BR = y_comp

        # 3. convert force → RPM → rad/s
        FL_cmd = rpm_to_radsec(force_to_rpm(F_FL))
        FR_cmd = rpm_to_radsec(force_to_rpm(F_FR))
        BL_cmd = rpm_to_radsec(force_to_rpm(F_BL))
        BR_cmd = rpm_to_radsec(force_to_rpm(F_BR))

        # 4. publish
        self.front_left_pub.publish(Float64(data=FL_cmd))
        self.front_right_pub.publish(Float64(data=FR_cmd))
        self.back_left_pub.publish(Float64(data=BL_cmd))
        self.back_right_pub.publish(Float64(data=BR_cmd))

        print("\n--- SENT COMMANDS ---")
        print(f"Front Left  (rad/s): {FL_cmd}")
        print(f"Front Right (rad/s): {FR_cmd}")
        print(f"Back Left   (rad/s): {BL_cmd}")
        print(f"Back Right  (rad/s): {BR_cmd}")
        print("---------------------\n")


def main():
    rclpy.init()

    node = ThrusterCommander()

    print("\nThruster Control Started.")
    print("Enter 'exit' at any time to quit.\n")

    while rclpy.ok():

        # x
        x_in = input("Enter x component (or 'exit'): ")
        if x_in.lower() == "exit":
            break

        # y
        y_in = input("Enter y component (or 'exit'): ")
        if y_in.lower() == "exit":
            break

        # theta
        th_in = input("Enter theta in radians (or 'exit'): ")
        if th_in.lower() == "exit":
            break

        try:
            x = float(x_in)
            y = float(y_in)
            theta = float(th_in)
        except ValueError:
            print("\nInvalid input. Must be numbers or 'exit'. Try again.\n")
            continue

        # compute + publish commands
        node.compute_and_send(x, y, theta)

        # ROS executor run callbacks briefly
        rclpy.spin_once(node, timeout_sec=0.05)

    print("\nShutting down thruster control...")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
