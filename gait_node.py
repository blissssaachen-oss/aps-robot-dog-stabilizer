#!/usr/bin/env python3
"""
Trot Gait Node for Pupper v3
Publishes sinusoidal trot foot targets to /foot_targets
Posture stabilizer subscribes and applies correction on top.

Trot diagonal pairs:
  Pair A: FR (0) + BL (3)
  Pair B: FL (1) + BR (2)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

# Gait parameters
GAIT_FREQ      = 1.0    # Hz — full trot cycle frequency
SWING_HEIGHT   = 0.03   # meters — how high feet lift during swing
PUBLISH_RATE   = 100.0  # Hz — how often foot targets are published

class GaitNode(Node):

    def __init__(self):
        super().__init__("gait_node")

        # Home positions (x, y, z) in meters — must match posture_stabilizer
        self.feet_home = np.array([
            [ 0.06, -0.09, -0.14],  # FR (index 0)
            [ 0.06,  0.09, -0.14],  # FL (index 1)
            [-0.11, -0.09, -0.14],  # BR (index 2)
            [-0.11,  0.09, -0.14],  # BL (index 3)
        ])

        # Diagonal trot pairs — these swing in opposite phase
        # Pair A: FR + BL,  Pair B: FL + BR
        self.pair_a = [0, 3]  # FR, BL — phase = 0
        self.pair_b = [1, 2]  # FL, BR — phase = pi (opposite)

        self.start_time = self.get_clock().now()

        self.foot_target_pub = self.create_publisher(
            Float64MultiArray,
            "/foot_targets",
            10)

        self.timer = self.create_timer(
            1.0 / PUBLISH_RATE,
            self.publish_targets)

        self.get_logger().info("Gait Node started — publishing trot pattern to /foot_targets")

    def publish_targets(self):
        current_time = self.get_clock().now()
        t = (current_time - self.start_time).nanoseconds * 1e-9  # seconds

        targets = self.feet_home.copy()

        omega = 2.0 * np.pi * GAIT_FREQ  # angular frequency

        for i in range(4):
            # Pair A is phase 0, Pair B is phase pi
            phase = 0.0 if i in self.pair_a else np.pi

            # Sinusoidal swing: only lift during positive half of sine wave
            # sin > 0 → foot is in swing phase (lifted)
            # sin <= 0 → foot is in stance phase (on ground, no z change)
            swing = np.sin(omega * t + phase)
            dz = SWING_HEIGHT * max(0.0, swing)  # only lift, never push into ground

            targets[i][2] += dz

        # Flatten to 12 floats: [FR_x, FR_y, FR_z, FL_x, FL_y, FL_z, ...]
        msg = Float64MultiArray()
        msg.data = targets.flatten().tolist()
        self.foot_target_pub.publish(msg)

        # Optional logging — comment out to reduce noise
        # self.get_logger().info(f"t={t:.2f} targets published")


def main(args=None):
    rclpy.init(args=args)
    node = GaitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Gait node shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()