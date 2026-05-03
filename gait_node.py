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
GAIT_CYCLE_TIME = 1.0   # seconds per full cycle
PUBLISH_RATE   = 100.0  # Hz — how often foot targets are published
# SWING_HEIGHT   = 0.03   # meters: for dz-swing sim- how high feet lift during swing

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

        '''
        # For dz-swing sim: Diagonal trot pairs
        # Pair A: FR + BL,  Pair B: FL + BR
        self.pair_a = [0, 3]  # FR, BL — phase = 0
        self.pair_b = [1, 2]  # FL, BR — phase = pi (opposite)
        '''

        # Waypoints (relative to home) (adapted from Lab3)
        # 6 waypoints per leg per cycle:
        # touchdown - stand1 - stand2 - stand3 - liftoff - mid_swing
        touch_down  = np.array([ 0.03,  0,  0.00])  # front of stance, on ground
        stand_1     = np.array([ 0.015, 0,  0.00])
        stand_2     = np.array([ 0.00,  0,  0.00])
        stand_3     = np.array([-0.015, 0,  0.00])
        liftoff     = np.array([-0.03,  0,  0.00])  # back of stance, lifts off
        mid_swing   = np.array([ 0.00,  0,  0.05])  # peak of swing
 
        # Waypoint sequences per leg
        # Pair A (FR=0, BL=3): starts at touchdown
        # Pair B (FL=1, BR=2): starts at stand_3 (half cycle offset)
        self.waypoints = [
            np.array([touch_down, stand_1, stand_2, stand_3, liftoff, mid_swing]),  # FR
            np.array([stand_3, liftoff, mid_swing, touch_down, stand_1, stand_2]),  # FL
            np.array([stand_3, liftoff, mid_swing, touch_down, stand_1, stand_2]),  # BR
            np.array([touch_down, stand_1, stand_2, stand_3, liftoff, mid_swing]),  # BL
        ]

        self.start_time = self.get_clock().now()

        self.foot_target_pub = self.create_publisher(
            Float64MultiArray,
            "/foot_targets",
            10)

        self.timer = self.create_timer(
            1.0 / PUBLISH_RATE,
            self.publish_targets)

        # self.get_logger().info("Gait Node started — publishing trot pattern to /foot_targets")
        self.get_logger().info(
            f"Gait Node started — trot at {GAIT_CYCLE_TIME}s/cycle, publishing to /foot_targets"
        )

    def interpolate_waypoints(self, t_norm, leg_index):
        # Linearly interpolate between waypoints- adapted from Lab3
        # returns xyz offset from home pos
        waypts = self.waypoints[leg_index]
        n = len(waypts)                    # 6
        segment = t_norm * n               # which segment [0, 6)
        seg_idx = int(segment) % n         # segment index 0-5
        seg_t   = segment - int(segment)   # local t within segment [0, 1)
 
        p0 = waypts[seg_idx]
        p1 = waypts[(seg_idx + 1) % n]
        return p0 + seg_t * (p1 - p0)
        
    def publish_targets(self):
        current_time = self.get_clock().now()
        t = (current_time - self.start_time).nanoseconds * 1e-9  # seconds

        # normalized cycle time [0, 1)
        t_norm = (t % GAIT_CYCLE_TIME) / GAIT_CYCLE_TIME

        targets = self.feet_home.copy()

        # omega = 2.0 * np.pi * GAIT_FREQ  # angular frequency

        for i in range(4):
            offset = self.interpolate_waypoints(t_norm, i)
            targets[i][0] += offset[0]   # x: fore/aft swing
            targets[i][2] += offset[2]   # z: lift height
            # y unchanged — no lateral movement

            '''
            For dz-swing sim
            # Pair A is phase 0, Pair B is phase pi
            phase = 0.0 if i in self.pair_a else np.pi

            # Sinusoidal swing: only lift during positive half of sine wave
            # sin > 0 → foot is in swing phase (lifted)
            # sin <= 0 → foot is in stance phase (on ground, no z change)
            swing = np.sin(omega * t + phase)
            dz = SWING_HEIGHT * max(0.0, swing)  # only lift, never push into ground

            targets[i][2] += dz
            '''
  
        # Flatten to 12 floats: [FR_x, FR_y, FR_z, FL_x, FL_y, FL_z, ...]
        msg = Float64MultiArray()
        msg.data = targets.flatten().tolist()
        self.foot_target_pub.publish(msg)

        ## to log clean z position input: 
        # self.get_logger().info(
        #     f"GAIT_CSV,{t:.3f},{targets[0][2]:.6f},{targets[3][2]:.6f}"
        # )


def main(args=None):
    rclpy.init(args=args)
    node = GaitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Gait node shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()