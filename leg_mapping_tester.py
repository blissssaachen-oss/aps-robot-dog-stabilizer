#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

from ik_solver import IKSolver

np.set_printoptions(precision=3, suppress=True)


class LegMappingTester(Node):
    def __init__(self):
        super().__init__("leg_mapping_tester")

        self.ik = IKSolver()

        # Fake body tilt angle for testing
        self.test_roll_deg = 10.0
        self.test_pitch_deg = 10.0

        self.feet_home = np.array([
            [ 0.06, -0.09, -0.14],  # FR
            [ 0.06,  0.09, -0.14],  # FL
            [-0.11, -0.09, -0.14],  # BR
            [-0.11,  0.09, -0.14],  # BL
        ])

        self.joint_command_pub = self.create_publisher(
            Float64MultiArray,
            "/forward_command_controller/commands",
            10
        )

        # Runs every 0.1 seconds
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("Leg mapping tester started")

    def timer_callback(self):
        roll = np.radians(self.test_roll_deg)
        pitch = np.radians(self.test_pitch_deg)

        # Direct counter-rotation, no PID
        phi = -roll
        theta = -pitch

        R_y = np.array([
            [np.cos(theta),  0, np.sin(theta)],
            [0,              1, 0            ],
            [-np.sin(theta), 0, np.cos(theta)]
        ])

        R_x = np.array([
            [1, 0,           0           ],
            [0, np.cos(phi), -np.sin(phi)],
            [0, np.sin(phi),  np.cos(phi)]
        ])

        R_total = R_y @ R_x

        targets = []
        for foot in self.feet_home:
            new_target = R_total @ foot
            targets.append(new_target)

        joint_angles = self.ik.solve_ik_all_legs(targets)

        cmd = Float64MultiArray()
        cmd.data = joint_angles.tolist()
        self.joint_command_pub.publish(cmd)

        self.get_logger().info(
            f"fake_roll={self.test_roll_deg:.1f}, "
            f"fake_pitch={self.test_pitch_deg:.1f}, "
            f"targets={np.array(targets)}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = LegMappingTester()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping leg mapping tester")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()