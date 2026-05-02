#!/usr/bin/env python3
"""
Active Posture Stabilization for Pupper v3
Uses PID controller + IK solver to maintain level orientation
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
import numpy as np
from scipy.spatial.transform import Rotation
import time

from ik_solver import IKSolver

np.set_printoptions(precision=3, suppress=True)


class PostureStabilizer(Node):
    """
    ROS2 Node for active posture stabilization using PID + IK
    """
    # TODO

    def __init__(self):
        super().__init__("posture_stabilizer")

        self.ik = IKSolver()
        self.start_time = self.get_clock().now()

        self.feet_home = np.array([
            [ 0.06, -0.09, -0.14],
            [ 0.06,  0.09, -0.14],
            [-0.11, -0.09, -0.14],
            [-0.11,  0.09, -0.14],
        ])

        # PD state
        self.kp = 1.1
        self.kd = 0.005
        self.prev_roll = 0.0
        self.prev_pitch = 0.0
        self.prev_time = None

        # bias state — populated during calibration, then frozen
        self.roll_bias = 0.0
        self.pitch_bias = 0.0
        self.calibrating = True
        self.calib_samples = []
        self.calib_target = 100  # number of samples to average

        # publisher first so it's ready
        self.joint_command_pub = self.create_publisher(
            Float64MultiArray,
            "/forward_command_controller/commands",
            10)

        # subscriber starts the calibration callback
        self.imu_sub = self.create_subscription(
            Imu,
            "/imu_sensor_broadcaster/imu",
            self.imu_callback,
            10)

        self.get_logger().info('Posture Stabilizer started — calibrating IMU, hold robot level...')


    def imu_callback(self, msg):
        q = msg.orientation
        rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
        roll, pitch, yaw = rot.as_euler('xyz')

        # calibration phase: collect samples, then freeze the average as bias
        if self.calibrating:
            self.calib_samples.append((roll, pitch))
            if len(self.calib_samples) >= self.calib_target:
                samples = np.array(self.calib_samples)
                self.roll_bias = float(np.mean(samples[:, 0]))
                self.pitch_bias = float(np.mean(samples[:, 1]))
                self.calibrating = False
                self.get_logger().info(
                    f"Calibration done: roll_bias={np.degrees(self.roll_bias):.3f}°, "
                    f"pitch_bias={np.degrees(self.pitch_bias):.3f}°"
                )
            return  # don't command the robot during calibration

        # normal control path
        current_time = self.get_clock().now()
        t = (current_time - self.start_time).nanoseconds * 1e-9

        if self.prev_time is None:
            dt = 0.01
        else:
            dt = (current_time - self.prev_time).nanoseconds * 1e-9
        self.prev_time = current_time
        if dt <= 0 or dt > 0.5:
            dt = 0.01

        roll  -= self.roll_bias
        pitch -= self.pitch_bias

        roll_rate  = (roll  - self.prev_roll)  / dt
        pitch_rate = (pitch - self.prev_pitch) / dt
        self.prev_roll = roll
        self.prev_pitch = pitch

        theta = self.kp * pitch + self.kd * pitch_rate
        phi   = self.kp * roll  + self.kd * roll_rate

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

        targets = [R_total @ self.feet_home[i] for i in range(4)]
        joint_angles = self.ik.solve_ik_all_legs(targets)

        cmd = Float64MultiArray()
        cmd.data = joint_angles.tolist()
        self.joint_command_pub.publish(cmd)

        self.get_logger().info(
            f"time={t:.2f}, roll={np.degrees(roll):.2f}, pitch={np.degrees(pitch):.2f}"
        )
        



def main(args=None):
    rclpy.init(args=args)
    
    node = PostureStabilizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down posture stabilizer...')
    finally:
        # Send zero commands to stop robot
        zero_cmd = Float64MultiArray()
        zero_cmd.data = [0.0] * 12
        node.joint_command_pub.publish(zero_cmd)
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


