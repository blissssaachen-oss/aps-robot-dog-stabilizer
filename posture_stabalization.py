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

# controller mode selection
CONTROLLER = "P" # "P" or "ADRC"

class PostureStabilizer(Node):
    """
    ROS2 Node for active posture stabilization using PID + IK
    """
    # TODO

    def __init__(self):
        super().__init__("posture_stabilizer")

        self.ik = IKSolver()

        # time
        self.start_time = self.get_clock().now()
        self.prev_time = self.get_clock().now()

        # Definining home positions (x, y, z) in meters
        self.feet_home = np.array([
        [ 0.06, -0.09, -0.14],  # FR
        [ 0.06,  0.09, -0.14],  # FL
        [-0.11, -0.09, -0.14],  # BR
        [-0.11,  0.09, -0.14],  # BL
         ])
        
        # ADRC state variables
        self.rz1, self.rz2, self.rz3 = 0.0, 0.0, 0.0
        self.r_uprev, self.r_init    = 0.0, False
        self.pz1, self.pz2, self.pz3 = 0.0, 0.0, 0.0
        self.p_uprev, self.p_init    = 0.0, False

        # Subscriber that listens to Imu messages on '/imu/data'
        self.imu_sub = self.create_subscription(
            Imu, 
            "/imu_sensor_broadcaster/imu", 
            self.imu_callback, 
            10)
        
        # Publisher that sends angles to '/joint_commands'
        self.joint_command_pub = self.create_publisher(
            Float64MultiArray, 
            "/forward_command_controller/commands", 
            10)

        self.get_logger().info('Posture Stabilizer Node has started!')
    
    def imu_callback(self, msg):
        # starting timer for logging time
        current_time = self.get_clock().now()
        t = (current_time - self.start_time).nanoseconds * 1e-9  # seconds
        dt = (current_time - self.prev_time).nanoseconds * 1e-9
        self.prev_time = current_time

        # unpacking imu info (packaged as quaternion so turning to euler)
        q = msg.orientation
        rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
        roll, pitch, yaw = rot.as_euler('xyz')

        if CONTROLLER == "P":
            # GAINS
            K_p = 1.25
            # target angle
            theta = pitch 
            phi = roll

            # Rotation matrices
            R_y = np.array([ # pitch
                [np.cos(theta),  0, np.sin(theta)],
                [0,              1, 0            ],
                [-np.sin(theta), 0, np.cos(theta)]
            ])
            R_x = np.array([ #roll
                [1, 0,           0           ],
                [0, np.cos(phi), -np.sin(phi)],
                [0, np.sin(phi),  np.cos(phi)]
            ])
            # how much rotation has occurred to each foot
            R_total = R_y @ R_x 

            # produce target ee positions
            targets = []
            for i in range(4):
                rotated = R_total @ self.feet_home[i] # calculate new positions for all 4 feet
                error = rotated - self.feet_home[i] # difference between new and home pos
                correction = K_p * error # P-control
                new_target = self.feet_home[i] + correction # apply it to original pos
                targets.append(new_target)

        elif CONTROLLER == "ADRC":
            # GAINS
            b0 = 1.0  # model gain (~1.0)
            wc = 3.0  # desired closed-loop BW
            w0 = 12.0  # observer BW
            u_limit = 0.05
            # Derived GAINS
            kp = wc ** 2
            kd = 2.0 * wc
            # Observer GAINS
            beta1 = 3.0 * w0
            beta2 = 3.0 * w0**2
            beta3 = w0**3

            # roll ESO
            if not self.r_init:
                self.rz1 = roll;  self.r_init = True
            e = roll - self.rz1 # position error
            self.rz1 += dt * (self.rz2 + beta1 * e) # position/angle
            self.rz2 += dt * (self.rz3 + b0 * self.r_uprev + beta2 * e) # velocity
            self.rz3 += dt * (beta3 * e) # total ext disturbance
            u_roll = float(np.clip((kp * (0.0 - self.rz1) - kd * self.rz2 - self.rz3) / b0, -u_limit, u_limit))
            self.r_uprev = u_roll # for tracking last command

            # pitch ESO
            if not self.p_init:
                self.pz1 = pitch;  self.p_init = True
            e = pitch - self.pz1
            self.pz1 += dt * (self.pz2 + beta1 * e)
            self.pz2 += dt * (self.pz3 + b0 * self.p_uprev + beta2 * e)
            self.pz3 += dt * (beta3 * e)
            u_pitch = float(np.clip((kp * (0.0 - self.pz1) - kd * self.pz2 - self.pz3) / b0, -u_limit, u_limit))
            self.p_uprev = u_pitch
        
            # u_roll, u_pitch are correction angles in radians
            # lever arms convert angle correction → height delta per foot
            roll_lever  = 0.09   # half lateral stance width (y-distance foot to center)
            pitch_lever = 0.10   # half fore-aft stance length (x-distance foot to center)

            # produce target ee pos (leg mapping)
            targets = []
            for i in range(4):
                x_sign = 1.0 if self.feet_home[i][0] > 0 else -1.0  # front/back
                y_sign = 1.0 if self.feet_home[i][1] < 0 else -1.0  # right/left
                dz = (y_sign * u_roll * roll_lever) + (x_sign * u_pitch * pitch_lever)
                new_target = self.feet_home[i].copy()
                new_target[2] += dz
                targets.append(new_target)
    
        # finds all corresponding joint angles
        joint_angles = self.ik.solve_ik_all_legs(targets)

        #publishes to robot
        cmd = Float64MultiArray()
        cmd.data = joint_angles.tolist()
        self.joint_command_pub.publish(cmd)

        #logging
        self.get_logger().info(
            f"time={t:.2f}, roll={(np.degrees(roll) +0.35):.2f}, pitch={(np.degrees(pitch) -2.05):.2f}"
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


