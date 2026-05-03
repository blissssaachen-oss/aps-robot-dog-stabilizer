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

# ==== controller mode selection =========================
CONTROLLER = "ADRC" # "P" or "ADRC"
# ========================================================

class PostureStabilizer(Node):
    """
    ROS2 Node for active posture stabilization using PID + IK
    """

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
        self.rz1, self.rz2, self.rz3 = 0.0, 0.0, 0.0 # position(angle) / velocity / total ext disturbance
        self.r_uprev, self.r_init    = 0.0, False
        self.pz1, self.pz2, self.pz3 = 0.0, 0.0, 0.0
        self.p_uprev, self.p_init    = 0.0, False
        self.r_was_saturated = False # to reduce sticky memory
        self.p_was_saturated = False

        # Subscriber that listens to Imu messages on '/imu/data'
        self.imu_sub = self.create_subscription(
            Imu, 
            "/imu_sensor_broadcaster/imu", 
            self.imu_callback, 
            10)
        
        # Subscriber that listens to updated foot targets from 'gait_node' (Gait mode only)
        self.foot_target_sub = self.create_subscription(
            Float64MultiArray,
            "/foot_targets",
            self.foot_targets_callback,
            10)
        
        # Publisher that sends angles to '/joint_commands'
        self.joint_command_pub = self.create_publisher(
            Float64MultiArray, 
            "/forward_command_controller/commands", 
            10)

        self.get_logger().info('Posture Stabilizer Node has started!')

    def foot_targets_callback(self, msg):
        # Reshape flat 12-float array back to (4, 3) foot positions
        self.feet_home = np.array(msg.data).reshape(4, 3)
    
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

        # # IMU sanity check
        # self.get_logger().info(
        # f"RAW IMU | roll={np.degrees(roll):.2f}, pitch={np.degrees(pitch):.2f}"
        # )

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
            wc = 2.0  # desired closed-loop BW: control effort, response speed (was 1.5)
            w0 = 10.0  # observer BW (>~5*wc): ESO speed (Nyquist: w0 < pi * fs(:945Hz) = 2969rad/s)
            
            # Derived GAINS
            kp = 1.4  # wc ** 2
            kd = 4.8 # 2.0 * wc 
            # Observer GAINS (Gao canonical parametrization for 3rd-order ESO)
            beta1 = 3.0 * w0
            beta2 = 0.5 * w0 # 3.0 * w0**2
            beta3 = 0.1 * w0 # was w0**3 but for slow z3 accum

            # clamping
            dt = np.clip(dt, 0.0005, 0.005)  # for IMU~945Hz, dt~0.001s
            u_limit = 0.25 # clamp range
            max_slew = 0.01 # rad/callback
            # setpoints (offset resting angles)
            roll_sp  = np.radians(0.975) 
            pitch_sp = np.radians(-0.52) 
            
            # ---ROLL ESO---
            if not self.r_init:
                self.rz1 = roll;  self.r_init = True
            e = roll - self.rz1  # position error

            # Anti-windup: prevent saturation
            u_roll_raw_prev = (kp * (roll_sp - self.rz1) - kd * self.rz2 - self.rz3) / b0
            saturated_roll = (self.r_uprev >= u_limit and u_roll_raw_prev > 0) or \
                            (self.r_uprev <= -u_limit and u_roll_raw_prev < 0)
            # Detect saturation exit and reset z2
            if self.r_was_saturated and not saturated_roll:
                self.rz2 = 0.0   # flush accum velocity estimate
            self.r_was_saturated = saturated_roll

            if not saturated_roll:
                self.rz1 += dt * (self.rz2 + beta1 * e) 
                self.rz2 += dt * (self.rz3 + b0 * self.r_uprev + beta2 * e) 
                self.rz2 = float(np.clip(self.rz2, -2.0, 2.0))  # hard cap: ~115°/s max body rate
                self.rz3 += dt * (beta3 * e)
            else: # if saturated, prevent windup accum
                self.rz1 += dt * (self.rz2 + beta1 * e) # only update z1 so it tracks position. freeze z2 and z3

            u_roll_raw = (kp * (roll_sp - self.rz1) - kd * self.rz2 - self.rz3) / b0
            u_roll_raw = float(np.clip(u_roll_raw, self.r_uprev-max_slew, self.r_uprev+max_slew )) # clamp slew (roc of command)
            u_roll = float(np.clip(u_roll_raw, -u_limit, u_limit)) # clamp magnitude (hard limit)
            self.r_uprev = u_roll # for tracking last command

            # ---PITCH ESO---
            if not self.p_init:
                self.pz1 = pitch;  self.p_init = True
            e = pitch - self.pz1 # position error

            # Anti-windup
            u_pitch_raw_prev = (kp * (pitch_sp - self.pz1) - kd * self.pz2 - self.pz3) / b0
            saturated_pitch = (self.p_uprev >= u_limit and u_pitch_raw_prev > 0) or \
                  (self.p_uprev <= -u_limit and u_pitch_raw_prev < 0)
            # Detect saturation exit and reset z2
            if self.p_was_saturated and not saturated_pitch:
                self.pz2 = 0.0   # flush accum velocity estimate
            self.p_was_saturated = saturated_pitch

            if not saturated_pitch:
                self.pz1 += dt * (self.pz2 + beta1 * e)
                self.pz2 += dt * (self.pz3 + b0 * self.p_uprev + beta2 * e)
                self.pz2 = float(np.clip(self.pz2, -2.0, 2.0))
                self.pz3 += dt * (beta3 * e)
            else: 
                self.pz1 += dt * (self.pz2 + beta1 * e)
            
            u_pitch_raw = (kp * (pitch_sp - self.pz1) - kd * self.pz2 - self.pz3) / b0
            u_pitch_raw = float(np.clip(u_pitch_raw, self.p_uprev-max_slew, self.p_uprev+max_slew))
            u_pitch = float(np.clip(u_pitch_raw, -u_limit, u_limit))
            self.p_uprev = u_pitch
            
            # lever arms convert angle correction → height delta per foot
            roll_lever  = 0.17/2   # half lateral stance width (y)
            pitch_lever = 0.17/2   # half fore-aft stance length (x)

            # produce target ee pos (leg mapping)
            targets = []
            for i in range(4):
                x_sign = 1.0 if self.feet_home[i][0] > 0 else -1.0  # front/back
                y_sign = 1.0 if self.feet_home[i][1] < 0 else -1.0  # right/left
                dz = (y_sign * u_roll * roll_lever) + (x_sign * u_pitch * pitch_lever)
                new_target = self.feet_home[i].copy()
                new_target[2] += dz
                targets.append(new_target)
            
            """
            ## control output mapping ver.2
            # Build rotation matrices from control outputs (same structure as P mode)
            # not suitable for ADRC on ground b/c ground contact fights x,y movement -> ESO sees persistent error.
            # must use dz method for ADRC
            # just keep it for now, will be useful for gait mode
            
            R_y = np.array([
                [ np.cos(u_pitch), 0, np.sin(u_pitch)],
                [ 0,               1, 0              ],
                [-np.sin(u_pitch), 0, np.cos(u_pitch)]
            ])
            R_x = np.array([
                [1, 0,              0             ],
                [0, np.cos(u_roll), -np.sin(u_roll)],
                [0, np.sin(u_roll),  np.cos(u_roll)]
            ])
            R_total = R_y @ R_x
            
            targets = []
            for i in range(4):
                rotated = R_total @ self.feet_home[i]
                delta = rotated - self.feet_home[i]

                new_target = self.feet_home[i].copy()
                new_target[2] += delta[2]        # z correction 
                new_target[0] += delta[0] * 0.3  # x correction (partial — fore/aft has some compliance)
                # new_target[1] += delta[1]      # y correction (skip — lateral is ground-constrained)
                targets.append(new_target)
            """
    
        # finds all corresponding joint angles
        joint_angles = self.ik.solve_ik_all_legs(targets)

        #publishes to robot
        cmd = Float64MultiArray()
        cmd.data = joint_angles.tolist()
        self.joint_command_pub.publish(cmd)

        #logging
        if CONTROLLER == "P":  
            self.get_logger().info(
                f"time={t:.2f}, roll={np.degrees(roll):.2f}, pitch={np.degrees(pitch):.2f}, "
                f"u_roll={np.degrees(K_p*roll):.2f}, u_pitch={np.degrees(K_p*pitch):.2f}"
                )

        elif CONTROLLER == "ADRC":
            #logging (ADRC specific)
            self.get_logger().info(
                # Below is for logging r/p + u_r/u_p(ADRC)
                f"time={t:.2f}, roll={(np.degrees(roll)):.2f}, pitch={(np.degrees(pitch)):.2f}, u_roll={(np.degrees(u_roll)):.2f}, u_pitch={(np.degrees(u_pitch)):.2f}"
            )
            self.get_logger().info(
                f"t={t:.2f} | roll={np.degrees(roll):.2f} "
                f"z1={np.degrees(self.rz1):.2f} z2={np.degrees(self.rz2):.2f} z3={self.rz3:.4f} "
                f"u={np.degrees(u_roll):.2f}"
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


