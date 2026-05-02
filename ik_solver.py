"""
Inverse Kinematics Solver for Pupper v3
Copied and adapted from lab_2.py and lab_3_playground.py
"""

import numpy as np
import scipy

np.set_printoptions(precision=3, suppress=True)


def rotation_x(angle):
    return np.array([
        [1, 0, 0, 0],
        [0, np.cos(angle), -np.sin(angle), 0],
        [0, np.sin(angle), np.cos(angle), 0],
        [0, 0, 0, 1]
    ])

def rotation_y(angle):
    return np.array([
        [np.cos(angle), 0, np.sin(angle), 0],
        [0, 1, 0, 0],
        [-np.sin(angle), 0, np.cos(angle), 0],
        [0, 0, 0, 1]
    ])

def rotation_z(angle):
    return np.array([
        [np.cos(angle), -np.sin(angle), 0, 0],
        [np.sin(angle), np.cos(angle), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def translation(x, y, z):
    return np.array([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ])


class IKSolver:
    """
    Solves IK for all 4 legs independently.
    """

    # TODO: change to IK for all 4 legs together?
    
    def __init__(self):
        """Initialize IK solver with forward kinematics for each leg"""
        self.fk_functions = [
            self.fr_leg_fk,  # Front Right
            self.fl_leg_fk,  # Front Left
            self.br_leg_fk,  # Back Right
            self.bl_leg_fk   # Back Left
        ]
        
    def fr_leg_fk(self, theta):
        """Front Right leg"""
        T_RF_0_1 = translation(0.07500, -0.08350, 0) @ rotation_x(1.57080) @ rotation_z(theta[0])
        T_RF_1_2 = rotation_y(-1.57080) @ rotation_z(theta[1])
        T_RF_2_3 = translation(0, -0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(theta[2])
        T_RF_3_ee = translation(0.06231, -0.06216, 0.01800)
        T_RF_0_ee = T_RF_0_1 @ T_RF_1_2 @ T_RF_2_3 @ T_RF_3_ee
        return T_RF_0_ee[:3, 3]

    def fl_leg_fk(self, theta):
        """Front Left leg"""
        T_LF_0_1 = translation(0.07500, 0.08350, 0) @ rotation_x(1.57080) @ rotation_z(-theta[0])
        T_LF_1_2 = rotation_y(-1.57080) @ rotation_z(theta[1])
        T_LF_2_3 = translation(0, -0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(-theta[2])
        T_LF_3_ee = translation(0.06231, -0.06216, -0.01800)
        T_LF_0_ee = T_LF_0_1 @ T_LF_1_2 @ T_LF_2_3 @ T_LF_3_ee
        return T_LF_0_ee[:3, 3]

    def br_leg_fk(self, theta):
        """Back Right leg"""
        T_RB_0_1 = translation(-0.07500, -0.07250, 0) @ rotation_x(1.57080) @ rotation_z(theta[0])
        T_RB_1_2 = rotation_y(-1.57080) @ rotation_z(theta[1])
        T_RB_2_3 = translation(0, -0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(theta[2])
        T_RB_3_ee = translation(0.06231, -0.06216, 0.01800)
        T_RB_0_ee = T_RB_0_1 @ T_RB_1_2 @ T_RB_2_3 @ T_RB_3_ee
        return T_RB_0_ee[:3, 3]

    def bl_leg_fk(self, theta):
        """Back Left leg"""
        T_LB_0_1 = translation(-0.07500, 0.07250, 0) @ rotation_x(1.57080) @ rotation_z(-theta[0])
        T_LB_1_2 = rotation_y(-1.57080) @ rotation_z(theta[1])
        T_LB_2_3 = translation(0, -0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(-theta[2])
        T_LB_3_ee = translation(0.06231, -0.06216, -0.01800)
        T_LB_0_ee = T_LB_0_1 @ T_LB_1_2 @ T_LB_2_3 @ T_LB_3_ee
        return T_LB_0_ee[:3, 3]

    def get_error_leg(self, theta, desired_position):
        error = desired_position - self.leg_forward_kinematics(theta)
        return error.dot(error)

    def solve_ik_single_leg(self, target_ee, leg_index, initial_guess=None):
        if initial_guess is None:
            initial_guess = [0.0, 0.0, 0.0]
            
        self.leg_forward_kinematics = self.fk_functions[leg_index]
        res = scipy.optimize.minimize(self.get_error_leg, initial_guess, args=(target_ee,), method='SLSQP')  # Sequential Least Squares Programming
        return res.x
    
    # experimental. not used.
    def solve_ik_all_legs(self, target_positions, initial_guesses=None):
        """
        Solve IK for all 4 legs simultaneously.
        
        Args:
            target_positions: List of 4 target positions [[x,y,z], [x,y,z], ...]
                             Order: FR, FL, BR, BL
            initial_guesses: List of 4 initial guesses (optional)
            
        Returns:
            Array of 12 joint angles [FR1, FR2, FR3, FL1, FL2, FL3, ...]
        """
        if initial_guesses is None:
            initial_guesses = [[0.0, 0.0, 0.0]] * 4
            
        joint_angles = []
        for leg_idx in range(4):
            angles = self.solve_ik_single_leg(
                target_positions[leg_idx],
                leg_idx,
                initial_guesses[leg_idx]
            )
            joint_angles.extend(angles)
            
        return np.array(joint_angles)


def test_ik_solver():
    """Test the IK solver with some example positions"""
    solver = IKSolver()
    
    # Test front right leg
    print("Testing IK Solver...")
    target_ee = [0.10, -0.09, -0.14]  # x, y, z in body frame
    joint_angles = solver.solve_ik_single_leg(target_ee, leg_index=0)
    
    # Verify with forward kinematics
    result_ee = solver.fr_leg_fk(joint_angles)
    
    print(f"Target EE: {target_ee}")
    print(f"Joint angles: {joint_angles}")
    print(f"Result EE: {result_ee}")
    print(f"Error: {np.linalg.norm(target_ee - result_ee):.6f} meters")


if __name__ == '__main__':
    test_ik_solver()
