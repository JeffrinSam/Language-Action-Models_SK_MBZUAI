import numpy as np
import math
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R


class PandaKinematics:
    def __init__(self):
        # Joint limits for Panda arm (in radians)
        self.joint_limits = [
            (-2.8973, 2.8973),   # joint1
            (-1.7628, 1.7628),   # joint2
            (-2.8973, 2.8973),   # joint3
            (-3.0718, -0.0698),  # joint4
            (-2.8973, 2.8973),   # joint5
            (-0.0175, 3.7525),   # joint6
            (-2.8973, 2.8973),   # joint7
        ]

        # DH parameters for Panda arm
        self.dh_params = [
            [0.0, 0.333, 0.0, 0.0],
            [0.0, 0.0, 0.0, -math.pi/2],
            [0.0, 0.316, 0.0, math.pi/2],
            [0.0825, 0.0, 0.0, math.pi/2],
            [-0.0825, 0.384, 0.0, -math.pi/2],
            [0.0, 0.0, 0.0, math.pi/2],
            [0.088, 0.0, 0.0, math.pi/2]
        ]

    def forward_kinematics(self, joints: np.ndarray) -> np.ndarray:
        T = np.eye(4)
        for i in range(7):
            a, d, alpha, theta_offset = self.dh_params[i]
            theta = joints[i] + theta_offset
            ca, sa = math.cos(alpha), math.sin(alpha)
            ct, st = math.cos(theta), math.sin(theta)
            T_i = np.array([
                [ct, -st * ca, st * sa, a * ct],
                [st, ct * ca, -ct * sa, a * st],
                [0, sa, ca, d],
                [0, 0, 0, 1]
            ])
            T = T @ T_i
        return T

    def inverse_kinematics(self, target_pose: np.ndarray, initial_joints: np.ndarray) -> np.ndarray:
        def cost_function(joints):
            current_pose = self.forward_kinematics(joints)
            pos_error = np.linalg.norm(current_pose[:3, 3] - target_pose[:3, 3])
            R_current = current_pose[:3, :3]
            R_target = target_pose[:3, :3]
            q_current = R.from_matrix(R_current).as_quat()
            q_target = R.from_matrix(R_target).as_quat()
            if np.dot(q_current, q_target) < 0:
                q_current = -q_current
            dot_product = float(np.dot(q_current, q_target))
            orient_error = 1.0 - abs(dot_product)
            joint_penalty = 0.0
            for i, joint in enumerate(joints):
                lo, hi = self.joint_limits[i]
                if joint < lo:
                    joint_penalty += 100.0 * (lo - joint) ** 2
                elif joint > hi:
                    joint_penalty += 100.0 * (joint - hi) ** 2
            return pos_error * 10.0 + orient_error * 2.0 + joint_penalty

        bounds = self.joint_limits
        res = minimize(
            cost_function,
            np.asarray(initial_joints, dtype=float),
            method='SLSQP',
            bounds=bounds,
            options={'maxiter': 200, 'ftol': 1e-6, 'disp': False},
        )
        if res.success:
            return res.x
        return np.asarray(initial_joints, dtype=float)

    @staticmethod
    def pose_from_xyzrpy(x, y, z, roll, pitch, yaw):
        T = np.eye(4)
        T[:3, 3] = [x, y, z]
        T[:3, :3] = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
        return T

