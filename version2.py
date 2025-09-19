#!/usr/bin/env python3
"""
Version 2: Franka Panda Data Collection with Pose Matrix Format
- Saves data as record_trail_XXXX.csv with incremental numbering
- Includes end-effector position p = [x, y, z]^T and quaternion q = [qx, qy, qz, qw]
- Based on 4x4 pose matrix: T = [[R(q), p], [0^T, 1]]
"""

import os
os.environ['RMW_FASTRTPS_USE_SHM_TRANSPORT'] = '0'

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import argparse
import csv
import numpy as np
import time
from scipy.spatial.transform import Rotation as R

# ---------------------------------------------------------------------------- #
# USER-TUNABLE PARAMETERS                                                      #
# ---------------------------------------------------------------------------- #
DEFAULT_OBJECT_B = (0.4, 0.2, 0.065)  # Default object position
DEFAULT_BIN_POS = (0.4, -0.3, 0.5)    # Default bin position

LOOPS = 5                               # Number of episodes per task
TRAJ_DT = 0.01                          # Time step
TRAJ_TIME = 6.0                         # Trajectory time
GRIPPER_TIME = 2.0                      # Gripper operation time
WAIT_FIRST = 5.0                        # Wait for topics
DROP_Z = 0.0005                         # Final drop height
WAIT_BETWEEN = 4.0                      # Pause between phases
PRECISION_STEPS = 3                     # Number of precision descent steps
PRECISION_STEP_SIZE = 0.002             # Each precision step size (meters)

# Task definitions
TASK_PICKUP = "pick_up"
TASK_LIFT = "lift"
TASK_PLACE = "place_in_bin"
TASK_COMPLETE_PNP = "complete_pick_place"
TASK_PLACE_TRAY = "place_on_tray"
TASK_STACK_CUBE = "stack_cube"
TASK_UNSTACK_CUBE = "unstack_cube"
TASK_OPEN_DRAWER = "open_drawer"
TASK_CLOSE_DRAWER = "close_drawer"
TASK_PUSH_BUTTON = "push_button"

# Trajectory randomization
RANDOMIZE_TRAJECTORY = True
APPROACH_HEIGHT_RANGE = (0.08, 0.15)
LIFT_HEIGHT_RANGE = (0.12, 0.20)
PLACE_APPROACH_RANGE = (0.08, 0.15)
TRAJECTORY_SPEED_RANGE = (1.5, 2.5)
WAYPOINT_JITTER = 0.02

# Advanced trajectory algorithms
TRAJECTORY_ALGORITHM = 'mixed'

# Speed control from environment variable
TASK_SPEED_MULTIPLIER = float(os.environ.get('TASK_SPEED_MULTIPLIER', '1.0'))
RRT_MAX_ITERATIONS = 50
RRT_STEP_SIZE = 0.05
RRT_GOAL_THRESHOLD = 0.02
GAUSSIAN_NOISE_SCALE = 0.03
GAUSSIAN_NUM_WAYPOINTS = 5
BSPLINE_DEGREE = 3
BSPLINE_NUM_CONTROL_POINTS = 8
CUBIC_SPLINE_NUM_POINTS = 6
TRAJECTORY_CURVATURE_RANGE = (0.1, 0.8)
TRAJECTORY_SPEED_VARIATION = 0.3

# Additional randomization parameters
APPROACH_ANGLE_RANGE = (-0.3, 0.3)
LIFT_OFFSET_RANGE = (0.02, 0.08)
PLACE_OFFSET_RANGE = (0.01, 0.05)
TRAJECTORY_SMOOTHNESS_RANGE = (0.1, 0.9)
ORIENTATION_VARIATION = 0.2

# ---------------------------------------------------------------------------- #
# FK/IK Functions                                                              #
# ---------------------------------------------------------------------------- #
DH = np.array([[0.333, 0.0, 0.0],
               [0.0, -np.pi/2, 0.0],
               [0.316, np.pi/2, 0.0825],
               [0.0, np.pi/2, -0.0825],
               [0.384, -np.pi/2, 0.0],
               [0.0, np.pi/2, 0.088],
               [0.107, np.pi/2, 0.0]])

JOINT_MIN = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
JOINT_MAX = np.array([ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973])

def fk(q):
    """Forward kinematics returning position p and pose matrix T."""
    T = np.eye(4)
    for i in range(7):
        d, alpha, a = DH[i, 0], DH[i, 1], DH[i, 2]
        theta = q[i]
        ct, st, ca, sa = np.cos(theta), np.sin(theta), np.cos(alpha), np.sin(alpha)
        Ti = np.array([[ct, -st, 0, a],
                       [st*ca, ct*ca, -sa, -sa*d],
                       [st*sa, ct*sa, ca, ca*d],
                       [0, 0, 0, 1]])
        T = T @ Ti
    T = T @ np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0],
                      [0, 0, 1, 0.103],
                      [0, 0, 0, 1]])
    xyz = T[:3, 3]  # Position p = [x, y, z]^T
    return xyz, T

def _vee(M):
    return np.array([M[2, 1] - M[1, 2], M[0, 2] - M[2, 0], M[1, 0] - M[0, 1]])

def _log_so3(R):
    tr = float(np.trace(R))
    cos_theta = max(-1.0, min(1.0, 0.5 * (tr - 1.0)))
    theta = float(np.arccos(cos_theta))
    if theta < 1e-9:
        return np.zeros(3)
    return (theta / (2.0 * np.sin(theta))) * _vee(R - R.T)

def se3_jacobian(q):
    delta = 1e-5
    J = np.zeros((6, 7))
    for i in range(7):
        dq_plus = q.copy(); dq_plus[i] += delta
        dq_minus = q.copy(); dq_minus[i] -= delta
        p_plus, T_plus = fk(dq_plus)
        p_minus, T_minus = fk(dq_minus)
        J[:3, i] = (p_plus - p_minus) / (2.0 * delta)
        dR = T_plus[:3, :3] @ T_minus[:3, :3].T
        rotvec = _log_so3(dR) / (2.0 * delta)
        J[3:, i] = rotvec
    return J

def ik(xyz_target, q0=None, max_iter=300, tol=1e-4, damping=1e-3, max_step=0.15, 
       avoid_limits=True, null_gain=0.05, seeds=5, keep_orientation=True, ori_weight=1.0):
    if q0 is None:
        q0 = np.zeros(7)

    def solve_from(seed_q):
        q = np.clip(seed_q.copy(), JOINT_MIN, JOINT_MAX)
        if keep_orientation:
            _, T_seed = fk(q)
            R_des = T_seed[:3, :3].copy()
        for _ in range(max_iter):
            xyz, T = fk(q)
            R = T[:3, :3]
            err_pos = xyz_target - xyz
            if keep_orientation:
                err_rot = _log_so3(R_des @ R.T)
            else:
                err_rot = np.zeros(3)
            if keep_orientation and ori_weight > 0.0:
                err = np.hstack([err_pos, ori_weight * err_rot])
            else:
                err = err_pos
            if np.linalg.norm(err) < tol:
                return q
            if keep_orientation and ori_weight > 0.0:
                J = se3_jacobian(q)
                J[3:, :] *= ori_weight
                m = 6
            else:
                J = se3_jacobian(q)[:3, :]
                m = 3
            JJt = J @ J.T
            H = JJt + (damping ** 2) * np.eye(m)
            try:
                J_p = J.T @ np.linalg.inv(H)
            except np.linalg.LinAlgError:
                J_p = np.linalg.pinv(J)
            dq_task = J_p @ err
            dq_null = np.zeros(7)
            if avoid_limits and null_gain > 0.0:
                mid = 0.5 * (JOINT_MIN + JOINT_MAX)
                span = (JOINT_MAX - JOINT_MIN)
                grad = 2.0 * (q - mid) / (span ** 2 + 1e-8)
                N = np.eye(7) - J_p @ J
                dq_null = (-null_gain) * (N @ grad)
            dq = dq_task + dq_null
            step_norm = np.linalg.norm(dq)
            if step_norm > max_step:
                dq *= (max_step / (step_norm + 1e-9))
            alpha = 1.0
            ok = False
            for _ls in range(6):
                q_try = np.clip(q + alpha * dq, JOINT_MIN, JOINT_MAX)
                xyz_try, _ = fk(q_try)
                if np.linalg.norm(xyz_target - xyz_try) < np.linalg.norm(err):
                    q = q_try
                    ok = True
                    break
                alpha *= 0.5
            if not ok:
                q = np.clip(q + 0.1 * dq, JOINT_MIN, JOINT_MAX)
        return None

    seeds_list = [q0, np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])]
    rng = np.random.default_rng()
    for _ in range(max(0, seeds - 2)):
        jitter = rng.uniform(-0.2, 0.2, size=7)
        seeds_list.append(np.clip(q0 + jitter, JOINT_MIN, JOINT_MAX))

    best_q = None
    best_err = np.inf
    for s in seeds_list:
        q_sol = solve_from(s)
        if q_sol is not None:
            xyz_sol, _ = fk(q_sol)
            e = np.linalg.norm(xyz_target - xyz_sol)
            if e < best_err:
                best_err = e
                best_q = q_sol
                if e < tol:
                    break
    return best_q

# ---------------------------------------------------------------------------- #
# Trajectory Generation                                                        #
# ---------------------------------------------------------------------------- #
def linear_traj(q0, q1, dt, T):
    n = int(T / dt)
    traj = []
    for i in range(n + 1):
        alpha = i / n
        q = q0 + alpha * (q1 - q0)
        traj.append(q)
    return traj

def go_to(node, q_goal, dt=TRAJ_DT, T=TRAJ_TIME):
    """Blocking motion to q_goal."""
    q0 = node.current_q[:7].copy()
    traj = linear_traj(q0, q_goal, dt, T)
    
    for q in traj:
        node.send_joint_command(list(q), node.gripper_pos)
        try:
            rclpy.spin_once(node, timeout_sec=0.0)
        except Exception:
            pass
        time.sleep(dt)
    node.current_q[:7] = q_goal

def set_gripper(node, pos, dt=TRAJ_DT, T=GRIPPER_TIME):
    """Single-shot gripper command."""
    g = float(pos)
    node.send_joint_command(list(node.current_q[:7]), g)
    node.gripper_pos = g
    _spin_sleep(node, T)

def _spin_sleep(node: Node, duration: float, step: float = 0.05):
    """Sleep while pumping callbacks."""
    t_end = time.time() + max(0.0, duration)
    while time.time() < t_end:
        try:
            rclpy.spin_once(node, timeout_sec=0.0)
        except Exception:
            pass
        time.sleep(step)

# ---------------------------------------------------------------------------- #
# Main Node                                                                    #
# ---------------------------------------------------------------------------- #
class Version2Node(Node):
    def __init__(self):
        super().__init__('version2_node')
        self.pub = self.create_publisher(JointState, '/joint_command', 10)
        
        # Subscribe to object and bin topics
        self.sub_b = self.create_subscription(String, '/topicB', lambda msg: self.string_cb(msg, 'B'), 10)
        self.sub_end = self.create_subscription(String, '/topicend', lambda msg: self.string_cb(msg, 'end'), 10)
        self.state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 50)

        # Current state
        self.current_q = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
        self.measured_q = self.current_q.copy()
        self.measured_q_dot = np.zeros(7)
        self.measured_torques = np.zeros(7)
        self.measured_gripper = 0.0
        self.gripper_pos = 0.0
        self.gripper_velocity = 0.0
        self.gripper_torque = 0.0
        
        # Object and bin positions
        self.object_pose = None
        self.bin_pose = None
        
        self.loops = LOOPS
        self.rng = np.random.default_rng()
        
        # CSV logging with incremental numbering
        self._ts = time.strftime('%Y%m%d_%H%M%S')
        self._t0 = None
        self.csv_path = None
        self.csv_file = None
        self.csv_writer = None
        self.phase = 'init'
        self.episode_idx = 0
        self.trail_counter = 1  # Start from 1, will increment

    def get_next_trail_filename(self, task_name: str):
        """Get next available trail filename with incremental naming in task folder."""
        # Create task-specific folder path
        task_folder = os.path.join("/media/isr-lab-4/Main/jeff_ws/BImanuel/frankapanda", "version2_data", task_name)
        os.makedirs(task_folder, exist_ok=True)
        
        while True:
            filename = f"record_trail_{self.trail_counter:04d}.csv"
            filepath = os.path.join(task_folder, filename)
            if not os.path.exists(filepath):
                return filepath
            self.trail_counter += 1

    def open_episode_csv(self, episode_idx: int, task_name: str):
        self.episode_idx = int(episode_idx)
        self._t0 = time.time()
        
        # Get incremental trail filename
        self.csv_path = self.get_next_trail_filename(task_name)
        
        if self.csv_file:
            try:
                self.csv_file.close()
            except Exception:
                pass
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # CSV header matching record_trail_0012.csv format
        header = ['time']
        # O_T_EE[0] through O_T_EE[15] (4x4 transformation matrix flattened)
        for i in range(16):
            header.append(f'O_T_EE[{i}]')
        # q[0] through q[6] (7 joint positions)
        for i in range(7):
            header.append(f'q[{i}]')
        self.csv_writer.writerow(header)
        self.csv_file.flush()
        
        return self.csv_path

    def string_cb(self, msg: String, topic_type: str):
        try:
            x, y, z = map(float, msg.data.strip().split(','))
        except ValueError:
            self.get_logger().warn(f'Bad string on /topic{topic_type}: {msg.data}')
            return
        
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'panda_link0'
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        ps.pose.orientation.w = 1.0
        
        if topic_type == 'end':
            self.bin_pose = ps
            if not hasattr(self, '_printed_bin_position'):
                print(f'Bin position: {x:.3f}, {y:.3f}, {z:.3f}')
                self._printed_bin_position = True
        else:
            self.object_pose = ps
            if not hasattr(self, '_printed_object_position'):
                print(f'Object B position: ({x:.3f},{y:.3f},{z:.3f})')
                self._printed_object_position = True

    def send_joint_command(self, arm_positions, finger):
        """Publish JointState for arm + finger."""
        msg = JointState()
        msg.name = [f'panda_joint{i+1}' for i in range(7)] + ['panda_finger_joint1']
        msg.position = list(arm_positions) + [float(finger)]
        self.pub.publish(msg)

    def joint_state_cb(self, msg: JointState):
        """Record measured joint states and calculate pose matrix components."""
        try:
            name_to_idx = {n: i for i, n in enumerate(msg.name)}
            
            # Extract joint positions
            q = []
            q_dot = []
            torques = []
            
            for i in range(7):
                n = f'panda_joint{i+1}'
                if n in name_to_idx:
                    idx = name_to_idx[n]
                    q.append(msg.position[idx])
                    
                    if len(msg.velocity) > idx:
                        q_dot.append(msg.velocity[idx])
                    else:
                        q_dot.append(0.0)
                    
                    if len(msg.effort) > idx:
                        torques.append(msg.effort[idx])
                    else:
                        torques.append(0.0)
                else:
                    return
            
            self.measured_q = np.array(q)
            self.measured_q_dot = np.array(q_dot)
            self.measured_torques = np.array(torques)
            
            # Extract gripper data
            g_pos = None
            g_vel = None
            g_torque = None
            
            if 'panda_finger_joint1' in name_to_idx:
                idx = name_to_idx['panda_finger_joint1']
                g_pos = msg.position[idx]
                if len(msg.velocity) > idx:
                    g_vel = msg.velocity[idx]
                if len(msg.effort) > idx:
                    g_torque = msg.effort[idx]
            else:
                finger_indices = [i for n, i in name_to_idx.items() if 'finger' in n]
                if finger_indices:
                    g_pos = float(sum(msg.position[i] for i in finger_indices) / len(finger_indices))
                    if len(msg.velocity) > max(finger_indices):
                        g_vel = float(sum(msg.velocity[i] for i in finger_indices) / len(finger_indices))
                    if len(msg.effort) > max(finger_indices):
                        g_torque = float(sum(msg.effort[i] for i in finger_indices) / len(finger_indices))
            
            if g_pos is None:
                g_pos = self.gripper_pos
            if g_vel is None:
                g_vel = self.gripper_velocity
            if g_torque is None:
                g_torque = self.gripper_torque
                
            self.measured_gripper = float(g_pos)
            self.gripper_velocity = float(g_vel)
            self.gripper_torque = float(g_torque)

            # Emit CSV row matching record_trail_0012.csv format
            if self.csv_writer is not None and self._t0 is not None:
                t_rel = time.time() - self._t0
                
                # Calculate 4x4 transformation matrix T
                pos_xyz, T = fk(self.measured_q)
                
                # Flatten the 4x4 matrix to 16 elements (O_T_EE[0] through O_T_EE[15])
                T_flat = T.flatten()
                
                # Create row: time + O_T_EE[0-15] + q[0-6]
                row = [f"{t_rel:.6f}"]  # time
                row.extend([float(T_flat[i]) for i in range(16)])  # O_T_EE[0-15]
                row.extend([float(self.measured_q[i]) for i in range(7)])  # q[0-6]
                
                self.csv_writer.writerow(row)
                self.csv_file.flush()
        except Exception as e:
            if not hasattr(self, '_log_err_once'):
                self.get_logger().warn(f'/joint_states log error: {e}')
                self._log_err_once = True

    def get_object_position(self):
        """Get object position with fallback to default."""
        if self.object_pose is not None:
            return np.array([self.object_pose.pose.position.x,
                             self.object_pose.pose.position.y,
                             self.object_pose.pose.position.z])
        return np.array(DEFAULT_OBJECT_B)

    def get_bin_position(self):
        """Get bin position with fallback to default."""
        if self.bin_pose is not None:
            return np.array([self.bin_pose.pose.position.x,
                             self.bin_pose.pose.position.y,
                             self.bin_pose.pose.position.z])
        return np.array(DEFAULT_BIN_POS)

    def execute_pickup_task(self, episode_params):
        """Execute pickup task."""
        pick_xyz = self.get_object_position()
        jitter = episode_params['waypoint_jitter']
        pick_xyz += np.array([jitter, jitter, 0.0])
        
        approach_xyz = pick_xyz + np.array([0, 0, episode_params['approach_height']])
        
        q_approach = ik(approach_xyz, self.current_q[:7])
        if q_approach is None:
            self.get_logger().warn('IK failed on approach – skipping')
            return False
            
        q_pick = ik(pick_xyz, self.current_q[:7])
        if q_pick is None:
            self.get_logger().warn('IK failed on pick – skipping')
            return False

        traj_time = episode_params['traj_time']
        
        # Approach
        self.phase = 'approach_pick'
        self.get_logger().info('Moving to approach (pick)...')
        go_to(self, q_approach, T=traj_time)
        _spin_sleep(self, WAIT_BETWEEN)
        
        # Open gripper
        self.phase = 'gripper_open_before_pick'
        self.get_logger().info('Opening gripper (before pick)...')
        set_gripper(self, 0.04)
        _spin_sleep(self, WAIT_BETWEEN)
        
        # Pick
        self.phase = 'pick'
        self.get_logger().info('Moving to pick...')
        go_to(self, q_pick, T=traj_time)
        _spin_sleep(self, WAIT_BETWEEN)
        
        # Close gripper
        self.phase = 'gripper_close'
        self.get_logger().info('Closing gripper...')
        set_gripper(self, 0.0)
        _spin_sleep(self, WAIT_BETWEEN)

        return True

    def execute_lift_task(self, episode_params):
        """Execute lift task."""
        pick_xyz = self.get_object_position()
        jitter = episode_params['waypoint_jitter']
        pick_xyz += np.array([jitter, jitter, 0.0])
        
        lift_xyz = pick_xyz + np.array([0, 0, episode_params['lift_height']])
        
        q_pick = ik(pick_xyz, self.current_q[:7])
        if q_pick is None:
            self.get_logger().warn('IK failed on pick – skipping')
            return False
            
        q_lift = ik(lift_xyz, self.current_q[:7])
        if q_lift is None:
            self.get_logger().warn('IK failed on lift – skipping')
            return False

        traj_time = episode_params['traj_time']
        
        # Move to pick position
        self.phase = 'move_to_pick'
        self.get_logger().info('Moving to pick position...')
        go_to(self, q_pick, T=traj_time)
        _spin_sleep(self, WAIT_BETWEEN)
        
        # Close gripper
        self.phase = 'gripper_close'
        self.get_logger().info('Closing gripper...')
        set_gripper(self, 0.0)
        _spin_sleep(self, WAIT_BETWEEN)
        
        # Lift
        self.phase = 'lift_object'
        self.get_logger().info('Lifting object...')
        go_to(self, q_lift, T=traj_time)
        _spin_sleep(self, WAIT_BETWEEN)

        return True

    def execute_place_task(self, episode_params):
        """Execute place task."""
        bin_xyz = self.get_bin_position()
        jitter = episode_params['waypoint_jitter']
        bin_xyz += np.array([jitter, jitter, 0.0])
        
        place_approach_xyz = bin_xyz + np.array([0, 0, episode_params['place_approach_height']])
        place_lower_xyz = np.array([bin_xyz[0], bin_xyz[1], bin_xyz[2] + DROP_Z - 0.12])
        
        q_place_approach = ik(place_approach_xyz, self.current_q[:7])
        if q_place_approach is None:
            self.get_logger().warn('IK failed on place-approach – skipping')
            return False
            
        q_place = ik(bin_xyz, self.current_q[:7])
        if q_place is None:
            self.get_logger().warn('IK failed on place – skipping')
            return False
            
        q_place_lower = ik(place_lower_xyz, self.current_q[:7])
        if q_place_lower is None:
            min_z = bin_xyz[2] + DROP_Z - 0.12
            for dz in [-0.020, -0.015, -0.010, -0.005, 0.0]:
                alt_z = max(min_z, float(bin_xyz[2] + dz))
                alt_xyz = np.array([bin_xyz[0], bin_xyz[1], alt_z])
                q_place_lower = ik(alt_xyz, self.current_q[:7])
                if q_place_lower is not None:
                    place_lower_xyz = alt_xyz
                    self.get_logger().warn(f'place-lower fallback to z={alt_z:.3f} m')
                    break

        traj_time = episode_params['traj_time']
        
        # Approach place
        self.phase = 'approach_place'
        self.get_logger().info('Moving to approach (place)...')
        go_to(self, q_place_approach, T=traj_time)
        _spin_sleep(self, WAIT_BETWEEN)
        
        # Place
        self.phase = 'place'
        self.get_logger().info('Moving to place...')
        go_to(self, q_place, T=traj_time)
        _spin_sleep(self, WAIT_BETWEEN)
        
        # Final drop
        if q_place_lower is not None:
            self.phase = 'place_final_drop'
            self.get_logger().info('Final precision drop...')
            go_to(self, q_place_lower, T=traj_time/2)
            _spin_sleep(self, WAIT_BETWEEN)
            
        # Open gripper
        self.phase = 'gripper_open_at_place'
        self.get_logger().info('Opening gripper at place...')
        set_gripper(self, 0.04)
        _spin_sleep(self, WAIT_BETWEEN)

        return True

    def execute_complete_pnp_task(self, episode_params):
        """Execute complete pick and place task."""
        pick_xyz = self.get_object_position()
        bin_xyz = self.get_bin_position()
        jitter = episode_params['waypoint_jitter']
        
        # Add randomization to positions
        pick_xyz += np.array([jitter, jitter, 0.0])
        bin_xyz += np.array([jitter, jitter, 0.0])
        
        # Pick up sequence
        pick_approach_xyz = pick_xyz + np.array([0, 0, episode_params['approach_height']])
        q_pick_approach = ik(pick_approach_xyz, self.current_q[:7])
        if q_pick_approach is None:
            self.get_logger().warn('IK failed on pick-approach – skipping')
            return False
            
        q_pick = ik(pick_xyz, self.current_q[:7])
        if q_pick is None:
            self.get_logger().warn('IK failed on pick – skipping')
            return False
            
        # Place sequence
        place_approach_xyz = bin_xyz + np.array([0, 0, episode_params['place_approach_height']])
        place_lower_xyz = np.array([bin_xyz[0], bin_xyz[1], bin_xyz[2] + DROP_Z - 0.12])
        
        q_place_approach = ik(place_approach_xyz, self.current_q[:7])
        if q_place_approach is None:
            self.get_logger().warn('IK failed on place-approach – skipping')
            return False
            
        q_place = ik(bin_xyz, self.current_q[:7])
        if q_place is None:
            self.get_logger().warn('IK failed on place – skipping')
            return False
            
        q_place_lower = ik(place_lower_xyz, self.current_q[:7])
        if q_place_lower is None:
            min_z = bin_xyz[2] + DROP_Z - 0.12
            for dz in [-0.020, -0.015, -0.010, -0.005, 0.0]:
                alt_z = max(min_z, float(bin_xyz[2] + dz))
                alt_xyz = np.array([bin_xyz[0], bin_xyz[1], alt_z])
                q_place_lower = ik(alt_xyz, self.current_q[:7])
                if q_place_lower is not None:
                    place_lower_xyz = alt_xyz
                    break
        
        # Execute complete sequence
        traj_time = episode_params['traj_time']
        
        # Pick up
        go_to(self, q_pick_approach, T=traj_time)
        _spin_sleep(self, WAIT_BETWEEN)
        
        go_to(self, q_pick, T=traj_time)
        _spin_sleep(self, WAIT_BETWEEN)
        
        set_gripper(self, 0.0)
        _spin_sleep(self, WAIT_BETWEEN)
        
        go_to(self, q_pick_approach, T=traj_time)
        _spin_sleep(self, WAIT_BETWEEN)
        
        # Place
        go_to(self, q_place_approach, T=traj_time)
        _spin_sleep(self, WAIT_BETWEEN)
        
        go_to(self, q_place_lower, T=traj_time)
        _spin_sleep(self, WAIT_BETWEEN)
        
        go_to(self, q_place, T=traj_time)
        _spin_sleep(self, WAIT_BETWEEN)
        
        set_gripper(self, 0.04)
        _spin_sleep(self, WAIT_BETWEEN)
        
        go_to(self, q_place_approach, T=traj_time)
        _spin_sleep(self, WAIT_BETWEEN)

        return True

    def execute_place_tray_task(self, episode_params):
        """Execute place on tray task."""
        pick_xyz = self.get_object_position()
        jitter = episode_params['waypoint_jitter']
        pick_xyz += np.array([jitter, jitter, 0.0])
        
        # Define tray position (randomized)
        tray_offset = self.rng.uniform(-0.1, 0.1, 2)  # Random x,y offset
        tray_xyz = np.array([0.5 + tray_offset[0], 0.0 + tray_offset[1], 0.05])
        
        # Approach and place sequence
        tray_approach_xyz = tray_xyz + np.array([0, 0, episode_params['approach_height']])
        
        q_tray_approach = ik(tray_approach_xyz, self.current_q[:7])
        if q_tray_approach is None:
            self.get_logger().warn('IK failed on tray-approach – skipping')
            return False
            
        q_tray = ik(tray_xyz, self.current_q[:7])
        if q_tray is None:
            self.get_logger().warn('IK failed on tray-place – skipping')
            return False
            
        traj_time = episode_params['traj_time']
        
        # Execute sequence
        go_to(self, q_tray_approach, T=traj_time)
        _spin_sleep(self, WAIT_BETWEEN)
        
        go_to(self, q_tray, T=traj_time)
        _spin_sleep(self, WAIT_BETWEEN)
        
        set_gripper(self, 0.04)
        _spin_sleep(self, WAIT_BETWEEN)
        
        go_to(self, q_tray_approach, T=traj_time)
        _spin_sleep(self, WAIT_BETWEEN)

        return True

    def execute_stack_cube_task(self, episode_params):
        """Execute stack cube task."""
        pick_xyz = self.get_object_position()
        jitter = episode_params['waypoint_jitter']
        pick_xyz += np.array([jitter, jitter, 0.0])
        
        # Define stacking position (randomized)
        stack_offset = self.rng.uniform(-0.05, 0.05, 2)  # Random x,y offset
        stack_xyz = np.array([0.3 + stack_offset[0], 0.2 + stack_offset[1], 0.15])  # Higher for stacking
        
        # Approach and stack sequence
        stack_approach_xyz = stack_xyz + np.array([0, 0, episode_params['approach_height']])
        
        q_stack_approach = ik(stack_approach_xyz, self.current_q[:7])
        if q_stack_approach is None:
            self.get_logger().warn('IK failed on stack-approach – skipping')
            return False
            
        q_stack = ik(stack_xyz, self.current_q[:7])
        if q_stack is None:
            self.get_logger().warn('IK failed on stack-place – skipping')
            return False
            
        traj_time = episode_params['traj_time']
        
        # Execute sequence
        go_to(self, q_stack_approach, T=traj_time)
        _spin_sleep(self, WAIT_BETWEEN)
        
        go_to(self, q_stack, T=traj_time)
        _spin_sleep(self, WAIT_BETWEEN)
        
        set_gripper(self, 0.04)
        _spin_sleep(self, WAIT_BETWEEN)
        
        go_to(self, q_stack_approach, T=traj_time)
        _spin_sleep(self, WAIT_BETWEEN)

        return True

    def execute_unstack_cube_task(self, episode_params):
        """Execute unstack cube task."""
        # Define stacked cube position (randomized)
        stack_offset = self.rng.uniform(-0.05, 0.05, 2)  # Random x,y offset
        stacked_xyz = np.array([0.3 + stack_offset[0], 0.2 + stack_offset[1], 0.15])  # Higher position
        
        # Define unstack destination (randomized)
        unstack_offset = self.rng.uniform(-0.1, 0.1, 2)  # Random x,y offset
        unstack_xyz = np.array([0.4 + unstack_offset[0], -0.1 + unstack_offset[1], 0.05])
        
        # Approach and unstack sequence
        stacked_approach_xyz = stacked_xyz + np.array([0, 0, episode_params['approach_height']])
        unstack_approach_xyz = unstack_xyz + np.array([0, 0, episode_params['approach_height']])
        
        q_stacked_approach = ik(stacked_approach_xyz, self.current_q[:7])
        if q_stacked_approach is None:
            self.get_logger().warn('IK failed on stacked-approach – skipping')
            return False
            
        q_stacked = ik(stacked_xyz, self.current_q[:7])
        if q_stacked is None:
            self.get_logger().warn('IK failed on stacked-pick – skipping')
            return False
            
        q_unstack_approach = ik(unstack_approach_xyz, self.current_q[:7])
        if q_unstack_approach is None:
            self.get_logger().warn('IK failed on unstack-approach – skipping')
            return False
            
        q_unstack = ik(unstack_xyz, self.current_q[:7])
        if q_unstack is None:
            self.get_logger().warn('IK failed on unstack-place – skipping')
            return False
            
        traj_time = episode_params['traj_time']
        
        # Execute sequence
        go_to(self, q_stacked_approach, T=traj_time)
        _spin_sleep(self, WAIT_BETWEEN)
        
        go_to(self, q_stacked, T=traj_time)
        _spin_sleep(self, WAIT_BETWEEN)
        
        set_gripper(self, 0.0)
        _spin_sleep(self, WAIT_BETWEEN)
        
        go_to(self, q_stacked_approach, T=traj_time)
        _spin_sleep(self, WAIT_BETWEEN)
        
        go_to(self, q_unstack_approach, T=traj_time)
        _spin_sleep(self, WAIT_BETWEEN)
        
        go_to(self, q_unstack, T=traj_time)
        _spin_sleep(self, WAIT_BETWEEN)
        
        set_gripper(self, 0.04)
        _spin_sleep(self, WAIT_BETWEEN)
        
        go_to(self, q_unstack_approach, T=traj_time)
        _spin_sleep(self, WAIT_BETWEEN)

        return True

    def execute_open_drawer_task(self, episode_params):
        """Execute open drawer task."""
        # Define drawer handle position (randomized)
        drawer_offset = self.rng.uniform(-0.05, 0.05, 2)  # Random x,y offset
        drawer_handle_xyz = np.array([0.2 + drawer_offset[0], 0.4 + drawer_offset[1], 0.1])
        
        # Define drawer opening trajectory (pull motion)
        drawer_open_offset = self.rng.uniform(-0.1, -0.05, 1)  # Pull back 5-10cm
        drawer_open_xyz = np.array([drawer_handle_xyz[0] + drawer_open_offset[0], 
                                   drawer_handle_xyz[1], 
                                   drawer_handle_xyz[2]])
        
        # Approach and open sequence
        drawer_approach_xyz = drawer_handle_xyz + np.array([0, 0, episode_params['approach_height']])
        
        q_drawer_approach = ik(drawer_approach_xyz, self.current_q[:7])
        if q_drawer_approach is None:
            self.get_logger().warn('IK failed on drawer-approach – skipping')
            return False
            
        q_drawer_grasp = ik(drawer_handle_xyz, self.current_q[:7])
        if q_drawer_grasp is None:
            self.get_logger().warn('IK failed on drawer-grasp – skipping')
            return False
            
        q_drawer_open = ik(drawer_open_xyz, self.current_q[:7])
        if q_drawer_open is None:
            self.get_logger().warn('IK failed on drawer-open – skipping')
            return False
            
        traj_time = episode_params['traj_time']
        
        # Execute sequence
        go_to(self, q_drawer_approach, T=traj_time)
        _spin_sleep(self, WAIT_BETWEEN)
        
        go_to(self, q_drawer_grasp, T=traj_time)
        _spin_sleep(self, WAIT_BETWEEN)
        
        set_gripper(self, 0.0)  # Close gripper to grasp handle
        _spin_sleep(self, WAIT_BETWEEN)
        
        go_to(self, q_drawer_open, T=traj_time)
        _spin_sleep(self, WAIT_BETWEEN)
        
        set_gripper(self, 0.04)  # Open gripper to release handle
        _spin_sleep(self, WAIT_BETWEEN)
        
        go_to(self, q_drawer_approach, T=traj_time)
        _spin_sleep(self, WAIT_BETWEEN)

        return True

    def execute_close_drawer_task(self, episode_params):
        """Execute close drawer task."""
        # Define drawer handle position (randomized) - drawer is already open
        drawer_offset = self.rng.uniform(-0.05, 0.05, 2)  # Random x,y offset
        drawer_open_xyz = np.array([0.15 + drawer_offset[0], 0.4 + drawer_offset[1], 0.1])  # Open position
        
        # Define drawer closing trajectory (push motion)
        drawer_close_offset = self.rng.uniform(0.05, 0.1, 1)  # Push forward 5-10cm
        drawer_close_xyz = np.array([drawer_open_xyz[0] + drawer_close_offset[0], 
                                   drawer_open_xyz[1], 
                                   drawer_open_xyz[2]])
        
        # Approach and close sequence
        drawer_approach_xyz = drawer_open_xyz + np.array([0, 0, episode_params['approach_height']])
        
        q_drawer_approach = ik(drawer_approach_xyz, self.current_q[:7])
        if q_drawer_approach is None:
            self.get_logger().warn('IK failed on drawer-approach – skipping')
            return False
            
        q_drawer_grasp = ik(drawer_open_xyz, self.current_q[:7])
        if q_drawer_grasp is None:
            self.get_logger().warn('IK failed on drawer-grasp – skipping')
            return False
            
        q_drawer_close = ik(drawer_close_xyz, self.current_q[:7])
        if q_drawer_close is None:
            self.get_logger().warn('IK failed on drawer-close – skipping')
            return False
            
        traj_time = episode_params['traj_time']
        
        # Execute sequence
        go_to(self, q_drawer_approach, T=traj_time)
        _spin_sleep(self, WAIT_BETWEEN)
        
        go_to(self, q_drawer_grasp, T=traj_time)
        _spin_sleep(self, WAIT_BETWEEN)
        
        set_gripper(self, 0.0)  # Close gripper to grasp handle
        _spin_sleep(self, WAIT_BETWEEN)
        
        go_to(self, q_drawer_close, T=traj_time)
        _spin_sleep(self, WAIT_BETWEEN)
        
        set_gripper(self, 0.04)  # Open gripper to release handle
        _spin_sleep(self, WAIT_BETWEEN)
        
        go_to(self, q_drawer_approach, T=traj_time)
        _spin_sleep(self, WAIT_BETWEEN)

        return True

    def execute_push_button_task(self, episode_params):
        """Execute push button task."""
        # Define button position (randomized)
        button_offset = self.rng.uniform(-0.05, 0.05, 2)  # Random x,y offset
        button_xyz = np.array([0.3 + button_offset[0], 0.0 + button_offset[1], 0.2])
        
        # Define button press trajectory (push down motion)
        button_press_offset = self.rng.uniform(-0.02, -0.01, 1)  # Press down 1-2cm
        button_press_xyz = np.array([button_xyz[0], 
                                   button_xyz[1], 
                                   button_xyz[2] + button_press_offset[0]])
        
        # Approach and press sequence
        button_approach_xyz = button_xyz + np.array([0, 0, episode_params['approach_height']])
        
        q_button_approach = ik(button_approach_xyz, self.current_q[:7])
        if q_button_approach is None:
            self.get_logger().warn('IK failed on button-approach – skipping')
            return False
            
        q_button_press = ik(button_xyz, self.current_q[:7])
        if q_button_press is None:
            self.get_logger().warn('IK failed on button-press – skipping')
            return False
            
        q_button_push = ik(button_press_xyz, self.current_q[:7])
        if q_button_push is None:
            self.get_logger().warn('IK failed on button-push – skipping')
            return False
            
        traj_time = episode_params['traj_time']
        
        # Execute sequence
        go_to(self, q_button_approach, T=traj_time)
        _spin_sleep(self, WAIT_BETWEEN)
        
        go_to(self, q_button_press, T=traj_time)
        _spin_sleep(self, WAIT_BETWEEN)
        
        go_to(self, q_button_push, T=traj_time)
        _spin_sleep(self, WAIT_BETWEEN)
        
        # Hold button press briefly
        _spin_sleep(self, 1.0)
        
        go_to(self, q_button_press, T=traj_time)
        _spin_sleep(self, WAIT_BETWEEN)
        
        go_to(self, q_button_approach, T=traj_time)
        _spin_sleep(self, WAIT_BETWEEN)

        return True

    def generate_episode_params(self):
        """Generate randomized parameters for this episode."""
        if not RANDOMIZE_TRAJECTORY:
            return {
                'approach_height': 0.10,
                'lift_height': 0.15,
                'place_approach_height': 0.10,
                'traj_time': TRAJ_TIME / TASK_SPEED_MULTIPLIER,
                'waypoint_jitter': 0.0
            }
        
        params = {
            'approach_height': self.rng.uniform(*APPROACH_HEIGHT_RANGE),
            'lift_height': self.rng.uniform(*LIFT_HEIGHT_RANGE),
            'place_approach_height': self.rng.uniform(*PLACE_APPROACH_RANGE),
            'traj_time': self.rng.uniform(*TRAJECTORY_SPEED_RANGE) / TASK_SPEED_MULTIPLIER,
            'waypoint_jitter': self.rng.uniform(-WAYPOINT_JITTER, WAYPOINT_JITTER),
            'episode_seed': self.rng.integers(0, 10000)
        }
        
        self.rng = np.random.default_rng(params['episode_seed'])
        return params

    def run_task(self, task_name):
        """Run specific task with 10 episodes."""
        self.current_task = task_name
        self.get_logger().info(f'Starting {task_name} task with {self.loops} episodes')
        self.get_logger().info('Waiting for object and bin positions (max %.1f s) ...' % WAIT_FIRST)
        t0 = time.time()
        
        # Wait for object and bin position
        while rclpy.ok() and (time.time() - t0) < WAIT_FIRST:
            rclpy.spin_once(self, timeout_sec=0.1)
            
            if self.object_pose is not None and self.bin_pose is not None:
                break

        # Use defaults if no data received
        if self.object_pose is None:
            self.get_logger().warn('No object data → using default object position')
            self.object_pose = PoseStamped()
            self.object_pose.pose.position.x = DEFAULT_OBJECT_B[0]
            self.object_pose.pose.position.y = DEFAULT_OBJECT_B[1]
            self.object_pose.pose.position.z = DEFAULT_OBJECT_B[2]

        if self.bin_pose is None:
            self.get_logger().warn('No bin data → using default bin position')
            self.bin_pose = PoseStamped()
            self.bin_pose.pose.position.x = DEFAULT_BIN_POS[0]
            self.bin_pose.pose.position.y = DEFAULT_BIN_POS[1]
            self.bin_pose.pose.position.z = DEFAULT_BIN_POS[2]
        
        self.get_logger().info(f'Starting {task_name} collection loops.')
        for k in range(self.loops):
            self.get_logger().info(f'=== {task_name.upper()} Episode {k+1}/{self.loops} ===')
            
            # Generate episode-specific parameters
            episode_params = self.generate_episode_params()
            self.get_logger().info(f'Episode params: {episode_params}')
            
            # Open CSV for this episode
            csv_file_path = self.open_episode_csv(k + 1, task_name)
            print(f'📁 {task_name.upper()} Episode {k+1} CSV file: {csv_file_path}')

            # Execute task-specific sequence
            success = False
            if task_name == TASK_PICKUP:
                success = self.execute_pickup_task(episode_params)
            elif task_name == TASK_LIFT:
                success = self.execute_lift_task(episode_params)
            elif task_name == TASK_PLACE:
                success = self.execute_place_task(episode_params)
            elif task_name == TASK_COMPLETE_PNP:
                success = self.execute_complete_pnp_task(episode_params)
            elif task_name == TASK_PLACE_TRAY:
                success = self.execute_place_tray_task(episode_params)
            elif task_name == TASK_STACK_CUBE:
                success = self.execute_stack_cube_task(episode_params)
            elif task_name == TASK_UNSTACK_CUBE:
                success = self.execute_unstack_cube_task(episode_params)
            elif task_name == TASK_OPEN_DRAWER:
                success = self.execute_open_drawer_task(episode_params)
            elif task_name == TASK_CLOSE_DRAWER:
                success = self.execute_close_drawer_task(episode_params)
            elif task_name == TASK_PUSH_BUTTON:
                success = self.execute_push_button_task(episode_params)

            if success:
                self.get_logger().info(f'{task_name} sequence completed successfully')
            else:
                self.get_logger().warn(f'{task_name} sequence failed')

            # Return to home
            q_home = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
            self.phase = 'home'
            self.get_logger().info('Going home...')
            go_to(self, q_home, T=episode_params['traj_time'])
            _spin_sleep(self, WAIT_BETWEEN)

            self.get_logger().info(f'{task_name.upper()} Episode {k+1} completed')
            
            # Close CSV for this episode
            try:
                if self.csv_file:
                    self.csv_file.close()
                    print(f'✅ {task_name.upper()} Episode {k+1} data saved: {csv_file_path}')
                    print(f'📊 File size: {os.path.getsize(csv_file_path)} bytes')
            except Exception as e:
                print(f'❌ Error saving {task_name} episode {k+1}: {e}')

            # Pause for Isaac Sim environment reset
            pause_time = 5.0 / TASK_SPEED_MULTIPLIER
            print(f'⏳ Pausing {pause_time:.1f} seconds for Isaac Sim environment reset...')
            time.sleep(pause_time)

        print(f'\n🎉 {task_name.upper()} Data Collection Complete!')
        print(f'📊 Total Episodes: {self.loops}')
        print(f'📁 Data saved with incremental trail naming: record_trail_XXXX.csv')
        print(f'✅ Perfect! All {self.loops} episodes completed successfully!')
        
        self.get_logger().info(f'All {task_name} episodes finished. Shutting down.')

# ---------------------------------------------------------------------------- #
# Main Function                                                                #
# ---------------------------------------------------------------------------- #
def main():
    parser = argparse.ArgumentParser(description='Version 2: Franka Panda Data Collection with Pose Matrix Format')
    parser.add_argument('--auto', action='store_true', help='Run automatic episode sequence')
    parser.add_argument('--task', choices=['pick_up', 'lift', 'place_in_bin', 'complete_pick_place', 'place_on_tray', 'stack_cube', 'unstack_cube', 'open_drawer', 'close_drawer', 'push_button'], 
                       help='Specific task to run (pick_up, lift, place_in_bin, complete_pick_place, place_on_tray, stack_cube, unstack_cube, open_drawer, close_drawer, push_button)')
    args = parser.parse_args()

    rclpy.init()
    node = Version2Node()
    
    try:
        if args.auto and args.task:
            print(f"🤖 Running {args.task.upper()} task in AUTOMATIC mode")
            node.run_task(args.task)
        else:
            # Interactive choice
            print("\n🎯 Choose your task:")
            print("1. Pick Up Task (5 episodes)")
            print("2. Lift Task (5 episodes)")
            print("3. Place in Bin Task (5 episodes)")
            print("4. Complete Pick and Place Task (5 episodes)")
            print("5. Place on Tray Task (5 episodes)")
            print("6. Stack Cube Task (5 episodes)")
            print("7. Unstack Cube Task (5 episodes)")
            print("8. Open Drawer Task (5 episodes)")
            print("9. Close Drawer Task (5 episodes)")
            print("10. Push Button Task (5 episodes)")
            
            while True:
                choice = input("\nEnter your choice (1-10): ").strip()
                if choice == '1':
                    print("🤖 Starting PICK UP task...")
                    node.run_task(TASK_PICKUP)
                    break
                elif choice == '2':
                    print("🤖 Starting LIFT task...")
                    node.run_task(TASK_LIFT)
                    break
                elif choice == '3':
                    print("🤖 Starting PLACE IN BIN task...")
                    node.run_task(TASK_PLACE)
                    break
                elif choice == '4':
                    print("🤖 Starting COMPLETE PICK AND PLACE task...")
                    node.run_task(TASK_COMPLETE_PNP)
                    break
                elif choice == '5':
                    print("🤖 Starting PLACE ON TRAY task...")
                    node.run_task(TASK_PLACE_TRAY)
                    break
                elif choice == '6':
                    print("🤖 Starting STACK CUBE task...")
                    node.run_task(TASK_STACK_CUBE)
                    break
                elif choice == '7':
                    print("🤖 Starting UNSTACK CUBE task...")
                    node.run_task(TASK_UNSTACK_CUBE)
                    break
                elif choice == '8':
                    print("🤖 Starting OPEN DRAWER task...")
                    node.run_task(TASK_OPEN_DRAWER)
                    break
                elif choice == '9':
                    print("🤖 Starting CLOSE DRAWER task...")
                    node.run_task(TASK_CLOSE_DRAWER)
                    break
                elif choice == '10':
                    print("🤖 Starting PUSH BUTTON task...")
                    node.run_task(TASK_PUSH_BUTTON)
                    break
                else:
                    print("❌ Invalid choice. Please enter 1-10.")
                    
    except KeyboardInterrupt:
        print("\n👋 Exiting...")
    finally:
        try:
            if node.csv_file:
                node.csv_file.close()
                print(f'Trajectory saved to: {node.csv_path}')
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
