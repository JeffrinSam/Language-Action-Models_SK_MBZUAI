import os
import csv
import time
import math
import numpy as np
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import UInt32
from sensor_msgs.msg import JointState

from .kinematics import PandaKinematics
from .rrt import RRTPlanner, RRTConfig


def path_to_trajectory(path: np.ndarray, names, rate_hz: float) -> JointTrajectory:
    traj = JointTrajectory()
    traj.joint_names = names
    t = 0.0
    dt = 1.0 / max(rate_hz, 1e-3)
    for q in path:
        pt = JointTrajectoryPoint()
        pt.positions = list(q)
        pt.time_from_start.sec = int(t)
        pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
        traj.points.append(pt)
        t += dt
    return traj


class PickPlaceManagerNode(Node):
    def __init__(self):
        super().__init__('pick_place_manager')
        # Parameters
        self.declare_parameter('arm_side', 'Left')
        self.declare_parameter('publish_topic', '/Left/joint_command')
        self.declare_parameter('input_trajectory_topic', 'input_trajectory')
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('max_joint_step', 0.05)
        self.declare_parameter('max_iterations', 5000)
        self.declare_parameter('goal_threshold', 0.03)
        self.declare_parameter('log_dir', 'trajectories')
        self.declare_parameter('num_runs', 20)
        self.declare_parameter('vary_targets', True)
        # xyz [m], rpy [rad]
        self.declare_parameter('pick_pose_xyzrpy', [0.5, 0.0, 0.25, 0.0, math.pi, 0.0])
        self.declare_parameter('place_pose_xyzrpy', [0.5, 0.25, 0.25, 0.0, math.pi, 0.0])

        self.arm_side = self.get_parameter('arm_side').get_parameter_value().string_value
        self.publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        self.input_traj_topic = self.get_parameter('input_trajectory_topic').get_parameter_value().string_value
        self.rate_hz = self.get_parameter('rate_hz').get_parameter_value().double_value
        self.max_joint_step = self.get_parameter('max_joint_step').get_parameter_value().double_value
        self.max_iterations = int(self.get_parameter('max_iterations').get_parameter_value().integer_value or 5000)
        self.goal_threshold = self.get_parameter('goal_threshold').get_parameter_value().double_value
        self.log_dir = self.get_parameter('log_dir').get_parameter_value().string_value
        self.num_runs = int(self.get_parameter('num_runs').get_parameter_value().integer_value or 20)
        self.vary_targets = bool(self.get_parameter('vary_targets').get_parameter_value().bool_value)
        self.pick_pose = self.get_parameter('pick_pose_xyzrpy').get_parameter_value().double_array_value
        self.place_pose = self.get_parameter('place_pose_xyzrpy').get_parameter_value().double_array_value

        qos = QoSProfile(depth=10)
        self.traj_pub = self.create_publisher(JointTrajectory, self.input_traj_topic, qos)
        self.run_pub = self.create_publisher(UInt32, 'trajectory_run', qos)
        self.cmd_pub = self.create_publisher(JointState, self.publish_topic, qos)

        os.makedirs(self.log_dir, exist_ok=True)

        # Set up kinematics and planner
        self.kin = PandaKinematics()
        lower = np.array([lo for lo, _ in self.kin.joint_limits], dtype=float)
        upper = np.array([hi for _, hi in self.kin.joint_limits], dtype=float)
        self.rrt = RRTPlanner(
            dim=7,
            lower=lower,
            upper=upper,
            collision_fn=None,  # plug Isaac collision checker here if available
            config=RRTConfig(
                step_size=self.max_joint_step,
                max_iter=self.max_iterations,
                goal_threshold=self.goal_threshold,
                goal_sample_rate=0.1,
            ),
        )

        # Default home/start configuration and gripper open
        self.home = np.array([0.012, -0.5686, 0.0, -2.8106, 0.0, 3.0367, 0.741], dtype=float)
        self.gripper_open = 0.04
        self.gripper_closed = 0.0

        self.timer = self.create_timer(1.0, self._kickoff_once)
        self._started = False

    def _kickoff_once(self):
        if self._started:
            return
        self._started = True
        self.timer.cancel()
        self.get_logger().info(f"Starting pick-and-place for {self.num_runs} runs…")
        self._run_multiple()

    def _publish_gripper(self, position: float, names=None, base_joints=None):
        js = JointState()
        if names is None:
            names = [f'panda_joint{i+1}' for i in range(7)] + ['panda_finger_joint1']
        if base_joints is None:
            base_joints = self.home
        js.name = names
        js.position = list(base_joints) + [float(position)]
        self.cmd_pub.publish(js)

    def _plan_segment(self, start_q: np.ndarray, goal_q: np.ndarray):
        path = self.rrt.plan(start_q, goal_q)
        if path is None:
            return None
        dense = self.rrt.interpolate(path, self.max_joint_step * 0.5)
        return np.array(dense)

    def _run_single(self, run_idx: int):
        # Optionally vary pick/place a bit
        pick_xyzrpy = np.array(self.pick_pose, dtype=float)
        place_xyzrpy = np.array(self.place_pose, dtype=float)
        if self.vary_targets:
            noise_pos = np.random.uniform(-0.01, 0.01, size=3)
            noise_ang = np.random.uniform(-0.05, 0.05, size=3)
            pick_xyzrpy[:3] += noise_pos
            pick_xyzrpy[3:] += noise_ang
            place_xyzrpy[:3] += np.random.uniform(-0.01, 0.01, size=3)
            place_xyzrpy[3:] += np.random.uniform(-0.05, 0.05, size=3)

        # Pre-grasp and pre-place offsets
        pre_offset = np.array([0.0, 0.0, 0.08, 0.0, 0.0, 0.0])
        pick_pre = pick_xyzrpy + pre_offset
        place_pre = place_xyzrpy + pre_offset

        # IK for targets
        start_q = self.home.copy()
        q_pick_pre = self.kin.inverse_kinematics(self.kin.pose_from_xyzrpy(*pick_pre), start_q)
        q_pick = self.kin.inverse_kinematics(self.kin.pose_from_xyzrpy(*pick_xyzrpy), q_pick_pre)
        q_lift = q_pick_pre  # lift back to pre pose
        q_place_pre = self.kin.inverse_kinematics(self.kin.pose_from_xyzrpy(*place_pre), q_lift)
        q_place = self.kin.inverse_kinematics(self.kin.pose_from_xyzrpy(*place_xyzrpy), q_place_pre)

        segments = [
            (start_q, q_pick_pre, 'to_pick_pre'),
            (q_pick_pre, q_pick, 'descend_pick'),
            (q_pick, q_lift, 'lift_after_pick'),
            (q_lift, q_place_pre, 'to_place_pre'),
            (q_place_pre, q_place, 'descend_place'),
            (q_place, q_place_pre, 'retract'),
            (q_place_pre, self.home, 'home'),
        ]

        # Plan all segments
        planned_paths = []
        for a, b, label in segments:
            path = self._plan_segment(np.array(a), np.array(b))
            if path is None:
                self.get_logger().error(f"RRT failed for segment {label}")
                return False, 0.0
            planned_paths.append((label, path))

        # Build a unified JointTrajectory with gripper channel (8th)
        names = [f'panda_joint{i+1}' for i in range(7)] + ['panda_finger_joint1']
        dt = 1.0 / max(self.rate_hz, 1e-3)
        traj = JointTrajectory()
        traj.joint_names = names
        t = 0.0
        # open gripper before pick
        for label, path in planned_paths:
            for i, q in enumerate(path):
                pt = JointTrajectoryPoint()
                pt.positions = list(q) + [self.gripper_open]
                pt.time_from_start.sec = int(t)
                pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
                traj.points.append(pt)
                t += dt
            if label == 'descend_pick':
                # close gripper at end of pick
                pt = JointTrajectoryPoint()
                pt.positions = list(path[-1]) + [self.gripper_closed]
                pt.time_from_start.sec = int(t)
                pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
                traj.points.append(pt)
                t += dt
            if label == 'descend_place':
                # open gripper to place
                pt = JointTrajectoryPoint()
                pt.positions = list(path[-1]) + [self.gripper_open]
                pt.time_from_start.sec = int(t)
                pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
                traj.points.append(pt)
                t += dt

        # Publish run-start signal so recorders can rotate files
        self.run_pub.publish(UInt32(data=run_idx + 1))

        # Publish trajectory for controller
        self.traj_pub.publish(traj)
        self.get_logger().info(f"Published trajectory with {len(traj.points)} points (run {run_idx+1})")

        # Also log to CSV
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_path = os.path.join(self.log_dir, f'pick_place_run_{run_idx+1:02d}_{stamp}.csv')
        with open(csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            header = ['t'] + [f'j{i+1}' for i in range(7)] + ['gripper']
            w.writerow(header)
            t = 0.0
            for pt in traj.points:
                row = [t] + list(pt.positions[:7]) + [pt.positions[7]]
                w.writerow(row)
                t += dt
        self.get_logger().info(f"Saved trajectory CSV: {csv_path}")
        # Duration estimate (seconds)
        total_duration = (traj.points[-1].time_from_start.sec 
                          + traj.points[-1].time_from_start.nanosec / 1e9)
        return True, total_duration

    def _run_multiple(self):
        ok_count = 0
        for i in range(self.num_runs):
            result = self._run_single(i)
            if not result:
                self.get_logger().error(f"Run {i+1} failed; stopping further runs.")
                break
            success, total_duration = result
            ok_count += 1
            # Sleep for trajectory duration plus a small gap
            time.sleep(float(total_duration) + 0.5)
        self.get_logger().info(f"Completed {ok_count}/{self.num_runs} runs.")


def main():
    rclpy.init()
    node = PickPlaceManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
