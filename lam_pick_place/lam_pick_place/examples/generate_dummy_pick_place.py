#!/usr/bin/env python3
"""
Generate dummy pick-and-place joint trajectories for a Franka Panda and save as CSV.

This example does not require ROS. It uses the package's RRT and kinematics
utilities to plan between joint-space waypoints corresponding to pre-pick,
pick, lift, pre-place, place, retract, and home. The 8th channel is a simple
gripper position that closes at pick and opens at place.

Outputs CSV files under `trajectories/` with columns:
    t, j1, j2, j3, j4, j5, j6, j7, gripper

Usage:
    python -m lam_pick_place.examples.generate_dummy_pick_place 

Optional CLI args:
    --num_runs  N            number of trajectories to generate (default 5)
    --rate_hz   HZ           sampling rate of the saved trajectory (default 50)
    --max_step  RAD          max joint delta for interpolation (default 0.05)
    --vary                     add small noise to pick/place poses per run
    --out_dir  PATH          output directory (default trajectories)
"""

import os
import csv
import math
import argparse
from datetime import datetime
from typing import List, Tuple

import numpy as np

from lam_pick_place.kinematics import PandaKinematics
from lam_pick_place.rrt import RRTPlanner, RRTConfig


def plan_segment(planner: RRTPlanner, a: np.ndarray, b: np.ndarray, interp_step: float) -> List[np.ndarray]:
    path = planner.plan(a, b)
    if path is None:
        return []
    return planner.interpolate(path, interp_step)


def generate_single_run(
    kin: PandaKinematics,
    planner: RRTPlanner,
    rate_hz: float,
    max_step: float,
    pick_xyzrpy: np.ndarray,
    place_xyzrpy: np.ndarray,
) -> Tuple[List[np.ndarray], List[float]]:
    # Home/start and gripper states
    home = np.array([0.012, -0.5686, 0.0, -2.8106, 0.0, 3.0367, 0.741], dtype=float)
    gripper_open = 0.04
    gripper_closed = 0.0

    # Pre-grasp and pre-place offsets (lift up)
    pre_offset = np.array([0.0, 0.0, 0.08, 0.0, 0.0, 0.0])
    pick_pre = pick_xyzrpy + pre_offset
    place_pre = place_xyzrpy + pre_offset

    # Inverse kinematics for targets
    q_start = home.copy()
    q_pick_pre = kin.inverse_kinematics(kin.pose_from_xyzrpy(*pick_pre), q_start)
    q_pick = kin.inverse_kinematics(kin.pose_from_xyzrpy(*pick_xyzrpy), q_pick_pre)
    q_lift = q_pick_pre  # lift back to pre-pick
    q_place_pre = kin.inverse_kinematics(kin.pose_from_xyzrpy(*place_pre), q_lift)
    q_place = kin.inverse_kinematics(kin.pose_from_xyzrpy(*place_xyzrpy), q_place_pre)

    segments = [
        (q_start, q_pick_pre, 'to_pick_pre'),
        (q_pick_pre, q_pick, 'descend_pick'),
        (q_pick, q_lift, 'lift_after_pick'),
        (q_lift, q_place_pre, 'to_place_pre'),
        (q_place_pre, q_place, 'descend_place'),
        (q_place, q_place_pre, 'retract'),
        (q_place_pre, q_start, 'home'),
    ]

    dt = 1.0 / max(rate_hz, 1e-3)
    out_q: List[np.ndarray] = []
    out_t: List[float] = []
    t = 0.0

    for a, b, label in segments:
        dense = plan_segment(planner, np.array(a), np.array(b), max_step)
        if not dense:
            raise RuntimeError(f"RRT failed for segment {label}")
        for i, q in enumerate(dense):
            # Default gripper open
            g = gripper_open
            # Close at end of descend_pick; open at end of descend_place
            if label == 'descend_pick' and i == len(dense) - 1:
                g = gripper_closed
            if label == 'descend_place' and i == len(dense) - 1:
                g = gripper_open
            out_q.append(np.concatenate([q, [g]]))
            out_t.append(t)
            t += dt
    return out_q, out_t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--num_runs', type=int, default=5)
    ap.add_argument('--rate_hz', type=float, default=50.0)
    ap.add_argument('--max_step', type=float, default=0.05)
    ap.add_argument('--out_dir', type=str, default='trajectories')
    ap.add_argument('--vary', action='store_true', help='add noise to pick/place per run')
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    kin = PandaKinematics()
    lower = np.array([lo for lo, _ in kin.joint_limits], dtype=float)
    upper = np.array([hi for _, hi in kin.joint_limits], dtype=float)
    planner = RRTPlanner(
        dim=7,
        lower=lower,
        upper=upper,
        collision_fn=None,
        config=RRTConfig(
            step_size=args.max_step,
            max_iter=5000,
            goal_threshold=0.03,
            goal_sample_rate=0.1,
        ),
    )

    base_pick = np.array([0.5, 0.0, 0.25, 0.0, math.pi, 0.0], dtype=float)
    base_place = np.array([0.5, 0.25, 0.25, 0.0, math.pi, 0.0], dtype=float)

    for i in range(args.num_runs):
        pick = base_pick.copy()
        place = base_place.copy()
        if args.vary:
            pick[:3] += np.random.uniform(-0.02, 0.02, size=3)
            pick[3:] += np.random.uniform(-0.05, 0.05, size=3)
            place[:3] += np.random.uniform(-0.01, 0.01, size=3)
            place[3:] += np.random.uniform(-0.05, 0.05, size=3)

        try:
            qs, ts = generate_single_run(
                kin=kin,
                planner=planner,
                rate_hz=args.rate_hz,
                max_step=args.max_step,
                pick_xyzrpy=pick,
                place_xyzrpy=place,
            )
        except RuntimeError as e:
            print(f"[Run {i+1}] Failed: {e}")
            break

        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_path = os.path.join(args.out_dir, f'dummy_pick_place_{i+1:02d}_{stamp}.csv')
        with open(csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t'] + [f'j{k+1}' for k in range(7)] + ['gripper'])
            for t, qg in zip(ts, qs):
                w.writerow([t] + list(qg[:7]) + [qg[7]])
        print(f"[Run {i+1}] Saved: {csv_path}  ({len(qs)} points)")


if __name__ == '__main__':
    main()

