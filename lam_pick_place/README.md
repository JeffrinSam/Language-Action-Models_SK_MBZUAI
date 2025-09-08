lam_pick_place (ROS 2 Python Package)

Overview
- RRT planner and pick-and-place manager for a Franka Panda in Isaac Sim.
- Publishes `sensor_msgs/JointState` to your existing topics (e.g., `/Left/joint_command`).
- Records trajectories to CSV and fits a linear Gaussian dynamics model.

Nodes
- `pick_place_manager_node`: Plans and publishes a complete pick-and-place trajectory (7 joints + gripper) and saves CSV per run.
- `trajectory_controller_node`: Plays `trajectory_msgs/JointTrajectory` as `JointState` at a fixed rate.
- `trajectory_recorder_node`: Subscribes to joint commands and logs to CSV.
- `rrt_planner_node`: Standalone RRT planner from start/goal joint states to a `JointTrajectory`.
- `linear_gaussian_learner`: Fits a linear Gaussian model x[t+1] ≈ A x[t] + B u[t] + c, where u[t] = x[t+1] − x[t].

Non-ROS Dummy Example
- Generate CSV trajectories offline (no ROS):
  python -m lam_pick_place.examples.generate_dummy_pick_place --num_runs 5 --rate_hz 50 --max_step 0.05 --vary --out_dir trajectories
  This saves files like `trajectories/dummy_pick_place_01_YYYYMMDD_HHMMSS.csv` with columns `t, j1..j7, gripper`.

Quick Start (colcon)
1) Put this package folder in a ROS 2 workspace src and build:
   colcon build --symlink-install
   source install/setup.bash

2) Start Isaac Sim + your Panda controllers that subscribe to `/Left/joint_command` (or adjust params).

3) Start the controller and manager:
   ros2 run lam_pick_place trajectory_controller_node --ros-args -p publish_topic:=/Left/joint_command -p input_trajectory_topic:=input_trajectory -p rate_hz:=50.0
   ros2 run lam_pick_place pick_place_manager_node --ros-args \
     -p publish_topic:=/Left/joint_command \
     -p input_trajectory_topic:=input_trajectory \
     -p rate_hz:=50.0 -p num_runs:=20 -p log_dir:=trajectories

   This will publish 20 pick-and-place trajectories and save CSVs under `trajectories/`.

4) Alternative: combined launch (controller + manager together):
   ros2 launch lam_pick_place pick_place_full.launch.py

5) Learn linear Gaussian dynamics from the recorded CSVs:
   ros2 run lam_pick_place linear_gaussian_learner --ros-args -p log_dir:=trajectories -p output:=linear_gaussian_model.npz

Key Parameters (pick_place_manager_node)
- `publish_topic` (string): Where to publish `JointState` (default `/Left/joint_command`).
- `input_trajectory_topic` (string): Topic to publish `JointTrajectory` for the controller.
- `rate_hz` (double): Playback frequency for timing stamps.
- `max_joint_step` (double): RRT step size and interpolation.
- `max_iterations` (int): RRT iterations.
- `goal_threshold` (double): Distance threshold to goal in joint space.
- `log_dir` (string): Output directory for CSVs and learned models.
- `num_runs` (int): Number of pick-and-place runs (e.g., 20).
- `vary_targets` (bool): Adds small noise to pick/place poses per run.
- `pick_pose_xyzrpy` / `place_pose_xyzrpy` (double[6]): Target EE poses (m, rad).

Notes
- Collision checking is currently a stub in the RRT; if you have a ROS 2 service/topic from Isaac Sim for collision queries, wire it via `collision_fn` in `lam_pick_place/rrt.py`.
- Gripper is the 8th joint (`panda_finger_joint1`). Open/close events are inserted at appropriate points in the trajectory.
- If your controllers require `trajectory_msgs/JointTrajectory` directly, you can bypass `trajectory_controller_node` and map to your action server instead.
