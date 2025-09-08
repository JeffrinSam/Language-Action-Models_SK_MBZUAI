from launch import LaunchDescription
from launch.actions import ExecuteProcess
import math


def generate_launch_description():
    # Fallback launch: use `ros2 run` to start nodes so we don't rely on
    # the package's libexec layout. Useful when libexec wasn't created.
    controller_cmd = [
        'ros2', 'run', 'lam_pick_place', 'trajectory_controller_node',
        '--ros-args',
        '-p', 'publish_topic:=/Left/joint_command',
        '-p', 'input_trajectory_topic:=input_trajectory',
        '-p', 'rate_hz:=50.0',
    ]

    manager_cmd = [
        'ros2', 'run', 'lam_pick_place', 'pick_place_manager_node',
        '--ros-args',
        '-p', 'arm_side:=Left',
        '-p', 'publish_topic:=/Left/joint_command',
        '-p', 'input_trajectory_topic:=input_trajectory',
        '-p', 'rate_hz:=50.0',
        '-p', 'max_joint_step:=0.05',
        '-p', 'max_iterations:=5000',
        '-p', 'goal_threshold:=0.03',
        '-p', 'log_dir:=trajectories',
        '-p', 'num_runs:=20',
        '-p', 'vary_targets:=true',
        '-p', f'pick_pose_xyzrpy:=[0.5, 0.0, 0.25, 0.0, {math.radians(180)}, 0.0]',
        '-p', f'place_pose_xyzrpy:=[0.5, 0.25, 0.25, 0.0, {math.radians(180)}, 0.0]',
    ]

    return LaunchDescription([
        ExecuteProcess(cmd=controller_cmd, output='screen'),
        ExecuteProcess(cmd=manager_cmd, output='screen'),
    ])

