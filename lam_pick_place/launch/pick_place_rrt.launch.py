from launch import LaunchDescription
from launch_ros.actions import Node
import math


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lam_pick_place',
            executable='pick_place_manager_node',
            name='pick_place_manager',
            output='screen',
            parameters=[{
                'arm_side': 'Left',
                'publish_topic': '/Left/joint_command',
                'rate_hz': 50.0,
                'max_joint_step': 0.05,
                'max_iterations': 5000,
                'goal_threshold': 0.03,
                'log_dir': 'trajectories',
                'num_runs': 20,
                'vary_targets': True,
                'pick_pose_xyzrpy': [0.5, 0.0, 0.25, 0.0, math.radians(180), 0.0],
                'place_pose_xyzrpy': [0.5, 0.25, 0.25, 0.0, math.radians(180), 0.0],
            }]
        ),
    ])
