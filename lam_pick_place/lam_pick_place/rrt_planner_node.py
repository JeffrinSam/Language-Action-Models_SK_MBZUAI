import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .rrt import RRTPlanner, RRTConfig


class RRTPlannerNode(Node):
    def __init__(self):
        super().__init__('rrt_planner_node')
        self.declare_parameter('max_joint_step', 0.05)
        self.declare_parameter('max_iterations', 5000)
        self.declare_parameter('goal_threshold', 0.03)
        self.declare_parameter('output_topic', 'planned_trajectory')
        self.declare_parameter('rate_hz', 50.0)

        self.max_joint_step = self.get_parameter('max_joint_step').get_parameter_value().double_value
        self.max_iterations = int(self.get_parameter('max_iterations').get_parameter_value().integer_value or 5000)
        self.goal_threshold = self.get_parameter('goal_threshold').get_parameter_value().double_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.rate_hz = self.get_parameter('rate_hz').get_parameter_value().double_value

        qos = QoSProfile(depth=10)
        # Publish planned path as JointTrajectory
        self.pub = self.create_publisher(JointTrajectory, self.output_topic, qos)
        self.sub_start = self.create_subscription(JointState, 'start_joint_state', self.on_start, qos)
        self.sub_goal = self.create_subscription(JointState, 'goal_joint_state', self.on_goal, qos)

        self.start_q = None
        self.goal_q = None
        self.names = None

    def on_start(self, msg: JointState):
        self.start_q = np.array(msg.position[:7], dtype=float)
        self.names = list(msg.name)
        self.try_plan()

    def on_goal(self, msg: JointState):
        self.goal_q = np.array(msg.position[:7], dtype=float)
        if self.names is None:
            self.names = list(msg.name)
        self.try_plan()

    def try_plan(self):
        if self.start_q is None or self.goal_q is None:
            return
        lower = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
        upper = np.array([ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973])
        planner = RRTPlanner(
            dim=7,
            lower=lower,
            upper=upper,
            collision_fn=None,
            config=RRTConfig(
                step_size=self.max_joint_step,
                max_iter=self.max_iterations,
                goal_threshold=self.goal_threshold,
                goal_sample_rate=0.1,
            ),
        )
        path = planner.plan(self.start_q, self.goal_q)
        if path is None:
            self.get_logger().error('RRT failed to find a path')
            return
        dense = planner.interpolate(path, self.max_joint_step * 0.5)
        traj = JointTrajectory()
        # Default names if not provided
        names = self.names if self.names else [f'panda_joint{i+1}' for i in range(7)]
        traj.joint_names = names
        t = 0.0
        dt = 1.0 / max(self.rate_hz, 1e-3)
        for q in dense:
            pt = JointTrajectoryPoint()
            pt.positions = list(q)
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
            traj.points.append(pt)
            t += dt
        self.pub.publish(traj)
        self.get_logger().info(f'Published trajectory with {len(traj.points)} points')


def main():
    rclpy.init()
    node = RRTPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
