import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time as TimeMsg
import time


class TrajectoryControllerNode(Node):
    def __init__(self):
        super().__init__('trajectory_controller')
        self.declare_parameter('publish_topic', '/Left/joint_command')
        self.declare_parameter('input_trajectory_topic', 'input_trajectory')
        self.declare_parameter('rate_hz', 50.0)

        self.publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        in_topic = self.get_parameter('input_trajectory_topic').get_parameter_value().string_value
        self.rate_hz = self.get_parameter('rate_hz').get_parameter_value().double_value

        qos = QoSProfile(depth=10)
        self.pub = self.create_publisher(JointState, self.publish_topic, qos)
        self.sub = self.create_subscription(JointTrajectory, in_topic, self.on_trajectory, qos)
        self.get_logger().info(f"Publishing JointState to {self.publish_topic} at {self.rate_hz} Hz")

    def on_trajectory(self, msg: JointTrajectory):
        if not msg.points:
            self.get_logger().warn('Received empty trajectory')
            return
        names = msg.joint_names
        dt = 1.0 / max(self.rate_hz, 1e-3)
        last_time = self.get_clock().now()
        for pt in msg.points:
            js = JointState()
            js.name = names
            js.position = list(pt.positions)
            # attempt to preserve timing roughly
            if pt.time_from_start is not None and (pt.time_from_start.sec != 0 or pt.time_from_start.nanosec != 0):
                target_time = TimeMsg(sec=pt.time_from_start.sec, nanosec=pt.time_from_start.nanosec)
                # busy-waiting is simple here since IsaacSim sim-time may differ; rely on rate otherwise
            self.pub.publish(js)
            time.sleep(dt)


def main():
    rclpy.init()
    node = TrajectoryControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

