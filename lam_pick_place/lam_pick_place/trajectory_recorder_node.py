import os
import csv
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from datetime import datetime
from std_msgs.msg import UInt32


class TrajectoryRecorderNode(Node):
    def __init__(self):
        super().__init__('trajectory_recorder')
        self.declare_parameter('subscribe_topic', '/Left/joint_command')
        self.declare_parameter('log_dir', 'trajectories')
        self.declare_parameter('filename_prefix', 'recorded')
        self.declare_parameter('append_timestamp', True)
        self.declare_parameter('flush_every', 25)
        self.declare_parameter('rotate_per_run', True)

        self.topic = self.get_parameter('subscribe_topic').get_parameter_value().string_value
        self.log_dir = self.get_parameter('log_dir').get_parameter_value().string_value
        self.prefix = self.get_parameter('filename_prefix').get_parameter_value().string_value
        self.append_ts = self.get_parameter('append_timestamp').get_parameter_value().bool_value
        self.flush_every = int(self.get_parameter('flush_every').get_parameter_value().integer_value or 25)
        self.rotate_per_run = self.get_parameter('rotate_per_run').get_parameter_value().bool_value

        os.makedirs(self.log_dir, exist_ok=True)
        self.f = None
        self.w = None
        self.csv_path = None
        self.count = 0
        self.start_time = self.get_clock().now()
        if not self.rotate_per_run:
            self._open_new_file()

        qos = QoSProfile(depth=100)
        self.sub = self.create_subscription(JointState, self.topic, self.on_js, qos)
        if self.rotate_per_run:
            self.sub_run = self.create_subscription(UInt32, 'trajectory_run', self.on_run_start, qos)
            self.get_logger().info(f"Recording JointState from {self.topic}; waiting for run-start to open CSVs in {self.log_dir}")
        else:
            self.get_logger().info(f"Recording JointState from {self.topic} to {self.csv_path}")

    def _open_new_file(self, run_id: int = None):
        if self.f:
            try:
                self.f.flush()
                self.f.close()
            except Exception:
                pass
        ts = datetime.now().strftime('%Y%m%d_%H%M%S') if self.append_ts else ''
        suffix_parts = []
        if run_id is not None:
            suffix_parts.append(f"run_{run_id:02d}")
        if ts:
            suffix_parts.append(ts)
        suffix = ('_' + '_'.join(suffix_parts)) if suffix_parts else ''
        self.csv_path = os.path.join(self.log_dir, f"{self.prefix}{suffix}.csv")
        self.f = open(self.csv_path, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t'] + [f'j{i+1}' for i in range(7)] + ['gripper'])
        self.count = 0
        self.start_time = self.get_clock().now()
        self.get_logger().info(f"Opened new CSV: {self.csv_path}")

    def on_run_start(self, msg: UInt32):
        if not self.rotate_per_run:
            return
        self._open_new_file(run_id=int(msg.data))

    def on_js(self, msg: JointState):
        if self.rotate_per_run and self.f is None:
            # Wait for run-start signal to open file
            return
        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds / 1e9
        pos = list(msg.position)
        # Ensure we have 8 positions; pad or truncate
        if len(pos) < 8:
            pos = (pos + [0.0] * 8)[:8]
        row = [t] + pos[:7] + [pos[7]]
        if self.w:
            self.w.writerow(row)
        self.count += 1
        if self.count % self.flush_every == 0:
            if self.f:
                self.f.flush()

    def destroy_node(self):
        try:
            if self.f:
                self.f.flush()
                self.f.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = TrajectoryRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
