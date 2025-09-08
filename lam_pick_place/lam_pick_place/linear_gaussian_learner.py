import os
import glob
import numpy as np
import rclpy
from rclpy.node import Node


class LinearGaussianLearner(Node):
    def __init__(self):
        super().__init__('linear_gaussian_learner')
        self.declare_parameter('log_dir', 'trajectories')
        self.declare_parameter('output', 'linear_gaussian_model.npz')
        self.log_dir = self.get_parameter('log_dir').get_parameter_value().string_value
        self.output = self.get_parameter('output').get_parameter_value().string_value
        self.timer = self.create_timer(0.5, self._run_once)
        self._done = False

    def _run_once(self):
        if self._done:
            return
        self._done = True

        csvs = sorted(glob.glob(os.path.join(self.log_dir, 'pick_place_run_*.csv')))
        if not csvs:
            self.get_logger().error(f"No CSV files found in {self.log_dir}")
            return
        X_list = []
        U_list = []
        Y_list = []
        for path in csvs:
            data = np.genfromtxt(path, delimiter=',', names=True)
            # columns: t, j1..j7, gripper
            q = np.vstack([data[f'j{i+1}'] for i in range(7)]).T
            # Build (x_t, u_t, x_{t+1}) pairs
            x_t = q[:-1]
            x_tp1 = q[1:]
            u_t = x_tp1 - x_t  # commanded increment as proxy for input
            X_list.append(x_t)
            U_list.append(u_t)
            Y_list.append(x_tp1)

        X = np.vstack(X_list)
        U = np.vstack(U_list)
        Y = np.vstack(Y_list)

        # Stack features [x; u; 1]
        ones = np.ones((X.shape[0], 1))
        Phi = np.hstack([X, U, ones])  # (N, 7+7+1)
        Theta, *_ = np.linalg.lstsq(Phi, Y, rcond=None)  # (15, 7)
        # Unpack
        A = Theta[:7, :].T  # (7,7)
        B = Theta[7:14, :].T  # (7,7)
        c = Theta[14, :].reshape(7, 1)  # (7,1)

        # Residual covariance Q
        Y_hat = Phi @ Theta  # (N,7)
        resid = Y - Y_hat
        Q = (resid.T @ resid) / max(resid.shape[0] - Phi.shape[1], 1)

        out_path = os.path.join(self.log_dir, self.output)
        np.savez(out_path, A=A, B=B, c=c, Q=Q)
        self.get_logger().info(f"Saved linear Gaussian model to {out_path}")


def main():
    rclpy.init()
    node = LinearGaussianLearner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

