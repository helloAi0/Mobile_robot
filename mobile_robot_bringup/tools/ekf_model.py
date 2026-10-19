#!/usr/bin/env python3
"""A small planar EKF that mirrors the robot_localization configuration in config/ekf.yaml.

robot_localization fuses (see ekf.yaml):  wheel odometry  -> forward speed vx, yaw rate vyaw
                                          IMU gyroscope   -> yaw rate vyaw
This file is the same estimator in 60 lines of numpy: state [x, y, yaw, v, w], constant-velocity prediction, two
measurement updates per step. It exists so the fusion design can be tested and scored without ROS (the 2D benchmark
simulator drives it with synthetic wheel slip) and so the numbers in ekf.yaml are checked against each other
(tests/test_ekf_config.py). It is NOT robot_localization: it shows what the configuration should achieve, the node
itself is exercised on the robot / in Gazebo.
"""
import math

import numpy as np


class PlanarEKF:
    def __init__(self, q_pos=0.05, q_yaw=0.03, q_v=0.05, q_w=0.02, p0=1e-4):
        # process noise rates (variance per second) in the units of ekf.yaml's process_noise_covariance diagonal
        self.Q = np.diag([q_pos, q_pos, q_yaw, q_v, q_w])
        self.s = np.zeros(5)
        self.P = np.eye(5) * p0

    @classmethod
    def from_yaml(cls, path):
        """Process noise from ekf.yaml (diagonal entries of process_noise_covariance: x, yaw, vx, vyaw)."""
        import yaml
        n = yaml.safe_load(open(path))['ekf_filter_node']['ros__parameters']['process_noise_covariance']
        d = lambda i: n[i * 15 + i]  # noqa: E731
        return cls(q_pos=d(0), q_yaw=d(5), q_v=d(6), q_w=d(11))

    def reset(self, x=0.0, y=0.0, yaw=0.0):
        self.s = np.array([x, y, yaw, 0.0, 0.0])
        self.P = np.eye(5) * 1e-4

    def predict(self, dt):
        x, y, th, v, w = self.s
        c, s = math.cos(th), math.sin(th)
        self.s = np.array([x + v * c * dt, y + v * s * dt, th + w * dt, v, w])
        F = np.eye(5)
        F[0, 2], F[0, 3] = -v * s * dt, c * dt
        F[1, 2], F[1, 3] = v * c * dt, s * dt
        F[2, 4] = dt
        self.P = F @ self.P @ F.T + self.Q * dt

    def _update(self, H, z, R):
        H = np.atleast_2d(H)
        z = np.atleast_1d(z)
        S = H @ self.P @ H.T + np.atleast_2d(R)
        K = self.P @ H.T @ np.linalg.inv(S)
        self.s = self.s + K @ (z - H @ self.s)
        I = np.eye(5)
        KH = I - K @ H
        self.P = KH @ self.P @ KH.T + K @ np.atleast_2d(R) @ K.T      # Joseph form

    def update_wheel(self, v, w, var_v, var_w):
        self._update(np.array([[0, 0, 0, 1, 0], [0, 0, 0, 0, 1]]), [v, w], np.diag([var_v, var_w]))

    def update_gyro(self, w, var_w):
        self._update([0, 0, 0, 0, 1], [w], [[var_w]])

    @property
    def pose(self):
        return float(self.s[0]), float(self.s[1]), float(self.s[2])


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))
