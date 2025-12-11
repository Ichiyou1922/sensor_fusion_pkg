# SPDX-FileCopyrightText: 2025 Kazuha Mogi <mogi2fruits.kazu@gmail.com>
# SPDX-License-Identifier: BSD-3-Clause
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64, Float64MultiArray

from .kalman_filters import KalmanFilter


def validate_dim_and_topics(dim_z: int, sensor_topics):
    """
    Validate consistency between dim_z and sensor_topics.

    dim_z is the measurement dimension, and sensor_topics is the list of
    sensor topic names. Their lengths must match, and dim_z must be a
    positive integer.

    Raises
    ------
        ValueError
            If dim_z is not a positive integer, sensor_topics is not a list or
            tuple, or len(sensor_topics) != dim_z.

    """
    if not isinstance(dim_z, int) or dim_z <= 0:
        raise ValueError("dim_z must be a positive integer")

    if not isinstance(sensor_topics, (list, tuple)):
        raise ValueError(
                "sensor_topics must be a list or tuple of topic names"
        )

    if len(sensor_topics) != dim_z:
        raise ValueError(
            f"dim_z({dim_z}) and num of topics ({len(sensor_topics)}) dont fit"
        )


class GenericKalmanNode(Node):
    def __init__(self):
        super().__init__('generic_kf')
        # ---- 1. Read parameters ----
        self.dim_x = self.declare_parameter('dim_x').value
        self.dim_z = self.declare_parameter('dim_z').value
        self.sensor_topics = self.declare_parameter('sensor_topics').value
        self.output_topic = self.declare_parameter
        ('output_topic', '/kf_state').value
        validate_dim_and_topics(self.dim_z, self.sensor_topics)
        # matrices: flat lists
        F_list = self.declare_parameter('F').value
        Q_list = self.declare_parameter('Q').value
        H_list = self.declare_parameter('H').value
        R_list = self.declare_parameter('R').value
        x0_list = self.declare_parameter('x0').value
        P0_list = self.declare_parameter('P0').value
        # reshape
        self.F = np.array(F_list, dtype=float).reshape(self.dim_x, self.dim_x)
        self.Q = np.array(Q_list, dtype=float).reshape(self.dim_x, self.dim_x)
        self.H = np.array(H_list, dtype=float).reshape(self.dim_z, self.dim_x)
        self.R = np.array(R_list, dtype=float).reshape(self.dim_z, self.dim_z)
        self.x0 = np.array(x0_list, dtype=float).reshape(self.dim_x)
        self.P0 = np.array(P0_list, dtype=float).reshape(
                self.dim_x, self.dim_x
                )
        # ---- 2. Init Kalman filter ----
        self.kf = KalmanFilter(
                self.x0, self.P0, self.F, self.Q, self.H, self.R
                )
        # ---- 3. Sensor buffers ----
        self.last_z = np.zeros(self.dim_z, dtype=float)
        self.has_z = [False] * self.dim_z

        # ---- 4. Create subscriptions ----
        self.subs = []

        def _make_sensor_calback(self, idx: int):
            def _callback(msg, Float64):
                self.sensor_callback(msg, idx)
            return _callback

        for i, topic in enumerate(self.sensor_topics):
            cb = self._make._make_sensor_callback(i)
            sub = self.create_subscription(Float64, topic, cb, 10)
            self.subs.append(sub)
        # ---- 5. Publisher ----
        self.pub_state = self.create_publisher(
                Float64MultiArray, self.output_topic, 10
                )
        # ---- 6. Timer ----
        self.dt = self.declare_parameter('dt', 0.05).value
        self.timer = self.create_timer(self.dt, self.timer_callback)

    def sensor_callback(self, msg, idx):
        self.last_z[idx] = msg.data
        self.has_z[idx] = True

    def timer_callback(self):
        # センサが値を持つまで待つ
        if not all(self.has_z):
            return

        z = self.last_z.copy()

        self.kf.predict()
        self.kf.update(z)

        # publish
        msg = Float64MultiArray()
        msg.data = self.kf.x.tolist()
        self.pub_state.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GenericKalmanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
