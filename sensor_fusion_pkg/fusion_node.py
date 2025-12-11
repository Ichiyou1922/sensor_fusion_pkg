# SPDX-FileCopyrightText: 2025 Kazuha Mogi <mogi2fruits.kazu@gmail.com>
# SPDX-License-Identifier: BSD-3-Clause
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64
from .kalman_filters import KalmanFilter


class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')

        q = 1.0
        # センサ1，2の観測雑音分散（例：センサ1が高精度，センサ2が低精度）
        self.r1 = 1.0
        self.r2 = 5.0

        x0 = np.array([25.0])
        P0 = np.array([[1.0]])
        F = np.array([[1.0]])
        Q = np.array([[q]])
        H = np.array([[1.0]])
        # 初期状態ではとりあえずセンサ1側のRを入れておく
        R = np.array([[self.r1]])

        # カルマンフィルタの初期化
        self.kf = KalmanFilter(x0, P0, F, Q, H, R)

        # 2つのセンサトピックを購読
        self.sub1 = self.create_subscription(
            Float64, 'sensor_1/data', self.sensor1_callback, 10
        )
        self.sub2 = self.create_subscription(
            Float64, 'sensor_2/data', self.sensor2_callback, 10
        )

        self.pub_ = self.create_publisher(Float64, 'fused_estimate', 10)

    def sensor1_callback(self, msg):
        # センサ1用の雑音分散で更新
        self.process_measurement(msg.data, self.r1)

    def sensor2_callback(self, msg):
        # センサ2用の雑音分散で更新
        self.process_measurement(msg.data, self.r2)

    def process_measurement(self, z, r_var):
        # センサごとに観測雑音共分散Rを切り替える
        self.kf.R = np.array([[r_var]])

        # 予測
        self.kf.predict()
        # 更新（zは1次元なので shape (1,) にして渡す）
        self.kf.update(np.array([z]))
        x_est = float(self.kf.x[0])

        # 出力
        out = Float64()
        out.data = x_est
        self.pub_.publish(out)

        self.get_logger().info(f'Update R={r_var}: est={x_est:.3f}')


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

