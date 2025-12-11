import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64
from .kalman_filters import KalmanFilter


class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')

        q = 1.0
        r = 1.0

        x0 = np.array([25.0])
        P0 = np.array([[1.0]])
        F = np.array([[1.0]])
        Q = np.array([[q]])
        H = np.array([[1.0]])
        R = np.array([[r]])

        # カルマンフィルタの初期化
        self.kf = KalmanFilter(x0, P0, F, Q, H, R)

        self.sub = self.create_subscription(
            Float64, 'sensor_1/data', self.callback, 10)

        self.pub_ = self.create_publisher(Float64, 'fused_estimate', 10)

    def callback(self, msg):
        z = msg.data
        self.kf.predict()
        self.kf.update(np.array([z]))
        x_est = self.kf.x[0]

        # 出力
        msg = Float64()
        msg.data = x_est
        self.pub_.publish(msg)


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
