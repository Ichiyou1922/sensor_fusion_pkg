import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random

class NoisySensor(Node):
    def __init__(self):
        super().__init__('noisy_sensor')
        # ノイズの分散とセンサID
        self.declare_parameter('variance', 1.0)
        self.declare_parameter('sensor_id', 1)

        self.variance = self.get_parameter('variance').value
        self.sensor_id = self.get_parameter('sensor_id').value

        topic_name = f'sensor_{self.sensor_id}/data'
        self.publisher = self.create_publisher(Float64, topic_name, 10)

        self.timer = self.create_timer(0.1, self.publish_data) # 10Hzでデータ送信
        self.true_value = 25.0 # 室温25度など

        self.get_logger().info(f'Sensor {self.sensor_id} started. Variance: {self.variance}')

    def publish_data(self):
        msg = Float64()
        # ガウス分布に従うノイズを加える->N(mu, sigma^2)
        noise = random.gauss(0, self.variance ** 0.5) # random.gaussはsigmaなので平方根を取る
        msg.data = self.true_value + noise
        self.publisher_.publish(msg)
        # デバッグ用
        self.get_logger().info(f'Sendor {self.sensor_id}: {msg.data:.4} (Var: {self.variance})')

def main(args=None):
    rclpy.init(args=args)
    node = NoisySensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
