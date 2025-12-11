# SPDX-FileCopyrightText: 2025 Kazuha Mogi <mogi2fruits.kazu@gmail.com>
# SPDX-License-Identifier: BSD-3-Clause
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random


class NoisySensor(Node):
    def __init__(self):
        super().__init__('noisy_sensor')
        self.declare_parameter('variance', 1.0)
        self.declare_parameter('sensor_id', 1)

        self.variance = self.get_parameter('variance').value
        self.sensor_id = self.get_parameter('sensor_id').value

        self.sigma = self.variance ** 0.5

        topic_name = f'sensor_{self.sensor_id}/data'
        self.publisher_ = self.create_publisher(Float64, topic_name, 10)

        # 10Hz
        self.timer = self.create_timer(0.1, self.publish_data)
        self.true_value = 25.0

        self.get_logger().info(
            f'Sensor {self.sensor_id} started. Var: {self.variance}')

    def publish_data(self):
        msg = Float64()
        noise = random.gauss(0, self.sigma)
        msg.data = self.true_value + noise
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NoisySensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
