import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')
        
        self.declare_parameter('var1', 1.0)
        self.declare_parameter('var2', 5.0) # とりあえずセンサ2のほうが精度が悪いことに

        self.sub1 = self.create_subscription(Float64, 'sensor_1/data', self.callback1, 10)
        self.sub2 = self.create_subscription(Float64, 'sensor_2/data', self.callback2, 10)

        self.pub_ = self.create_publisher(Float64, 'fused_estimate', 10)
        
        self.latest_z1 = None
        self.latest_z2 = None


    def callback1(self, msg):
        self.latest_z1 = msg.data
        self.calculate_fusion()

    def callback2(self, msg):
        self.latest_z2 = msg.data
        self.calculate_fusion()
    
    def attempt_fusion(self):
        if self.z1 is None or self.z2 is None:
            return

        # 重み付け平均を取る
        w1 = 1.0 / self.var1
        w2 = 1.0 / self.var2
        normalization = w1 + w2

        estimated_value = (self.latest_z1 * w1 + self.latest_z2 * w2) / normalization

        msg = Float64()
        msg.data = estimated_value
        self.pub_.publish(msg)
        self.get_logger().info(f'Estimate: {estimated_value:.4f}')

def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown
