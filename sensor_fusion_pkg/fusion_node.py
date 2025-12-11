import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class KalmanFilter1D:
    def __init__(self, initial_x, initial_p, process_noise_q):
        self.x = initial_x # 状態推定値(平均)
        self.p = initial_p # 推定誤差共分散
        self.q = process_noise_q # プロセスノイズ

        def predict(self):
            self.p = self.p + self.q

        def update(self, z, r):
            k = self.p / (self.p + r) # KalmanGain K = P / (P + R)

            self.x = self.x + k * (z - self.x) # update x

            self.p = (1 - k) * self.p # update p

            return self.x, self.p

class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')
        
        self.declare_parameter('q_process_noise', 0.01) # 時間経過で劣化
        self.declare_parameter('var1', 1.0)
        self.declare_parameter('var2', 5.0) # とりあえずセンサ2のほうが精度が悪いことに
        
        self.q = self.get_parameter('q_process_noise').value
        self.r1 = self.get_parameter('r_sensor1').value
        self.r2 = self.get_parameter('r_sensor2').value

        # カルマンフィルタの初期化
        self.kf = KalmanFilter1D(initial_x=0.0, initial_p=100.0, process_noise_q=self.q)

        self.sub1 = self.create_subscription(Float64, 'sensor_1/data', self.callback1, 10)
        self.sub2 = self.create_subscription(Float64, 'sensor_2/data', self.callback2, 10)

        self.pub_ = self.create_publisher(Float64, 'fused_estimate', 10)
    
    def callback1(self, msg):
        self.process_measurement(msg.data, self.r1)
    
    def callback2(self, msg):
        self.process_measurement(msg.data, self.r2) 
    
    def process_measurement(self, z, r):
        # 予測
        self.kf.predict()

        # 更新
        x_est, p_est = self.kf.update(z, r)

        # 出力
        msg = Float64()
        msg.data = x_est
        self.pub_.publish(msg)

        self.get_logger().info(f'Update with R={r}: Est={x_est:.4f}, Var={p_est:.4f}')
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
