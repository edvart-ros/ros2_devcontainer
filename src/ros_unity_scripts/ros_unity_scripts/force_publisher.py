import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class ForcePublisher(Node):
    def __init__(self):
        super().__init__('force_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_force', 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 400000.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    pub = ForcePublisher()
    rclpy.spin(pub)
    
    pub.destroy_node()
    rclpy.shutdown()