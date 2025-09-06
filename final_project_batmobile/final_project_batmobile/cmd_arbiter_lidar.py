import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class LidarArbiter(Node):
    def __init__(self):
        super().__init__('lidar_arbiter_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(Twist, '/constant_cmd', self.constant_cb, 10)
        self.create_subscription(Twist, '/lidar_cmd', self.lidar_cb, 10)

        self.constant_cmd = Twist()
        self.lidar_override = None
        self.lidar_timeout = 0.0

        self.timer = self.create_timer(0.05, self.publish_cmd)

    def constant_cb(self, msg):
        self.constant_cmd = msg

    def lidar_cb(self, msg):
        self.lidar_override = msg
        self.lidar_timeout = time.time() + 0.2  # Use for 0.2 seconds

    def publish_cmd(self):
        if self.lidar_override and time.time() < self.lidar_timeout:
            self.cmd_pub.publish(self.lidar_override)
        else:
            self.cmd_pub.publish(self.constant_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LidarArbiter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
