import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ConstantSpeedNode(Node):
    def __init__(self):
        super().__init__('constant_speed_node')
        self.publisher = self.create_publisher(Twist, '/constant_cmd', 10)
        self.timer = self.create_timer(0.05, self.publish_cmd)  # 20Hz

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = 0.7
        msg.angular.z = 0.0
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ConstantSpeedNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received. Stopping...')
        stop = Twist()
        stop.linear.x = 0.0
        stop.angular.z = 0.0
        node.publisher.publish(stop)
    finally:
        node.destroy_node()
        rclpy.shutdown()

