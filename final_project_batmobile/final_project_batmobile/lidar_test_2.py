import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math


class LidarAvoidance(Node):
    def __init__(self):
        super().__init__('lidar_avoidance_node')
        self.publisher = self.create_publisher(Twist, '/lidar_cmd', 10)
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.obstacle_distance_threshold = 0.6  # meters

    def scan_callback(self, msg):
        # Define the front 30° field of view
        fov_deg = 30
        fov_rad = math.radians(fov_deg)
        center_index = len(msg.ranges) // 2
        angle_increment = msg.angle_increment

        # Calculate indices for the FOV
        half_window = int((fov_rad / 2) / angle_increment)
        start_idx = max(0, center_index - half_window)
        end_idx = min(len(msg.ranges), center_index + half_window)

        min_distance = float('inf')
        min_index = None

        for i in range(start_idx, end_idx):
            distance = msg.ranges[i]
            if 0.01 < distance < min_distance:
                min_distance = distance
                min_index = i

        if min_index is not None and min_distance < self.obstacle_distance_threshold:
            # Obstacle detected
            angle = msg.angle_min + min_index * msg.angle_increment
            angle_deg = math.degrees(angle)

            self.get_logger().info(
                f"🚧 Obstacle detected! Distance: {min_distance:.2f} m | Angle: {angle_deg:.1f}°"
            )

            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.5  # Turn to avoid
            self.publisher.publish(twist)

        else:
            # Clear path, move forward
            twist = Twist()
            twist.linear.x = 0.4
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            self.get_logger().debug("✅ Path clear — moving forward")


def main(args=None):
    rclpy.init(args=args)
    node = LidarAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
