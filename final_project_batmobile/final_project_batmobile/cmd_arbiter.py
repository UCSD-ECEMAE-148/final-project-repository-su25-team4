#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist

class CmdVelArbiter(Node):
    def __init__(self):
        super().__init__('cmd_vel_arbiter')

        # ---- Parameters ----
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('stop_override_sec', 3.0)
        self.declare_parameter('stop_reliable', True)      # set False if your /stop_cmd publisher is BEST_EFFORT
        self.declare_parameter('const_reliable', True)     # set False if your /constant_cmd publisher is BEST_EFFORT

        rate_hz = float(self.get_parameter('rate_hz').value)
        self.stop_override_sec = float(self.get_parameter('stop_override_sec').value)
        stop_reliable = bool(self.get_parameter('stop_reliable').value)
        const_reliable = bool(self.get_parameter('const_reliable').value)

        # QoS profiles (tunable to match your publishers)
        def sub_qos(reliable: bool):
            return QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )

        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ---- Publisher ----
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', pub_qos)

        # ---- Subscriptions ----
        self.create_subscription(Twist, '/constant_cmd', self._constant_cb, sub_qos(const_reliable))
        self.create_subscription(Twist, '/stop_cmd', self._stop_cb, sub_qos(stop_reliable))

        # ---- State ----
        self._constant_cmd = Twist()            # defaults to zeros until first msg
        self._stop_until = self.get_clock().now()  # ROS time
        self._last_mode = None                  # "STOP" | "CONST"

        # ---- Timer ----
        self.timer = self.create_timer(1.0 / max(rate_hz, 1.0), self._tick)

        self.get_logger().info(
            f"Arbiter up: rate={rate_hz} Hz, stop_window={self.stop_override_sec}s, "
            f"QoS (/constant_cmd={'RELIABLE' if const_reliable else 'BEST_EFFORT'}, "
            f"/stop_cmd={'RELIABLE' if stop_reliable else 'BEST_EFFORT'})"
        )

    # ---------- Callbacks ----------
    def _constant_cb(self, msg: Twist):
        self._constant_cmd = msg

    def _stop_cb(self, msg: Twist):
        # Force zero twist regardless of incoming values (safety)
        self._stop_until = self.get_clock().now() + Duration(seconds=self.stop_override_sec)

    # ---------- Main loop ----------
    def _tick(self):
        now = self.get_clock().now()
        stop_active = now < self._stop_until

        if stop_active:
            cmd = self._zero_twist()
            mode = "STOP"
        else:
            cmd = self._constant_cmd
            mode = "CONST"

        if mode != self._last_mode:
            if mode == "STOP":
                self.get_logger().warn(f"STOP active for up to {self.stop_override_sec:.1f}s")
            else:
                self.get_logger().info("Resuming CONSTANT command")
            self._last_mode = mode

        self.cmd_pub.publish(cmd)

    @staticmethod
    def _zero_twist():
        t = Twist()
        # all fields already zero by default, but explicit:
        t.linear.x = t.linear.y = t.linear.z = 0.0
        t.angular.x = t.angular.y = t.angular.z = 0.0
        return t

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelArbiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt -> sending stop")
        node.cmd_pub.publish(CmdVelArbiter._zero_twist())
        rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
