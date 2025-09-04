#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist

class CmdVelColorArbiter(Node):
    def __init__(self):
        super().__init__('cmd_vel_color_arbiter')

        # ---- Parameters ----
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('color_override_sec', 2.0)
        self.declare_parameter('color_reliable', True)     # set False if your /color_cmd publisher is BEST_EFFORT
        self.declare_parameter('const_reliable', True)     # set False if your /constant_cmd publisher is BEST_EFFORT

        rate_hz = float(self.get_parameter('rate_hz').value)
        self.color_override_sec = float(self.get_parameter('color_override_sec').value)
        color_reliable = bool(self.get_parameter('color_reliable').value)
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
        self.create_subscription(Twist, '/color_cmd',    self._color_cb,    sub_qos(color_reliable))

        # ---- State ----
        self._constant_cmd = Twist()               # defaults to zeros until first msg
        self._color_cmd = None                     # last color override Twist (we only use linear.x)
        self._color_until = self.get_clock().now() # ROS time
        self._last_mode = None                     # "COLOR" | "CONST"

        # ---- Timer ----
        self.timer = self.create_timer(1.0 / max(rate_hz, 1.0), self._tick)

        self.get_logger().info(
            f"Color Arbiter up: rate={rate_hz:.1f} Hz, color_window={self.color_override_sec:.1f}s, "
            f"QoS (/constant_cmd={'RELIABLE' if const_reliable else 'BEST_EFFORT'}, "
            f"/color_cmd={'RELIABLE' if color_reliable else 'BEST_EFFORT'})"
        )

    # ---------- Callbacks ----------
    def _constant_cb(self, msg: Twist):
        self._constant_cmd = msg

    def _color_cb(self, msg: Twist):
        # Use only linear.x from color; keep angular from constant path
        self._color_cmd = msg
        self._color_until = self.get_clock().now() + Duration(seconds=self.color_override_sec)

    # ---------- Main loop ----------
    def _tick(self):
        now = self.get_clock().now()
        color_active = (self._color_cmd is not None) and (now < self._color_until)

        if color_active:
            # Take constant command and override linear.x with color speed
            cmd = Twist()
            cmd.linear.x = self._color_cmd.linear.x
            cmd.angular.z = self._constant_cmd.angular.z  # pass-through yaw from constant
            mode = "COLOR"
        else:
            # No valid color override -> just forward constant command
            cmd = self._constant_cmd
            mode = "CONST"

        if mode != self._last_mode:
            if mode == "COLOR":
                self.get_logger().info(f"COLOR override active for up to {self.color_override_sec:.1f}s "
                                       f"(vx={cmd.linear.x:.3f})")
            else:
                self.get_logger().info("Resuming CONSTANT command")
            self._last_mode = mode

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelColorArbiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt -> sending stop")
        stop = Twist()  # zeros
        node.cmd_pub.publish(stop)
        rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
