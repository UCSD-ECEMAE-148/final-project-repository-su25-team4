#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LidarAvoidance
- Subscribes to /scan (LaserScan).
- Computes an avoidance yaw command using a repulsive field in a forward FOV.
- Publishes /lidar_cmd (Twist) with ONLY angular.z set, and only when needed.
- Prints detailed obstacle info: nearest distance/angle/direction, left/right clearance,
  and the chosen yaw command.

This node does NOT change speed; your arbiter should combine this yaw with your constant speed.

Params (ros2 run ... --ros-args -p name:=value):
  scan_topic:           string   LaserScan topic (default: /scan)
  steer_topic:          string   output twist topic (default: /lidar_cmd)
  fov_deg:              float    forward field-of-view centered at 0° (default: 120)
  react_distance:       float    start steering when any point <= this (m) (default: 1.0)
  min_range:            float    ignore returns < this (m) (default: 0.05)
  max_considered_range: float    cap ranges at this for scoring (m) (default: 6.0)
  gain:                 float    scales steering strength (default: 1.2)
  max_yaw_rate:         float    clamp |angular.z| (rad/s) (default: 1.5)
  deadzone:             float    if |yaw| < deadzone => treat as 0 (rad/s) (default: 0.02)
  reliable_qos:         bool     True=RELIABLE sub, False=BEST_EFFORT (default: False)
  print_rate_hz:        float    max console print rate (default: 5.0)
  top_k_report:         int      how many nearest points to list in logs (default: 3)
"""

import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class LidarAvoidance(Node):
    def __init__(self):
        super().__init__('lidar_avoidance_node')

        # -------- Parameters --------
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('steer_topic', '/lidar_cmd')
        self.declare_parameter('fov_deg', 120.0)
        self.declare_parameter('react_distance', 1.0)
        self.declare_parameter('min_range', 0.05)
        self.declare_parameter('max_considered_range', 6.0)
        self.declare_parameter('gain', 1.2)
        self.declare_parameter('max_yaw_rate', 1.5)
        self.declare_parameter('deadzone', 0.02)
        self.declare_parameter('reliable_qos', False)   # many LiDAR drivers are BEST_EFFORT
        self.declare_parameter('print_rate_hz', 5.0)
        self.declare_parameter('top_k_report', 3)

        P = lambda k: self.get_parameter(k).value
        self.scan_topic = str(P('scan_topic'))
        self.steer_topic = str(P('steer_topic'))
        self.fov_deg = float(P('fov_deg'))
        self.react_distance = float(P('react_distance'))
        self.min_range = float(P('min_range'))
        self.max_considered_range = float(P('max_considered_range'))
        self.gain = float(P('gain'))
        self.max_yaw = float(P('max_yaw_rate'))
        self.deadzone = float(P('deadzone'))
        self.print_dt = 1.0 / float(P('print_rate_hz'))
        self.top_k = int(P('top_k_report'))
        reliable = bool(P('reliable_qos'))

        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # -------- Pub/Sub --------
        self.publisher = self.create_publisher(Twist, self.steer_topic, pub_qos)
        self.subscriber = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, sub_qos)

        # -------- State --------
        self._last_print = 0.0

        self.get_logger().info(
            "LidarAvoidance started\n"
            f"  scan_topic: {self.scan_topic}\n"
            f"  steer_topic: {self.steer_topic}\n"
            f"  FOV: ±{self.fov_deg/2:.1f}°  react_distance: {self.react_distance:.2f} m\n"
            f"  gain: {self.gain:.2f}  max_yaw_rate: {self.max_yaw:.2f} rad/s  deadzone: {self.deadzone:.2f}\n"
            f"  QoS: {'RELIABLE' if reliable else 'BEST_EFFORT'}  print_rate: {1.0/self.print_dt:.1f} Hz"
        )

    # -------- Helpers --------
    @staticmethod
    def _clamp(x, lo, hi):
        return lo if x < lo else hi if x > hi else x

    # -------- Main callback --------
    def scan_callback(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0 or msg.angle_increment == 0.0:
            return

        angles = msg.angle_min + np.arange(n, dtype=np.float32) * msg.angle_increment
        ranges = np.asarray(msg.ranges, dtype=np.float32)

        # Filter invalids; cap very large to max_considered_range (helps ranking)
        valid = np.isfinite(ranges) & (ranges >= self.min_range)
        if not np.any(valid):
            return
        angles = angles[valid]
        ranges = ranges[valid]
        ranges = np.minimum(ranges, self.max_considered_range)

        # Restrict to forward FOV
        half = math.radians(self.fov_deg * 0.5)
        fwd = (angles >= -half) & (angles <= +half)
        if not np.any(fwd):
            return
        a = angles[fwd]
        r = ranges[fwd]

        # Nearest points (for reporting)
        k = min(self.top_k, r.size)
        idx_sorted = np.argpartition(r, k-1)[:k]
        nearest = sorted([(float(r[i]), float(a[i])) for i in idx_sorted], key=lambda x: x[0])

        # Left/right clearance: min distance in halves
        left_mask = a > 0.0
        right_mask = a < 0.0
        left_min = float(np.min(r[left_mask])) if np.any(left_mask) else float('inf')
        right_min = float(np.min(r[right_mask])) if np.any(right_mask) else float('inf')

        # Repulsive field only from points within react_distance
        react_mask = r <= self.react_distance
        if np.any(react_mask):
            ar = a[react_mask]
            rr = r[react_mask]

            # Weights: closer points push harder (1/r^2)
            w = 1.0 / np.maximum(rr, self.min_range)**2

            # Repulsion heading = away from obstacle angle (add pi)
            vx = np.sum(w * np.cos(ar + math.pi))
            vy = np.sum(w * np.sin(ar + math.pi))
            desired_heading = math.atan2(vy, vx)  # +left, -right

            yaw = self.gain * desired_heading
            if abs(yaw) < self.deadzone:
                yaw = 0.0
            yaw = self._clamp(yaw, -self.max_yaw, self.max_yaw)

            # Publish steer override (ONLY angular.z)
            out = Twist()
            out.angular.z = float(yaw)
            self.publisher.publish(out)

            reason = "avoid"
        else:
            # No close obstacles: do NOT publish (arbiter will fall back to constant yaw)
            yaw = 0.0
            reason = "clear"

        # ---- Pretty printing (throttled) ----
        now = time.monotonic()
        if now - self._last_print >= self.print_dt:
            if nearest:
                d0, a0 = nearest[0]
                ang_deg0 = math.degrees(a0)
                dir0 = "CENTER" if abs(ang_deg0) <= 5.0 else ("LEFT" if ang_deg0 > 0 else "RIGHT")
                nearest_str = f"{d0:.2f} m @ {'+' if ang_deg0>=0 else ''}{ang_deg0:.1f}° ({dir0})"
            else:
                nearest_str = "n/a"

            # List top-k
            parts = []
            for d, ang in nearest:
                angd = math.degrees(ang)
                parts.append(f"{d:.2f}m@{'+' if angd>=0 else ''}{angd:.1f}°")
            topk_str = ", ".join(parts)

            self.get_logger().info(
                f"[{reason.upper()}] nearest: {nearest_str} | "
                f"left_clear: {'inf' if math.isinf(left_min) else f'{left_min:.2f} m'}  "
                f"right_clear: {'inf' if math.isinf(right_min) else f'{right_min:.2f} m'} | "
                f"yaw_cmd: {yaw:+.3f} rad/s | top{self.top_k}: {topk_str}"
            )
            self._last_print = now


def main(args=None):
    rclpy.init(args=args)
    node = LidarAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
