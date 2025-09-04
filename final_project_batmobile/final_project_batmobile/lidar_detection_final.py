#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def _wrap_pi(arr):
    """Wrap angles (rad) to [-pi, pi]."""
    return (arr + np.pi) % (2.0 * np.pi) - np.pi


class LidarAvoidance(Node):
    def __init__(self):
        super().__init__('lidar_avoidance_node')

        # ---- Topics ----
        self.publisher = self.create_publisher(Twist, '/lidar_cmd', 10)
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # ---- Tunables ----
        self.obstacle_distance_threshold = 2.0  # meters
        self.fov_deg = 30.0  # check ±(fov_deg/2) around forward
        self.front_offset_deg = 0.0  # rotate scan so 0° = true forward, if needed
        self.turn_speed_min = 0.6  # rad/s at threshold edge
        self.turn_speed_max = 2.0  # rad/s when very close (sharper turns)
        self.avoid_linear_speed = 0.5  # m/s while avoiding
        self.min_range = 0.05  # ignore returns < this (m)

        # ---- Hold behavior ----
        self.hold_sec = 2.5  # << lock the turn for 2 seconds
        self.cmd_rate_hz = 20.0  # republish during hold
        self.yaw_slew_rate = 4.0  # rad/s^2 limit (0 disables slew)

        # ---- State ----
        self._last_log = 0.0
        self._printed_scan_meta = False

        self._hold_active = False
        self._hold_until = 0.0
        self._hold_lin = 0.0
        self._hold_yaw_target = 0.0
        self._yaw_cur = 0.0

        self._last_tick = time.monotonic()
        self.create_timer(1.0 / max(self.cmd_rate_hz, 1.0), self._tick)

        self.get_logger().info(
            "LidarAvoidance up | "
            f"threshold={self.obstacle_distance_threshold:.2f} m, FOV=±{self.fov_deg/2:.0f}°, "
            f"yaw=[{self.turn_speed_min:.2f}..{self.turn_speed_max:.2f}] rad/s, "
            f"avoid_linear={self.avoid_linear_speed:.2f} m/s, hold={self.hold_sec:.1f}s"
        )

    # ---------- LaserScan callback ----------
    def scan_callback(self, msg: LaserScan):
        # If we're currently holding, ignore scans for triggering (no refresh/interrupt).
        if self._hold_active:
            return

        n = len(msg.ranges)
        if n == 0 or msg.angle_increment == 0.0:
            return

        if not self._printed_scan_meta:
            self.get_logger().info(
                "LaserScan meta: "
                f"angle_min={math.degrees(msg.angle_min):.1f}°, "
                f"angle_max={math.degrees(msg.angle_max):.1f}°, "
                f"angle_inc={math.degrees(msg.angle_increment):.3f}°"
            )
            self._printed_scan_meta = True

        # Angles (+CCW=left, -CW=right) and ranges
        angles = msg.angle_min + np.arange(n, dtype=np.float32) * msg.angle_increment
        ranges = np.asarray(msg.ranges, dtype=np.float32)

        # Optional forward offset and wrap to [-pi, pi]
        ang_off = math.radians(self.front_offset_deg)
        angles = _wrap_pi(angles + ang_off)

        # Filter invalid/tiny readings
        valid = np.isfinite(ranges) & (ranges > self.min_range)
        if not np.any(valid):
            return
        angles = angles[valid]
        ranges = ranges[valid]

        # Restrict to forward FOV
        half = math.radians(self.fov_deg * 0.5)
        mask = (angles >= -half) & (angles <= +half)
        if not np.any(mask):
            return
        a = angles[mask]
        r = ranges[mask]

        # Nearest point in the FOV
        i_min = int(np.argmin(r))
        min_d = float(r[i_min])
        ang = float(a[i_min])
        ang_deg = math.degrees(ang)

        # Cartesian position in LiDAR frame (x forward, y left)
        x = min_d * math.cos(ang)
        y = min_d * math.sin(ang)

        # Direction label
        if abs(ang_deg) <= 5.0:
            direction = "CENTER"
        elif ang_deg > 0.0:
            direction = "LEFT"
        else:
            direction = "RIGHT"

        now = time.monotonic()

        if min_d < self.obstacle_distance_threshold:
            # Proximity in [0..1]; higher => closer => sharper turn
            prox = (self.obstacle_distance_threshold - min_d) / self.obstacle_distance_threshold
            prox = max(0.0, min(1.0, prox))
            yaw_mag = self.turn_speed_min + prox * (self.turn_speed_max - self.turn_speed_min)

            # Turn AWAY from obstacle
            yaw = -yaw_mag if ang_deg > 0.0 else yaw_mag

            # Arm the hold ONCE; do not refresh/modify during hold
            self._hold_active = True
            self._hold_until = now + self.hold_sec
            self._hold_lin = self.avoid_linear_speed
            self._hold_yaw_target = yaw
            # Optionally reset the current yaw so slew starts from 0 each hold
            # self._yaw_cur = 0.0

            # Log once on trigger
            self.get_logger().warn(
                f"🚧 HOLD START ({self.hold_sec:.1f}s): nearest={min_d:.2f} m @ "
                f"{'+' if ang_deg>=0 else ''}{ang_deg:.1f}° ({direction}), pos=(x={x:.2f}, y={y:.2f}) m | "
                f"target yaw={self._hold_yaw_target:+.2f} rad/s, lin={self._hold_lin:.2f} m/s"
            )
        else:
            # No hold triggered; optional low-rate log
            if now - self._last_log >= 1.0:
                self.get_logger().info(
                    f"Clear: nearest={min_d:.2f} m @ {'+' if ang_deg>=0 else ''}{ang_deg:.1f}° ({'CENTER' if abs(ang_deg)<=5 else ('LEFT' if ang_deg>0 else 'RIGHT')}), "
                    f"pos=(x={x:.2f}, y={y:.2f}) m"
                )
                self._last_log = now

    # ---------- Periodic publisher during hold ----------
    def _tick(self):
        now = time.monotonic()
        dt = now - self._last_tick
        self._last_tick = now

        if self._hold_active:
            if now < self._hold_until:
                # Slew toward target yaw (optional smoothing)
                if self.yaw_slew_rate > 0.0:
                    max_delta = self.yaw_slew_rate * dt
                    dyaw = self._hold_yaw_target - self._yaw_cur
                    if dyaw > 0.0:
                        self._yaw_cur = min(self._yaw_cur + max_delta, self._hold_yaw_target)
                    else:
                        self._yaw_cur = max(self._yaw_cur - max_delta, self._hold_yaw_target)
                else:
                    self._yaw_cur = self._hold_yaw_target

                out = Twist()
                out.linear.x = self._hold_lin
                out.angular.z = self._yaw_cur
                self.publisher.publish(out)
            else:
                # Hold expired — stop overriding; arbiter falls back to constant path
                self._hold_active = False
                self.get_logger().info("Hold finished → releasing back to constant steering.")


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
