#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ColorDetectOAKSpeed: uses OAK-D (DepthAI) directly to detect GREEN / BLUE / RED in an ROI
and publish target speed to /color_cmd (geometry_msgs/Twist).

- GREEN -> linear.x = green_speed  (should be > base constant speed)
- BLUE  -> linear.x = blue_speed   (should be < base constant speed)
- RED   -> linear.x = 0.0          (stop)

Debounced detections; republishes at most cmd_rate_hz while a color is visible.
Prints startup info, warns if no frames, logs predictions 1 Hz, logs each publish.

Run:
  ros2 run final_project_batmobile color_detection --ros-args \
    -p green_speed:=0.60 -p blue_speed:=0.20 -p min_area_frac:=0.02 \
    -p require_consecutive:=2 -p cmd_rate_hz:=3.0 -p show_debug:=false
"""
import time
import threading
import numpy as np
import cv2
import depthai as dai

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist


class ColorDetectOAKSpeed(Node):
    def __init__(self):
        super().__init__('color_detect_oak_speed')

        # -------- Parameters --------
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('roi_bottom', 0.60)        # start of ROI (fraction of height)
        self.declare_parameter('min_area_frac', 0.02)     # trigger threshold within ROI
        self.declare_parameter('require_consecutive', 2)  # frames to confirm detection
        self.declare_parameter('cmd_rate_hz', 3.0)        # re-publish while visible (Hz)
        self.declare_parameter('show_debug', False)       # OpenCV window
        self.declare_parameter('tune', False)             # HSV trackbars

        # Speeds (set relative to your constant speed)
        self.declare_parameter('green_speed', 1.0)       # > constant speed
        self.declare_parameter('blue_speed',  0.17)       # < constant speed
        self.declare_parameter('red_speed',   0.00)       # stop (keep 0.0)

        # HSV thresholds (OpenCV Hue 0..179)
        # GREEN
        self.declare_parameter('green_h_low', 40)
        self.declare_parameter('green_s_low', 80)
        self.declare_parameter('green_v_low', 70)
        self.declare_parameter('green_h_high', 85)
        self.declare_parameter('green_s_high', 255)
        self.declare_parameter('green_v_high', 255)
        # BLUE
        self.declare_parameter('blue_h_low', 100)
        self.declare_parameter('blue_s_low', 80)
        self.declare_parameter('blue_v_low', 60)
        self.declare_parameter('blue_h_high', 130)
        self.declare_parameter('blue_s_high', 255)
        self.declare_parameter('blue_v_high', 255)
        # RED (two hue bands around 0 and 180)
        self.declare_parameter('red_h1_low', 0)
        self.declare_parameter('red_h1_high', 10)
        self.declare_parameter('red_h2_low', 170)
        self.declare_parameter('red_h2_high', 179)
        self.declare_parameter('red_s_low', 80)
        self.declare_parameter('red_v_low', 60)
        self.declare_parameter('red_s_high', 255)
        self.declare_parameter('red_v_high', 255)

        # Read params
        P = lambda k: self.get_parameter(k).value
        self.w, self.h, self.fps = int(P('width')), int(P('height')), int(P('fps'))
        self.roi_bottom = float(P('roi_bottom'))
        self.min_area   = float(P('min_area_frac'))
        self.require_consec = int(P('require_consecutive'))
        self.cmd_dt     = 1.0 / float(P('cmd_rate_hz'))
        self.show_debug = bool(P('show_debug'))
        self.tune       = bool(P('tune'))

        self.green_speed = float(P('green_speed'))
        self.blue_speed  = float(P('blue_speed'))
        self.red_speed   = float(P('red_speed'))  # default 0.0

        # HSV arrays
        self.g_lo = np.array([int(P('green_h_low')), int(P('green_s_low')), int(P('green_v_low'))], dtype=np.uint8)
        self.g_hi = np.array([int(P('green_h_high')),int(P('green_s_high')),int(P('green_v_high'))], dtype=np.uint8)
        self.b_lo = np.array([int(P('blue_h_low')),  int(P('blue_s_low')),  int(P('blue_v_low'))], dtype=np.uint8)
        self.b_hi = np.array([int(P('blue_h_high')), int(P('blue_s_high')), int(P('blue_v_high'))], dtype=np.uint8)
        self.r1_lo = np.array([int(P('red_h1_low')), int(P('red_s_low')), int(P('red_v_low'))], dtype=np.uint8)
        self.r1_hi = np.array([int(P('red_h1_high')),int(P('red_s_high')),int(P('red_v_high'))], dtype=np.uint8)
        self.r2_lo = np.array([int(P('red_h2_low')), int(P('red_s_low')), int(P('red_v_low'))], dtype=np.uint8)
        self.r2_hi = np.array([int(P('red_h2_high')),int(P('red_s_high')),int(P('red_v_high'))], dtype=np.uint8)

        # Publisher (RELIABLE control)
        cmd_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
        self.cmd_pub = self.create_publisher(Twist, '/color_cmd', cmd_qos)

        # DepthAI pipeline
        try:
            self.device, self.queue = self._start_pipeline(self.w, self.h, self.fps)
        except Exception as e:
            self.get_logger().error(f'Failed to start OAK: {e}')
            raise

        # Optional HSV tuner window
        self._tune_name = None
        if self.show_debug and self.tune:
            self._tune_name = "tune_hsv"
            cv2.namedWindow(self._tune_name, cv2.WINDOW_NORMAL)
            sliders = [
                ('green_h_low', int(self.g_lo[0])), ('green_s_low', int(self.g_lo[1])), ('green_v_low', int(self.g_lo[2])),
                ('green_h_high',int(self.g_hi[0])), ('green_s_high',int(self.g_hi[1])), ('green_v_high',int(self.g_hi[2])),
                ('blue_h_low',  int(self.b_lo[0])), ('blue_s_low',  int(self.b_lo[1])), ('blue_v_low',  int(self.b_lo[2])),
                ('blue_h_high', int(self.b_hi[0])), ('blue_s_high', int(self.b_hi[1])), ('blue_v_high', int(self.b_hi[2])),
                ('red_h1_low',  int(self.r1_lo[0])), ('red_h1_high', int(self.r1_hi[0])),
                ('red_h2_low',  int(self.r2_lo[0])), ('red_h2_high', int(self.r2_hi[0])),
                ('red_s_low',   int(self.r1_lo[1])), ('red_s_high', int(self.r1_hi[1])),
                ('red_v_low',   int(self.r1_lo[2])), ('red_v_high', int(self.r1_hi[2])),
            ]
            for name, val in sliders:
                maxv = 179 if name.endswith(('h_low','h_high')) or name.startswith('red_h') else 255
                cv2.createTrackbar(name, self._tune_name, val, maxv, lambda _x: None)

        # State
        self._prev_verdict = "none"
        self._consec = 0
        self._last_cmd_pub = 0.0
        self._last_log = 0.0
        self._last_frame = 0.0
        self._first_frame_logged = False

        # Start capture thread + heartbeat
        self._thr = threading.Thread(target=self._capture_loop, daemon=True)
        self._thr.start()
        self.create_timer(1.0, self._heartbeat)

        # Startup info
        self.get_logger().info(
            "ColorDetectOAKSpeed starting (DepthAI direct)\n"
            f"  OAK preview: {self.w}x{self.h}@{self.fps} FPS\n"
            f"  speeds: green={self.green_speed:.3f}  blue={self.blue_speed:.3f}  red={self.red_speed:.3f}\n"
            f"  roi_bottom: {self.roi_bottom:.2f}, min_area_frac: {self.min_area:.3f}\n"
            f"  require_consecutive: {self.require_consec}, cmd_rate_hz: {1.0/self.cmd_dt:.1f}"
        )

    # ---------- DepthAI ----------
    def _start_pipeline(self, w, h, fps):
        pipe = dai.Pipeline()
        cam  = pipe.create(dai.node.ColorCamera)
        cam.setPreviewSize(w, h)
        cam.setFps(fps)
        cam.setInterleaved(False)
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        xout = pipe.create(dai.node.XLinkOut)
        xout.setStreamName("rgb")
        cam.preview.link(xout.input)

        dev = dai.Device(pipe)  # Only one process can own the OAK
        q = dev.getOutputQueue("rgb", maxSize=4, blocking=False)
        self.get_logger().info(f"OAK started: {w}x{h}@{fps}")
        return dev, q

    # ---------- Helpers ----------
    @staticmethod
    def _clean_mask(mask):
        mask = cv2.medianBlur(mask, 5)
        k = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=1)
        return mask

    def _update_thresholds_from_trackbars(self):
        if not self._tune_name: return
        # green & blue
        self.g_lo = np.array([cv2.getTrackbarPos('green_h_low', self._tune_name),
                              cv2.getTrackbarPos('green_s_low', self._tune_name),
                              cv2.getTrackbarPos('green_v_low', self._tune_name)], dtype=np.uint8)
        self.g_hi = np.array([cv2.getTrackbarPos('green_h_high', self._tune_name),
                              cv2.getTrackbarPos('green_s_high', self._tune_name),
                              cv2.getTrackbarPos('green_v_high', self._tune_name)], dtype=np.uint8)
        self.b_lo = np.array([cv2.getTrackbarPos('blue_h_low', self._tune_name),
                              cv2.getTrackbarPos('blue_s_low', self._tune_name),
                              cv2.getTrackbarPos('blue_v_low', self._tune_name)], dtype=np.uint8)
        self.b_hi = np.array([cv2.getTrackbarPos('blue_h_high', self._tune_name),
                              cv2.getTrackbarPos('blue_s_high', self._tune_name),
                              cv2.getTrackbarPos('blue_v_high', self._tune_name)], dtype=np.uint8)
        # red (two hue bands share same S/V thresholds)
        r_h1_lo = cv2.getTrackbarPos('red_h1_low', self._tune_name)
        r_h1_hi = cv2.getTrackbarPos('red_h1_high', self._tune_name)
        r_h2_lo = cv2.getTrackbarPos('red_h2_low', self._tune_name)
        r_h2_hi = cv2.getTrackbarPos('red_h2_high', self._tune_name)
        r_s_lo  = cv2.getTrackbarPos('red_s_low', self._tune_name)
        r_s_hi  = cv2.getTrackbarPos('red_s_high', self._tune_name)
        r_v_lo  = cv2.getTrackbarPos('red_v_low', self._tune_name)
        r_v_hi  = cv2.getTrackbarPos('red_v_high', self._tune_name)
        self.r1_lo = np.array([r_h1_lo, r_s_lo, r_v_lo], dtype=np.uint8)
        self.r1_hi = np.array([r_h1_hi, r_s_hi, r_v_hi], dtype=np.uint8)
        self.r2_lo = np.array([r_h2_lo, r_s_lo, r_v_lo], dtype=np.uint8)
        self.r2_hi = np.array([r_h2_hi, r_s_hi, r_v_hi], dtype=np.uint8)

    # ---------- Capture loop ----------
    def _capture_loop(self):
        while rclpy.ok():
            inp = self.queue.tryGet()
            if inp is None:
                time.sleep(0.001)
                continue

            frame = inp.getCvFrame()  # BGR
            self._last_frame = time.monotonic()

            if not self._first_frame_logged:
                h0, w0 = frame.shape[:2]
                self.get_logger().info(f"First frame received: {w0}x{h0}")
                self._first_frame_logged = True

            h, w = frame.shape[:2]
            y0 = int(h * self.roi_bottom)
            roi = frame[y0:h, :]

            if self._tune_name:
                self._update_thresholds_from_trackbars()

            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            mg = self._clean_mask(cv2.inRange(hsv, self.g_lo, self.g_hi))
            mb = self._clean_mask(cv2.inRange(hsv, self.b_lo, self.b_hi))
            # RED = band1 OR band2
            mr1 = self._clean_mask(cv2.inRange(hsv, self.r1_lo, self.r1_hi))
            mr2 = self._clean_mask(cv2.inRange(hsv, self.r2_lo, self.r2_hi))
            mr  = cv2.bitwise_or(mr1, mr2)

            roi_area = float(max(1, roi.shape[0]*roi.shape[1]))
            fg = float(cv2.countNonZero(mg)) / roi_area
            fb = float(cv2.countNonZero(mb)) / roi_area
            fr = float(cv2.countNonZero(mr)) / roi_area

            have_g = fg >= self.min_area
            have_b = fb >= self.min_area
            have_r = fr >= self.min_area

            # Choose the dominant color among those present
            verdict = "none"
            if have_g or have_b or have_r:
                # pick by max area
                areas = {'green': fg if have_g else 0.0,
                         'blue':  fb if have_b else 0.0,
                         'red':   fr if have_r else 0.0}
                verdict = max(areas, key=areas.get)

            # Debounce logic (require consecutive frames)
            if verdict == self._prev_verdict:
                self._consec = self._consec + 1 if verdict != "none" else 0
            else:
                self._consec = 1 if verdict != "none" else 0
                self._prev_verdict = verdict

            now = time.monotonic()

            # Publish /color_cmd while visible (throttled)
            if verdict in ("green", "blue", "red") and self._consec >= self.require_consec:
                if now - self._last_cmd_pub >= self.cmd_dt:
                    out = Twist()
                    if verdict == "green":
                        out.linear.x = self.green_speed
                    elif verdict == "blue":
                        out.linear.x = self.blue_speed
                    else:  # red -> stop
                        out.linear.x = self.red_speed  # typically 0.0
                    out.angular.z = 0.0
                    self.cmd_pub.publish(out)
                    self.get_logger().info(
                        f"PUBLISH /color_cmd: verdict={verdict} speed={out.linear.x:.3f} "
                        f"(green={fg:.3f}, blue={fb:.3f}, red={fr:.3f}, consec={self._consec})"
                    )
                    self._last_cmd_pub = now

            # Optional debug view
            if self.show_debug:
                vis = frame.copy()
                cv2.line(vis, (0, y0), (w, y0), (0,255,255), 2)
                overlay = np.zeros_like(roi)
                overlay[mg>0] = (0,255,0)     # green
                overlay[mb>0] = (255,0,0)     # blue
                overlay[mr>0] = (0,0,255)     # red
                vis[y0:h, :] = cv2.addWeighted(roi, 0.6, overlay, 0.4, 0)
                label = f"{verdict.upper():5s}  G:{fg:.3f}  B:{fb:.3f}  R:{fr:.3f}"
                cv2.putText(vis, label, (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2, cv2.LINE_AA)
                cv2.imshow("color_detect_oak_speed", vis)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    cv2.destroyAllWindows()
                    break

            # Throttled prediction log (1 Hz)
            if now - self._last_log >= 1.0:
                self.get_logger().info(
                    f"PREDICTION: verdict={verdict:5s}  green={fg:.3f}  blue={fb:.3f}  red={fr:.3f}  consec={self._consec}"
                )
                self._last_log = now

    # ---------- Heartbeat ----------
    def _heartbeat(self):
        now = time.monotonic()
        if self._last_frame == 0.0:
            self.get_logger().warn("No frames received from OAK yet...")
        elif now - self._last_frame > 2.0:
            self.get_logger().warn(f"No frames for {now - self._last_frame:.1f}s from OAK")

    def destroy_node(self):
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectOAKSpeed()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
