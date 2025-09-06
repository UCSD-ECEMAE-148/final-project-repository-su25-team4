#!/usr/bin/env python3
"""
ROS 2 node (no pubs/subs): OAK-D Lite color detector for GREEN / BLUE.
- Uses DepthAI directly (no image topics).
- Prints verdict + mask area fractions to the ROS log ~1 Hz.
- Optional OpenCV window and live HSV tuning sliders.

Run:
  ros2 run final_project_batmobile color_detect_oak --ros-args \
    -p show_debug:=true -p tune:=false -p min_area_frac:=0.02
"""
import time, threading
import numpy as np
import cv2
import depthai as dai

import rclpy
from rclpy.node import Node

class ColorDetectOAK(Node):
    def __init__(self):
        super().__init__('color_detect_oak')

        # -------- Parameters --------
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('roi_bottom', 0.60)    # start of ROI (fraction of height)
        self.declare_parameter('min_area_frac', 0.02) # trigger threshold within ROI
        self.declare_parameter('show_debug', False)   # OpenCV window
        self.declare_parameter('tune', False)         # trackbars to adjust HSV live

        # HSV thresholds (good starters for printed paper indoors)
        self.declare_parameter('green_h_low', 40)
        self.declare_parameter('green_s_low', 80)
        self.declare_parameter('green_v_low', 70)
        self.declare_parameter('green_h_high', 85)
        self.declare_parameter('green_s_high', 255)
        self.declare_parameter('green_v_high', 255)

        self.declare_parameter('blue_h_low', 100)
        self.declare_parameter('blue_s_low', 80)
        self.declare_parameter('blue_v_low', 60)
        self.declare_parameter('blue_h_high', 130)
        self.declare_parameter('blue_s_high', 255)
        self.declare_parameter('blue_v_high', 255)

        # Read params
        P = lambda k: self.get_parameter(k).value
        self.w, self.h, self.fps = int(P('width')), int(P('height')), int(P('fps'))
        self.roi_bottom = float(P('roi_bottom'))
        self.min_area   = float(P('min_area_frac'))
        self.show_debug = bool(P('show_debug'))
        self.tune       = bool(P('tune'))

        # HSV arrays (updated if tune==True)
        self.g_lo = np.array([int(P('green_h_low')), int(P('green_s_low')), int(P('green_v_low'))], dtype=np.uint8)
        self.g_hi = np.array([int(P('green_h_high')),int(P('green_s_high')),int(P('green_v_high'))], dtype=np.uint8)
        self.b_lo = np.array([int(P('blue_h_low')),  int(P('blue_s_low')),  int(P('blue_v_low'))], dtype=np.uint8)
        self.b_hi = np.array([int(P('blue_h_high')), int(P('blue_s_high')), int(P('blue_v_high'))], dtype=np.uint8)

        # DepthAI pipeline
        try:
            self.device, self.queue = self._start_pipeline(self.w, self.h, self.fps)
            self.get_logger().info(f'OAK started: {self.w}x{self.h}@{self.fps}')
        except Exception as e:
            self.get_logger().error(f'Failed to start OAK: {e}')
            raise

        # Optional tuner window
        self._tune_name = None
        if self.show_debug and self.tune:
            self._tune_name = "tune"
            cv2.namedWindow(self._tune_name, cv2.WINDOW_NORMAL)
            for name, val in [
                ('green_h_low', int(self.g_lo[0])), ('green_s_low', int(self.g_lo[1])), ('green_v_low', int(self.g_lo[2])),
                ('green_h_high',int(self.g_hi[0])), ('green_s_high',int(self.g_hi[1])), ('green_v_high',int(self.g_hi[2])),
                ('blue_h_low',  int(self.b_lo[0])), ('blue_s_low',  int(self.b_lo[1])), ('blue_v_low',  int(self.b_lo[2])),
                ('blue_h_high', int(self.b_hi[0])), ('blue_s_high', int(self.b_hi[1])), ('blue_v_high', int(self.b_hi[2])),
            ]:
                cv2.createTrackbar(name, self._tune_name, val, 180 if 'h_' in name else 255, lambda _x: None)

        # Shared state from capture thread
        self._lock = threading.Lock()
        self._verdict = "none"  # 'green' | 'blue' | 'none'
        self._green_frac = 0.0
        self._blue_frac  = 0.0

        # Start capture thread
        self._thr = threading.Thread(target=self._capture_loop, daemon=True)
        self._thr.start()

        # Log once per second
        self._last_log = 0.0
        self.create_timer(1.0/60.0, self._tick)

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

        dev = dai.Device(pipe)  # NOTE: only one process can own the OAK
        q = dev.getOutputQueue("rgb", maxSize=4, blocking=False)
        return dev, q

    # ---------- Vision helpers ----------
    @staticmethod
    def _clean_mask(mask):
        mask = cv2.medianBlur(mask, 5)
        k = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=1)
        return mask

    def _update_thresholds_from_trackbars(self):
        if not self._tune_name: return
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

    def _capture_loop(self):
        while rclpy.ok():
            inp = self.queue.tryGet()
            if inp is None:
                time.sleep(0.001)
                continue

            frame = inp.getCvFrame()  # BGR
            h, w = frame.shape[:2]
            y0 = int(h * self.roi_bottom)
            roi = frame[y0:h, :]

            if self._tune_name:
                self._update_thresholds_from_trackbars()

            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            mg = self._clean_mask(cv2.inRange(hsv, self.g_lo, self.g_hi))
            mb = self._clean_mask(cv2.inRange(hsv, self.b_lo, self.b_hi))

            roi_area = float(max(1, roi.shape[0]*roi.shape[1]))
            fg = float(cv2.countNonZero(mg)) / roi_area
            fb = float(cv2.countNonZero(mb)) / roi_area

            have_g = fg >= self.min_area
            have_b = fb >= self.min_area
            verdict = "none"
            if have_g and have_b:
                verdict = "green" if fg >= fb else "blue"
            elif have_g:
                verdict = "green"
            elif have_b:
                verdict = "blue"

            with self._lock:
                self._verdict = verdict
                self._green_frac = fg
                self._blue_frac  = fb

            if self.show_debug:
                vis = frame.copy()
                cv2.line(vis, (0, y0), (w, y0), (0,255,255), 2)
                # overlay masks as translucent color
                overlay = np.zeros_like(roi)
                overlay[mg>0] = (0,255,0)
                overlay[mb>0] = (255,0,0)
                vis[y0:h, :] = cv2.addWeighted(roi, 0.6, overlay, 0.4, 0)
                label = f"{verdict.upper():5s}  G:{fg:.3f}  B:{fb:.3f}"
                cv2.putText(vis, label, (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2, cv2.LINE_AA)
                cv2.imshow("color_detect_oak", vis)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    cv2.destroyAllWindows()
                    break

    def _tick(self):
        # print once per second
        now = time.monotonic()
        if now - self._last_log >= 1.0:
            with self._lock:
                v, g, b = self._verdict, self._green_frac, self._blue_frac
            self.get_logger().info(f"verdict={v:5s}  green={g:.3f}  blue={b:.3f}")
            self._last_log = now

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectOAK()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
