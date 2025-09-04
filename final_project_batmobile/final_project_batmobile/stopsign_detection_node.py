#!/usr/bin/env python3

import json
import time
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist

# RoboflowOak model interface
from roboflowoak import RoboflowOak


class StopSignDetectorNode(Node):
    """
    Detects stop signs with Roboflow + OAK-D-Lite and publishes stop Twist messages to /stop_cmd.
    """

    def __init__(self):
        super().__init__('stopsign_detector_node')

        # --- QoS settings ---
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- Publisher to arbiter ---
        self.stop_pub = self.create_publisher(Twist, '/stop_cmd', qos)

        # --- Roboflow model config ---
        try:
            self.rf = RoboflowOak(
                model='stop-sign-h0vwm',
                version='2',
                api_key='3vRU2WUA78e0FmTjXpDN',
                confidence=0.7,
                overlap=0.5,
                rgb=True,
                depth=False,
                device=None,
                blocking=True
            )
        except Exception as e:
            self.get_logger().error(f'Failed to initialize RoboflowOak: {e}')
            raise

        # --- Detection loop timer ---
        self.frame_count = 0
        self.timer = self.create_timer(0.05, self.run_model_loop)

        self.get_logger().info('StopSignDetectorNode started and ready.')

    def run_model_loop(self):
        try:
            result, frame, raw_frame, depth = self.rf.detect()
            predictions = result.get('predictions', [])

            # --- Log and act on detection ---
            if predictions:
                self.get_logger().info(f'STOP SIGN DETECTED! Count: {len(predictions)}')

                # Send zero-velocity Twist
                stop_twist = Twist()
                stop_twist.linear.x = 0.0
                stop_twist.angular.z = 0.0
                self.stop_pub.publish(stop_twist)

            # Log FPS every ~30 frames
            self.frame_count += 1
            if self.frame_count % 30 == 0:
                self.get_logger().info(f'FPS: {result.get("fps", 0.0):.2f} | Detections: {len(predictions)}')

        except Exception as e:
            self.get_logger().error(f'Error in detection: {e}')

    def destroy_node(self):
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StopSignDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt — Shutting down stop sign detector...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
