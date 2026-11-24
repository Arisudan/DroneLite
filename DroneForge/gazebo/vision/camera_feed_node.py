#!/usr/bin/env python3
"""DroneForge Universal Camera Feed Node for Live Drone / Perception.
Captures video from a physical device (/dev/video*), network stream
(DJI O4 / CosmoStreamer UDP/RTSP), or test video, and publishes to ROS 2
/droneforge/camera/image_raw so SuperPoint + LightGlue VO runs on real video.

Usage:
  python3 camera_feed_node.py --source 0                      # V4L2 /dev/video0 (webcam / capture card)
  python3 camera_feed_node.py --source udp://127.0.0.1:5600   # DJI O4 via CosmoStreamer MPEG-TS
  python3 camera_feed_node.py --source udp://0.0.0.0:3000     # Direct raw DJI stream
"""

import argparse
import sys
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraFeedNode(Node):
    def __init__(self, source):
        super().__init__("droneforge_camera_feed")
        self.source = source
        self.bridge = CvBridge()
        self.pub_image = self.create_publisher(Image, "/droneforge/camera/image_raw", 10)

        # Parse source (int for /dev/videoN, str for URL/file)
        if isinstance(source, str) and source.isdigit():
            self.cap_source = int(source)
        else:
            self.cap_source = source

        self.get_logger().info(f"Opening video capture source: {self.cap_source}")
        self.cap = cv2.VideoCapture(self.cap_source)

        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open video source: {self.cap_source}")
            sys.exit(1)

        # Configure capture properties for low latency (if V4L2)
        if isinstance(self.cap_source, int):
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.frame_count = 0
        self.t0 = time.time()

        # Run capture loop at up to 60 Hz
        self.timer = self.create_timer(1.0 / 60.0, self.capture_frame)
        self.get_logger().info("Camera Feed Node active. Publishing to /droneforge/camera/image_raw.")

    def capture_frame(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return

        self.frame_count += 1
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_optical_link"
        msg.height = int(frame.shape[0])
        msg.width = int(frame.shape[1])
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = int(frame.shape[1] * 3)
        msg.data = frame.tobytes()
        self.pub_image.publish(msg)

        if self.frame_count % 90 == 0:
            elapsed = time.time() - self.t0
            fps = 90.0 / max(0.001, elapsed)
            self.t0 = time.time()
            h, w = frame.shape[:2]
            self.get_logger().info(f"Streaming {w}x{h} @ {fps:.1f} FPS to perception pipeline")

    def destroy_node(self):
        if self.cap and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description="DroneForge Camera Feed Node")
    parser.add_argument(
        "--source",
        default="0",
        help="Capture source: device index (0, 1) or stream URL (udp://127.0.0.1:5600, rtsp://...)"
    )
    args = parser.parse_args()

    rclpy.init()
    node = CameraFeedNode(source=args.source)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
