#!/usr/bin/env python3
"""DroneForge DJI O4 FPV Camera Viewer & Live Telemetry HUD
Renders the real-time camera feed from the simulated DJI O4 camera
with an authentic FPV OSD and flight controller keyboard controls.
"""

import json
import math
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String


class CameraViewerNode(Node):
    def __init__(self):
        super().__init__("droneforge_camera_viewer")
        self.bridge = CvBridge()
        self.latest_frame = None
        self.last_frame_time = time.time()
        self.fps = 0.0
        self.frame_count = 0

        self.telem = {
            "mode": "DISARMED",
            "z": 0.0,
            "vz": 0.0,
            "vx": 0.0,
            "vy": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "vbat": 16.2,
            "m": [0.0, 0.0, 0.0, 0.0],
            "range": 0.0
        }

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscribers
        self.create_subscription(Image, "/droneforge/camera/image_raw", self.on_image, sensor_qos)
        self.create_subscription(String, "/droneforge/fc/telemetry", self.on_telem, 10)
        self.create_subscription(String, "/droneforge/vision/telemetry", self.on_vision, 10)

        # Vision State Cache
        self.vision = {
            "anchor_locked": False,
            "anchor_matches": 0,
            "num_kpts": 0,
            "err_fwd_m": 0.0,
            "err_right_m": 0.0,
            "vx_m_s": 0.0,
            "vy_m_s": 0.0,
            "fps": 0.0,
            "tracks": []
        }

        # Publisher for flight control commands
        self.pub_cmd = self.create_publisher(String, "/droneforge/fc/command", 10)

        self.get_logger().info("DJI O4 Camera Viewer initialized. Waiting for video stream...")

    def on_telem(self, msg: String):
        try:
            self.telem = json.loads(msg.data)
        except Exception:
            pass

    def on_vision(self, msg: String):
        try:
            self.vision = json.loads(msg.data)
        except Exception:
            pass

    def on_image(self, msg: Image):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            now = time.time()
            self.frame_count += 1
            if now - self.last_frame_time >= 1.0:
                self.fps = self.frame_count / (now - self.last_frame_time)
                self.frame_count = 0
                self.last_frame_time = now
        except Exception as e:
            self.get_logger().error(f"Image decode error: {e}")

    def send_cmd(self, cmd_text: str):
        msg = String()
        msg.data = cmd_text
        self.pub_cmd.publish(msg)
        self.get_logger().info(f"Command Sent -> {cmd_text}")

    def draw_hud(self, img):
        h, w = img.shape[:2]
        cx, cy = w // 2, h // 2

        # Semi-transparent overlay style
        hud = img.copy()

        # Colors
        GREEN = (0, 230, 0)
        CYAN = (230, 230, 0)
        WHITE = (240, 240, 240)
        RED = (0, 0, 230)
        ORANGE = (0, 165, 255)
        DARK = (20, 20, 20)

        # 1. Top Status Bar
        cv2.rectangle(hud, (0, 0), (w, 42), DARK, -1)
        cv2.addWeighted(hud, 0.6, img, 0.4, 0, img)

        mode_str = self.telem.get("mode", "DISARMED")
        mode_color = GREEN if "HOLD" in mode_str else (ORANGE if mode_str in ("ARMED", "TAKEOFF") else RED)
        cv2.putText(img, f"DJI O4 HD  [720p 30fps]", (20, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, CYAN, 2)
        cv2.putText(img, f"MODE: {mode_str}", (w // 2 - 80, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.75, mode_color, 2)

        vbat = self.telem.get("vbat", 16.2)
        cv2.putText(img, f"{vbat:.1f} V (4S)", (w - 180, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, GREEN, 2)

        # 2. Artificial Horizon / Crosshair
        roll_rad = math.radians(self.telem.get("roll", 0.0))
        pitch_deg = self.telem.get("pitch", 0.0)

        # Draw SuperPoint + LightGlue Feature Tracks
        tracks = self.vision.get("tracks", [])
        sx = w / 320.0
        sy = h / 180.0
        for tr in tracks:
            p0 = (int(tr[0] * sx), int(tr[1] * sy))
            p1 = (int(tr[2] * sx), int(tr[3] * sy))
            cv2.circle(img, p1, 3, GREEN, -1)
            dist_px = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            if dist_px > 2.0:
                cv2.arrowedLine(img, p0, p1, (0, 255, 120), 1, tipLength=0.25)

        # Vision Status Panel (Upper Center)
        v_locked = self.vision.get("anchor_locked", False)
        v_matches = self.vision.get("anchor_matches", 0)
        v_kpts = self.vision.get("num_kpts", 0)
        v_err_fwd = self.vision.get("err_fwd_m", 0.0)
        v_err_right = self.vision.get("err_right_m", 0.0)
        v_fps = self.vision.get("fps", 0.0)

        cv2.rectangle(img, (w // 2 - 250, 48), (w // 2 + 250, 92), (25, 25, 25), -1)
        if v_locked:
            cv2.putText(img, f"SUPERPOINT + LIGHTGLUE: LOCKED [{v_matches} MATCHES]", (w // 2 - 235, 68),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, GREEN, 2)
            cv2.putText(img, f"POS ERR: FWD {v_err_fwd:+5.2f}m | RT {v_err_right:+5.2f}m ({v_fps:.1f} FPS)",
                        (w // 2 - 235, 86), cv2.FONT_HERSHEY_SIMPLEX, 0.45, CYAN, 1)
        else:
            cv2.putText(img, f"SUPERPOINT + LIGHTGLUE: TRACKING ({v_kpts} kpts)", (w // 2 - 210, 68),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, ORANGE, 2)
            cv2.putText(img, f"Waiting for POS_HOLD anchor... ({v_fps:.1f} FPS)", (w // 2 - 160, 86),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)

        # Pitch offset pixels
        pitch_px = int(pitch_deg * 4.0)

        # Crosshair center
        cv2.drawMarker(img, (cx, cy), WHITE, cv2.MARKER_CROSS, 20, 1)

        # Horizon line rotated by roll
        cos_r = math.cos(roll_rad)
        sin_r = math.sin(roll_rad)
        bar_len = 80
        x1 = int(cx - bar_len * cos_r)
        y1 = int(cy + pitch_px - bar_len * sin_r)
        x2 = int(cx + bar_len * cos_r)
        y2 = int(cy + pitch_px + bar_len * sin_r)
        cv2.line(img, (x1, y1), (x2, y2), GREEN, 2)

        # 3. Altitude & Vertical Speed (Left OSD)
        alt = self.telem.get("z", 0.0)
        vz = self.telem.get("vz", 0.0)
        cv2.rectangle(img, (20, h - 140), (220, h - 60), (30, 30, 30), -1)
        cv2.putText(img, f"ALT: {alt:5.2f} m", (30, h - 105), cv2.FONT_HERSHEY_SIMPLEX, 0.75, WHITE, 2)
        vz_color = GREEN if abs(vz) < 0.1 else ORANGE
        cv2.putText(img, f"V_SPD: {vz:+4.2f} m/s", (30, h - 75), cv2.FONT_HERSHEY_SIMPLEX, 0.60, vz_color, 2)

        # 4. Ground Speed & Motor Outputs (Right OSD)
        vx = self.telem.get("vx", 0.0)
        vy = self.telem.get("vy", 0.0)
        spd = math.hypot(vx, vy)
        cv2.rectangle(img, (w - 240, h - 140), (w - 20, h - 60), (30, 30, 30), -1)
        cv2.putText(img, f"H_SPD: {spd:4.2f} m/s", (w - 225, h - 105), cv2.FONT_HERSHEY_SIMPLEX, 0.70, WHITE, 2)

        motors = self.telem.get("m", [0, 0, 0, 0])
        m_str = f"M: {int(motors[0]*100)} {int(motors[1]*100)} {int(motors[2]*100)} {int(motors[3]*100)} %"
        cv2.putText(img, m_str, (w - 225, h - 75), cv2.FONT_HERSHEY_SIMPLEX, 0.50, CYAN, 1)

        # 5. Bottom Controls Guide Bar
        cv2.rectangle(img, (0, h - 45), (w, h), DARK, -1)
        guide = "[A] Arm  |  [T] Takeoff (0.50m)  |  [H] Alt/Pos Hold  |  [L] Land  |  [D] Disarm  |  [Q] Quit"
        cv2.putText(img, guide, (w // 2 - 360, h - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

        return img


def main(args=None):
    rclpy.init(args=args)
    node = CameraViewerNode()

    window_name = "DJI O4 Feed - DroneForge v4.5 Simulation"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)

            if node.latest_frame is not None:
                display_img = node.latest_frame.copy()
                display_img = node.draw_hud(display_img)
                cv2.imshow(window_name, display_img)
            else:
                # Standby splash screen while waiting for video feed
                splash = np.zeros((720, 1280, 3), dtype=np.uint8)
                cv2.putText(splash, "DJI O4 AIR UNIT - CONNECTING TO GAZEBO SIMULATION...",
                            (260, 340), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 220, 220), 2)
                cv2.putText(splash, "Waiting for /droneforge/camera/image_raw topic...",
                            (380, 390), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 150, 150), 1)
                cv2.imshow(window_name, splash)

            key = cv2.waitKey(10) & 0xFF
            if key in (ord('q'), ord('Q'), 27):  # Q or Esc
                break
            elif key in (ord('a'), ord('A')):
                node.send_cmd("arm")
            elif key in (ord('t'), ord('T')):
                node.send_cmd("takeoff")
            elif key in (ord('h'), ord('H')):
                cur = node.telem.get("mode", "")
                if cur == "POS_HOLD":
                    node.send_cmd("alt_hold")
                else:
                    node.send_cmd("pos_hold")
            elif key in (ord('l'), ord('L')):
                node.send_cmd("land")
            elif key in (ord('d'), ord('D')):
                node.send_cmd("disarm")

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
