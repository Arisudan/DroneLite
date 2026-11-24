#!/usr/bin/env python3
"""DroneForge Visual Odometry & Station-Keeping Node
Powered by SuperPoint and LightGlue ONNX models.

Subscribes to DJI O4 camera feed (/droneforge/camera/image_raw) and flight telemetry.
Extracts deep invariant features, tracks visual anchors with zero-drift LightGlue matching,
and publishes high-rate metric position error and body velocity for Betaflight POS HOLD.
"""

import json
import math
import os
import sys
import time

import cv2
import numpy as np
import onnxruntime as ort
try: ort.preload_dlls()                                      # pip CUDA / cuDNN libraries without LD_LIBRARY_PATH
except Exception: pass
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


class SuperPointLightGlueVO:
    def __init__(self, sp_path: str, lg_path: str, max_kpts: int = 256):
        self.max_kpts = max_kpts
        self.target_w = 320
        self.target_h = 180

        # ONNX Runtime session options for fast CPU inference
        sess_opts = ort.SessionOptions()
        sess_opts.intra_op_num_threads = 4
        sess_opts.inter_op_num_threads = 1
        sess_opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL

        providers = ["CPUExecutionProvider"]
        self.sp_session = ort.InferenceSession(sp_path, sess_opts, providers=providers)
        self.lg_session = ort.InferenceSession(lg_path, sess_opts, providers=providers)

        # Camera intrinsics at 320x180 (DJI O4 120 deg HFOV)
        self.fx = (self.target_w / 2.0) / math.tan(math.radians(60.0))
        self.fy = self.fx
        self.cx = self.target_w / 2.0
        self.cy = self.target_h / 2.0

        # Anchor Keyframe State
        self.anchor_img = None
        self.anchor_kpts = None
        self.anchor_desc = None
        self.anchor_z = 0.50
        self.anchor_roll = 0.0
        self.anchor_pitch = 0.0
        self.anchor_time = 0.0

        # Frame-to-frame VO State
        self.prev_kpts = None
        self.prev_desc = None
        self.prev_roll = 0.0
        self.prev_pitch = 0.0
        self.prev_time = 0.0

        # Cumulative odometry
        self.x_vo = 0.0
        self.y_vo = 0.0
        self.vx = 0.0
        self.vy = 0.0

    def extract_features(self, gray_img: np.ndarray):
        """Extracts SuperPoint keypoints and descriptors."""
        h, w = gray_img.shape[:2]
        if (w, h) != (self.target_w, self.target_h):
            gray_img = cv2.resize(gray_img, (self.target_w, self.target_h), interpolation=cv2.INTER_AREA)

        inp = (gray_img.astype(np.float32) / 255.0)[None, None, :, :]
        kpts, scores, desc = self.sp_session.run(None, {"image": inp})

        num = kpts.shape[1]
        if self.max_kpts is not None and num > self.max_kpts:
            top_idx = np.argsort(scores[0])[::-1][: self.max_kpts]
            kpts = kpts[:, top_idx]
            scores = scores[:, top_idx]
            desc = desc[:, top_idx]

        return kpts, scores, desc

    def normalize_keypoints(self, kpts: np.ndarray):
        """LightGlue keypoint normalization: (kpts - size/2) / (max(size)/2)."""
        size = np.array([self.target_w, self.target_h], dtype=np.float32)
        shift = size / 2.0
        scale = float(np.max(size)) / 2.0
        return ((kpts.astype(np.float32) - shift) / scale).astype(np.float32)

    def match_features(self, kpts0, desc0, kpts1, desc1):
        """Matches features using LightGlue."""
        if kpts0.shape[1] < 4 or kpts1.shape[1] < 4:
            return np.empty((0, 2), dtype=np.int64), np.empty((0,), dtype=np.float32)

        kpts0_norm = self.normalize_keypoints(kpts0)
        kpts1_norm = self.normalize_keypoints(kpts1)

        matches, mscores = self.lg_session.run(
            None,
            {
                "kpts0": kpts0_norm,
                "kpts1": kpts1_norm,
                "desc0": desc0.astype(np.float32),
                "desc1": desc1.astype(np.float32),
            },
        )
        return matches, mscores

    def set_anchor(self, gray_img: np.ndarray, z_alt: float, roll: float = 0.0, pitch: float = 0.0):
        """Records current frame and attitude as station-keeping visual anchor."""
        kpts, scores, desc = self.extract_features(gray_img)
        self.anchor_img = gray_img.copy()
        self.anchor_kpts = kpts
        self.anchor_desc = desc
        self.anchor_z = max(0.08, z_alt)
        self.anchor_roll = roll
        self.anchor_pitch = pitch
        self.anchor_time = time.time()
        return kpts.shape[1]

    def estimate_drift(self, curr_img: np.ndarray, z_alt: float, dt: float, roll: float = 0.0, pitch: float = 0.0):
        """Computes metric station-keeping error against anchor and velocity with gyro de-rotation."""
        kpts_curr, scores_curr, desc_curr = self.extract_features(curr_img)
        num_kpts = kpts_curr.shape[1]

        result = {
            "num_kpts": num_kpts,
            "anchor_matches": 0,
            "anchor_locked": False,
            "err_forward_m": 0.0,
            "err_right_m": 0.0,
            "vx_m_s": 0.0,
            "vy_m_s": 0.0,
            "tracks": [],
        }

        if num_kpts < 4:
            return result

        current_z = max(0.08, z_alt)

        # 1. Match against Anchor for absolute station-keeping
        if self.anchor_kpts is not None and self.anchor_kpts.shape[1] >= 4:
            matches_anc, scores_anc = self.match_features(
                self.anchor_kpts, self.anchor_desc, kpts_curr, desc_curr
            )
            # Filter matches by confidence
            valid_mask = scores_anc > 0.15
            matches_anc = matches_anc[valid_mask]
            scores_anc = scores_anc[valid_mask]

            if len(matches_anc) >= 4:
                pts_anc = self.anchor_kpts[0, matches_anc[:, 0]].astype(np.float32)
                pts_curr = kpts_curr[0, matches_anc[:, 1]].astype(np.float32)

                # Robust 2D affine / rigid motion estimation via RANSAC
                M, inliers = cv2.estimateAffinePartial2D(
                    pts_anc, pts_curr, method=cv2.RANSAC, ransacReprojThreshold=3.0
                )

                if M is not None and inliers is not None and np.sum(inliers) >= 4:
                    inlier_idx = np.where(inliers.ravel() == 1)[0]
                    result["anchor_matches"] = int(len(inlier_idx))
                    result["anchor_locked"] = True

                    # Raw optical feature shift from anchor to current frame
                    du = float(M[0, 2])
                    dv = float(M[1, 2])

                    # Gyro de-rotation: subtract rotational displacement from camera tilt
                    d_roll = roll - self.anchor_roll
                    d_pitch = pitch - self.anchor_pitch
                    du_rot = -self.fx * math.tan(d_roll)
                    dv_rot = self.fy * math.tan(d_pitch)

                    du_trans = du - du_rot
                    dv_trans = dv - dv_rot

                    # Scale pure translational shift to metric metres
                    scale = current_z / self.fx
                    err_fwd = -dv_trans * scale
                    err_right = -du_trans * scale

                    result["err_forward_m"] = round(float(err_fwd), 4)
                    result["err_right_m"] = round(float(err_right), 4)

                    # Export sample tracks for HUD overlay (up to 30 tracks)
                    sample_inliers = inlier_idx[:30]
                    for idx in sample_inliers:
                        p0 = pts_anc[idx]
                        p1 = pts_curr[idx]
                        result["tracks"].append(
                            [float(p0[0]), float(p0[1]), float(p1[0]), float(p1[1])]
                        )

        # 2. Frame-to-frame matching for smooth velocity estimation
        if self.prev_kpts is not None and self.prev_kpts.shape[1] >= 4 and dt > 0.001:
            matches_prev, scores_prev = self.match_features(
                self.prev_kpts, self.prev_desc, kpts_curr, desc_curr
            )
            valid_prev = scores_prev > 0.15
            matches_prev = matches_prev[valid_prev]

            if len(matches_prev) >= 4:
                p_prev = self.prev_kpts[0, matches_prev[:, 0]].astype(np.float32)
                p_curr = kpts_curr[0, matches_prev[:, 1]].astype(np.float32)

                M_vo, inliers_vo = cv2.estimateAffinePartial2D(
                    p_prev, p_curr, method=cv2.RANSAC, ransacReprojThreshold=3.0
                )
                if M_vo is not None and inliers_vo is not None and np.sum(inliers_vo) >= 4:
                    du_vo = float(M_vo[0, 2])
                    dv_vo = float(M_vo[1, 2])

                    # Frame-to-frame gyro de-rotation
                    d_roll_step = roll - self.prev_roll
                    d_pitch_step = pitch - self.prev_pitch
                    du_vo_rot = -self.fx * math.tan(d_roll_step)
                    dv_vo_rot = self.fy * math.tan(d_pitch_step)

                    du_vo_trans = du_vo - du_vo_rot
                    dv_vo_trans = dv_vo - dv_vo_rot

                    scale_vo = current_z / self.fx
                    dx_fwd = -dv_vo_trans * scale_vo
                    dy_right = -du_vo_trans * scale_vo

                    inst_vx = dx_fwd / dt
                    inst_vy = dy_right / dt

                    alpha_v = 0.35
                    self.vx = (1.0 - alpha_v) * self.vx + alpha_v * inst_vx
                    self.vy = (1.0 - alpha_v) * self.vy + alpha_v * inst_vy

                    self.x_vo += dx_fwd
                    self.y_vo += dy_right

        result["vx_m_s"] = round(float(self.vx), 4)
        result["vy_m_s"] = round(float(self.vy), 4)

        # Update previous frame state
        self.prev_kpts = kpts_curr
        self.prev_desc = desc_curr
        self.prev_roll = roll
        self.prev_pitch = pitch
        self.prev_time = time.time()

        return result


class DroneForgeVONode(Node):
    def __init__(self):
        super().__init__("droneforge_vo_node")
        self.bridge = CvBridge()

        # Locate ONNX models
        here = os.path.dirname(os.path.abspath(__file__))
        sp_path = os.path.join(here, "models", "superpoint.onnx")
        lg_path = os.path.join(here, "models", "superpoint_lightglue.onnx")

        if not os.path.exists(sp_path) or not os.path.exists(lg_path):
            alt_here = os.path.expanduser("~/Pictures/DRONEforge/DroneForge/gazebo/vision/models")
            sp_path = os.path.join(alt_here, "superpoint.onnx")
            lg_path = os.path.join(alt_here, "superpoint_lightglue.onnx")

        self.get_logger().info(f"Loading SuperPoint model: {sp_path}")
        self.get_logger().info(f"Loading LightGlue model: {lg_path}")
        self.vo = SuperPointLightGlueVO(sp_path, lg_path, max_kpts=128)

        # Drone State Cache
        self.current_z = 0.50
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_mode = "DISARMED"
        self.last_proc_time = time.time()
        self.frame_count = 0
        self.fps = 0.0
        self.anchor_requested = False

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )

        # Subscribers
        self.create_subscription(
            Image, "/droneforge/camera/image_raw", self.on_camera_image, sensor_qos
        )
        self.create_subscription(String, "/droneforge/fc/telemetry", self.on_telemetry, 10)
        self.create_subscription(String, "/droneforge/fc/command", self.on_command, 10)

        # Publishers
        self.pub_vision = self.create_publisher(String, "/droneforge/vision/telemetry", 10)

        self.get_logger().info(
            "SuperPoint + LightGlue Visual Odometry Node Initialized and Ready."
        )

    def on_telemetry(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.current_z = float(data.get("z", 0.50))
            self.current_roll = math.radians(float(data.get("roll", 0.0)))
            self.current_pitch = math.radians(float(data.get("pitch", 0.0)))
            new_mode = data.get("mode", self.current_mode)
            if new_mode == "POS_HOLD" and self.current_mode != "POS_HOLD":
                # Mode just switched to POS_HOLD -> set visual anchor
                self.anchor_requested = True
            self.current_mode = new_mode
        except Exception:
            pass

    def on_command(self, msg: String):
        raw = msg.data.strip().lower()
        cmd = raw
        if raw.startswith("{"):
            try:
                cmd = json.loads(raw).get("cmd", "").lower()
            except Exception:
                pass
        if cmd in ("hold", "pos_hold", "reposition", "anchor", "takeoff"):
            self.anchor_requested = True

    def on_camera_image(self, msg: Image):
        now = time.time()
        dt = max(0.005, now - self.last_proc_time)
        self.last_proc_time = now

        self.frame_count += 1
        if self.frame_count % 15 == 0:
            self.fps = 15.0 / max(0.001, (now - getattr(self, "_last_fps_t", now)))
            self._last_fps_t = now

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
            return

        # If anchor reset was requested or hover is reached without anchor
        if self.anchor_requested or (
            self.current_mode in ("POS_HOLD", "ALT_HOLD")
            and self.vo.anchor_kpts is None
            and self.current_z > 0.15
        ):
            kpts_cnt = self.vo.set_anchor(
                gray, self.current_z, roll=self.current_roll, pitch=self.current_pitch
            )
            self.anchor_requested = False
            self.get_logger().info(
                f"[VISION] Station-keeping anchor set at z={self.current_z:.2f} m with {kpts_cnt} SuperPoint features."
            )

        # Run drift estimation with gyro de-rotation
        res = self.vo.estimate_drift(
            gray,
            self.current_z,
            dt,
            roll=self.current_roll,
            pitch=self.current_pitch,
        )

        # Publish vision telemetry
        out = {
            "mode": self.current_mode,
            "anchor_locked": res["anchor_locked"],
            "anchor_matches": res["anchor_matches"],
            "num_kpts": res["num_kpts"],
            "err_fwd_m": res["err_forward_m"],
            "err_right_m": res["err_right_m"],
            "vx_m_s": res["vx_m_s"],
            "vy_m_s": res["vy_m_s"],
            "z_m": round(self.current_z, 3),
            "fps": round(self.fps, 1),
            "tracks": res["tracks"],
        }

        msg_out = String()
        msg_out.data = json.dumps(out)
        self.pub_vision.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = DroneForgeVONode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
