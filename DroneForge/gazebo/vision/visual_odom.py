#!/usr/bin/env python3
"""RGB-D visual odometry for the forward O4 camera: SuperPoint features + LightGlue matching against a KEYFRAME (anchor), metric scale from the DA3 depth, pose by PnP.
Output (/droneforge/vo/pose, JSON): where the camera is relative to the anchor's camera, in body terms: d_fwd, d_left, d_up (m), d_yaw (rad, + = turned left), inliers.
The Kalman filter uses it as a drift-free position measurement while the anchor stays in view (position = anchor position + R(anchor yaw) * [d_fwd, d_left]).
The old vo_node.py read pixel shifts as if the camera looked at the floor; this one uses the depth, so it works for a forward-looking camera.
No flight commands: it only publishes a measurement."""
import argparse, json, math, os, sys, time
import numpy as np
import cv2
import onnxruntime as ort
try: ort.preload_dlls()                                      # loads the pip CUDA / cuDNN libraries so CUDA works without LD_LIBRARY_PATH
except Exception: pass
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
HERE = os.path.dirname(os.path.abspath(__file__))
FX = 292.2957; CX = 320.0; CY = 240.0           # the rectified Nimbus-profile picture (640x480)
K = np.array([[FX, 0, CX], [0, FX, CY], [0, 0, 1.0]])


def pnp_motion(obj_pts, img_pts, K=K):
    """obj_pts: Nx3 points in the ANCHOR camera frame (x right, y down, z forward), img_pts: Nx2 pixels in the CURRENT frame.
    Returns (d_fwd, d_left, d_up, d_yaw_left, n_inliers) of the current camera relative to the anchor camera, or None."""
    if len(obj_pts) < 8: return None
    ok, rvec, tvec, inl = cv2.solvePnPRansac(obj_pts.astype(np.float64), img_pts.astype(np.float64), K, None, iterationsCount=150, reprojectionError=3.0, confidence=0.99, flags=cv2.SOLVEPNP_EPNP)
    if not ok or inl is None or len(inl) < 8: return None
    R, _ = cv2.Rodrigues(rvec); C = -R.T @ tvec.ravel()               # camera centre of the current frame in the anchor frame
    Rc = R.T                                                           # current camera axes expressed in the anchor frame
    yaw_cw = math.atan2(Rc[0, 2], Rc[2, 2])                            # rotation about the anchor's down axis (clockwise seen from above)
    return float(C[2]), float(-C[0]), float(-C[1]), float(-yaw_cw), int(len(inl))


class RgbdVo:
    def __init__(self, models_dir, gpu=True):
        so = ort.SessionOptions(); so.intra_op_num_threads = 4; prov = ["CPUExecutionProvider"]
        if gpu and "CUDAExecutionProvider" in ort.get_available_providers(): prov = ["CUDAExecutionProvider", "CPUExecutionProvider"]
        self.sp = ort.InferenceSession(os.path.join(models_dir, "superpoint.onnx"), so, providers=prov); self.lg = ort.InferenceSession(os.path.join(models_dir, "superpoint_lightglue.onnx"), so, providers=prov)
        self.w, self.h = 320, 240; self.anchor = None; self.anchor_id = 0; self.provider = self.sp.get_providers()[0]

    def features(self, gray, max_kpts=300):
        g = cv2.resize(gray, (self.w, self.h), interpolation=cv2.INTER_AREA); k, s, d = self.sp.run(None, {"image": (g.astype(np.float32) / 255.0)[None, None]})
        if k.shape[1] > max_kpts: i = np.argsort(s[0])[::-1][:max_kpts]; k, s, d = k[:, i], s[:, i], d[:, i]
        return k, d                                                   # keypoints in the 320x240 picture

    def match(self, k0, d0, k1, d1):
        if k0.shape[1] < 8 or k1.shape[1] < 8: return np.empty((0, 2), np.int64)
        n = lambda k: ((k.astype(np.float32) - np.array([self.w / 2, self.h / 2], np.float32)) / (self.w / 2)).astype(np.float32)
        m, _ = self.lg.run(None, {"kpts0": n(k0), "kpts1": n(k1), "desc0": d0.astype(np.float32), "desc1": d1.astype(np.float32)}); return m

    def set_anchor(self, gray, depth_mm, stamp):
        k, d = self.features(gray); px = k[0] * 2.0                    # to 640x480 pixels
        u = np.clip(px[:, 0].astype(int), 0, 639); v = np.clip(px[:, 1].astype(int), 0, 479); z = depth_mm[v, u].astype(np.float32) / 1000.0; ok = (z > 0.3) & (z < 5.0)
        pts = np.column_stack([(px[:, 0] - CX) * z / FX, (px[:, 1] - CY) * z / FX, z]); self.anchor_id += 1
        self.anchor = dict(k=k, d=d, pts=pts, ok=ok, stamp=stamp, id=self.anchor_id); return int(ok.sum())

    def track(self, gray, depth_mm=None):
        a = self.anchor; k1, d1 = self.features(gray); m = self.match(a["k"], a["d"], k1, d1)
        if len(m) < 8: return None, len(m)
        sel = m[a["ok"][m[:, 0]]]
        if len(sel) < 8: return None, len(m)
        return pnp_motion(a["pts"][sel[:, 0]], k1[0, sel[:, 1]] * 2.0), len(m)


class VoNode(Node):
    def __init__(self, a):
        super().__init__("droneforge_visual_odom"); self.vo = RgbdVo(a.models, not a.cpu); self.a = a; self.pending = {}; self.last = 0.0; self.n = 0
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(Image, "/droneforge/rgb/image", self.on_rgb, qos); self.create_subscription(Image, "/droneforge/depth/image", self.on_depth, qos)
        self.pub = self.create_publisher(String, "/droneforge/vo/pose", 10); self.get_logger().info(f"visual odometry: SuperPoint + LightGlue on {self.vo.provider}")

    def key(self, m): return (m.header.stamp.sec, m.header.stamp.nanosec)

    def on_rgb(self, m): self.pending.setdefault(self.key(m), {})["rgb"] = m; self.try_run(self.key(m))

    def on_depth(self, m): self.pending.setdefault(self.key(m), {})["depth"] = m; self.try_run(self.key(m))

    def try_run(self, k):
        p = self.pending.get(k)
        if not p or "rgb" not in p or "depth" not in p: return
        del self.pending[k]
        for old in [x for x in self.pending if x < k]: del self.pending[old]
        if time.time() - self.last < self.a.min_period: return
        self.last = time.time(); rgb, dep = p["rgb"], p["depth"]; t0 = time.time()
        img = np.frombuffer(rgb.data, np.uint8).reshape(rgb.height, rgb.step)[:, :rgb.width * 3].reshape(rgb.height, rgb.width, 3); gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        depth = np.frombuffer(dep.data, np.uint16).reshape(dep.height, dep.width); stamp = k[0] + k[1] * 1e-9
        out = {"stamp": stamp}
        if self.vo.anchor is None: n = self.vo.set_anchor(gray, depth, stamp); out.update(anchor_id=self.vo.anchor_id, anchor_stamp=stamp, anchor_set=True, inliers=-1, d_fwd=0.0, d_left=0.0, d_up=0.0, d_yaw=0.0)
        else:
            r, nm = self.vo.track(gray); a = self.vo.anchor
            if r is None or r[4] < self.a.min_inliers or math.hypot(r[0], r[1]) > self.a.max_move or abs(r[3]) > self.a.max_yaw:       # lost or far from the keyframe: new keyframe here
                self.vo.set_anchor(gray, depth, stamp); out.update(anchor_id=self.vo.anchor_id, anchor_stamp=stamp, anchor_set=True, inliers=-1, d_fwd=0.0, d_left=0.0, d_up=0.0, d_yaw=0.0)
            else: out.update(anchor_id=a["id"], anchor_stamp=a["stamp"], anchor_set=False, d_fwd=r[0], d_left=r[1], d_up=r[2], d_yaw=r[3], inliers=r[4], matches=nm)
        out["vo_ms"] = round((time.time() - t0) * 1000); self.pub.publish(String(data=json.dumps(out))); self.n += 1
        if self.n % 10 == 1: self.get_logger().info(f"VO {self.n}: anchor {out['anchor_id']} d=({out['d_fwd']:+.2f},{out['d_left']:+.2f}) m yaw {math.degrees(out['d_yaw']):+.1f} deg inliers {out['inliers']} {out['vo_ms']} ms")


def main():
    p = argparse.ArgumentParser(); p.add_argument("--map", action="store_true", help="started by ../../map.sh (a tag: no effect)"); p.add_argument("--models", default=os.path.join(HERE, "models")); p.add_argument("--cpu", action="store_true"); p.add_argument("--min-inliers", dest="min_inliers", type=int, default=15)
    p.add_argument("--max-move", dest="max_move", type=float, default=1.0); p.add_argument("--max-yaw", dest="max_yaw", type=float, default=0.45); p.add_argument("--min-period", dest="min_period", type=float, default=0.15); a = p.parse_args()
    rclpy.init(); n = VoNode(a)
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass


if __name__ == "__main__": main()
