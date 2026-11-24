#!/usr/bin/env python3
"""Nimbus perception for the Gazebo house: the same two models Nimbus ships, run on the simulated camera.

* Depth Anything 3 metric (da3_metric.onnx + .weights.safetensors, ONNX Runtime). Input [1,3,336,504] RGB, ImageNet mean/std, plain resize,
  output metres, no extra scale (as Nimbus does it). Frame -> 6x8 occupancy grid with per-cell min / mean / max / count (Nimbus
  DepthOccupancyLoop) and the Nimbus obstacle test (valid 0.2-1.0 m, more than 19 samples).
* YOLOv12n-face (optional, --faces): same preprocessing as Nimbus' person loop; only reported, it never steers here.

Publishes JSON on /droneforge/perception/obstacles (rows, cols, min_m, mean_m, max_m, p10_m, count, nearest_m, obstacle, latency_ms, fps).
No cv_bridge (broken against the installed OpenCV): the sensor_msgs/Image is decoded with numpy.
"""
import argparse, collections, json, math, os, sys, time
import numpy as np
import cv2
import onnxruntime as ort
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String

import nimbus_depth_core as core
from nimbus_depth_core import Da3, FloorScale, grid_stats, result_json, ROWS, COLS
decode = core.decode_msg
MODELS = core.MODELS


class Face:
    def __init__(self):
        self.s = ort.InferenceSession(os.path.join(MODELS, "yolov12n-face.onnx"), providers=["CPUExecutionProvider"]); self.inp = self.s.get_inputs()[0].name

    def __call__(self, rgb, thr=0.4):
        h, w = rgb.shape[:2]; sc = 640 / max(h, w); nw, nh = int(round(w * sc)), int(round(h * sc))
        can = np.full((640, 640, 3), 114, np.uint8); can[(640 - nh) // 2:(640 - nh) // 2 + nh, (640 - nw) // 2:(640 - nw) // 2 + nw] = cv2.resize(rgb, (nw, nh))
        o = self.s.run(None, {self.inp: (can.astype(np.float32) / 255.0).transpose(2, 0, 1)[None]})[0].reshape(-1, 6)
        return int((o[:, 4] > thr).sum())


class Perception(Node):
    def __init__(self, a):
        super().__init__("nimbus_perception"); self.a = a
        self.da3 = None if a.server else Da3(a.threads); self.face = Face() if a.faces else None
        self.pub = self.create_publisher(String, "/droneforge/perception/obstacles", 10)
        if a.ros_depth:                                              # RGB-D for RTAB-Map: rgb + dense depth with the SAME header stamp, plus camera info
            self.pub_rgb = self.create_publisher(Image, "/droneforge/rgb/image", 5); self.pub_depth = self.create_publisher(Image, "/droneforge/depth/image", 5)
            self.pub_info = self.create_publisher(CameraInfo, "/droneforge/rgb/camera_info", 5)
        self.latest = None; self.n = 0; self.t_hist = []; self.rect = None; self.inflight = 0; self.done = collections.deque(); self.last_pub_stamp = -1.0
        self.fscale = FloorScale(); self.est = None                                   # 2026-11-21: floor-based per-frame DA3 scale from the drone's height estimate (/odom: range-based, as the FC's)
        if not a.no_floor_scale:
            from nav_msgs.msg import Odometry
            self.create_subscription(Odometry, "/odom", self.on_est, 10)
            self.create_subscription(String, "/droneforge/fc/telemetry", self.on_fc, 10)      # attitude as the FC reports it (/odom carries yaw only)
        self.att = (0.0, 0.0)
        self.create_subscription(Image, a.topic, self.on_img, QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1))
        self.get_logger().info(f"depth server {a.server}" if a.server else f"DA3 ready (input {self.da3.inp}, output {self.da3.out}), threads {a.threads}")
        if a.dump: os.makedirs(a.dump, exist_ok=True)

    def on_img(self, m): self.latest = m

    def on_est(self, m):
        import math
        self.est = (m.pose.pose.position.z, self.att[0], self.att[1])

    def on_fc(self, m):
        import math
        try: t = json.loads(m.data); self.att = (math.radians(float(t.get("pitch", 0.0))), math.radians(float(t.get("roll", 0.0))))   # pitch + = nose down (the sim FC's convention)
        except Exception: pass

    def publish_rgbd(self, m, out, d):
        """RGB + metric depth (16UC1 millimetres, resized to the camera size) with the camera frame's stamp, so RTAB-Map can look up the pose at the moment the picture was taken."""
        import base64
        if d is not None: depth = d
        elif out.get("depth_png"): depth = cv2.imdecode(np.frombuffer(base64.b64decode(out["depth_png"]), np.uint8), cv2.IMREAD_UNCHANGED).astype(np.float32) / 1000.0
        else: return
        mm = np.clip(cv2.resize(depth, (m.width, m.height), interpolation=cv2.INTER_LINEAR) * 1000.0, 0, 65535).astype(np.uint16)
        dm = Image(); dm.header = m.header; dm.header.frame_id = "camera_optical"; dm.height, dm.width, dm.encoding, dm.is_bigendian, dm.step = m.height, m.width, "16UC1", 0, 2 * m.width; dm.data = mm.tobytes()
        rm = Image(); rm.header = dm.header; rm.height, rm.width, rm.encoding, rm.is_bigendian, rm.step, rm.data = m.height, m.width, m.encoding, m.is_bigendian, m.step, m.data
        ci = CameraInfo(); ci.header = dm.header; ci.width, ci.height = m.width, m.height; f = 292.2957 * m.width / 640.0   # the sim camera has the Nimbus Air75 field of view (fx 292.3 at 640 px), square pixels
        ci.k = [f, 0.0, m.width / 2.0, 0.0, f, m.height / 2.0, 0.0, 0.0, 1.0]; ci.p = [f, 0.0, m.width / 2.0, 0.0, 0.0, f, m.height / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0]; ci.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]; ci.distortion_model = "plumb_bob"; ci.d = [0.0] * 5
        self.pub_rgb.publish(rm); self.pub_depth.publish(dm); self.pub_info.publish(ci)

    def fetch(self, m, rgb, body, t0):
        import urllib.request
        try: out = json.loads(urllib.request.urlopen(urllib.request.Request(self.a.server.rstrip("/") + ("/depth?depth=1" if self.a.ros_depth else "/depth"), body), timeout=self.a.timeout).read())
        except Exception as e: self.get_logger().warn(f"depth server: {e}"); self.inflight -= 1; return
        self.inflight -= 1; self.done.append((m, rgb, out, t0, time.time()))

    def finish(self, m, rgb, out, t0, t1):
        """Runs on the main thread: publishes one answer. An answer older than the newest one already published is dropped (out-of-order delivery)."""
        t_cap = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if t_cap <= self.last_pub_stamp: return
        self.last_pub_stamp = t_cap; dt = t1 - t0
        out["server_ms"] = out.get("latency_ms"); out["latency_ms"] = round(dt * 1000); out["stamp"] = time.time()
        p10 = np.array(out["p10_m"]).reshape(ROWS, COLS); nearest = out["nearest_m"]; d = None
        self.t_hist = (self.t_hist + [dt])[-20:]; out["fps"] = round(1.0 / max(1e-3, float(np.mean(self.t_hist))), 2)
        if self.a.ros_depth: self.publish_rgbd(m, out, d)
        out["feed_ms"] = self.a.feed_latency_ms
        out.pop("depth_png", None); self.pub.publish(String(data=json.dumps(out))); self.n += 1
        if self.a.show: cv2.imshow("DroneForge | Nimbus view (camera | live floor plan)", np.hstack([rgb[..., ::-1], cv2.resize(core.render_floorplan_cv(out["floorplan"]), (rgb.shape[0], rgb.shape[0]))])); cv2.waitKey(1)
        if self.n % 5 == 1: self.get_logger().info(f"frame {self.n}: {dt * 1000:.0f} ms, nearest valid {nearest}, centre p10 {p10[2:4, 3:5].min():.2f}, in flight {self.inflight}")

    def process(self):
        while self.done: self.finish(*self.done.popleft())
        m = self.latest
        if m is None: return False
        self.latest = None; t0 = time.time(); rgb = decode(m)
        if self.a.rectify:                                                    # real O4 feed: undistort + crop to the pinhole profile DA3 expects (the sim camera already has it)
            if self.rect is None:
                import yaml; sys.path.insert(0, os.path.join(HERE, "..", "..", "software", "pc")); from nimbus_core.rectify import Rectifier
                cfg = yaml.safe_load(open(os.path.join(HERE, "..", "config", "o4_camera.yaml"))); cal = cfg.get("calibration")
                K = None if not cal else [[cal["fx"], 0, cal["cx"]], [0, cal["fy"], cal["cy"]], [0, 0, 1]]
                if cal and cal.get("size") and tuple(cal["size"]) != (m.width, m.height): sc = m.width / cal["size"][0]; K = [[cal["fx"] * sc, 0, cal["cx"] * sc], [0, cal["fy"] * sc, cal["cy"] * sc], [0, 0, 1]]      # calibrated at another resolution: scale the intrinsics
                v = cfg["variants"][cfg["variant"]]; self.rect = Rectifier(m.width, m.height, K, None if not cal else cal.get("dist"), v["diagonal_fov_deg"], lens=v.get("lens", "rectilinear")); self.get_logger().info(f"rectifier: O4 {cfg['variant']}, calibration {'yes' if cal else 'NO (nominal lens assumed)'}")
            rgb = self.rect(rgb)
        if self.a.server:
            # remote depth: several requests may be in flight (the wifi to the GPU machine is bursty: one stalled request must not freeze the map). Workers post their answers to self.done.
            import threading
            ok, jpg = cv2.imencode(".jpg", rgb[..., ::-1], [cv2.IMWRITE_JPEG_QUALITY, 85])
            if self.inflight >= self.a.max_inflight: return False                                   # all workers busy: drop this frame (a newer one is coming)
            self.inflight += 1; threading.Thread(target=self.fetch, args=(m, rgb, jpg.tobytes(), t0), daemon=True).start(); return True
        else:
            d = self.da3(rgb); dt = time.time() - t0
            if self.est is not None and not self.a.no_floor_scale:                       # metric scale from the floor: camera 0.025 m above the body centre, level mount
                d = d * self.fscale.update(d, self.est[0] + 0.025, self.est[1], self.est[2])
            out, _, p10, nearest = result_json(d, dt, self.face(rgb) if self.face else None); out["stamp"] = time.time()
        self.t_hist = (self.t_hist + [dt])[-20:]; out["fps"] = round(1.0 / max(1e-3, float(np.mean(self.t_hist))), 2)
        if self.a.ros_depth: self.publish_rgbd(m, out, d)
        out["feed_ms"] = self.a.feed_latency_ms                      # calibrated latency of the O4 feed before the picture reached us (the depth latency is measured separately)
        out.pop("depth_png", None); self.pub.publish(String(data=json.dumps(out))); self.n += 1
        if d is not None and ((self.a.dump and self.n % self.a.dump_every == 0) or self.a.show):
            dn = np.clip(d / 6.0, 0, 1); col = cv2.resize(cv2.applyColorMap((255 * (1 - dn)).astype(np.uint8), cv2.COLORMAP_TURBO), (rgb.shape[1], rgb.shape[0]))
            for r in range(ROWS):                      # the 6x8 grid with each cell's nearest depth, the way Nimbus draws its occupancy
                for c in range(COLS):
                    x0, y0 = c * rgb.shape[1] // COLS, r * rgb.shape[0] // ROWS
                    cv2.rectangle(col, (x0, y0), (x0 + rgb.shape[1] // COLS, y0 + rgb.shape[0] // ROWS), (255, 255, 255), 1)
                    if np.isfinite(p10[r, c]): cv2.putText(col, f"{p10[r, c]:.1f}", (x0 + 6, y0 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            both = np.hstack([rgb[..., ::-1], col]); cv2.putText(both, f"DA3 depth (m, cell 10th percentile)  {dt * 1000:.0f} ms  nearest {nearest}", (10, both.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
            if self.a.dump and self.n % self.a.dump_every == 0: cv2.imwrite(os.path.join(self.a.dump, f"view_{self.n:04d}.jpg"), both)
            if self.a.show: cv2.imshow("DroneForge | Nimbus depth", both); cv2.waitKey(1)
        if self.n % 5 == 1: self.get_logger().info(f"frame {self.n}: {dt * 1000:.0f} ms, nearest valid {nearest}, centre p10 {p10[2:4, 3:5].min():.2f}, floor scale {self.fscale.s:.3f} (used {self.fscale.used}), est z {None if self.est is None else round(self.est[0], 3)}, pitch {math.degrees(self.att[0]):+.1f}")
        return True


def main():
    p = argparse.ArgumentParser(); p.add_argument("--threads", type=int, default=6); p.add_argument("--topic", default="/droneforge/camera/image_raw", help="camera topic (use /droneforge/camera/image_delayed for the latency-emulated O4 feed)"); p.add_argument("--feed-latency-ms", dest="feed_latency_ms", type=float, default=0.0); p.add_argument("--max-inflight", dest="max_inflight", type=int, default=3, help="parallel depth requests"); p.add_argument("--timeout", type=float, default=4.0, help="seconds before a depth request is given up"); p.add_argument("--faces", action="store_true"); p.add_argument("--rectify", action="store_true", help="rectify the real O4 picture to the Nimbus pinhole profile (config/o4_camera.yaml)"); p.add_argument("--ros-depth", dest="ros_depth", action="store_true", help="also publish rgb + dense depth + camera info for RTAB-Map"); p.add_argument("--server", default=None, help="URL of depth_server.py on the GPU PC, e.g. http://172.16.100.188:8765"); p.add_argument("--show", action="store_true", help="live window: camera + depth + 6x8 grid")
    p.add_argument("--no-floor-scale", dest="no_floor_scale", action="store_true", help="keep DA3's raw metric scale (default: per-frame floor scale from the height estimate)")
    p.add_argument("--dump", default=None, help="folder for rgb+depth preview jpgs"); p.add_argument("--dump-every", dest="dump_every", type=int, default=5)
    a = p.parse_args(); rclpy.init(); n = Perception(a)
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.02)
        n.process()


if __name__ == "__main__": main()
