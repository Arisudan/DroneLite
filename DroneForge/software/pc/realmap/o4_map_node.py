#!/usr/bin/env python3
"""DroneForge mapping from the REAL drone's data (also runs on the simulator's O4 feed + sim FC through the same GCS paths): the RTAB-Map input that the sim
used to get from the simulator itself.

  GCS shared memory "droneforge_o4_frames" (the decoded O4 video, nimbus_gcs/map_feed.py)
     -> [--rectify: O4 lens -> the 640x480 Nimbus-profile pinhole, config/o4_camera.yaml] -> Depth Anything 3 metric depth, scaled on the floor with the FC range + attitude
     -> /droneforge/rgb/image, /droneforge/depth/image (16UC1 mm), /droneforge/rgb/camera_info   (RTAB-Map, visual_odom.py)
     -> /droneforge/perception/obstacles (Nimbus 6x8 grid, the explorer)
  GCS UDP 127.0.0.1:5767 (FC telemetry: range, flow speeds, attitude, heading)
     -> the PC-side Kalman filters (nimbus_core.kalman: LateralKF on flow + attitude, VerticalKF on range) + the SuperPoint/LightGlue key-frame fixes (/droneforge/vo/pose)
     -> /odom, TF odom -> base_link, base_link -> camera_link -> camera_optical
Time: wall clock everywhere (the frame's capture time from the GCS). Conventions of the FC telemetry: yaw clockwise (FRD) -> ROS counter-clockwise; pitch + = nose down;
flow speed forward/right -> body forward/left. The FC sign conventions are UNVERIFIED on the real drone (bench check before flight)."""
import argparse, json, math, os, socket, struct, sys, threading, time
import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__)); DF = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(DF, "gazebo", "vision")); sys.path.insert(0, os.path.join(DF, "software", "pc"))
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from nimbus_depth_core import Da3, FloorScale, result_json
from nimbus_core.kalman import FlowParams, LateralKF, Plant, VerticalKF

SHM_NAME = "droneforge_o4_frames"; HDR = 64; MAX_W, MAX_H = 1920, 1088; READER_HB = 32           # header byte 32: f64 wall time of the last read (map_feed.FrameShare)
W, H, F = 640, 480, 292.2957                                       # the Nimbus-profile picture RTAB-Map and the VO assume (the sim camera is exactly this)


def stamp_of(t: float) -> TimeMsg:
    m = TimeMsg(); m.sec = int(t); m.nanosec = int((t - int(t)) * 1e9); return m


def tf(parent, child, stamp, x, y, z, qx, qy, qz, qw):
    t = TransformStamped(); t.header.stamp = stamp; t.header.frame_id = parent; t.child_frame_id = child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x, y, z
    t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = qx, qy, qz, qw
    return t


class FrameReader:
    """Reads the newest complete frame from the GCS shared memory (sequence lock: odd = being written)."""
    def __init__(self):
        from multiprocessing import shared_memory, resource_tracker
        self.shm = shared_memory.SharedMemory(name=SHM_NAME)
        try: resource_tracker.unregister(self.shm._name, "shared_memory")           # the GCS owns the segment: never unlink it from here
        except Exception: pass
        self.last = 0; self.t_new = time.time()

    def stale(self) -> bool:
        """No new frame for 3 s: the GCS may have been restarted (it makes a NEW segment): reopen."""
        return time.time() - self.t_new > 3.0

    def read(self):
        b = self.shm.buf
        struct.pack_into("<d", b, READER_HB, time.time())                       # tells the GCS a reader is here: it copies frames only while this is fresh (map_feed.FrameShare)
        for _ in range(3):
            s1 = struct.unpack_from("<Q", b, 0)[0]
            if s1 % 2 or s1 == self.last: return None
            h, w, t = struct.unpack_from("<IId", b, 8)
            if not (0 < h <= MAX_H and 0 < w <= MAX_W): return None
            f = np.ndarray((h, w, 3), np.uint8, buffer=b, offset=HDR).copy()
            if struct.unpack_from("<Q", b, 0)[0] == s1: self.last = s1; self.t_new = time.time(); return f, t
        return None


class O4Map(Node):
    def __init__(self, a):
        super().__init__("droneforge_o4_map"); self.a = a
        self.pub_rgb = self.create_publisher(Image, "/droneforge/rgb/image", 5); self.pub_depth = self.create_publisher(Image, "/droneforge/depth/image", 5)
        self.pub_info = self.create_publisher(CameraInfo, "/droneforge/rgb/camera_info", 5); self.pub_obs = self.create_publisher(String, "/droneforge/perception/obstacles", 10)
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10); self.br = TransformBroadcaster(self); self.sbr = StaticTransformBroadcaster(self)
        st = stamp_of(time.time())                                  # camera mount: 6.5 cm forward, 2.5 cm up, LEVEL (optical frame: z forward, x right, y down)
        self.sbr.sendTransform([tf("base_link", "camera_link", st, 0.065, 0.0, 0.025, 0.0, 0.0, 0.0, 1.0), tf("camera_link", "camera_optical", st, 0.0, 0.0, 0.0, -0.5, 0.5, -0.5, 0.5)])
        self.da3 = Da3(4); self.fscale = FloorScale(); self.rect = None; self.reader = None
        cal = self._calib(); self.kf = LateralKF(self._plant(), FlowParams(r0=cal.get("flow_r0", 0.01)), cal.get("att_var", 0.0027416), cal.get("gyro_var", 0.043865), cal.get("q_scale", 1.0))
        self.vkf = VerticalKF(); self.tel = None; self.hist = []; self.yaw0 = None; self.lock = threading.Lock(); self.n_tel = 0; self.n_img = 0
        self.vo_anchor = None; self.vo_last = -1.0; self.vo_n = 0; self.vo_rej = 0
        self.create_subscription(String, "/droneforge/vo/pose", self.on_vo, 10)
        threading.Thread(target=self._telemetry, daemon=True).start()
        self.create_timer(0.01, self._frames); self.create_timer(5.0, self._say)
        self.get_logger().info(f"O4 map node: frames from shared memory '{SHM_NAME}', FC telemetry on udp {a.telem_port}, DA3 on {self.da3.s.get_providers()[0]}" + (", rectifying" if a.rectify else ""))

    # ---- calibration files (the same as the sim estimator)
    def _calib(self):
        try: c = json.load(open(os.path.join(DF, "gazebo", "config", "kf_calib.json"))); return dict(flow_r0=c["flow_r0"], att_var=c["att_var"], gyro_var=c["gyro_var"], q_scale=c["q_scale"])
        except Exception: return {}

    def _plant(self):
        try: d = json.load(open(os.path.join(DF, "gazebo", "config", "plant.json"))); return Plant(d.get("k_f", 9.81), d.get("k_r", 9.81), d.get("d_f", 0.4), d.get("d_r", 0.4))
        except Exception: return Plant()

    # ---- FC telemetry -> Kalman filters -> /odom + TF
    def _telemetry(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.bind(("127.0.0.1", self.a.telem_port))
        while rclpy.ok():
            try: p = json.loads(s.recv(4096).decode())
            except Exception: continue
            with self.lock: self._fuse(p)

    def _fuse(self, p):
        t = time.time(); yaw = -float(p.get("yaw_cw", 0.0))                     # FC heading clockwise -> ROS counter-clockwise
        if self.yaw0 is None: self.yaw0 = yaw
        yaw -= self.yaw0                                                         # the odom frame starts along the first heading
        rng = float(p.get("range", 0.0)); roll, pitch = float(p.get("roll", 0.0)), float(p.get("pitch", 0.0)); q = int(p.get("q", 0))
        flying = rng > 0.15
        self.kf.predict(t, yaw, flying); self.vkf.predict(t)
        self.kf.update_attitude(roll, pitch)
        if rng > 0.02: self.kf.update_flow(float(p.get("flow_f", 0.0)), float(p.get("flow_r", 0.0)), rng, 255 if q > 0 else 0)
        if rng > 0.02 and q > 0: self.vkf.update_range(rng, roll, pitch)
        x, y = self.kf.pos; z = float(self.vkf.x[0]) if self.vkf.x[0] > 0 else rng
        self.tel = dict(t=t, x=x, y=y, z=z, yaw=yaw, roll=roll, pitch=pitch, range=rng); self.hist.append((t, x, y, yaw)); self.hist = self.hist[-600:]; self.n_tel += 1
        ef, el = self.kf.vel_body(); st = stamp_of(t); qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
        self.br.sendTransform(tf("odom", "base_link", st, x, y, z, 0.0, 0.0, qz, qw))
        o = Odometry(); o.header.stamp = st; o.header.frame_id = "odom"; o.child_frame_id = "base_link"
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = x, y, z; o.pose.pose.orientation.z, o.pose.pose.orientation.w = qz, qw
        o.twist.twist.linear.x = float(ef); o.twist.twist.linear.y = float(el); self.pub_odom.publish(o)

    def on_vo(self, m):
        """The SuperPoint/LightGlue key-frame fix, as the sim estimator (vision/sim_tf_odom.py) uses it."""
        try: o = json.loads(m.data)
        except Exception: return
        with self.lock:
            if not self.hist: return
            at = lambda ts: min(self.hist, key=lambda h: abs(h[0] - ts))
            if o.get("anchor_set") or self.vo_anchor is None or o["anchor_id"] != self.vo_anchor[0]:
                if o.get("anchor_set"): h = at(o["anchor_stamp"]); self.vo_anchor = (o["anchor_id"], h[1], h[2], h[3])
                return
            now = self.hist[-1]
            if o["inliers"] < 15 or now[0] - self.vo_last < 0.33: return
            self.vo_last = now[0]; _, ax, ay, ayaw = self.vo_anchor; c, s = math.cos(ayaw), math.sin(ayaw)
            px = ax + c * o["d_fwd"] - s * o["d_left"]; py = ay + s * o["d_fwd"] + c * o["d_left"]
            dt = max(0.0, min(1.0, now[0] - o["stamp"])); vx, vy = self.kf.vel_world; px += vx * dt; py += vy * dt
            var = (0.08 + 0.10 * math.hypot(o["d_fwd"], o["d_left"])) ** 2 + 0.03 ** 2
            if self.kf.update_position(px, py, var): self.vo_n += 1
            else: self.vo_rej += 1

    # ---- frames -> DA3 -> RGB-D + obstacles
    def _frames(self):
        if self.reader is None:
            try: self.reader = FrameReader()
            except FileNotFoundError: return                                     # the GCS (and its video) is not up yet
        r = self.reader.read()
        if r is None:
            if self.reader.stale(): self.reader = None
            return
        bgr, t_cap = r; t0 = time.time()
        if self.a.rectify:
            if self.rect is None:
                import yaml; from nimbus_core.rectify import Rectifier
                cfg = yaml.safe_load(open(os.path.join(DF, "gazebo", "config", "o4_camera.yaml"))); cal = cfg.get("calibration"); v = cfg["variants"][cfg["variant"]]; h0, w0 = bgr.shape[:2]
                K = None if not cal else [[cal["fx"], 0, cal["cx"]], [0, cal["fy"], cal["cy"]], [0, 0, 1]]
                if cal and cal.get("size") and tuple(cal["size"]) != (w0, h0): sc = w0 / cal["size"][0]; K = [[K[0][0] * sc, 0, K[0][2] * sc], [0, K[1][1] * sc, K[1][2] * sc], [0, 0, 1]]
                lens = {"fisheye": "equidistant", "pinhole": "rectilinear"}.get((cal or {}).get("model"), v.get("lens", "rectilinear"))     # a calibration says which model it was solved with
                self.rect = Rectifier(w0, h0, K, None if not cal else cal.get("dist"), v["diagonal_fov_deg"], lens=lens)
                self.get_logger().info(f"rectify: {'CALIBRATED lens (' + cal.get('model', '?') + ', ' + str(cal.get('date', '')) + ')' if cal else 'NOMINAL lens (no calibration yet: Video page, Calibrate lens)'}, {w0}x{h0}")
            bgr = self.rect(bgr)
        if bgr.shape[:2] != (H, W): bgr = cv2.resize(bgr, (W, H), interpolation=cv2.INTER_AREA)
        rgb = bgr[..., ::-1].copy(); d = self.da3(rgb)
        with self.lock: tel = dict(self.tel) if self.tel else None
        if tel is not None: d = d * self.fscale.update(d, tel["z"] + 0.025, tel["pitch"], tel["roll"])
        st = stamp_of(t_cap)
        mm = np.clip(cv2.resize(d, (W, H), interpolation=cv2.INTER_LINEAR) * 1000.0, 0, 65535).astype(np.uint16)
        dm = Image(); dm.header.stamp = st; dm.header.frame_id = "camera_optical"; dm.height, dm.width, dm.encoding, dm.is_bigendian, dm.step, dm.data = H, W, "16UC1", 0, 2 * W, mm.tobytes()
        rm = Image(); rm.header = dm.header; rm.height, rm.width, rm.encoding, rm.is_bigendian, rm.step, rm.data = H, W, "rgb8", 0, 3 * W, rgb.tobytes()
        ci = CameraInfo(); ci.header = dm.header; ci.width, ci.height = W, H; ci.k = [F, 0.0, W / 2.0, 0.0, F, H / 2.0, 0.0, 0.0, 1.0]
        ci.p = [F, 0.0, W / 2.0, 0.0, 0.0, F, H / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0]; ci.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]; ci.distortion_model = "plumb_bob"; ci.d = [0.0] * 5
        self.pub_rgb.publish(rm); self.pub_depth.publish(dm); self.pub_info.publish(ci)
        out = result_json(d, time.time() - t0)[0]                               # (message, ...) as in vision/nimbus_perception.py
        out["stamp"] = time.time(); self.pub_obs.publish(String(data=json.dumps(out))); self.n_img += 1

    def _say(self):
        with self.lock: tel = self.tel
        self.get_logger().info(f"frames {self.n_img}, FC telemetry {self.n_tel}, floor scale {self.fscale.s:.3f}, VO fixes {self.vo_n} (rejected {self.vo_rej})"
                               + (f", pose ({tel['x']:.2f}, {tel['y']:.2f}, {tel['z']:.2f}) m" if tel else ", no FC telemetry yet"))


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--rectify", action="store_true", help="real O4: undistort to the Nimbus pinhole profile (config/o4_camera.yaml)")
    ap.add_argument("--telem-port", dest="telem_port", type=int, default=5767); a = ap.parse_args()
    rclpy.init(); n = O4Map(a)
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass


if __name__ == "__main__":
    main()
