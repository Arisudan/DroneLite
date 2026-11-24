#!/usr/bin/env python3
"""The drone's state estimate as the real system produces it, published as /odom + TF odom->base_link, plus the fixed camera transforms.

--link ideal: the simulator's exact pose (for comparison).
--link real : simulated SENSORS (attitude with yaw drift, gyro, MTF-02P-style optical flow with noise / scale error / dropouts, range) -> the PC-side Kalman filters
              (nimbus_core/kalman.py: LateralKF = the DfKalman2 structure, VerticalKF) -> the estimate. RTAB-Map and the explorer read the pose only from here.
The true simulator state feeds the sensor models only; it is never published. Origin = the start point."""
import argparse, collections, json, math, os, random, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "..", "..", "software", "pc"))
from nimbus_core.kalman import LateralKF, VerticalKF, Plant, FlowParams


def tf(parent, child, stamp, x, y, z, qx, qy, qz, qw):
    t = TransformStamped(); t.header.stamp = stamp; t.header.frame_id = parent; t.child_frame_id = child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x, y, z
    t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = qx, qy, qz, qw
    return t


def euler(q):
    sr = 2 * (q.w * q.x + q.y * q.z); cr = 1 - 2 * (q.x * q.x + q.y * q.y)
    return math.atan2(sr, cr), math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x)))), math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def load_calib(path):
    """Noise measured on the real drone's logs (tools_scripts/calibrate_kf.py); defaults = my earlier guesses when the file is missing."""
    d = dict(flow_sigma=0.05, range_a=0.01, range_b=0.01, gyro_sigma=0.03, att_sigma=0.01, flow_r0=0.01, q_scale=1.0, att_var=0.0027416, gyro_var=0.043865, real=False)
    try:
        c = json.load(open(path)); d.update(flow_sigma=c["flow_sigma_ms"], range_a=c["range_sigma_a"], range_b=c["range_sigma_b"], gyro_sigma=c["gyro_sigma_rad_s"], att_sigma=max(c["att_sigma_rad"], 0.002), flow_r0=c["flow_r0"], q_scale=c["q_scale"], att_var=c["att_var"], gyro_var=c["gyro_var"], real=True)
    except Exception: pass
    return d


def load_plant(path):
    try: d = json.load(open(path)); return Plant(d.get("k_f", 9.81), d.get("k_r", 9.81), d.get("d_f", 0.4), d.get("d_r", 0.4))
    except Exception: return Plant()


class SimTf(Node):
    def __init__(self, link, seed, plant_path, calib_path="", vo=False):
        super().__init__("droneforge_sim_tf_odom"); self.real = link == "real"; self.rng = np.random.default_rng(seed); self.p0 = None
        self.br = TransformBroadcaster(self); self.sbr = StaticTransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, "/odom", 10)
        self.create_subscription(Odometry, "/droneforge/odometry", self.on_odom, qos_profile_sensor_data)
        st = self.get_clock().now().to_msg()      # camera mount: 6.5 cm forward, 2.5 cm up, LEVEL, centred (optical frame: z forward, x right, y down)
        self.sbr.sendTransform([tf("base_link", "camera_link", st, 0.065, 0.0, 0.025, 0.0, 0.0, 0.0, 1.0), tf("camera_link", "camera_optical", st, 0.0, 0.0, 0.0, -0.5, 0.5, -0.5, 0.5)])
        self.cal = load_calib(calib_path); self.kf = LateralKF(load_plant(plant_path), FlowParams(r0=self.cal["flow_r0"]), self.cal["att_var"], self.cal["gyro_var"], self.cal["q_scale"]); self.vkf = VerticalKF()
        self.get_logger().info(f"sensor noise: {'measured on the real drone logs' if self.cal['real'] else 'default guesses (no kf_calib.json)'}; flow {self.cal['flow_sigma']:.3f} m/s, gyro {math.degrees(self.cal['gyro_sigma']):.2f} deg/s, process-noise scale {self.cal['q_scale']:.2f}")
        self.true = collections.deque(maxlen=200); self.yaw_drift = 0.0; self.last_t = None; self.flow_scale = 1.0 + self.rng.normal(0, 0.02); self.gyro_bias = self.rng.normal(0, 0.01, 2)
        self.use_vo = vo; self.hist_kf = collections.deque(maxlen=600); self.yaw_off = 0.0; self.vo_anchor = None; self.vo_last = -1.0; self.vo_n = 0; self.vo_rejected = 0
        if vo: self.create_subscription(String, "/droneforge/vo/pose", self.on_vo, 10); self.get_logger().info("visual odometry (SuperPoint + LightGlue) feeds the Kalman filter")
        self.n = 0; self.last_flow = -1.0; self.flow_delay = 0.05; self.flow_rate = 25.0

    def hist_at(self, t):
        return min(self.hist_kf, key=lambda h: abs(h[0] - t)) if self.hist_kf else None

    def on_vo(self, m):
        """Visual odometry against a keyframe: position = the filter's position when the keyframe was taken + R(yaw then) * [d_fwd, d_left]. Drift-free while the keyframe is in view."""
        try: o = json.loads(m.data)
        except Exception: return
        if not self.real: return
        if o.get("anchor_set") or self.vo_anchor is None or o["anchor_id"] != self.vo_anchor[0]:
            h = self.hist_at(o["anchor_stamp"])
            if h is not None and o.get("anchor_set"): self.vo_anchor = (o["anchor_id"], h[1], h[2], h[3])                    # filter position and heading at the keyframe
            return
        h_now = self.hist_kf[-1] if self.hist_kf else None
        if h_now is None or o["inliers"] < 15: return
        if h_now[0] - self.vo_last < 0.33: return                                                                                   # at most ~3 fixes per second: more only adds jitter
        self.vo_last = h_now[0]
        _, ax, ay, ayaw = self.vo_anchor; c, s = math.cos(ayaw), math.sin(ayaw)
        px = ax + c * o["d_fwd"] - s * o["d_left"]; py = ay + s * o["d_fwd"] + c * o["d_left"]
        dt = max(0.0, min(1.0, h_now[0] - o["stamp"])); vx, vy = self.kf.vel_world; px += vx * dt; py += vy * dt                    # the picture is a little old: bring the position forward
        dist = math.hypot(o["d_fwd"], o["d_left"]); var = (0.08 + 0.10 * dist) ** 2 + 0.03 ** 2          # the visual position is good to a few cm but noisy (about 5 cm frame to frame): a gentle weight
        if self.kf.update_position(px, py, var): self.vo_n += 1
        else: self.vo_rejected += 1
        hs = self.hist_at(o["stamp"]); target = ayaw + o["d_yaw"]                                                                 # heading fix: pull the drifting yaw toward the visual heading
        if hs is not None: self.yaw_off += float(np.clip(0.05 * ((target - hs[3] + math.pi) % (2 * math.pi) - math.pi), -0.004, 0.004))      # a slow heading pull (a fast one rotates the whole estimate frame and shakes the hold)

    def on_odom(self, m):
        p = m.pose.pose.position; q = m.pose.pose.orientation; t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.p0 is None: self.p0 = (p.x, p.y)
        roll, pitch, yaw = euler(q)
        # body velocity (the odometry twist is in the body frame: x forward, y left), body rates
        vf, vl = m.twist.twist.linear.x, m.twist.twist.linear.y; wx, wy = m.twist.twist.angular.x, m.twist.twist.angular.y
        self.true.append((t, vf, vl, p.z, roll, pitch)); stamp = m.header.stamp
        if not self.real:
            x, y, z, yw, ef, el, rr, rp = p.x - self.p0[0], p.y - self.p0[1], p.z, yaw, vf, vl, wx, wy
        else:
            r = self.rng; dt = 0.0 if self.last_t is None else max(0.0, t - self.last_t); self.last_t = t
            self.yaw_drift += r.normal(0, 0.003 * math.sqrt(dt))                                   # gyro-integrated yaw drifts (no magnetometer)
            yaw_m = yaw + self.yaw_drift + self.yaw_off; flying = p.z > 0.15
            self.kf.predict(t, yaw_m, flying); self.vkf.predict(t)
            self.kf.update_attitude(roll + r.normal(0, self.cal["att_sigma"]), pitch + r.normal(0, self.cal["att_sigma"]))
            self.kf.update_gyro(wx + self.gyro_bias[0] + r.normal(0, self.cal["gyro_sigma"]), wy + self.gyro_bias[1] + r.normal(0, self.cal["gyro_sigma"]))
            if t - self.last_flow >= 1.0 / self.flow_rate:                                          # MTF-02P packet: flow velocity + range, ~25 Hz, ~50 ms old
                self.last_flow = t; s = min(self.true, key=lambda h: abs(h[0] - (t - self.flow_delay))); _, svf, svl, sz, sroll, spitch = s
                speed = math.hypot(svf, svl); quality = 255 if (sz > 0.12 and speed < 2.0 and r.random() > 0.01) else 0
                rng_m = max(0.0, sz / max(0.2, math.cos(sroll) * math.cos(spitch)) + r.normal(0, self.cal["range_a"] + self.cal["range_b"] * sz))
                self.kf.update_flow(self.flow_scale * svf + r.normal(0, self.cal["flow_sigma"]), self.flow_scale * (-svl) + r.normal(0, self.cal["flow_sigma"]), rng_m, quality)       # flow right = -left
                if quality: self.vkf.update_range(rng_m, sroll, spitch)
            x, y = self.kf.pos; yw = yaw_m; self.hist_kf.append((t, x, y, yw)); ef, el = self.kf.vel_body(); rr, rp = self.kf.rates(); z = float(self.vkf.x[0]) if self.vkf.x[0] > 0 else p.z
        qz, qw = math.sin(yw / 2), math.cos(yw / 2)
        self.br.sendTransform(tf("odom", "base_link", stamp, x, y, z, 0.0, 0.0, qz, qw))
        o = Odometry(); o.header.stamp = stamp; o.header.frame_id = "odom"; o.child_frame_id = "base_link"
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = x, y, z; o.pose.pose.orientation.z, o.pose.pose.orientation.w = qz, qw
        o.twist.twist.linear.x = float(ef); o.twist.twist.linear.y = float(el); o.twist.twist.angular.x = float(rr); o.twist.twist.angular.y = float(rp); self.pub.publish(o)
        self.n += 1
        if self.real and self.n % 500 == 0: self.get_logger().info(f"KF: pos ({x:.2f},{y:.2f}) true ({p.x - self.p0[0]:.2f},{p.y - self.p0[1]:.2f}) err {math.hypot(x - (p.x - self.p0[0]), y - (p.y - self.p0[1])):.2f} m, flow accepted {self.kf.accepted} rejected {self.kf.rejected}" + (f", visual fixes {self.vo_n} (rejected {self.vo_rejected})" if self.use_vo else ""))


def main():
    a = argparse.ArgumentParser(); a.add_argument("--link", choices=("ideal", "real"), default="ideal"); a.add_argument("--seed", type=int, default=1)
    a.add_argument("--plant", default=os.path.join(HERE, "..", "config", "plant.json")); a.add_argument("--calib", default=os.path.join(HERE, "..", "config", "kf_calib.json")); a.add_argument("--vo", action="store_true", help="fuse the SuperPoint+LightGlue visual odometry"); a = a.parse_args()
    rclpy.init(); n = SimTf(a.link, a.seed, a.plant, a.calib, a.vo)
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass


if __name__ == "__main__": main()
