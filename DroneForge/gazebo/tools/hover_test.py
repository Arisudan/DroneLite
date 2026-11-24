#!/usr/bin/env python3
"""Hover quality test in the SIMULATOR (it arms the simulated drone): takes off, holds 0.6 m, and measures
  vertical hop  : the highest overshoot above the target during take-off/hand-over, and the height peak-to-peak while holding
  horizontal shake : the standard deviation of the true position and of the roll/pitch angles while holding
  drift : how far the TRUE position moved during the measurement window
for  --control fc  (the FC's own position hold) or  --control pc  (the PC holds with the Nimbus-style lateral law on the estimated pose from /odom).
The pass limits are my proposal for a 3-inch drone indoors, not a standard: overshoot <= 5 cm, height p2p <= 4 cm, position std <= 3 cm, drift <= 10 cm, tilt std <= 0.5 deg."""
import argparse, json, math, os, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "..", "..", "software", "pc"))
from nimbus_core.lateral_lqr import LateralLqr, PlantID, deg_to_stick

LIMITS = {"overshoot_m": 0.05, "z_p2p_m": 0.04, "pos_std_m": 0.03, "drift_m": 0.10, "tilt_std_deg": 0.5}


class T(Node):
    def __init__(self, control):
        super().__init__("droneforge_hover_test"); self.control = control; self.pub = self.create_publisher(String, "/droneforge/fc/command", 10)
        self.true = None; self.est = None; self.rows = []; self.zmax_takeoff = 0.0; self.phase = "init"; self.trace = []; self.motors = []; self.t_start = None; self.z_rest = None; self.ctl = []; self.sim_t = None
        self.create_subscription(String, "/droneforge/fc/telemetry", self.on_telem, 10); self.create_subscription(Odometry, "/droneforge/odometry", self.on_true, qos_profile_sensor_data); self.create_subscription(Odometry, "/odom", self.on_est, 10)

    def on_telem(self, m):
        try: d = json.loads(m.data)
        except Exception: return
        if self.phase == "measure" and d.get("m"): self.motors.append(float(np.mean(d["m"])))                  # mean motor output (0..1) while hovering = the throttle this drone needs

    def on_true(self, m):
        q = m.pose.pose.orientation; p = m.pose.pose.position; self.sim_t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)); pitch = math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x))))
        self.true = (p.x, p.y, p.z, math.degrees(roll), math.degrees(pitch))
        if self.z_rest is None: self.z_rest = p.z                       # resting height of the model on the floor: the height hold works above THIS (the range sensor reads the distance to the floor)
        if self.phase in ("takeoff", "settle", "measure"):
            if self.t_start is None: self.t_start = self.now()
            self.trace.append((round(self.now() - self.t_start, 3), round(p.z, 4), self.phase))
        if self.phase in ("takeoff", "settle"): self.zmax_takeoff = max(self.zmax_takeoff, p.z)
        if self.phase == "measure": self.rows.append(self.true)

    def on_est(self, m):
        q = m.pose.pose.orientation; self.est = (m.pose.pose.position.x, m.pose.pose.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)), m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.x, m.twist.twist.angular.y)

    def send(self, cmd=None, pitch=0.0, roll=0.0, lateral=False):
        d = {"pitch": pitch, "roll": roll, "yaw": 0.0, "target_z": 0.6}
        if lateral: d["lateral"] = "pc"
        if cmd: d["cmd"] = cmd
        self.pub.publish(String(data=json.dumps(d)))

    def now(self): return self.sim_t if self.sim_t is not None else time.time()          # SIMULATION time: independent of how fast this machine runs the simulation

    def run_for(self, secs, lqr=None, ref=None):
        t0 = self.now(); last = self.now()
        while self.now() - t0 < secs:
            rclpy.spin_once(self, timeout_sec=0.02); now = self.now(); dt = now - last
            if dt < 0.04: continue
            last = now
            if lqr and self.est:
                x, y, yaw, vf, vl, wr, wp = self.est; vw = (vf * math.cos(yaw) - vl * math.sin(yaw), vf * math.sin(yaw) + vl * math.cos(yaw))
                pd, rd = lqr.step(dt, yaw, (x, y), ref, vw, (0.0, 0.0), (wp, wr)); self.send(pitch=deg_to_stick(pd), roll=deg_to_stick(rd), lateral=True)
                if self.phase == "measure" and self.true: self.ctl.append((self.now(), x - ref[0], y - ref[1], vf, vl, pd, rd, self.true[0], self.true[1], self.true[3], self.true[4], wr, wp))
            else: self.send()


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--control", choices=("fc", "pc"), default="fc"); ap.add_argument("--seconds", type=float, default=40.0); ap.add_argument("--settle", type=float, default=20.0, help="seconds of hold before measuring (the first visual correction moves the drone to the keyframe-based position: that is not drift)"); a = ap.parse_args()
    rclpy.init(); n = T(a.control)
    while n.true is None or (a.control == "pc" and n.est is None): rclpy.spin_once(n, timeout_sec=0.1)
    time.sleep(1); n.send("arm"); n.run_for(1.5); n.phase = "takeoff"; n.send("takeoff"); n.run_for(9); n.send("pos_hold"); n.phase = "settle"
    lq = LateralLqr(PlantID.load(os.path.join(HERE, "..", "config", "plant.json"))) if a.control == "pc" else None; ref = (n.est[0], n.est[1]) if a.control == "pc" else None
    n.run_for(a.settle, lq, ref); n.phase = "measure"; w0 = time.time(); s0 = n.now(); n.run_for(a.seconds, lq, ref); rtf = (n.now() - s0) / max(1e-6, time.time() - w0); n.phase = "done"; n.send("land"); n.run_for(1.0)
    if n.ctl:
        c = np.array(n.ctl); t = c[:, 0] - c[0, 0]; tt = np.arange(0, t[-1], 0.04); sig = np.interp(tt, t, c[:, 5]); sig -= sig.mean(); F = np.abs(np.fft.rfft(sig * np.hanning(len(sig)))); fr = np.fft.rfftfreq(len(sig), 0.04); k = int(np.argmax(F[1:])) + 1
        print(f"PC control: dominant oscillation of the pitch command {fr[k]:.2f} Hz (command std {np.std(c[:, 5]):.2f} deg, estimated-position std {np.std(c[:, 1]):.3f} m, true tilt std {np.std(c[:, 10]):.2f} deg)")
        np.savetxt(os.path.join(HERE, "..", "worlds", "last_hover_pc_trace.csv"), c, delimiter=",", header="t,est_dx,est_dy,est_vf,est_vl,cmd_pitch_deg,cmd_roll_deg,true_x,true_y,true_roll_deg,true_pitch_deg,est_wr,est_wp", comments="")
    r = np.array(n.rows); zr = n.z_rest or 0.0; r[:, 2] -= zr; n.zmax_takeoff -= zr; n.trace = [(t, z - zr, ph) for t, z, ph in n.trace]
    res = {"control": a.control, "overshoot_m": round(float(n.zmax_takeoff - 0.6), 3), "z_p2p_m": round(float(np.ptp(r[:, 2])), 3), "z_mean_m": round(float(r[:, 2].mean()), 3),
           "pos_std_m": round(float(np.std(r[:, :2], axis=0).max()), 3), "drift_m": round(float(math.hypot(*(r[-20:, :2].mean(0) - r[:20, :2].mean(0)))), 3),
           "tilt_std_deg": round(float(max(np.std(r[:, 3]), np.std(r[:, 4]))), 2), "samples": int(len(r)), "measure_s": a.seconds, "settle_s": a.settle, "real_time_factor": round(float(rtf), 2)}
    tr = [(t, z) for t, z, ph in n.trace if ph in ("takeoff", "settle")]; zs = np.array([z for _, z in tr]) if tr else np.array([0.0]); ts = np.array([t for t, _ in tr]) if tr else np.array([0.0])
    res["hover_throttle_mean_motor"] = round(float(np.mean(n.motors)), 3) if n.motors else None; res["bridge_assumes_hover_throttle"] = 0.62
    res["takeoff"] = {"t_reach_0.57": round(float(ts[np.argmax(zs >= 0.57)]), 2) if (zs >= 0.57).any() else None, "t_peak": round(float(ts[np.argmax(zs)]), 2), "z_peak": round(float(zs.max()), 3), "t_settled_within_2cm": None}
    inb = np.nonzero(np.abs(zs - 0.6) > 0.02)[0]; res["takeoff"]["t_settled_within_2cm"] = round(float(ts[inb[-1] + 1]), 2) if len(inb) and inb[-1] + 1 < len(ts) else None
    open(os.path.join(HERE, "..", "worlds", "last_hover_trace.csv"), "w").write("t,z,phase\n" + "\n".join(f"{t},{z},{ph}" for t, z, ph in n.trace))
    res["pass"] = {k: bool(res[k] <= LIMITS[k]) for k in LIMITS}; print(json.dumps(res, indent=1))
    print("RESULT:", "PASS" if all(res["pass"].values()) else "FAIL: " + ", ".join(k for k, v in res["pass"].items() if not v) + f"   (limits {LIMITS})")
    json.dump(res, open(os.path.join(HERE, "..", "worlds", "last_hover_test.json"), "w"), indent=1); n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__": main()
