#!/usr/bin/env python3
"""System identification of the drone's tilt-to-motion plant in the simulator (SIMULATION ONLY: it arms the simulated drone).
Flies: arm, take off, hold at 0.6 m, then PC-lateral tilt steps through the Nimbus angle curve (+-3 deg and +-5 deg, forward and sideways, 2 s each, settle in between).
Fits, per axis:  dv/dt = K * tilt_rad - D * v   (K m/s^2 per rad, D 1/s), the attitude lag tau (first order) and the command-to-attitude dead time.
Writes config/plant.json, which the Kalman filter (sim_tf_odom.py) and the lateral controller (nimbus_core/lateral_lqr.py) read."""
import json, math, os, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "software", "pc"))
from nimbus_core.lateral_lqr import deg_to_stick

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "plant.json")


class Sysid(Node):
    def __init__(self):
        super().__init__("droneforge_sysid"); self.pub = self.create_publisher(String, "/droneforge/fc/command", 10); self.rows = []; self.cmd = (0.0, 0.0); self.z = 0.0
        self.create_subscription(Odometry, "/droneforge/odometry", self.on_odom, qos_profile_sensor_data)

    def on_odom(self, m):
        q = m.pose.pose.orientation; t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9; self.z = m.pose.pose.position.z
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)); pitch = math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x))))
        self.rows.append((t, m.twist.twist.linear.x, m.twist.twist.linear.y, pitch, roll, self.cmd[0], self.cmd[1]))

    def send(self, cmd=None, pitch_deg=0.0, roll_deg=0.0, lateral=True):
        self.cmd = (pitch_deg, roll_deg); d = {"pitch": deg_to_stick(pitch_deg), "roll": deg_to_stick(roll_deg), "yaw": 0.0, "target_z": 0.6}
        if lateral: d["lateral"] = "pc"
        if cmd: d["cmd"] = cmd
        self.pub.publish(String(data=json.dumps(d)))

    def spin_for(self, secs, **kw):
        t0 = time.time()
        while time.time() - t0 < secs: self.send(**kw); rclpy.spin_once(self, timeout_sec=0.02)


def fit(rows, lane):
    """lane 0 = forward (vx, pitch, forward-positive pitch cmd); lane 1 = right (-vy, roll, roll cmd)."""
    a = np.array(rows); t = a[:, 0]; v = a[:, 1] if lane == 0 else -a[:, 2]; th = a[:, 3] if lane == 0 else a[:, 4]; cmd = a[:, 5] if lane == 0 else a[:, 6]
    keep = np.diff(t) > 1e-4; dt = np.diff(t)[keep]
    # smooth, then dv/dt = K th - D v
    k = 5; ker = np.ones(k) / k; vs = np.convolve(v, ker, mode="same"); ths = np.convolve(th, ker, mode="same")
    acc = (np.diff(vs)[keep]) / dt; X = np.column_stack([ths[:-1][keep], -vs[:-1][keep]]); ok = np.abs(ths[:-1][keep]) < 0.4
    (K, D), *_ = np.linalg.lstsq(X[ok], acc[ok], rcond=None)
    # attitude lag + dead time: simulate th(t) = first order lag of cmd delayed by d, grid-search tau, d
    tr = np.radians(cmd); best = (1e9, 0.08, 0.0); tt = t - t[0]
    for d in np.arange(0.0, 0.4, 0.02):
        for tau in np.arange(0.02, 0.4, 0.02):
            y = np.zeros_like(th); 
            for i in range(1, len(tt)):
                j = np.searchsorted(tt, tt[i] - d) - 1; u = tr[max(j, 0)]; y[i] = y[i - 1] + (u - y[i - 1]) * (tt[i] - tt[i - 1]) / tau
            e = float(np.mean((y - th) ** 2))
            if e < best[0]: best = (e, tau, d)
    return float(K), float(max(D, 0.0)), best[1], best[2]


def main():
    rclpy.init(); n = Sysid()
    while not n.rows: rclpy.spin_once(n, timeout_sec=0.1)
    time.sleep(1); n.send("arm", lateral=False); n.spin_for(1.5, lateral=False); n.send("takeoff", lateral=False); n.spin_for(9, lateral=False); n.send("pos_hold", lateral=False); n.spin_for(3, lateral=False)
    print("airborne at %.2f m, starting tilt steps" % n.z, flush=True); n.rows.clear()
    for lane in (0, 1):
        for amp in (3.0, -3.0, 5.0, -5.0):
            kw = {"pitch_deg": amp} if lane == 0 else {"roll_deg": amp}; n.spin_for(2.0, **kw); n.spin_for(3.0)          # step, then level and let it drift down
        n.spin_for(3.0)
    n.send("land", lateral=False); n.spin_for(1.0, lateral=False)
    kf, df, tauf, dlf = fit(n.rows, 0); kr, dr, taur, dlr = fit(n.rows, 1)
    plant = {"k_f": kf, "k_r": kr, "d_f": df, "d_r": dr, "tau_s": (tauf + taur) / 2, "delay_s": (dlf + dlr) / 2}
    print(json.dumps(plant, indent=1)); json.dump(plant, open(OUT, "w"), indent=1); print("written", os.path.abspath(OUT))
    n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__": main()
