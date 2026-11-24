#!/usr/bin/env python3
"""Gazebo adapter for the camera-only roamer (nimbus_core/roam.py: the same code the GCS camera-roam mode will run).

Everything the roamer gets comes through the interfaces the real system has: the FC pose (/odom + TF from sim_tf_odom.py: with --link real noisy, delayed, drifting),
the Nimbus depth outputs (/droneforge/perception/obstacles) and, with --map rtabmap (default), the RTAB-Map occupancy grid (/map) and its map->odom correction (TF).
Sticks leave through the link emulation (--link real: ~0.15 s late, CRSF-quantised). The true simulator position is used ONLY to score (rooms, collisions, coverage).
It arms, takes off and lands the SIMULATED drone only.
"""
import argparse, json, math, os, random, sys, time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, "..", "..", "software", "pc"))
from house_nav import FloorPlan
from link_emu import LinkEmu
from nimbus_core.roam import Roamer, RoamConfig, Pose
from nimbus_core.grid_map import GridMap

ROOMS = {"bedroom": lambda x, y: x < -2.8, "kitchen/dining": lambda x, y: x > 4.9 and y > -5.0, "balcony alcove": lambda x, y: y > 2.0 and -2.3 < x < 4.6, "living room": lambda x, y: -2.3 < x < 4.6 and -5.5 < y < 1.8}


def yaw_of(q): return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Explorer(Node):
    def __init__(self, a):
        super().__init__("droneforge_house_explore"); self.a = a; self.score = FloorPlan()
        self.pub = self.create_publisher(String, "/droneforge/fc/command", 10)
        self.create_subscription(Odometry, "/droneforge/odometry", self.on_true, qos_profile_sensor_data)      # truth: tilt + scoring only
        self.create_subscription(Odometry, "/odom", self.on_fc, 10)                                              # the FC pose as the link delivers it
        self.create_subscription(String, "/droneforge/perception/obstacles", self.on_obs, 10)
        self.cfg = RoamConfig(cruise=a.cruise, accel=a.accel, jerk=a.jerk, kff=a.kff, kp=a.kp, ki=a.ki, stop_m=a.stop_m, slow_m=a.slow_m, back_m=a.back_m, inflate=a.inflate,
                              replan_s=a.replan_s, goal_timeout=a.goal_timeout, max_time=a.max_time, control=a.control, plant_path=os.path.join(HERE, "..", "config", "plant.json"), max_tilt_deg=a.max_tilt)
        self.use_rtab = a.map == "rtabmap"
        self.roam = Roamer(self.cfg, GridMap() if self.use_rtab else None); self.link = LinkEmu(a.link == "real", random.Random(a.seed))
        if self.use_rtab:
            self.tfb = Buffer(); self.tfl = TransformListener(self.tfb, self)
            self.create_subscription(OccupancyGrid, "/map", self.on_map, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self.p0 = None; self.wx = self.wy = self.z = 0.0; self.tilt = 0.0; self.sim_t = None; self.dt = 0.05; self.have_true = False
        self.fc = None; self.fc_t = 0.0; self.stage = "wait"; self.t0 = time.time()
        self.collisions = 0; self.in_hit = False; self.rooms = set(); self.dist = 0.0; self.last_true = None; self.last_save = 0.0; self.last_say = 0.0; self.why = ""
        self.gcs = None; self.gcs_on = False; self.gcs_t = 0.0                        # --via-gcs: the sticks go to the GCS (EXPLORE), as on the real drone
        if a.via_gcs:
            import socket
            self.gcs = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); self.gcs.setblocking(False); self.gcs_addr = ("127.0.0.1", a.gcs_port)
        self.timer = self.create_timer(0.05, self.step)

    # ---------- inputs
    def on_true(self, m):
        p = m.pose.pose.position; q = m.pose.pose.orientation; t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.p0 is None: self.p0 = (p.x, p.y)
        self.wx, self.wy, self.z = p.x, p.y, p.z
        sr = 2 * (q.w * q.x + q.y * q.z); cr = 1 - 2 * (q.x * q.x + q.y * q.y)
        self.tilt = max(abs(math.atan2(sr, cr)), abs(math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x))))))
        if self.sim_t is not None: self.dt = float(np.clip(t - self.sim_t, 0.005, 0.2))
        self.sim_t = t; self.have_true = True

    def map_pose(self, x, y, yaw, stamp):
        """The pose in the SLAM map frame: odom pose corrected by the map->odom transform RTAB-Map publishes (loop closures)."""
        if not self.use_rtab: return x, y, yaw
        try:
            T = self.tfb.lookup_transform("map", "odom", rclpy.time.Time()); t = T.transform; c, s = yaw_of(t.rotation), 0.0
            cy, sy = math.cos(c), math.sin(c); return t.translation.x + cy * x - sy * y, t.translation.y + sy * x + cy * y, yaw + c
        except Exception:
            return x, y, yaw

    def on_fc(self, m):
        p = m.pose.pose.position; yaw = yaw_of(m.pose.pose.orientation); x, y, yw = self.map_pose(p.x, p.y, yaw, m.header.stamp)
        self.fc = (x, y, yw, m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.x, m.twist.twist.angular.y); self.fc_t = time.time(); self.roam.push_pose(self.fc_t, x, y, yw)

    def on_map(self, m):
        self.roam.map.set_grid(m.info.width, m.info.height, m.info.resolution, m.info.origin.position.x, m.info.origin.position.y, m.data)

    def on_obs(self, m):
        try: o = json.loads(m.data)
        except Exception: return
        self.roam.on_depth(o, self.tilt, time.time())

    # ---------- outputs
    def send(self, cmd=None, pitch=0.0, roll=0.0, yaw=0.0, z=None, lateral=False):
        d = {"pitch": pitch, "roll": roll, "yaw": yaw}
        if lateral: d["lateral"] = "pc"                # the PC controls the lateral axes: the FC must not run its own xy hold
        if cmd: d["cmd"] = cmd
        if z is not None: d["target_z"] = z
        self.link.out(time.time(), d)

    def say(self, s): self.get_logger().info(s)

    def to_gcs(self, d):
        """Stick wishes to the GCS (nimbus_core/steering.py), Betaflight conventions: yaw + = clockwise (the roamer's yaw is counter-clockwise). Reads the GCS answer {"explore": bool}."""
        try: self.gcs.sendto(json.dumps(d).encode(), self.gcs_addr)
        except OSError: pass
        while True:
            try: r = json.loads(self.gcs.recv(512).decode()); self.gcs_on = bool(r.get("explore")); self.gcs_t = time.time()
            except (BlockingIOError, OSError, ValueError): break
        if time.time() - self.gcs_t > 1.0: self.gcs_on = False                                  # no answer: the GCS is not listening

    def step_gcs(self, now):
        if self.stage in ("wait", "parked"):
            self.to_gcs({"roll": 0.0, "pitch": 0.0, "yaw": 0.0})                               # heartbeat; centred sticks
            if self.gcs_on:
                if self.stage == "parked" or self.roam.stage == "finished":                     # 2026-11-21: EXPLORE pressed again after a stop: a fresh search (the map is kept)
                    self.roam.restart(now, self.sim_t, self.fc[2]); self.say("GCS EXPLORE again: fresh search (360 deg look, given-up goals forgotten)")
                else:
                    self.roam.begin(now, self.sim_t, self.fc[2])
                self.stage = "roam"; self.say("GCS EXPLORE is on: roaming (take-off, land and SAFE stay with the GCS)")
            elif now - self.last_say > 5.0: self.last_say = now; self.say("parked: press EXPLORE in the GCS for a fresh search" if self.stage == "parked" else "waiting: take off in the GCS and press EXPLORE")
        elif self.stage == "roam":
            if not self.gcs_on:
                self.stage = "wait"; self.say("GCS EXPLORE is off: paused (the FC holds); press EXPLORE to continue"); return
            x, y, yaw, vf, vl, wr, wp = self.fc
            c = self.roam.step(now, self.sim_t, self.dt, Pose(x, y, yaw, vf, vl, self.tilt, now - self.fc_t, wr, wp))
            self.to_gcs({"roll": c.roll, "pitch": c.pitch, "yaw": -c.yaw})
            if now - self.last_say > 2.0 and c.text: self.last_say = now; self.say("  " + c.text + f" | true ({self.wx:.1f},{self.wy:.1f})")
            if c.done:                                                                          # 2026-11-21: never exit in GCS mode: park (the FC holds), tell the GCS why, wait for EXPLORE again
                self.why = c.reason; self.finish(); self.to_gcs({"done": True, "reason": c.reason}); self.stage = "parked"; self.gcs_on = False
            self.account(now)

    def step(self):
        if not self.have_true or self.fc is None: return
        now = time.time()
        if self.gcs is not None: return self.step_gcs(now)
        for d in self.link.due(now): self.pub.publish(String(data=json.dumps(d)))
        if self.stage == "wait":
            if now - self.t0 > 1: self.send("arm"); self.stage = "arm"; self.t0 = now          # simulation only
        elif self.stage == "arm":
            if now - self.t0 > 1.5: self.send("takeoff", z=self.a.height); self.stage = "takeoff"; self.t0 = now
        elif self.stage == "takeoff":
            self.send(z=self.a.height)
            if self.z > self.a.height - 0.08 and now - self.t0 > 3:
                self.send("pos_hold", z=self.a.height); self.stage = "roam"; self.roam.begin(now, self.sim_t, self.fc[2]); self.say(f"airborne, roaming (link {self.a.link}, map {self.a.map})")
        elif self.stage == "roam":
            x, y, yaw, vf, vl, wr, wp = self.fc
            c = self.roam.step(now, self.sim_t, self.dt, Pose(x, y, yaw, vf, vl, self.tilt, now - self.fc_t, wr, wp))
            self.send(pitch=c.pitch, roll=c.roll, yaw=c.yaw, z=self.a.height, lateral=c.lateral)
            if now - self.last_say > 2.0 and c.text: self.last_say = now; self.say("  " + c.text + f" | true ({self.wx:.1f},{self.wy:.1f})")
            if c.done: self.why = c.reason; self.finish()
            self.account(now)
        elif self.stage == "land":
            self.send("land")
            if self.z < 0.08 or now - self.t0 > 15: self.stage = "done"

    def account(self, now):
        hit = self.score.collided(self.wx, self.wy) or self.tilt > math.radians(35)
        if hit and not self.in_hit: self.collisions += 1; self.say(f"COLLISION at world ({self.wx:.2f},{self.wy:.2f}) tilt {math.degrees(self.tilt):.0f}")
        self.in_hit = hit
        if self.last_true: self.dist += math.dist(self.last_true, (self.wx, self.wy))
        self.last_true = (self.wx, self.wy)
        for n, f in ROOMS.items():
            if f(self.wx, self.wy) and n not in self.rooms: self.rooms.add(n); self.say(f"entered {n}")
        if now - self.last_save > 3.0:
            self.last_save = now; p = self.roam.pose; pose = (p.x, p.y, p.yaw) if p else None
            m = self.roam.map
            if hasattr(m, "ox"): np.savez_compressed(os.path.join(HERE, "..", "worlds", "last_grid.npz"), L=m.L, ox=m.ox, oy=m.oy, res=m.res, pose=np.array(pose if pose else (0, 0, 0)))      # the raw SLAM grid, for tuning on real data
            img = self.roam.map.render(pose, self.roam.path, self.roam.goal); cv2.imwrite(os.path.join(HERE, "..", "worlds", "last_map.png"), img)
            if self.a.show: cv2.imshow("DroneForge | map", img); cv2.waitKey(1)

    def finish(self):
        if self.gcs is None: self.send()
        self.stage = "land"; self.t0 = time.time(); p = self.roam.pose
        cv2.imwrite(os.path.join(HERE, "..", "worlds", "last_map.png"), self.roam.map.render((p.x, p.y, p.yaw), None, None, scale=8))
        s = {"map": self.a.map, "link": self.a.link, "stopped": self.why, "coverage_pct": self.coverage(), "rooms_visited": sorted(self.rooms), "collisions": self.collisions,
             "true_distance_m": round(self.dist, 1), "sim_seconds": round(self.sim_t - self.roam.sim0), "map_updates": self.roam.map.frames}
        self.say("SUMMARY " + json.dumps(s)); open(os.path.join(HERE, "..", "worlds", "last_explore_summary.json"), "w").write(json.dumps(s, indent=1))

    def coverage(self):
        """Share of the reachable floor (true plan, scoring only) that the drone's map marks as seen free space."""
        pl = self.score; m = self.roam.map; ys, xs = np.nonzero(pl.free); ys, xs = ys[::4], xs[::4]; known = 0; Lm = m.L
        for c, r in zip(xs, ys):
            wx, wy = pl.to_xy(c, r); col, row = m.to_cell(wx - self.p0[0], wy - self.p0[1])
            if 0 <= row < Lm.shape[0] and 0 <= col < Lm.shape[1] and Lm[row, col] <= -0.5: known += 1
        return round(100.0 * known / max(1, len(xs)), 1)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--height", type=float, default=0.60); p.add_argument("--cruise", type=float, default=0.35, help="m/s; Nimbus default waypoint speed is 0.25")
    p.add_argument("--accel", type=float, default=0.35); p.add_argument("--jerk", type=float, default=0.8)
    p.add_argument("--kff", type=float, default=1.5); p.add_argument("--kp", type=float, default=1.0); p.add_argument("--ki", type=float, default=0.6)
    p.add_argument("--stop-m", dest="stop_m", type=float, default=0.6); p.add_argument("--slow-m", dest="slow_m", type=float, default=1.3); p.add_argument("--back-m", dest="back_m", type=float, default=0.35)
    p.add_argument("--inflate", type=float, default=0.25); p.add_argument("--replan-s", dest="replan_s", type=float, default=0.8); p.add_argument("--goal-timeout", dest="goal_timeout", type=float, default=60)
    p.add_argument("--max-time", dest="max_time", type=float, default=720); p.add_argument("--show", action="store_true")
    p.add_argument("--control", choices=("lqr", "stick"), default="lqr", help="lqr = Nimbus-style tilt control on the PC (FC keeps attitude + height only)"); p.add_argument("--max-tilt", dest="max_tilt", type=float, default=15.0)
    p.add_argument("--link", choices=("ideal", "real"), default="ideal"); p.add_argument("--seed", type=int, default=1); p.add_argument("--map", choices=("rtabmap", "internal"), default="rtabmap")
    p.add_argument("--via-gcs", dest="via_gcs", action="store_true", help="send the sticks to the GCS (EXPLORE button) instead of the sim FC: the real-drone path")
    p.add_argument("--gcs-port", dest="gcs_port", type=int, default=5766)
    a = p.parse_args(); rclpy.init(); n = Explorer(a)
    try:
        while rclpy.ok() and n.stage != "done": rclpy.spin_once(n, timeout_sec=0.05)
    except KeyboardInterrupt: pass
    n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__": main()
