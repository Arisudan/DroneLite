#!/usr/bin/env python3
"""Nimbus-style camera-only roaming for the Gazebo house.

What the drone is allowed to use (what the real one has): the Nimbus depth grid (DA3 metric, 6x8 min per cell, 0.2-1.0 m obstacle test),
its own attitude/heading, the flow velocity (odometry twist) and its height. NO floor plan, NO position. The route is a list of compass headings
with a distance budget (the flow velocity integrated), like telling an agent "go that way until you have gone N metres". The simulator's true
position is read ONLY to score the run (rooms visited, collisions) and is never used to steer.
Output: stick commands on /droneforge/fc/command like the GCS.
"""
import argparse, json, math, os, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from house_nav import FloorPlan, wrap, HERE

HFOV = 1.668                                   # Nimbus Air75 profile at 640 px width (fx 292.3)
# (compass heading in the world frame, metres to travel by flow, label). Start: living room facing -y.
LEGS = [(180.0, 6.5, "west, through the opening into the bedroom"), (90.0, 3.5, "north in the bedroom"), (0.0, 11.0, "east, back across the living room to the kitchen"),
        (90.0, 4.0, "north toward the balcony alcove"), (180.0, 5.0, "west along the alcove")]
ROOMS = {"bedroom": lambda x, y: x < -2.8, "kitchen/dining": lambda x, y: x > 4.9 and y > -5.0, "balcony alcove": lambda x, y: y > 2.0 and -2.3 < x < 4.6, "living room": lambda x, y: -2.3 < x < 4.6 and -5.5 < y < 1.8}


class Cam(Node):
    def __init__(self, a):
        super().__init__("droneforge_house_nav_cam"); self.a = a; self.score = FloorPlan()   # the plan is used for scoring only
        self.pub = self.create_publisher(String, "/droneforge/fc/command", 10)
        self.create_subscription(Odometry, "/droneforge/odometry", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(String, "/droneforge/perception/obstacles", self.on_obs, 10)
        self.x = self.y = self.z = self.yaw = 0.0; self.vf = 0.0; self.vr = 0.0; self.tilt = 0.0; self.have = False
        self.obs = None; self.obs_t = 0.0; import collections; self.hist = collections.deque(maxlen=800)   # (t, cumulative forward distance by flow, yaw)
        self.cum = 0.0; self.dr_x = 0.0; self.dr_y = 0.0; self.leg_dr0 = (0.0, 0.0); self.last_t = None; self.stage = "wait"; self.t0 = time.time(); self.leg = 0; self.leg_cum0 = 0.0; self.turn_dir = 0.0
        self.bypass_dir = 0.0; self.bypass_t = 0.0; self.fly_t0 = 1e18; self.collisions = 0; self.in_hit = False; self.rooms = set(); self.dist_true = 0.0; self.last_xy = None; self.leg_t0 = 0.0; self.results = []; self.stuck_t = time.time(); self.stuck_cum = 0.0
        self.timer = self.create_timer(0.05, self.step)

    def on_odom(self, m):
        p = m.pose.pose.position; q = m.pose.pose.orientation; now = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9   # SIM time: with the Gazebo window open the sim runs slower than the wall clock
        self.x, self.y, self.z = p.x, p.y, p.z                      # true position: scoring only
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        sr = 2 * (q.w * q.x + q.y * q.z); cr = 1 - 2 * (q.x * q.x + q.y * q.y)
        self.tilt = max(abs(math.atan2(sr, cr)), abs(math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x))))))
        self.vf = m.twist.twist.linear.x; self.vr = -m.twist.twist.linear.y     # body-frame velocity = what the flow sensor gives
        if self.last_t is not None:                                               # dead reckoning: flow velocity rotated by the heading, integrated (drifts, like the real thing)
            dt = now - self.last_t; vl = -self.vr
            self.dr_x += (self.vf * math.cos(self.yaw) - vl * math.sin(self.yaw)) * dt; self.dr_y += (self.vf * math.sin(self.yaw) + vl * math.cos(self.yaw)) * dt
            self.cum += max(0.0, self.vf) * dt
        self.last_t = now; self.hist.append((time.time(), self.cum)); self.have = True

    def on_obs(self, m):
        try: self.obs = json.loads(m.data); self.obs_t = time.time()
        except Exception: pass

    def send(self, cmd=None, pitch=0.0, roll=0.0, yaw=0.0, z=None):
        d = {"pitch": pitch, "roll": roll, "yaw": yaw}
        if cmd: d["cmd"] = cmd
        if z is not None: d["target_z"] = z
        self.pub.publish(String(data=json.dumps(d)))

    def say(self, s): self.get_logger().info(s)

    def columns(self):
        """Per-column clearance (metres) from the Nimbus grid, brought forward to now by the distance flown since the frame was taken. None if stale."""
        o = self.obs
        if o is None or time.time() - self.obs_t > 4.0: return None
        g = np.array(o["min_m"], float).reshape(o["rows"], o["cols"]) * self.a.depth_scale
        t_cap = o.get("stamp", self.obs_t) - o.get("latency_ms", 0) / 1000.0
        if t_cap < self.fly_t0: return None                          # taken during take-off (camera near the floor)
        c0 = min(self.hist, key=lambda h: abs(h[0] - t_cap))[1] if self.hist else self.cum
        g = g - max(0.0, self.cum - c0)
        # rows 1-5; below the horizon a cell only counts when it is nearer than the floor would be at the drone's height (known from its range sensor)
        H = 480.0; rows = g.shape[0]; out = np.full(g.shape[1], 9.0)
        for r in range(1, min(rows, 6 if self.a.low_rows else 4)):      # default rows 1-3 (above the floor); --low-rows also uses 4-5 with the floor filter (DA3 reads the floor too near: false obstacles, see 2026-11-19 run)
            ang_b = math.atan(max(0.0, (r + 1) * H / rows - 259.7) / 305.2)          # Nimbus Air75 cy / fy; angle below the horizon of the cell's lower edge
            floor_d = (self.z / math.tan(ang_b)) if ang_b > 0.02 else 99.0
            cell = np.where(g[r] < 0.8 * floor_d, g[r], 9.0)
            out = np.minimum(out, cell)
        return np.where(out > 0.05, out, 0.05)

    def step(self):
        if not self.have: return
        now = time.time()
        if self.stage == "wait":
            if now - self.t0 > 1: self.send("arm"); self.stage = "arm"; self.t0 = now
        elif self.stage == "arm":
            if now - self.t0 > 1.5: self.send("takeoff", z=self.a.height); self.stage = "takeoff"; self.t0 = now
        elif self.stage == "takeoff":
            self.send(z=self.a.height)
            if self.z > self.a.height - 0.08 and now - self.t0 > 3:
                self.send("pos_hold", z=self.a.height); self.stage = "fly"; self.obs = None; self.fly_t0 = now; self.leg = 0; self.leg_cum0 = self.cum; self.leg_dr0 = (self.dr_x, self.dr_y); self.leg_t0 = now; self.stuck_t = now; self.stuck_cum = self.cum
                self.say(f"airborne {self.z:.2f} m, camera-only roam, {len(LEGS)} legs")
        elif self.stage == "fly": self.fly(now)
        elif self.stage == "land":
            self.send("land")
            if self.z < 0.08 or now - self.t0 > 15: self.stage = "done"

    def fly(self, now):
        hit = self.score.collided(self.x, self.y) or self.tilt > math.radians(35)
        if hit and not self.in_hit: self.collisions += 1; self.say(f"COLLISION at ({self.x:.2f},{self.y:.2f}) tilt {math.degrees(self.tilt):.0f}")
        self.in_hit = hit
        if self.last_xy: self.dist_true += math.dist(self.last_xy, (self.x, self.y))
        self.last_xy = (self.x, self.y)
        for n, f in ROOMS.items():
            if f(self.x, self.y) and n not in self.rooms: self.rooms.add(n); self.say(f"entered {n}")
        head, budget, label = LEGS[self.leg]
        hv = (math.cos(math.radians(head)), math.sin(math.radians(head))); prog = (self.dr_x - self.leg_dr0[0]) * hv[0] + (self.dr_y - self.leg_dr0[1]) * hv[1]
        if prog >= budget or now - self.leg_t0 > self.a.leg_timeout:
            self.results.append((label, round(prog, 1), "done" if prog >= budget else "timeout"))
            self.say(f"leg {self.leg + 1} '{label}' finished: {prog:.1f} m of progress along the heading")
            self.leg += 1; self.leg_cum0 = self.cum; self.leg_dr0 = (self.dr_x, self.dr_y); self.leg_t0 = now; self.turn_dir = 0.0; self.bypass_dir = 0.0
            if self.leg >= len(LEGS): return self.finish()
            head, budget, label = LEGS[self.leg]
        cols = self.columns()
        want = wrap(math.radians(head) - self.yaw)                      # where the hint heading is, relative to the nose
        if cols is None: self.send(z=self.a.height); return             # no fresh depth: hold still
        n = len(cols); ang = ((n - 1) / 2 - np.arange(n)) * (HFOV / n)  # column centre angles; column 0 is the LEFT of the image = positive (counter-clockwise, like yaw)
        stop, slow = self.a.stop_m, self.a.slow_m
        # steering: the column that is free AND closest to the wanted heading; "free" = clearance beyond slow
        free = cols > slow
        Cmin = float(cols[n // 2 - 1:n // 2 + 1].min())
        if Cmin < slow:                                                  # something ahead: commit to the more open side and keep it (else the hint pulls the drone back into the obstacle)
            if self.bypass_dir == 0.0: self.bypass_dir = 1.0 if cols[:n // 2].mean() > cols[n // 2:].mean() else -1.0
            self.bypass_t = now
        elif self.bypass_dir and now - self.bypass_t > self.a.bypass_s: self.bypass_dir = 0.0
        if self.bypass_dir: want = want + self.bypass_dir * self.a.bypass_rad     # steer toward a heading rotated away from the hint, on the committed side
        cost = np.abs(wrap_arr(ang - want)) + np.where(free, 0.0, 10.0) - 0.15 * np.minimum(cols, 4.0)
        j = int(np.argmin(cost)); rel = float(ang[j]) if cost[j] < 9 else None
        C = float(cols[n // 2 - 1:n // 2 + 1].min())                    # the two centre columns
        if rel is None:                                                 # nothing free: turn in place toward the most open side
            if self.turn_dir == 0.0: self.turn_dir = 1.0 if cols[:n // 2].mean() > cols[n // 2:].mean() else -1.0
            yaw, pitch = self.turn_dir * self.a.max_yaw, 0.0
        else:
            self.turn_dir = 0.0; yaw = float(np.clip(1.2 * rel, -self.a.max_yaw, self.a.max_yaw))
            pitch = self.a.speed * float(np.clip((C - stop) / (slow - stop), 0, 1)) * max(0.0, 1 - abs(rel) / 0.6)
        if C < self.a.back_m: pitch = -self.a.back_speed                 # too close: back away while turning (Nimbus' tracker also moves back)
        if self.vf > self.a.vmax: pitch = min(pitch, 0.0)
        self.send(pitch=pitch, roll=float(np.clip(-0.4 * self.vr, -0.3, 0.3)), yaw=yaw, z=self.a.height)
        if now - getattr(self, "_pp", 0) > 2.0:
            self._pp = now; self.say(f"  leg {self.leg + 1}: progress {(self.dr_x - self.leg_dr0[0]) * math.cos(math.radians(head)) + (self.dr_y - self.leg_dr0[1]) * math.sin(math.radians(head)):.1f}/{budget:.1f} m, heading {math.degrees(self.yaw):.0f} (hint {head:.0f}), centre clear {C:.2f} m, cols {np.round(cols, 1).tolist()} | true pos ({self.x:.2f},{self.y:.2f}) plan-ray {self.score.ray(self.x, self.y, self.yaw):.2f}")

    def finish(self):
        self.send(); self.stage = "land"; self.t0 = time.time()
        s = {"rooms_visited": sorted(self.rooms), "collisions": self.collisions, "true_distance_m": round(self.dist_true, 1), "legs": self.results}
        self.say("SUMMARY " + json.dumps(s)); open(os.path.join(HERE, "..", "worlds", "last_cam_summary.json"), "w").write(json.dumps(s, indent=1))


def wrap_arr(a): return (a + math.pi) % (2 * math.pi) - math.pi


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--height", type=float, default=0.60); p.add_argument("--speed", type=float, default=0.45); p.add_argument("--vmax", type=float, default=0.5)
    p.add_argument("--stop-m", dest="stop_m", type=float, default=0.6); p.add_argument("--slow-m", dest="slow_m", type=float, default=1.3)
    p.add_argument("--depth-scale", dest="depth_scale", type=float, default=1.0, help="1.0 = Nimbus as is (the sim camera now has the Nimbus intrinsics)")
    p.add_argument("--back-m", dest="back_m", type=float, default=0.4); p.add_argument("--back-speed", dest="back_speed", type=float, default=0.25); p.add_argument("--low-rows", dest="low_rows", action="store_true"); p.add_argument("--bypass-s", dest="bypass_s", type=float, default=3.0); p.add_argument("--bypass-rad", dest="bypass_rad", type=float, default=0.9); p.add_argument("--max-yaw", dest="max_yaw", type=float, default=0.35); p.add_argument("--leg-timeout", dest="leg_timeout", type=float, default=90)
    a = p.parse_args(); rclpy.init(); n = Cam(a)
    try:
        while rclpy.ok() and n.stage != "done": rclpy.spin_once(n, timeout_sec=0.05)
    except KeyboardInterrupt: pass
    n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__": main()
