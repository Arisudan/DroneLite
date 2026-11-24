#!/usr/bin/env python3
"""Room-to-room navigator for the Gazebo house.

* Route: A* over the floor plan (worlds/house_plan.npz, sliced from the world file at flight height) through the rooms below.
* Position: simulator odometry (/droneforge/odometry). The real drone has no such sensor; this is the one non-camera input.
* Obstacles: camera only. Nimbus depth model (Depth Anything 3 metric) -> 6x8 occupancy grid (vision/nimbus_perception.py) on
  /droneforge/perception/obstacles. The drone slows, side-steps and turns away from whatever the grid shows ahead.
* Output: stick commands on /droneforge/fc/command, exactly like the GCS sends them (pitch/roll/yaw sticks, target_z).
"""
import argparse, heapq, json, math, os, sys, time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String

HERE = os.path.dirname(os.path.abspath(__file__))
PLAN = os.path.join(HERE, "..", "worlds", "house_plan.npz")
START = (2.6, -3.2)
# name, x, y  (chosen from house_plan.png; snapped to the nearest free cell)
TOUR = [("bedroom, TV side", -6.8, -3.4), ("bedroom, bed side", -5.6, 0.2), ("kitchen / dining", 6.4, -2.2),
        ("balcony alcove", 1.2, 3.4), ("back to start", START[0], START[1])]


class FloorPlan:
    def __init__(self, inflate_m=0.30, res_out=0.10):
        d = np.load(PLAN); g = d["grid"]; self.x0 = float(d["x0"]); self.y0 = float(d["y0"]); self.res = float(d["res"])
        r = int(round(inflate_m / self.res))
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))
        self.raw = g > 0
        self.occ = cv2.dilate(g, k) > 0
        # outside the house is not free space: flood from the start and keep only the connected free area
        n, lab = cv2.connectedComponents((~self.occ).astype(np.uint8), connectivity=4)
        c, rr = self.to_cell(*START)
        self.free = lab == lab[rr, c]
        self.hit_r = int(round(0.12 / self.res))

    def to_cell(self, x, y): return int((x - self.x0) / self.res), int((y - self.y0) / self.res)
    def to_xy(self, c, r): return self.x0 + (c + .5) * self.res, self.y0 + (r + .5) * self.res
    def snap(self, x, y):
        c, r = self.to_cell(x, y)
        if self.free[r, c]: return x, y
        ys, xs = np.nonzero(self.free); i = int(np.argmin((xs - c) ** 2 + (ys - r) ** 2)); return self.to_xy(xs[i], ys[i])
    def collided(self, x, y):
        c, r = self.to_cell(x, y); h = self.hit_r
        return bool(self.raw[max(0, r - h):r + h + 1, max(0, c - h):c + h + 1].any())
    def ray(self, x, y, yaw, maxd=6.0):
        for d in np.arange(0.1, maxd, 0.05):
            c, r = self.to_cell(x + d * math.cos(yaw), y + d * math.sin(yaw))
            if not (0 <= r < self.raw.shape[0] and 0 <= c < self.raw.shape[1]): return float(d)
            if self.raw[r, c]: return float(d)
        return maxd
    def los(self, a, b):
        n = int(math.hypot(b[0] - a[0], b[1] - a[1]) / (self.res * 0.5)) + 1
        for t in np.linspace(0, 1, n):
            c, r = self.to_cell(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
            if not self.free[r, c]: return False
        return True
    def path(self, a, b):
        a = self.snap(*a); b = self.snap(*b); s = self.to_cell(*a); g = self.to_cell(*b); H, W = self.free.shape
        pq = [(0, 0, s)]; came = {s: None}; cost = {s: 0}
        nb = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1), (1, 1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (-1, -1, 1.414)]
        while pq:
            _, cst, cur = heapq.heappop(pq)
            if cur == g: break
            for dc, dr, w in nb:
                n = (cur[0] + dc, cur[1] + dr)
                if not (0 <= n[0] < W and 0 <= n[1] < H) or not self.free[n[1], n[0]]: continue
                nc = cst + w
                if nc < cost.get(n, 1e9):
                    cost[n] = nc; came[n] = cur; heapq.heappush(pq, (nc + math.hypot(g[0] - n[0], g[1] - n[1]), nc, n))
        if g not in came: return None
        cells = []; c = g
        while c is not None: cells.append(c); c = came[c]
        pts = [self.to_xy(*c) for c in cells[::-1]]
        out = [pts[0]]; i = 0                                  # string-pull: keep only the corners that matter
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1 and not self.los(pts[i], pts[j]): j -= 1
            out.append(pts[j]); i = j
        return out


def wrap(a): return (a + math.pi) % (2 * math.pi) - math.pi


class Nav(Node):
    def __init__(self, a):
        super().__init__("droneforge_house_nav")
        self.a = a; self.plan = FloorPlan()
        self.pub = self.create_publisher(String, "/droneforge/fc/command", 10)
        self.rep = self.create_publisher(String, "/droneforge/nav/status", 10)
        self.create_subscription(Odometry, "/droneforge/odometry", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(String, "/droneforge/perception/obstacles", self.on_obs, 10)
        self.x = self.y = self.z = self.yaw = 0.0; self.vf = self.vr = 0.0; self.tilt = 0.0; self.have = False
        self.obs = None; self.obs_t = 0.0; self.turn_dir = 0.0; self.blocked = False; import collections; self.hist = collections.deque(maxlen=400)
        self.t0 = time.time(); self.stage = "wait"; self.legs = []; self.leg = -1; self.path = []; self.wp = 0
        self.log = []; self.collisions = 0; self.in_hit = False; self.leg_t0 = 0.0; self.results = []
        self.dist = 0.0; self.last_xy = None; self.stuck_t = time.time(); self.stuck_xy = None; self.replans = 0
        self.timer = self.create_timer(0.05, self.step)

    def on_odom(self, m):
        p = m.pose.pose.position; q = m.pose.pose.orientation
        self.x, self.y, self.z = p.x, p.y, p.z
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        sr = 2 * (q.w * q.x + q.y * q.z); cr = 1 - 2 * (q.x * q.x + q.y * q.y); roll = math.atan2(sr, cr)
        pitch = math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x)))); self.tilt = max(abs(roll), abs(pitch))
        vx, vy = m.twist.twist.linear.x, m.twist.twist.linear.y
        self.vf = vx; self.vr = -vy          # odometry twist is in the body frame (x forward, y left); vr is positive to the right
        self.vw = (vx * math.cos(self.yaw) - vy * math.sin(self.yaw), vx * math.sin(self.yaw) + vy * math.cos(self.yaw))
        self.have = True
        self.hist.append((time.time(), self.x, self.y, self.yaw))

    def on_obs(self, m):
        try: self.obs = json.loads(m.data); self.obs_t = time.time()
        except Exception: pass

    def send(self, cmd=None, pitch=0.0, roll=0.0, yaw=0.0, z=None):
        d = {"pitch": pitch, "roll": roll, "yaw": yaw}
        if cmd: d["cmd"] = cmd
        if z is not None: d["target_z"] = z
        self.pub.publish(String(data=json.dumps(d)))

    def say(self, s):
        self.get_logger().info(s); self.rep.publish(String(data=s))

    def start_leg(self, i):
        self.leg = i; name, gx, gy = TOUR[i]
        self.path = self.plan.path((self.x, self.y), (gx, gy)); self.wp = 1; self.leg_t0 = time.time(); self.leg_hits = self.collisions
        self.stuck_t = time.time(); self.stuck_xy = (self.x, self.y)
        if not self.path: self.say(f"leg {i + 1} '{name}': no path"); self.results.append((name, "no path", 0.0)); return False
        self.say(f"leg {i + 1}/{len(TOUR)} -> {name}: {len(self.path)} corners, {sum(math.dist(self.path[k], self.path[k + 1]) for k in range(len(self.path) - 1)):.1f} m planned")
        return True

    def avoid(self):
        """(speed scale, side-step roll stick, yaw bias) from the camera obstacle grid. Grid columns run left->right."""
        o = self.obs
        if o is None or time.time() - self.obs_t > 8.0: return (1.0 if self.a.no_depth else 0.0), 0.0, 0.0, None
        near = np.array(o.get("p10_m", o["min_m"]), float).reshape(o["rows"], o["cols"]) * self.a.depth_scale   # per-cell nearest depth, metres
        # the depth model needs ~3 s per frame on this CPU: move the frame forward to now by the distance flown since it was captured
        t_cap = o.get("stamp", self.obs_t) - o.get("latency_ms", 0) / 1000.0
        h0 = min(self.hist, key=lambda h: abs(h[0] - t_cap)) if self.hist else None
        if h0: near = near - max(0.0, (self.x - h0[1]) * math.cos(h0[3]) + (self.y - h0[2]) * math.sin(h0[3]))
        mid = near[1:4]                                                            # rows 1-3 only: row 0 is ceiling, rows 4-5 are floor (a floor that is 'near' is not an obstacle)
        cols = np.where(np.isnan(mid), 9.0, mid).min(0)
        c = len(cols); L = cols[:c // 3].min(); C = cols[c // 3:2 * c // 3].min(); R = cols[2 * c // 3:].min()
        stop, slow = self.a.stop_m, self.a.slow_m
        scale = float(np.clip((C - stop) / (slow - stop), 0.0, 1.0))
        side = 0.0; yawb = 0.0
        self.blocked = False
        if C >= slow * 1.1: self.turn_dir = 0.0
        if C < slow:                                                               # something ahead: turn and shift toward the freer side (direction kept until clear)
            if self.turn_dir == 0.0: self.turn_dir = 1.0 if R > L else -1.0
            sgn = self.turn_dir
            side = 0.0
            yawb = sgn * float(np.clip((slow - C) / (slow - stop), 0.3, 1.0))
            self.blocked = True
        if L < stop * 1.1: side = max(side, 0.3); yawb = max(yawb, 0.2)           # wall close on the left: move right
        if R < stop * 1.1: side = min(side, -0.3); yawb = min(yawb, -0.2)
        return scale, side, yawb, (round(float(L), 2), round(float(C), 2), round(float(R), 2))

    def step(self):
        if not self.have: return
        now = time.time()
        if self.stage == "wait":
            if now - self.t0 > 1.0: self.send("arm"); self.stage = "arm"; self.t0 = now
        elif self.stage == "arm":
            if now - self.t0 > 1.5: self.send("takeoff", z=self.a.height); self.stage = "takeoff"; self.t0 = now
        elif self.stage == "takeoff":
            self.send(z=self.a.height)
            if self.z > self.a.height - 0.08 and now - self.t0 > 3:
                self.say(f"airborne at {self.z:.2f} m, starting tour"); self.send("pos_hold", z=self.a.height); self.stage = "fly"; self.obs = None
                self.start_leg(0) or self.finish()
        elif self.stage == "fly":
            self.fly(now)
        elif self.stage == "land":
            self.send("land")
            if self.z < 0.08 or now - self.t0 > 15: self.stage = "done"

    def fly(self, now):
        # collision accounting (floor-plan footprint test + crash attitude)
        hit = self.plan.collided(self.x, self.y) or self.tilt > math.radians(35)
        if hit and not self.in_hit: self.collisions += 1; self.say(f"COLLISION at ({self.x:.2f},{self.y:.2f}) tilt {math.degrees(self.tilt):.0f} deg")
        self.in_hit = hit
        if self.last_xy: self.dist += math.dist(self.last_xy, (self.x, self.y))
        self.last_xy = (self.x, self.y)
        name = TOUR[self.leg][0]
        tgt = self.path[self.wp]
        # reached the corner?
        reach = 0.35 if self.wp < len(self.path) - 1 else 0.30
        if math.dist((self.x, self.y), tgt) < reach:
            self.wp += 1
            if self.wp >= len(self.path):
                self.results.append((name, "reached", now - self.leg_t0)); self.say(f"reached '{name}' in {now - self.leg_t0:.0f} s")
                self.send(z=self.a.height)
                if self.leg + 1 < len(TOUR):
                    if not self.start_leg(self.leg + 1): self.finish()
                else: self.finish()
                return
            tgt = self.path[self.wp]
        # stuck / timeout
        if self.stuck_xy is None or math.dist(self.stuck_xy, (self.x, self.y)) > 0.4: self.stuck_xy = (self.x, self.y); self.stuck_t = now
        if now - self.stuck_t > 20 or now - self.leg_t0 > self.a.leg_timeout:
            self.say(f"'{name}': {'stuck' if now - self.stuck_t > 20 else 'timeout'}, replanning"); self.replans += 1
            if self.replans > 3 or now - self.leg_t0 > self.a.leg_timeout:
                self.results.append((name, "failed", now - self.leg_t0))
                if self.leg + 1 < len(TOUR): self.start_leg(self.leg + 1) or self.finish()
                else: self.finish()
                return
            self.start_leg(self.leg); return
        # steer
        want = math.atan2(tgt[1] - self.y, tgt[0] - self.x); err = wrap(want - self.yaw)
        scale, side, yawb, cols = self.avoid()
        yaw_cmd = float(np.clip((yawb * self.a.max_yaw) if self.blocked else (0.8 * err), -self.a.max_yaw, self.a.max_yaw))
        align = max(0.0, 1.0 - abs(err) / 0.6)                                      # rotate on the spot when the heading is far off
        d = math.dist((self.x, self.y), tgt)
        speed = self.a.speed * min(1.0, 0.35 + d / 1.2)
        pitch = float(np.clip(speed * scale * align - 0.25 * (self.vf - 0.0) * (1 if self.vf > self.a.vmax else 0), -1, 1))
        if self.vf > self.a.vmax: pitch = min(pitch, 0.0)
        roll = float(np.clip(side - 0.3 * self.vr, -1, 1)) if side else float(np.clip(-0.4 * self.vr, -0.3, 0.3))
        self.send(pitch=pitch, roll=roll, yaw=yaw_cmd, z=self.a.height)
        if int(now * 2) != getattr(self, "_p", -1):
            self._p = int(now * 2)
            if int(now) % 2 == 0 and now - getattr(self, "_pp", 0) > 1.9:
                self._pp = now; self.say(f"  {name}: pos ({self.x:.2f},{self.y:.2f}) z {self.z:.2f} yaw {math.degrees(self.yaw):.0f} tilt {math.degrees(self.tilt):.0f} v {self.vf:.2f} cam L/C/R {cols} | plan-ray ahead {self.plan.ray(self.x, self.y, self.yaw):.2f}")

    def finish(self):
        self.send(); self.stage = "land"; self.t0 = time.time()
        ok = sum(1 for r in self.results if r[1] == "reached")
        summary = {"legs_reached": ok, "legs_total": len(TOUR), "collisions": self.collisions, "distance_m": round(self.dist, 1),
                   "results": [(n, s, round(t, 0)) for n, s, t in self.results]}
        self.say("SUMMARY " + json.dumps(summary)); open(os.path.join(HERE, "..", "worlds", "last_nav_summary.json"), "w").write(json.dumps(summary, indent=1))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--no-depth", action="store_true", help="plan-following only (no camera avoidance), for testing the route")
    p.add_argument("--depth-scale", dest="depth_scale", type=float, default=0.65, help="metres per DA3 metre, measured in this house (DA3 reads about 1.6x too far with the sim camera)"); p.add_argument("--max-yaw", dest="max_yaw", type=float, default=0.35); p.add_argument("--height", type=float, default=0.60); p.add_argument("--speed", type=float, default=0.45)
    p.add_argument("--vmax", type=float, default=0.8); p.add_argument("--stop-m", dest="stop_m", type=float, default=0.9)
    p.add_argument("--slow-m", dest="slow_m", type=float, default=2.0); p.add_argument("--leg-timeout", dest="leg_timeout", type=float, default=120)
    a = p.parse_args(); rclpy.init(); n = Nav(a)
    try:
        while rclpy.ok() and n.stage != "done": rclpy.spin_once(n, timeout_sec=0.05)
    except KeyboardInterrupt: pass
    n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__": main()
