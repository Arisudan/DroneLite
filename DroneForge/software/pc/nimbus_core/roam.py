"""Camera roaming logic shared by the Gazebo explorer and (later) the GCS "camera roam" mode. ROS-free, Qt-free, takes its inputs as arguments.

What it uses (all of it exists on the real drone): the Nimbus depth outputs (6x8 grid + live voxel floor plan), the FC's EKF pose (x, y, yaw, body velocity,
tilt), time. What it returns: stick values for pitch / roll / yaw. It never arms, takes off or lands: the caller does that. Safety: no fresh pose or no
fresh depth -> zero sticks.
Design (Nimbus-style): jerk-limited speed reference with the Nimbus DfSpeedControl limits (0.35 m/s^2, 0.8 m/s^3), Nimbus yaw law (P 0.65, max 0.35, done 0.08 rad),
face the direction of travel; depth only slows / stops / backs the drone; an accumulated occupancy map + frontier exploration + A* decides where to go.
"""
from __future__ import annotations

import collections
import math
from dataclasses import dataclass, field

import cv2
import numpy as np

from .lateral_lqr import LateralLqr, LqrConfig, PlantID, deg_to_stick
from .occupancy_map import OccupancyMap

K3 = np.ones((3, 3), np.uint8)


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass
class RoamConfig:
    cruise: float = 0.35            # m/s (Nimbus default waypoint speed is 0.25)
    accel: float = 0.35             # m/s^2   Nimbus DfSpeedControl
    jerk: float = 0.8               # m/s^3   Nimbus DfSpeedControl
    kff: float = 1.5                # stick per m/s (measured in the sim: 0.5 stick ~ 0.33 m/s)
    kp: float = 1.0
    ki: float = 0.6
    stop_m: float = 0.6
    slow_m: float = 1.3
    back_m: float = 0.35
    inflate: float = 0.25           # safety margin around obstacles (m)
    replan_s: float = 0.8
    goal_timeout: float = 60.0
    max_time: float = 720.0
    max_pose_age: float = 0.6       # s, stop when the FC pose is older than this
    max_depth_age: float = 2.5      # s, hold still when the depth is older than this
    control: str = "lqr"            # "lqr" = Nimbus-style tilt-angle control on the PC; "stick" = speed-to-stick (legacy)
    plant_path: str = ""            # config/plant.json from tools/sysid_plant.py (K, D of our drone)
    wn: float = 1.1
    zeta: float = 0.77
    max_tilt_deg: float = 15.0
    min_map_frames: int = 8          # never call the house explored before the map has had this many updates
    min_explore_s: float = 60.0
    no_front_limit: int = 8          # consecutive plans without a frontier before the house counts as explored
    reach_wait_s: float = 3.0        # after reaching a goal, look around this long so the (slow) SLAM map can update before the goal is given up


@dataclass
class Pose:
    x: float; y: float; yaw: float; vf: float; vl: float; tilt: float; age: float = 0.0   # vl: body velocity to the LEFT
    wr: float = 0.0; wp: float = 0.0          # body rates (rad/s): roll rate, pitch rate (from the estimator)


@dataclass
class Command:
    pitch: float = 0.0; roll: float = 0.0; yaw: float = 0.0
    text: str = ""; done: bool = False; reason: str = ""
    lateral: bool = False                    # True: pitch/roll are PC tilt commands (the FC must not run its own xy hold); False: the FC keeps holding by itself


class SpeedProfile:
    """Jerk- and acceleration-limited speed reference."""
    def __init__(self, a_max: float, j_max: float) -> None:
        self.v = 0.0; self.a = 0.0; self.a_max = a_max; self.j_max = j_max

    def step(self, v_target: float, dt: float) -> float:
        a_des = float(np.clip(2.0 * (v_target - self.v), -self.a_max, self.a_max))
        self.a += float(np.clip(a_des - self.a, -self.j_max * dt, self.j_max * dt)); self.v = max(0.0, self.v + self.a * dt)
        return self.v


def yaw_stick(target: float, yaw: float) -> tuple[float, float]:
    """Nimbus YawTargetController: P 0.65, max 0.35 (stick units), stop within the tolerance."""
    err = wrap(target - yaw)
    return (0.0 if abs(err) <= 0.04 else float(np.clip(0.65 * err, -0.35, 0.35))), err


class PathTrack:
    """A polyline with arc length: reference position and tangent at a given distance along it."""
    def __init__(self, pts) -> None:
        self.pts = list(pts); self.cum = [0.0]
        for i in range(1, len(self.pts)): self.cum.append(self.cum[-1] + math.dist(self.pts[i - 1], self.pts[i]))
        self.total = self.cum[-1]

    def at(self, s: float):
        s = max(0.0, min(s, self.total))
        for i in range(1, len(self.pts)):
            if s <= self.cum[i] or i == len(self.pts) - 1:
                seg = max(self.cum[i] - self.cum[i - 1], 1e-9); t = (s - self.cum[i - 1]) / seg; a, b = self.pts[i - 1], self.pts[i]
                return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t), ((b[0] - a[0]) / seg, (b[1] - a[1]) / seg)
        return self.pts[-1], (0.0, 0.0)

    def project(self, q) -> float:
        best, bs = 1e9, 0.0
        for i in range(1, len(self.pts)):
            a, b = self.pts[i - 1], self.pts[i]; dx, dy = b[0] - a[0], b[1] - a[1]; L2 = dx * dx + dy * dy
            t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((q[0] - a[0]) * dx + (q[1] - a[1]) * dy) / L2)); d = math.hypot(q[0] - (a[0] + t * dx), q[1] - (a[1] + t * dy))
            if d < best: best, bs = d, self.cum[i - 1] + t * math.sqrt(L2)
        return bs


class Roamer:
    def __init__(self, cfg: RoamConfig | None = None, map=None) -> None:
        self.cfg = cfg or RoamConfig(); self.map = map if map is not None else OccupancyMap(); self.nearest = None; self.nearest_t = 0.0
        self.lqr = LateralLqr(PlantID.load(self.cfg.plant_path) if self.cfg.plant_path else PlantID(), LqrConfig(wn=self.cfg.wn, zeta=self.cfg.zeta, max_tilt_deg=self.cfg.max_tilt_deg))
        self.track = None; self.s_ref = 0.0; self.p_ref = None; self.hold_ref = None; self.prof = SpeedProfile(self.cfg.accel, self.cfg.jerk)
        self.hist: collections.deque = collections.deque(maxlen=800)
        self.cols = None; self.cols_t = 0.0; self.cols_pose = None
        self.stage = "idle"; self.fly_t0 = 1e18; self.sim0 = 0.0
        self.path: list = []; self.goal = None; self.goal_t = 0.0; self.blacklist: list = []; self.no_front = 0; self.last_plan = 0.0
        self.backing = False; self.back_n = 0; self.back_goal = None
        self.boxed_since = None; self.escaping = False; self._occ_cache = (-1, None)
        self.boxed_xy = None; self.relax_until = -1e9                                   # boxed-in: the relaxed margin is kept until the drone has really left that spot      # 2026-11-21: boxed-in handling, map side clearance      # back-off latch + count per goal (the 2026-11-21 shuttle fix: forward/back limit cycle in front of walls)
        self.scan_acc = 0.0; self.prev_yaw = 0.0; self.vint = 0.0; self.blocked_t = None; self.trail = []; self.plan_info = ""; self.no_front_since = None; self.last_unblack = -1e9; self.relax = 0; self.explore_t0 = 0.0; self.wait_until = 0.0; self.waiting_goal = None; self.stall_t = None; self.pinned_t = None; self.retreat_until = 0.0; self.pose = None

    # ---------- inputs
    def begin(self, now: float, sim_t: float, yaw: float) -> None:
        self.stage = "scan"; self.fly_t0 = now; self.sim0 = sim_t; self.prev_yaw = yaw; self.scan_acc = 0.0

    def push_pose(self, t_wall: float, x: float, y: float, yaw: float) -> None:
        self.hist.append((t_wall, x, y, yaw))

    def on_depth(self, obs: dict, tilt: float, now: float) -> None:
        """obs: the perception JSON (min_m grid, floorplan, stamp, latency_ms). Called when a new depth answer arrives."""
        if self.stage not in ("scan", "explore") or not self.hist: return
        t_cap = obs.get("stamp", now) - (obs.get("latency_ms", 0) + obs.get("feed_ms", 0)) / 1000.0      # depth round trip + the O4 feed latency
        if t_cap < self.fly_t0: return
        h = min(self.hist, key=lambda q: abs(q[0] - t_cap))
        self.cols = np.array(obs["min_m"], float).reshape(obs["rows"], obs["cols"])[1:4].min(0); self.cols_t = now; self.cols_pose = h
        self.nearest = obs.get("nearest_m") if obs.get("obstacle") else None; self.nearest_t = now      # Nimbus obstacle test: valid 0.2-1.0 m, more than 19 samples
        if getattr(self.map, "accumulates", True) and tilt < 0.14 and obs.get("floorplan"): self.map.update(obs["floorplan"], h[1], h[2], h[3])

    def clearance(self, p: Pose, now: float):
        if self.cols is None or now - self.cols_t > self.cfg.max_depth_age: return None
        h = self.cols_pose; c = self.cols - max(0.0, (p.x - h[1]) * math.cos(h[3]) + (p.y - h[2]) * math.sin(h[3]))   # bring the frame forward by the distance flown since it was taken
        c = np.where(c > 0.05, c, 0.05); n = len(c)
        return float(c[n // 2 - 1:n // 2 + 1].min()), float(c[:n // 2].min()), float(c[n // 2:].min())

    def restart(self, now: float, sim_t: float, yaw: float) -> None:
        """A fresh search on request (GCS EXPLORE pressed again after it stopped): forget given-up goals and margins, look around 360 deg, plan again. The map is kept."""
        self.blacklist.clear(); self.relax = 0; self.no_front = 0; self.no_front_since = None; self.boxed_since = None; self.escaping = False; self.boxed_xy = None; self.relax_until = -1e9
        self.path = []; self.goal = None; self.waiting_goal = None; self.wait_until = 0.0; self.retreat_until = 0.0; self.backing = False; self.back_n = 0
        self.prof.v = 0.0; self.prof.a = 0.0; self.hold_ref = None; self.track = None; self.p_ref = None
        self.begin(now, sim_t, yaw)

    def map_side_clearance(self, p: Pose, reach: float = 0.8):
        """Distance to the nearest mapped wall on the drone's LEFT and RIGHT (the forward camera never sees them), searched over a band 0.15 m fore and aft. None = no map."""
        m = self.map
        if m is None or not hasattr(m, "occupied") or m.frames == 0: return None
        if self._occ_cache[0] != m.frames: self._occ_cache = (m.frames, m.occupied(min_area=3))
        occ = self._occ_cache[1]; c, s_ = math.cos(p.yaw), math.sin(p.yaw); out = []
        for side in (1.0, -1.0):                                                       # +1 = left (y+ in the body frame), -1 = right
            lx, ly = -s_ * side, c * side; best = reach
            for along in (-0.15, 0.0, 0.15):
                bx, by = p.x + c * along, p.y + s_ * along; d = 0.05
                while d < best:
                    col, row = m.to_cell(bx + lx * d, by + ly * d)
                    if 0 <= row < occ.shape[0] and 0 <= col < occ.shape[1] and occ[row, col]: best = d; break
                    d += m.res
            out.append(best)
        return out[0], out[1]

    # ---------- planning on the accumulated map
    def plan(self, p: Pose, relax: int = 0):
        m = self.map
        if not (~m.unknown()).any(): return "wait"                                          # no map yet
        c0, c1, r0, r1 = m.bounds(margin=25)
        occ = m.occupied(min_area=(12 if getattr(m, "accumulates", True) is False else 3) + 18 * relax)[r0:r1, c0:c1]
        occ_hard = (m._bump_mask()[r0:r1, c0:c1] > 0) if hasattr(m, "_bump_mask") else np.zeros(occ.shape, bool)       # walls learned by bumping: never carved
        unk = m.unknown()[r0:r1, c0:c1]; fre = m.free()[r0:r1, c0:c1]
        rad = max(1, int(round(max(0.12, self.cfg.inflate - 0.06 * relax) / m.res))); infl = cv2.dilate(occ.astype(np.uint8), cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * rad + 1, 2 * rad + 1))) > 0
        trav = fre & ~infl
        if self.trail:                                                                      # the drone has flown through these cells: they are free whatever speckle the depth later put there
            tm = np.zeros(trav.shape, np.uint8)
            for tx, ty in self.trail:
                tc, tr_ = m.to_cell(tx, ty); tc -= c0; tr_ -= r0
                if 0 <= tr_ < tm.shape[0] and 0 <= tc < tm.shape[1]: cv2.circle(tm, (tc, tr_), max(2, int(round(0.18 / m.res))), 1, -1)
            trav |= (tm > 0) & ~occ_hard
        sc, sr = m.to_cell(p.x, p.y); sc -= c0; sr -= r0
        if not (0 <= sr < trav.shape[0] and 0 <= sc < trav.shape[1]): return None
        if not trav[sr, sc]:                                                               # inside a wall's safety margin: first fly OUT to the nearest cell with full clearance
            passable = (fre | (~unk & ~occ)) & ~occ & ~occ_hard
            Dq = np.full(trav.shape, -1, np.int32); Dq[sr, sc] = 0; fr0 = np.zeros(trav.shape, bool); fr0[sr, sc] = True; seen = fr0.copy(); k = 0; hit = None
            while fr0.any() and k < 40 and hit is None:                                      # <= 2 m of escape
                k += 1; nf = cv2.dilate(fr0.astype(np.uint8), K3).astype(bool) & passable & ~seen; Dq[nf] = k; seen |= nf; fr0 = nf
                cand = nf & trav
                if cand.any(): ys, xs = np.nonzero(cand); hit = (int(xs[0]), int(ys[0]))
            if hit is not None:
                cur = hit; cells = [cur]
                while Dq[cur[1], cur[0]] > 0:
                    nxt = next(((cur[0] + dx, cur[1] + dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1) if (dx or dy) and 0 <= cur[1] + dy < Dq.shape[0] and 0 <= cur[0] + dx < Dq.shape[1]
                                and Dq[cur[1] + dy, cur[0] + dx] == Dq[cur[1], cur[0]] - 1), None)
                    if nxt is None: break
                    cur = nxt; cells.append(cur)
                pts = [m.to_xy(cx + c0, cy + r0) for cx, cy in cells[::-1]]
                self.plan_info = f"escape: {len(pts) * m.res:.2f} m out of the safety margin"
                return "escape", pts
        yy, xx = np.ogrid[:trav.shape[0], :trav.shape[1]]; trav |= ((xx - sc) ** 2 + (yy - sr) ** 2 <= (rad + 1) ** 2) & ~occ
        D = np.full(trav.shape, -1, np.int32); D[sr, sc] = 0; front = np.zeros(trav.shape, bool); front[sr, sc] = True; reach = front.copy(); step = 0
        while front.any() and step < 900:
            step += 1; nf = cv2.dilate(front.astype(np.uint8), K3).astype(bool) & trav & ~reach; D[nf] = step; reach |= nf; front = nf
        fr_all = trav & reach & (cv2.dilate(unk.astype(np.uint8), K3) > 0)
        self.plan_info = f"relax {relax}: traversable {int(trav.sum())}, reachable {int(reach.sum())}, frontier cells {int(fr_all.sum())}, blacklist {len(self.blacklist)}"
        er = max(1, int(round(0.15 / m.res))); safe = cv2.erode(trav.astype(np.uint8), cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * er + 1, 2 * er + 1))) > 0
        fr = fr_all                                                                         # clusters are found on all frontier cells; a cell with 15 cm of extra room is preferred inside each cluster (not a corner)
        n, lab, st, _ = cv2.connectedComponentsWithStats(fr.astype(np.uint8), connectivity=8)
        best = None
        for k in range(1, n):
            if st[k, cv2.CC_STAT_AREA] < 3: continue
            ys, xs = np.nonzero(lab == k); sm = safe[ys, xs]; unsafe_pen = 0 if sm.any() else 6
            if sm.any(): ys, xs = ys[sm], xs[sm]
            d = D[ys, xs]; i = int(np.argmin(d))
            if d[i] < 3: continue
            gx, gy = m.to_xy(xs[i] + c0, ys[i] + r0)
            if any(math.hypot(gx - bx, gy - by) < 0.6 for bx, by in self.blacklist): continue
            cost = d[i] + unsafe_pen - (1000 if (self.goal is not None and math.hypot(gx - self.goal[0], gy - self.goal[1]) < 1.2) else 0)    # keep the current goal while it is still a valid frontier (changing it every replan makes the drone dither)
            if best is None or cost < best[0]: best = (cost, int(xs[i]), int(ys[i]), gx, gy)
        if best is None:
            return "boxed" if (int(trav.sum()) > 0 and int(reach.sum()) < 0.5 * int(trav.sum())) else "none"      # most of the open map out of reach = boxed in, NOT explored
        cur = (best[1], best[2]); cells = [cur]
        while D[cur[1], cur[0]] > 0:
            nxt = None
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    q = (cur[0] + dx, cur[1] + dy)
                    if (dx or dy) and 0 <= q[1] < D.shape[0] and 0 <= q[0] < D.shape[1] and D[q[1], q[0]] == D[cur[1], cur[0]] - 1: nxt = q; break
                if nxt: break
            if nxt is None: break
            cur = nxt; cells.append(cur)
        pts = [m.to_xy(cx + c0, cy + r0) for cx, cy in cells[::-1]]

        def los(a, b):
            k = int(math.hypot(b[0] - a[0], b[1] - a[1]) / (m.res * 0.5)) + 1
            for t in np.linspace(0, 1, k):
                col, row = m.to_cell(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t); col -= c0; row -= r0
                if not (0 <= row < trav.shape[0] and 0 <= col < trav.shape[1]) or not trav[row, col]: return False
            return True
        out = [pts[0]]; i = 0
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1 and not los(pts[i], pts[j]): j -= 1
            out.append(pts[j]); i = j
        return out, (best[3], best[4])

    def _vel_world(self, p: Pose):
        c, s = math.cos(p.yaw), math.sin(p.yaw); return p.vf * c - p.vl * s, p.vf * s + p.vl * c

    def _hover(self, p: Pose, dt: float, yaw: float = 0.0, text: str = "") -> Command:
        """Hold the position with the same PC law (a PC-controlled drone must keep sending tilt commands; if they stop, the FC takes over its own hold)."""
        if self.cfg.control != "lqr": return Command(yaw=yaw, text=text)
        if self.hold_ref is None: self.hold_ref = (p.x, p.y); self.lqr.reset(); self.track = None; self.p_ref = None
        pd, rd = self.lqr.step(dt, p.yaw, (p.x, p.y), self.hold_ref, self._vel_world(p), (0.0, 0.0), (p.wp, p.wr))
        return Command(deg_to_stick(pd), deg_to_stick(rd), yaw, text, lateral=True)

    # ---------- one control step
    def step(self, now: float, sim_t: float, dt: float, p: Pose) -> Command:
        """now = wall time (depth ages), sim_t / dt = simulation time (speed profile), p = the FC pose. Returns the sticks to send."""
        self.pose = p
        if not self.trail or math.hypot(p.x - self.trail[-1][0], p.y - self.trail[-1][1]) > 0.12: self.trail.append((p.x, p.y)); self.trail = self.trail[-1500:]         # where the drone has physically been
        if self.stage not in ("scan", "explore"): return Command(done=self.stage == "finished", reason="finished")
        if p.age > self.cfg.max_pose_age: self.prof.v = 0.0; return Command(text=f"STOP: FC position is {p.age:.1f} s old")
        if sim_t - self.sim0 > self.cfg.max_time: return self._finish("time limit")
        if self.stage == "scan":
            self.scan_acc += abs(wrap(p.yaw - self.prev_yaw)); self.prev_yaw = p.yaw
            if self.scan_acc >= 2 * math.pi * 0.97 and self.map.frames >= self.cfg.min_map_frames: self.stage = "explore"; self.explore_t0 = sim_t; return self._hover(p, dt, 0.0, f"scan done, {int(self.map.occupied().sum())} obstacle cells")
            rate = 0.35 if now - self.cols_t < 1.0 else 0.12                                                                  # slow depth (a slow link): turn slowly so the map can keep up
            return self._hover(p, dt, rate, f"scanning {math.degrees(self.scan_acc):.0f}/360 deg, map updates {self.map.frames}")
        # explore
        if self.waiting_goal is not None and now >= self.wait_until: self.blacklist.append(self.waiting_goal); self.waiting_goal = None      # the map had its time: whatever is still unknown there will not be revealed from here
        if now < self.wait_until: return self._hover(p, dt, 0.25, "reached a goal: looking around while the map updates")
        if now < self.retreat_until and self.cfg.control == "lqr": return Command(deg_to_stick(-3.0), 0.0, 0.0, "retreating from a wall", lateral=True)
        if now - self.last_plan > self.cfg.replan_s or not self.path:
            self.last_plan = now; r = self.plan(p, self.relax)
            if r == "wait": return self._hover(p, dt, 0.0, "waiting for the map")
            if isinstance(r, tuple) and r[0] == "escape":                                  # out of the safety margin first: a short path to clear space, then plan again
                self.escaping = True; self.path = r[1][1:] or r[1]; self.goal = None; self.hold_ref = None
                self.track = PathTrack([(p.x, p.y)] + self.path); self.s_ref = 0.0; self.p_ref = None; r = "escaping"
            elif r == "boxed":
                self.boxed_since = self.boxed_since or sim_t; self.relax = min(2, self.relax + 1); self.path = []
                self.boxed_xy = self.boxed_xy or (p.x, p.y); self.relax_until = sim_t + 30.0
                if self.blacklist and now - self.last_unblack > 10.0: self.blacklist.clear(); self.last_unblack = now
                return self._hover(p, dt, 0.35, f"boxed in ({sim_t - self.boxed_since:.0f} s): turning to look, margins relaxed; {self.plan_info}")
            else:
                self.escaping = False
                if self.boxed_xy is not None and math.hypot(p.x - self.boxed_xy[0], p.y - self.boxed_xy[1]) > 1.0:
                    self.boxed_since = None; self.boxed_xy = None                     # really left the boxed-in spot
            if r == "escaping": r = (self.path, self.path[-1])
            if (r == "none" or r is None) and self.blacklist and now - self.last_unblack > 20.0:      # nothing left except places given up on: the map has changed since, so try them again before calling the house explored
                self.blacklist.clear(); self.last_unblack = now; self.no_front = 0; return self._hover(p, dt, 0.25, "nothing left: retrying the places given up on")
            if r == "none" or r is None:
                if self.no_front_since is None: self.no_front_since = sim_t
                self.relax = min(2, (self.no_front + 1) // 2)                      # nothing reachable: ignore more speckle and shrink the safety margin step by step
            if (r == "none" or r is None) and (self.map.frames < self.cfg.min_map_frames or sim_t - self.explore_t0 < self.cfg.min_explore_s):
                self.path = []; return self._hover(p, dt, 0.12, f"map too small to judge ({self.map.frames} updates): waiting")
            if r == "none" or r is None:
                self.no_front += 1; self.path = []
                if self.no_front >= self.cfg.no_front_limit and sim_t - self.no_front_since >= 25.0: return self._finish("no frontier left" if r == "none" else "pose left the map")
                return self._hover(p, dt, 0.35, f"no frontier yet ({sim_t - self.no_front_since:.0f} s): turning to look; {self.plan_info}")
            self.no_front = 0; self.no_front_since = None; path, goal = r
            if not self.escaping and sim_t > self.relax_until and self.boxed_xy is None: self.relax = 0      # (was: reset on every path, so it thrashed boxed <-> relaxed)
            if self.goal is None or math.hypot(goal[0] - self.goal[0], goal[1] - self.goal[1]) > 1.0: self.goal_t = now
            self.path, self.goal = path, goal; self.hold_ref = None
            newtrack = PathTrack([(p.x, p.y)] + path); self.s_ref = newtrack.project(self.p_ref) if self.p_ref is not None else 0.0; self.track = newtrack      # keep the reference where it was along the new path
            if now - self.goal_t > self.cfg.goal_timeout:
                self.blacklist.append(goal); t = f"goal ({goal[0]:.1f},{goal[1]:.1f}) not reached in {self.cfg.goal_timeout:.0f} s: blacklisted"; self.goal = None; self.path = []; return self._hover(p, dt, 0.0, t)
        while len(self.path) > 1 and math.hypot(self.path[0][0] - p.x, self.path[0][1] - p.y) < 0.3: self.path.pop(0)
        if len(self.path) == 1 and math.hypot(self.path[0][0] - p.x, self.path[0][1] - p.y) < 0.35:
            if self.goal is not None: self.waiting_goal = self.goal; self.wait_until = now + self.cfg.reach_wait_s            # look around first; it is blacklisted only after the map has had time to change
            self.path = []; self.goal = None; return self._hover(p, dt, 0.25, "goal reached")
        look = next((q for q in self.path if math.hypot(q[0] - p.x, q[1] - p.y) >= 0.5), self.path[-1])
        ys, err = yaw_stick(math.atan2(look[1] - p.y, look[0] - p.x), p.yaw)
        d_end = sum(math.dist(self.path[i], self.path[i + 1]) for i in range(len(self.path) - 1)) + math.hypot(self.path[0][0] - p.x, self.path[0][1] - p.y)
        v_t = min(self.cfg.cruise, math.sqrt(2 * self.cfg.accel * max(0.0, d_end - 0.15))) if abs(err) < 0.35 else 0.0       # rotate first; brake to stop at the goal
        cl = self.clearance(p, now); back = False
        if cl is None: v_t = 0.0                                                                                          # no fresh depth: hold still
        else:
            C, L, R = cl; v_t *= float(np.clip((C - self.cfg.stop_m) / (self.cfg.slow_m - self.cfg.stop_m), 0.0, 1.0)); back = C < self.cfg.back_m
            pinned = C < self.cfg.stop_m and min(L, R) < 0.3                                                                  # squeezed: the way ahead is blocked and a wall is close on a side
            self.pinned_t = (self.pinned_t or now) if pinned else None
            if self.pinned_t and now - self.pinned_t > 2.5:                                                                   # give this goal up, back away from the wall
                if self.goal is not None: self.blacklist.append(self.goal)
                self.path = []; self.goal = None; self.pinned_t = None; self.retreat_until = now + 1.5
            if self.nearest is not None and now - self.nearest_t < self.cfg.max_depth_age and self.nearest < 0.7: v_t = 0.0; back = back or self.nearest < 0.45        # Nimbus obstacle_detected inside 0.7 m: stop, and back off when very close
            if back and not self.backing:                                                                                     # a new back-off
                self.backing = True; self.back_n = (self.back_n + 1) if self.back_goal == self.goal else 1; self.back_goal = self.goal
                if self.back_n >= 2 and self.goal is not None:                                                                # the second one on the way to the same goal: blocked here, give the goal up
                    self.blacklist.append(self.goal); self.path = []; self.goal = None; self.back_n = 0; self.backing = False; self.prof.v = 0.0; self.prof.a = 0.0
                    return self._hover(p, dt, 0.0, "backed off twice on the way to this goal: blacklisted, replanning")
            if self.backing:
                near = self.nearest if (self.nearest is not None and now - self.nearest_t < self.cfg.max_depth_age) else C
                if min(C, near) >= 0.8: self.backing = False                                                                  # hysteresis: forward again only with 0.8 m clear (the stop acts at 0.7 m)
                else: v_t = 0.0
            if C < self.cfg.stop_m and ys == 0.0 and abs(err) < 0.35: ys = 0.3 * (1.0 if L > R else -1.0)
            self.blocked_t = (self.blocked_t or now) if v_t < 0.02 and abs(err) < 0.35 else None
            if self.blocked_t and now - self.blocked_t > 4.0:                                                              # blocked by something the map does not show (glass, a wall seen through a gap): learn it
                self.map.mark_obstacle(p.x, p.y, p.yaw, C, 1.2)
                if self.goal is not None: self.blacklist.append(self.goal)
                self.path = []; self.goal = None; self.blocked_t = None
        v_ref = self.prof.step(v_t, dt)
        if v_ref > 0.12 and abs(p.vf) < 0.05:                                                                              # commanded to move, not moving: stuck on something the depth does not show
            self.stall_t = self.stall_t or now
            if now - self.stall_t > 5.0:
                self.map.mark_obstacle(p.x, p.y, p.yaw, 0.25, 1.2)
                if self.goal is not None: self.blacklist.append(self.goal)
                self.path = []; self.goal = None; self.stall_t = None; self.prof.v = 0.0; self.prof.a = 0.0
                return self._hover(p, dt, 0.0, "STALL: not moving although commanded; obstacle marked, replanning")
        else: self.stall_t = None
        text = f"pos ({p.x:.1f},{p.y:.1f}) v {p.vf:.2f}/{v_ref:.2f} m/s goal {None if self.goal is None else (round(self.goal[0], 1), round(self.goal[1], 1))} path {len(self.path)} clear {None if cl is None else round(cl[0], 2)} frames {self.map.frames}"
        if self.cfg.control == "lqr" and self.track is not None and (self.backing or abs(err) >= 0.35):
            self.s_ref = self.track.project((p.x, p.y)); self.p_ref = None                                                   # the reference stays at the drone while backing off: no forward surge afterwards
        if self.cfg.control == "lqr" and self.track is not None:
            if self.p_ref is None or math.hypot(self.p_ref[0] - p.x, self.p_ref[1] - p.y) < 0.6: self.s_ref = min(self.s_ref + v_ref * dt, self.track.total)     # the reference waits for the drone when it lags
            self.p_ref, tan = self.track.at(self.s_ref); vr = (v_ref * tan[0], v_ref * tan[1]) if self.s_ref < self.track.total - 1e-6 else (0.0, 0.0)
            pd, rd = self.lqr.step(dt, p.yaw, (p.x, p.y), self.p_ref, self._vel_world(p), vr, (p.wp, p.wr))
            if back: pd = -3.0 if p.vf > -0.15 else 0.0                                                                   # too close: tilt back to brake and retreat, at most 0.15 m/s backwards
            if p.vf > 1.5 * self.cfg.cruise: pd = min(pd, 0.0)                                                              # far above the commanded speed: never push forward
            side = self.map_side_clearance(p)                                                                                # 2026-11-21: walls BESIDE the drone from the map (the camera only looks ahead)
            dl = min(cl[1] if cl is not None else 9.0, side[0] if side else 9.0); dr = min(cl[2] if cl is not None else 9.0, side[1] if side else 9.0)
            if min(dl, dr) < 0.45 and abs(dl - dr) > 0.1: rd = float(np.clip(rd + (1.5 if dl < dr else -1.5) * (0.45 - min(dl, dr)) / 0.25, -self.cfg.max_tilt_deg, self.cfg.max_tilt_deg))
            if dr < 0.40: rd = min(rd, 0.0)                                                                                 # never tilt TOWARDS a wall closer than 0.4 m (roll + = right)
            if dl < 0.40: rd = max(rd, 0.0)
            return Command(deg_to_stick(pd), deg_to_stick(rd), ys, text, lateral=True)
        ev = v_ref - p.vf; self.vint = float(np.clip(self.vint + ev * dt, -0.25, 0.25)) if v_ref > 0.02 else 0.0
        pitch = -0.2 if back else float(np.clip(self.cfg.kff * v_ref + self.cfg.kp * ev + self.cfg.ki * self.vint, -0.8, 0.8))
        roll = float(np.clip(0.4 * p.vl, -0.3, 0.3))                                                                       # cancel sideways drift (stick + = right)
        if cl is not None:
            if cl[1] < 0.45: roll = min(0.3, roll + 0.25)
            if cl[2] < 0.45: roll = max(-0.3, roll - 0.25)
        return Command(pitch, roll, ys, text)

    def _finish(self, why: str) -> Command:
        self.stage = "finished"; return Command(done=True, reason=why, text=f"finished: {why}")
