#!/usr/bin/env python3
"""Real-drone explorer (2026-11-25): nimbus_core/roam.py on the REAL map pipeline, started by ../../../map.sh with the rest of the mapping.

Inputs (all from realmap/o4_map_node.py and RTAB-Map, no simulator, no truth topic, no simulator clock):
  /odom                              Kalman odometry from the FC's flow / range / attitude + the SuperPoint-LightGlue fixes (body velocities in twist.linear x forward, y left)
  TF map -> odom                     RTAB-Map's loop-closure correction
  /map                               RTAB-Map 2D occupancy grid
  /droneforge/perception/obstacles   Nimbus depth grid (6 x 8, the 0.2-1.0 m obstacle test)
  /droneforge/fc/attitude            roll / pitch (rad) from the FC telemetry (the tilt the depth projection needs)
Output: stick wishes on udp 127.0.0.1:5766 to the GCS (nimbus_core/steering.py). The GCS applies them ONLY while EXPLORE is on and the FC holds the height (ALT HOLD, tilt capped at 0.40 =
10.9 deg); take-off, landing, SAFE and the height stay with the GCS. The GCS answers every packet with {"explore": bool}: the explorer stays parked (centred sticks) until that is true.

Safety of its own: if the pose is older than 0.5 s, the depth answer older than 1.5 s, or no map has arrived, it sends CENTRED sticks (the FC holds) and says why; after 3 s of that it parks and
tells the GCS {"done": true, "reason": ...} so EXPLORE switches off. It never arms, flies or lands anything.
Run: python3 real_explore.py [--cruise 0.25] [--max-tilt 10] ...   (needs the sourced ROS 2 Jazzy environment)"""
import argparse
import json
import math
import os
import socket
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__)); DF = os.path.normpath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(DF, "software", "pc"))
from nimbus_core.grid_map import GridMap                                    # noqa: E402
from nimbus_core.roam import Pose, RoamConfig, Roamer                       # noqa: E402

FC_STALE_S = 0.5            # the pose
OBS_STALE_S = 1.5           # the depth answer (the depth model answers 10-16 Hz on a GPU; ~0.3 Hz on a CPU laptop: do not explore on that)
ATT_STALE_S = 1.0
LOST_PARK_S = 3.0           # inputs missing this long: park and tell the GCS
STATE_DIR = os.path.expanduser("~/.local/state/droneforge/map")


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def make_config(a) -> RoamConfig:
    return RoamConfig(cruise=a.cruise, accel=a.accel, jerk=a.jerk, kff=a.kff, kp=a.kp, ki=a.ki, stop_m=a.stop_m, slow_m=a.slow_m, back_m=a.back_m, inflate=a.inflate, replan_s=a.replan_s,
                      goal_timeout=a.goal_timeout, max_time=a.max_time, control="lqr", plant_path=os.path.join(DF, "gazebo", "config", "plant.json"), max_tilt_deg=a.max_tilt)


class RealCore:
    """The explorer's logic without ROS: feed it messages, call step(now). (The ROS node below only forwards topics to it, so it can be tested without a robot.)"""

    def __init__(self, a, send, clock=time.time) -> None:
        self.a = a; self.send_fn = send; self.clock = clock
        self.roam = Roamer(make_config(a), GridMap())
        self.fc = None; self.fc_t = 0.0; self.obs_t = 0.0; self.att = None; self.map_t = None
        self.stage = "wait"; self.gcs_on = False; self.gcs_t = 0.0; self.last_say = 0.0; self.lost_since = None; self.why = ""
        self.t_roam0 = None; self.dist = 0.0; self.last_xy = None; self.last_step = None; self.said_att = False; self.log = []

    # ---- inputs
    def on_fc(self, x: float, y: float, yaw: float, vf: float, vl: float, now: float = None) -> None:
        now = self.clock() if now is None else now
        self.fc = (x, y, yaw, vf, vl); self.fc_t = now; self.roam.push_pose(now, x, y, yaw)
        if self.stage == "roam":
            if self.last_xy is not None: self.dist += math.dist(self.last_xy, (x, y))
            self.last_xy = (x, y)

    def on_att(self, roll: float, pitch: float, now: float = None) -> None:
        self.att = (float(roll), float(pitch), self.clock() if now is None else now)

    def on_map(self, w: int, h: int, res: float, ox: float, oy: float, data, now: float = None) -> None:
        self.roam.map.set_grid(w, h, res, ox, oy, data); self.map_t = self.clock() if now is None else now

    def on_obs(self, o: dict, now: float = None) -> None:
        now = self.clock() if now is None else now
        self.obs_t = now; self.roam.on_depth(o, self.tilt(now), now)

    def on_gcs(self, answer: dict, now: float = None) -> None:
        self.gcs_on = bool(answer.get("explore")); self.gcs_t = self.clock() if now is None else now

    def tilt(self, now: float) -> float:
        if self.att is None or now - self.att[2] > ATT_STALE_S:
            return 0.0
        return max(abs(self.att[0]), abs(self.att[1]))

    # ---- output
    def say(self, s: str) -> None:
        self.log.append(s)
        print(f"[real_explore] {s}", flush=True)

    def missing(self, now: float) -> str:
        """Why it cannot steer right now ('' = everything is fresh)."""
        if self.fc is None or now - self.fc_t > FC_STALE_S: return "no fresh pose (/odom older than %.1f s)" % FC_STALE_S
        if now - self.obs_t > OBS_STALE_S: return "no fresh depth (the obstacle answer is older than %.1f s: depth model too slow or stopped)" % OBS_STALE_S
        if self.map_t is None: return "no map yet (RTAB-Map has not published /map)"
        return ""

    def step(self, now: float = None) -> None:
        now = self.clock() if now is None else now
        dt = 0.05 if self.last_step is None else min(max(now - self.last_step, 0.01), 0.2); self.last_step = now
        if now - self.gcs_t > 1.0: self.gcs_on = False                       # the GCS stopped answering: not listening
        miss = self.missing(now)
        if self.stage in ("wait", "parked"):
            self.send_fn({"roll": 0.0, "pitch": 0.0, "yaw": 0.0})            # heartbeat, centred sticks
            if self.gcs_on:
                if miss:
                    if now - self.last_say > 2.0: self.last_say = now; self.say("EXPLORE is on but I cannot start: " + miss)
                    return
                (self.roam.restart if (self.stage == "parked" or self.roam.stage == "finished") else self.roam.begin)(now, now, self.fc[2])
                self.stage = "roam"; self.t_roam0 = now; self.lost_since = None; self.dist = 0.0; self.last_xy = (self.fc[0], self.fc[1])
                self.say("EXPLORE is on: roaming (take-off, landing and SAFE stay with the GCS)")
            elif now - self.last_say > 5.0:
                self.last_say = now; self.say("parked: press EXPLORE in the GCS for a fresh search" if self.stage == "parked" else "waiting: take off in the GCS and press EXPLORE" + (f" (not ready: {miss})" if miss else ""))
            return
        if not self.gcs_on:                                                  # EXPLORE off in the GCS: pause (the FC holds the position)
            self.stage = "wait"; self.say("EXPLORE is off: paused (the FC holds); press EXPLORE to continue"); return
        if miss:                                                             # an input went stale mid-flight: centred sticks at once, park after LOST_PARK_S
            self.send_fn({"roll": 0.0, "pitch": 0.0, "yaw": 0.0})
            self.lost_since = self.lost_since if self.lost_since is not None else now
            if now - self.last_say > 1.0: self.last_say = now; self.say("STOPPED STEERING: " + miss)
            if now - self.lost_since > LOST_PARK_S:
                self.park(f"inputs lost: {miss}")
            return
        self.lost_since = None
        x, y, yaw, vf, vl = self.fc
        c = self.roam.step(now, now, dt, Pose(x, y, yaw, vf, vl, self.tilt(now), now - self.fc_t, 0.0, 0.0))
        self.send_fn({"roll": c.roll, "pitch": c.pitch, "yaw": -c.yaw})        # Betaflight: yaw + = clockwise, the roamer's yaw is counter-clockwise
        if now - self.last_say > 2.0 and c.text: self.last_say = now; self.say("  " + c.text)
        if c.done:
            self.park(c.reason)

    def park(self, reason: str) -> None:
        self.why = reason; self.stage = "parked"; self.gcs_on = False
        self.send_fn({"done": True, "reason": reason}); self.save_summary()
        self.say(f"parked: {reason}")

    def save_summary(self) -> None:
        try:
            os.makedirs(STATE_DIR, exist_ok=True)
            s = {"stopped": self.why, "odometry_distance_m": round(self.dist, 1), "seconds": None if self.t_roam0 is None else round(self.clock() - self.t_roam0), "map_updates": getattr(self.roam.map, "frames", None)}
            open(os.path.join(STATE_DIR, "last_explore_summary.json"), "w").write(json.dumps(s, indent=1))
            import cv2
            p = self.roam.pose; pose = (p.x, p.y, p.yaw) if p else None
            cv2.imwrite(os.path.join(STATE_DIR, "last_explore_map.png"), self.roam.map.render(pose, None, None, scale=8))
        except Exception as e:                                               # a failed summary must never stop the explorer
            print(f"[real_explore] summary not saved: {e}", flush=True)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Real-drone explorer (stick wishes to the GCS, EXPLORE button)")
    p.add_argument("--cruise", type=float, default=0.25, help="m/s; the Nimbus waypoint speed is 0.25, the simulator explorer used 0.35")
    p.add_argument("--accel", type=float, default=0.35); p.add_argument("--jerk", type=float, default=0.8)
    p.add_argument("--kff", type=float, default=1.5); p.add_argument("--kp", type=float, default=1.0); p.add_argument("--ki", type=float, default=0.6)
    p.add_argument("--stop-m", dest="stop_m", type=float, default=0.6); p.add_argument("--slow-m", dest="slow_m", type=float, default=1.3); p.add_argument("--back-m", dest="back_m", type=float, default=0.35)
    p.add_argument("--inflate", type=float, default=0.25); p.add_argument("--replan-s", dest="replan_s", type=float, default=0.8); p.add_argument("--goal-timeout", dest="goal_timeout", type=float, default=60)
    p.add_argument("--max-time", dest="max_time", type=float, default=600)
    p.add_argument("--max-tilt", dest="max_tilt", type=float, default=10.0, help="deg; the GCS caps the sticks at 0.40 = 10.9 deg, so ask for no more")
    p.add_argument("--gcs-port", dest="gcs_port", type=int, default=5766)
    return p


def main() -> None:
    import rclpy
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener
    a = build_parser().parse_args()

    class RealExplorer(Node):
        def __init__(self) -> None:
            super().__init__("droneforge_real_explore")
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); self.sock.setblocking(False); self.addr = ("127.0.0.1", a.gcs_port)
            self.core = RealCore(a, self.to_gcs)
            self.tfb = Buffer(); self.tfl = TransformListener(self.tfb, self)
            self.create_subscription(Odometry, "/odom", self.on_odom, 10)
            self.create_subscription(String, "/droneforge/perception/obstacles", self.on_obs, 10)
            self.create_subscription(String, "/droneforge/fc/attitude", self.on_att, 10)
            self.create_subscription(OccupancyGrid, "/map", self.on_map, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
            self.create_timer(0.05, self.step)
            self.get_logger().info("real explorer ready: parked until EXPLORE is pressed in the GCS (sticks -> udp %d)" % a.gcs_port)

        def map_pose(self, x, y, yaw):
            try:
                t = self.tfb.lookup_transform("map", "odom", rclpy.time.Time()).transform; c = yaw_of(t.rotation); cy, sy = math.cos(c), math.sin(c)
                return t.translation.x + cy * x - sy * y, t.translation.y + sy * x + cy * y, yaw + c
            except Exception:
                return x, y, yaw

        def on_odom(self, m) -> None:
            p = m.pose.pose.position; x, y, yaw = self.map_pose(p.x, p.y, yaw_of(m.pose.pose.orientation))
            self.core.on_fc(x, y, yaw, m.twist.twist.linear.x, m.twist.twist.linear.y)

        def on_obs(self, m) -> None:
            try: self.core.on_obs(json.loads(m.data))
            except ValueError: pass

        def on_att(self, m) -> None:
            try: d = json.loads(m.data); self.core.on_att(d.get("roll", 0.0), d.get("pitch", 0.0))
            except ValueError: pass

        def on_map(self, m) -> None:
            self.core.on_map(m.info.width, m.info.height, m.info.resolution, m.info.origin.position.x, m.info.origin.position.y, m.data)

        def to_gcs(self, d: dict) -> None:
            try: self.sock.sendto(json.dumps(d).encode(), self.addr)
            except OSError: pass
            while True:                                                       # the GCS answers every packet: {"explore": bool}
                try: self.core.on_gcs(json.loads(self.sock.recv(512).decode()))
                except (BlockingIOError, OSError, ValueError): break

        def step(self) -> None:
            self.core.step()

    rclpy.init(); n = RealExplorer()
    from rclpy.executors import ExternalShutdownException
    try: rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException): pass          # Ctrl+C / map.sh stop: a clean exit, no traceback
    n.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    main()
