#!/usr/bin/env python3
"""DroneForge Flight Controller Bridge (Betaflight v4.5 firmware core)
Simulates on-board Matek H743 running Betaflight v4.5 height hold and position hold control laws.
Uses the compiled C library libhh.so directly from the firmware repository.
Accepts commands via ROS topic (/droneforge/fc/command), OpenCV viewer, and terminal console.
"""

import argparse
import ctypes
import json
import math
import os
import random
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from actuator_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String

# The FC firmware's hold laws, compiled from the firmware source (tools_scripts/fc_hold_sim/build.sh). DF_FW picks the build (2026-11-21):
#   v47  = v4.7 ST (default; built, NOT flashed): stick-through, brake-then-hold, trim kept, PC visual-odometry position aid
#   rev1 = v4.6-rev1 (FLASHED on the drone, not yet flown); the last FLOWN build is v4.6
HERE = os.path.dirname(os.path.abspath(__file__))
DF_FW = os.environ.get("DF_FW", "v47")
SO_PATH = os.path.join(HERE, {"v47": "libhh_v47.so", "v471": "libhh_v471.so", "v472": "libhh_v471.so", "v473r": "libhh_v473R.so", "rev1": "libhh_v46rev1.so"}.get(DF_FW, "libhh.so"))      # v471 = v4.7.1 (2026-11-24: height-change feed-forward)
if not os.path.exists(SO_PATH):
    SO_PATH = os.path.expanduser("~/Music/DroneForge/tools_scripts/fc_hold_sim/libhh.so")


def stick_to_deg(s_: float) -> float:
    """Angle mode: stick -> tilt in degrees with the Nimbus mapping  deg = 60 * (70|s| + 600 s^2) / 670  (the GCS side uses the inverse)."""
    s_ = max(-1.0, min(1.0, s_)); return math.copysign(60.0 * (70.0 * abs(s_) + 600.0 * s_ * s_) / 670.0, s_)


class FcBridgeNode(Node):
    def __init__(self, auto_takeoff=False):
        super().__init__("droneforge_fc_bridge", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, os.environ.get("DF_SIM_TIME", "1") != "0")])   # the control loop ticks on SIMULATION time (/clock): a slow machine then changes only the speed, not the controller dynamics
        self.auto_takeoff = auto_takeoff
        self.start_time = time.time()
        self.auto_stage = 0

        # Load pure-C flight controller logic
        if not os.path.exists(SO_PATH):
            self.get_logger().error(f"Cannot find libhh.so at {SO_PATH}")
            sys.exit(1)

        self.lib = ctypes.CDLL(SO_PATH)
        self.lib.hh_defaults()
        # Simulation tuning of the firmware core's take-off parameters (the firmware source and the real drone are NOT changed): the stock values (climb governor 0.30 m/s, hand-over 5 cm
        # below the target) enter the closed height loop still climbing and overshoot by 7-10 cm. DF_FW_DEFAULTS=1 keeps the stock values.
        self.lib.hh_params.restype = ctypes.POINTER(ctypes.c_float)
        hp = self.lib.hh_params()                                      # the library may have more fields appended (36 in the v4.6 build): the two we touch are identified by their stock values
        # 2026-11-21: the sim runs the firmware's OWN take-off values by default (it is the proof before the port); DF_SIM_TAKEOFF_TUNING=1 brings back the old sim-only tuning
        if os.environ.get("DF_SIM_TAKEOFF_TUNING") == "1" and self.lib.hh_nparams() >= 35 and abs(hp[32] - 0.30) < 1e-3 and abs(hp[34] - 0.05) < 1e-3:
            hp[32] = float(os.environ.get("DF_TAKEOFF_VZ_MAX", "0.15")); hp[34] = float(os.environ.get("DF_TAKEOFF_MARGIN", "0.10"))      # takeoff_vz_max, takeoff_handover_margin
            self.get_logger().info(f"take-off tuning: climb governor {hp[32]:.2f} m/s, hand-over {hp[34]:.2f} m below the target (DF_FW_DEFAULTS=1 for the stock values)")
        self.lib.xy_defaults()

        # Set up C types
        self.lib.hh_takeoff.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float]
        self.lib.hh_update.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float]
        self.lib.hh_update.restype = ctypes.c_float
        self.lib.hh_land.restype = None
        self.lib.hh_mode.restype = ctypes.c_int
        self.lib.hh_landed.restype = ctypes.c_int

        self.lib.xy_update.argtypes = [
            ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float,
            ctypes.c_int, ctypes.POINTER(ctypes.c_float)
        ]
        self.lib.xy_reposition.restype = None
        self.v47 = hasattr(self.lib, "xy_update_st")
        if self.v47:
            self.lib.xy_update_st.argtypes = [ctypes.c_float] * 4 + [ctypes.c_int, ctypes.c_float, ctypes.c_float, ctypes.c_int, ctypes.c_float, ctypes.c_float, ctypes.POINTER(ctypes.c_float)]
        self.xy_paused = False; self.prev_hold = 0; self.aid = None; self.aid_t = 0.0
        self.get_logger().info(f"FC firmware hold laws: {os.path.basename(SO_PATH)} ({'v4.7 ST' if self.v47 else 'v4.6-rev1 (flashed)'})")
        self.out_xy = (ctypes.c_float * 2)()

        # Vehicle State
        self.z = 0.03
        self.vz = 0.0
        self.vx_body = 0.0
        self.vy_body = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.gyro = [0.0, 0.0, 0.0]  # p, q, r
        self.range_m = 0.03

        # Battery simulation (4S LiPo, 16.2V full, nominal sag ~13.6V under load)
        self.vbat = 16.2
        self.f_hover_ref = 0.62  # 382g 4S quad hovers at ~62% throttle
        self.target_height = 0.50

        # Flight Mode: DISARMED, ARMED, TAKEOFF, ALT_HOLD, POS_HOLD, LANDING
        self.mode = "DISARMED"
        self.pos_hold_enabled = True
        self.target_yaw = 0.0
        self.motor_outputs = [0.0, 0.0, 0.0, 0.0]

        # Inner-loop attitude controller gains (Betaflight Angle Mode emulation)
        self.kp_angle = 2.0
        self.kd_rate = 0.18
        self.kp_yaw = 0.6
        self.kd_yaw = 0.05

        # Slew-filtered angle commands

        # Manual Stick Control Overlays
        self.stick_pitch = 0.0
        self.stick_roll = 0.0
        self.stick_yaw = 0.0
        self.prev_stick_active = False
        self.stick_throttle = 0.0

        # Motor constants
        self.max_omega = 1000.0  # rad/s max motor speed
        self.idle_omega = 200.0   # rad/s idle spin

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.pc_until = 0.0; self.pc_prev = False
        # --- the REAL drone's imperfections (2026-11-21, owner: "always use the drone spec we have"); DF_SIM_IDEAL=1 = the old perfect sensors
        self.ideal = os.environ.get("DF_SIM_IDEAL") == "1"; self.rng = random.Random(int(os.environ.get("DF_SIM_SEED", "7")))
        lv = [float(v) for v in os.environ.get("DF_LEVEL_ERR", "1.0,0.0").split(",")]                       # deg roll, pitch: the FC's level reference vs true level (logs: 0.75-1 deg, drift right ~0.17 m/s^2)
        self.level_err = (0.0, 0.0) if self.ideal else (math.radians(lv[0]), math.radians(lv[1]))
        self.flow_sigma = 0.081; self.flow_scale = 1.0 + (0.0 if self.ideal else self.rng.gauss(0.0, 0.03)); self.flow_delay = 0.05; self.flow_dt = 1.0 / 25.0   # MTF-02P (kf_calib.json: 0.081 m/s)
        self.flow_hist = []; self.t_flow = 0.0; self.v_meas = [0.0, 0.0]; self.v_est = [0.0, 0.0]                                                   # FC EKF velocity (flow, low-passed)
        self.z_meas = 0.03; self.vz_est = 0.0; self.z_prev = None

        # Publishers
        self.pub_motor = self.create_publisher(Actuators, "/droneforge/command/motor_speed", 10)
        self.pub_telem = self.create_publisher(String, "/droneforge/fc/telemetry", 10)

        # Subscribers
        self.create_subscription(Odometry, "/droneforge/odometry", self.on_odom, sensor_qos)
        self.create_subscription(LaserScan, "/droneforge/sensors/rangefinder", self.on_range, sensor_qos)
        self.create_subscription(Imu, "/droneforge/sensors/imu", self.on_imu, sensor_qos)
        self.create_subscription(String, "/droneforge/fc/command", self.on_command, 10)

        # Main 100 Hz control loop timer
        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("DroneForge FC Bridge (Betaflight v4.5) Initialized.")

        # Start terminal console input listener
        self.stdin_thread = threading.Thread(target=self.terminal_listener, daemon=True)
        self.stdin_thread.start()

    def terminal_listener(self):
        time.sleep(1.0)
        print("\n" + "="*60)
        print("  TERMINAL FLIGHT CONTROL (Type letter + Enter):")
        print("    [a] Arm  |  [t] Takeoff (0.50m)  |  [h] Hold  |  [l] Land  |  [d] Disarm")
        print("="*60 + "\n")
        while True:
            try:
                line = sys.stdin.readline()
                if not line:
                    break
                cmd = line.strip().lower()
                if cmd in ("a", "arm"):
                    self.process_command("arm")
                elif cmd in ("t", "takeoff"):
                    self.process_command("takeoff")
                elif cmd in ("h", "hold"):
                    cur = self.mode
                    self.process_command("alt_hold" if cur == "POS_HOLD" else "pos_hold")
                elif cmd in ("l", "land"):
                    self.process_command("land")
                elif cmd in ("d", "disarm"):
                    self.process_command("disarm")
            except Exception:
                break

    def on_range(self, msg: LaserScan):
        self.t_scan = time.time()
        if len(msg.ranges) > 0 and not math.isinf(msg.ranges[0]) and not math.isnan(msg.ranges[0]):
            self.range_m = float(msg.ranges[0])

    def _computed_range(self, z: float) -> None:
        """MTF-02P range without a rendered sensor (the house drone: a gpu_lidar cost the 30 fps camera ~2 fps): the slant distance to a FLAT floor along the tilted
        body axis, sensor 0.015 m below the body origin, 24 Hz, 5 mm noise, 0.02..3.0 m (below 0.02 m it reads the 0.03 m floor value, as on the real drone).
        NOT modelled: furniture under the drone (the real sensor reads the table top)."""
        now = time.time()
        if now - getattr(self, "t_scan", 0.0) < 1.0 or now - getattr(self, "t_rng", 0.0) < 1.0 / 24.0:
            return
        self.t_rng = now
        c = max(0.5, math.cos(self.roll) * math.cos(self.pitch)); d = (z - 0.015) / c + random.gauss(0.0, 0.005)
        self.range_m = 0.03 if d < 0.02 else min(3.0, d)

    def on_imu(self, msg: Imu):
        self.gyro = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]

    def on_odom(self, msg: Odometry):
        # Position & Velocity
        self.z = msg.pose.pose.position.z
        self.vz = msg.twist.twist.linear.z
        self._computed_range(self.z)

        # Orientation (quaternion to Euler)
        q = msg.pose.pose.orientation
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self.roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        self.pitch = math.asin(max(-1.0, min(1.0, sinp)))

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

        # World velocity to body forward/right velocity
        vx_w = msg.twist.twist.linear.x
        vy_w = msg.twist.twist.linear.y
        # gz OdometryPublisher reports the twist in the BODY frame (checked in the house at yaw 180 deg), so no yaw rotation here.
        # (Rotating it by yaw, as before, was only right when the yaw was 0.)
        # y is positive to the LEFT in that frame; the hold core wants right-positive (measured: +roll stick -> y velocity negative).
        self.vx_body = vx_w
        self.vy_body = -vy_w

        # Angular rates
        self.gyro = [msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z]

    def on_command(self, msg: String):
        raw = msg.data.strip()
        if raw.startswith("{"):
            try:
                data = json.loads(raw)
                cmd = data.get("cmd", "")
                if "target_z" in data: self.target_height = float(data["target_z"])        # BEFORE the command: take-off must climb to the height sent with it (it used the old 0.5 m, then the height loop pulled up: a vertical hop)
                if cmd:
                    self.process_command(cmd.lower())
                self.stick_pitch = float(data.get("pitch", 0.0))
                self.stick_roll = float(data.get("roll", 0.0))
                self.stick_yaw = float(data.get("yaw", 0.0))
                if "throttle" in data: self.stick_throttle = max(0.0, min(1.0, float(data["throttle"])))      # throttle stick (sim_radio.py): used only in MANUAL (FC height hold off)
                if "aid" in data:                                                    # v4.7: the PC's visual-odometry position aid (MSP2_DF_POS_AID through sim_radio.py): [forward m, right m, valid]
                    a = data["aid"]; self.aid = (float(a[0]), float(a[1])) if a[2] else None; self.aid_t = time.time()
                if "target_z" in data:
                    self.target_height = float(data["target_z"])
                return
            except Exception:
                pass
        self.process_command(raw.lower())

    def process_command(self, cmd: str):
        self.get_logger().info(f"Command -> {cmd.upper()}")
        if cmd == "arm":
            if self.mode == "DISARMED":
                self.mode = "ARMED"
                self.target_yaw = self.yaw
                print("\n[FC] >>> ARMED: Motors at idle spin. Ready for takeoff.")
        elif cmd == "disarm":
            self.mode = "DISARMED"
            print("\n[FC] >>> DISARMED: Motors stopped.")
        elif cmd == "takeoff":
            if self.mode in ("ARMED", "DISARMED"):
                self.mode = "TAKEOFF"
                self.target_yaw = self.yaw
                self.lib.hh_takeoff(
                    ctypes.c_float(self.f_hover_ref),
                    ctypes.c_float(self.target_height),
                    ctypes.c_float(self.vbat)
                )
                print(f"\n[FC] >>> TAKEOFF ACTIVE: Climbing smoothly to target {self.target_height:.2f} m.")
        elif cmd in ("pos_hold", "hold"):
            if self.mode in ("TAKEOFF", "ALT_HOLD"):
                self.mode = "POS_HOLD"
                self.pos_hold_enabled = True
                self.target_yaw = self.yaw
                print("\n[FC] >>> POS_HOLD: Station-keeping active.")
        elif cmd == "alt_hold":
            if self.mode in ("TAKEOFF", "POS_HOLD"):
                self.mode = "ALT_HOLD"
                self.pos_hold_enabled = False
                print("\n[FC] >>> ALT_HOLD: Altitude hold only.")
        elif cmd == "manual":                                              # FC height hold switched off (AUX5 min): the throttle stick flies, ANGLE mode (sim_radio.py)
            if self.mode in ("ARMED", "TAKEOFF", "ALT_HOLD", "POS_HOLD"):
                self.mode = "MANUAL"; self.target_yaw = self.yaw
                print("\n[FC] >>> MANUAL: height hold off, throttle stick.")
        elif cmd == "land" and self.mode == "MANUAL":                      # failsafe without the height hold: a fixed descending throttle until the floor
            self.mode = "MANUAL_LAND"
        elif cmd == "land":
            if self.mode in ("TAKEOFF", "ALT_HOLD", "POS_HOLD"):
                self.mode = "LANDING"
                self.lib.hh_land()
                print("\n[FC] >>> LANDING: Gentle descent to floor.")

    def control_loop(self):
        dt = self.dt
        now = time.time()

        # Optional Auto-Takeoff sequence
        if self.auto_takeoff:
            elapsed = now - self.start_time
            if self.auto_stage == 0 and elapsed > 2.0:
                self.process_command("arm")
                self.auto_stage = 1
            elif self.auto_stage == 1 and elapsed > 3.5:
                self.process_command("takeoff")
                self.auto_stage = 2

        # Battery discharge simulation
        if self.mode != "DISARMED":
            mean_thr = sum(self.motor_outputs) / 4.0
            self.vbat = max(13.0, self.vbat - (0.002 + 0.012 * mean_thr) * dt)      # ~0.009 V/s at hover: 16.2 -> 14.0 V in ~4 min (a 4S pack; was 4x faster)
        else:
            self.vbat = min(16.2, self.vbat + 0.005 * dt)      # small rebound at rest

        # State machine
        throttle_f = 0.0
        target_roll_deg = 0.0
        target_pitch_deg = 0.0

        if self.mode == "DISARMED":
            self.motor_outputs = [0.0, 0.0, 0.0, 0.0]
            self.send_motors([0.0, 0.0, 0.0, 0.0])
            self.publish_telemetry()
            return

        elif self.mode == "ARMED":
            throttle_f = 0.08  # Idle throttle

        elif self.mode in ("MANUAL", "MANUAL_LAND"):
            throttle_f = self.stick_throttle if self.mode == "MANUAL" else 0.9 * self.f_hover_ref
            if self.mode == "MANUAL_LAND" and self.z < 0.04 and abs(self.vz) < 0.05:
                self.mode = "DISARMED"; self.send_motors([0.0, 0.0, 0.0, 0.0]); self.publish_telemetry(); return

        elif self.mode in ("TAKEOFF", "ALT_HOLD", "POS_HOLD", "LANDING"):
            # In simulation, self.z is the true physical altitude of the drone
            measured_z, meas_vz = self._fc_height(dt)
            throttle_f = float(self.lib.hh_update(
                ctypes.c_float(dt),
                ctypes.c_float(measured_z),
                ctypes.c_float(meas_vz),
                ctypes.c_float(self.vbat)
            ))

            c_mode = self.lib.hh_mode()
            if self.mode == "TAKEOFF" and (c_mode == 0 or (measured_z >= self.target_height - 0.03 and self.vz < 0.15)):
                self.mode = "POS_HOLD" if self.pos_hold_enabled else "ALT_HOLD"
                self.target_yaw = self.yaw
                self.get_logger().info(f"Target altitude reached ({measured_z:.2f} m). Handover to POS_HOLD.")

            # (2026-11-21: the sim-only extra height correction that used to sit here is gone: the firmware law alone holds the height, as on the drone)

            if self.mode == "LANDING" and (self.lib.hh_landed() or (measured_z < 0.04 and self.vz > -0.05)):
                self.mode = "DISARMED"
                self.get_logger().info("Touchdown detected. Disarmed.")
                self.send_motors([0.0, 0.0, 0.0, 0.0])
                self.publish_telemetry()
                return

            # Sideways hold exactly as the firmware's height_hold.c: ALT HOLD (AUX7) pauses it; back in POS HOLD and at the hand-over it re-anchors (v4.7: brake first);
            # the hold terms act once the height hold HOLDS (take-off climb: speed damping, v4.7 also learns the level trim); v4.7 gets the sticks and the PC position aid
            if self.mode == "ALT_HOLD":
                self.xy_paused = True
            else:
                hold = 1 if (self.lib.hh_mode() == 0 and self.mode == "POS_HOLD") else 0
                if self.xy_paused or (hold and not self.prev_hold): self._xy_anchor()
                self.xy_paused = False; self.prev_hold = hold
                vf, vr = self._fc_velocity(dt)
                if self.v47:
                    aid = self.aid if (self.aid is not None and time.time() - self.aid_t < 0.3) else None
                    self.lib.xy_update_st(dt, vf, vr, measured_z, hold, self.stick_pitch, self.stick_roll, 1 if aid else 0, aid[0] if aid else 0.0, aid[1] if aid else 0.0, self.out_xy)
                else:
                    self.lib.xy_update(dt, vf, vr, measured_z, hold, self.out_xy)
                target_roll_deg = float(self.out_xy[0])
                target_pitch_deg = float(self.out_xy[1])

        # ANGLE mode, as Betaflight pid.c: the stick angle PLUS the hold angle (rev1: the hold fights a deflected stick; v4.7 ST: the hold adds nothing while a stick is deflected)
        target_pitch_deg += stick_to_deg(self.stick_pitch)
        target_roll_deg += stick_to_deg(self.stick_roll)

        if abs(self.stick_yaw) > 0.05:
            self.target_yaw += self.stick_yaw * 1.5 * dt
            # keep the yaw target close to the real heading so a fast stick cannot wind the yaw loop up (as a rate-mode FC would not)
            lead = (self.target_yaw - self.yaw + math.pi) % (2.0 * math.pi) - math.pi
            if abs(lead) > 0.35:
                self.target_yaw = self.yaw + math.copysign(0.35, lead)

        # Inner-loop attitude PD (Betaflight Angle Mode emulation)
        roll_target_rad = math.radians(target_roll_deg)
        pitch_target_rad = math.radians(target_pitch_deg)

        err_roll = roll_target_rad - (self.roll - self.level_err[0])      # the FC levels to ITS reference: the true attitude ends up off by the level error
        err_pitch = pitch_target_rad - (self.pitch - self.level_err[1])
        err_yaw = (self.target_yaw - self.yaw + math.pi) % (2.0 * math.pi) - math.pi

        # Clamped attitude authorities (Betaflight mixer safety)
        u_roll = max(-0.15, min(0.15, self.kp_angle * err_roll - self.kd_rate * self.gyro[0]))
        u_pitch = max(-0.15, min(0.15, self.kp_angle * err_pitch - self.kd_rate * self.gyro[1]))
        u_yaw = max(-0.06, min(0.06, self.kp_yaw * err_yaw - self.kd_yaw * self.gyro[2]))

        # Standard Quad-X mixer (props-out: M1 CW, M2 CCW, M3 CCW, M4 CW)
        m1 = throttle_f - u_roll + u_pitch + u_yaw
        m2 = throttle_f - u_roll - u_pitch - u_yaw
        m3 = throttle_f + u_roll + u_pitch - u_yaw
        m4 = throttle_f + u_roll - u_pitch + u_yaw

        raw_motors = [m1, m2, m3, m4]
        clamped_motors = [max(0.0, min(1.0, m)) for m in raw_motors]
        self.motor_outputs = clamped_motors

        # Map throttle fraction [0..1] to angular velocity: omega = omega_max * sqrt(fraction)
        omegas = [self.max_omega * math.sqrt(m) for m in clamped_motors]
        self.send_motors(omegas)
        self.publish_telemetry()

    def _xy_anchor(self):
        """The firmware's re-anchor: v4.7 brakes first and anchors where the drone stopped (xyBrakeThenHold), rev1 anchors at once (xyReposition)."""
        (self.lib.xy_brake if self.v47 else self.lib.xy_reposition)()

    def _fc_height(self, dt):
        """What the FC's height estimate sees: the MTF-02P range (24 Hz, 5 mm, tilt-corrected back to height) and a filtered vertical speed (EKF-like)."""
        if self.ideal: return max(0.01, self.z), self.vz
        c = max(0.5, math.cos(self.roll) * math.cos(self.pitch)); z = max(0.01, self.range_m * c)
        if self.z_prev is None: self.z_prev = z
        a = min(1.0, dt / 0.08); self.vz_est += a * ((self.vz + self.rng.gauss(0.0, 0.03)) - self.vz_est)       # accel + range fusion, ~80 ms
        self.z_meas = z; self.z_prev = z
        return z, self.vz_est

    def _fc_velocity(self, dt):
        """What the FC's xy hold sees: MTF-02P flow velocity (25 Hz, 50 ms old, 0.081 m/s noise, small scale error), low-passed by the EKF (~0.1 s)."""
        if self.ideal: return self.vx_body, self.vy_body
        now = time.time(); self.flow_hist.append((now, self.vx_body, self.vy_body)); self.flow_hist = [h for h in self.flow_hist if now - h[0] < 0.5]
        if now - self.t_flow >= self.flow_dt:
            self.t_flow = now; old = min(self.flow_hist, key=lambda h: abs(h[0] - (now - self.flow_delay)))
            self.v_meas = [self.flow_scale * old[1] + self.rng.gauss(0.0, self.flow_sigma), self.flow_scale * old[2] + self.rng.gauss(0.0, self.flow_sigma)]
        a = min(1.0, dt / 0.10)
        for i in (0, 1): self.v_est[i] += a * (self.v_meas[i] - self.v_est[i])
        return self.v_est[0], self.v_est[1]

    def send_motors(self, omegas):
        msg = Actuators()
        msg.velocity = [float(w) for w in omegas]
        self.pub_motor.publish(msg)

    def publish_telemetry(self):
        telem = {
            "mode": self.mode,
            "z": round(self.z, 3),
            "vz": round(self.vz, 3),
            "vx": round(self.vx_body, 3),
            "vy": round(self.vy_body, 3),
            "roll": round(math.degrees(self.roll), 1),
            "pitch": round(math.degrees(self.pitch), 1),
            "yaw": round(math.degrees(self.yaw), 1),
            "vbat": round(self.vbat, 2),
            "m": [round(m, 2) for m in self.motor_outputs],
            "range": round(self.range_m, 3)
        }
        msg = String()
        msg.data = json.dumps(telem)
        self.pub_telem.publish(msg)


def main(args=None):
    parser = argparse.ArgumentParser(description="DroneForge FC Bridge")
    parser.add_argument("--auto", action="store_true", help="Auto arm and takeoff after launch")
    cli_args, unknown = parser.parse_known_args()

    rclpy.init(args=args)
    node = FcBridgeNode(auto_takeoff=cli_args.auto)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        if rclpy.ok():
            try:
                node.send_motors([0.0, 0.0, 0.0, 0.0])
            except Exception:
                pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
