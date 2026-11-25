"""NimbusCore: owns the serial link (Session), the controls source and the stale/arm/watchdog supervision, and publishes everything on the bus.
Clients (the Qt GCS, later recorder, autonomy, a ZMQ bridge) only subscribe to topics and call the command methods; none of them touches the port."""
from __future__ import annotations

import math
from dataclasses import replace
import time
from typing import Optional

import math as _m

from nimbus_pc.gamepad import GamepadInput
from nimbus_pc.sender import Controls
from nimbus_pc.session import Session
from nimbus_pc.watchdog import NAMES as WATCHDOG_NAMES

from . import topics as T
from . import settings as S
from .attitude_select import AttitudeSelector
from .autonomy import HoverAutonomy
from .bus import TopicBus
from .motor_test import MotorTest
from .radio_fc import RadioFcProbe
from .rc_check import RcChannelCheck, TEST_FIELDS
from .recorder import FlightRecorder
from .throttle_ramp import ThrottleRamp
from .simsession import SimSession
from .bench import BenchPlant
from nimbus_pc.sender import CRSF_MAX
from .safety import SafetyConfig, SafetyMonitor
from .director import FlightDirector
from .link_supervisor import LinkSupervisor
from .notifications import Notifications

SOURCES = ("idle", "sweep", "gamepad", "external")      # "external": an autonomy module (hover controller, mission) feeds set_external_controls()


AID_FRESH_S = 0.25                 # the VO aid goes on the RC channels only while younger than this


class NimbusCore:
    def __init__(self, bus: Optional[TopicBus] = None) -> None:
        self.bus = bus or TopicBus()
        self.notifications = Notifications(self)       # before anything can call event()
        self.sess: Optional[Session] = None
        self.port = ""
        self.source = "idle"
        self.allow_arm = False
        self.watch_optrange = True
        self.gamepad_path = ""
        self.pad: Optional[GamepadInput] = None
        self.last_controls = Controls()
        self.external_controls = Controls()
        self.fc_hold_aux = None                  # (aux5, aux6, throttle01 or None): the FC-side height hold request (target+switch, hover reference) added to every outgoing frame
        self._t0 = time.monotonic()
        self._prev = {"arm": False, "fs": False, "wd": 0, "pad": True, "latched": None}
        self.safety = SafetyMonitor(SafetyConfig())
        self.bus.subscribe(T.TELEMETRY, self._feed_safety)
        self.state_source = "live"
        self.bench: Optional[BenchPlant] = None
        self.probe: Optional[RadioFcProbe] = None
        self.rc_check = RcChannelCheck()        # do all 16 channels reach the FC intact? (nimbus_core/rc_check.py; runs by itself once per radio session, disarmed)
        self._fc_reader = None
        self.bench_scale = 0.4
        self.last_sent = Controls()
        self.att_sel = AttitudeSelector(event=self.event)
        self.autonomy = HoverAutonomy(self)
        self.ramp = ThrottleRamp(self)
        self.link = LinkSupervisor(self)
        self.director = FlightDirector(self)          # the GUI's only flight interface
        self.recorder = FlightRecorder(self)
        from .hold_log import HoldLog
        self.vo_status: dict = {}; self.hold_log = HoldLog(self)       # live hold log (nimbus_core/hold_log.py): what is sent, what the FC reads back, the aid, the FC speed
        self.motor_test = MotorTest(self)               # SETUP > Motor test: hold-to-spin one motor while disarmed

    def _feed_safety(self, msg) -> None:
        name, d = msg
        now = time.monotonic()
        if name == "attitude":
            self.safety.feed_attitude(now, _m.degrees(d["pitch_rad"]), _m.degrees(d["roll_rad"]))
        elif name == "battery":
            self.safety.feed_battery(now, d["voltage_v"])
        elif name == "optrange":
            self.safety.feed_range(now, d["distance_m"])
        elif name == "link":
            self.safety.feed_link(now, d["up_lq"])

    def _publish_frame_in(self, f) -> None:
        """Frame statistics (rates, ages, the sensor pages) count the compact 0xD6 frame as the D3 and the D4 frame it replaces."""
        from nimbus_pc.crsf import TYPE_OPTRANGE, TYPE_MOTOR_PWM, TYPE_STATE, Frame
        if f.type == TYPE_STATE:
            for t in (TYPE_OPTRANGE, TYPE_MOTOR_PWM):
                self.bus.publish(T.FRAME_IN, Frame(f.sync, t, f.payload), cache=False)
            return
        self.bus.publish(T.FRAME_IN, f, cache=False)

    def _on_session_telemetry(self, name, d) -> None:
        """Live telemetry -> bus. Attitude comes from the fresher of the ATTITUDE frame and the D4 EKF frame (see attitude_select); bench mode replaces range/attitude by the simulator's."""
        if name == "state_compact":                          # one compact frame carries what D3 + D4 carried: the rest of the program sees the two usual messages
            from nimbus_pc.crsf import split_state
            for n2, d2 in split_state(d):
                self._on_session_telemetry(n2, d2)
            return
        for n, dd in self.att_sel.feed(name, d):
            if self.bench is not None and n in ("optrange", "attitude"):
                continue
            self.bus.publish(T.TELEMETRY, (n, dd), cache=False)

    # --- helpers -----------------------------------------------------------------------------
    def event(self, text: str) -> None:
        self.bus.publish(T.EVENT, text, cache=False)       # Notifications turns every event line into a notice

    def notify(self, text: str, level: str = "info", source: str = "") -> None:
        self.notifications.add(text, level, source)

    def _publish_source(self) -> None:
        self.bus.publish(T.SOURCE, {"source": self.source, "allow_arm": self.allow_arm})

    @property
    def connected(self) -> bool:
        return self.sess is not None

    # --- connection --------------------------------------------------------------------------
    def connect(self, port: str, hz: float = 100.0, log_path: Optional[str] = None, bench_fc_port: Optional[str] = None, fc=None) -> None:
        """bench_fc_port (or an injected `fc` reader) = BENCH MODE: props OFF, real FC + link, simulated sensors from the FC's motor outputs."""
        self.disconnect()
        self._t0 = time.monotonic()
        self._prev.update(arm=False, fs=False, wd=0, pad=True)
        bus = self.bus
        sess = Session(port, hz=hz, allow_arm=self.allow_arm, ping=True, watch_optrange=self.watch_optrange, log_path=log_path,
                       on_telemetry=self._on_session_telemetry,
                       on_frame=self._publish_frame_in)
        sess.link.on_write = lambda data: bus.publish(T.FRAME_OUT, data, cache=False)
        if bench_fc_port or fc is not None:
            if fc is None:
                from nimbus_pc.hover.hil import FcReader
                fc = FcReader(bench_fc_port)
            self._fc_reader = fc
            self.bench = BenchPlant(fc, scale=self.bench_scale, params=S.plant_params(self.autonomy.settings), on_telemetry=lambda n, d: bus.publish(T.TELEMETRY, (n, d), cache=False))
        sess.start()                                     # raises OSError if the port cannot be opened
        self.sess, self.port = sess, port
        if self.bench is not None:
            self.bench.start()
        else:                                            # live link: read the FC's motor outputs over the radio (read-only MSP, own thread, 2 Hz) for the log
            self.probe = RadioFcProbe(sess, period_s=0.3); self.probe.start(); self.recorder.probe = self.probe
            self.rc_check.reset(); self.rc_check.auto_done = False
        self.state_source = "bench" if self.bench is not None else "live"
        self.bus.publish(T.CONNECTION, {"connected": True, "port": port, "source": self.state_source})
        self.event(f"connected to {port}" + ("  [BENCH MODE: props OFF, sensors simulated from the FC's motor outputs, throttle x%.1f]" % self.bench_scale if self.bench is not None else ""))

    def connect_sim(self, params=None, seed: int = 1, ctrl_delay_s: float = 0.022) -> None:
        """State source = Sim: the physics simulator instead of the radio link (no hardware)."""
        self.disconnect()
        self._t0 = time.monotonic()
        self._prev.update(arm=False, fs=False, wd=0, pad=True)
        bus = self.bus
        params = params or S.plant_params(self.autonomy.settings)           # the Sim plant has this drone's measured numbers
        sess = SimSession(params, seed, ctrl_delay_s=ctrl_delay_s, on_telemetry=lambda name, d: bus.publish(T.TELEMETRY, (name, d), cache=False),
                          on_frame=lambda f: bus.publish(T.FRAME_IN, f, cache=False))
        sess.link.on_write = lambda data: bus.publish(T.FRAME_OUT, data, cache=False)
        sess.sender.allow_arm = self.allow_arm
        if self.autonomy.settings.profile == "fc":                          # the hybrid in the simulator: the firmware's own height-hold law models the FC side
            from .fc_hold_model import FcHoldModel, lib_path
            if lib_path():
                sess.fc_hold = FcHoldModel()
            else:
                self.event("FC hold model not built (tools_scripts/fc_hold_sim/build.sh): the simulator cannot fly the FC loop")
        sess.start()
        self.sess, self.port, self.state_source = sess, "SIM", "sim"
        self.bus.publish(T.CONNECTION, {"connected": True, "port": "SIM", "source": "sim"})
        self.event("connected to the SIMULATOR (state source = Sim)")

    def disconnect(self) -> None:
        self.motor_test.stop("disconnect", blocking=True)
        self.fc_hold_aux = None
        if self.sess:
            self.sess.set_controls(Controls())
            self.autonomy.abort()
            self.recorder.probe = None
            if self.probe is not None:
                self.probe.stop(); self.probe = None
            if self.bench is not None:
                self.bench.stop(); self.bench = None
                try: self._fc_reader.close()
                except Exception: pass
                self._fc_reader = None
            try:
                self.sess.stop()
            finally:
                self.sess = None
                self.bus.publish(T.CONNECTION, {"connected": False, "port": self.port})
                self.event("disconnected")
        if self.pad:
            self.pad.close(); self.pad = None

    # --- commands ----------------------------------------------------------------------------
    def set_source(self, name: str, gamepad_path: str = "") -> None:
        assert name in SOURCES
        if self.pad:
            self.pad.close(); self.pad = None
        self.source = name
        if name == "gamepad":
            self.gamepad_path = gamepad_path or self.gamepad_path
            self.pad = GamepadInput(self.gamepad_path)
        self.event(f"controls source -> {name}")
        self._publish_source()

    def set_allow_arm(self, allow: bool) -> None:
        """Takes effect on the running sender too (allow_arm gates the arm channel in CrsfSender)."""
        self.allow_arm = allow
        if self.sess:
            self.sess.sender.allow_arm = allow
        self.event("arming ALLOWED (props off!)" if allow else "arming disabled")
        self._publish_source()

    def safe_now(self) -> None:
        """Big red button: arm channel low, throttle minimum, source idle, arming disallowed."""
        self.motor_test.stop("SAFE")
        self.fc_hold_aux = None
        self.allow_arm = False
        self.source = "idle"
        if self.pad:
            self.pad.close(); self.pad = None
        if self.sess:
            self.sess.sender.allow_arm = False
            self.sess.set_controls(Controls())
        self.event("SAFE pressed: disarm + throttle min + source idle")
        self._publish_source()

    def set_fc_hold(self, aux5: int, aux6: int, throttle: Optional[float] = None) -> None:
        """Request the FC-side height hold: aux5 = switch + target height (mapper.height_to_aux5, AUX5_LAND to land), aux6 = hover-reference throttle for the FC's take-off,
        throttle = stick throttle to send meanwhile (None: whatever the active controller sends, normally 0). Cleared by clear_fc_hold() and by disarm."""
        self.fc_hold_aux = (int(aux5), int(aux6), throttle)

    def clear_fc_hold(self) -> None:
        self.fc_hold_aux = None

    def set_external_controls(self, c: Controls) -> None:
        self.external_controls = c

    # --- control tick (~50 Hz, driven by the caller: a Qt timer now, the core's own loop later) ---
    def start_rc_check(self) -> bool:
        """Run the RC channel test now (the Diagnostics tab button). Only on the live radio link, with the FC disarmed and no flight in progress."""
        if self.state_source != "live" or self.probe is None or self.autonomy.fc_armed or self.autonomy.busy:
            return False
        self.rc_check.start(); self.event("RC channel test started (pattern on CH9-16, disarmed)")
        return True

    def _rc_channel_test(self, c):
        """Apply the test pattern while the test runs and the FC is disarmed; start it once per radio session by itself; cancel it the moment anything arms."""
        chk = self.rc_check
        if self.probe is None or self.state_source != "live":
            return c
        armed_now = bool(c.arm) or bool(self.autonomy.fc_armed)
        before = chk.state
        chk.feed(self.probe.rc, self.probe.rc_req_t)
        if chk.state == "running" and armed_now:
            chk.cancel("the drone is arming")
        if chk.state == "idle" and not chk.auto_done and not armed_now and not self.autonomy.busy and self.probe.replies >= 2:
            chk.auto_done = True; chk.start(); self.event("RC channel test started (pattern on CH9-16, disarmed)")
        if chk.state != before and chk.state != "running":
            self.event(f"RC channel test: {chk.state.upper()}: {chk.detail}")
        ov = chk.overrides()
        if ov and not armed_now:
            c = replace(c, **ov)
        return c

    def set_pos_aid(self, forward_m: float, right_m: float) -> None:
        """The VO displacement from the hold point (drone axes, m). Sent to the FC on CH13-15 while younger than AID_FRESH_S (firmware v4.7.2)."""
        self.pos_aid = (float(forward_m), float(right_m), time.monotonic())

    def clear_pos_aid(self) -> None:
        self.pos_aid = None

    def tick(self) -> None:
        self.link.update()
        self.director.housekeeping()
        if not self.sess:
            return
        t = time.monotonic() - self._t0
        if self.source == "gamepad" and self.pad:
            self.pad.poll()
            c = self.pad.controls() if (self.pad.connected and self.pad.ready) else Controls()
            if not self.pad.connected and self._prev["pad"]:
                self.event("gamepad LOST -> safe frames")
            self._prev["pad"] = self.pad.connected
        elif self.source == "external":
            if self.ramp.phase in ("arming", "ramping"):
                c = self.ramp.tick(time.monotonic())
            else:
                c = self.autonomy.controls(time.monotonic()) if self.autonomy.phase in ("arming", "armed", "flying") else self.external_controls
        elif self.source == "sweep":
            c = Controls(roll=0.5 * math.sin(2 * t), pitch=0.5 * math.sin(1.4 * t), yaw=0.5 * math.sin(t))
        else:
            c = Controls()
        # --- safety layer: tilt/inverted disarm overrides every source (a land request is for the active controller; pilots get the warning)
        req_arm = bool(c.arm and self.sess.sender.allow_arm)
        if not req_arm:
            self.safety.reset()
        turtle = c.aux3 > 1400                                          # CRSF raw; AUX3 high = flip-over (turtle) mode on this FC
        for ev in self.safety.check(time.monotonic(), armed=req_arm, turtle=turtle):
            self.event(f"SAFETY [{ev.level}] {ev.text}")
            self.bus.publish(T.SAFETY, dict(self.safety.status(), event=ev))
        if self.safety.disarm_latched:
            c = Controls()                                              # arm low, throttle minimum, sticks centred
            if self._prev["latched"] != self.safety.disarm_latched:
                self.event(f"SAFETY: arm forced LOW ({self.safety.disarm_latched}); flip the arm switch off to re-arm")
        self._prev["latched"] = self.safety.disarm_latched
        if self.fc_hold_aux is not None and c.arm and not self.safety.disarm_latched:       # hybrid: the FC runs the height loop, the PC only sends the target and the hover reference
            a5, a6, thr = self.fc_hold_aux
            c = replace(c, aux5=a5, aux6=a6, throttle=c.throttle if thr is None else thr)
        aid = getattr(self, "pos_aid", None)                            # v4.7.2: the VO position aid rides on CH13-15 while it is fresh (vision_bar sets it at 10 Hz)
        if aid is not None and c.arm and not self.safety.disarm_latched and time.monotonic() - aid[2] < AID_FRESH_S:
            from nimbus_pc.mapper import AID_FLAG_TICKS, aid_to_ticks
            c = replace(c, aux9=aid_to_ticks(aid[0]), aux10=aid_to_ticks(aid[1]), aux11=AID_FLAG_TICKS)
        c = self._rc_channel_test(c)
        self.last_controls = c
        send = c
        if self.bench is not None:                                       # props OFF: slow motors, ACRO (no I-term wind-up on the still IMU)
            self.bench.set_controls(c)
            send = Controls(roll=c.roll, pitch=c.pitch, yaw=c.yaw, throttle=(c.throttle * self.bench_scale) if c.arm else 0.0, arm=c.arm,
                            aux2=CRSF_MAX, aux3=c.aux3)
        self.last_sent = send
        if self.probe is not None: self.probe.armed = bool(self.autonomy.fc_armed or send.arm)
        try: self.hold_log.sample(send)
        except Exception as e: self.event(f"hold log stopped: {e}"); self.hold_log.close(); self.hold_log.sample = lambda *_: None          # noqa: BLE001  (a log problem never touches the flight path)
        self.sess.set_controls(send)
        self.bus.publish(T.CONTROLS, c)
        arm = bool(c.arm and self.sess.sender.allow_arm)
        if arm != self._prev["arm"]:
            self.event("arm channel " + ("HIGH" if arm else "low")); self._prev["arm"] = arm
            self.bus.publish(T.ARM, {"channel_high": arm})
        fs = self.sess.sender.failsafe_active
        if fs != self._prev["fs"]:
            self.event("stale-input failsafe " + ("ON" if fs else "off")); self._prev["fs"] = fs
            self.bus.publish(T.FAILSAFE, {"active": fs})
        wd = self.sess.watchdog.state if self.sess.watchdog else 0
        if wd != self._prev["wd"]:
            self.event("sensor watchdog -> " + WATCHDOG_NAMES[wd]); self._prev["wd"] = wd
            self.bus.publish(T.WATCHDOG, {"state": WATCHDOG_NAMES[wd]})
