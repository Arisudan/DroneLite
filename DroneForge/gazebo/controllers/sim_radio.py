#!/usr/bin/env python3
"""Simulated radio link for the GCS: the ELRS TX dongle + receiver + our FC firmware's telemetry, on a pseudo-terminal. SIMULATION ONLY (no hardware is touched).

The GCS opens the pty exactly like the CP2102 dongle port (python3 -m nimbus_gcs --port <link> --connect) and talks plain CRSF:
  uplink   RC_CHANNELS (AETR + AUX) -> decoded the way the flashed FC firmware reads them -> JSON commands to the sim FC (fc_v4_5_bridge.py):
             ch5  AUX1 arm (refused with the throttle stick up, like Betaflight)
             ch9  AUX5 height hold: min = off (throttle stick flies), 0.2-2.5 m = take off / hold at that height, 1792 = land
             ch11 AUX7 mode: 1100-1399 us = ALT HOLD, anything else = POS HOLD
             ch1/2/4 roll/pitch/yaw sticks: ANGLE mode (tilt through the Nimbus angle curve); centred sticks = the FC's own position hold
             no RC frame for 0.5 s = failsafe: lands when flying, disarms on the floor
  downlink what the flashed firmware (fc-state-0xD6, measured on the bench 2026-11-13) sends: 0xD6 state ~18 Hz (range, flow, EKF body velocity, attitude, hold bits),
           BATTERY ~1 Hz (0.01 V units, as the Matek build), FLIGHT_MODE text ~0.6 Hz with the rotating motor diagnostic; the dongle adds LINK_STATISTICS and answers
           DEVICE_PING / MSP like the emulator in nimbus_pc/emulator.py.
Delays: uplink --uplink-ms (default 20 ms radio + FC), downlink --downlink-ms (default 30 ms).
UNVERIFIED conventions (marked so they can be checked against the real FC): attitude pitch positive = nose down (forward, the stick sign), yaw clockwise, vz up-positive.
The pty path is linked to --link (default ~/.config/droneforge/sim_radio) so the GCS always finds it at the same name."""
import argparse, collections, json, math, os, struct, sys, threading, time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "../../software/pc")))
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nimbus_pc.crsf import (CRSF_MAX, CRSF_MID, CRSF_MIN, SYNC_RADIO, TYPE_BATTERY, TYPE_FLIGHT_MODE, TYPE_LINK_STATISTICS, TYPE_RC_CHANNELS,
                            encode_frame, encode_state, unpack_channels)
from nimbus_pc import commands
from nimbus_pc.emulator import DongleEmulator
from nimbus_pc.mapper import AUX5_LAND, aux5_to_height

FLYING = ("TAKEOFF", "ALT_HOLD", "POS_HOLD", "LANDING", "MANUAL")


def ticks_to_us(t): return 5.0 * t / 8.0 + 880.0
def us_to_ticks(us): return int(round((us - 880.0) * 8.0 / 5.0))


# What the real ELRS link delivers to the FC (measured 2026-11-24, ELRS 4.1.0, 250 Hz, switch mode Wide): CH6-12 in ~15.9 us steps (64 levels over 1000-2000 us),
# CH13-16 not carried. DF_ELRS=full ("100Hz Full": all 16 channels at 10 bit, ~1 us) passes every channel through.
ELRS_MODE = os.environ.get("DF_ELRS", "full")      # 2026-11-24: the real TX is now 100Hz Full + 16ch Rate/2 (all channels ~1 us); DF_ELRS=wide = the old 250 Hz Wide link
DF_FW_SIM = os.environ.get("DF_FW", "v47")


def elrs_channels(ch):
    if ELRS_MODE == "full": return list(ch)
    out = list(ch)
    for i in range(5, 12):
        us = min(2000.0, max(1000.0, ticks_to_us(ch[i]))); step = 1000.0 / 63.0
        out[i] = us_to_ticks(1000.0 + round((us - 1000.0) / step) * step)
    for i in range(12, 16): out[i] = 992                       # not carried: the RX outputs the centre
    return out


def stick(t): return max(-1.0, min(1.0, (t - CRSF_MID) / 820.0))


class SimRadio(DongleEmulator):
    """The dongle emulator's pty, ping and MSP handling, with the telemetry and the RC decoding replaced by the simulated drone."""

    def __init__(self, node, uplink_s, downlink_s):
        super().__init__()
        self.node = node; self.up = uplink_s; self.down = downlink_s
        self.rcq = collections.deque(); self.txq = collections.deque(); self.lock = threading.Lock()
        self.tel = None; self.t_rc = 0.0; self.armed_cmd = False; self.prev = {}; self.next = {"state": 0.0, "bat": 0.0, "fm": 0.0, "link": 0.0}; self.diag = 0
        self.arm_refused = False; self.aidq = collections.deque()
        if os.environ.get("DF_FW", "v47") in ("v47", "v471", "v472", "v473r"): self.KNOWN_WRITES = set(self.KNOWN_WRITES) | {commands.MSP2_DF_POS_AID}   # rev1 does not know the aid: it answers an error byte, as the real FC

    # ---- uplink
    def _handle(self, f):
        if f.type == TYPE_RC_CHANNELS and len(f.payload) == 22:
            self.rc_frames += 1; ch = unpack_channels(f.payload); self.last_channels = ch
            with self.lock: self.rcq.append((time.time() + self.up, elrs_channels(ch)))
            return
        super()._handle(f)

    def _handle_write(self, f):
        """MSP writes to the FC: v4.7's MSP2_DF_POS_AID (the PC's visual-odometry position aid) goes to the sim FC like the RC, after the uplink delay."""
        msg = self._write_asm.feed(f)
        if not msg: return
        if msg["cmd"] == commands.MSP2_DF_POS_AID and msg["cmd"] in self.KNOWN_WRITES:      # DF_FW=rev1: the FC ignores it
            a = commands.decode_msp_pos_aid(msg["payload"])
            if a: self.aidq.append((time.time() + self.up, a))
        elif msg["cmd"] == commands.MSP_SET_VTX_CONFIG:
            self.vtx_writes.append(commands.decode_vtx_set_config(msg["payload"]))
        self._reply_write(msg["cmd"])

    def _apply(self, ch, now):
        """One RC frame, as the FC firmware would act on it."""
        self.t_rc = now; tel = self.tel or {}; mode = tel.get("mode", "DISARMED"); cmd = {}
        thr = (max(CRSF_MIN, min(CRSF_MAX, ch[2])) - CRSF_MIN) / 1639.0
        arm = ch[4] > 1500; a5 = ch[8]; mode_alt = 1100 <= ticks_to_us(ch[10]) <= 1399
        if arm and mode == "DISARMED":
            if thr > 0.10: self.arm_refused = True                                       # Betaflight: no arming with the throttle up
            elif not self.arm_refused or thr < 0.05: self.arm_refused = False; self._cmd({"cmd": "arm"}); return
        if not arm:
            self.arm_refused = False
            if mode != "DISARMED": self._cmd({"cmd": "disarm"})
            return
        land = abs(a5 - AUX5_LAND) <= 16; hold = (not land) and a5 > CRSF_MIN + 16
        h = max(0.2, min(2.5, aux5_to_height(a5))) if hold else None
        if land and mode in FLYING and mode != "LANDING": self._cmd({"cmd": "land"}); return
        if hold and mode == "ARMED": self._cmd({"cmd": "takeoff", "target_z": h}); return
        if not hold and not land and (mode in ("TAKEOFF", "ALT_HOLD", "POS_HOLD") or (mode == "ARMED" and thr > 0.12)): cmd["cmd"] = "manual"     # hold switched off: the throttle stick flies
        # (switching the hold back on in MANUAL flight is not modelled: the GCS never does it; the FC stays in MANUAL until it lands or disarms)
        if hold and h is not None and abs(h - self.prev.get("h", -1)) > 0.004: cmd["target_z"] = h; self.prev["h"] = h
        want = "alt_hold" if mode_alt else "pos_hold"
        if mode in ("ALT_HOLD", "POS_HOLD") and want.upper() != mode and not cmd.get("cmd"): cmd["cmd"] = want
        if DF_FW_SIM in ("v472", "v473r") and ticks_to_us(ch[14]) > 1900.0:           # v4.7.2: the VO aid on CH13-15 (firmware hhReadRcAid: 1500 us = 0, 1 us = 1 mm, 1100..1900 us)
            fu, ru = ticks_to_us(ch[12]), ticks_to_us(ch[13])
            if 1100.0 <= fu <= 1900.0 and 1100.0 <= ru <= 1900.0: cmd["aid"] = [(fu - 1500.0) * 0.001, (ru - 1500.0) * 0.001, True]
        r, p, y = stick(ch[0]), stick(ch[1]), stick(ch[3])
        cmd.update({"roll": r, "pitch": p, "yaw": -y, "throttle": thr})      # yaw: Betaflight stick + = clockwise; the sim FC turns counter-clockwise for + (ROS)
        # (the sim FC does what the firmware does with the sticks and AUX7: ANGLE stick + the firmware's hold angle; ALT HOLD pauses the hold)
        self._cmd(cmd)

    def _cmd(self, d):
        m = String(); m.data = json.dumps(d); self.node.pub.publish(m)

    def _failsafe(self, now):
        tel = self.tel or {}; mode = tel.get("mode", "DISARMED")
        if mode != "DISARMED" and self.t_rc and now - self.t_rc > 0.5:
            self._cmd({"cmd": "land"} if mode in FLYING and mode != "LANDING" and tel.get("z", 0) > 0.08 else {"cmd": "disarm"} if mode == "ARMED" else {})
            self.t_rc = now                                                              # once per 0.5 s

    # ---- downlink
    def _queue(self, frame, now): self.txq.append((now + self.down, frame))

    def _telemetry(self):
        now = time.time()
        with self.lock:
            while self.rcq and self.rcq[0][0] <= now: self._apply(self.rcq.popleft()[1], now)
        while self.aidq and self.aidq[0][0] <= now:
            a = self.aidq.popleft()[1]; self._cmd({"aid": [a["forward_m"], a["right_m"], a["valid"]]})
        self._failsafe(now)
        t = self.tel
        if t is not None:
            n = self.next
            if now >= n["state"]:
                n["state"] = now + 1 / 18.0; self._queue(self._state(t), now)
            if now >= n["bat"]:
                n["bat"] = now + 1.0; v = int(round(t.get("vbat", 0.0) * 100))
                self._queue(encode_frame(TYPE_BATTERY, struct.pack(">HH", v, 0) + (0).to_bytes(3, "big") + bytes([100])), now)
            if now >= n["fm"]:
                n["fm"] = now + 1 / 0.6; self._queue(encode_frame(TYPE_FLIGHT_MODE, self._fm(t).encode() + b"\x00"), now)
        if now >= self.next["link"]:
            self.next["link"] = now + 0.2
            lq = 100 if (self.t_rc and now - self.t_rc < 0.5) else 0
            self._queue(encode_frame(TYPE_LINK_STATISTICS, bytes([40, 42, lq, 10, 0, 7, 3, 45, lq, 9]), sync=SYNC_RADIO), now)
        while self.txq and self.txq[0][0] <= now: self._send(self.txq.popleft()[1])

    def _state(self, t):
        d = t.get("range", 0.0) or 0.0; dmm = int(round(d * 1000)) if 0.001 <= d <= 5.999 else 0
        vf, vr = t.get("vx", 0.0), t.get("vy", 0.0)
        fx = int(max(-32767, min(32767, round(vf * 100 / d)))) if dmm else 0; fy = int(max(-32767, min(32767, round(vr * 100 / d)))) if dmm else 0      # raw flow_y + = right, as the real FC (2026-11-23 bench)
        mode = t.get("mode", "DISARMED"); active = mode in ("TAKEOFF", "ALT_HOLD", "POS_HOLD", "LANDING")
        hm = 1 if mode == "TAKEOFF" else 2 if mode == "LANDING" else 0
        bits = (4 if active else 0) | (hm << 4)
        e4 = lambda deg: int(max(-32767, min(32767, round(math.radians(deg) * 10000))))
        yaw_cw = -t.get("yaw", 0.0)
        mm = lambda v: int(max(-32767, min(32767, round(v * 1000))))
        return encode_state(dmm, fx, fy, 200 if dmm else 0, True, 1 if dmm else 0, mm(vf), mm(vr), mm(t.get("vz", 0.0)),
                            e4(t.get("roll", 0.0)), e4(t.get("pitch", 0.0)), e4(yaw_cw), hold_bits=bits, body_vel=True)

    def _fm(self, t):
        mode = t.get("mode", "DISARMED"); txt = "STAB" + (" HH" if mode in ("TAKEOFF", "ALT_HOLD", "POS_HOLD", "LANDING") else "")
        m = [int(1000 + 1000 * x) for x in (t.get("m") or [0, 0, 0, 0])]; thr = int(sum(m) / 4) if m else 1000
        self.diag += 1
        if self.diag % 3 == 0: txt += f" M{m[0]},{m[1]},{m[2]},{m[3]} T{thr}"
        return txt + ("*" if mode == "DISARMED" else "")


class RadioNode(Node):
    def __init__(self):
        super().__init__("droneforge_sim_radio")
        self.pub = self.create_publisher(String, "/droneforge/fc/command", 10)
        self.radio = None
        self.create_subscription(String, "/droneforge/fc/telemetry", self.on_tel, 10)

    def on_tel(self, m):
        try: self.radio.tel = json.loads(m.data)
        except Exception: pass


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--link", default=os.path.expanduser("~/.config/droneforge/sim_radio"), help="stable name for the pty (a symlink)")
    ap.add_argument("--uplink-ms", type=float, default=20.0); ap.add_argument("--downlink-ms", type=float, default=30.0)
    a = ap.parse_args()
    rclpy.init(); node = RadioNode()
    radio = SimRadio(node, a.uplink_ms / 1000.0, a.downlink_ms / 1000.0); node.radio = radio
    os.makedirs(os.path.dirname(a.link), exist_ok=True)
    try: os.unlink(a.link)
    except FileNotFoundError: pass
    os.symlink(radio.path, a.link); radio.start()
    node.get_logger().info(f"simulated radio on {radio.path} (linked as {a.link}): GCS --port {a.link}")
    try: rclpy.spin(node)
    except (KeyboardInterrupt, Exception): pass
    finally:
        try:
            if os.readlink(a.link) == radio.path: os.unlink(a.link)
        except OSError: pass
        radio.stop()


if __name__ == "__main__":
    main()
