"""Hardware-in-the-loop hover test, step 1: the REAL FC and radio link in the loop, the plant simulated.

    HoverController --CRSF (real TX/RX, ~22 ms)--> real FC (ANGLE mode, arming, mixer)
                                                       |  motor outputs over MSP (USB, read-only: 101 status, 104 motors)
        simulated sensors (range/flow/attitude) <--- DroneSim <--- thrust from the FC's real motor outputs

What this proves: arming guards, the real link delay and jitter, the FC's throttle->motor mapping, mixer direction
(roll/pitch signs seen in the motor differences), motor-guard/failsafe behaviour and the controller's reaction to all of it.
What it cannot prove: the real attitude dynamics (the bench IMU sits still, so tilt comes from the simulator's ANGLE model)
and real thrust (props OFF; throttle is scaled down so the motors stay slow).

PROPS OFF. Default is DISARMED (a plumbing check). Use --arm only with props off and the drone held down.
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import select
import struct
import sys
import termios
import time
from dataclasses import replace
from typing import Optional

from nimbus_pc.sender import CRSF_MAX, Controls
from nimbus_pc.session import Session

from .controller import HoverConfig, HoverController, HoverState
from .sim import DroneSim, SimParams

MSP_STATUS, MSP_MOTOR, MSP_STATUS_EX = 101, 104, 150
MSP_READ_ONLY = (MSP_STATUS, MSP_MOTOR, MSP_STATUS_EX)


class FcReader:
    """Read-only MSP over the FC's USB port (never writes anything but requests)."""

    def __init__(self, path: str):
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
        a = termios.tcgetattr(self.fd)
        a[0] = a[1] = a[3] = 0
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        a[4] = a[5] = termios.B115200
        a[6][termios.VMIN], a[6][termios.VTIME] = 0, 0
        termios.tcsetattr(self.fd, termios.TCSANOW, a)
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def close(self) -> None:
        os.close(self.fd)

    def request(self, cmd: int, timeout: float = 0.03) -> Optional[bytes]:
        assert cmd in MSP_READ_ONLY, "HIL only sends read-only MSP requests"
        os.write(self.fd, b"$M<" + bytes([0, cmd, cmd]))
        buf, end = b"", time.monotonic() + timeout
        while time.monotonic() < end:
            if select.select([self.fd], [], [], 0.005)[0]:
                buf += os.read(self.fd, 512)
            i = buf.find(b"$M>")
            if i >= 0 and len(buf) >= i + 6 and len(buf) >= i + 6 + buf[i + 3]:
                return buf[i + 5:i + 5 + buf[i + 3]]
        return None

    def armed(self) -> Optional[bool]:
        p = self.request(MSP_STATUS)
        return bool(struct.unpack_from("<I", p, 6)[0] & 1) if p and len(p) >= 10 else None

    def blockers(self) -> Optional[int]:
        """Betaflight arming-disable mask (0 = nothing blocks arming)."""
        p = self.request(MSP_STATUS_EX, 0.05)
        if not p or len(p) < 22: return None
        idx = 16 + p[15]
        return struct.unpack_from("<I", p, idx + 1)[0]

    def motors(self) -> Optional[list]:
        p = self.request(MSP_MOTOR)
        return list(struct.unpack_from("<4H", p, 0)) if p and len(p) >= 8 else None


def run(adapter: str, fc_port: str, duration: float, arm: bool, scale: float, idle: float, min_check: float = 1050.0, acro: bool = True, land_at: Optional[float] = 40.0, scenario_cfg: Optional[HoverConfig] = None,
        csv_path: Optional[str] = None, log=print) -> dict:
    cfg = scenario_cfg or HoverConfig()
    sim = DroneSim(replace(SimParams(), ctrl_delay_s=0.0), seed=1)
    ctl = HoverController(cfg)
    fc = FcReader(fc_port)
    rows, events = [], []
    fc_armed, motors = False, [1000] * 4
    t_start, dt = 3.5, 0.01
    refused = None
    stuck_armed = 0.0
    hs_t0 = None
    ready_t = None
    thr_hist: list = []
    n_fallback = n_total = 0
    session = Session(adapter, hz=100.0, allow_arm=arm, ping=True)
    started = False
    try:
        session.start(); started = True
        t0 = time.monotonic()
        k = 0
        while sim.t < duration:
            now = time.monotonic() - t0
            if now < sim.t:                                    # real-time pacing
                time.sleep(min(sim.t - now, 0.005))
                continue
            t = sim.t
            if arm and hs_t0 is None and not fc_armed and k % 25 == 0:
                blk = fc.blockers()
                ready_t = t if (blk == 0 and ready_t is None) else ready_t
                if t > 25.0 and ready_t is None:
                    events.append((round(t, 2), f"FC never became ready to arm (blockers 0x{blk or 0:X})")); break
            handshake = arm and ctl.state == HoverState.IDLE and t >= t_start and ready_t is not None and not fc_armed
            if handshake:                                      # arm at ZERO throttle first, like a pilot; controller starts once the FC reports armed
                hs_t0 = t if hs_t0 is None else hs_t0
                if t - hs_t0 > 4.0:
                    events.append((round(t, 2), f"FC did not arm within 4 s of an arm request at zero throttle (blockers 0x{fc.blockers() or 0:X})")); break
            elif ctl.state == HoverState.IDLE and t >= t_start and not ctl.refused and (not arm or fc_armed):
                if not ctl.start(t):
                    refused = list(ctl.refused); break
            if land_at is not None and t >= land_at and ctl.state in (HoverState.TAKEOFF, HoverState.SIGN_CHECK, HoverState.HOVER):
                ctl.land(t, "scheduled")
            samples = sim.poll()
            if ctl.state == HoverState.IDLE:
                ctl.observe(t, samples); samples = []
            cmd = ctl.update(t, samples, dt)
            if handshake:
                cmd = Controls(arm=True)
            send = Controls(roll=cmd.roll, pitch=cmd.pitch, yaw=0.0, throttle=cmd.throttle * scale if cmd.arm else 0.0, arm=bool(cmd.arm and arm), aux2=CRSF_MAX if acro else Controls().aux2)
            session.set_controls(send)
            if k % 2 == 0:
                m = fc.motors()
                if m: motors = m
            if k % 10 == 0:
                a = fc.armed()
                if a is not None: fc_armed = a
            # throttle the FC actually produced -> stick-equivalent for the plant (undo the scale)
            mn = (sum(motors) / 4 - 1000) / 1000.0
            rc_us = min_check + (2000.0 - min_check) * max(mn - idle, 0.0) / (1 - idle)   # undo BF: motor = idle + (1-idle)*(rc-min_check)/(2000-min_check)
            thr_sent = max(rc_us - 1000.0, 0.0) / 1000.0 if mn > idle + 0.002 else 0.0
            thr_equiv = thr_sent / scale
            # bench wind-up guard: a clipped motor (at the idle floor) with a big spread means the mean is no longer collective thrust
            floor = 1000.0 + idle * 1000.0 + 4
            clipped = fc_armed and min(motors) <= floor and (max(motors) - min(motors)) > 40
            thr_hist.append(send.throttle / scale)
            if len(thr_hist) > 3: thr_hist.pop(0)
            if clipped:
                thr_equiv = thr_hist[0]                       # the verified FC throttle map, ~20 ms link delay
                n_fallback += 1
            n_total += 1
            sim.step(dt, cmd.roll, cmd.pitch, thr_equiv, fc_armed)
            if (not arm) and fc_armed:
                stuck_armed += dt
                if stuck_armed > 0.5:
                    events.append((round(t, 2), "FC armed although we never asked: stopping")); break
            if k % 10 == 0:
                roll_d = (motors[2] + motors[3]) - (motors[0] + motors[1])     # BF quad-X: M1 RR, M2 FR, M3 RL, M4 FL; roll right -> left faster
                pitch_d = (motors[0] + motors[2]) - (motors[1] + motors[3])    # pitch forward (nose down) -> rear faster
                rows.append({"t": round(t, 2), "state": ctl.state.value, "cmd_roll": round(cmd.roll, 3), "cmd_pitch": round(cmd.pitch, 3),
                             "cmd_thr": round(cmd.throttle, 3), "sent_thr": round(send.throttle, 3), "fc_armed": int(fc_armed),
                             "m1": motors[0], "m2": motors[1], "m3": motors[2], "m4": motors[3], "roll_diff": roll_d, "pitch_diff": pitch_d,
                             "z": round(sim.z, 3), "z_est": round(ctl.z, 3), "x": round(sim.x, 3), "y": round(sim.y, 3)})
            k += 1
            if sim.crashed:
                events.append((round(t, 2), "simulated drone crashed")); break
    finally:
        if started:
            session.set_controls(Controls())
            time.sleep(0.15)
            session.stop()
        fc.close()
    if csv_path and rows:
        with open(csv_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    res = summarize(rows, ctl, refused, events, arm)
    res["thrust_from_motors_pct"] = round(100.0 * (1 - n_fallback / max(n_total, 1)), 1)
    return res


def _corr(a, b) -> Optional[float]:
    n = len(a)
    if n < 5: return None
    ma, mb = sum(a) / n, sum(b) / n
    sa = math.sqrt(sum((x - ma) ** 2 for x in a)); sb = math.sqrt(sum((x - mb) ** 2 for x in b))
    return None if sa < 1e-9 or sb < 1e-9 else sum((x - ma) * (y - mb) for x, y in zip(a, b)) / (sa * sb)


def summarize(rows, ctl, refused, events, arm) -> dict:
    out = {"armed_run": arm, "refused": refused, "final_state": ctl.state.value, "end_reason": ctl.reason, "events": events, "controller_events": [str(e) for e in list(ctl.events)[-12:]]}
    armed_rows = [r for r in rows if r["fc_armed"]]
    out["samples"] = len(rows); out["armed_samples"] = len(armed_rows)
    if armed_rows:
        out["max_motor"] = max(max(r["m1"], r["m2"], r["m3"], r["m4"]) for r in armed_rows)
        out["roll_sign_corr"] = _corr([r["cmd_roll"] for r in armed_rows], [r["roll_diff"] for r in armed_rows])
        out["pitch_sign_corr"] = _corr([r["cmd_pitch"] for r in armed_rows], [r["pitch_diff"] for r in armed_rows])
        out["max_z_m"] = max(r["z"] for r in armed_rows)
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--adapter", required=True, help="CP2102 port to the TX (CRSF)")
    ap.add_argument("--fc", required=True, help="FC USB port (MSP, read-only)")
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--arm", action="store_true", help="really arm the FC (PROPS OFF, drone held down)")
    ap.add_argument("--thr-scale", type=float, default=0.4, help="scale throttle sent to the FC so motors stay slow (plant undoes it)")
    ap.add_argument("--motor-idle", type=float, default=0.055, help="FC motor idle fraction (dshot_idle_value/10000)")
    ap.add_argument("--min-check", type=float, default=1050.0, help="Betaflight min_check (us)")
    ap.add_argument("--angle", action="store_true", help="keep ANGLE mode on the bench (the still IMU makes the I-term wind up; not recommended)")
    ap.add_argument("--land-at", type=float, default=40.0)
    ap.add_argument("--csv", default=None)
    a = ap.parse_args(argv)
    if a.arm:
        print("ARMED RUN: props must be OFF and the drone held down. Starting in 5 s (Ctrl-C to abort)...")
        time.sleep(5)
    res = run(a.adapter, a.fc, a.duration, a.arm, a.thr_scale, a.motor_idle, min_check=a.min_check, acro=not a.angle, land_at=a.land_at, csv_path=a.csv)
    for k, v in res.items(): print(f"{k}: {v}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
