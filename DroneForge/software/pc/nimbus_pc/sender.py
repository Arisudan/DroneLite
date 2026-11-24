"""PC-side CRSF sender for a NimbusOS-style ELRS transmitter dongle (USB serial).

Behaviour taken from nimbus_core 2.9.0: raw 115200 8N1, DTR/RTS cleared on connect,
26-byte RC frame about every 10 ms, throttle forced to minimum when control input goes missing.
Safe by default: the arm channel stays off unless allow_arm=True is passed explicitly.
"""
from __future__ import annotations

import argparse
import errno
import glob
import math
import sys
import os
import select
import struct
import termios
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

from .crsf import (ADDR_FC, ADDR_RX, ADDR_TX, TYPE_OPTRANGE, CRSF_MID, CRSF_MIN, CRSF_MAX, FrameParser, decode_telemetry,
                   encode_device_ping, encode_rc_frame)
from .mapper import crsf_to_throttle01, stick_to_crsf, throttle01_to_crsf
from .watchdog import NAMES as WATCHDOG_NAMES, SensorWatchdog

TIOCM_DTR, TIOCM_RTS = 0x002, 0x004


class SerialLink:
    """Serial port opened the way NimbusOS does it."""

    def __init__(self, path: str, baud: int = 115200):
        if baud != 115200:
            raise ValueError("NimbusOS only supports 115200 baud on this link")
        self.path = path
        self.fd: Optional[int] = None
        self._wlock = threading.Lock()
        self.on_write: Optional[Callable[[bytes], None]] = None     # tap: called with every chunk written (live view)

    def open(self) -> None:
        self.fd = os.open(self.path, os.O_RDWR | os.O_NOCTTY)
        try:                      # one NimbusPC program per port: two readers split the bytes and corrupt every frame
            import fcntl
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            os.close(self.fd); self.fd = None
            raise OSError(errno.EBUSY, f"{self.path} is already open in another DroneForge program (GCS, live, doctor...). "
                                       f"Close it first: two programs on one serial port corrupt both")
        except OSError:
            pass                  # pseudo-terminals in tests may not support flock
        a = termios.tcgetattr(self.fd)
        a[0] = a[1] = a[3] = 0                                  # raw mode
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL     # 8N1, no flow control, no HUPCL
        a[4] = a[5] = termios.B115200
        a[6][termios.VMIN], a[6][termios.VTIME] = 0, 1
        termios.tcsetattr(self.fd, termios.TCSANOW, a)
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        try:  # clear DTR/RTS so an ESP32-S3 USB-Serial-JTAG port does not reset into its bootloader
            import fcntl
            fcntl.ioctl(self.fd, termios.TIOCMBIC, struct.pack("I", TIOCM_DTR | TIOCM_RTS))
        except OSError:
            pass  # pseudo-terminals used in tests do not support modem lines
        try:
            termios.tcflow(self.fd, termios.TCOON)
        except termios.error:
            pass

    def write(self, data: bytes) -> None:
        with self._wlock:
            os.write(self.fd, data)
        if self.on_write:
            self.on_write(data)

    def read(self, timeout: float = 0.05) -> bytes:
        r, _, _ = select.select([self.fd], [], [], timeout)
        return os.read(self.fd, 512) if r else b""

    def close(self) -> None:
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


@dataclass
class Controls:
    """High-level control input. Channel order is AETR (ch1 roll, ch2 pitch, ch3 throttle, ch4 yaw)."""
    roll: float = 0.0       # -1..1
    pitch: float = 0.0      # -1..1
    yaw: float = 0.0        # -1..1
    throttle: float = 0.0   # 0..1
    arm: bool = False       # AUX1 (ch5): 1811 when armed, 172 otherwise
    aux2: int = CRSF_MIN    # raw CRSF values for ch6 and ch7
    aux3: int = CRSF_MIN
    aux6: int = CRSF_MIN    # ch10: the PC controller's hover-reference throttle (same scale as the throttle channel) while the FC hold is requested: the FC starts its throttle from it
    aux5: int = CRSF_MIN    # ch9: FC height hold switch + target height (see mapper.height_to_aux5); CRSF_MIN = hold off
    aux7: int = CRSF_MID    # ch11: FC flight mode (see mapper.MODE_ALT_TICKS / MODE_POS_TICKS); the idle value (CRSF_MID, as Nimbus) = POS HOLD on the FC
    aux9: int = CRSF_MID    # ch13: v4.7.2 VO position aid, forward (1500 us = 0, 1 us = 1 mm; mapper.aid_to_ticks); needs a full-resolution ELRS rate
    aux10: int = CRSF_MID   # ch14: v4.7.2 VO position aid, right
    aux11: int = CRSF_MID   # ch15: v4.7.2 aid-valid flag (> 1900 us = valid); the centre (as Nimbus leaves it) = no aid

    def to_channels(self, allow_arm: bool) -> list[int]:
        ch = [CRSF_MID] * 16
        ch[0] = stick_to_crsf(self.roll)
        ch[1] = stick_to_crsf(self.pitch)
        ch[2] = throttle01_to_crsf(self.throttle)
        ch[3] = stick_to_crsf(self.yaw)
        ch[4] = CRSF_MAX if (self.arm and allow_arm) else CRSF_MIN
        ch[5] = max(CRSF_MIN, min(CRSF_MAX, self.aux2))
        ch[6] = max(CRSF_MIN, min(CRSF_MAX, self.aux3))
        ch[8] = max(CRSF_MIN, min(CRSF_MAX, self.aux5))
        ch[9] = max(CRSF_MIN, min(CRSF_MAX, self.aux6))
        ch[10] = max(CRSF_MIN, min(CRSF_MAX, self.aux7))
        ch[12] = max(CRSF_MIN, min(CRSF_MAX, self.aux9)); ch[13] = max(CRSF_MIN, min(CRSF_MAX, self.aux10)); ch[14] = max(CRSF_MIN, min(CRSF_MAX, self.aux11))
        return ch


SAFE_CONTROLS = Controls()  # sticks centred, throttle minimum, disarmed


class CrsfSender:
    """Sends RC frames at a fixed rate from a background thread and applies the stale-input failsafe."""

    PING_INTERVAL_S = 0.25   # Nimbus sends device pings every 250 ms
    PING_MAX_PER_TARGET = 12  # then give up (3 s): endless pings to a device that never answers stall the telemetry
                              # (measured 2026-11-03: 14 gaps >0.4 s in 172 s with pings, none in 112 s without)

    def __init__(self, link: SerialLink, hz: float = 100.0, stale_after_s: float = 0.25, allow_arm: bool = False,
                 ping_targets: tuple = (ADDR_TX, ADDR_RX, ADDR_FC), watchdog: Optional[SensorWatchdog] = None):
        self.link, self.period, self.stale_after_s, self.allow_arm = link, 1.0 / hz, stale_after_s, allow_arm
        self.watchdog = watchdog
        self.pings_sent = 0
        self._ping_due = {addr: 0.0 for addr in ping_targets}   # pinged until a DEVICE_INFO answer arrives (or the cap)
        self._ping_count = {addr: 0 for addr in ping_targets}
        self._ping_all = tuple(ping_targets)
        self._answered: set = set()
        self._controls = Controls()
        self._updated = time.monotonic()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.frames_sent = 0
        self.failsafe_active = False
        self.max_interval_s = 0.0

    def set_controls(self, controls: Controls) -> None:
        with self._lock:
            self._controls = controls
            self._updated = time.monotonic()

    def mark_device_info(self, origin: int) -> None:
        """A DEVICE_INFO reply came from `origin`: stop pinging that address."""
        self._answered.add(origin)
        self._ping_due.pop(origin, None)

    def rearm_pings(self) -> None:
        """The radio link just came up: devices behind it (RX, FC) could not answer before. Start a fresh capped ping burst for every
        address that has not answered yet (the cap keeps endless pings from stalling the telemetry)."""
        for addr in self._ping_all:
            if addr not in self._answered:
                self._ping_count[addr] = 0
                self._ping_due[addr] = 0.0

    @property
    def alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def current_controls(self) -> "Controls":
        with self._lock:
            return self._controls

    def feed_sensor(self, payload: bytes) -> None:
        """Feed one sensor sample (e.g. range/flow telemetry payload) to the watchdog, if enabled."""
        if self.watchdog is not None:
            self.watchdog.feed(payload, time.monotonic())

    def _run(self) -> None:
        nxt = time.monotonic()
        last = None
        while not self._stop.is_set():
            now = time.monotonic()
            with self._lock:
                controls, age = self._controls, now - self._updated
            self.failsafe_active = age > self.stale_after_s
            if self.failsafe_active:
                controls = SAFE_CONTROLS
            channels = controls.to_channels(self.allow_arm)
            if self.watchdog is not None:
                armed = controls.arm and self.allow_arm
                tripped = self.watchdog.update(armed, crsf_to_throttle01(channels[2]), now)
                if tripped:
                    print(f"[sensor-watchdog] {WATCHDOG_NAMES[tripped]} throttle=zero", file=sys.stderr, flush=True)
                if self.watchdog.forcing_zero:
                    channels[2] = CRSF_MIN
            try:
                self.link.write(encode_rc_frame(channels))
                for addr, due in list(self._ping_due.items()):
                    if now >= due:
                        self.link.write(encode_device_ping(addr))
                        self._ping_due[addr] = now + self.PING_INTERVAL_S
                        self.pings_sent += 1
                        self._ping_count[addr] = self._ping_count.get(addr, 0) + 1
                        if self._ping_count[addr] >= self.PING_MAX_PER_TARGET:
                            self._ping_due.pop(addr, None)
            except OSError:
                break
            self.frames_sent += 1
            if last is not None:
                self.max_interval_s = max(self.max_interval_s, now - last)
            last = now
            nxt += self.period
            if nxt < time.monotonic() - 5 * self.period:   # fell far behind: resynchronise instead of bursting
                nxt = time.monotonic() + self.period
            delay = nxt - time.monotonic()
            if delay > 0:
                self._stop.wait(delay)

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="crsf-sender", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        try:   # last frame is a safe one: centred sticks, throttle minimum, disarmed
            self.link.write(encode_rc_frame(SAFE_CONTROLS.to_channels(False)))
        except OSError:
            pass


class TelemetryReader:
    """Reads the same port, parses CRSF frames and keeps the latest value of each telemetry type."""

    def __init__(self, link: SerialLink, on_update: Optional[Callable[[str, dict], None]] = None,
                 on_frame: Optional[Callable] = None):
        self.link, self.on_update, self.on_frame = link, on_update, on_frame
        self.parser = FrameParser()
        self.latest: dict[str, dict] = {}
        self.frame_count = 0
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                data = self.link.read(0.05)
            except OSError:
                break
            for frame in self.parser.feed(data):
                self.frame_count += 1
                if self.on_frame:
                    self.on_frame(frame)
                decoded = decode_telemetry(frame)
                if decoded:
                    name, values = decoded
                    key = f"device_info_0x{values['origin']:02x}" if name == "device_info" else name
                    self.latest[key] = {**values, "t": time.monotonic()}
                    if self.on_update:
                        self.on_update(name, values)

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="crsf-telemetry", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)


def main() -> None:
    ap = argparse.ArgumentParser(description="NimbusOS-style CRSF sender (bench use, props off).")
    ap.add_argument("--port", required=True)
    ap.add_argument("--hz", type=float, default=100.0)
    ap.add_argument("--duration", type=float, default=10.0)
    ap.add_argument("--demo", choices=["idle", "sweep"], default="idle",
                    help="idle: sticks centred, throttle min. sweep: slow roll/pitch/yaw sine, throttle min")
    ap.add_argument("--allow-arm", action="store_true", help="let the arm channel go high (REMOVE PROPS FIRST)")
    ap.add_argument("--arm", action="store_true", help="request arming (needs --allow-arm)")
    ap.add_argument("--no-ping", action="store_true", help="do not send DEVICE_PING frames")
    ap.add_argument("--watch-frame-type", type=lambda v: int(v, 0), default=None, metavar="TYPE",
                    help="enable the sensor watchdog, fed by payloads of this CRSF frame type")
    ap.add_argument("--watch-optrange", action="store_true",
                    help="enable the sensor watchdog on Nimbus's range/flow frame (type 0xD3); the drone must send it")
    ap.add_argument("--list-ports", action="store_true", help="list candidate serial ports and exit")
    args = ap.parse_args()

    if args.list_ports:
        for p in sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*") + glob.glob("/dev/serial/by-id/*")):
            print(p)
        return
    link = SerialLink(args.port)
    try:
        link.open()
    except OSError as e:
        ports = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
        print(f"cannot open {args.port}: {e.strerror}. Candidate ports: {', '.join(ports) or 'none found'}. "
              f"(Use the ELRS dongle's port; the flight controller's USB port is MSP, not CRSF.)", file=sys.stderr)
        raise SystemExit(2)
    if args.watch_optrange and args.watch_frame_type is None:
        args.watch_frame_type = TYPE_OPTRANGE
    watchdog = SensorWatchdog() if args.watch_frame_type is not None else None
    sender = CrsfSender(link, hz=args.hz, allow_arm=args.allow_arm, watchdog=watchdog,
                        ping_targets=() if args.no_ping else (ADDR_TX, ADDR_RX, ADDR_FC))

    def on_frame(frame):
        if args.watch_frame_type is not None and frame.type == args.watch_frame_type:
            sender.feed_sensor(frame.payload)

    def on_update(name, values):
        if name == "device_info":
            sender.mark_device_info(values["origin"])
        print(f"[telemetry] {name}: {values}")

    telemetry = TelemetryReader(link, on_update=on_update, on_frame=on_frame)
    sender.start()
    telemetry.start()
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < args.duration:
            t = time.monotonic() - t0
            if args.demo == "sweep":
                c = Controls(roll=0.5 * math.sin(t), pitch=0.5 * math.sin(0.7 * t), yaw=0.5 * math.sin(0.5 * t),
                             throttle=0.0, arm=args.arm)
            else:
                c = Controls(arm=args.arm)
            sender.set_controls(c)
            time.sleep(0.02)
    finally:
        sender.stop()
        telemetry.stop()
        link.close()
    print(f"sent {sender.frames_sent} frames + {sender.pings_sent} pings, max interval {sender.max_interval_s*1000:.1f} ms, "
          f"telemetry frames {telemetry.frame_count} (crc errors {telemetry.parser.crc_errors})")


if __name__ == "__main__":
    main()
