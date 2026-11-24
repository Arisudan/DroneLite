"""Linux joystick (/dev/input/js*) to Controls, with arming guards. Default map is the RadioMaster Boxer in USB
Joystick mode as measured on this machine (axes 1-7: roll, pitch, throttle, yaw, AUX1 arm, AUX2, AUX3).

Safety: nothing is armed until every mapped axis has reported once; the arm switch must be seen OFF at least once
before it can arm (so starting the program with the switch already ON never arms); arming is only accepted while
throttle is low at the moment the arm switch goes high (otherwise it stays blocked until the switch is cycled off);
losing the device returns SAFE controls so the sender's stale-input failsafe takes over.
"""
from __future__ import annotations

import errno
import os
import select
import struct
from dataclasses import dataclass
from typing import Optional

from .crsf import CRSF_MAX, CRSF_MIN
from .sender import Controls

JS_EVENT = struct.Struct("IhBB")        # time ms, value, type, axis/button number
AXIS_FULL = 32767.0


@dataclass
class GamepadMap:
    roll: int = 0
    pitch: int = 1
    throttle: int = 2
    yaw: int = 3
    arm: int = 4
    aux2: int = 5
    aux3: int = 6
    invert: frozenset = frozenset()     # axis indices to invert

    def mapped(self) -> set:
        return {self.roll, self.pitch, self.throttle, self.yaw, self.arm, self.aux2, self.aux3}


def _crsf_from_axis(v: float) -> int:
    return int(round(CRSF_MIN + (v + AXIS_FULL) / (2 * AXIS_FULL) * (CRSF_MAX - CRSF_MIN)))


class GamepadInput:
    def __init__(self, path: Optional[str] = None, fd: Optional[int] = None, mapping: GamepadMap = GamepadMap(),
                 arm_max_throttle: float = 0.05):
        if fd is None:
            fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        self._fd = fd
        self.map = mapping
        self.arm_max_throttle = arm_max_throttle
        self._axes = [0] * 32
        self._seen: set = set()
        self.connected = True
        self.arm_blocked = False
        self._arm_prev = False
        self._seen_arm_off = False
        self._buf = b""

    def poll(self) -> None:
        """Drain pending joystick events (non-blocking)."""
        while self.connected:
            try:
                r, _, _ = select.select([self._fd], [], [], 0)
                if not r:
                    return
                data = os.read(self._fd, 8 * 64)
            except BlockingIOError:
                return
            except OSError as e:
                if e.errno in (errno.ENODEV, errno.EIO, errno.EBADF):
                    self.connected = False
                    return
                raise
            if not data:                # EOF: device gone
                self.connected = False
                return
            self._buf += data
            while len(self._buf) >= JS_EVENT.size:
                _, value, typ, num = JS_EVENT.unpack(self._buf[:JS_EVENT.size])
                self._buf = self._buf[JS_EVENT.size:]
                if (typ & 0x7F) == 2 and num < len(self._axes):
                    self._axes[num] = value
                    self._seen.add(num)

    @property
    def ready(self) -> bool:
        return self.map.mapped() <= self._seen

    def _axis(self, idx: int) -> float:
        v = float(self._axes[idx])
        return -v if idx in self.map.invert else v

    def controls(self) -> Controls:
        """Current controls; SAFE (disarmed, throttle min) until ready or when the device is gone."""
        if not (self.connected and self.ready):
            return Controls()
        m = self.map
        throttle = (self._axis(m.throttle) + AXIS_FULL) / (2 * AXIS_FULL)
        want_arm = self._axis(m.arm) > 0
        if not want_arm:
            self._seen_arm_off = True
            self.arm_blocked = False
        elif not self._seen_arm_off:
            self.arm_blocked = True            # switch was already ON when we started: refuse until it is cycled off
        elif not self._arm_prev and throttle > self.arm_max_throttle:
            self.arm_blocked = True            # armed switch flipped with throttle up: refuse until cycled
        self._arm_prev = want_arm
        return Controls(roll=self._axis(m.roll) / AXIS_FULL, pitch=self._axis(m.pitch) / AXIS_FULL,
                        yaw=self._axis(m.yaw) / AXIS_FULL, throttle=throttle,
                        arm=want_arm and not self.arm_blocked,
                        aux2=_crsf_from_axis(self._axis(m.aux2)), aux3=_crsf_from_axis(self._axis(m.aux3)))

    def close(self) -> None:
        try:
            os.close(self._fd)
        except OSError:
            pass
