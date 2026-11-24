"""Notifications: ring buffer of operator notices (bell list). Every core.event() line becomes an info/caution/alarm notice; core.notify() adds explicit ones. Qt-free."""
from __future__ import annotations

import re
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable

from . import topics as T

LEVELS = ("info", "caution", "alarm")
_ALARM = re.compile(r"SAFETY|LET GO|no longer armed|failsafe ON|inverted", re.I)
_CAUTION = re.compile(r"REFUSED|battery low|LOST|watchdog|timed out|cannot ", re.I)


@dataclass(frozen=True)
class Notice:
    t: float
    level: str          # info | caution | alarm
    text: str
    source: str
    seq: int = 0


def infer_level(text: str) -> str:
    return "alarm" if _ALARM.search(text) else "caution" if _CAUTION.search(text) else "info"


class Notifications:
    def __init__(self, core, clock: Callable[[], float] = time.time, size: int = 200) -> None:
        self.core, self.clock = core, clock
        self._buf: deque = deque(maxlen=size)
        self._seq = 0
        self._read = 0
        self._lock = threading.Lock()
        core.bus.subscribe(T.EVENT, lambda text: self.add(str(text), infer_level(str(text)), "event"))

    def add(self, text: str, level: str = "info", source: str = "") -> Notice:
        if level not in LEVELS:
            level = "info"
        with self._lock:
            self._seq += 1
            n = Notice(self.clock(), level, text, source, self._seq)
            self._buf.append(n)
        self.core.bus.publish(T.NOTIFY, n, cache=False)
        return n

    def all(self) -> list:
        with self._lock:
            return list(self._buf)

    def unread(self) -> list:
        with self._lock:
            return [n for n in self._buf if n.seq > self._read]

    def mark_read(self) -> None:
        with self._lock:
            self._read = self._seq
