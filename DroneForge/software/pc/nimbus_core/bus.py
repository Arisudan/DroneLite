"""Tiny thread-safe pub/sub topic bus with a latest-value cache (in-process; a ZMQ bridge can be added later without changing clients)."""
from __future__ import annotations

import threading
from collections import defaultdict
from typing import Any, Callable


class TopicBus:
    def __init__(self) -> None:
        self._subs: dict[str, list[Callable[[Any], None]]] = defaultdict(list)
        self._last: dict[str, Any] = {}
        self._lock = threading.Lock()
        self.errors = 0                                   # exceptions raised by subscribers (never propagate to the publisher)

    def subscribe(self, topic: str, cb: Callable[[Any], None], replay: bool = False) -> Callable[[], None]:
        with self._lock:
            self._subs[topic].append(cb)
            last = self._last.get(topic, _MISSING)
        if replay and last is not _MISSING:
            self._call(cb, last)
        return lambda: self.unsubscribe(topic, cb)

    def unsubscribe(self, topic: str, cb: Callable[[Any], None]) -> None:
        with self._lock:
            if cb in self._subs.get(topic, []):
                self._subs[topic].remove(cb)

    def publish(self, topic: str, msg: Any, cache: bool = True) -> None:
        with self._lock:
            if cache:
                self._last[topic] = msg
            subs = list(self._subs.get(topic, ()))
        for cb in subs:
            self._call(cb, msg)

    def last(self, topic: str, default: Any = None) -> Any:
        with self._lock:
            return self._last.get(topic, default)

    def _call(self, cb: Callable[[Any], None], msg: Any) -> None:
        try:
            cb(msg)
        except Exception:                                 # a broken UI widget must never stop the link
            self.errors += 1


_MISSING = object()
