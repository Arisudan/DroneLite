"""Depth client: sends the newest video frame to the Nimbus depth server (depth_server.py on a GPU machine) and keeps the newest answer:
the Nimbus 6x8 grid and the live voxel floor plan. Qt-free. Safe by design: it only reads video and shows the result; it never sends a flight command."""
from __future__ import annotations

import json
import os
import threading
import time
import urllib.parse
import urllib.request

import subprocess
import sys

import cv2

DEFAULT_URL = "http://127.0.0.1:8765"            # every model runs on this laptop (a GPU laptop later: same address, nothing to edit); the RTAB-Map stream (port 5765) is on the same machine
PATH = os.environ.get("DRONEFORGE_DEPTH", os.path.expanduser("~/.config/droneforge/depth.json"))


def load_url() -> str:
    try:
        return json.load(open(PATH)).get("url") or DEFAULT_URL
    except Exception:
        return DEFAULT_URL


def host_of(url: str) -> str:
    """The machine name or address in a server URL (the map stream is on the same machine as the depth server)."""
    from urllib.parse import urlparse
    u = urlparse(url if "//" in url else "http://" + url)
    return u.hostname or ""


def save_url(url: str) -> None:
    try:
        os.makedirs(os.path.dirname(PATH), exist_ok=True); json.dump({"url": url}, open(PATH, "w"))
    except Exception:
        pass


class DepthClient:
    def __init__(self, get_frame) -> None:
        self.get_frame = get_frame; self.url = ""; self.running = False; self.result = None; self.t_result = 0.0; self.rtt_ms = None; self.rate_hz = 0.0; self.error = ""
        self._th = None; self._times = []

    def start(self, url: str) -> None:
        self.stop(); self.url = url.strip().rstrip("/"); self.error = ""; self.running = True
        self._th = threading.Thread(target=self._loop, daemon=True); self._th.start()

    def stop(self) -> None:
        self.running = False

    def age_s(self) -> float:
        return time.time() - self.t_result if self.result else float("inf")

    def _loop(self) -> None:
        while self.running:
            f = self.get_frame()
            if f is None:
                self.error = "No video frame yet."; time.sleep(0.2); continue
            t = time.time()
            try:
                ok, jpg = cv2.imencode(".jpg", f, [cv2.IMWRITE_JPEG_QUALITY, 85])
                self.result = json.loads(urllib.request.urlopen(urllib.request.Request(self.url + "/depth", jpg.tobytes()), timeout=6).read())
                now = time.time(); self.t_result = now; self.rtt_ms = (now - t) * 1000; self.error = ""
                self._times = (self._times + [now])[-10:]
                self.rate_hz = (len(self._times) - 1) / (self._times[-1] - self._times[0]) if len(self._times) > 1 else 0.0
            except Exception as e:
                self.error = f"Depth server: {str(e)[:70]}"; time.sleep(0.5)


SERVER_SCRIPT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "gazebo", "vision", "depth_server.py"))
LOG_PATH = os.path.expanduser("~/.config/droneforge/depth_server.log")


def is_local(url: str) -> bool:
    return host_of(url) in ("127.0.0.1", "localhost", "::1", "0.0.0.0")


def providers(url: str, timeout: float = 1.0) -> str:
    """What the depth server answers to a plain GET ('ok CPUExecutionProvider' or '... CUDAExecutionProvider ...'); '' when it is not answering."""
    try:
        return urllib.request.urlopen(url.rstrip("/") + "/", timeout=timeout).read().decode(errors="replace")
    except Exception:
        return ""


def ensure_local_server(url: str, script: str = SERVER_SCRIPT):
    """Starts the depth model on this laptop when the address is local and nothing answers there yet. Returns (process or None, text for the status line).
    It is the same depth_server.py the PC used to run, so a GPU laptop only needs onnxruntime with CUDA: it picks the GPU by itself."""
    if not is_local(url):
        return None, ""
    if providers(url):
        return None, "Depth model already running on this laptop."
    if not os.path.isfile(script):
        return None, f"Depth server script not found: {script}"
    port = urllib.parse.urlparse(url if "//" in url else "http://" + url).port or 8765
    try:
        os.makedirs(os.path.dirname(LOG_PATH), exist_ok=True)
        proc = subprocess.Popen([sys.executable, script, "--port", str(port)], cwd=os.path.dirname(script), stdout=open(LOG_PATH, "w"), stderr=subprocess.STDOUT)
    except Exception as e:
        return None, f"Could not start the depth model: {str(e)[:60]}"
    return proc, "Starting the depth model on this laptop (loading takes about 10 s)."
