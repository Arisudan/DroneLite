#!/usr/bin/env python3
"""Loopback bench: how much delay does the PC-SIDE video path add? (encode + local transport + DECODE + the capture library's own buffering.)
A sender renders frames carrying a QR code with its own clock, encodes them with x264 (zerolatency) and streams them to localhost as MPEG-TS over UDP. Several receivers decode the stream; the QR code in
each received frame is read back and compared with the clock at the moment the receiver got the frame (the same method as the GCS Latency test). No camera, no Wi-Fi, no CosmoStreamer: it only shows what the
RECEIVING SOFTWARE adds. The camera's own delay and the Wi-Fi are not in it.
usage: python3 tools_scripts/video_latency_bench.py [seconds_per_variant]"""
import os, subprocess, sys, threading, time
import numpy as np
sys.path.insert(0, os.path.expanduser("~/Music/DroneForge/software/pc"))
import cv2
from nimbus_gcs import video_latency as L

W, H, FPS, PORT = 640, 480, 30, 3111
SECS = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0
URL = f"udp://127.0.0.1:{PORT}?fifo_size=1000000&overrun_nonfatal=1"
LOW = "fflags;nobuffer|flags;low_delay|probesize;32768|analyzeduration;0|max_delay;0"


def sender(stop):
    cmd = ["ffmpeg", "-hide_banner", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-",
           "-c:v", "libx264", "-preset", "ultrafast", "-tune", "zerolatency", "-bf", "0", "-g", "15", "-pix_fmt", "yuv420p",
           "-f", "mpegts", "-flush_packets", "1", "-muxdelay", "0", "-muxpreload", "0", f"udp://127.0.0.1:{PORT}?pkt_size=1316"]
    p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=subprocess.DEVNULL)
    n = 0; t0 = time.monotonic()
    try:
        while not stop.is_set():
            code = L.render_code(L.clock_ms(), scale=9)
            f = np.full((H, W, 3), 110, np.uint8); y = (H - code.shape[0]) // 2; x = (W - code.shape[1]) // 2
            f[y:y + code.shape[0], x:x + code.shape[1]] = code
            p.stdin.write(f.tobytes()); n += 1
            d = t0 + n / FPS - time.monotonic()
            if d > 0: time.sleep(d)
    except Exception:
        pass
    finally:
        try: p.stdin.close()
        except Exception: pass
        p.terminate()


def measure(read_frame, secs):
    out = []; t_end = time.monotonic() + secs; t_start = time.monotonic()
    while time.monotonic() < t_end:
        f = read_frame()
        if f is None: continue
        t = time.monotonic()
        if t - t_start < 2.5: continue                          # warm-up
        sent = L.decode_code(f)
        if sent is not None:
            d = L.delay_ms(sent, t)
            if d is not None: out.append(d)
    return out


def cv_variant(opts, params=None):
    def run():
        if opts is None: os.environ.pop("OPENCV_FFMPEG_CAPTURE_OPTIONS", None)
        else: os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = opts
        cap = cv2.VideoCapture(URL, cv2.CAP_FFMPEG, params or [])
        if not cap.isOpened(): return None
        def rd():
            ok, f = cap.read(); return f if ok else None
        r = measure(rd, SECS); cap.release(); return r
    return run


def ffmpeg_pipe_variant():
    def run():
        cmd = ["ffmpeg", "-hide_banner", "-loglevel", "error", "-fflags", "nobuffer", "-flags", "low_delay", "-probesize", "32768", "-analyzeduration", "0", "-threads", "1",
               "-i", f"udp://127.0.0.1:{PORT}?fifo_size=1000000&overrun_nonfatal=1", "-f", "rawvideo", "-pix_fmt", "bgr24", "-"]
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0); n = W * H * 3
        def rd():
            buf = b""
            while len(buf) < n:
                c = p.stdout.read(n - len(buf))
                if not c: return None
                buf += c
            return np.frombuffer(buf, np.uint8).reshape(H, W, 3)
        r = measure(rd, SECS); p.terminate(); return r
    return run


def gcs_variant():
    from nimbus_gcs import video_source as VS
    def run():
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = LOW
        cap = VS.open_capture(URL)
        if not cap.isOpened(): return None
        def rd():
            ok, f = cap.read(); return f if ok else None
        r = measure(rd, SECS); cap.release(); return r
    return run


VARIANTS = [("OpenCV, no options (library defaults)", cv_variant(None)),
            ("OpenCV, GCS options (nobuffer, low_delay, no probe)", cv_variant(LOW)),
            ("OpenCV, GCS options + 1 decoder thread", cv_variant(LOW, [cv2.CAP_PROP_N_THREADS, 1])),
            ("OpenCV, GCS options + 2 decoder threads", cv_variant(LOW, [cv2.CAP_PROP_N_THREADS, 2])),
            ("OpenCV, GCS options + 4 decoder threads", cv_variant(LOW, [cv2.CAP_PROP_N_THREADS, 4])),
            ("THE GCS ITSELF (video_source.open_capture, now)", gcs_variant()),
            ("ffmpeg program, 1 thread, no buffering (frames piped)", ffmpeg_pipe_variant())]
if __name__ == "__main__":
    print(f"loopback bench: {W}x{H} {FPS} fps x264 zerolatency -> MPEG-TS/UDP on localhost, {SECS:.0f} s per receiver (first 2.5 s ignored)")
    print("receiver                                               readings   delay ms: mean   min   max")
    for name, fn in VARIANTS:
        stop = threading.Event(); th = threading.Thread(target=sender, args=(stop,), daemon=True); th.start(); time.sleep(1.0)
        try: r = fn()
        except Exception as e: r = None; print("   error:", e)
        stop.set(); th.join(timeout=3); time.sleep(0.5)
        if r: print(f"{name:55s} {len(r):5d}          {sum(r)/len(r):6.0f} {min(r):6.0f} {max(r):6.0f}")
        else: print(f"{name:55s}  no frames read")
