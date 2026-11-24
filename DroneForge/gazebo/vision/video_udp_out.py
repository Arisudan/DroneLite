#!/usr/bin/env python3
"""The simulated O4 camera to the GCS the way CosmoStreamer sends it: raw H.264 over UDP (no container), so the GCS Video page opens it with `udp://0.0.0.0:3000`
exactly like the real feed. SIMULATION ONLY. Input: the undelayed camera topic, so what the GCS shows
is the camera itself plus the real encode / UDP / decode path (2026-11-21: the owner gives the O4 as <= 30 ms at a fixed 30 fps; the RTSP measurement of
2026-11-20 gave ~172 ms, kept in config/o4_latency.json). The autonomy reads /droneforge/camera/image_delayed (camera_feed_latency.py, 30 ms in sim.sh)."""
import argparse, subprocess, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


class VideoOut(Node):
    def __init__(self, a):
        super().__init__("droneforge_video_udp_out")
        self.a = a; self.ff = None; self.size = None; self.n = 0; self.cc = None; self.sock = None
        self.create_subscription(Image, a.topic, self.on_img, QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST))

    AUD = b"\x00\x00\x00\x01\x09\xf0"       # access-unit delimiter sent AFTER each frame: the receiver's parser knows at once that the frame is complete (saves one frame of delay)

    def _start_av(self, w, h):
        """In-process encoder (PyAV): one packet per frame, sent straight to UDP. 2026-11-21: ~1 ms encode, 3 ms to a GCS frame."""
        import av, socket
        from fractions import Fraction
        name = "h264_nvenc" if self.a.encoder == "nvenc" else "libx264"
        cc = av.CodecContext.create(name, "w"); cc.width, cc.height, cc.pix_fmt = w, h, "yuv420p"
        cc.time_base = Fraction(1, self.a.fps); cc.framerate = Fraction(self.a.fps, 1); cc.gop_size = self.a.fps; cc.max_b_frames = 0; cc.bit_rate = 4_000_000
        cc.options = {"preset": "p1", "tune": "ull", "zerolatency": "1", "delay": "0", "forced-idr": "1"} if name == "h264_nvenc" else {"preset": "ultrafast", "tune": "zerolatency", "x264-params": "repeat-headers=1"}
        cc.open(); self.cc = cc; self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); self.size = (w, h); self.pts = 0; self._av = av
        self.get_logger().info(f"camera {w}x{h} @ {self.a.fps} fps -> raw H.264 ({name}, in-process) udp://{self.a.host}:{self.a.port} (GCS Video page: udp://0.0.0.0:{self.a.port})")

    def _send_av(self, a):
        fr = self._av.VideoFrame.from_ndarray(a, format="rgb24").reformat(format="yuv420p"); fr.pts = self.pts; self.pts += 1
        for p in self.cc.encode(fr):
            b = bytes(p) + self.AUD
            for o in range(0, len(b), 1316): self.sock.sendto(b[o:o + 1316], (self.a.host, self.a.port))

    def _start(self, w, h):
        if self.a.encoder == "nvenc":      # GPU encoder, ultra-low-latency: ~2-4 ms per frame, no frame queue
            enc = ["-c:v", "h264_nvenc", "-preset", "p1", "-tune", "ull", "-zerolatency", "1", "-delay", "0", "-rc", "cbr", "-b:v", "4M", "-g", str(self.a.fps), "-bf", "0", "-forced-idr", "1"]
        else:
            enc = ["-c:v", "libx264", "-preset", "ultrafast", "-tune", "zerolatency", "-x264-params", "repeat-headers=1", "-g", str(self.a.fps), "-bf", "0", "-b:v", "4M"]
        cmd = ["ffmpeg", "-loglevel", "error", "-fflags", "nobuffer", "-use_wallclock_as_timestamps", "1", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}", "-i", "-",
               *enc, "-pix_fmt", "yuv420p", "-flush_packets", "1", "-bsf:v", "dump_extra", "-f", "h264", f"udp://{self.a.host}:{self.a.port}?pkt_size=1316"]
        self.ff = subprocess.Popen(cmd, stdin=subprocess.PIPE); self.size = (w, h)
        self.get_logger().info(f"camera {w}x{h} -> raw H.264 udp://{self.a.host}:{self.a.port} (GCS Video page: udp://0.0.0.0:{self.a.port})")

    def on_img(self, m):
        ch = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(m.encoding)
        if ch is None: return
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * ch].reshape(m.height, m.width, ch)
        if m.encoding.startswith("bgr"): a = a[..., 2::-1]
        a = np.ascontiguousarray(a[..., :3])
        if self.a.stamp:                                                           # test only: wall-clock ms as 32 black/white 10x10 blocks in the top-left (tools: measure the glass-to-GCS delay)
            ms = int(time.time() * 1000) & 0xFFFFFFFF; a = a.copy()
            for i in range(32): a[0:10, i * 10:(i + 1) * 10] = 255 if (ms >> i) & 1 else 0
        if self.a.backend == "av":
            if self.cc is None or self.size != (m.width, m.height):
                try: self._start_av(m.width, m.height)
                except ImportError: self.get_logger().warn("PyAV missing (pip install av): falling back to the ffmpeg encoder"); self.a.backend = "ffmpeg"
            if self.a.backend == "av":
                self._send_av(a); self.n += 1; return
        if self.ff is None or self.size != (m.width, m.height):
            if self.ff: self.ff.kill()
            self._start(m.width, m.height)
        try: self.ff.stdin.write(a.tobytes()); self.n += 1
        except (BrokenPipeError, OSError):
            self.get_logger().warn("ffmpeg stopped; restarting"); self.ff = None


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--topic", default="/droneforge/camera/image_raw")
    ap.add_argument("--stamp", action="store_true", help="burn a wall-clock timestamp into the picture (latency measurement)")
    ap.add_argument("--host", default="127.0.0.1"); ap.add_argument("--port", type=int, default=3000); ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--encoder", choices=("nvenc", "x264"), default="nvenc"); ap.add_argument("--backend", choices=("av", "ffmpeg"), default="av")
    a = ap.parse_args(); rclpy.init(); n = VideoOut(a)
    try: rclpy.spin(n)
    except (KeyboardInterrupt, Exception): pass
    finally:
        if n.ff: n.ff.kill()


if __name__ == "__main__":
    main()
