#!/usr/bin/env python3
"""The DJI O4 feed as the PC actually receives it: delayed (and slightly jittered) copies of the simulated camera picture.
/droneforge/camera/image_raw  (the camera, no delay)  ->  /droneforge/camera/image_delayed  (clean, delayed: what perception, RTAB-Map and the VO read)
                                                       ->  /droneforge/camera/feed          (the same picture with a latency read-out burned in: shown in the Gazebo window)
Latency model: delay = mean + Gaussian jitter (never below 0), a rare dropped frame. Measure the real mean with the GCS latency test (tools_scripts/video_latency_bench.py).
Stamps: a real receiver does not know when a frame was captured; it only knows when it arrived and its calibrated mean latency. So the delayed image carries
stamp = capture time + (actual delay - mean delay): the calibrated estimate, off by the jitter (this is what the pose lookup in RTAB-Map and the VO then sees)."""
import argparse, collections, time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from builtin_interfaces.msg import Time


def add(stamp, seconds):
    ns = stamp.sec * 10**9 + stamp.nanosec + int(seconds * 1e9); t = Time(); t.sec, t.nanosec = ns // 10**9, ns % 10**9; return t


class Feed(Node):
    def __init__(self, a):
        super().__init__("droneforge_camera_feed_latency", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)]); self.a = a; self.rng = np.random.default_rng(a.seed); self.q = collections.deque(); self.n = 0; self.shown = collections.deque(maxlen=30)
        self.sub = self.create_subscription(Image, a.topic, self.on_img, qos_profile_sensor_data)
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=5)
        self.pub = self.create_publisher(Image, "/droneforge/camera/image_delayed", qos); self.pub_view = self.create_publisher(Image, "/droneforge/camera/feed", qos)
        self.timer = self.create_timer(0.005, self.tick)
        self.get_logger().info(f"O4 feed latency: mean {a.latency_ms:.0f} ms, jitter {a.jitter_ms:.0f} ms, drop {a.drop*100:.1f} %")

    def on_img(self, m):
        if self.rng.random() < self.a.drop: return
        d = max(0.0, self.rng.normal(self.a.latency_ms, self.a.jitter_ms)) / 1000.0
        t = self.now(); self.q.append((t + d, t, m)); self.n += 1                                # (due, arrival, message)

    def now(self): return self.get_clock().now().nanoseconds * 1e-9              # SIMULATION time: the O4's latency is physical, so it must not change with the speed of this machine

    def tick(self):
        now = self.now()
        while self.q and self.q[0][0] <= now:
            _, t_arr, m = self.q.popleft(); mean = self.a.latency_ms / 1000.0; d = now - t_arr                       # the TRUE age of this picture now, in simulation seconds (planned delay + processing + queueing)
            out = Image(); out.header = m.header; out.header.stamp = add(m.header.stamp, d - mean); out.header.frame_id = "camera_optical"
            out.height, out.width, out.encoding, out.is_bigendian, out.step, out.data = m.height, m.width, m.encoding, m.is_bigendian, m.step, m.data
            self.pub.publish(out); self.shown.append(d)
            img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3).copy()      # rgb8
            txt = f"O4 feed  latency {d * 1000:4.0f} ms   (mean {self.a.latency_ms:.0f}, jitter {self.a.jitter_ms:.0f})   avg {np.mean(self.shown) * 1000:3.0f} ms"
            cv2.rectangle(img, (0, 0), (m.width, 26), (0, 0, 0), -1); cv2.putText(img, txt, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (235, 235, 235), 1, cv2.LINE_AA)
            bar = int(min(1.0, d / 0.5) * (m.width - 16)); cv2.rectangle(img, (8, 28), (8 + bar, 33), (255, 203, 169)[::-1], -1)        # bar = latency, full width = 0.5 s
            v = Image(); v.header = out.header; v.height, v.width, v.encoding, v.is_bigendian, v.step = m.height, m.width, "rgb8", 0, m.width * 3; v.data = img.tobytes(); self.pub_view.publish(v)


def main():
    p = argparse.ArgumentParser(); p.add_argument("--latency-ms", dest="latency_ms", type=float, default=150.0); p.add_argument("--jitter-ms", dest="jitter_ms", type=float, default=25.0)
    p.add_argument("--drop", type=float, default=0.01); p.add_argument("--seed", type=int, default=1); p.add_argument("--topic", default="/droneforge/camera/image_raw"); a = p.parse_args()
    rclpy.init(); n = Feed(a)
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass


if __name__ == "__main__": main()
