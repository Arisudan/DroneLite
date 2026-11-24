#!/usr/bin/env python3
"""Map stream for the GCS (Flop's map_thinning_node + tcp_map_streamer_node in one, without the PX4 parts).
RTAB-Map /map -> wall skeleton /map_thin (nimbus_core.map_wire.WallSkeleton) -> TCP port 5765 to the GCS (MapServer), with the drone pose from TF map->base_link.
The GCS "Clear map" button sends RESET: this node calls /rtabmap/reset and clears the wall lock.
Run (done by run_house.sh): python3 vision/map_stream_node.py [--port 5765]      Needs the sourced ROS 2 Jazzy environment."""
import math
import os
import sys
import threading

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "software", "pc"))
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener

from nimbus_core.map_wire import DEFAULT_PORT, MapServer, WallSkeleton


class MapStream(Node):
    def __init__(self, port: int) -> None:
        super().__init__("droneforge_map_stream")
        self.thin = WallSkeleton(); self.lock = threading.Lock()
        self.tf = Buffer(); self.tfl = TransformListener(self.tf, self)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(OccupancyGrid, "/map_thin", qos)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, qos)
        self.reset_cli = self.create_client(Empty, "/rtabmap/reset")
        self.server = MapServer(port=port, on_reset=self.do_reset); self.server.start()
        self.create_timer(0.5, self.send_pose)            # the marker moves between map updates (RTAB-Map publishes /map only every few seconds)
        self.last = None
        self.get_logger().info(f"map stream on tcp port {self.server.port}")

    def pose(self):
        try:
            t = self.tf.lookup_transform("map", "base_link", rclpy.time.Time())
            q = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            return (t.transform.translation.x, t.transform.translation.y, yaw)
        except Exception:
            return None

    def on_map(self, m: OccupancyGrid) -> None:
        w, h = m.info.width, m.info.height
        if w == 0 or h == 0:
            return
        grid = np.array(m.data, dtype=np.int8).reshape(h, w); org = (m.info.origin.position.x, m.info.origin.position.y); res = m.info.resolution
        with self.lock:
            thin = self.thin.update(grid, org)
        out = OccupancyGrid(); out.header = m.header; out.info = m.info; out.data = thin.flatten().tolist(); self.pub.publish(out)
        self.last = (grid, thin, res, org)
        self.send(self.pose())

    def send(self, pose) -> None:
        if self.last is None:
            return
        grid, thin, res, org = self.last
        self.server.publish("map_raw", grid, res, org, pose); self.server.publish("map_thin", thin, res, org, pose)

    def send_pose(self) -> None:
        self.send(self.pose())

    def do_reset(self) -> str:
        msg = []
        if self.reset_cli.wait_for_service(timeout_sec=2.0):
            try: self.reset_cli.call_async(Empty.Request()); msg.append("rtabmap reset")
            except Exception as e: return f"ERR: rtabmap reset failed: {e}"
        else:
            msg.append("rtabmap reset service not available")
        with self.lock:
            self.thin.reset()
        self.last = None; msg.append("wall lock cleared")
        return ("OK: " if "not available" not in msg[0] else "ERR: ") + "; ".join(msg)


def main() -> None:
    port = DEFAULT_PORT
    if "--port" in sys.argv:
        port = int(sys.argv[sys.argv.index("--port") + 1])
    rclpy.init(); node = MapStream(port); ex = MultiThreadedExecutor(); ex.add_node(node)
    try: ex.spin()
    except KeyboardInterrupt: pass
    finally: node.server.stop(); node.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    main()
