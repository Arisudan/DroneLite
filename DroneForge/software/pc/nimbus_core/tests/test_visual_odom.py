import importlib.util, math, os, sys
import numpy as np
import cv2
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
for n in ("rclpy", "rclpy.node", "rclpy.qos", "sensor_msgs", "sensor_msgs.msg", "std_msgs", "std_msgs.msg"):
    if n not in sys.modules: sys.modules[n] = type(sys)(n)
sys.modules["rclpy.node"].Node = object; sys.modules["rclpy.qos"].QoSProfile = object; sys.modules["rclpy.qos"].ReliabilityPolicy = object; sys.modules["rclpy.qos"].HistoryPolicy = object
sys.modules["sensor_msgs.msg"].Image = object; sys.modules["std_msgs.msg"].String = object
spec = importlib.util.spec_from_file_location("visual_odom", os.path.join(ROOT, "gazebo", "vision", "visual_odom.py")); vo = importlib.util.module_from_spec(spec); spec.loader.exec_module(vo)


def _project(pts_anchor, d_fwd, d_left, d_up, yaw_left):
    """Camera moves by (fwd, left, up) in the anchor's body terms and turns left by yaw_left; returns pixel positions of the points."""
    C = np.array([-d_left, -d_up, d_fwd])                                   # camera centre in the anchor camera frame (x right, y down, z forward)
    th = -yaw_left; Rc = np.array([[math.cos(th), 0, math.sin(th)], [0, 1, 0], [-math.sin(th), 0, math.cos(th)]])    # current axes in the anchor frame (rotation about the down axis)
    Xc = (pts_anchor - C) @ Rc                                              # = Rc^T (X - C)
    return np.column_stack([vo.FX * Xc[:, 0] / Xc[:, 2] + vo.CX, vo.FX * Xc[:, 1] / Xc[:, 2] + vo.CY]), Xc[:, 2]


def test_pnp_recovers_the_camera_motion():
    rng = np.random.default_rng(0); pts = np.column_stack([rng.uniform(-2, 2, 120), rng.uniform(-0.8, 0.8, 120), rng.uniform(1.5, 5.0, 120)])
    for mv in ((0.30, 0.10, 0.0, math.radians(5)), (-0.20, -0.25, 0.05, math.radians(-8)), (0.0, 0.0, 0.0, 0.0)):
        px, z = _project(pts, *mv); ok = z > 0.3; px = px + rng.normal(0, 0.3, px.shape)
        r = vo.pnp_motion(pts[ok], px[ok]); assert r is not None
        assert np.allclose(r[:3], mv[:3], atol=0.03) and abs(r[3] - mv[3]) < math.radians(0.6) and r[4] > 60, (mv, r)


def test_outlier_matches_are_rejected():
    rng = np.random.default_rng(1); pts = np.column_stack([rng.uniform(-2, 2, 100), rng.uniform(-0.8, 0.8, 100), rng.uniform(1.5, 5.0, 100)]); px, z = _project(pts, 0.2, 0.0, 0.0, 0.0)
    px[:25] = rng.uniform(0, 640, (25, 2))                                   # 25 % wrong matches
    r = vo.pnp_motion(pts, px); assert r is not None and abs(r[0] - 0.2) < 0.05 and abs(r[1]) < 0.05
