import math
import numpy as np
from nimbus_core.occupancy_map import OccupancyMap
from nimbus_core.roam import Roamer, RoamConfig, Pose, SpeedProfile, yaw_stick, wrap


def test_speed_profile_respects_nimbus_limits():
    p = SpeedProfile(0.35, 0.8); vs = []; acc = []; last = 0.0
    for _ in range(2000):
        v = p.step(0.35, 0.01); vs.append(v); acc.append(abs(v - last) / 0.01); last = v
    assert abs(vs[-1] - 0.35) < 0.01                    # reaches the cruise speed
    assert max(acc) <= 0.35 + 1e-6                      # never exceeds the acceleration limit
    assert vs[10] < 0.01 and vs[50] < 0.12              # jerk limit: it ramps (v = 0.4 t^2 at 0.8 m/s^3), it does not jump


def test_yaw_law_is_nimbus():
    s, e = yaw_stick(1.0, 0.0); assert abs(s - 0.35) < 1e-9 and abs(e - 1.0) < 1e-9     # 0.65 * 1.0 clamps to 0.35
    s, _ = yaw_stick(0.2, 0.0); assert abs(s - 0.13) < 1e-9
    s, _ = yaw_stick(0.03, 0.0); assert s == 0.0                                       # inside the dead zone
    assert abs(wrap(3 * math.pi) - math.pi) < 1e-9 or abs(wrap(3 * math.pi) + math.pi) < 1e-9


def _fp_wall(dist, nx=48, vox=0.125):
    return {"vox": vox, "nx": nx, "nz": 48, "cells": [[ix, int(dist / vox), 9] for ix in range(nx)]}


def test_map_accumulates_and_roamer_plans():
    r = Roamer(RoamConfig()); r.begin(0.0, 0.0, 0.0)
    for k in range(8):
        r.push_pose(0.1 * k, 0.0, 0.0, k * math.pi / 4); obs = {"rows": 6, "cols": 8, "min_m": [3.0] * 48, "floorplan": _fp_wall(3.0), "stamp": 0.1 * k + 0.05, "latency_ms": 0}
        r.on_depth(obs, 0.0, 0.1 * k + 0.05)
    assert r.map.frames >= 6 and r.map.occupied().sum() > 20
    res = r.plan(Pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    assert res not in (None, "none")                                                    # there is a frontier to go to
    path, goal = res; assert len(path) >= 2


def test_stale_pose_stops_the_sticks():
    r = Roamer(RoamConfig()); r.begin(0.0, 0.0, 0.0)
    c = r.step(1.0, 1.0, 0.05, Pose(0, 0, 0, 0, 0, 0, age=2.0))
    assert (c.pitch, c.roll, c.yaw) == (0.0, 0.0, 0.0) and "STOP" in c.text


def test_no_depth_means_no_forward_motion():
    r = Roamer(RoamConfig()); r.begin(0.0, 0.0, 0.0); r.stage = "explore"
    r.path = [(0.0, 0.0), (3.0, 0.0)]; r.goal = (3.0, 0.0); r.last_plan = 10.0     # a path exists but no depth has arrived
    c = r.step(10.1, 10.1, 0.05, Pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    assert c.pitch == 0.0


def test_roamer_plans_on_an_external_slam_grid():
    from nimbus_core.grid_map import GridMap
    g = GridMap(); H, W = 120, 160; p = np.full((H, W), -1, np.int8); p[30:90, 30:100] = 0; p[30:90, 30:33] = 100; p[30:33, 30:100] = 100; p[87:90, 30:100] = 100
    g.set_grid(W, H, 0.05, -4.0, -3.0, p.ravel())                      # a room with one open side toward unknown space on the right
    r = Roamer(RoamConfig(), g); r.begin(0.0, 0.0, 0.0)
    res = r.plan(Pose(-1.5, 0.0, 0.0, 0.0, 0.0, 0.0))
    assert res not in (None, "none", "wait"); path, goal = res; assert goal[0] > 0.0   # the frontier is on the open side
    g.mark_obstacle(-1.5, 0.0, 0.0, 1.0)                                 # a bump wall ahead is remembered across new grids
    before = int(g.occupied().sum()); g.set_grid(W, H, 0.05, -4.0, -3.0, p.ravel()); assert int(g.occupied().sum()) == before


def test_empty_grid_waits_for_the_map():
    from nimbus_core.grid_map import GridMap
    r = Roamer(RoamConfig(), GridMap()); r.begin(0.0, 0.0, 0.0); assert r.plan(Pose(0, 0, 0, 0, 0, 0)) == "wait"


# ---- 2026-11-21: stuck-near-a-wall fixes (escape the safety margin, boxed-in is not "explored", walls beside the drone from the map)
def _grid_room(walls, unknown_right=False, w=80, h=80, res=0.05):
    """An RTAB-Map style grid (0 free, 100 occupied, -1 unknown), origin (-2, -2): a 4 x 4 m room with the given wall cells."""
    from nimbus_core.grid_map import GridMap
    d = np.zeros((h, w), np.int16)
    d[0, :] = d[-1, :] = d[:, 0] = d[:, -1] = 100
    for (r0, r1, c0, c1) in walls: d[r0:r1, c0:c1] = 100
    if unknown_right: d[1:-1, 60:-1] = -1
    m = GridMap(); m.set_grid(w, h, res, -2.0, -2.0, d.ravel().tolist()); return m


def test_side_clearance_sees_a_wall_beside_the_drone():
    m = _grid_room([(0, 80, 49, 51)]); r = Roamer(RoamConfig(), m)                  # a wall at x = +0.45..0.55 m
    left, right = r.map_side_clearance(Pose(0.15, 0.0, math.pi / 2, 0, 0, 0))       # facing +y: the wall is on the RIGHT, ~0.3 m away
    assert right < 0.4 and left > 0.6


def test_inside_the_safety_margin_the_plan_is_an_escape():
    m = _grid_room([(0, 80, 49, 51)], unknown_right=False); r = Roamer(RoamConfig(), m)
    res = r.plan(Pose(0.38, 0.0, 0.0, 0, 0, 0))                                      # 0.07 m from the wall: inside the 0.25 m margin
    assert isinstance(res, tuple) and res[0] == "escape"
    end = res[1][-1]; assert end[0] < 0.38 - 0.05                                     # it leads away from the wall


def test_boxed_in_is_not_explored():
    # the drone's corner is closed off by a wall with a gap narrower than the safety margin; unknown space waits behind it
    m = _grid_room([(0, 80, 19, 21)], unknown_right=True); m.L[38:42, 19:21] = -3.0      # a 0.2 m gap (passing needs 0.5 m with the 0.25 m margin); the drone's corner is the small part
    r = Roamer(RoamConfig(), m)
    assert r.plan(Pose(-1.6, 0.0, 0.0, 0, 0, 0)) == "boxed"
