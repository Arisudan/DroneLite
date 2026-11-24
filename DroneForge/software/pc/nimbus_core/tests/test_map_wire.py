import time

import numpy as np

from nimbus_core.map_wire import MapClient, MapServer, WallSkeleton


def _room(n=120, wall=4):
    g = np.full((n, n), -1, np.int8); g[10:110, 10:110] = 0
    g[10:10 + wall, 10:110] = 100; g[106:110, 10:110] = 100; g[10:110, 10:10 + wall] = 100; g[10:110, 106:110] = 100
    return g


def test_thick_walls_become_one_cell_lines():
    out = WallSkeleton().update(_room(), (0.0, 0.0))
    ys, xs = np.nonzero(out == 100)
    assert 300 < len(ys) < 600                      # a 4-wide square wall (~1600 cells) is now a ~400-cell line
    assert (out[12, 20:100] == 100).sum() <= 1      # a column of the top wall has at most one skeleton cell... checked per column below
    cols = [(out[:, c] == 100)[10:60].sum() for c in range(30, 90)]
    assert max(cols) <= 2
    assert (out[(out >= 0) & (out != 100)] == 0).all()   # free space stays free, thick obstacle cells are cleared


def test_small_specks_are_purged():
    g = _room(); g[60, 60] = 100; g[61, 60] = 100
    out = WallSkeleton().update(g, (0.0, 0.0))
    assert out[60, 60] != 100 and out[61, 60] != 100


def test_a_seen_wall_is_held_through_a_short_occlusion_and_released_after_a_long_one():
    s = WallSkeleton(unlock_miss_count=5); g = _room(); s.update(g, (0, 0))
    gone = g.copy(); gone[10:14, 10:110] = 0
    assert (s.update(gone, (0, 0))[10:14, 10:110] == 100).any()          # still there after one clear view
    for _ in range(6):
        out = s.update(gone, (0, 0))
    assert not (out[10:14, 10:110] == 100).any()                          # released after enough clear views


def test_reset_clears_the_wall_lock():
    s = WallSkeleton(); s.update(_room(), (0, 0)); s.reset()
    empty = np.zeros((120, 120), np.int8)
    assert not (s.update(empty, (0, 0)) == 100).any()


def _wait(f, t=3.0):
    t0 = time.time()
    while time.time() - t0 < t:
        if f(): return True
        time.sleep(0.02)
    return False


def test_server_to_client_round_trip_with_pose_and_late_join():
    resets = []
    srv = MapServer(port=0, host="127.0.0.1", on_reset=lambda: resets.append(1) or "OK: reset"); srv.start()
    g = _room(); thin = WallSkeleton().update(g, (0, 0))
    srv.publish("map_raw", g, 0.05, (-3.0, -3.0), pose=(1.0, 2.0, 0.5)); srv.publish("map_thin", thin, 0.05, (-3.0, -3.0), pose=(1.0, 2.0, 0.5))
    c = MapClient(); c.start("127.0.0.1", srv.port)            # joins after the packets: must get the cached ones at once
    assert _wait(lambda: c.raw is not None and c.thin is not None)
    assert c.raw["grid"].shape == (120, 120) and (c.raw["grid"] == g).all() and (c.thin["grid"] == thin).all()
    assert c.raw["res"] == 0.05 and c.raw["ox"] == -3.0 and c.pose == (1.0, 2.0, 0.5)
    assert c.reset() and _wait(lambda: c.reply.startswith("OK")) and resets == [1] and c.raw is None
    c.stop(); srv.stop()


def test_client_reconnects_when_the_server_appears_later():
    import socket
    s = socket.socket(); s.bind(("127.0.0.1", 0)); port = s.getsockname()[1]; s.close()
    c = MapClient(); c.start("127.0.0.1", port); time.sleep(0.3); assert c.raw is None and c.state == "connecting"
    srv = MapServer(port=port, host="127.0.0.1"); srv.start(); srv.publish("map_raw", _room(), 0.05, (0, 0))
    assert _wait(lambda: c.raw is not None, 5.0)
    c.stop(); srv.stop()
