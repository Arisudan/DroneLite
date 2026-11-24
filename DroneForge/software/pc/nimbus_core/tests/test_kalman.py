import math
import numpy as np
from nimbus_core.kalman import LateralKF, VerticalKF, Plant, FlowParams


def _sim(kf, seconds=20.0, noise=True, seed=3, tilt_fn=lambda t: 0.0):
    """Truth: the same plant the filter assumes, flown straight (yaw 0), flow at 25 Hz, attitude+gyro at 50 Hz."""
    rng = np.random.default_rng(seed); dt = 0.01; t = 0.0; x = y = vx = vy = 0.0; pitch = 0.0; h = 0.6; kf.flying = True; err = []
    for i in range(int(seconds / dt)):
        t += dt; pitch = tilt_fn(t); a = kf.plant.k_f * pitch - kf.plant.d_f * vx; vx += a * dt; x += vx * dt
        kf.predict(t, 0.0, True)
        if i % 2 == 0: kf.update_attitude(0.0 + (rng.normal(0, 0.02) if noise else 0), pitch + (rng.normal(0, 0.02) if noise else 0)); kf.update_gyro(rng.normal(0, 0.05) if noise else 0, rng.normal(0, 0.05) if noise else 0)
        if i % 4 == 0: kf.update_flow(vx + (rng.normal(0, 0.08) if noise else 0), (rng.normal(0, 0.08) if noise else 0), h)
        err.append((kf.pos[0] - x, kf.vel_world[0] - vx))
    return x, vx, np.array(err)


def test_velocity_and_position_are_tracked_with_noisy_flow():
    kf = LateralKF(Plant(9.81, 9.81, 0.4, 0.4), FlowParams(r0=0.01))
    x, vx, err = _sim(kf, tilt_fn=lambda t: 0.04 if 2.0 < t < 6.0 else 0.0)
    assert abs(kf.vel_world[0] - vx) < 0.15                    # velocity from flow
    assert np.abs(err[-300:, 0]).mean() < 1.0                  # position error stays bounded (dead reckoning drifts, but not wildly)


def test_nis_gate_rejects_an_outlier_flow():
    kf = LateralKF(); kf.flying = True; kf.predict(0.0, 0.0, True)
    for k in range(50): kf.predict(0.02 * k, 0.0, True); kf.update_flow(0.0, 0.0, 0.6)
    before = kf.x[2]; kf.update_flow(5.0, 0.0, 0.6)           # a 5 m/s spike while hovering
    assert kf.rejected >= 1 and abs(kf.x[2] - before) < 0.2


def test_flow_is_ignored_when_quality_is_low_or_not_flying():
    kf = LateralKF(); kf.flying = True; kf.predict(0.0, 0.0, True); assert not kf.update_flow(0.1, 0.0, 0.6, quality=5)
    kf.flying = False; assert not kf.update_flow(0.1, 0.0, 0.6)


def test_vertical_filter_tilt_compensates_the_range():
    v = VerticalKF()
    for k in range(100): v.predict(0.02 * k); v.update_range(0.6 / math.cos(0.2), 0.2, 0.0)
    assert abs(v.x[0] - 0.6) < 0.03


def test_filter_recovers_when_the_drone_is_blocked_while_tilted():
    """Tilt command pushes the model forward but a wall holds the drone (true speed 0): the gate must not lock out the flow sensor."""
    kf = LateralKF(Plant(9.81, 9.81, 0.4, 0.4), FlowParams(r0=0.01)); kf.flying = True; t = 0.0
    for i in range(1500):
        t += 0.02; kf.predict(t, 0.0, True)
        kf.update_attitude(0.0, 0.05); kf.update_gyro(0.0, 0.0)                    # 2.9 deg forward tilt, held against the wall
        if i % 2 == 0: kf.update_flow(0.0, 0.0, 0.6)                                # flow says: not moving
    assert abs(kf.vel_world[0]) < 0.4


def test_visual_position_updates_stop_the_flow_drift():
    """Flow with a 6 % scale error drifts; position fixes against a keyframe (noise 3 cm) must keep the error small."""
    def run(use_vo, seed=2):
        rng = np.random.default_rng(seed); kf = LateralKF(Plant(), FlowParams(r0=0.0066)); kf.flying = True; t = 0.0; x = vx = 0.0; err = []
        for i in range(60 * 50):
            t += 0.02; vx = 0.3 * math.sin(0.4 * t); x += vx * 0.02                       # slow back-and-forth motion, true state
            kf.predict(t, 0.0, True); kf.update_attitude(0.0, 0.0); kf.update_gyro(0.0, 0.0)
            if i % 2 == 0: kf.update_flow(1.06 * vx + rng.normal(0, 0.08), rng.normal(0, 0.08), 0.6)
            if use_vo and i % 7 == 0: kf.update_position(x + rng.normal(0, 0.03), 0.0 + rng.normal(0, 0.03), 0.03 ** 2 + 0.02 ** 2)
            err.append(abs(kf.pos[0] - x))
        return float(np.mean(err[-500:]))
    assert run(True) < 0.5 * run(False) and run(True) < 0.08
