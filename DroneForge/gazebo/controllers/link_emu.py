"""Emulation of what the real link gives us, shared by the TF/odom publisher (pose) and the explorer (sticks).
real=False: exact simulator state, sticks at once. real=True: FC EKF pose that is noisy, 25 Hz, ~0.1 s old and drifting (flow dead reckoning);
sticks delayed ~0.15 s and quantised to the CRSF steps (820 per unit)."""
import collections, math


class LinkEmu:
    def __init__(self, real, rng):
        self.real = real; self.rng = rng; self.true = collections.deque(maxlen=300); self.dx = self.dy = self.dyaw = 0.0
        self.pose = None; self.t_pose = 0.0; self.sim_pose = 0.0; self.t_upd = 0.0; self.q = collections.deque()
        self.pose_delay = 0.10; self.rate = 25.0; self.cmd_delay = 0.15; self.drift = 0.03; self.noise_xy = 0.01; self.noise_v = 0.02; self.noise_yaw = 0.01

    def on_true(self, now, sim_t, x, y, yaw, vf, vl, tilt):
        """Returns True when a new pose was produced. pose = (x, y, yaw, vf, vl, tilt); sim_pose = the simulation time it describes."""
        self.true.append((now, sim_t, x, y, yaw, vf, vl, tilt))
        if not self.real: self.pose = (x, y, yaw, vf, vl, tilt); self.t_pose = now; self.sim_pose = sim_t; return True
        if now - self.t_upd < 1.0 / self.rate: return False
        dt = now - self.t_upd if self.t_upd else 0.04; self.t_upd = now
        s = min(self.true, key=lambda h: abs(h[0] - (now - self.pose_delay)))
        sp = math.hypot(s[5], s[6]); g = self.rng.gauss
        self.dx += g(0, self.drift * sp * dt + 0.002 * math.sqrt(dt)); self.dy += g(0, self.drift * sp * dt + 0.002 * math.sqrt(dt)); self.dyaw += g(0, 0.003 * math.sqrt(dt))
        self.pose = (s[2] + self.dx + g(0, self.noise_xy), s[3] + self.dy + g(0, self.noise_xy), s[4] + self.dyaw + g(0, self.noise_yaw), s[5] + g(0, self.noise_v), s[6] + g(0, self.noise_v), s[7])
        self.t_pose = now; self.sim_pose = s[1]; return True

    def age(self, now): return now - self.t_pose

    def out(self, now, d):
        if self.real:
            q = lambda s: round(max(-1.0, min(1.0, s)) * 820) / 820.0
            d = dict(d); [d.__setitem__(k, q(d[k])) for k in ("pitch", "roll", "yaw") if k in d]; self.q.append((now + self.cmd_delay, d))
        else: self.q.append((now, d))

    def due(self, now):
        o = []
        while self.q and self.q[0][0] <= now: o.append(self.q.popleft()[1])
        return o
