"""Simulation harness for the FC-side height hold (firmware/.../flight/height_hold_core.c, compiled to libhh.so by build.sh) on a plant fitted to the real drone.
Plant physics as in nimbus_pc/hover/sim.py (thrust ~ u^2 * (V/V0)^2, ground effect, motor lag) but with the FC-SIDE signal path: the controller runs at 50 Hz on the FC's own
range/EKF data, so there is no radio delay; only sensor age, EKF velocity lag, the command->ESC delay and the motor lag."""
import ctypes, math, os, random
G = 9.81
HERE = os.path.dirname(os.path.abspath(__file__))
NAMES = ['kp_z', 'kv_z', 'ki_z', 'i_max', 'vz_lim', 'climb_peak_frac', 'descent_peak_frac', 'thr_idle', 'thr_max', 'drop_max', 'overshoot_m', 'overshoot_vz', 'overshoot_cut', 'height_min', 'height_max',
         'base_tau', 'base_follow_max', 'lift_vz', 'lift_m', 'takeoff_start_frac', 'takeoff_creep_per_s', 'takeoff_cap_frac', 'takeoff_fail_s', 'land_rate', 'land_slow_rate', 'land_slow_below',
         'touch_m', 'touch_s', 'idle_ramp_per_s', 'takeoff_ramp_per_s', 'cap_land_s', 'lift_base_boost', 'takeoff_vz_max', 'takeoff_vz_back', 'takeoff_handover_margin', 'takeoff_fast_per_s', 'takeoff_closed']

class FcCore:
    def __init__(self, lib=None):
        L = ctypes.CDLL(lib or os.path.join(HERE, "libhh.so"))
        for n in ("hh_update", "hh_integ", "hh_base", "hh_zref"): getattr(L, n).restype = ctypes.c_float
        L.hh_params.restype = ctypes.POINTER(ctypes.c_float)
        for n, a in (("hh_engage", 3), ("hh_takeoff", 3), ("hh_set_target", 1), ("hh_follow", 3), ("hh_update", 4)): getattr(L, n).argtypes = [ctypes.c_float] * a
        self.L = L; L.hh_defaults(); self.p = L.hh_params()
    def set(self, **kw):
        for k, v in kw.items(): self.p[NAMES.index(k)] = v
    def get(self, k): return self.p[NAMES.index(k)]

class Plant:
    def __init__(self, hover=0.72, ge=0.25, ge_h=0.1, tau=0.06, act_delay=0.03, sensor_delay=0.06, vz_lag=0.05, vz_sigma=0.04, v0=13.8, sag=0.005, ground_range=0.07, bias=0.09, seed=1, mass=1.0, drag=0.15, tether_g=0.0, tether_h=0.0, vz_bias=0.0, vz_bias_h=0.12):
        self.__dict__.update(locals()); del self.__dict__["self"]
        self.rng = random.Random(seed); self.z = 0.0; self.vz = 0.0; self.thr_eff = 0.0; self.t = 0.0; self.pipe = []; self.hist = []; self.max_impact = 0.0; self.crashed = False; self.cmd = 0.0
        self.vz_f = 0.0
    def step(self, dt, cmd):
        self.pipe.append((self.t + self.act_delay, cmd))
        while self.pipe and self.pipe[0][0] <= self.t: self.cmd = self.pipe.pop(0)[1]
        self.thr_eff += (self.cmd - self.thr_eff) * dt / self.tau
        vb = max(self.v0 - self.sag * self.t, 9.0)
        u = max(self.thr_eff - 0.05, 0.0) / max(self.hover - 0.05, 1e-6)
        thrust = u * u * (vb / self.v0) ** 2 / self.mass
        if self.ge: thrust *= 1.0 + self.ge * max(0.0, 1.0 - self.z / self.ge_h)
        az = G * (thrust - 1.0) - self.drag * self.vz
        if self.tether_g and self.z < self.tether_h: az -= self.tether_g * G            # something holds the drone down near the floor (a snagging tether)
        if self.z <= 0.0 and self.vz <= 0.0 and az <= 0.0: self.z = 0.0; self.vz = 0.0
        else:
            self.vz += az * dt; self.z += self.vz * dt
            if self.z < 0.0:
                self.max_impact = max(self.max_impact, -self.vz)
                if -self.vz > 2.0: self.crashed = True
                self.z = 0.0; self.vz = 0.0
        self.t += dt
        self.hist.append((self.t, self.z, self.vz)); 
        if len(self.hist) > 400: self.hist.pop(0)
        return vb
    def sensed(self):
        """(range reading incl. ground offset and over-read, EKF vertical speed) as the FC sees them now."""
        def at(td):
            for (t, z, v) in reversed(self.hist):
                if t <= self.t - td: return z, v
            return self.hist[0][1], self.hist[0][2]
        zs, _ = at(self.sensor_delay); _, vs = at(0.05)
        rb = zs
        bias = self.bias * (min(max((rb - 0.06) / 0.14, 0.0), 1.0) if rb < 0.32 else max(0.0, (0.45 - rb) / 0.13)) if rb < 0.45 else 0.0
        rng = zs + self.ground_range + bias + self.rng.gauss(0, 0.01)
        self.vz_f += (vs - self.vz_f) * 0.02 / max(self.vz_lag, 0.02)
        vb = self.vz_bias if (self.z < self.vz_bias_h and self.thr_eff > 0.05) else 0.0      # the FC's EKF speed drifts while the range reads its floor value (measured +0.2 m/s on the floor with the motors spinning)
        return rng, self.vz_f + vb + self.rng.gauss(0, self.vz_sigma)

def fly(plant, core, f_hover_ref=0.66, target=0.5, hold_s=14.0, step_to=None, land=True, dt=0.002, fc_dt=0.02, t_max=90.0, cap_land_s=0.0):
    """take off from the floor (FC creep), hold, optionally change height, land. Returns metrics."""
    L = core.L; ground = None
    # the FC learns the ground reading while armed on the floor (stick at minimum, still)
    gz = []
    for _ in range(40):
        plant.step(dt, 0.0)
    for _ in range(30): plant.step(dt, 0.0); rng, _v = plant.sensed(); gz.append(rng)
    ground = sum(gz) / len(gz)
    L.hh_takeoff(ctypes.c_float(f_hover_ref), ctypes.c_float(target), ctypes.c_float(plant.v0))
    out = {"t": [], "z": [], "thr": [], "vz": [], "mode": []}
    t_next = 0.0; cmd = 0.0; t_air = None; t_land = None; capt = 0.0; auto_landed = False
    zmax = 0.0; vzmax = 0.0; state = "takeoff"; t_hold0 = None; hold_err = []
    while plant.t < t_max:
        if plant.t >= t_next:
            t_next += fc_dt
            rng, vzs = plant.sensed(); zf = rng - ground
            vb = max(plant.v0 - plant.sag * plant.t, 9.0)
            cmd = L.hh_update(ctypes.c_float(fc_dt), ctypes.c_float(zf), ctypes.c_float(vzs), ctypes.c_float(vb))
            mode = L.hh_mode()
            if mode == 0 and t_air is None: t_air = plant.t; t_hold0 = plant.t
            if cap_land_s > 0 and mode == 0 and cmd >= core.get("thr_max") - 0.005 and vzs < 0.1:
                capt += fc_dt
                if capt >= cap_land_s and not auto_landed: L.hh_land(); auto_landed = True
            else: capt = max(capt - fc_dt, 0.0)
            if step_to and t_air is not None and plant.t > t_air + hold_s * 0.5 and not getattr(fly, "_stepped", False) and state == "takeoff": L.hh_set_target(ctypes.c_float(step_to)); state = "step"
            if land and t_air is not None and plant.t > t_air + hold_s and t_land is None and not auto_landed: L.hh_land(); t_land = plant.t
            if t_hold0 is not None and t_land is None and plant.t > t_hold0 + 9.0: hold_err.append(abs(plant.z - (step_to if (step_to and state == "step") else target)))
            if L.hh_fail() or (L.hh_landed() and cmd <= core.get("thr_idle") + 0.001 and plant.z < 0.02 and (t_land or auto_landed)): break
        plant.step(dt, cmd)
        zmax = max(zmax, plant.z); vzmax = max(vzmax, plant.vz)
        out["t"].append(plant.t); out["z"].append(plant.z); out["thr"].append(cmd); out["vz"].append(plant.vz); out["mode"].append(L.hh_mode())
        if plant.crashed: break
    return dict(crashed=plant.crashed, fail=bool(L.hh_fail()), zmax=zmax, vzmax=vzmax, hold_max=max(hold_err, default=9.0), hold_rms=(sum(e * e for e in hold_err) / len(hold_err)) ** 0.5 if hold_err else 9.0,
                touchdown=plant.max_impact, t_air=t_air, landed=bool(L.hh_landed()), auto_landed=auto_landed, base=L.hh_base(), trace=out)
