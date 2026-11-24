v4.6 (responsive landing + hover anchor, built 2026-11-17 16:44:57)

Key Updates in this Build:
1. Instant & Responsive Landing:
   - Immediate descent trigger: s->z_ref instantly snaps to current height if z_ref > z upon receiving Land command (zero lag/deadband).
   - land_rate: 0.16 m/s (prompt, active descent from 0.50 m).
   - land_slow_below: 0.15 m (only cushions when close to the floor, not at 0.30 m).
   - land_slow_rate: 0.07 m/s (gentle touchdown).
   - touch_s: 0.30 s, idle_ramp_per_s: 1.5/s.
   - Total landing duration reduced from ~6 s to ~2.3 s with soft touchdown.
2. Station-Keeping Hover Anchor:
   - Horizontal loop operates in velocity damping mode only during takeoff climb (hold = false), preventing floor turbulence from integrating phantom position error.
   - Handover anchor: The moment the FC enters HH_MODE_HOLD, xyReposition() is triggered to anchor (0,0) position setpoint at the exact hover spot.
   - Horizontal gains: kp=3.2, kv=5.0, ki=0.50, max_deg=4.5 deg.
3. Preserves Lift-off Yaw Stabilization:
   - Capped floor yaw integrator (+-100) eliminates M3 torque swing at lift-off (-0.9 to -2.0 deg shift in real flight).

Status: BUILT, NOT FLASHED.
