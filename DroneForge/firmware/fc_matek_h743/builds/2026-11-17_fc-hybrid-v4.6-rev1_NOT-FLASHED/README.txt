v4.6-rev1 (Fixed POS HOLD stability + responsive landing + hover anchor)
Built: 2026-11-17 17:55:29

Key Fixes and Enhancements in v4.6-rev1:
1. Fixed Position Hold (POS HOLD) Stability & Mode Switch Re-engagement:
   - Runaway Latch Reset: In xy_hold_core.c, xyReposition() now explicitly resets s->disabled = false, enabling clean re-engagement of POS HOLD when switching from ALT HOLD without being locked out.
   - Tuned Gains for 382 g Frame:
     * kp = 2.4 deg/m: smooth, progressive position centering without aggressive overshoot.
     * kv = 5.0 deg/(m/s): solid velocity damping to arrest drift immediately upon engagement.
     * ki = 0.15 deg/(m*s): gentle trim integration to counteract steady frame level bias without double-integral phase lag or oscillation.
     * max_deg = 4.0 deg: protective tilt ceiling.
     * runaway_pos = 2.5 m, runaway_speed = 1.0 m/s: expanded threshold preventing false trips during transition braking.
   - Verified Feedback Signs: Verified nominal signs (+1.0, +1.0) provide negative feedback across both pitch and roll axes. All 20 simulator unit tests pass (100%).

2. Responsive, Fast & Gentle Landing:
   - Instant reference snap: s->z_ref snaps immediately to current measured height if z_ref > z upon receiving Land command (zero lag/deadband).
   - Fast main descent: land_rate = 0.16 m/s from hover to cushion threshold.
   - Low cushion threshold: land_slow_below = 0.15 m at land_slow_rate = 0.07 m/s for soft touchdown.
   - Prompt motor shutdown: touch_s = 0.30 s, idle_ramp_per_s = 1.5/s.

3. Station-Keeping Hover Anchor:
   - Velocity-only damping during climb (hold = false) prevents ground turbulence from corrupting position reference.
   - Zero-offset handover: xyReposition() triggered upon transition to HH_MODE_HOLD anchors (0,0) target at the exact hover spot.

4. Floor Yaw Integrator Protection:
   - Capped yaw integrator (+-100) prevents lift-off motor torque skew.

Status: BUILT, NOT FLASHED.
Flash command:
python3 tools_scripts/df.py flash firmware/fc_matek_h743/builds/2026-11-17_fc-hybrid-v4.6-rev1_NOT-FLASHED/fc-hybrid-v4.6-rev1_matek-h743_betaflight-4.5.4.bin
