v4.8.4 (FULL: kv + learning + range anchoring + feed-forward + step threshold) = v4.8.3 + the height-hold changes below. BUILT 2026-11-27 16:48:02, NOT FLASHED, NOT FLOWN. 808,837 bytes, 0 warnings in the changed files.
The FC will report "built Nov 27 2026 16:48:02" after a flash. Earlier v4.8.4 builds are kept: previous_kv0.32_only_built_2026-11-27_16-22-14/ (kv 0.32 only) and, inside it, previous_kv0.15_built_2026-11-27_15-16-04/ (the 11-27 flights).
Flash only with the owner's go-ahead (FC on USB, battery off, disarmed):
   cd <DroneForge> && python3 tools_scripts/df.py flash firmware/fc_matek_h743/builds/2026-11-27_fc-hybrid-v4.8.4_BUILT_NOT-FLASHED/fc-hybrid-v4.8.4_matek-h743_betaflight-4.5.4.bin --debug-hold
Back: the same command with previous_kv0.32_only_built_2026-11-27_16-22-14/fc-hybrid-v4.8.4_matek-h743_betaflight-4.5.4.bin, or builds/2026-11-26_fc-hybrid-v4.8.3_BUILT_NOT-FLASHED/*.bin.

GOAL: the drone holds the height the GCS sends (+-2 cm), and follows height changes from the GCS (0.5 -> 0.6 -> 0.45 m).
CHANGES (v4.8.3_to_v4.8.4.patch: height_hold.c, height_hold_core.c/.h)
 A. kv_z 0.15 -> 0.32 (kp 1.40, ki 0.10, i_max 0.35 as in the earlier v4.8.4). Measured plant gain 11.6 m/s2 per throttle unit vs ~27 assumed; damping was ~0.55 for every gain set tried.
 B. HOVER-BASE LEARNING (new, hhUpdate, param learn_tau = 8 s): in HOLD, while |error| < 6 cm, |vz| < 0.10 m/s, no target change running, no motor saturated and level: the integral moves into f_base (d = integ*dt/8; f_base += d/scale; integ -= d*...),
    f_base clamped to 0.40 .. thr_max-0.05, the output unchanged. Replaces the ~20 s integral wind-up after the hand-over (the hover needs ~+0.18 more than the lift-off throttle the base starts from). Designs: PX4 hover thrust estimator, ArduPilot MOT_HOVER_LEARN.
 C. RANGE ANCHORING (new, height_hold.c): b += ((range - ground_range) - z_hold - b) * dt/3 s, |b| <= 0.10 m, only in HOLD with a fresh range, level (< 8 deg) and above 0.15 m; the hold regulates z + b (a complementary filter: EKF for the fast part, the live range for the slow part).
    Removes the measured +3.7 cm EKF-above-range bias (v4.8.3 log: z - range = +0.037 m). The range reading on the floor is learned together with the ground offset at arming.
 D. az_ff_gain 0.0 -> 2.3 (= 27 / 11.6): acceleration feed-forward in height changes, scaled to the measured plant gain.
 E. hhSetTarget change threshold 0.02 m -> 0.005 m (the 10-bit height channel steps by 2 mm): GCS steps under 2 cm were ignored before.
 Debug (needs --debug-hold): debug[4] = the height the hold regulates (z + b) mm, debug[5] = vz mm/s, debug[6] = the range anchor b mm (was z_ref; the GCS target is in the hold CSV), debug[7] = integral x1000 (+10000 while a motor is saturated).
TESTS (tools_scripts/fc_hold_sim/test_v484.py + test_v483.py + test_v48.py, 9 pass): (1) with the base 0.12 too low the learning cuts the integral to < half and raises the base, height stays on 0.50 +-2 cm; (2) a step 0.50 -> 0.62 m lands within +-2 cm, overshoot < 4 cm (2 hover references);
 (3) a 1 cm step is accepted, 2 mm jitter ignored.
SIMULATION (weak plant 0.43x + slow thrust disturbance ~6 % + base 0.12 too low after the hand-over; 3 seeds x 2 hover refs): before (kv 0.15): sd 9.0 cm, post-take-off dip to 0.04 m, mean 0.478 m | kv 0.32 only: sd 4.2 cm, dip 0.17 m, mean 0.489 | this build: sd 4.2 cm, dip 0.19 m, mean 0.489.
 The simulator has no EKF/range bias, so C is NOT exercised there; the disturbance is a model (its real source is unknown).
NOT VERIFIED: nothing here has flown. C (range anchoring) and B on the real drone are the new risks: if the height hunts slowly (period 5-15 s), the anchor or learning fights the EKF: flash back. Expect sd 4-5 cm, mean on the target; +-2 cm is NOT guaranteed (the slow disturbance remains; the airframe imbalance, spread 0.21-0.25, is the main suspect).
PASS / FAIL for the first flight: >= 15 s after the hand-over the height within +-2 cm for most of the hold and sd < 0.03 m; minimum height in the first 12 s after take-off > 0.35 m; a step 0.50 -> 0.60 -> 0.45 m reaches +-2 cm within 4 s, overshoot < 3 cm.
