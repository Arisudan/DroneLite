v4.8.3 = v4.8.2 + motor-saturation awareness in the height hold + hold internals in the blackbox. BUILT 2026-11-26 16:48:15, NOT FLASHED, NOT FLOWN.
The FC will report "built Nov 26 2026 16:48:15" after a flash. Flash only with the owner's go-ahead (FC on USB, battery off, disarmed):
   cd <DroneForge> && python3 tools_scripts/df.py flash firmware/fc_matek_h743/builds/2026-11-26_fc-hybrid-v4.8.3_BUILT_NOT-FLASHED/fc-hybrid-v4.8.3_matek-h743_betaflight-4.5.4.bin --debug-hold
Back: the same command with builds/2026-11-26_fc-hybrid-v4.8.2_BUILT_NOT-FLASHED/*.bin (the build on the drone before this one).
THE PROBLEM: the height bobs +-0.2 m (7-11 s period, sd 0.09-0.10 m) with v4.8.1 (v4.5 gains) AND v4.8.2 (stiff gains): the gains are not the cause.
EVIDENCE for thrust saturation (not proof): (1) the same hold code in the PC simulator holds +-0.5 cm (so the law is fine on an ideal plant); (2) real LOG00002 (v4.8.2, 83 s hold): M2 averages 92 % of full scale and sits at >= 97 % for 11 % of the time,
 M3 only 69 % (spread 0.23, the drone is out of balance); in the 1 s windows with a saturated motor the height falls -3.5 cm/s, otherwise +1.0 cm/s; (3) the baro tracks the range (corr +0.81, slope 1.06), so it is not a baro problem.
 The hold law does not know a motor is saturated: it keeps raising the throttle (the integral winds up), then overshoots.
CHANGES (v4.8.2_to_v4.8.3.patch; height_hold.c, height_hold_core.c/.h):
 1. height_hold.c: every cycle it computes motor_sat = the highest motor output of the previous mixer cycle is >= 97 % of its range.
 2. hhUpdate: while motor_sat and the drone is BELOW the target (HOLD mode): the integral does not grow and the throttle is not raised above its previous value. Not saturated: identical to v4.8.2.
 3. DF_HOLD debug (needs debug_mode DF_HOLD, the --debug-hold flash): debug[4] = the hold's own height in mm (EKF, above the ground reference), debug[5] = its vertical speed mm/s, debug[6] = the reference it follows mm, debug[7] = the integral x1000 (+10000 while a motor is saturated).
 Compare debug[4] with surfaceRaw (the range) in the next log: if they differ by more than a few cm the hold regulates a wrong height (the cause of the 3-10 cm offsets from the commanded height).
TESTS: tools_scripts/fc_hold_sim/test_v483.py (2 tests: saturated => integral and throttle frozen; not saturated => as before) + test_v48.py pass.
NOT VERIFIED: it has not flown. This addresses one suspected cause. The root of the saturation is mechanical: M2 92 % vs M3 69 % (the centre of gravity sits toward M2: move the battery / camera toward M3 by a few mm) and a half-empty pack (the FC read 14.2-15.6 V).
 A balanced drone and a full pack are expected to help more than any firmware change.
