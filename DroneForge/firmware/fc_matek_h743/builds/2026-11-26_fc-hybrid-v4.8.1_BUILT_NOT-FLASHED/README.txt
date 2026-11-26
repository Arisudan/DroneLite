v4.8.1 = v4.8 + ONE line: the EKF body velocity's forward and right are swapped back. BUILT 2026-11-26 15:27:42, NOT FLASHED, NOT FLOWN.
The FC will report "built Nov 26 2026 15:27:42" after a flash. Flash only with the owner's go-ahead (FC on USB, battery off, disarmed):
   cd <DroneForge> && python3 tools_scripts/df.py flash firmware/fc_matek_h743/builds/2026-11-26_fc-hybrid-v4.8.1_BUILT_NOT-FLASHED/fc-hybrid-v4.8.1_matek-h743_betaflight-4.5.4.bin --debug-hold
Back: the same command with builds/2026-11-26_fc-hybrid-v4.8_BUILT_NOT-FLASHED/*.bin (v4.8) or 2026-11-25_..v4.7.4R.. .

WHY: tools_scripts/ekf_bench_live.py --guided (2026-11-26 15:20, disarmed, by hand, two repeats of forward / back / right / back 0.5 m, docs/status_logs/hold_live/20261126_152049_hold.csv):
   moved FORWARD -> EKF right +0.28 / +0.47      moved BACK -> EKF right -0.35 / -0.67
   moved RIGHT   -> EKF forward +0.26 / +0.23     moved LEFT -> EKF forward -0.33 / -0.40
 = a pure swap (both signs positive): the "forward" the FC reports is the drone's right and the "right" is its forward.
 Consistent with everything seen: the sideways hold fought the wrong axis (runaway back-and-right while the EKF said forward-and-left), VO (right) disagreed with the EKF in the air.
CHANGE (v4.8_to_v4.8.1.patch, src/kinematic_estimator.c): kinematicEstimatorGetBodyVelocity: *forward = py; *right = px (was px / py). That getter feeds the sideways hold (height_hold.c),
 the 0xD6 state frame (crsf.c) and DF_HOLD, so all three are corrected together. Nothing else changed from v4.8 (17 v4.8 files copied unchanged into the build tree).
VERIFY BEFORE ANY FLIGHT (disarmed, 3 minutes): python3 tools_scripts/ekf_bench_live.py --guided. Expected after the flash: forward axis SIGN OK, right axis SIGN OK.
 If it says INVERTED on both, the swap is wrong: flash back to v4.8.
NOT VERIFIED: nothing has flown. The estimator is not in the PC simulator, so no simulation covers this. The earlier chair test (11-23) found the right axis correct; the sensor or its mapping has changed since or that test was wrong.
 The scale looked 0.5-1.0 of true in the bench run (hand moves are crude), not corrected here. v4.8's lift-off change (creep start 0.92) is still untested.
BUILD: ~/droneforge/bf_v473R/repo = R1 + the 17 files of v4.8 copied over + this line; make -j8 hex CONFIG=MATEKH743 ... OPTIONS="USE_EKF USE_EKF_BARO USE_RANGEFINDER USE_RANGEFINDER_OPTFLOW_MTF USE_OPTICALFLOW NIMBUS_SEND_STATE" (same as v4.8). 0 warnings in the changed files.
