v4.7.4R = v4.7.3R (v4.6-rev1 feel) + level-trim learning in the climb (POS HOLD) + the DF_HOLD debug log        2026-11-25
Base: v4.7.3R (FLASHED 2026-11-24 17:07, flown 2026-11-24 / 25 on the RTX laptop). Only 4 source files differ (v4.7.3R_to_v4.7.4R.patch, 87 lines).
Status: BUILT 2026-11-25, NOT FLASHED, NOT FLOWN. The FC will report "built Nov 25 2026 11:40:56" after a flash. Flash only with the owner's go-ahead:
   cd ~/Music/DroneForge && python3 tools_scripts/df.py flash firmware/fc_matek_h743/builds/2026-11-25_fc-hybrid-v4.7.4R_BUILT_NOT-FLASHED/fc-hybrid-v4.7.4R_matek-h743_betaflight-4.5.4.bin
   (FC on USB, battery off, disarmed. Back: the same command with builds/2026-11-24_fc-hybrid-v4.7.3R_BUILT_NOT-FLASHED/*.bin.)
   Optional, only when you want the new log fields:  add --debug-hold to the flash command, or later  python3 tools_scripts/df.py debughold on   (sets ONLY debug_mode = DF_HOLD; "off" = NONE).

WHY
FC logs of 11-25 (v4.7.3R): ALT HOLD is smooth, POS HOLD wanders. v4.7.3R's sideways hold is exactly rev1's, which never learned the drone's level error: the simulator bench shows 0.37 m of
wander in 60 s with the flow only, and the FC logs carry no EKF / flow / aid data, so POS HOLD could not be judged from them at all.

1. LEVEL-TRIM LEARNING (xy_hold_core.c)
   climb_trim 0 -> 3 deg/m: during the take-off climb the integral learns the level trim from the drift speed; the integral now acts in the climb (ki = p->ki, was: only in the hold) and is KEPT by
   xyReposition (hand-over and return from ALT HOLD; xyStart clears it for a new flight). Everything else is unchanged: sideways kv 5.0 / kp 2.4 / ki 0.15, tilt ceiling 4.0 deg, runaway guard 1.0 m/s / 2.5 m,
   stick-through OFF (stick_db 2.0), no brake-then-hold, height hold / take-off / landing = rev1, the VO aid (MSP 0x3F00 + RC CH13-15) kept.
   Bench (tools_scripts/fc_hold_sim/xy_bench.py, 20 seeded 60 s hovers with this drone's flow noise and 1 deg level error), maximum drift from the hold point, median (worst):
                         v4.7.3R / rev1       v4.7.4R
      flow only           371 mm (430)        174 mm (372)
      with the VO aid     434 mm (458)         51 mm (60)
   The stiff v4.7 gains (7 / 6 / 0.6) add nothing once the trim is learned (178 mm), so they stay out: ALT HOLD / lift-off / landing feel does not change.
2. DEBUG_MODE DF_HOLD (build/debug.h, debug.c, height_hold.c): a new debug mode that logs, once per hold update (~50 Hz), in the blackbox debug[0..3]:
      debug[0] EKF body velocity forward mm/s    debug[1] right mm/s
      debug[2] VO aid forward mm                 debug[3] aid right mm      (-32768 = no fresh aid: not sent, older than 0.3 s, ALT HOLD, or not in the hold)
   Turned on by `df.py debughold on` (or `flash ... --debug-hold`): debug_mode stays NONE by default (the flash restore script sets NONE on purpose). The GCS reads it:
   `df.py logs` / `df.py summary` print "Sideways hold (DF_HOLD, N s airborne): EKF speed rms ..., VO aid present in X % of the airborne time, longest gap without it ..." (nimbus_gcs/bfl.hold_report).
   Cost (an ESTIMATE, not measured): about +16 KB/s of log (4 slowly changing values at the 4 kHz log rate); the SD card had 5.9 MB free after `df.py logs --free`.

PROOF (host: the firmware's own cores compiled for the PC; tools_scripts/fc_hold_sim/test_v474r.py, 5 tests pass)
  * 18 simulated flights (hover reference 0.66 / 0.72 / 0.78, take-off+hold+land / step / cap-land): v4.7.4R vs rev1 worst difference 1.8e-07 of full throttle, take-off time identical: height is untouched.
  * Every rev1 sideways parameter equals rev1; stick_db 2.0, climb_trim 3.0.
  * The learned trim survives the hand-over reposition and is cleared by a new flight; deflected sticks change nothing; 0.20 m of aid -> 0.48 deg (kp 2.4 x 0.20 m).
BUILD: reconstructed tree (v4.7.3R) + 4 edited files; GNU Arm Embedded 10.3-2021.10 (~/droneforge/toolchain_x86), MATEKH743, config 4.5-config; 807,581 bytes (v4.7.3R 807,229; +352), 0 warnings in the changed files.
  make -j8 hex CONFIG=MATEKH743 CONFIG_DIR=$HOME/droneforge/bf_fork_src/config_repo/config ARM_SDK_DIR=$HOME/droneforge/toolchain_x86/gcc-arm-none-eabi-10.3-2021.10
       OPTIONS="USE_EKF USE_EKF_BARO USE_RANGEFINDER USE_RANGEFINDER_OPTFLOW_MTF USE_OPTICALFLOW NIMBUS_SEND_STATE"   (rm -rf obj first; work copy ~/droneforge/bf_v473R/repo; ~/droneforge/bf_fixed/repo = rev1)
Files: src/ = every flight / msp / debug source of this build; libhh_v474R.so = the PC build the simulator uses (gazebo/controllers/libhh_v474R.so, DF_FW=v474r).

NOT VERIFIED
  - The FC-side glue (height_hold.c DEBUG_SET lines, xy trim) has not run on the FC; only the pure-C cores were tested on the PC. Not run in Gazebo (this laptop has no GPU).
  - The bench numbers are a simulation of this drone (flow noise 0.081 m/s, 1.0 deg level error): the real gain may differ. First flight TETHERED, FC take-off / hold stay opt-in.
  - The trim acts in the take-off climb now (kv-damped as in rev1 plus the learned trim, max 4 deg): never flown. v4.7.2 had a stronger version of this in the climb and its flights were smooth, but that is not proof.
  - Whether DF_HOLD log writes disturb anything: the debug[] writes are four int16 stores per hold update; no other stock debug mode is touched.
