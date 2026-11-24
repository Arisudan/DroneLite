v4.7.3R = v4.6-rev1 flight characteristics + the v4.7.2 VO position aid        2026-11-24
R = rev1 feel. Base: v4.7.2 (FLASHED on the drone 2026-11-24 12:46, FC reported build Nov 24 2026 11:43:54). Only 4 source files differ from v4.7.2 (v4.7.2_to_v4.7.3R.patch, 97 lines).
Status: BUILT 2026-11-24 17:04, NOT FLASHED, NOT FLOWN. The FC will report "built Nov 24 2026 17:04:04" after a flash. Flash only with the owner's go-ahead:
   cd ~/Music/DroneForge && python3 tools_scripts/df.py flash firmware/fc_matek_h743/builds/2026-11-24_fc-hybrid-v4.7.3R_BUILT_NOT-FLASHED/fc-hybrid-v4.7.3R_matek-h743_betaflight-4.5.4.bin
   (FC on USB, battery off, disarmed. Back to v4.7.2: the same command with builds/2026-11-24_fc-hybrid-v4.7.2_BUILT_NOT-FLASHED/*.bin. Back to rev1: builds/2026-11-17_fc-hybrid-v4.6-rev1_NOT-FLASHED/*.bin.)

WHY
Owner (2026-11-24): v4.7.2 flies much faster than v4.5 / v4.6-rev1 and the basics that worked are missing; v4.6-rev1 was the last build that felt right (FC logs 2026-11-18 show that build, stamp Nov 17 17:55:03, flying).
v4.7.3R puts every flight-feel value and behaviour back to v4.6-rev1 and keeps only what v4.7.2 added as an ability: the VO position aid.

REVERTED TO v4.6-rev1 (v4.7.2 -> v4.7.3R)
  sideways gains kv / kp / ki        7.0 / 6.0 / 0.60   ->  5.0 / 2.4 / 0.15
  stick-through (stick_db)           0.05 (on)          ->  2.0 (can never trigger: the stick is +-1.0)
  trim learned in the climb          climb_trim 3.0     ->  0   (and ki acts in the hold only, as rev1)
  xyReposition                       kept the integral  ->  clears it (rev1)
  hand-over / return from ALT HOLD   brake first        ->  reposition at once (rev1; xyBrakeThenHold is still compiled but no longer called)
  height ki_z                        0.15               ->  0.06
  climb / descent peak fraction      0.35               ->  0.45
  take-off                           takeoff_closed 1   ->  0 (open-loop creep, as rev1)
  acceleration feed-forward          on                 ->  off (new parameter az_ff_gain, appended at the END of hhParams_t, default 0; 1 = the v4.7.1 / v4.7.2 behaviour)
KEPT FROM v4.7.2
  MSP2_DF_POS_AID 0x3F00 (msp.c, msp_protocol_v2_betaflight.h); the RC-channel aid hhReadRcAid (CH13 forward, CH14 right, CH15 > 1900 us = valid, 1 us = 1 mm, +-0.40 m,
  ignored without signal / in failsafe; needs ELRS 100Hz Full + 16ch, set 2026-11-24); the aid replaces the flow-integrated displacement while < 0.3 s old (aid_max_m 3.0).
  With the aid ON the sideways command is rev1's kp 2.4 x the displacement (0.20 m of aid = 0.48 deg), i.e. a gentle correction, not v4.7's 6.0.
UNCHANGED since rev1 anyway: height kp 1.40 / kv 0.15, landing (0.16 m/s, slow below 0.15 m, touch 0.3 s), overshoot guards, runaway thresholds, take-off start 0.82 + creep 0.03 / 0.010 per s,
  tilt ceiling 4.0 deg, lift detection, ground-hold gate, estimator gates, 0xD6 state frame.

PROOF (host, the firmware's own cores compiled for the PC: libhh_v473R.so vs libhh_v46rev1.so; tools_scripts/fc_hold_sim/test_v473r_matches_rev1.py, 4 tests pass)
  * Every height / take-off / landing parameter (36) and every sideways parameter (15) equals rev1; the 2 new height switches are 0; stick_db 2.0, climb_trim 0.
  * 36 simulated flights (hover reference 0.66 / 0.72 / 0.78, 3 seeds, take-off+hold+land / step 0.6->0.8 / 0.8 m / cap-land): worst difference to rev1 = 1.8e-07 of full throttle and 7.7e-08 m in height,
    take-off time identical (float rounding of the same curve). For contrast v4.7.2 vs rev1 on the same flights: throttle up to 0.74, peak height up to 0.13 m, take-off 12.4 s earlier.
  * Sideways loop, 12 runs x 3000 steps with centred AND fully deflected random sticks, no aid, with hand-over repositions: max tilt difference 0.000000000 deg.
  * Aid: 0.20 m forward aid -> 0.48 deg = rev1 kp 2.4 x 0.20 m, as intended.
BUILD PROVENANCE: the source tree was reconstructed on this laptop (rev1 tree + the shipped v4.7.2 files) and first rebuilt as v4.7.2: same size (807,181 bytes), 15 bytes differ = the build date/time only.
  v4.7.3R .bin = 807,229 bytes (v4.7.2 + 48). Compiler GNU Arm Embedded 10.3-2021.10 (~/droneforge/toolchain_x86), MATEKH743, config 4.5-config. 0 warnings in the changed files.
  make -j8 hex CONFIG=MATEKH743 CONFIG_DIR=$HOME/droneforge/bf_fork_src/config_repo/config ARM_SDK_DIR=$HOME/droneforge/toolchain_x86/gcc-arm-none-eabi-10.3-2021.10
       OPTIONS="USE_EKF USE_EKF_BARO USE_RANGEFINDER USE_RANGEFINDER_OPTFLOW_MTF USE_OPTICALFLOW NIMBUS_SEND_STATE"   (rm -rf obj first; work copy ~/droneforge/bf_v473R/repo; ~/droneforge/bf_fixed/repo is untouched = rev1)
Files: src/ = every flight / msp source of this build (the rest of the tree = rev1); v4.6-rev1_to_v4.7.3R_flight.patch = the whole delta to rev1 (the aid); libhh_v473R.so = the PC build the simulator uses
  (gazebo/controllers/libhh_v473R.so, select DF_FW=v473r); source snapshot: source_snapshots/2026-11-24_v4.7.3R.

NOT VERIFIED
  - The FC-side glue (height_hold.c, msp.c) of this build has not run on the FC; only the pure-C cores were tested on the PC. Not run in Gazebo end to end (this laptop has no GPU).
  - The VO aid itself: the MSP path delivered ~1 write in 10 over ELRS; the RC-channel path was only checked at the FC (MSP_RC read-back), never flown. Keep VO off in flight until it is flown tethered.
  - "Feels like rev1" is proven for the control laws, not for the airframe: first flight TETHERED, FC take-off / hold stay opt-in.
