v4.7 ST (stick-through hold + VO position aid + closed-loop take-off climb)   2026-11-21
Base: v4.6-rev1 (FLASHED on the drone, not yet flown; the last FLOWN build is v4.6).
Status: BUILT 2026-11-23 11:45, FLASHED 2026-11-23 14:44 (df.py flash: dfu downloaded, config identical to the pre-flash backup, FC reports build Nov 23 2026 11:45:25). NOT FLOWN, bench checks (props off) not done yet.
   Compiler: GNU Arm Embedded 10.3-2021.10 (gcc 10.3.1, `make arm_sdk_install` in the repo); config: betaflight/config branch 4.5-config, MATEKH743.
   Build check: the SAME setup rebuilt v4.6-rev1 byte-identical to the flashed rev1 except the build date/time and the config-repo hash
   (19 bytes) -> the flashed options are known now: OPTIONS="USE_EKF USE_EKF_BARO USE_RANGEFINDER USE_RANGEFINDER_OPTFLOW_MTF USE_OPTICALFLOW NIMBUS_SEND_STATE" (no NIMBUS_SEND_KINEMATIC).
   v4.7 compiles with 0 compiler warnings; the .bin is 1000 bytes larger than rev1; xyUpdateST / xyBrakeThenHold / heightHoldSetPosAid are in the objects (LTO inlines them in the ELF).
The hold laws (xy_hold_core.c, height_hold_core.c) ARE compiled for the PC (libhh_v47.so), host-tested (21/21) and flown in the Gazebo simulation of the drone.

Files: src/ = the changed firmware files; v4.6-rev1_to_v4.7-ST.patch = the whole change; libhh_v47.so = the PC build the simulator uses (gazebo/controllers/libhh_v47.so).
Source of record: ~/droneforge/bf_fixed/repo (v4.6-rev1 snapshot: firmware/fc_matek_h743/source_snapshots/2026-11-21_v4.6-rev1_FLASHED).

CHANGES
1. Sideways hold (xy_hold_core.c)
   - Stick-through (ST): a roll/pitch stick beyond 5 % flies pure ANGLE mode, the hold adds nothing (v4.6: the hold's tilt was added and fought the stick).
     After the sticks centre: speed damping only until < 0.15 m/s for 0.2 s, then the hold re-anchors THERE (xyBrakeThenHold).
   - The hand-over to the hold and the return from ALT HOLD also brake first, then anchor (height_hold.c).
   - Gains kv 7.0 / kp 6.0 / ki 0.60 (v4.6-rev1: 5.0 / 2.4 / 0.15).
   - The level trim (integral) is KEPT by a re-anchor, and the take-off climb already learns it (climb_trim 3 deg/m).
   - Position aid: while fresh (< 0.3 s), the PC's visual-odometry displacement from the hold point replaces the flow-integrated displacement (no flow drift).
2. MSP2_DF_POS_AID 0x3F00 (msp.c, msp_protocol_v2_betaflight.h, height_hold.c): int16 forward mm, int16 right mm, u8 flags (bit0 valid).
   Sent by the GCS (VO button, nimbus_gcs/vision_bar.py) over MSP-over-CRSF while the drone hovers; older firmware ignores it.
3. Take-off (height_hold_core.c): takeoff_closed = 1: the height hold takes over as soon as lift-off is detected and the drone is above 0.15 m; the rest of the climb is the
   hold's smooth trajectory starting at the climb speed (quintic with start speed). v4.6 climbed open loop to 5 cm below the target and overshot
   (real flights 0.6 -> 0.78 m; simulation 0.80 -> 1.08 m, past the 1.0 m ceiling -> auto-land). takeoff_closed = 0 restores v4.6.
   Height hold gains are UNCHANGED (the flown v4.6 values).

EVIDENCE (simulation of THIS drone: 0.3825 kg, MTF-02P flow 25 Hz / 0.081 m/s noise / 50 ms, 1.0 deg level error, range 24 Hz; tools_scripts/fc_hold_sim/xy_bench.py and Gazebo house)
   Gazebo 0.80 m, 60 s:      v4.6-rev1   horizontal drift max 404 mm (end 387)   climb overshoot 187 mm (sim-only take-off tuning ON)
                             v4.7        150 / 108 mm (flow only)                 96 mm (firmware's own take-off values)
                             v4.7 + VO   56 / 22 mm                               108 mm
   Gazebo 0.60 m, 60 s, v4.7 + VO: horizontal max 47 mm, end 4 mm; height (rangefinder) after 20 s +6 mm +-3 mm; attitude <= 1.1 deg.
   Bench, 20 seeds: level-error transient 365 -> 68 mm; with VO aid worst 77 mm; fitted height plant: worst climb overshoot 203 -> 108 mm (median 18 -> 44 mm: WORSE median).

NOT VERIFIED / RISKS
   - Compiled cleanly, but the FC-side glue (height_hold.c sticks/aid, msp.c MSP2_DF_POS_AID) has only run in the PC simulation of the cores, not on the FC.
   - Closed-loop climb: the median overshoot on the fitted plant is higher than v4.6 (44 vs 18 mm); the hand-over happens near the floor (~0.15 m), where v4.4 once dropped
     (fixed in v4.5 by the base >= f_cur - i_max/2 rule, kept). First flights TETHERED, takeoff_closed can be set to 0.
   - The MSP2_DF_POS_AID path over the real ELRS link (MSP uplink rate) is untested; the FC falls back to flow when the aid stops.
   - Sign conventions of the aid (forward/right in the drone's axes) follow the core's own convention; check on the bench (VO on, move the drone by hand, read the debug).

BUILD (verified 2026-11-23):
   cd ~/droneforge/bf_fixed/repo && rm -rf obj      (always clean: stale objects survive a source change with older file dates)
   make -j8 hex CONFIG=MATEKH743 CONFIG_DIR=$HOME/droneforge/bf_fork_src/config_repo/config OPTIONS="USE_EKF USE_EKF_BARO USE_RANGEFINDER USE_RANGEFINDER_OPTFLOW_MTF USE_OPTICALFLOW NIMBUS_SEND_STATE"
   tools/gcc-arm-none-eabi-10.3-2021.10/bin/arm-none-eabi-objcopy -I ihex -O binary obj/betaflight_4.5.4_STM32H743_MATEKH743.hex <name>.bin
FLASH: only on the owner's word: python3 tools_scripts/df.py flash firmware/fc_matek_h743/builds/2026-11-23_fc-hybrid-v4.7-ST_FLASHED/fc-hybrid-v4.7-ST_matek-h743_betaflight-4.5.4.bin  (FC on USB, battery off). Rollback: the v4.6-rev1 .bin.
