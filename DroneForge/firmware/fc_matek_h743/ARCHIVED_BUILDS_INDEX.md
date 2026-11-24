# Archived firmware builds (moved here 2026-11-16 into archive_old_builds_2026-11-16.tar.gz, one folder per build, binaries + hex + README + checksums inside)
# Restore one build:  tar -xzf archive_old_builds_2026-11-16.tar.gz <folder name>

## 2026-10-09_cli-status-opticalflow-fix_HISTORY
**STATUS: HISTORY - build of 2026-10-09, flashed before 2026-11-03, superseded | DATE: 2026-10-09**

# FC firmware on the Matek H743-Slim (identified 2026-11-03)

## 2026-11-03_crsf-prefix-00C8_HISTORY
**STATUS: HISTORY - superseded, not to be flashed | DATE: 2026-11-03**

# Candidate FC build: [dest 0x00][origin 0xC8] prefix on custom frames, 0xD4 not scheduled (2026-11-03)

## 2026-11-03_no-0xD4-schedule_HISTORY
**STATUS: HISTORY - was flashed 2026-11-03, superseded | DATE: 2026-11-03**

# Candidate FC build: 0xD4 (kinematic state) no longer scheduled (2026-11-03) - FLASHED 2026-11-03 (mass-erase; config restored with tools_scripts/fc_config/fc_restore_config_after_flash.py)

## 2026-11-04_kinematic-0xD4-on_HISTORY
**STATUS: HISTORY - superseded, not to be flashed | DATE: 2026-11-04**

# Candidate FC build: 0xD4 (FC's own EKF state) ON, with the 00 C8 address prefix (2026-11-04)

## 2026-11-06_fc-takeoff-v1_UNSAFE-do-not-flash
**STATUS: UNSAFE - DO NOT FLASH (full-throttle burst; was flashed 11:35, replaced by v3) | DATE: 2026-11-06**

# 2026-11-06_fc-takeoff-v1_UNSAFE-do-not-flash (BUILT and FLASHED 2026-11-06, build "Nov 6 2026 11:35:34"; not yet flown)

## 2026-11-06_fc-takeoff-v2_UNSAFE-do-not-flash
**STATUS: UNSAFE - DO NOT FLASH (full-throttle burst at take-off) | DATE: 2026-11-06**

# 2026-11-06_fc-takeoff-v2_UNSAFE-do-not-flash (BUILT 2026-11-06, NOT FLASHED)

## 2026-11-06_fc-takeoff-v3_HISTORY
**STATUS: FLASHED (currently on the FC, build 2026-11-06 15:56:18) | DATE: 2026-11-06**

# 2026-11-06_fc-takeoff-v3_FLASHED (BUILT and FLASHED 2026-11-06 ~16:00; not yet flown)

## 2026-11-09_fc-takeoff-v4_NOT-FLASHED
**STATUS: NOT FLASHED (latest build) | DATE: 2026-11-09**

# 2026-11-09_fc-takeoff-v4_NOT-FLASHED (BUILT, NOT FLASHED, not flown)

## 2026-11-10_fc-takeoff-v5_FLASHED
**STATUS: FLASHED 2026-11-10 15:42 (build date read back from the FC) (latest build) | DATE: 2026-11-10**

# 2026-11-10_fc-takeoff-v5_FLASHED (BUILT, FLASHED 2026-11-10 15:42 (build date read back from the FC), not flown)

## 2026-11-13_fc-hybrid_FLASHED
**STATUS: FLASHED 2026-11-13 (build date read back from the FC: Nov 13 2026 14:24:16, config identical to the pre-flash backup), NOT FLOWN | DATE: 2026-11-13**

# fc-hybrid: FC-side height hold (take-off, hold, landing) made robust + hold status in the 0xD6 state frame

## 2026-11-13_fc-hybrid-v2_FLASHED
**STATUS: FLASHED 2026-11-13 (build date read back: Nov 13 2026 15:06:59; the flash tool's own read-back dump was taken too early and showed a false CONFIG DIFFERS, the live FC config read afterwards is identical to the pre-flash 

# fc-hybrid-v2: fixes the stall found in the first tethered run of fc-hybrid (11-13 15:00)

## 2026-11-13_fc-hybrid-v3_FLASHED
**STATUS: FLASHED 2026-11-13 (build date read back: Nov 13 2026 16:14:38, config identical to the pre-flash backup), NOT FLOWN | DATE: 2026-11-13**

# fc-hybrid-v3: the take-off CLIMBS to the target, then the hold takes over there (owner's required flow)

## 2026-11-13_fc-state-0xD6_FLASHED
**STATUS: FLASHED 2026-11-13 (build date read back from the FC: Nov 13 2026 11:15), NOT FLOWN | DATE: 2026-11-13**

# fc-state-0xD6: one compact telemetry frame instead of D3 + D4 + attitude

## 2026-11-14_fc-hybrid-v4.1_FLASHED
v4.1 = v4 with the floor zero-velocity update switched off (NIMBUS_FLOOR_ZUPT not defined).
Why: first real v4 runs 2026-11-14 12:48 and 12:52: fc_vz ran to 2 m/s right after arming; the FC take-off needs |vz|<0.15 so it never started (motors stayed at idle; FC logs 12 and 15). v3 recordings from 11-13 read 0.02 m/s at a
Cause is a hypothesis (the update pins the speed while the accel-bias state keeps adapting); not proven.

## 2026-11-14_fc-hybrid-v4.2-fastcreep_REGRESSED_tips-over
v4.2 = v4.1 + faster take-off creep: start at 0.85 of the hover reference (was 0.70), creep 0.015 per s (was 0.010).
Why: v4.1 flights 2026-11-14 took about 25 s of creeping to reach lift-off and were stopped by SAFE while still rising (FC LOG00003).
Host tests (15) pass. Not flown. NOT FLASHED.

## 2026-11-14_fc-hybrid-v4_FLASHED_no-takeoff
**STATUS: BUILT 2026-11-14, NOT FLASHED, NOT FLOWN | DATE: 2026-11-14**

# fc-hybrid-v4: sideways hold on the FC, floor zero-velocity update, level trim learned in flight, slimmer telemetry

## 2026-11-16_fc-hybrid-v4.2_NOT-FLASHED
v4.2 (flight modes) = v4.1 (original creep 0.70 / 0.010, ZUPT off) + mode channel.
AUX7 (rcData[10]): 1100..1399 us = ALT HOLD (sideways loop paused, height hold runs); anything else incl. minimum (988), the 1500 us idle of an unset channel and no signal = POS HOLD (same as v4.1).
Switching back to POS HOLD clears displacement and integral (new stay-here point), keeps the sign test result and runaway latch.

## 2026-11-16_fc-hybrid-v4.4_FLASHED
v4.4 = v4.3 (ALT/POS mode on AUX7, capped floor integrator +-100) + two changes from the 3 flights of 2026-11-16 15:02-15:07 (FC LOG1/2/4):

1. Overshoot damper after the hand-over (height_hold_core.c): the brake is now  base + integral - overshoot_cut  (was  base - overshoot_cut)  and the integral is KEPT (it was zeroed).

