**STATUS: ROLLBACK - flew cleanly 2026-11-05; flash this to go back | DATE: 2026-11-05**

# Candidate FC build: FC-side height hold (2026-11-05) - BUILT, NOT FLASHED, NOT FLOWN
Same as 2026-11-04_kinematic-0xD4-on_HISTORY (0xD4 on, 00 C8 prefix) plus a height hold running on the FC at the range-sensor rate (50 Hz).
Files: height-hold_matek-h743_betaflight-4.5.4.hex/.bin/.elf, patches/2026-11-05_height-hold_source-changes-vs-kinematic-on.patch (all FC source edits), height_hold*.c/.h (the new module), sha256 of hex 085b087e021b2739..., bin 01050e0d003fcf29...
Build: PATH=<toolchain_x86>/gcc-arm-none-eabi-10.3-2021.10/bin; in bf_fixed/repo: make -j8 hex CONFIG=MATEKH743 CONFIG_DIR=/home/htic/droneforge/bf_fork_src/config_repo/config OPTIONS="USE_EKF USE_EKF_BARO USE_RANGEFINDER USE_RANGEFINDER_OPTFLOW_MTF USE_OPTICALFLOW NIMBUS_SEND_KINEMATIC"
(the Makefile only rebuilds the hex target when it does not exist: move the old hex away first).

## UPDATE 2026-11-05 (later): FC controls the height FROM LIFT-OFF (default), CH10 = hover reference
- The PC now requests the hold as soon as it has confirmed lift-off (sign-check height first, then the hover height), not only in HOVER. GCS checkbox "FC holds the height (from lift-off)" is ON by default (setting fc_hold, saved with "Save as default").
- CH10 (rcData[9]) carries the PC's hover-reference throttle (same scale as the throttle channel); the FC starts its throttle from it (valid 0.2..0.9, CH10 below 1100 us = not provided, then the throttle stick is used). Without this the FC started from the PC's climbing throttle and wobbled +-0.12 m in sim; with it <= 0.02 m on 6 seeds.
- A radio transmitter with CH10 at mid (1500 us) would give the FC a base of about 0.5 if it requested the hold: keep CH9/CH10 unassigned on the Radiomaster.
- NEW firmware build (this README's hex/bin are the NEW ones, sha256 of the bin starts 3a1653b8...): it must be flashed for CH10 to be used; the old build (4e7c6099...) ignores CH10 and still works with the PC (it then starts from the throttle stick).

## What it does
- Switch + target on ONE RC channel, CH9 (AUX5): < 1100 us off; 1100..1149 us hold at the current height; 1150..1950 us hold at (us-1150)*2.5/800 m (PC: nimbus_pc.mapper.height_to_aux5). Mode: `aux 5 40 4 1100 2100 0 0` (USER1 on AUX5 > 1100 us), in bf_restore_config.py and bf_apply.py.
- Control law = the PC hover controller's vertical law (kp_z 0.9923, kv_z 0.0538, ki_z 0.01, limit 0.078, vz_lim 0.4581, minimum-jerk height changes at 0.45*vz_lim, overshoot damper), on the FC EKF height and vertical speed. Throttle is in stick fractions (hover 0.70 means the same as in the PC). Bumpless: it starts from the throttle the PC is sending when it engages.
- Height = EKF height minus the sensor's ground offset (~0.07 m), learned while armed on the floor.
- It engages only when: USER1 requested, armed, no failsafe, EKF finite, range sample fresher than 0.25 s, 20+ ground samples, height > 0.15 m above ground and |vz| < 0.5 m/s. It LETS GO (and reports it) when: disarmed, failsafe, EKF not finite, range stale > 0.25 s, height > 2.8 m or below -0.2 m. The mixer then uses the PC's throttle again.
- Status to the PC in the CRSF flight-mode text: " HH" = holding, " HF" = it let go (e.g. "STAB HH").
- Takeoff, the sign check, the XY (position) hold and landing stay on the PC. The PC keeps computing its own throttle as the fallback.

## Tested
- C control law against the flight simulator through ctypes (nimbus_pc/tests/test_fc_hold.py): bumpless takeover, hold within 0.04 m, steps 0.5 -> 1 -> 1.5 -> 2 -> 2.5 -> 1 -> 0.5 m: overshoot <= 0.16 m, final error < 0.05 m, no crash. Compiles for the H743 with no warnings in the touched files.
## NOT tested on the drone (needs your hardware)
- USER1 mode activation through the `aux 5` line, the throttle override in the mixer, the ground-offset learning, the range-age check, the telemetry text.

## Flash and check (props OFF for the first checks)
1. DFU flash: dfu-util -S <FC dfu serial> -a 0 -s 0x08000000:mass-erase:force:leave -D height-hold_matek-h743_betaflight-4.5.4.bin
2. tools_scripts/fc_config/fc_restore_config_after_flash.py (adds serial 2 + aux 5), then re-check vbat_scale against a multimeter (bf_set_vbat_scale.py).
3. tools_scripts/link_bench_tests/bench_check_ekf_and_flow_data.py must still pass.
4. Hand-lift check, PROPS OFF (a script for it will be written when you are ready): arm, lift the drone by hand to about 0.5 m, set CH9 to "hold now" (1100..1149 us); the FC must report " HH" in the flight mode text and the motor outputs must rise when the drone is lowered and fall when it is raised.
5. First flight: tethered, props on, GCS box "FC holds the height" ticked. Watch the log column fc_hold (1 = FC holding, 2 = the FC let go) and the events.

## Safety notes
- CH9 is sent at its MINIMUM (hold off) by the PC. A radio transmitter whose AUX5 sits at mid (1500 us) WOULD request a hold at about 1.1 m once this firmware and the `aux 5` line are in the FC: keep that switch off or unassigned on the Radiomaster (the hold also needs: armed, airborne above 0.15 m, healthy range sample).
- If the FC lets go, the throttle goes back to what the PC sends. Landing is always done by the PC (the hold is released in LAND).

## UPDATE 2026-11-05 (evening): wireless diagnostics + GROUND INTEGRATOR HOLD
- Flight-mode text carries one diagnostic group per frame in rotation: motors+throttle, PID terms (P I D per axis), gyro rate + PID sum (decoded in nimbus_pc/crsf.py, logged in docs/flights/*.csv columns pidI_*, pidP_*, pidD_*, gyro_*, pidS_*).
- FINDING (ramp 15:47): gyro ~0, P/D tiny, but the I term went 17,-5,-9 -> -79,-195,-77 within 2 s at 80 % throttle on the floor (tilt error ~0.3 deg, boosted by anti-gravity while the throttle rises). It saturates a motor, the mixer cuts the common throttle, no lift-off.
- FIX (height_hold.c heightHoldOnGround + fc/core.c): while ARMED and within 0.05 m of the floor (ground offset learned while armed with the throttle stick below 1300 us) the PID integrator is held at zero (pidSetItermReset(true)). Above 0.05 m the stock behaviour applies.
- Also: the ground offset is now learned by throttle (stick low) and not by height (it used to need a ground reading below 0.12 m, which failed on a surface reading 0.17 m).
