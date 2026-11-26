v4.7.4R1 = v4.7.4R with the v4.5 numbers in the gains. BUILT 2026-11-26 10:12:48, NOT FLASHED, NOT FLOWN.
The FC will report "built Nov 26 2026 10:12:48" after a flash. Flash only with the owner's go-ahead:
   cd ~/Music/DroneForge && python3 tools_scripts/df.py flash firmware/fc_matek_h743/builds/2026-11-26_fc-hybrid-v4.7.4R1_BUILT_NOT-FLASHED/fc-hybrid-v4.7.4R1_matek-h743_betaflight-4.5.4.bin [--debug-hold]
   (FC on USB, battery off, disarmed. Back: the same command with builds/2026-11-25_fc-hybrid-v4.7.4R_BUILT_NOT-FLASHED/*.bin, which is the build now on the drone.)

CHANGES (2 files, 22 patch lines: v4.7.4R_to_v4.7.4R1.patch)
  height_hold_core.c   height gains kp / kv / ki  1.40 / 0.15 / 0.06  ->  0.9923 / 0.10 / 0.04   (the v4.5 values; v4.6 had tightened them)
  xy_hold_core.c       sideways kp 2.4 -> 2.5, ki 0.15 -> 0.10 (kv stays 5.0, max tilt 4.0 deg: unchanged since v4.5)
UNCHANGED ON PURPOSE
  take-off creep (0.82 x hover ref, 0.03 / 0.010 per s), climb limit 0.30 m/s, vz_lim 0.458 m/s, climb peak 0.45, landing (0.16 / 0.07 m/s; the v4.5 landing values were not saved),
  DF_HOLD debug, VO aid, climb_trim 3 deg/m (the learned level trim).
  climb_trim is NOT a v4.5 value (v4.5 had no trim). Simulation (tools_scripts/fc_hold_sim/xy_bench.py, 12 seeds, 1 deg level error, flow only / with VO aid):
      rev1 / v4.5-style trim 0      drift median 371 mm / (no aid path in rev1);  v4.7.4R1 with trim 0: 372 mm / 428 mm
      v4.7.4R                       146 mm / 50 mm
      v4.7.4R1 (v4.5 gains + trim 3) 144 mm / 50 mm      -> the trim, not the gains, removes the sideways drift, so it is kept.
Height in simulation (take-off + 10 s hold at 0.50 m, three hover references): overshoot 3.3 / 2.8 / 4.5 cm (v4.7.4R 2.2 / 1.8 / 3.6), settled error 2.8 / 2.4 / 0.7 cm (v4.7.4R 1.4 / 0.9 / 0.3): softer response as expected, lift-off time identical.
PROOF: tools_scripts/fc_hold_sim/test_v474r1.py (3 tests) and test_v474r.py pass. The firmware itself is built with the same toolchain and options as v4.7.4R (0 warnings in the changed files; 807,605 bytes, v4.7.4R 807,581).
BUILD: rm -rf obj in ~/droneforge/bf_v473R/repo, then
   make -j8 hex CONFIG=MATEKH743 CONFIG_DIR=$HOME/droneforge/bf_fork_src/config_repo/config ARM_SDK_DIR=$HOME/droneforge/toolchain_x86/gcc-arm-none-eabi-10.3-2021.10 OPTIONS="USE_EKF USE_EKF_BARO USE_RANGEFINDER USE_RANGEFINDER_OPTFLOW_MTF USE_OPTICALFLOW NIMBUS_SEND_STATE"
Simulator: gazebo/controllers/libhh_v474R1.so, selector DF_FW=v474r1.
NOT VERIFIED: nothing here has flown. Whether the softer height gains cure the take-off bobbing is a hypothesis; the pack voltage (13.9 V) may also play a part. The v4.5 xy values are from the nearest saved source (2026-11-14 snapshot); v4.5's own source was not kept.
