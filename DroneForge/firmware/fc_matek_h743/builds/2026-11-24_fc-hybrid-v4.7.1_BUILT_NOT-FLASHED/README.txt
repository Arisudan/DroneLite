v4.7.1 (v4.7 ST + smoother, more exact in-flight height changes)   2026-11-24
Base: v4.7 ST (FLASHED 2026-11-23 14:44, never flown). Only flight/height_hold_core.c/.h changed (patch: v4.7-ST_to_v4.7.1.patch, 42 lines).
Status: BUILT 2026-11-24 11:02, NOT FLASHED, NOT FLOWN. Flash only with the owner's go-ahead:
   cd ~/Music/DroneForge && python3 tools_scripts/df.py flash firmware/fc_matek_h743/builds/2026-11-24_fc-hybrid-v4.7.1_BUILT_NOT-FLASHED/fc-hybrid-v4.7.1_matek-h743_betaflight-4.5.4.bin
Build: same as v4.7 ST: GNU Arm Embedded 10.3-2021.10, betaflight/config 4.5-config, MATEKH743,
   make -j8 hex CONFIG=MATEKH743 CONFIG_DIR=$HOME/droneforge/bf_fork_src/config_repo/config OPTIONS="USE_EKF USE_EKF_BARO USE_RANGEFINDER USE_RANGEFINDER_OPTFLOW_MTF USE_OPTICALFLOW NIMBUS_SEND_STATE"
   (rm -rf obj first); .bin = arm-none-eabi-objcopy -O binary of the elf. 96 bytes larger than v4.7 ST.

What changed (height hold, flight/height_hold_core.c):
 1. Acceleration feed-forward while a height change runs: the reference's acceleration (2nd derivative of the quintic) adds (f_base - idle) * a / 2g to the throttle
    (thrust ~ u^2). The drone no longer lags the planned climb/descent and then overshoots.
 2. ki_z 0.06 -> 0.15: the integral removes the remaining offset faster (take-off mean error 6.4 -> 2.3 mm in the bench).
 3. climb_peak_frac / descent_peak_frac 0.45 -> 0.35: a slightly gentler height change (a 0.1 m step takes ~1.5-2 s).
 GCS -> FC path unchanged: target height on AUX5 (0.002 m resolution); changes under 0.02 m are ignored by the FC as RC jitter (the GCS step is 0.05 m).

Bench (tools_scripts/fc_hold_sim, the firmware's own law on the plant fitted to the real drone; corrected floor averaging), targets >= 0.5 m,
steps 0.5 -> 0.6 -> 0.7 -> 0.55 -> 0.8 -> 0.5, 12 s each, 5 seeds:
                         v4.7 ST     v4.7.1
   step overshoot          42 mm      21 mm
   take-off overshoot      59 mm      45 mm
   mean error (last 4 s)   <= 7.8 mm  <= 8.0 mm (median 1.3 mm)
   instantaneous error     <= 15 mm   <= 16 mm  (range-sensor noise ~1 cm per sample: the instantaneous height cannot stay inside +-0.01 m)
 Robust over hover throttle 0.66/0.78, slower sensor/actuator/EKF, battery sag: step overshoot 19-37 mm (v4.7: 42-59 mm), no take-off worse.
 test_fc_hold.py on this source: 23 pass, 2 fail (both fail on v4.7 too: harshest ground-effect take-off surge 1.7002 m/s vs 1.7 limit (v4.7 1.714); an xy test).
 Below ~0.45 m both versions sit 6-10 cm low in the bench: the bench's range over-read model and the firmware's hhCorrectHeight table disagree there (sensor
 calibration question: measure range reading vs tape at 0.20-0.50 m on the real drone).
Simulator: gazebo/controllers/libhh_v471.so, select with DF_FW=v471 (default stays v47 = what is flashed).
