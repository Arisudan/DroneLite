v4.8.4 (REBUILT with kv 0.32) = v4.8.3 + height-hold ki 0.10, i_max 0.35, kv 0.32. BUILT 2026-11-27 16:22:14, NOT FLASHED, NOT FLOWN.
The FC will report "built Nov 27 2026 16:22:14" after a flash (the earlier v4.8.4 with kv 0.15, stamp 15:16:04, is kept in previous_kv0.15_built_2026-11-27_15-16-04/ and is what the 3 flights of 2026-11-27 ran).
Flash only with the owner's go-ahead (FC on USB, battery off, disarmed):
   cd <DroneForge> && python3 tools_scripts/df.py flash firmware/fc_matek_h743/builds/2026-11-27_fc-hybrid-v4.8.4_BUILT_NOT-FLASHED/fc-hybrid-v4.8.4_matek-h743_betaflight-4.5.4.bin --debug-hold
Back: the same command with previous_kv0.15_built_2026-11-27_15-16-04/fc-hybrid-v4.8.4_matek-h743_betaflight-4.5.4.bin (kv 0.15) or builds/2026-11-26_fc-hybrid-v4.8.3_BUILT_NOT-FLASHED/*.bin.
CAUSE (measured): the drone accelerates by only 11.6 m/s2 per unit of throttle (cross-spectrum of vertical acceleration vs mean motor output, flight 2026-11-27 LOG00001_053000_2, coherence 0.84, 0.08-0.5 Hz);
 the hold was tuned for ~27 (and the simulator has ~27-29). Damping of the cascade is zeta = 0.5*sqrt(G*kv/kp): 0.54 for v4.8.1 gains (kp 0.99, kv 0.10) and 0.56 for v4.8.2-4.8.4 gains (kp 1.40, kv 0.15):
 raising kp and kv together never changed it, so every gain set bobbed alike (height sd 0.08-0.10 m). Ruled out with data: motor saturation (0.0 %), baro (corr +0.81 with range), battery-voltage jitter (0.8-1.5 %, no correlation),
 measurement (hold z follows the range, corr +0.98), vz lag (<= 0.2 s, gain 0.74-0.88), integral limit (peak 0.10-0.14 of 0.35).
CHANGE (height_hold_core.c, hhDefaults): kv_z 0.15 -> 0.32 (kp 1.40 and ki 0.10 unchanged). Expected zeta about 0.86 at G = 13.
SIMULATION (tools_scripts/fc_hold_sim, libhh_v483.so; plant weakened to 0.43x with a slow random thrust disturbance of ~6 % throttle, which reproduces the real sd: 9.3 cm with kv 0.15, 11.6 cm with the v4.8.1 gains):
   kv 0.15 -> sd 9.3 cm | kv 0.26 -> 4.9 | kv 0.32 -> 4.2 | kv 0.40 -> 3.7 | kv 0.50 -> 3.5.   If the real plant gain is higher (0.8x design) a stiffer kv gets worse: 6.8 (0.32), 8.2 (0.40), 9.2 (0.50). 0.32 is the compromise.
   On the undisturbed design plant (1.0x) kv 0.32 is too stiff (sd 0.5 -> 2.7 cm, 7x throttle activity): not a concern if the measured gain is right.
NOT VERIFIED: it has not flown. The simulator's disturbance is a model, not a measurement: the source of the ~6 % slow thrust disturbance on the real drone is unknown. Expect about half the bobbing (sd 0.04-0.05 m), not +-2 cm.
 If the first flight rings faster (period 2-3 s, audible throttle chatter) the real gain is higher than estimated: flash kv 0.15 back. The hold internals (debug[4..7]) stay in the log for the next analysis.
