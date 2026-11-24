v4.7.2 (v4.7.1 + the VO position aid on RC channels)   2026-11-24
Base: v4.7.1 (built, not flashed) = v4.7 ST (FLASHED 2026-11-23, never flown) + the height-change feed-forward. Only flight/height_hold.c changed vs v4.7.1 (v4.7.1_to_v4.7.2.patch).
Status: BUILT 2026-11-24, NOT FLASHED, NOT FLOWN. Flash only with the owner's go-ahead (df.py flash <this .bin>).

Why: the MSP aid write (MSP2_DF_POS_AID) got through ELRS ~1 time in 10 (bench 2026-11-23: 10 replies to 100 writes at 10 Hz, gaps to 1.07 s); the FC uses an aid only while
it is < 0.3 s old, so the aid was almost always stale.
What: the FC also reads the aid from RC channels, which ride in EVERY ELRS packet:
   CH13 (AUX9) = forward, CH14 (AUX10) = right: 1500 us = 0, 1 us = 1 mm, valid 1100..1900 us (+-0.40 m); CH15 (AUX11) > 1900 us = "valid".
   Ignored without an RC signal and in failsafe (failsafe holds AUX channels at their last value). The MSP path still works (unchanged).
NEEDS the ELRS packet rate "100Hz Full" (16 channels at 10 bit). Measured 2026-11-24 on this link (ELRS 4.1.0, 250 Hz, switch mode Wide): CH6-12 arrive in ~15.9 us
steps and CH13-16 are NOT carried -> in Wide mode v4.7.2 simply never sees an aid (CH15 is not > 1900), it behaves like v4.7.1.
Same Wide-mode measurement: the target height on AUX5 arrives ~+22 mm too high (15.9 us steps = 4.9 cm of height): "100Hz Full" fixes that too (~3 mm).
GCS: core.set_pos_aid() -> CH13-15 while the aid is < 0.25 s old and armed (vision_bar, 10 Hz); MSP copy at 2 Hz for v4.7 / v4.7.1 FCs.
Sim: DF_FW=v472 (height law = libhh_v471.so; sim_radio reads CH13-15 like the firmware); DF_ELRS=wide (default, as the real link now) or full.
Build: as v4.7.1 (same toolchain, config, OPTIONS). Compiles without warnings in height_hold.c.
