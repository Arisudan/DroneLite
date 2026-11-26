v4.8.2 = v4.8.1 (EKF forward/right swap fixed) + the stiff height gains back. BUILT 2026-11-26 16:22:29, NOT FLASHED, NOT FLOWN.
The FC will report "built Nov 26 2026 16:22:29" after a flash. Flash only with the owner's go-ahead (FC on USB, battery off, disarmed):
   cd <DroneForge> && python3 tools_scripts/df.py flash firmware/fc_matek_h743/builds/2026-11-26_fc-hybrid-v4.8.2_BUILT_NOT-FLASHED/fc-hybrid-v4.8.2_matek-h743_betaflight-4.5.4.bin --debug-hold
Back to the flight-proven v4.8.1: the same command with builds/2026-11-26_fc-hybrid-v4.8.1_BUILT_NOT-FLASHED/*.bin.
CHANGE (v4.8.1_to_v4.8.2.patch, height_hold_core.c): height gains kp / kv / ki 0.9923 / 0.10 / 0.04 (v4.5, in R1 / v4.8 / v4.8.1)  ->  1.40 / 0.15 / 0.06 (v4.6 / v4.7.4R). Nothing else.
WHY: today's POS HOLD + VO flight on v4.8.1 held 55 s but the height bobbed +-0.2 m with a 10 s period (sd 0.096 m; 0.24-0.71 m), which triggered the range-lost auto-landing at 0.62-0.74 m.
 The bobbing is real vertical motion (height acceleration vs accelerometer-z corr +0.43) with the hold acting the right way (throttle lower when high, corr -0.42): an underdamped, too soft loop.
 Same-length windows of FC logs: v4.7.4R (stiff gains, 11-25 11:40:56) height sd 0.035 and 0.054 m (9-10 s airborne); R1 / v4.8.1 (v4.5 gains) sd 0.09-0.16 m (12-55 s).
NOT VERIFIED: the stiff-gain flights are short (9-10 s); this combination (stiff gains + the swap fix + v4.8's lift-off change) has never flown. Stiff gains may bring back the 11-25 take-off bobbing (0.30 -> 0.02 m)
 or more motor noise; judge by one short flight at 0.4-0.5 m. If it is worse, flash v4.8.1 back (the proven build).
