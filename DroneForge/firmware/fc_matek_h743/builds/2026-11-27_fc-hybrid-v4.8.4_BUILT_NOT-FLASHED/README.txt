v4.8.4 = v4.8.3 + a bigger and faster integral in the height hold (ki 0.06 -> 0.10, i_max 0.20 -> 0.35). BUILT 2026-11-27 15:16:04, NOT FLASHED, NOT FLOWN.
The FC will report "built Nov 27 2026 15:16:04" after a flash. Flash only with the owner's go-ahead (FC on USB, battery off, disarmed):
   cd <DroneForge> && python3 tools_scripts/df.py flash firmware/fc_matek_h743/builds/2026-11-27_fc-hybrid-v4.8.4_BUILT_NOT-FLASHED/fc-hybrid-v4.8.4_matek-h743_betaflight-4.5.4.bin --debug-hold
Back: the same command with builds/2026-11-26_fc-hybrid-v4.8.3_BUILT_NOT-FLASHED/*.bin (or v4.8.2 / v4.8.1).
EVIDENCE (v4.8.3 flight 2026-11-27, hold internals in debug[4..7], docs/fc_logs/20261127/LOG00001_053000.BFL):
 * the hold's own height z tracks the real range (corr +0.98, offset +0.037 m): the measurement is fine. Motor saturation 0.0 %: the v4.8.3 saturation rule did nothing (v4.8.2 theory rejected).
 * the hold takes over at 0.44 m (15 s), the throttle falls ~3 % short of what holds the drone, it SINKS to 0.03 m (18 s), the integral winds up slowly (0 -> 0.09 in 4 s, 0.16 by 23 s, 0.19 by 29 s = 95 % of the 0.20 limit),
   the drone climbs, overshoots, sinks again to 0.22 m (27 s) and only settles at 31 s (0.55-0.58 m, 5-8 cm above the 0.50 target, integral 0.17-0.19).
 * so the hover needs ~+0.18 more throttle than the base the hold starts from (the lift-off throttle in ground effect), the integral supplies it too slowly and is almost at its clamp.
CHANGE (height_hold_core.c, hhDefaults): ki_z 0.06 -> 0.10, i_max 0.20 -> 0.35. With the integral below 0.2 the steady state is unchanged; only the speed of the wind-up and the headroom change.
TESTS: the PC simulator (ideal ground effect) holds +-0.5 cm with and without this change (it does NOT reproduce the real sink, so it cannot rank fixes); test_v483.py passes.
NOT VERIFIED: it has not flown. A faster integral can overshoot more. If the first flight overshoots above 0.7 m or oscillates faster, flash v4.8.3 or v4.8.2 back.
 A larger lift_base_boost (starting the hold nearer the real hover throttle) was tried in the simulator and overshot to ~1 m, so it is NOT included.
