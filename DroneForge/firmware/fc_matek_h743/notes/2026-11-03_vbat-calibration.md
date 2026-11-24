# Battery voltage calibration, Matek H743-Slim (2026-11-03)
Pack: Tattu R-Line v5.0 850 mAh 4S 150C (14.8 V nominal). Drone: 3-inch props, 160 mm tip to tip, 177 mm motor to motor, T-Motor F1404 KV2900 (motor_poles = 14).

| | vbat_scale | vbat_divider | vbat_multiplier | FC reads (16-bit MSP_BATTERY_STATE) | real (multimeter) |
|---|---|---|---|---|---|
| before | 110 | 10 | 1 | ~4.5 V (inferred; telemetry showed 44.7 V because of the unit issue below) | 16.06 V |
| try 1 (wrong formula guess) | 69 | 10 | 1 | 2.79 V, 0 cells | 16.06 V |
| try 2 (multiplier assumed to multiply) | 199 | 10 | 2 | 4.04 V | 16.06 V |
| **final** | **199** | **5** | **1** | **16.18 V, 4 cells (+0.7 %)** | 16.06 V |

- On this FC the reading is proportional to vbat_scale / (vbat_divider x vbat_multiplier). The FC only saw ~28 % of the pack voltage with the default 110/10/1 (the sense divider is not 11:1); gain needed ~39.7, hence 199 / 5.
- MSP_ANALOG's voltage byte (0.1 V) clips at 25.5 V and was misleading; use MSP_BATTERY_STATE (130, 16-bit, 0.01 V) for calibration.
- This FC sends the CRSF battery voltage in 0.01 V (1616 = 16.16 V), not the 0.1 V of the CRSF spec; `nimbus_pc.crsf.decode_battery` now accepts both.
- Check the calibration again at another charge level (pack near 15 V); if the error grows, the sense wiring is not a simple divider.
- Do not leave the Betaflight CLI with `exit` on this FC (needs a power cycle); end with `save`.

## Re-calibration after the reflash with the no-0xD4 build (2026-11-03, later)
Restored 199/5/1 read 19.26 V while the multimeter showed 15.77 V (ratio 0.819), so the old value no longer fit (reason not found; the same settings read 16.18 V vs 16.06 V earlier).
New: vbat_scale = 163 (divider 5, multiplier 1), saved. FC then read 15.77 V (median of 15 reads) = multimeter. `bf_restore_config.py` now restores 163.
Re-check against a multimeter whenever the FC is reflashed or the pack/wiring changes.

## After the prefix build flash (2026-11-03, evening)
With 163 the FC read 13.33 V while the multimeter showed 15.44 V (ratio 1.158) -> vbat_scale = 189 (divider 5, multiplier 1); FC then read 15.47 V.
Note: the same scale read differently after each full flash (199 gave +22 % after flash 2, 163 gave -14 % after flash 3). The cause is unknown, so ALWAYS recalibrate against a multimeter after any full flash (bf_set_vbat_scale.py: new = old x real / FC reading). `bf_restore_config.py` now restores 189 as a starting point only.
