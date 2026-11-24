# tools_scripts

Helper scripts used while building and bench-testing the replica. They are standalone; none is imported by the PC package.
Ports are arguments (use `/dev/serial/by-id/...`). Run with `python3 <script> ...`.

| Folder | Script | What it does | Writes to hardware? |
|---|---|---|---|
| `fc_config/` | `fc_probe_msp_readonly.py <port>` | read-only MSP probe: version, board, RX config, sensors | no |
| | `fc_read_config_cli_readonly.py <port> <out.txt>` | whitelisted CLI reads (`diff all`, `get ...`), leaves with `exit` (no save) | no |
| | `fc_set_debug_mode.py` | sets the Betaflight debug_mode in RAM/CLI (used for the rangefinder test) | **yes** |
| | `fc_apply_cli_lines.py <port>` | applies an exact whitelist of CLI lines, then `save`; refuses if armed (used for telemetry sensors and aux mode 35) | **yes** |
| `sensor_mtf02p/` | `mtf02p_passthrough_read_raw.py`, `mtf02p_ruler_test.py`, `mtf02p_capture_frames_in_passthrough.py`, `mtf02p_record_csv_background.py`, `mtf02p_report_window_stats.py` | read the MTF-02P through Betaflight `serialpassthrough` (FC must be power-cycled afterwards) | passthrough only |
| `link_bench_tests/` | `link_test_joystick_vs_fc_channels.py` | Boxer USB joystick vs channels the FC receives (MSP read-only) | no |
| | `link_capture_joystick_vs_fc_channels_free.py`, `failsafe_capture*.py`, `link_monitor_failsafe_motors_readonly.py` | earlier Boxer-based link and failsafe captures | no (motors only if you arm) |
| | `bench_read_fc_channels.py <fc_port> <seconds>` | prints the channels the FC receives (run the sender in another terminal) | no |
| | `bench_read_arming_flags.py <fc_port> <adapter_port>` | reads Betaflight arming-disabled flags over time with the sender running | no |
| | `elrs_tx_params_list_or_set.py <adapter> [set "<name>" <index>]` | lists the ELRS TX Lua parameters over CRSF (packet rate, telemetry ratio, power, VTX); `set` writes one select parameter (e.g. `set "Telem Ratio" 8` = 1:2) | **yes if `set`** |
| | `bench_check_gamepad_to_fc_channels.py <js_dev> <fc_port> <adapter> [s]` | Boxer USB joystick -> PC sender -> RF -> FC: does every stick/switch reach the right FC channel? Disarmed (arm channel forced low) | no |
| | `bench_test_gamepad_arm_guards.py <js_dev> <fc_port> <adapter> [s]` | **arms the FC (props OFF!)** from the Boxer: tests the arm guards (switch ON at start, throttle up at arming, device unplug); motor-command guard 1300 | **yes: motors idle** |
| | `bench_measure_pc_to_fc_latency.py <fc_port> <adapter> [flips]` | PC to FC control latency (roll step read over USB MSP), disarmed | no |
| | `bench_test_sensor_watchdog_armed.py <fc_port> <adapter_port> [label]` | **arms and raises throttle to 7% (props OFF!)**, logs when the PC sensor watchdog trips; FC commanded motors ramp on a bench, so use sparingly | **yes: motors** |
| | `bench_test_arm_then_kill_sender_failsafe.py <fc_port> <adapter_port>` | **arms the FC (props OFF!)**, hard-kills the sender, logs the failsafe disarm | **yes: motors idle** |
| `tx_build/` | `tx_build_patch_flash_elrs.sh <work_dir> <port> [phrase]` | build ELRS 4.1.0, patch layout + phrase, flash the DIY TX (not run end to end as one script) | **yes: flashes the TX** |
| `docs_tools/` | `make_elrs_briefing_pdf.py <out.pdf>` builds the one-page ELRS/TX briefing PDF (needs reportlab) | no |
| `reverse_engineering/` | `esp32_image_to_elf.py`, `ghidra_scripts/*.java` | turn the Nimbus ESP32 firmware into ELF and decompile it in Ghidra headless | no |

Notes
- `bench_*` scripts use read-only MSP requests (101, 104, 105, 150) and run `python3 -m nimbus_pc send` from `software/pc`.
- Props off, drone secured, before anything marked "yes: motors idle".
- The SX1281 SPI probe and the pin-search probe are in `firmware/tx_dongle/bringup_checklists/sx1281_probe_src/` (corrected-pin probe only).

## Matek H743 quirk (found 2026-11-03)
Leaving the Betaflight CLI with `exit` makes this FC drop off USB and it only comes back after a full power cycle (USB + battery).
Ending a CLI session with `save` restarted it normally. So: read settings over MSP where possible, and avoid `fc_read_config_cli_readonly.py` / any `exit`
unless you can power-cycle the FC afterwards. `fc_set_vbat_scale.py` ends with `save`, not `exit`.
The Boxer shows up as "STM32 BOOTLOADER" (0483:df11) when it is left in bootloader mode; power-cycle the radio to get the joystick (1209:4f54) back.
| link_bench_tests/bench_tool_find_hover_throttle.py | find the real lift-off/hover throttle (PROPS ON, TETHERED; --sim for a dry run) |
| link_bench_tests/bench_diag_why_fc_not_arming.py | PROPS OFF: why the FC does not arm (blockers, mode, motors) in ANGLE and ACRO |
