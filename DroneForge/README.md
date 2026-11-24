# DroneForge: Nimbus replica

Own build of the Nimbus system (NimbusOS 2.9.0): PC software -> DIY 2.4 GHz TX dongle -> ELRS receiver -> Betaflight FC.
Original analysis lives in `~/nimbus-elrs-analysis`. A backup of the layout before the 2026-11-02 reorganisation is `~/Music/DroneForge_before_reorg_2026-11-02.tar.gz`.

## Status (2026-11-02)
- Link works end to end: PC sender -> ESP32-C3 + E28 TX -> GEPRC 2.4 GHz RX -> Matek H743 (LQ 100%, telemetry back, arm and failsafe tested with props off).
- TX settings: ELRS Packet Rate 250 Hz, Telem Ratio 1:2 (needed for the 0.5 s sensor watchdog; reflashing resets it to Std).
- USB layout that is stable: Boxer on a laptop port, FC + CP2102 adapter on the hub, TX board on its own phone charger (see BENCH_TEST).
- Drone: 3 inch props, 160 mm tip to tip, 177 mm motor to motor, T-Motor F1404 KV2900, Tattu R-Line v5 850 mAh 4S (14.8 V). Battery voltage calibrated 2026-11-03 (vbat 199/5/1).
- Binding phrase: `<BINDING_PHRASE>` (UID `<BINDING_UID>`; kept out of the public repository). ELRS 4.1.0 on both TX and RX.
- Not done: real flight, a dead/unplugged-sensor test of the watchdog (no false trips with the steady 0xD3 stream; covering the sensor does not trip it, by design), VTX control, remaining custom frames (0xD2, 0xD4/D5, 0xE4). Details: `firmware/tx_dongle/bringup_checklists/2026-11-02_bench-test.md`.
- Do NOT accept the NimbusOS firmware-update prompt for the DIY TX: it targets Nimbus's ESP32-S3 hardware and would overwrite the working firmware. Close NimbusOS while flashing.

## Map
| Folder | What is in it |
|---|---|
| `tools_scripts/` | helper scripts for FC, sensor, link/bench tests, TX build and analysis (see its README.md) |
| `docs/` | `CRSF_LINK_SPEC.md` (exact baud, frames, channel map, failsafe), `MTF02P_NOTES.md`, `nimbus_architecture.png`, `REORG_LOG.md` (old -> new paths) |
| `software/pc/` | PC side (Python, 344 tests incl. GCS and hover sim): `nimbus_gcs/` = **PyQt6 ground control station** (`python3 -m nimbus_gcs`), and `nimbus_pc/` (live view, doctor health check, sender, telemetry, MSP/VTX commands, gamepad, watchdog, CLI, dongle emulator; see its README.md), `crsf_ref.py`, `install-udev-rule.sh`, `pyproject.toml` |
| `firmware/tx_dongle/builds/2026-11-02_elrs-4.1.0-tx-dongle_FLASHED/` | exact images flashed to your C3 board (bootloader 0x0, partitions 0x8000, boot_app0 0xe000, firmware 0x10000), README with sha256 |
| `firmware/tx_dongle/hardware_design/` | layouts (`*.hardware.json`), `PCB_REVIEW.md`, `PROTOTYPE_PLAN.md`, E28 manual facts, ELRS target entry |
| `firmware/tx_dongle/bringup_checklists/` | `BOARD_STATUS.md` (history and fixes), `BENCH_TEST_2026-11-02.md`, `sx1281_probe_src/`, blank factory flash backup |
| `firmware/fc_matek_h743/flashed_build/` | the FC firmware that is on the Matek (`fixed3_MATEKH743_4.5.4.bin/.hex/.elf`, matched by build time 2026-10-09 11:54:46), README with sha256; your build folder is `~/droneforge/firmware_out/` |
| `firmware/fc_matek_h743/config_backups/` | your H743 config before and after changes, failsafe test log |
| `firmware/fc_matek_h743/patches/2026-10-31_crsf-optrange-0xD3/` | Betaflight patch sketch for the range/flow frame (host-verified only) |
| `reference/nimbus_original/` | Nimbus's own files, read-only reference: `tx_esp32s3/`, `rx_drone/`, `betaflight_configs/`, `decompiled/`, `hardware/` (empty, for photos) |
| `archive/t3s3_draft/` | LilyGO T3-S3 plan (order cancelled), reference only |

## Key facts to remember
- DIY TX pin map (corrected): NRESET GPIO1, BUSY GPIO2, DIO1 GPIO3, SCK GPIO4, MISO GPIO5, MOSI GPIO6, NSS GPIO7, RXEN GPIO8, TXEN GPIO9, CRSF RX/TX GPIO20/21. The schematic's net labels sit ABOVE their wires; older notes in `design/` that list rst0/busy1/dio1 2/sck3/miso4/mosi5 are WRONG (off by one row).
- The TX only transmits while a PC sender feeds CRSF (115200) on GPIO20/21; its USB port carries power only.
- Ground control station window (PyQt6): `cd software/pc && python3 -m nimbus_gcs`.
  Look: "Mission control" (docs/gcs_screens/redesign_v5/concept2_mission-control, built screens in docs/gcs_screens/build_v5). Five workspaces: FLY, VIDEO, SETUP (sidebar: Sensors, Motor test, Calibration, Radio), ANALYZE, SETTINGS. ARM and TAKE OFF are press-and-hold; SAFE is one click or Esc; Ctrl+K opens the command palette. Greys plus one blue accent; amber = caution, red = SAFE and alarms only. Backend API: docs/specs/gcs-backend_api-reference.md.
- Hover-hold simulator and controller (simulation only, nothing flies): `cd software/pc && python3 -m nimbus_pc hover-sim --scenario all`; design, safety rules and unverified assumptions in `docs/specs/hover-hold_design-and-sim-results.md`.
- Live view of all telemetry and commands in the terminal: `cd software/pc && python3 -m nimbus_pc live --port <adapter> [--gamepad /dev/input/js1]`.
- Health check of everything (disarmed): `cd software/pc && python3 -m nimbus_pc doctor --port <adapter> --fc-port <FC USB>`.
- Quick link test (props off): `cd software/pc && python3 -m nimbus_pc send --port <adapter> --duration 20`. Arm test: add `--arm --arm-after 10 --allow-arm`.
- Run the tests: `cd software/pc && python3 -m unittest discover -s nimbus_pc/tests -t .`
