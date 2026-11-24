# Betaflight Configurator for the DroneForge custom firmware: contract (2026-11-11)

Local browser app. `run.sh` starts the Python server (127.0.0.1:8765) and opens the UI in Chromium/Firefox. Target: Betaflight 4.5.4 custom build, MSP API 1.46, Matek H743, USB serial `/dev/serial/by-id/*Betaflight*`. UI = the look and layout of the official Betaflight Configurator (dark top bar with logo, port + baud + orange CONNECT, left tab column with icons, status bar with packet error / I2C error / cycle time / CPU load, yellow/orange accent, "Save and Reboot" button bottom right, Open Sans-like font; no copyrighted assets copied, recreate icons in SVG).

## Why it exists (owner's problems)
1. The official Configurator will not let the owner change values on this custom firmware and its Save rewrites the serial ports (UART3 `serial 2 262144` for the MTF-02P range/flow sensor gets reset).
2. So: **every write is diff-only** (only values the owner changed), **everything is a CLI setting** (`get`/`set`/`aux`/`serial`/`feature`... so custom settings just work), and **reboot is one click**.

## Safety rules (hard, tested)
- Writes refused while the FC reports ARMED (MSP_STATUS flags) and while the FC USB is not the active link.
- Before the first write of a session: auto backup of `diff all` to `betaflight_configurator/backups/<UTC>_before-change.txt`.
- Never send MSP commands outside this allow-list: reads (API_VERSION 1, FC_VARIANT 2, FC_VERSION 3, BOARD_INFO 4, BUILD_INFO 5, STATUS 101, STATUS_EX 150, RAW_IMU 102, MOTOR 104, RC 105, ATTITUDE 108, ANALOG 110, UID 160, SENSOR_STATUS 151), MSP_ACC_CALIBRATION 205, MSP_SET_REBOOT 68, MSP_EEPROM_WRITE 250. NEVER: SET_MOTOR 214, SET_CF_SERIAL_CONFIG 55, SET_RAW_RC 200, RESET_CONF 208, any arm/RC injection.
- `serial` lines are shown read-only unless the owner clicks "unlock serial editing"; a permanent note says `serial 2 262144` is the MTF-02P sensor port.
- Motor test is NOT implemented (read-only motor outputs only). Props stay on in this project.
- CLI facts: entering CLI (`#`) is fine; leaving CLI with `exit` or `save` REBOOTS the FC. So the app has two modes: LIVE (MSP: Setup, Receiver, Sensors, Motors outputs) and SETTINGS (CLI: all editing). Switching LIVE to SETTINGS enters CLI; leaving SETTINGS = Reboot (button) then the app reconnects to LIVE. Show the mode in the status bar.

## WebSocket JSON protocol at ws://127.0.0.1:8765/ws
Client->server `{"id":n,"method":"...","params":{}}`; server->client `{"id":n,"result":...}` or `{"id":n,"error":"text"}`; server events `{"event":"...","data":...}`.
- `ports.list` -> `[{path,description,kind:"betaflight"|"radio"|"other"}]` (kind betaflight when by-id name contains Betaflight/STM; never auto-connect to the CP2102 radio).
- `fc.connect {port}` / `fc.disconnect` -> `info` below. `fc.info` -> `{api:"1.46",variant:"BTFL",version:"4.5.4",board:"MATEKH743",build:"Nov 10 2026 15:42:40",uid,armed:bool,mode:"live"|"settings",cycle_us,cpu_load,i2c_errors,packet_errors}`.
- Event `status` (2 Hz): the same fields plus `connected`. Event `live` (10 Hz, LIVE mode only): `{attitude:{roll,pitch,yaw},rc:[...],motors:[...],gyro:[x,y,z],acc:[x,y,z],analog:{vbat,amps,rssi},range_cm}`.
- `mode.settings` (enters CLI, loads and caches the dump) / `mode.live` (= reboot, reconnect) / `fc.reboot {target:"normal"|"bootloader"}` (MSP_SET_REBOOT; bootloader = DFU, label it clearly) / `fc.calibrate_acc` (MSP 205, disarmed only, tell the owner the drone must be level and still).
- `settings.dump` -> `{version_line, settings:[{name,value,group}], lines:{serial:[...],aux:[...],feature:[...],resource:[...],other:[...]}, profile, rateprofile}` (from `dump all`; `group` from the name prefix/known tables).
- `settings.meta {name}` -> `{min,max,default,table:[...]|null,type}` (from `get name`, cached).
- `settings.apply {changes:{name:value}, lines:["aux 0 0 1 900 1300 0", ...]}` -> `{applied:[{name,old,new,verified}],failed:[{name,error}],backup:path}`; diff-only (skips equal values), re-reads each changed setting to verify, does not save.
- `settings.save` -> sends `save` (FC reboots), reconnects LIVE, returns `{saved:true,info}`.
- `settings.diff` -> `{text}` (`diff all`) for the "view changes" panel; `settings.export`/`settings.import {text}` (import = stage `set`/other lines through apply, never auto-save).
- `cli.open`, `cli.send {line}` -> event `cli {text}`; `cli.close` (reboots; warn in UI text, no modal).
- Errors are plain strings the UI shows in a red banner; no modal dialogs, no yes/no popups (owner's rule).

## Layout
`server/` Python (msp.py, link.py, cli.py, safety.py, app.py ASGI via uvicorn, emulator.py = fake Betaflight FC over a pty for tests, __main__.py), `web/` (index.html, css/, js/, icons/), `tests/`, `backups/`, `run.sh`, `README.md`.
Tab list in the UI (in this order): Setup, Ports, Configuration, Power & Battery, Failsafe, PID Tuning, Receiver, Modes, Motors, Sensors, OSD, Blackbox, CLI. Each settings tab is a form bound to CLI setting names with ranges from `settings.meta`; Ports shows `serial` lines (read-only unless unlocked); Modes edits `aux` lines; Receiver/Sensors/Motors/Setup use `live` events.

## Backend notes (added by the backend agent, 2026-11-11)
- Added method `serial.unlock {on:bool}` -> `{serial_unlocked}`; resets to locked after save/reboot/connect. `settings.apply` puts locked `serial` lines in `failed`.
- `settings.apply` result also has `skipped:[...]`; `lines` accepts only aux/serial/feature/beeper/map/rxrange/adjrange/vtx/vtxtable/led/color. Line results use the line text as `name`.
- `mode.settings` returns the same object as `settings.dump` and refuses when armed. `cli.open` -> `{mode:"settings"}`. `cli.send` returns `{text}` and emits event `cli`; `save`/`exit` typed there reboot like the buttons.
- Extra event `error` (string) when the FC link is lost. `status` also has `port`. `live.range_cm` is always null (rangefinder MSP is not allow-listed).
- `fc.reboot {target:"bootloader"}` sends MSP_SET_REBOOT type 1 and leaves the app disconnected.
- Demo only: `GET /demo/armed/1|0` toggles the emulator ARMED flag.
