# nimbus_pc — PC side for a NimbusOS-style ELRS dongle

Pure Python 3 (standard library only). Built from measurements of NimbusOS 2.9.0 `nimbus_core`
(see `../../../docs/specs/crsf-link_protocol-spec.md` for every byte layout and where it was read from).

## Run (from `~/Music/DroneForge/software/pc`)
```
python3 -m unittest nimbus_pc.tests.test_nimbus_pc nimbus_pc.tests.test_pc_complete nimbus_pc.tests.test_custom_frames nimbus_pc.tests.test_vtx   # 66 tests, no hardware
python3 -m nimbus_pc ports
python3 -m nimbus_pc send    --port /dev/ttyUSB0 --duration 10            # disarmed idle frames + pings + telemetry
python3 -m nimbus_pc send    --port /dev/ttyUSB0 --gamepad /dev/input/js1 # fly it from the Boxer (USB Joystick mode)
python3 -m nimbus_pc monitor --port /dev/ttyUSB0 --log telemetry.csv
python3 -m nimbus_pc bind    --port /dev/ttyUSB0 --phrase "my phrase"     # set bind UID + receiver bind command
python3 -m nimbus_pc msp     --port /dev/ttyUSB0 board-info               # MSP-over-CRSF request to the FC
python3 -m nimbus_pc vrx     --port /dev/ttyUSB0 connect --band F --channel 4   # Nimbus-specific, see below
python3 -m nimbus_pc vtx     --port /dev/ttyUSB0 get                      # read the drone's VTX config (MSP-over-CRSF)
python3 -m nimbus_pc vtx     --port /dev/ttyUSB0 set --band F --channel 4 --power 2   # power = VTX table level index, not mW
python3 -m nimbus_pc.emulator                                             # fake dongle on a pty, to try everything offline
```
`pip install -e .` also gives a `nimbus-pc` command. Arming needs BOTH `--arm --allow-arm` (demo modes), or the
gamepad's arm switch plus `--allow-arm`. Remove props first. Never point it at the FC's own USB port (that is MSP).
On a LilyGO T3-S3 build the serial port is a USB-UART adapter wired to GPIO43/44
(see `../../../archive/t3s3_draft/README_DRAFT.md`).

## What it does
- **RC link** (`sender.py`): raw 115200 8N1, DTR/RTS cleared, 26-byte frame at 100 Hz on absolute deadlines, AETR channels,
  172/992/1811 mapping with C `lroundf` rounding, stale-input failsafe (0.25 s), safe final frame on stop.
- **Device pings** every 250 ms to TX (0xEE), RX (0xEC) and FC (0xC8) until each answers with DEVICE_INFO.
- **Telemetry** (`crsf.py`): link statistics, battery, attitude, flight mode, vario, baro, device info, and Nimbus's custom
  range/optical-flow frame 0xD3 (distance mm, strength, precision, flow, quality, status), and the other custom frames
  identified so far: rangefinder 0xD0, raw IMU 0xD1, drone linked 0xE2, FC UID 0xE3, video-receiver channel RSSI 0xE1, magnetometer 0x12.
  Frames whose layout is not recovered (0xD2, 0xD4, 0xD5, 0xE4) are logged raw so nothing is lost.
- **Sensor watchdog** (`watchdog.py`, port of `OptrangeFailsafe::Update`): armed and throttle >= 5%, then 0.5 s of no sensor
  data or 0.5 s of a byte-identical payload forces throttle to 172 until disarm. Enabled with `--watch-optrange`.
- **Commands** (`commands.py`): receiver bind (0x10/0x01), set bind UID (0x80/0x42), video-receiver scan/connect
  (0x80/0x40, 0x80/0x41), MSP-over-CRSF requests (board info 0x04, VTX config 0x58) with chunked-response reassembly, and the VTX channel/power write
  (MSP_SET_VTX_CONFIG as two chunked 0x7C frames, validated like Nimbus).
- **Gamepad** (`gamepad.py`): Linux joystick to controls. Default map = Boxer in USB Joystick mode (measured). Nothing arms
  until every mapped axis has reported; the arm switch must be seen OFF first (found on the real Boxer: starting with it ON must not
  arm); arming is refused unless throttle is low when the switch goes up; device loss hands control to the stale-input failsafe.
- **Session** (`session.py`): one object tying link, sender, reader, commands, MSP requests, watchdog and CSV log together.
- **Emulator** (`emulator.py`): a fake dongle on a pseudo-terminal that answers pings, MSP requests, records commands and
  streams telemetry, so the whole stack is testable offline.

## Verified
66 tests pass: frame bytes identical to Nimbus's idle frame, CRC, parser resync, mapper, telemetry and 0xD3 decoders,
exact bind/command/MSP frames, MSP chunk reassembly (incl. wrong-sequence drop), gamepad mapping and arming guards, device
loss, 100.3 Hz rate, stale failsafe, pings answered then stopped, MSP board-info round trip, commands received by the
emulator, sensor watchdog at full-system level (silent sensor: throttle zeroed after about 0.5 s; frozen sensor: about 0.6 s;
healthy sensor: never), and the CLI itself.

## NOT verified
- **Never run against a real ELRS dongle** (none yet). The Boxer-as-gamepad path was tested on the real radio (2 runs, 3,400+
  samples): roll/pitch/yaw/throttle map end to end over the full CRSF range (every sample matched on the wire), arming passes with
  throttle low and is blocked with throttle up. Not yet exercised on the real radio: AUX2/AUX3 switches (never flipped; AUX3 read
  mid-scale), the arm-switch-ON-at-start case (covered by a unit test only), and clearing a block by cycling the switch.
- The emulator is not a model of real ELRS: it only implements what this code exchanges.
- Telemetry decoders other than 0xD3 follow public CRSF definitions, not captured real traffic.
- The 0x80 Nimbus commands (video receiver, set bind UID) are likely handled only by Nimbus's custom dongle firmware; stock
  ELRS may ignore them. The bind command 0x10/0x01 is an ELRS-standard command.
- `uid_from_phrase` (MD5 of `-DMY_BINDING_PHRASE="..."`) is from memory of ELRS, unverified.
- 0xD3 needs a drone that sends it: stock Betaflight does not (see `../../../firmware/fc_matek_h743/patches/2026-10-31_crsf-optrange-0xD3/`).
- Not implemented: Nimbus's translation of a video-receiver channel into the drone's own VTX table (per-board frequency tables exist
  in the binary), so `vtx set` writes the band/channel exactly as given. Not decoded: the layouts of custom frames 0xD2 (MightyCam),
  0xD4/0xD5 (motor PWM/eRPM, probable) and 0xE4 (no consumer found), unverified scalings for 0xD1 and the 0xE1 layout,
  and Nimbus's autonomy/estimator layers.
- Python timing may jitter more than Nimbus's C++ loop on a loaded machine.

## Health check: `doctor`
One command checks the whole chain, disarmed (it never raises the arm channel, and aborts if the FC reports armed):

    python3 -m nimbus_pc doctor --port <adapter on the dongle UART> [--fc-port <FC USB>] [--duration 25] [--json]

Checks: dongle answers CRSF ping; RC frame rate and jitter; CRC errors; radio link (time to connect, then LQ/SNR/RSSI, dropouts);
flight telemetry (battery, attitude, mode); range+flow frame 0xD3; which Nimbus-specific frames appear; MSP-over-CRSF board info.
With `--fc-port` (read-only MSP 101/105/150): FC disarmed, arming-disable flags, and that the PC's roll/pitch/yaw sweep reaches the FC
with throttle and arm channel low. Exit code 0 = no FAIL, 1 = at least one FAIL, 2 = could not run (port missing, FC armed).
The receiver can need roughly 5-25 s to connect after a TX restart, so the default run is 25 s.

## Live view: `live`
Real-time terminal view of everything that flows over the link, both directions, laid out like what NimbusOS's core publishes to its own UI
(its `TelemetryMessage` groups, `ControlsMessage` and `DeviceInfoMessage`, read from the app's message schema):

    python3 -m nimbus_pc live --port <adapter on the dongle UART> [--gamepad /dev/input/js1] [--demo idle|sweep] [--raw 8] [--listen]

Panels: telemetry in (16 Nimbus groups: value, age, rate; custom frames marked `*`; undecoded frames shown as raw hex), RF timing from the TX
(`0x3A` radio sync: packet interval), commands out (roll/pitch/yaw/throttle/arm, channel microseconds, stale-input failsafe, sensor watchdog),
device names (TX, RX, FC from pings), frame tables with counts, rates and ages in both directions, an event log (link up/lost, flight mode,
arm channel, failsafe, watchdog, gamepad lost) and an optional raw frame list. A red banner appears if the data stops or the sender dies.
It never arms unless you pass `--allow-arm`. Ctrl-C quits. `--no-clear` appends frames (for logs), `--log file.csv` also records decoded telemetry.

