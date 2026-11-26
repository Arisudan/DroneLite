# DroneForge — START HERE

Read this first, whether you are a person or an AI model taking over the work. It explains what the project is, how the parts fit together, where every file lives, how to run and test it, what the rules are, and what is **proven versus still unproven**. Facts are dated; when this file and the code disagree, the code wins, then tell the owner.

State described: **2026-11-26**. Detailed running history: `HANDOFF.md` (sections 5a-5x) and `docs/` (local only, not on GitHub).

---

## 1. What this is

DroneForge is the owner's own replica of the "Nimbus" camera-based indoor drone system, adapted to one real drone. **Goal: autonomous indoor roaming using only the camera** (plus a range/flow sensor), flown through a ground-control station (GCS) on a laptop. It is verified in the Gazebo simulator first, then on the real drone.

**The drone (real):** 3-inch quad, Matek H743 flight controller running Betaflight 4.5.4 plus our own firmware additions, MTF-02P range+optical-flow sensor on UART3, DJI O4 Pro camera (video goes by its own path, not the radio), ExpressLRS 2.4 GHz radio link, 4S LiPo.

**The ground side:** a PyQt6 GCS (`software/pc/nimbus_gcs`) on a laptop. A DIY ESP32-C3 + E28 ELRS transmitter dongle (a CP2102 USB-serial adapter) sends 16 RC channels to the drone at 100 Hz and returns telemetry. Two laptops are used: a CPU-only one and an RTX 5090 one (GPU models); they sync through GitHub (`https://github.com/Arisudan/DroneLite`, code only, see section 12).

## 2. Rules (the owner's standing instructions — follow them)

1. **Safety first.** Warn on the FIRST line before anything that arms the drone ("THIS ARMS THE DRONE: the motors will spin..."). Never arm, flash firmware, change FC settings, enter the FC CLI, or delete data without the owner's explicit go-ahead. Propellers stay on (the owner flies with props on); the owner is always the one who decides to fly.
2. **Metres only** for every distance (never cm or mm in anything user-visible). Speed in m/s.
3. **Battery is a warning only.** It never blocks arming and never forces a landing.
4. **GCS UI rules:** no pop-up dialogs, no checkbox widgets, no "Advanced" toggles, no yes/no questions. Greys plus one soft accent colour; amber only for caution; red only for SAFE and real alarms. State must be readable without colour (words, shapes).
5. **Honesty.** Say what is verified and what is not. Do not claim a flight result you have not seen in a log.
6. **No secrets in Git.** The ELRS binding phrase and receiver UID never go to GitHub (section 12). No personal email in commits.
7. **Do not delete files;** archive them (the GCS recording trash is recoverable). Do not push to `main` unless asked.
8. The owner dislikes extra questions and wasted tokens: make a sensible choice, state it, continue.

## 3. System architecture

```
 LAPTOP (GCS, Python)                                              DRONE
 ┌──────────────────────────────────────────────┐        ┌──────────────────────────────┐
 │ nimbus_gcs (PyQt6 UI) ─ director/autonomy ─┐ │        │ Matek H743  Betaflight 4.5.4 │
 │ nimbus_core (Qt-free logic, bus, safety)   │ │  CRSF  │  + height_hold / xy_hold     │
 │ nimbus_pc (CRSF/MSP codec, sender, session)├─┼─ 2.4 ─►│  + compact state telemetry   │
 │   100 Hz RC frames, 16 ch   ───────────────┘ │  GHz   │  MTF-02P range+flow (UART3)  │
 │   telemetry + MSP replies   ◄────────────────┼─ELRS───┤  ELRS receiver  (4.1.0)      │
 │ vision process (SuperPoint+LightGlue VO,     │        └──────────────────────────────┘
 │   Depth Anything 3, detect, face, track)     │   DJI O4 camera ──► CosmoStreamer ─► RTSP / UDP
 │ mapping (ROS 2 Jazzy, RTAB-Map, optional)    │   (video is NOT on the radio; udp://0.0.0.0:3000)
 └──────────────────────────────────────────────┘
```

### 3.1 Uplink: 16 RC channels at 100 Hz (CRSF, ticks 172..1811, centre 992; 988..2012 µs)
| CH | Name | Meaning |
|---|---|---|
| 1-4 | ROLL, PITCH, THROTTLE, YAW | sticks (the explorer's steering in EXPLORE; throttle at minimum while the FC holds height) |
| 5 | ARM (AUX1) | >= 1700 µs armed; an RC frame older than 0.25 s makes the GCS disarm |
| 6, 7, 8 | ANGLE, TURTLE, BEEPER | held switches |
| 9 | HEIGHT (AUX5) | FC height hold on/target: 1150-1950 µs = 0-2.5 m; >= 1960 µs = LAND; 988 = hold off |
| 10 | HOVER REF (AUX6) | hover-reference throttle for the FC take-off |
| 11 | MODE (AUX7) | 1100-1400 µs = ALT HOLD (height only; the FC sideways loop pauses), otherwise POS HOLD. GCS sends 1200 (ALT) / 1900 (POS) / 1500 idle |
| 12, 16 | SPARE | no function (used by the channel test) |
| 13-15 | AID FWD, AID RIGHT, AID VALID | visual-odometry position aid: 1500 µs = 0, 1 µs = 1 mm, valid flag > 1900 µs; 10 Hz |
Channels 9-16 are only carried with ELRS **100Hz Full + Switch Mode 16ch Rate/2**; in Wide/Hybrid the FC silently reads CH13-16 as 1500. The DIAGNOSTICS channel test proves this by sending a pattern on CH9-16 while disarmed and reading it back (MSP_RC). Read-back order note: Betaflight reports roll, pitch, yaw, throttle; `radio_fc.decode_rc` swaps yaw/throttle to the CRSF order.

### 3.2 Downlink and MSP
- ELRS TX settings that are in use: **Packet Rate 100Hz Full, Telem Ratio 1:2, Switch Mode 16ch Rate/2** (read-only list script: `tools_scripts/link_bench_tests/elrs_tx_params_list_or_set.py`). A TX reflash resets Telem Ratio to Std.
- Frames from the FC: compact state **0xD6** (range + flow + EKF velocity + attitude in one frame, replacing the older 0xD3 + 0xD4 + attitude), battery, flight-mode text (a trailing `*` means disarmed), link statistics. The GCS re-labels each 0xD6 as the D3 and D4 frames it replaces before counting rates (`core._publish_frame_in`).
- MSP-over-CRSF is used for read-only probes (`nimbus_core/radio_fc.py`: channels 105, motors 104, arming flags 150) and the aid capability check (`nimbus_gcs/fc_caps.py`). MSP replies take telemetry slots from the state frames, so while armed the probe asks only once every 3 s.
- Measured on this link: about 15.8 state frames/s at 100Hz Full (18.1 at 250 Hz); the FC schedules frames inside what the ELRS link has left, so the PC cannot pin that rate.

### 3.3 Who flies what (the "hybrid" design)
- **The FC runs the control loops:** `height_hold` (target from CH9, hover reference CH10) and the sideways `xy_hold` (EKF velocity feedback; the VO position aid replaces its drift integral while the aid is fresh, only in HOLD).
- **The PC sends targets and sees telemetry:** flight director/autonomy (phases `ready, arming, armed, takeoff, climbing, holding, landing, landed, fault`), safety layer (tilt disarm, ceiling, lost-sensor landing), EXPLORE (the PC explorer sends stick wishes through `steering.py`; ALT HOLD is sent during EXPLORE so the sticks are pure tilt).
- **VO aid:** `vision_engine` (SuperPoint + LightGlue) tracks against a key frame set when the drone hovers still; the displacement goes to the FC on CH13-15 (and MSP2_DF_POS_AID for older firmware). It needs the 640x480 rectified picture, so 1920x1080 O4 frames are undistorted first (`nimbus_core/rectify.py`, calibration in `gazebo/config/o4_camera.yaml`). VO now starts by default after connect once video is live; the aid is sent only in HOLDING and never in EXPLORE.
- **Safety net:** SAFE (Esc) = disarm + throttle minimum + source idle. The GCS also forces the arm channel low on safety latches.

## 4. Firmware lineage (`firmware/fc_matek_h743/builds/`)
| Build | Status | Notes |
|---|---|---|
| v4.3, v4.5 | flashed earlier | first hybrid FC height hold |
| v4.6-rev1 | last "trusted feel" | rev1 gains; the baseline for later work |
| v4.7-ST | flashed 2026-11-23 | sideways hold + VO aid over radio (ST = sideways test) |
| v4.7.1, v4.7.2 | built | v4.7.2 reads the aid from RC channels CH13-15 |
| v4.7.3R | flashed 2026-11-24 | rev1 values + VO aid |
| **v4.7.4R** | **flashed on the drone (confirmed over USB 2026-11-25: build "Nov 25 2026 11:40:56", `debug_mode = DF_HOLD`)** | level-trim learning during the climb; debug mode `DF_HOLD` (debug[0..3] = EKF velocity fwd/right mm/s, VO aid fwd/right mm, -32768 = no fresh aid) |
| **v4.7.4R1** | built 2026-11-26, **not flashed, not flown** | v4.7.4R with the v4.5 height gains (0.9923 / 0.10 / 0.04) and sideways kp 2.5 / ki 0.10; the learned trim is kept (the simulation shows it, not the gains, removes the drift) |
| **v4.8** | built 2026-11-26, **not flashed, not flown** | R1 + sooner lift-off + VO aid usable in the climb + 20-byte state frame. **Update the GCS before flashing** (see its README) |
Flash with `python3 tools_scripts/df.py flash <fw.bin> [--debug-hold]` (owner go-ahead first; FC on USB, battery off, disarmed). Each build folder has its README, sources, patch and checksums; Betaflight source work tree lives outside the repo (`~/droneforge/bf_v473R/repo`, toolchain `~/droneforge/toolchain_x86`). The simulator loads the same control code as shared libraries (`gazebo/controllers/libhh_*.so`, selector `DF_FW=v474r`).
FC facts: serial port 2 = 262144 (MTF-02P) — Betaflight Configurator's Ports tab resets it; never enter the CLI within 60 s of power-up and note CLI `exit` reboots; the CLI pauses telemetry while open; FC USB must be unplugged to arm.

## 5. Repository map
| Path | What it is |
|---|---|
| `software/pc/nimbus_pc/` | CRSF/MSP codec, 100 Hz sender, Session (serial + telemetry), doctor, gamepad, sensor watchdog. CLI: `python3 -m nimbus_pc -h` |
| `software/pc/nimbus_core/` | Qt-free logic: `core.py` (owns the link, publishes on `bus.py`), `director.py` (the only thing the GUI calls to fly; pre-flight checks), `autonomy.py`, `safety.py`, `settings.py`, `radio_fc.py`, `rc_check.py`, `hold_log.py`, `rectify.py`, `steering.py`, `roam.py`, `simsession.py` (simulator backend), `recorder.py`, map helpers |
| `software/pc/nimbus_gcs/` | PyQt6 GCS. `main_window.py` shell; pages `fly_page.py`, `video_page.py`, `system_page.py` (SETUP), `flights_page.py`/analysis (ANALYZE), `diagnostics_page.py`, `settings_page.py`; `vision_bar.py` + `vision_engine.py` (models, VO aid loop); `components/` (mode tiles etc.), `viz/` (pre-flight list), `theme.py` (colours), `bfl.py` (Betaflight blackbox decoder) |
| `software/pc/realmap/` | `o4_map_node.py` (depth + FC odometry into RTAB-Map), `real_explore.py` (explorer on the real map); started by `map.sh` |
| `gazebo/` | Gazebo Harmonic + ROS 2 Jazzy simulator: worlds, controllers (FC bridge, house navigation), `vision/` (depth server, perception, map stream) |
| `firmware/fc_matek_h743/` | FC builds and patches (section 4). `firmware/tx_dongle/` is local-only (TX build material) |
| `tools_scripts/` | `df.py` (daily tool), `get_models.py`, FC config/log tools, link benches, calibration, video relay; its own `README.md` |
| `config/` | `flight_settings.json` (persisted measured numbers) |
| `docs/` | local only: specs, status logs, FC logs, flight recordings, analysis (`docs/00_START_HERE.md`) |
| root scripts | `setup_new_laptop.sh`, `map.sh`, `sim.sh`, `requirements*.txt`, `GIT_SYNC.md`, `HANDOFF.md` |

## 6. The GCS (six tabs, `1..6` keys)
FLY (live video + 2D map + height chart, pre-flight list, ARM/DISARM/LAND, target height, flight-mode tiles ALT HOLD | POS HOLD | EXPLORE), VIDEO (full video, vision overlays, latency test, map), SETUP (Sensors, Motor test, Calibration, Radio), ANALYZE (FC logs, live data), DIAGNOSTICS (RC channels 1-16 sent vs read back, rates, channel test), SETTINGS. Bottom bar = vision engines DEPTH, DETECT, FACE REC, TRACK, VO with one status line. Flight keys: hold A = ARM, hold T = TAKE OFF, D = DISARM, L = LAND, Esc = SAFE; `Ctrl+K` command palette.
**Pre-flight list** (`director.py`): "connected" and "no flight in progress" are hidden blocking gates; visible rows count as "N of 8 ready"; **FC armed** is an information-only row (ARMED/DISARMED from the FC's own mode text, never blocks, never counted). Mode tiles keep identical geometry in every state; a pressed tile shows an outline until the FC confirms.

## 7. Running things
```bash
./setup_new_laptop.sh [--install]          # check/install ROS 2, RTAB-Map, pip packages, models (GPU: requirements-gpu.txt)
python3 tools_scripts/get_models.py        # models ~1.4 GB (--check / --optional / --from <folder>)
python3 gazebo/vision/depth_server.py      # on the GPU laptop it must print CUDAExecutionProvider
cd software/pc && python3 -m nimbus_gcs --port /dev/serial/by-id/usb-Silicon_Labs_CP2102*-port0 --connect
./map.sh start --rectify                   # mapping (GCS must be running); ./map.sh stop|status|reset
./sim.sh start                             # Gazebo + GCS over the simulated radio; ./sim.sh stop
python3 tools_scripts/df.py test [-k name] # quiet test suite; df.py logs | flash | fcconfig | debughold on|off | summary <LOG.BFL>
QT_QPA_PLATFORM=offscreen python3 -m pytest software/pc/nimbus_core/tests software/pc/nimbus_gcs/tests software/pc/nimbus_pc/tests -q
```
Last full run (2026-11-25): 607 passed, 1 failed (an explore-button test that then followed the tile change and was fixed; the full suite was not re-run after). Vision/depth models and ELRS binding data are not in Git.
Typical real session: dongle in, DJI video to `udp://0.0.0.0:3000`, GCS up, mapping up, drone battery on, wait ~60 s, open DIAGNOSTICS (state frame ≈ 10-16 Hz, channel test ✔), then FLY. Never use `df.py logs` or open the FC CLI with the battery connected and a flight pending.

## 8. Data and logs you will use
- **Hold log (new):** `docs/status_logs/hold_live/<date>_<time>_hold.csv`, written only for a live radio session, 10 Hz armed / 1 Hz disarmed. Columns: what the GCS sends on CH5/9/11/13-15, what the FC reads back, the aid held by the core, FC velocity/attitude/range, flight phase, VO status (`vo_state, vo_hz, vo_msg, vo_anchor, vo_fresh_aid`), selected mode tile, safety latch and every GCS event line (e.g. `SAFE pressed`). Code: `nimbus_core/hold_log.py`.
- **FC blackbox logs:** `python3 tools_scripts/df.py logs --free` (FC on USB, disarmed, battery off, powered >= 60 s) saves to `docs/fc_logs/<date>/` and prints a summary including the DF_HOLD sideways-hold report. The FC stays in log mode until the USB is replugged.
- **GCS recordings:** `docs/flights/*.csv` (25 Hz) with `.events.txt`/`.summary.txt`.
Always line up the CSV and the FC log of the same flight before concluding anything.

## 9. Pitfalls that cost real time
- FC state frame "0.0 Hz" in DIAGNOSTICS was once a counting bug (frames are re-labelled); another time the FC was left in the CLI, which pauses telemetry.
- The VO engine ran at 30 Hz but produced nothing until the 1920x1080 frames were rectified to 640x480.
- The aid capability check over MSP can lose all replies on a busy link; it now retries every 10 s.
- Lift-off from the floor is decided by the ground I-term hold and motor balance (M4 was an imbalance earlier; M3 is the low motor now); take-off currently needs 6-8 s and bobs in height.
- LQ 0 with a live dongle: charge the pack, start the GCS first, power-cycle the drone battery.
- OpenCV 5.0 fisheye wants (1,N,3) arrays; code handles 4.x and 5.x. Every ONNX session needs `ort.preload_dlls()` first (done in the vision code).

## 10. Where things stand (2026-11-26) — honest
**Proven on the real drone:** link and 16-channel carriage (channel test ✔, error ≤ 1 µs), EKF/state telemetry, FC height hold with POS HOLD flights holding sideways speed at about 0.11-0.19 m/s rms (max about 0.3), SAFE/disarm path, hold logging, VO engine tracking on the bench (about 100 inliers, 30 Hz) after the rectify fix.
**Not yet proven:** the VO aid has never reached the FC's hold in a real flight (HOLDING was reached once, 2026-11-26 09:55, but SAFE followed 0.3 s later, before a key frame; see HANDOFF 5z); EXPLORE on the real drone; sign and scale of the aid in flight; whether ALT HOLD or POS HOLD is better (one flight each so far: ALT HOLD reached the 0.5 m hold, rms 0.11 m/s).
**Open problems, in order:** (1) slow, bobbing lift-off (6-8 s; a charged pack is advised, the pack read 13.9 V); (2) reach HOLDING and see the aid flow in a CSV; (3) idea queued: take off in ALT HOLD, switch to POS HOLD automatically at HOLDING; (4) possible ground key frame so the aid can start at take-off (the FC only applies it in HOLD, so using it during the climb needs a firmware change); (5) state-frame rate about 12 Hz at best (link limited; a smaller frame would need a new firmware).
**GCS redesign:** eight mock-up sets exist as PNGs in the owner's Pictures folder (not in the repo); no redesign has been implemented.

## 11. How to work in this repo
Qt-free logic goes in `nimbus_core`, UI in `nimbus_gcs`; the GUI only calls `FlightDirector`. Add a test with every change (`<package>/tests/`). Changes that touch flight behaviour: simulate first, state the unproven part, and never flash or arm without the owner. Update `HANDOFF.md` with a dated entry. Keep outputs short and checked.

## 12. Git and secrets
Publish with `bash ~/Music/publish_dronelite.sh --dry-run`, then `bash ~/Music/publish_dronelite.sh --commit-only "msg"` and `cd ~/DroneLite_push && git push` (the script sits outside the project folder on purpose). It copies the code only, excludes weights, flight logs and `docs/`, `archive/`, `reference/`, FC config dumps (they contain the receiver UID), TX material; replaces the ELRS binding phrase/UID with placeholders in the copy, refuses to push if either is still present, never force-pushes. New machine: `GIT_SYNC.md`.

## 13. Glossary
**CRSF** the serial radio protocol; **ELRS** ExpressLRS; **MSP** Betaflight's configuration protocol, here carried over CRSF; **EKF** the FC's Kalman filter; **VO** visual odometry (SuperPoint features + LightGlue matching); **aid** the VO displacement sent to the FC; **HOLDING** the director phase when the FC holds the target height; **HT / HH** mode-text suffixes seen in the logs while the height hold is engaged / has reached its target (inferred from the logs, not from Betaflight documentation); **DF_HOLD** the debug mode logging the sideways hold; **STAB** the mode text while disarmed/armed in angle mode; **O4** DJI O4 Pro camera; **DA3** Depth Anything 3; **SAFE** emergency disarm (Esc).
