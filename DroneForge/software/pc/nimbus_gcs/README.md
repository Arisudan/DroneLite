# DroneForge ground control station (PyQt6)

Run: `cd software/pc && python3 -m nimbus_gcs --port /dev/serial/by-id/usb-Silicon_Labs_CP2102*-port0 --connect`

## Layout: five workspaces (left rail; keys 1-5; Ctrl+Left/Right)
| Workspace | Sub-tabs | Use |
|---|---|---|
| FLY | (one page) | pre-flight checklist, ARM / TAKE OFF / LAND, target height, flight view, flight state, small live video |
| VIDEO | (one page) | live video (DJI via CosmoStreamer URL, USB capture, or analog VTX/VRX), snapshots |
| SETUP | Sensors, Motor test, Calibration, Radio | one-time and hardware checks |
| ANALYZE | Flight logs, Live charts, Telemetry, Radio frames, Replay | FC SD-card logs (download/decode/delete), recorded sessions |
| SETTINGS | Settings, Diagnostics | all configuration (built from `nimbus_core.settings.SCHEMA`), link and arming diagnostics |

Fixed header on every page: port + Connect + More (simulator / bench / rescan), LINK, BATT, MODE, STATE chips, and **SAFE** (Esc). One global status line under it. Ctrl+Tab cycles sub-tabs; the last workspace and sub-tab are remembered (`~/.config/droneforge/gcs_ui.json`).

## Rules
No dialogs, no tick boxes, no 'Advanced' sections, no yes/no questions: refusals and results appear in an inline status line. All flying goes through `core.director` (`nimbus_core/director.py`; API in `docs/specs/gcs-backend_api-reference.md`).

## Code map
`main_window.py` window shell + navigation; `workspace.py` sub-tab host (lazy pages); `fly_page.py`, `video_page.py` (+`video_source.py`, `video_pip.py`), `sensor_tab.py`, `motor_tab.py`, `calibration_page.py`, `logs_tab.py` (+`analysis_page.py`, `bfl.py`, `fc_logs.py`), `replay_page.py`, `settings_page.py`, `charts_page.py`, `telemetry_page.py`, `frames_page.py`, `system_page.py` (Radio + Diagnostics), `model.py` (state for the UI), `shell.py` / `widgets.py` (look and shared widgets).
Tests: `python3 ~/Music/DroneForge/tools_scripts/df.py test gcs` (GCS) or `df.py test` (everything).
