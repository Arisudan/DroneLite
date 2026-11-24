# Betaflight Configurator (local, DroneForge)

Local browser app for the custom Betaflight 4.5.4 build (Matek H743). Spec: `CONTRACT.md`.

## Run

Real FC: plug in the FC USB (battery off is recommended for configuration, props stay on but nothing arms), then

    ./run.sh

This starts the Python server on http://127.0.0.1:8765, opens `chromium --app=...` (`/snap/bin/chromium` if it exists, else firefox) and kills the server when the window closes. Choose the `*Betaflight*` port in the UI. The CP2102 radio port is never opened automatically; it is listed with kind `radio` and writes over it are refused.

Demo (no hardware, fake FC on a pty): `./run.sh --demo`. Toggle the fake ARMED flag with `curl http://127.0.0.1:8765/demo/armed/1` (and `/0`).

Server only: `python3 -m server [--demo] [--host 127.0.0.1] [--port 8765]`.
Tests (emulator only, no real port is ever opened): `python3 -m unittest discover -s tests`.

## Safety rules (enforced in code, tested)

- Writes are refused while ARMED (MSP_STATUS_EX is read before entering SETTINGS, which is refused when armed; entering CLI makes the FC refuse to arm) and when the active link is not the FC USB.
- The first write of a session first saves `diff all` to `backups/<UTC>_before-change.txt`; if that fails nothing is written.
- MSP allow-list lives in one function (`msp.encode_request`): reads, ACC_CALIBRATION, SET_REBOOT, EEPROM_WRITE. Motor, serial-config, RC and reset commands raise.
- `serial` lines are read-only until `serial.unlock` (the Ports tab button). `serial 2 262144` is the MTF-02P range/flow sensor on UART3.
- No motor test, no arming, no RC injection. The free-form CLI tab only accepts an allow-list of commands (`motor`, `dfu`, `bl`, `defaults`, `resource ...` are blocked).
- `range_cm` in `live` events is always `null`: the rangefinder MSP read is not on the allow-list.

## Reboot behaviour

Leaving the CLI with `save` or `exit` reboots the FC. The USB port disappears for about 2 s. The server closes the port, waits for the same path to return, reopens it and waits for MSP, then reports LIVE mode again. `settings.save` and `mode.live` block for that time (about 2 to 6 s on real hardware). `exit` discards unsaved changes.

## Why diff-only writes

The official Configurator rewrites whole sections (including the serial ports, which resets UART3 `serial 2 262144`) and refuses values on this custom firmware. Here only names whose value the owner actually changed are sent as `set name = value`, each is re-read with `get` to verify, and everything else on the FC is left untouched, so custom settings survive and every change is small, visible and reversible from the backup.

## Layout

`server/` msp, link, cli, safety, controller (modes, poller), app (ASGI, static `../web/` plus `/ws`), emulator, `__main__`. `tests/` unittest. `backups/` auto backups.
