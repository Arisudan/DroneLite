"""Controller: owns the link, mode (live/settings), caches and background poller. Methods are blocking."""
import glob
import os
import threading
import time

from . import msp
from .cli import Cli, CliError, group_of
from .link import Link, LinkError, classify
from .safety import Safety, SafetyError, read_armed

KNOWN = (LinkError, SafetyError, CliError, msp.MspError, ValueError, KeyError)


def list_ports(extra=None):
    ports, seen = [], set()
    for p in sorted(glob.glob("/dev/serial/by-id/*")):
        seen.add(os.path.realpath(p))
        ports.append({"path": p, "description": os.path.basename(p), "kind": classify(p)})
    try:
        from serial.tools import list_ports as lp
        for c in lp.comports():
            if os.path.realpath(c.device) not in seen and ("/ttyACM" in c.device or "/ttyUSB" in c.device or "/ttyBFEMU" in c.device):   # no built-in ttyS noise
                ports.append({"path": c.device, "description": c.description or c.device, "kind": "other"})
    except Exception:
        pass
    return (extra or []) + ports


class Controller:
    def __init__(self, backup_dir, extra_ports=None):
        self.link = Link()
        self.cli = Cli(self.link)
        self.safety = Safety(backup_dir, armed_fn=self._armed, kind_fn=lambda: self.link.kind if self.link.is_open else None)
        self.extra_ports = extra_ports or []
        self.oplock = threading.RLock()
        self.mode = None
        self.info = {}
        self.stat = {"armed": False, "cycle_us": 0, "cpu_load": 0, "i2c_errors": 0}
        self.armed_cache = False
        self.dump = None
        self.meta = {}
        self.subs = []
        self._stop = False
        self._poller = threading.Thread(target=self._poll, name="poller", daemon=True)
        self._poller.start()

    # ---- events
    def subscribe(self, cb):
        self.subs.append(cb)

    def unsubscribe(self, cb):
        if cb in self.subs:
            self.subs.remove(cb)

    def emit(self, event, data):
        for cb in list(self.subs):
            try:
                cb(event, data)
            except Exception:
                pass

    def shutdown(self):
        self._stop = True
        self.link.close()

    # ---- dispatch
    def call(self, method, params):
        fn = getattr(self, "m_" + method.replace(".", "_"), None)
        if fn is None:
            raise ValueError("unknown method: %s" % method)
        return fn(**(params or {}))

    # ---- state helpers
    def _armed(self):
        if self.mode == "live" and self.link.is_open:
            self.armed_cache = read_armed(self.link)
        return self.armed_cache

    def _status(self):
        s = {"connected": self.link.is_open, "mode": self.mode if self.link.is_open else None}
        s.update(self.info)
        s.update(self.stat)
        s["packet_errors"] = self.link.packet_errors
        s["port"] = self.link.path
        return s

    def _fetch_info(self):
        L = self.link
        g = lambda c: L.msp(c)
        info = msp.parse_info(g(1), g(2), g(3), g(4), g(5), g(160))
        st = msp.parse_status_ex(g(150))
        self.armed_cache = st["armed"]
        self.stat = {k: st[k] for k in ("armed", "cycle_us", "cpu_load", "i2c_errors")}
        self.info = info
        return info

    def _need_live(self):
        if self.mode != "live":
            raise LinkError("not in LIVE mode")

    def _need_settings(self):
        if self.mode != "settings":
            raise LinkError("not in SETTINGS mode; open Settings first")

    # ---- poller
    def _poll(self):
        tick, fails = 0, 0
        while not self._stop:
            time.sleep(0.1)
            tick += 1
            try:
                if not self.oplock.acquire(blocking=False):
                    continue
                try:
                    if self.mode == "live" and self.link.is_open:
                        live = self._live()
                        self.emit("live", live)
                        if tick % 5 == 0:
                            st = msp.parse_status_ex(self.link.msp(msp.STATUS_EX))
                            self.armed_cache = st["armed"]
                            self.stat = {k: st[k] for k in ("armed", "cycle_us", "cpu_load", "i2c_errors")}
                        fails = 0
                    if self.mode and not self.link.is_open and self.link.lost:
                        self.mode = None
                        self.emit("error", "FC connection lost")
                finally:
                    self.oplock.release()
            except (LinkError, msp.MspError):
                fails += 1
            except Exception:
                fails += 1
            if tick % 5 == 0:
                self.emit("status", self._status())

    def _live(self):
        L = self.link
        att = msp.parse_attitude(L.msp(msp.ATTITUDE))
        imu = msp.parse_imu(L.msp(msp.RAW_IMU))
        an = msp.parse_analog(L.msp(msp.ANALOG))
        return {"attitude": att, "rc": msp.parse_u16s(L.msp(msp.RC)), "motors": msp.parse_u16s(L.msp(msp.MOTOR)),
                "gyro": imu["gyro"], "acc": imu["acc"],
                "analog": {"vbat": an["vbat"], "amps": an["amps"], "rssi": an["rssi"]}, "range_cm": None}

    # ---- methods
    def m_ports_list(self):
        return list_ports(self.extra_ports)

    def m_fc_connect(self, port, baud=115200):
        with self.oplock:
            self.mode = None
            self.safety.reset()
            self.dump, self.meta = None, {}
            self.link.open(port, baud)
            try:
                info = self._fetch_info()
            except Exception:
                self.link.close()
                raise LinkError("no MSP response from %s (if the FC is stuck in CLI mode, replug it)" % port)
            self.mode = "live"
        return self.m_fc_info()

    def m_fc_disconnect(self):
        with self.oplock:
            self.mode = None
            self.link.close()
            self.dump = None
        return {"connected": False}

    def m_fc_info(self):
        return self._status()

    def _reconnect_live(self, write):
        self.link.reboot_reconnect(write)
        self.mode = "live"
        self.dump, self.meta = None, {}
        self.safety.serial_unlocked = False
        self._fetch_info()

    def m_mode_settings(self, refresh=False):
        with self.oplock:
            if self.mode == "settings":
                return self.m_settings_dump(refresh=refresh)
            self._need_live()
            if read_armed(self.link):
                self.armed_cache = True
                raise SafetyError("the FC is ARMED: refusing to enter SETTINGS. Disarm first")
            self.armed_cache = False
            self.cli.enter()
            self.mode = "settings"
            self.dump = self.cli.dump_all()
            self.emit("status", {**self._status(), "mode": "settings", "connected": True})
            return self.m_settings_dump()

    def m_cli_open(self):
        self.m_mode_settings()
        return {"mode": "settings"}

    def m_mode_live(self):
        with self.oplock:
            if self.mode == "settings":
                self._reconnect_live(b"exit\n")
            return self.m_fc_info()

    def m_cli_close(self):
        return self.m_mode_live()

    def m_fc_reboot(self, target="normal"):
        with self.oplock:
            self._need_live()
            if self.armed_cache or read_armed(self.link):
                raise SafetyError("refusing to reboot while ARMED")
            if target == "bootloader":
                self.link.msp(msp.SET_REBOOT, bytes([1]))
                self.mode = None
                self.link.close()
                return {"bootloader": True, "note": "FC is in DFU bootloader; reconnect after flashing"}
            self._reconnect_live(msp.encode_request(msp.SET_REBOOT, bytes([0])))
            return self.m_fc_info()

    def m_fc_calibrate_acc(self):
        with self.oplock:
            self._need_live()
            if read_armed(self.link):
                raise SafetyError("refusing to calibrate while ARMED")
            self.link.msp(msp.ACC_CALIBRATION)
            return {"started": True, "note": "Keep the drone level and still for a few seconds."}

    def m_settings_dump(self, refresh=False):
        with self.oplock:
            self._need_settings()
            if refresh or self.dump is None:
                self.dump = self.cli.dump_all()
            d = self.dump
            return {k: d[k] for k in ("version_line", "settings", "lines", "profile", "rateprofile")}

    def m_settings_meta(self, name):
        with self.oplock:
            self._need_settings()
            if name not in self.meta:
                m = self.cli.get(name)
                self.meta[name] = {"min": m["min"], "max": m["max"], "default": m["default"],
                                   "table": m["table"], "type": m["type"]}
            return self.meta[name]

    def m_settings_diff(self):
        with self.oplock:
            self._need_settings()
            return {"text": self.cli.diff_all()}

    def m_settings_export(self):
        with self.oplock:
            self._need_settings()
            if self.dump is None:
                self.dump = self.cli.dump_all()
            return {"text": self.dump["raw"]}

    def m_serial_unlock(self, on=True):
        self.safety.serial_unlocked = bool(on)
        return {"serial_unlocked": self.safety.serial_unlocked}

    def m_settings_apply(self, changes=None, lines=None):
        with self.oplock:
            self._need_settings()
            if self.dump is None:
                self.dump = self.cli.dump_all()
            cur = {s["name"]: s["value"] for s in self.dump["settings"]}
            res = self.safety.apply(self.cli, cur, self.dump["lines"], changes or {}, lines or [])
            fresh = res.pop("_dump", None)
            if fresh:
                self.dump = fresh
            by = {s["name"]: s for s in self.dump["settings"]}
            for a in res["applied"]:
                if a["name"] in by:
                    by[a["name"]]["value"] = a["new"]
            return res

    def m_settings_import(self, text):
        from .cli import parse_dump
        d = parse_dump(text)
        changes = {s["name"]: s["value"] for s in d["settings"]}
        lines = [l for fam in ("serial", "aux", "feature") for l in d["lines"][fam]]
        return self.m_settings_apply(changes=changes, lines=lines)

    def m_settings_save(self):
        with self.oplock:
            self._need_settings()
            self.safety.check_writable()
            self._reconnect_live(b"save\n")
            info = self.m_fc_info()
            self.emit("status", {**self._status(), **info, "mode": "live", "connected": True})
            return {"saved": True, "info": info}

    def m_cli_send(self, line):
        with self.oplock:
            self._need_settings()
            self.safety.check_cli(line)
            w = line.split()[0].lower() if line.split() else ""
            if w == "save":
                r = self.m_settings_save()
                text = "Saved. FC rebooted; back in LIVE mode."
            elif w == "exit":
                self.m_mode_live()
                text = "FC rebooted (unsaved changes lost); back in LIVE mode."
            else:
                text = self.cli.run(line)
            self.emit("cli", {"text": text})
            return {"text": text}
