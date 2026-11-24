"""Betaflight CLI helpers: dump/get parsing and a thin session wrapper over Link."""
import re

LINE_FAMILIES = ("serial", "aux", "feature", "resource")
IGNORED = {"batch", "defaults", "save", "diff", "dump", "exit"}

GROUPS = [
    ("config", ("small_angle", "name", "mixer", "craft", "pilot", "board", "manufacturer")),
    ("filters", ("gyro_lpf", "dterm_lpf", "gyro_notch", "dterm_notch", "dyn_notch", "gyro_rpm", "yaw_lowpass", "dyn_lpf")),
    ("receiver", ("rx_", "serialrx", "mid_rc", "min_check", "max_check", "deadband", "yaw_deadband", "rssi", "rc_smoothing", "airmode")),
    ("rates", ("roll_", "pitch_", "yaw_", "thr_", "rates_type", "throttle_limit")),
    ("pid", ("p_", "i_", "d_", "f_", "simplified", "feedforward", "anti_gravity", "iterm", "tpa", "throttle_boost", "level_", "angle", "horizon", "abs_control", "thrust")),
    ("motor", ("motor", "dshot", "dyn_idle", "pwm")),
    ("power", ("vbat", "ibata", "ibat", "battery", "batt_", "current_meter")),
    ("failsafe", ("failsafe",)),
    ("osd", ("osd_",)),
    ("blackbox", ("blackbox",)),
    ("sensors", ("acc_", "baro", "mag_", "gyro_hardware", "gps")),
]


class CliError(Exception):
    pass


def group_of(name):
    for g, prefixes in GROUPS:
        if name.startswith(prefixes):
            return g
    return "other"


def parse_dump(text):
    text = text.replace("\r", "")
    version_line, section, items = "", ("master", 0), []
    lines = {"serial": [], "aux": [], "feature": [], "resource": [], "other": []}
    prof = rate = 0
    for raw in text.split("\n"):
        l = raw.strip()
        if not l:
            continue
        if l.startswith("#"):
            if l.startswith("# Betaflight") and not version_line:
                version_line = l.lstrip("# ").strip()
            continue
        m = re.match(r"(profile|rateprofile)\s+(\d+)$", l)
        if m:
            n = int(m.group(2))
            if m.group(1) == "profile":
                prof, section = n, ("profile", n)
            else:
                rate, section = n, ("rateprofile", n)
            continue
        m = re.match(r"set\s+(\S+)\s*=\s*(.*)$", l)
        if m:
            items.append((m.group(1), m.group(2).strip(), section))
            continue
        word = l.split()[0]
        if word in IGNORED:
            continue
        lines[word if word in LINE_FAMILIES else "other"].append(" ".join(l.split()))
    settings = []
    for name, value, (sec, n) in items:
        if sec == "profile" and n != prof or sec == "rateprofile" and n != rate:
            continue
        settings.append({"name": name, "value": value, "group": group_of(name), "section": sec})
    return {"version_line": version_line, "settings": settings, "lines": lines, "profile": prof, "rateprofile": rate}


def effective_features(feature_lines):
    on = set()
    for l in feature_lines:
        parts = l.split()
        if len(parts) < 2:
            continue
        f = parts[1]
        if f.startswith("-"):
            on.discard(f[1:])
        else:
            on.add(f)
    return on


def _num(s):
    return float(s) if re.match(r"^-?\d+(\.\d+)?$", s.strip()) else None


def parse_get(text):
    out, cur = {}, None
    for l in text.replace("\r", "").split("\n"):
        l = l.strip()
        m = re.match(r"^([A-Za-z0-9_]+)\s*=\s*(.*)$", l)
        if m:
            cur = {"name": m.group(1), "value": m.group(2).strip(), "min": None, "max": None,
                   "default": None, "table": None, "type": "string"}
            out[cur["name"]] = cur
            continue
        if cur is None:
            continue
        m = re.match(r"Allowed range:\s*(-?\d+(?:\.\d+)?)\s*-\s*(-?\d+(?:\.\d+)?)", l)
        if m:
            cur["min"], cur["max"] = _num(m.group(1)), _num(m.group(2))
            if "." in m.group(1) + m.group(2):
                cur["type"] = "float"
            else:
                cur["min"], cur["max"] = int(cur["min"]), int(cur["max"])
                cur["type"] = "int"
            continue
        m = re.match(r"Allowed values:\s*(.*)$", l)
        if m:
            cur["table"] = [v.strip() for v in m.group(1).split(",") if v.strip()]
            cur["type"] = "table"
            continue
        m = re.match(r"Default value:\s*(.*)$", l)
        if m:
            d = m.group(1).strip()
            cur["default"] = int(d) if re.match(r"^-?\d+$", d) else (float(d) if re.match(r"^-?\d+\.\d+$", d) else d)
    return out


def same_value(a, b):
    a, b = str(a).strip(), str(b).strip()
    try:
        return float(a) == float(b)
    except ValueError:
        return a.upper() == b.upper()


class Cli:
    def __init__(self, link):
        self.link = link

    def enter(self):
        self.link.cli_enter()

    def run(self, line, timeout=8.0):
        return self.link.cli_cmd(line, timeout)

    def dump_all(self):
        text = self.run("dump all", 40.0)
        d = parse_dump(text)
        d["raw"] = text
        return d

    def diff_all(self):
        return self.run("diff all", 40.0)

    def get(self, name):
        out = parse_get(self.run("get " + name))
        if name not in out:
            raise CliError("unknown setting: %s" % name)
        return out[name]

    def set(self, name, value):
        out = self.run("set %s = %s" % (name, value))
        if "invalid" in out.lower() or "error" in out.lower() or "set to" not in out:
            raise CliError("%s: %s" % (name, out.strip().splitlines()[0] if out.strip() else "no reply"))
        return out

    def save(self):
        self.link.reboot_reconnect(b"save\n")

    def exit(self):
        self.link.reboot_reconnect(b"exit\n")
