"""Safety layer: armed check, backup before first write, diff-only apply with verify, serial lock."""
import datetime
import os
from pathlib import Path

from . import msp
from .cli import CliError, effective_features, same_value

LINE_PREFIXES = {"aux", "serial", "feature", "beeper", "map", "rxrange", "adjrange", "vtx", "vtxtable", "led", "color"}
CLI_READ = {"get", "dump", "diff", "version", "status", "help", "tasks", "flash_info", "sd_info"}
CLI_WRITE = {"set", "profile", "rateprofile", "name"} | LINE_PREFIXES
SERIAL_NOTE = "serial 2 262144 is the MTF-02P range/flow sensor port (UART3)"


class SafetyError(Exception):
    pass


def read_armed(link):
    return msp.parse_status_ex(link.msp(msp.STATUS_EX))["armed"]


def norm_line(l):
    return " ".join(l.split())


class Safety:
    def __init__(self, backup_dir, armed_fn=None, kind_fn=None):
        self.backup_dir = Path(backup_dir)
        self.armed_fn = armed_fn or (lambda: False)
        self.kind_fn = kind_fn or (lambda: "betaflight")
        self.backup_path = None
        self.serial_unlocked = False

    def reset(self):
        self.backup_path = None
        self.serial_unlocked = False

    def check_writable(self):
        if self.kind_fn() == "radio":
            raise SafetyError("refusing to write: the active link is the radio port, not the FC USB")
        if self.kind_fn() is None:
            raise SafetyError("refusing to write: FC USB is not the active link")
        if self.armed_fn():
            raise SafetyError("refusing to write: the FC is ARMED. Disarm first")

    def ensure_backup(self, cli):
        if self.backup_path:
            return self.backup_path
        text = cli.diff_all()
        self.backup_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        p = self.backup_dir / ("%s_before-change.txt" % stamp)
        n = 1
        while p.exists():
            p = self.backup_dir / ("%s_before-change_%d.txt" % (stamp, n))
            n += 1
        try:
            p.write_text(text + "\n")
        except OSError as e:
            raise SafetyError("cannot write backup, refusing to change anything: %s" % e)
        self.backup_path = str(p)
        return self.backup_path

    def check_line(self, line):
        parts = line.split()
        if not parts or parts[0] not in LINE_PREFIXES:
            raise SafetyError("line type not allowed: %s" % (parts[0] if parts else "(empty)"))
        if parts[0] == "serial" and not self.serial_unlocked:
            raise SafetyError("serial editing is locked (%s). Unlock it in Ports first" % SERIAL_NOTE)

    def check_cli(self, line):
        """Guard for the free-form CLI tab."""
        parts = line.split()
        if not parts:
            return
        w = parts[0].lower()
        if w == "resource" and len(parts) == 1:
            return
        if w in ("save", "exit"):
            return
        if w in CLI_READ:
            return
        if w in CLI_WRITE:
            if len(parts) == 1 and w in LINE_PREFIXES:
                return  # listing only
            self.check_writable()
            if w == "serial":
                self.check_line(line)
            return
        raise SafetyError("command '%s' is not allowed from the app CLI" % w)

    # ---- diff-only apply
    def apply(self, cli, settings, lines_cur, changes, lines):
        """settings: {name: value}, lines_cur: dict family->[lines]. Returns result dict
        (with '_dump' when a fresh dump was read for line verification)."""
        self.check_writable()
        todo, skipped, failed = [], [], []
        for name, val in (changes or {}).items():
            if name not in settings:
                failed.append({"name": name, "error": "unknown setting"})
            elif same_value(settings[name], val):
                skipped.append(name)
            else:
                todo.append((name, str(val).strip()))
        line_todo = []
        feats = effective_features(lines_cur.get("feature", []))
        all_cur = {norm_line(l) for fam in lines_cur.values() for l in fam}
        for raw in lines or []:
            l = norm_line(raw)
            try:
                self.check_line(l)
            except SafetyError as e:
                failed.append({"name": l, "error": str(e)})
                continue
            p = l.split()
            if p[0] == "feature" and len(p) == 2:
                want = not p[1].startswith("-")
                if (p[1].lstrip("-") in feats) == want:
                    skipped.append(l)
                    continue
            elif any(c == l or c.startswith(l + " ") for c in all_cur):
                skipped.append(l)
                continue
            line_todo.append(l)
        result = {"applied": [], "failed": failed, "skipped": skipped, "backup": self.backup_path}
        if not todo and not line_todo:
            return result
        result["backup"] = self.ensure_backup(cli)
        for name, new in todo:
            old = settings[name]
            try:
                cli.set(name, new)
                got = cli.get(name)["value"]
                result["applied"].append({"name": name, "old": old, "new": new, "verified": same_value(got, new)})
            except CliError as e:
                failed.append({"name": name, "error": str(e)})
        for l in line_todo:
            out = cli.run(l)
            if "invalid" in out.lower() or "error" in out.lower():
                failed.append({"name": l, "error": out.strip().splitlines()[0]})
        if line_todo:
            d = cli.dump_all()
            result["_dump"] = d
            cur = {norm_line(x) for fam in d["lines"].values() for x in fam}
            feats = effective_features(d["lines"]["feature"])
            bad = {f["name"] for f in failed}
            for l in line_todo:
                if l in bad:
                    continue
                p = l.split()
                if p[0] == "feature":
                    ok = (p[1].lstrip("-") in feats) == (not p[1].startswith("-"))
                else:
                    ok = any(c == l or c.startswith(l + " ") for c in cur)
                result["applied"].append({"name": l, "old": None, "new": l, "verified": ok})
        return result
