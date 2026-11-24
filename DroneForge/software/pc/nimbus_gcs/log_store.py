"""File operations on flight recordings: list, move to a recoverable trash, restore, rename. Nothing here ever unlinks a recording
except empty_trash_older_than(). A recording is the set of files sharing one stem (<date>_<time>_<kind>[_<label>]).
Every refusal is a plain-text reason, never an exception the UI must catch."""
from __future__ import annotations

import os
import re
import shutil
import time
from datetime import datetime, timedelta, timezone
from typing import List, Optional, Tuple

KINDS = ("real-fc", "simulator", "bench-props-off")
SUFFIXES = (".events.txt", ".summary.txt", ".csv")            # longest-specific first
TRASH = ".trash"
LIVE_WINDOW_S = 3.0
_STEM = re.compile(r"^(\d{4}-\d\d-\d\d)_(\d{6})_(.+)$")
_TS = re.compile(r"^(\d{8}T\d{6}Z)_(.+)$")


def stem_of(name: str) -> Optional[str]:
    for s in SUFFIXES:
        if name.endswith(s) and len(name) > len(s): return name[:-len(s)]
    return None


def split_stem(stem: str) -> Tuple[str, str, str, str]:
    """-> (date, time, kind, label); kind 'other' for names that are not date_time_kind."""
    m = _STEM.match(stem)
    if not m: return "", "", "other", ""
    rest = m.group(3)
    for k in KINDS:
        if rest == k: return m.group(1), m.group(2), k, ""
        if rest.startswith(k + "_"): return m.group(1), m.group(2), k, rest[len(k) + 1:]
    return m.group(1), m.group(2), "other", rest


def sanitise_label(label: str) -> str:
    return re.sub(r"[^A-Za-z0-9_-]", "_", label.strip())[:40].strip("_ ")


def _files(folder: str, stem: str) -> List[str]:
    return [stem + s for s in SUFFIXES if os.path.isfile(os.path.join(folder, stem + s))]


def list_recordings(folder: str) -> List[dict]:
    """Newest first. Entry: stem, files, size (bytes), mtime, kind, label, when ('YYYY-MM-DD HH:MM:SS' or '')."""
    try: names = os.listdir(folder)
    except OSError: return []
    stems = sorted({stem_of(n) for n in names if stem_of(n) and os.path.isfile(os.path.join(folder, n))}, reverse=True)
    out = []
    for st in stems:
        files = _files(folder, st); size = 0; mt = 0.0
        for f in files:
            try: s = os.stat(os.path.join(folder, f)); size += s.st_size; mt = max(mt, s.st_mtime)
            except OSError: pass
        d, t, kind, label = split_stem(st)
        when = f"{d} {t[:2]}:{t[2:4]}:{t[4:]}" if d else ""
        out.append({"stem": st, "files": files, "size": size, "mtime": mt, "kind": kind, "label": label, "when": when})
    return out


def fmt_size(n: int) -> str:
    return f"{n / 1e6:.1f} MB" if n >= 1e6 else f"{max(n / 1e3, 0.1):.1f} kB" if n else "0 kB"


def live_reason(folder: str, stem: str, live_path: Optional[str] = None) -> str:
    """'' when the recording is safe to touch, else why it is not."""
    if live_path:
        lp = os.path.basename(live_path)
        if os.path.abspath(os.path.dirname(live_path) or folder) == os.path.abspath(folder) and stem_of(lp) == stem:
            return "this recording is being written right now"
    now = time.time()
    for f in _files(folder, stem):
        try:
            if now - os.stat(os.path.join(folder, f)).st_mtime < LIVE_WINDOW_S: return "this recording was changed in the last 3 s (probably still being written)"
        except OSError: pass
    return ""


def _utc() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def delete_recording(folder: str, stem: str, live_path: Optional[str] = None) -> Tuple[Optional[str], str]:
    """Move every file of the stem into <folder>/.trash/<UTC>_<stem>/. Returns (trash_dir, '') or (None, reason)."""
    files = _files(folder, stem)
    if not files: return None, "no such recording"
    why = live_reason(folder, stem, live_path)
    if why: return None, why
    dest = os.path.join(folder, TRASH, f"{_utc()}_{stem}"); n = 1
    while os.path.exists(dest): n += 1; dest = os.path.join(folder, TRASH, f"{_utc()}_{stem}") + f"-{n}"
    try:
        os.makedirs(dest)
        for f in files: shutil.move(os.path.join(folder, f), os.path.join(dest, f))
    except OSError as e:
        return None, f"could not move to trash: {e}"
    return dest, ""


def list_trash(folder: str) -> List[dict]:
    """Entries: path, stem, when (UTC), age_days, files, size. Newest first."""
    root = os.path.join(folder, TRASH); out = []
    try: names = sorted(os.listdir(root), reverse=True)
    except OSError: return out
    now = datetime.now(timezone.utc)
    for n in names:
        m = _TS.match(n); p = os.path.join(root, n)
        if not m or not os.path.isdir(p): continue
        try: ts = datetime.strptime(m.group(1), "%Y%m%dT%H%M%SZ").replace(tzinfo=timezone.utc)
        except ValueError: continue
        files = sorted(os.listdir(p)); size = sum(os.path.getsize(os.path.join(p, f)) for f in files if os.path.isfile(os.path.join(p, f)))
        out.append({"path": p, "stem": re.sub(r"-\d+$", "", m.group(2)) if not os.path.isfile(os.path.join(p, m.group(2) + ".csv")) else m.group(2),
                    "when": ts.strftime("%Y-%m-%d %H:%M"), "age_days": (now - ts).total_seconds() / 86400, "files": files, "size": size})
    return out


def restore(folder: str, trash_path: str) -> Tuple[bool, str]:
    """Move the files of a trash entry back into the flights dir. Refuses if any name already exists."""
    try: files = os.listdir(trash_path)
    except OSError: return False, "trash entry not found"
    clash = [f for f in files if os.path.exists(os.path.join(folder, f))]
    if clash: return False, f"cannot restore: {clash[0]} already exists"
    try:
        for f in files: shutil.move(os.path.join(trash_path, f), os.path.join(folder, f))
        os.rmdir(trash_path)
    except OSError as e:
        return False, f"could not restore: {e}"
    return True, ""


def empty_trash_older_than(folder: str, days: float = 30) -> int:
    """Permanently delete trash entries older than `days`. Returns how many entries were removed. Never called automatically."""
    n = 0
    for e in list_trash(folder):
        if e["age_days"] >= days:
            try: shutil.rmtree(e["path"]); n += 1
            except OSError: pass
    return n


def new_stem(stem: str, label: str) -> Tuple[Optional[str], str]:
    """The stem a rename would give: <date>_<time>_<kind>_<label>. -> (stem, '') or (None, reason)."""
    d, t, kind, _ = split_stem(stem)
    if not d: return None, "only recordings named date_time_kind can be renamed"
    lab = sanitise_label(label)
    if not lab: return None, "the label is empty (use letters, digits, - or _)"
    return f"{d}_{t}_{kind}_{lab}", ""


def rename_recording(folder: str, stem: str, label: str, live_path: Optional[str] = None) -> Tuple[Optional[str], str]:
    """Rename all files of the stem. Returns (new stem, '') or (None, reason)."""
    files = _files(folder, stem)
    if not files: return None, "no such recording"
    ns, why = new_stem(stem, label)
    if not ns: return None, why
    if ns == stem: return None, "that is already the name"
    why = live_reason(folder, stem, live_path)
    if why: return None, why
    if _files(folder, ns) or any(os.path.exists(os.path.join(folder, ns + s)) for s in SUFFIXES):
        return None, f"a recording named {ns} already exists"
    done = []
    try:
        for f in files:
            dst = ns + f[len(stem):]; os.rename(os.path.join(folder, f), os.path.join(folder, dst)); done.append((f, dst))
    except OSError as e:
        for f, dst in done:                                           # roll back a partial rename
            try: os.rename(os.path.join(folder, dst), os.path.join(folder, f))
            except OSError: pass
        return None, f"could not rename: {e}"
    return ns, ""
