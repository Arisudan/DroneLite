"""FC flight logs on the SD card: list, download, delete. No Qt here, so it is unit-tested.

How it works (and why): Betaflight cannot list, read or delete SD-card files over the normal USB serial link (MSP can only report the card's status).
The FC can instead present its SD card to the PC as a USB drive ("mass storage mode", CLI command `msc`). While it does that the FC does not talk serial and
cannot fly, so this is only done on request, with the FC on USB and DISARMED. Ejecting the drive makes the FC restart normally.

Safety rules enforced here:
  * entering log mode is refused unless the FC reports DISARMED (MSP_STATUS), and the caller says the autonomy is idle;
  * a log is deleted from the card ONLY if a copy was downloaded and its SHA-256 equals the card file's SHA-256 right now (or the file is empty);
  * nothing outside the Betaflight blackbox files (*.BFL / *.BBL) on the card is ever touched.
"""
from __future__ import annotations

import glob
import hashlib
import json
import os
import select
import shutil
import struct
import subprocess
import termios
import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
DEST_ROOT = os.path.join(ROOT, "docs", "fc_logs")
LEDGER = os.path.join(DEST_ROOT, ".ledger.json")
MIN_UPTIME_S = 60
LOG_SUFFIXES = (".bfl", ".bbl")                       # Betaflight blackbox only: never .txt or anything else that may share the card


def fc_serial_port() -> Optional[str]:
    """The FC's own USB serial port (not the radio dongle), or None."""
    p = sorted(glob.glob("/dev/serial/by-id/*Betaflight*"))
    return p[0] if p else None


def _open(port: str) -> int:
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    a = termios.tcgetattr(fd); a[0] = a[1] = 0; a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL; a[3] = 0
    termios.tcsetattr(fd, termios.TCSANOW, a)
    return fd


def _msp(fd: int, cmd: int, timeout: float = 1.0) -> Optional[bytes]:
    os.write(fd, b"$M<" + bytes([0, cmd, cmd])); buf = b""; end = time.time() + timeout
    while time.time() < end:
        if select.select([fd], [], [], 0.1)[0]:
            buf += os.read(fd, 512)
        i = buf.find(b"$M>")
        if i >= 0 and len(buf) >= i + 5 and len(buf) >= i + 6 + buf[i + 3]:
            return buf[i + 5:i + 5 + buf[i + 3]]
    return None


def fc_is_disarmed(port: str) -> Tuple[Optional[bool], str]:
    """(True/False, why). None = could not tell (treated as NOT safe)."""
    try:
        fd = _open(port)
    except OSError as e:
        return None, f"cannot open the FC port ({e})"
    try:
        s = _msp(fd, 101)                                   # MSP_STATUS: boxModeFlags at byte 6, bit 0 = ARM
        if not s or len(s) < 10:
            return None, "the FC did not answer"
        flags = struct.unpack_from("<I", s, 6)[0]
        return (not bool(flags & 1)), ("disarmed" if not flags & 1 else "ARMED")
    finally:
        os.close(fd)


def fc_usb_age_s(port: str) -> Optional[float]:
    """Seconds since the FC's USB port appeared (udev creates the by-id link at plug-in / reboot). Reading it never touches the FC.
    NOTE: never use the FC's CLI to find the uptime: leaving the CLI ('exit') reboots the FC, which would itself interrupt the SD set-up."""
    try:
        return max(0.0, time.time() - os.lstat(port).st_mtime)
    except OSError:
        return None


def enter_mass_storage(port: str) -> Tuple[bool, str]:
    """CLI `msc`. Only call after fc_is_disarmed() said True and fc_usb_age_s() >= MIN_UPTIME_S (checked here)."""
    age = fc_usb_age_s(port)
    if age is None or age < MIN_UPTIME_S:
        return False, (f"the FC's USB connection is only {age:.0f} s old: wait until it has been powered for at least {MIN_UPTIME_S} s (its SD card file system is still being set up), then try again"
                       if age is not None else "cannot tell how long the FC has been powered: wait a minute after plugging it in")
    try:
        fd = _open(port)
    except OSError as e:
        return False, f"cannot open the FC port ({e})"
    try:
        os.write(fd, b"#\r\n"); time.sleep(0.5)
        os.write(fd, b"msc\r\n")
        out = b""; end = time.time() + 2.5
        while time.time() < end:
            if select.select([fd], [], [], 0.1)[0]:
                try:
                    out += os.read(fd, 1024)
                except OSError:
                    break                                    # the FC left: it is rebooting into mass storage
        text = out.decode(errors="replace")
        if "not present" in text or "failed to initialize" in text:
            return False, "the FC reports: storage not present or failed to initialize (is an SD card inserted?)"
        return True, "the FC is restarting in mass-storage mode"
    finally:
        try:
            os.close(fd)
        except OSError:
            pass


@dataclass
class Card:
    device: str
    mountpoint: Optional[str]
    label: str = ""


def _lsblk() -> List[dict]:
    try:
        out = subprocess.run(["lsblk", "-J", "-b", "-o", "NAME,PATH,TRAN,VENDOR,MODEL,SIZE,TYPE,RM,MOUNTPOINT,FSTYPE,LABEL"], capture_output=True, text=True, timeout=10).stdout
        return json.loads(out).get("blockdevices", [])
    except (OSError, ValueError, subprocess.SubprocessError):
        return []


def usb_partitions(devs: Optional[List[dict]] = None) -> List[dict]:
    """Partitions (or whole disks without partitions) of removable USB disks."""
    out = []
    for d in devs if devs is not None else _lsblk():
        if (d.get("tran") or "").lower() != "usb":
            continue
        kids = [c for c in (d.get("children") or []) if c.get("type") == "part"]
        out.extend(kids if kids else ([d] if d.get("fstype") else []))
    return out


def find_card(before: Optional[List[str]] = None) -> Optional[Card]:
    """The FC's SD card as a USB disk. `before` = device paths that existed before entering the mode (they are ignored)."""
    cands = [p for p in usb_partitions() if p.get("path") not in (before or [])]
    if not cands:
        return None
    def score(p):                                            # prefer something that looks like the FC
        t = f"{p.get('vendor') or ''} {p.get('model') or ''} {p.get('label') or ''}".lower()
        return -sum(k in t for k in ("betaflight", "stm", "msc", "virtual", "fc", "sd"))
    p = sorted(cands, key=score)[0]
    return Card(p["path"], p.get("mountpoint"), p.get("label") or "")


def mount(card: Card) -> Tuple[bool, str]:
    if card.mountpoint:
        return True, card.mountpoint
    try:
        r = subprocess.run(["udisksctl", "mount", "-b", card.device], capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.SubprocessError) as e:
        return False, f"cannot mount: {e}"
    txt = (r.stdout + r.stderr).strip()
    if r.returncode == 0 and " at " in txt:
        card.mountpoint = txt.rsplit(" at ", 1)[1].strip().rstrip(".")
        return True, card.mountpoint
    if "already mounted" in txt and " at " in txt:
        card.mountpoint = txt.rsplit(" at ", 1)[1].strip().strip("`'.")
        return True, card.mountpoint
    return False, txt or "mount failed"


def _disk_of(device: str) -> Tuple[str, List[str]]:
    """(whole-disk path, [mounted partition paths]) for a partition path such as /dev/sda1."""
    for d in _lsblk():
        kids = d.get("children") or []
        if d.get("path") == device or any(c.get("path") == device for c in kids):
            mounted = [c["path"] for c in kids if c.get("mountpoint")] or ([d["path"]] if d.get("mountpoint") else [])
            return d["path"], mounted
    return device, [device]


def eject(card: Card) -> str:
    """Flush, unmount EVERY partition of the card drive (a card with several partitions stays 'in use' otherwise) and power the drive off; the FC then restarts."""
    try:
        subprocess.run(["sync"], timeout=60)
        disk, mounted = _disk_of(card.device)
        for part in sorted(set(mounted + [card.device]), reverse=True):
            subprocess.run(["udisksctl", "unmount", "-b", part], capture_output=True, text=True, timeout=30)
        subprocess.run(["udisksctl", "power-off", "-b", disk], capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.SubprocessError) as e:
        return f"eject problem: {e}"
    return "ejected"


# ---------------------------------------------------------------------------------------------------- files
@dataclass
class LogFile:
    path: str
    name: str
    size: int
    mtime: float


def list_logs(mountpoint: str) -> List[LogFile]:
    out = []
    for r, _, fs in os.walk(mountpoint):
        for f in fs:
            if f.lower().endswith(LOG_SUFFIXES):
                p = os.path.join(r, f)
                try:
                    st = os.stat(p)
                except OSError:
                    continue
                out.append(LogFile(p, os.path.relpath(p, mountpoint), st.st_size, st.st_mtime))
    return sorted(out, key=lambda x: x.mtime)


def sha256(path: str, chunk: int = 1 << 20) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while True:
            b = f.read(chunk)
            if not b:
                break
            h.update(b)
    return h.hexdigest()


def _key(lf: LogFile) -> str:
    return f"{lf.name}|{lf.size}|{int(lf.mtime)}"


def _load_ledger(path: str = LEDGER) -> Dict[str, dict]:
    try:
        return json.load(open(path))
    except (OSError, ValueError):
        return {}


def _save_ledger(led: Dict[str, dict], path: str = LEDGER) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    json.dump(led, open(tmp, "w"), indent=1)
    os.replace(tmp, path)


def saved_info(lf: LogFile, ledger_path: str = LEDGER) -> Optional[dict]:
    """The recorded copy of this card file, if the copy still exists on the PC."""
    e = _load_ledger(ledger_path).get(_key(lf))
    return e if e and os.path.isfile(e.get("saved", "")) else None


def download(lf: LogFile, dest_dir: Optional[str] = None, ledger_path: str = LEDGER, progress: Optional[Callable[[int, int], None]] = None) -> Tuple[bool, str]:
    """Copy to the PC, verify with SHA-256 against the card file, record in the ledger."""
    dest_dir = dest_dir or os.path.join(DEST_ROOT, time.strftime("%Y%m%d"))
    os.makedirs(dest_dir, exist_ok=True)
    stamp = time.strftime("%H%M%S", time.localtime(lf.mtime))
    dst = os.path.join(dest_dir, f"{os.path.splitext(os.path.basename(lf.name))[0]}_{stamp}{os.path.splitext(lf.name)[1]}")
    base, ext = os.path.splitext(dst); n = 2
    while os.path.exists(dst):                           # the card restarts its numbering after it is cleared: never overwrite an older log with the same name
        dst = f"{base}_{n}{ext}"; n += 1
    done = 0
    try:
        with open(lf.path, "rb") as a, open(dst, "wb") as b:
            while True:
                buf = a.read(1 << 20)
                if not buf:
                    break
                b.write(buf); done += len(buf)
                if progress:
                    progress(done, lf.size)
            b.flush(); os.fsync(b.fileno())
    except OSError as e:
        return False, f"copy failed: {e}"
    h_card, h_pc = sha256(lf.path), sha256(dst)
    if h_card != h_pc or os.path.getsize(dst) != lf.size:
        os.remove(dst)
        return False, "the copy does not match the card file (removed the bad copy); nothing was deleted"
    led = _load_ledger(ledger_path); led[_key(lf)] = {"saved": dst, "sha256": h_pc, "size": lf.size, "when": time.time()}
    _save_ledger(led, ledger_path)
    return True, dst


def can_delete(lf: LogFile, ledger_path: str = LEDGER) -> Tuple[bool, str]:
    if lf.size == 0:
        return True, "empty file"
    e = saved_info(lf, ledger_path)
    if not e:
        return False, "not downloaded yet: download it first"
    try:
        if sha256(lf.path) != e["sha256"] or sha256(e["saved"]) != e["sha256"]:
            return False, "the saved copy no longer matches the card file"
    except OSError as ex:
        return False, f"cannot verify: {ex}"
    return True, "verified copy on the PC"


def delete(lf: LogFile, mountpoint: str, ledger_path: str = LEDGER) -> Tuple[bool, str]:
    real = os.path.realpath(lf.path)
    if not real.startswith(os.path.realpath(mountpoint) + os.sep):
        return False, "refused: the file is not on the mounted card"
    ok, why = can_delete(lf, ledger_path)
    if not ok:
        return False, why
    try:
        os.remove(lf.path)
    except OSError as e:
        return False, f"delete failed: {e}"
    return True, f"deleted from the SD card ({why})"
