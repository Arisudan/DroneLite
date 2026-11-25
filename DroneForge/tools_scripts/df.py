#!/usr/bin/env python3
"""DroneForge helper: df.py test [gcs|-k name] | logs [--free] | flash <fw.bin> [--debug-hold] | fcconfig | debughold on|off | summary <LOG.BFL>   (short output; never arms anything)"""
import glob, os, re, select, subprocess, sys, termios, time
R = os.path.expanduser("~/Music/DroneForge"); PC = R + "/software/pc"; FC = R + "/tools_scripts/fc_config"
sys.path.insert(0, PC)
from nimbus_gcs import fc_logs as FL, bfl


def wait_age(sec):
    while True:
        p = FL.fc_serial_port(); a = FL.fc_usb_age_s(p) if p else None
        if a is not None and a >= sec: return p
        time.sleep(2)


def cli_set(port, lines):
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY); a = termios.tcgetattr(fd); a[0] = a[1] = 0; a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL; a[3] = 0; termios.tcsetattr(fd, termios.TCSANOW, a)
    os.write(fd, b"#\r\n"); time.sleep(0.8)
    for l in lines: os.write(fd, (l + "\r\n").encode()); time.sleep(0.4)
    os.write(fd, b"save\r\n"); time.sleep(1.5)


def cfg(p):
    return {l.strip() for l in open(p) if re.match(r"^(set|aux|serial|feature|map|mixer|resource)", l)}


def dump(port, out):
    subprocess.run([sys.executable, FC + "/fc_read_config_cli_readonly.py", port, out], capture_output=True, timeout=120); time.sleep(6); wait_age(20)


NEWCFG = ["set throttle_boost = 0", "set anti_gravity_gain = 55", "set small_angle = 180", "set debug_mode = NONE", "set blackbox_sample_rate = 1/2"]


def cmd_fcconfig(a):
    """Apply the 2026-11-06 FC settings (throttle_boost 0, anti-gravity 55, small_angle 180, debug NONE, blackbox 1/2) to a live FC and verify. FC on USB, disarmed, powered >= 60 s. Backup is written first."""
    port = wait_age(60); d, why = FL.fc_is_disarmed(port); assert d is True, f"refused: {why}"
    bak = f"{R}/firmware/fc_matek_h743/config_backups/{time.strftime('%Y-%m-%d')}_before-fcconfig_{time.strftime('%Y%m%d_%H%M')}.txt"; dump(port, bak)
    cli_set(wait_age(20), NEWCFG); port = wait_age(25); after = bak.replace("before", "after"); dump(port, after); c = cfg(after)
    for l in NEWCFG:
        k, v = l[4:].split(" = "); dflt = (k == "debug_mode" and v == "NONE")
        ok = l in c or (dflt and not any(x.startswith("set debug_mode") for x in c)); print(("ok   " if ok else "FAIL ") + l)
    print("backup:", bak)


def cmd_debughold(a):
    """debughold on|off: set ONLY debug_mode (DF_HOLD = the FC log carries the EKF body velocity and the VO aid, firmware v4.7.4R and later; NONE = off, the flight default) and verify.
    FC on USB, disarmed, powered >= 60 s. A backup is written first. Changes one FC setting: run it only when you want it."""
    assert a[:1] in (["on"], ["off"]), "usage: df.py debughold on|off"
    value = "DF_HOLD" if a[0] == "on" else "NONE"
    port = wait_age(60); d, why = FL.fc_is_disarmed(port); assert d is True, f"refused: {why}"
    bak = f"{R}/firmware/fc_matek_h743/config_backups/{time.strftime('%Y-%m-%d')}_before-debughold_{time.strftime('%Y%m%d_%H%M')}.txt"; dump(port, bak)
    cli_set(wait_age(20), [f"set debug_mode = {value}"]); port = wait_age(25); after = bak.replace("before", "after"); dump(port, after); c = cfg(after)
    ok = f"set debug_mode = {value}" in c or (value == "NONE" and not any(x.startswith("set debug_mode") for x in c))
    print(("ok   " if ok else "FAIL ") + f"set debug_mode = {value}" + ("" if ok else "  (is this firmware v4.7.4R or later? older builds do not know DF_HOLD)")); print("backup:", bak)


def cmd_test(a):
    args = ["-k", a[1]] if a[:1] == ["-k"] else []
    sel = ["nimbus_gcs.tests.test_gcs"] if a[:1] == ["gcs"] else ["discover", "-s", ".", "-p", "test*.py"]
    r = subprocess.run([sys.executable, "-m", "unittest", *sel, *args], cwd=PC, capture_output=True, text=True)
    out = r.stderr + r.stdout; print("\n".join(l for l in out.splitlines() if re.match(r"^(FAIL|ERROR|Ran|OK|FAILED)", l)) or out[-600:])


def cmd_summary(a):
    lg = bfl.Log(a[0]); lg.decode(); print(bfl.summarize(lg))


def cmd_logs(a):
    port = wait_age(60); d, why = FL.fc_is_disarmed(port)
    if d is not True: sys.exit(f"refused: {why}")
    before = [p.get("path") for p in FL.usb_partitions()]; ok, msg = FL.enter_mass_storage(port)
    if not ok: sys.exit(msg)
    card = None
    for _ in range(30):
        time.sleep(1); card = FL.find_card(before)
        if card: break
    if not card: sys.exit("card did not appear: replug the FC USB")
    time.sleep(5)
    for _ in range(8):
        ok, mp = FL.mount(card)
        if ok: break
        time.sleep(2)
    saved = {os.path.basename(p).split("_")[0] for p in glob.glob(FL.DEST_ROOT + "/*/*.BFL")}
    for l in FL.list_logs(mp):
        n = os.path.splitext(os.path.basename(l.name))[0]
        if l.size and not FL.saved_info(l):                 # the ledger (name, size, time) decides, not the bare log number: the card restarts its numbering
            ok, dst = FL.download(l); print(("saved " if ok else "FAILED ") + dst)
            if ok: cmd_summary([dst]); print()
    if "--free" in a:                                   # opt-in: delete the logs that are saved and verified on the PC, and the exhausted FREESPAC.E reserve; the FC recreates the reserve at its next boot
        for l in FL.list_logs(mp):
            ok, why = FL.delete(l, mp); print(("freed   " if ok else "KEPT    ") + os.path.basename(l.name), "-", why)
        fs = os.path.join(mp, "FREESPAC.E")
        if os.path.isfile(fs) and os.path.getsize(fs) <= 4096: os.remove(fs); print("freed   FREESPAC.E (the FC rebuilds its log reserve at the next boot)")
    print("free log space:", [os.path.getsize(os.path.join(mp, f)) for f in os.listdir(mp) if f.upper().startswith("FREESPAC")]); FL.eject(card); print("ejected: replug the FC USB")


def cmd_flash(a):
    fw = os.path.abspath(a[0]); assert os.path.isfile(fw), "firmware file missing"
    port = wait_age(75); d, why = FL.fc_is_disarmed(port); assert d is True, f"refused: {why}"
    bak = f"{R}/firmware/fc_matek_h743/config_backups/{time.strftime('%Y-%m-%d')}_before-flash_{time.strftime('%Y%m%d_%H%M')}.txt"; dump(port, bak); port = FL.fc_serial_port()
    assert FL.fc_is_disarmed(port)[0] is True; cli_set_bl(port)
    time.sleep(3); r = subprocess.run(["dfu-util", "-S", "200364500000", "-a", "0", "-s", "0x08000000:mass-erase:force:leave", "-D", fw], capture_output=True, text=True)
    print("dfu:", "downloaded" if "File downloaded successfully" in r.stdout else "FAILED\n" + r.stdout[-400:])
    time.sleep(5); port = wait_age(20); subprocess.run([sys.executable, FC + "/fc_restore_config_after_flash.py", port], capture_output=True, timeout=180); time.sleep(6); port = wait_age(20)
    after = bak.replace("before", "after"); dump(port, after); b, c = cfg(bak), cfg(after)
    strip = lambda cfgset: {l for l in cfgset if not l.startswith("set debug_mode")}      # debug_mode is NONE on purpose (flight: the debug output only costs load); the pre-flash OPTICALFLOW must not come back
    b, c = strip(b), strip(c)
    fix = sorted(l for l in b - c if l.startswith("set "))                    # values the restore script overwrote (e.g. acc_calibration): put the pre-flash ones back
    if fix: cli_set(port, fix); wait_age(25); print("re-applied:", fix); dump(wait_age(20), after); c = strip(cfg(after))
    print("config identical to pre-flash backup" if b == c else f"CONFIG DIFFERS: {sorted(b ^ c)[:6]}")
    if "--debug-hold" in a[1:]:                                              # explicit, only when asked: log the sideways hold (firmware v4.7.4R+)
        cli_set(wait_age(20), ["set debug_mode = DF_HOLD"]); wait_age(25); print("debug_mode = DF_HOLD set (the FC log now carries the EKF velocity and the VO aid)")
    p = subprocess.run([sys.executable, FC + "/fc_probe_msp_readonly.py", wait_age(20)], capture_output=True, text=True).stdout
    print(*[l for l in p.splitlines() if "Build date" in l])


def cli_set_bl(port):
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY); a = termios.tcgetattr(fd); a[0] = a[1] = 0; a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL; a[3] = 0; termios.tcsetattr(fd, termios.TCSANOW, a)
    os.write(fd, b"#\r\n"); time.sleep(0.6); os.write(fd, b"bl\r\n"); time.sleep(1.0)


if __name__ == "__main__":
    c = sys.argv[1] if len(sys.argv) > 1 else ""
    {"fcconfig": cmd_fcconfig, "debughold": cmd_debughold, "test": cmd_test, "logs": cmd_logs, "flash": cmd_flash, "summary": cmd_summary}.get(c, lambda a: print(__doc__))(sys.argv[2:])
