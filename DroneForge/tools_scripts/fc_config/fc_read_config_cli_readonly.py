#!/usr/bin/env python3
"""Read-only Betaflight CLI reader. Whitelisted commands only; never saves or sets.
usage: fc_read_config_cli_readonly.py /dev/ttyACMx out.txt
Leaves CLI with 'exit' (FC reboots WITHOUT saving; no change to stored config).
"""
import os
import select
import sys
import termios
import time

ALLOWED = ("version", "feature", "serial", "status", "diff all", "get ", "resource", "mixer", "dshotprog")
ALLOWED_EXACT = ("version", "feature", "serial", "status", "diff all", "resource", "mixer")
COMMANDS = ["version", "feature", "serial", "status", "get opticalflow", "get rangefinder",
            "get serialrx", "get motor_poles", "get motor_pwm", "get dshot", "get gyro_1_sensor",
            "get small_angle", "get blackbox_device", "get vbat_scale", "get current_meter",
            "diff all"]


def safe(cmd):
    if cmd in ALLOWED_EXACT:
        return True
    return cmd.startswith("get ") and ";" not in cmd and "=" not in cmd


def port(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
    a = termios.tcgetattr(fd)
    a[0] = a[1] = a[3] = 0
    a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    a[4] = a[5] = termios.B115200
    a[6][termios.VMIN], a[6][termios.VTIME] = 0, 1
    termios.tcsetattr(fd, termios.TCSANOW, a)
    return fd


def drain(fd, quiet=0.8, limit=40.0):
    buf, last, end = b"", time.time(), time.time() + limit
    while time.time() < end:
        r, _, _ = select.select([fd], [], [], 0.1)
        if r:
            d = os.read(fd, 4096)
            if d:
                buf += d
                last = time.time()
        elif time.time() - last > quiet:
            break
    return buf.decode(errors="replace")


def main(path, out):
    fd = port(path)
    os.write(fd, b"#")
    log = [drain(fd)]
    for c in COMMANDS:
        assert safe(c), c
        os.write(fd, c.encode() + b"\r\n")
        log.append(f"\n>>> {c}\n" + drain(fd, quiet=1.5 if c == "diff all" else 0.6))
    os.write(fd, b"exit\r\n")
    drain(fd, quiet=0.5, limit=3.0)
    os.close(fd)
    open(out, "w").write("".join(log))


main(sys.argv[1], sys.argv[2])
