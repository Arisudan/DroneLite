#!/usr/bin/env python3
"""Read-only Betaflight MSP probe. Sends only GET-style MSP requests.
usage: fc_probe_msp_readonly.py /dev/ttyACMx
"""
import os
import select
import struct
import sys
import termios
import time

# Only these read commands are ever sent.
READ_CMDS = {
    "API_VERSION": 1, "FC_VARIANT": 2, "FC_VERSION": 3, "BOARD_INFO": 4, "BUILD_INFO": 5,
    "NAME": 10, "FEATURE_CONFIG": 36, "RX_CONFIG": 44, "RX_MAP": 64, "STATUS": 101,
    "MOTOR_CONFIG": 131, "SENSOR_CONFIG": 96, "ADVANCED_CONFIG": 90, "UID": 160,
    "RC": 105, "ARMING_CONFIG": 61, "FAILSAFE_CONFIG": 75, "BATTERY_CONFIG": 32,
}


def open_port(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
    a = termios.tcgetattr(fd)
    a[0] = a[1] = 0
    a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    a[3] = 0
    a[4] = a[5] = termios.B115200
    a[6][termios.VMIN], a[6][termios.VTIME] = 0, 1
    termios.tcsetattr(fd, termios.TCSANOW, a)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def request(fd, cmd, timeout=1.0):
    size = 0
    frame = b"$M<" + bytes([size, cmd, size ^ cmd])
    os.write(fd, frame)
    buf, end = b"", time.time() + timeout
    while time.time() < end:
        r, _, _ = select.select([fd], [], [], 0.1)
        if r:
            buf += os.read(fd, 512)
        i = buf.find(b"$M>")
        if i >= 0 and len(buf) >= i + 5:
            n = buf[i + 3]
            if len(buf) >= i + 6 + n and buf[i + 4] == cmd:
                return buf[i + 5 : i + 5 + n]
        j = buf.find(b"$M!")
        if j >= 0:
            return None
    return None


def main(path):
    fd = open_port(path)
    out = {name: request(fd, c) for name, c in READ_CMDS.items()}
    g = lambda k: out.get(k)
    v = g("API_VERSION")
    if v:
        print(f"MSP protocol {v[0]}, API {v[1]}.{v[2]}")
    if g("FC_VARIANT"):
        print("FC variant:", g("FC_VARIANT").decode())
    v = g("FC_VERSION")
    if v:
        print(f"Betaflight version: {v[0]}.{v[1]}.{v[2]}")
    b = g("BUILD_INFO")
    if b:
        print("Build date/time/rev:", b[:11].decode(), b[11:19].decode(), b[19:26].decode())
    b = g("BOARD_INFO")
    if b:
        ident = b[:4].decode()
        hw = b[4]
        extra = b[9:]
        print("Board id:", ident, "| hw rev:", struct.unpack_from("<H", b, 4)[0])
        print("Board raw tail:", extra.hex())
    if g("NAME") is not None:
        print("Craft name:", g("NAME").decode(errors="replace") or "(empty)")
    u = g("UID")
    if u:
        print("UID:", u.hex())
    f = g("FEATURE_CONFIG")
    if f:
        mask = struct.unpack("<I", f[:4])[0]
        names = {0: "RX_PPM", 3: "INFLIGHT_ACC_CAL", 4: "RX_SERIAL", 5: "MOTOR_STOP", 6: "SERVO_TILT", 7: "SOFTSERIAL",
                 8: "GPS", 10: "RANGEFINDER", 11: "TELEMETRY", 13: "3D", 14: "RX_PARALLEL_PWM", 15: "RX_MSP",
                 16: "RSSI_ADC", 17: "LED_STRIP", 18: "DISPLAY", 20: "OSD", 22: "CHANNEL_FORWARDING",
                 23: "TRANSPONDER", 24: "AIRMODE", 27: "RX_SPI", 28: "SOFTSPI", 29: "ESC_SENSOR", 30: "ANTI_GRAVITY",
                 31: "DYNAMIC_FILTER"}
        on = [n for bit, n in names.items() if mask & (1 << bit)]
        print(f"Features mask {mask:#x}:", ", ".join(on))
    r = g("RX_CONFIG")
    if r:
        prov = {0: "PPM", 1: "SERIAL", 2: "MSP", 3: "SPI"}
        print("RX: serialrx_provider id =", r[0], "(0 SPEKTRUM1024,1 SPEKTRUM2048,2 SBUS,3 SUMD,4 SUMH,5 XB-B,6 XB-B-RJ01,7 IBUS,8 JETIEXBUS,9 CRSF,10 SRXL,11 CUSTOM,12 FPORT,13 SRXL2,14 GHST)")
    m = g("MOTOR_CONFIG")
    if m:
        print("Motor config: minthrottle, maxthrottle, mincommand =", struct.unpack_from("<HHH", m, 0), "| motor count/poles:", m[6:])
    s = g("STATUS")
    if s:
        cyc, i2c, sens, flags = struct.unpack_from("<HHHI", s, 2) if len(s) >= 11 else (0, 0, 0, 0)
        sens_names = {0: "ACC", 1: "BARO", 2: "MAG", 3: "GPS", 4: "RANGEFINDER", 5: "GYRO"}
        print("Sensors present:", ", ".join(n for b, n in sens_names.items() if sens & (1 << b)) or "none")
        print("Cycle time us:", struct.unpack_from("<H", s, 0)[0], "| i2c errors:", struct.unpack_from("<H", s, 2)[0])
    os.close(fd)


main(sys.argv[1])
