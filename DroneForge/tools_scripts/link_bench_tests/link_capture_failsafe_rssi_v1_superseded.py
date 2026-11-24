#!/usr/bin/env python3
"""Failsafe capture: FC channels (MSP_RC) + RSSI (MSP_ANALOG) read-only, with bell cues, saved to CSV.
usage: link_capture_failsafe_rssi_v1_superseded.py <fc port> <joystick dev|-> <out.csv>
"""
import os, select, struct, subprocess, sys, termios, time
fc_port, js_dev, out = sys.argv[1:4]
ffd = os.open(fc_port, os.O_RDWR | os.O_NOCTTY)
a = termios.tcgetattr(ffd); a[0]=a[1]=a[3]=0; a[2]=termios.CS8|termios.CREAD|termios.CLOCAL; a[4]=a[5]=termios.B115200
a[6][termios.VMIN], a[6][termios.VTIME] = 0, 1
termios.tcsetattr(ffd, termios.TCSANOW, a); termios.tcflush(ffd, termios.TCIOFLUSH)
def msp(cmd):
    assert cmd in (105, 110)
    os.write(ffd, b"$M<" + bytes([0, cmd, cmd])); buf, end = b"", time.time() + 0.15
    while time.time() < end:
        if select.select([ffd], [], [], 0.02)[0]: buf += os.read(ffd, 512)
        i = buf.find(b"$M>")
        if i >= 0 and len(buf) >= i + 6 and len(buf) >= i + 6 + buf[i + 3]: return buf[i + 5:i + 5 + buf[i + 3]]
    return None
jfd = None
if js_dev != "-":
    try: jfd = os.open(js_dev, os.O_RDONLY | os.O_NONBLOCK)
    except OSError: jfd = None
def bell(txt):
    subprocess.Popen(["pw-play", "/usr/share/sounds/freedesktop/stereo/bell.oga"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.Popen(["spd-say", "-w", txt], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.Popen(["notify-send", "-u", "critical", "-t", "7000", "Failsafe test", txt], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
f = open(out, "w"); f.write("t,ch1,ch2,ch3,ch4,ch5,ch6,rssi,rc_ok,joy_alive,event\n")
t0 = time.time(); rssi = ""; last_an = 0; joy_alive = 1; events = {8: "BELL1_radio_off", 24: "BELL2_radio_on"}; fired = set()
while time.time() - t0 < 45:
    t = time.time() - t0; ev = ""
    for k, name in events.items():
        if t >= k and k not in fired:
            fired.add(k); ev = name
            bell("Switch the Boxer off now." if k == 8 else "Switch the Boxer back on now.")
    if jfd is not None:
        try:
            while select.select([jfd], [], [], 0)[0]:
                if len(os.read(jfd, 8)) < 8: break
        except OSError: joy_alive = 0
    rc = msp(105)
    if time.time() - last_an > 0.3:
        an = msp(110)
        if an and len(an) >= 5: rssi = struct.unpack_from("<H", an, 3)[0]
        last_an = time.time()
    if rc and len(rc) >= 12:
        v = struct.unpack_from("<6H", rc, 0)
        f.write(f"{t:.2f}," + ",".join(map(str, v)) + f",{rssi},1,{joy_alive},{ev}\n")
    else:
        f.write(f"{t:.2f},,,,,,,{rssi},0,{joy_alive},{ev}\n")
    time.sleep(0.03)
f.close(); print("done")
