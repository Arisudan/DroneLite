#!/usr/bin/env python3
"""Capture raw MTF-02P frames while the FC is already in serialpassthrough. Read-only on the port.
usage: mtf02p_capture_frames_in_passthrough.py <port> <label> <ruler_cm> <out_prefix>
"""
import os, select, statistics as st, struct, subprocess, sys, termios, time
port, label, ruler_cm, prefix = sys.argv[1], sys.argv[2], float(sys.argv[3]), sys.argv[4]
fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
a = termios.tcgetattr(fd); a[0]=a[1]=a[3]=0; a[2]=termios.CS8|termios.CREAD|termios.CLOCAL; a[4]=a[5]=termios.B115200
a[6][termios.VMIN], a[6][termios.VTIME] = 0, 1
termios.tcsetattr(fd, termios.TCSANOW, a); termios.tcflush(fd, termios.TCIFLUSH)
subprocess.Popen(["pw-play", "/usr/share/sounds/freedesktop/stereo/bell.oga"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
subprocess.Popen(["spd-say", "-w", f"Capturing {label}. Hold steady."], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(1.5)
raw, t0 = [], time.time()
while time.time() - t0 < 6:
    if select.select([fd], [], [], 0.05)[0]:
        d = os.read(fd, 4096)
        if d: raw.append((time.time(), d))
os.close(fd)
open(f"{prefix}_{label}.bin", "wb").write(b"".join(d for _, d in raw))
frames, carry = [], b""
for t, d in raw:
    buf = carry + d; j = 0
    while True:
        j = buf.find(b"\xef", j)
        if j < 0 or j + 7 > len(buf): carry = buf[j:] if j >= 0 else b""; break
        ln = buf[j+5]; end = j + 6 + ln + 1
        if ln > 64: j += 1; continue
        if end > len(buf): carry = buf[j:]; break
        if (sum(buf[j:end-1]) & 0xFF) == buf[end-1] and buf[j+3] == 0x51 and ln == 20:
            frames.append((t, buf[j+6:j+26])); j = end
        else: j += 1
if not frames:
    print("NO FRAMES: FC is probably no longer in passthrough (power-cycled?)"); sys.exit(1)
tend = frames[-1][0]
last = [p for t, p in frames if t >= tend - 4.0]
dist = [struct.unpack_from("<I", p, 4)[0] for p in last]
allv = [struct.unpack_from("<I", p, 4)[0] for _, p in frames]
fq = int(st.median(p[16] for p in last)); tof = sorted({p[10] for p in last}); stren = int(st.median(p[8] for p in last))
med = int(st.median(dist)); exp = ruler_cm * 10
print(f"{label}: frames={len(frames)} (last-4s n={len(last)}) ruler={exp:.0f} mm")
print(f"  distance mm  min={min(dist)} median={med} max={max(dist)} sd={st.pstdev(dist):.1f}")
print(f"  whole capture min={min(allv)} max={max(allv)} | strength={stren} tof_status={tof} flow_quality={fq}")
print(f"  error vs ruler: {med-exp:+.0f} mm ({100*(med-exp)/exp:+.0f}%)")
