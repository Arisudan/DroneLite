"""READ-ONLY FAT32 layout probe of the FC SD card partition (usage: fc_sd_probe_fat_layout_readonly.py sda1, FC in mass-storage mode). Reads through udisks OpenDevice("r"): writes nothing. Used 2026-11-06 to find why the blackbox logs are empty."""
import os, struct, sys, time, dbus
dev = sys.argv[1]                                           # e.g. sda1
bus = dbus.SystemBus()
obj = bus.get_object("org.freedesktop.UDisks2", f"/org/freedesktop/UDisks2/block_devices/{dev}")
fdobj = obj.OpenDevice("r", {}, dbus_interface="org.freedesktop.UDisks2.Block")
f = os.fdopen(fdobj.take(), "rb", buffering=0)
def rd(off, n):
    f.seek(off); return f.read(n)
bs = rd(0, 512)
bps, spc, rsv, nfat = struct.unpack_from("<HBHB", bs, 11); tot16 = struct.unpack_from("<H", bs, 19)[0]; tot32 = struct.unpack_from("<I", bs, 32)[0]
spf = struct.unpack_from("<I", bs, 36)[0]; root = struct.unpack_from("<I", bs, 44)[0]; fsinfo = struct.unpack_from("<H", bs, 48)[0]
total = tot32 or tot16; data_start = rsv + nfat * spf; clusters = (total - data_start) // spc
print(f"bytes/sector {bps}  sectors/cluster {spc}  cluster {bps*spc} B  reserved {rsv}  FATs {nfat}  sectors/FAT {spf}  root cluster {root}  total sectors {total}  data clusters {clusters}  OEM {bs[3:11]!r}")
fat = rd(rsv * bps, spf * bps); ent = lambda c: struct.unpack_from("<I", fat, 4 * c)[0] & 0x0FFFFFFF
cl_off = lambda c: (data_start + (c - 2) * spc) * bps
free = [c for c in range(2, clusters + 2) if ent(c) == 0]
print(f"free clusters {len(free)} ({len(free)*bps*spc/1e6:.1f} MB)  used {clusters-len(free)}")
runs = []; 
for c in free:
    if runs and c == runs[-1][0] + runs[-1][1]: runs[-1][1] += 1
    else: runs.append([c, 1])
runs.sort(key=lambda r: -r[1]); print("largest free runs (start cluster, length, MB):", [(a, n, round(n*bps*spc/1e6, 1)) for a, n in runs[:6]], " total runs", len(runs))
def chain(c, limit=100000):
    out = []
    while 2 <= c < 0x0FFFFFF8 and len(out) < limit:
        out.append(c); c = ent(c)
    return out
def entries(dirc):
    for c in chain(dirc):
        data = rd(cl_off(c), bps * spc)
        for i in range(0, len(data), 32):
            e = data[i:i + 32]
            if e[0] == 0: return
            if e[0] == 0xE5 or e[11] == 0x0F: continue
            name = e[:8].decode("latin1").strip() + ("." + e[8:11].decode("latin1").strip() if e[8:11].strip() else "")
            hi, lo = struct.unpack_from("<HH", e, 20)[0], struct.unpack_from("<H", e, 26)[0]; size = struct.unpack_from("<I", e, 28)[0]
            yield name, e[11], (hi << 16) | lo, size
rootents = list(entries(root)); print("root entries:"); 
for n, a, c, s in rootents: print(f"   {n:14s} attr {a:#04x} first cluster {c:8d} size {s}")
for n, a, c, s in rootents:
    if n.upper().startswith("FREESPAC"):
        ch = chain(c, 3); print(f"FREESPAC.E: first cluster {c}, dir size {s} B, FAT[first] = {ent(c):#x}, run of clusters starting at it that are NOT free: ", end="")
        k = 0
        while c + k <= clusters + 1 and ent(c + k) != 0 and k < 100000: k += 1
        print(k, "clusters")
        print("   the 8 clusters after its first one, FAT values:", [hex(ent(c + j)) for j in range(1, 9)])
        nxt = c + max(k, 1); print("   first free cluster after it:", next((x for x in free if x > c), None))
logs = [e for e in rootents if e[0].upper() == "LOGS"]
if logs:
    print("LOGS directory:")
    for n, a, c, s in entries(logs[0][2]):
        head = rd(cl_off(c), 48) if c >= 2 else b""
        print(f"   {n:14s} first cluster {c:8d} size {s:8d}  FAT={ent(c) if c>=2 else 0:#x}  first bytes: {head[:40]!r}")
