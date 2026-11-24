#!/usr/bin/env python3
"""Convert an ESP32-S3 or ESP8266 flash image to an Xtensa ELF for Ghidra.

usage: esp32_image_to_elf.py <chip: esp32s3|esp8266> <in.bin> <out.elf>
"""
import struct
import sys

chip, src, dst = sys.argv[1:4]
data = open(src, "rb").read()


def parse_segments(blob, off, header_len):
    """Return (entry, [(addr, bytes)]) for the image whose header is at off."""
    magic, nseg = blob[off], blob[off + 1]
    assert magic == 0xE9, f"bad magic {magic:#x} at {off:#x}"
    entry = struct.unpack_from("<I", blob, off + 4)[0]
    pos = off + header_len
    segs = []
    for _ in range(nseg):
        addr, size = struct.unpack_from("<II", blob, pos)
        pos += 8
        segs.append((addr, blob[pos : pos + size]))
        pos += size
    return entry, segs


if chip == "esp32s3":
    entry, segs = parse_segments(data, 0, 24)
elif chip == "esp8266":
    entry, segs = parse_segments(data, 0, 8)
    entry2, segs2 = parse_segments(data, 0x1000, 8)
    segs += segs2
    entry = entry2
else:
    raise SystemExit("unknown chip")

segs = [(a, d) for a, d in segs if len(d)]


def flags_for(addr):
    code = (
        0x40000000 <= addr < 0x40400000
        or 0x42000000 <= addr < 0x44000000
        or 0x40100000 <= addr < 0x40300000
    )
    return 5 if code else 6  # R+X or R+W


EHDR, PHDR = 52, 32
nph = len(segs)
off = EHDR + PHDR * nph
phdrs, body = b"", b""
for addr, d in segs:
    pad = (-off) % 4
    body += b"\0" * pad
    off += pad
    phdrs += struct.pack("<8I", 1, off, addr, addr, len(d), len(d), flags_for(addr), 4)
    body += d
    off += len(d)

ehdr = (
    b"\x7fELF" + bytes([1, 1, 1, 0]) + b"\0" * 8
    + struct.pack("<HHIIIIIHHHHHH", 2, 94, 1, entry, EHDR, 0, 0, EHDR, PHDR, nph, 40, 0, 0)
)
open(dst, "wb").write(ehdr + phdrs + body)
print(f"{chip}: entry={entry:#x} segments={len(segs)}")
for a, d in segs:
    print(f"  {a:#010x} len={len(d):#x} {'X' if flags_for(a) == 5 else 'RW'}")
