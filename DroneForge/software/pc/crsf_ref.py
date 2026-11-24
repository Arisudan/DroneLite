#!/usr/bin/env python3
"""Reference CRSF RC-channels encoder matching NimbusOS 2.9.0 nimbus_core
(droneforge::core::crsf::CrsfTransmitEncoder::EncodeControlFrame).
usage: crsf_ref.py <nimbus_core binary>   (verifies the CRC table against the binary)
"""
import struct
import sys

CRSF_MIN, CRSF_MID, CRSF_MAX = 172, 992, 1811
SYNC, FRAME_LEN, TYPE_RC = 0xC8, 0x18, 0x16


def crc8_d5(data, crc=0):
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def stick_to_crsf(v):          # CrsfMapper::StickToCrsf
    v = max(-1.0, min(1.0, v))
    return int(round(max(CRSF_MIN, min(CRSF_MAX, v * 820.0 + 992.0))))


def throttle_to_crsf(t):       # CrsfMapper::Throttle01ToCrsf
    t = max(0.0, min(1.0, t))
    return int(round(max(CRSF_MIN, min(CRSF_MAX, t * 1639.0 + 172.0))))


def encode_rc(ch7):
    """ch7: the 7 command channels (ch0..ch6); ch7..ch15 are fixed at 992."""
    ch = list(ch7) + [CRSF_MID] * 9
    bits = 0
    for i, c in enumerate(ch):
        bits |= (c & 0x7FF) << (11 * i)
    payload = bits.to_bytes(22, "little")
    body = bytes([TYPE_RC]) + payload
    return bytes([SYNC, FRAME_LEN]) + body + bytes([crc8_d5(body)])


if __name__ == "__main__":
    b = open(sys.argv[1], "rb").read()
    # CRC table at Ghidra 0x007f00f0 -> file vaddr 0x6f00f0 (PIE base 0x100000)
    phoff = struct.unpack_from("<Q", b, 0x20)[0]
    esz, n = struct.unpack_from("<HH", b, 0x36)
    off = None
    for i in range(n):
        t, fl, o, va, pa, fsz, msz, al = struct.unpack_from("<IIQQQQQQ", b, phoff + i * esz)
        if t == 1 and va <= 0x7F00F0 - 0x100000 < va + fsz:
            off = o + (0x7F00F0 - 0x100000 - va)
    table = b[off:off + 256]
    ref = bytes(crc8_d5([i]) for i in range(256))
    print("CRC table == CRC-8/DVB-S2 (poly 0xD5):", table == ref)
    f = encode_rc([stick_to_crsf(0), stick_to_crsf(0), throttle_to_crsf(0), stick_to_crsf(0), CRSF_MIN, CRSF_MIN, CRSF_MIN])
    print("idle frame:", f.hex(), "len", len(f), "tx time @115200 8N1: %.2f ms" % (len(f) * 10 / 115200 * 1000))
