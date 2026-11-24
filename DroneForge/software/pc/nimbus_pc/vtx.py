"""VTX / VRX helpers, Qt-free: frame builders and parsers on top of commands.py / crsf.py.

VTX is controlled through the FC with MSP-over-CRSF (MSP_VTX_CONFIG 0x58 read, MSP_SET_VTX_CONFIG 0x59 write).
The VRX scan comes from the TX as custom frame 0xE1 (48 RSSI bytes; layout order is INFERRED as band-major A,B,E,F,R,L x 8).
Power is the 1-based index into the drone's VTX power table; the mW values below are the common default table (UNVERIFIED on our VTX).
"""
from __future__ import annotations

import struct
from typing import List, Optional, Tuple

from . import commands
from .crsf import ADDR_FC, ADDR_RADIO, SYNC_FC, crc8

BANDS = commands.VRX_BANDS                      # "ABEFRL"
BAND_NAMES = {"A": "A (Boscam A)", "B": "B (Boscam B)", "E": "E (Boscam E)", "F": "F (Fatshark)", "R": "R (Raceband)", "L": "L (Low race)"}
# Standard 5.8 GHz table, MHz, channel 1..8 per band (Betaflight vtx_common band order).
FREQ_MHZ = {
    "A": (5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725),
    "B": (5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866),
    "E": (5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945),
    "F": (5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880),
    "R": (5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917),
    "L": (5362, 5399, 5436, 5473, 5510, 5547, 5584, 5621),
}
DEFAULT_POWERS_MW = (25, 100, 200, 400, 600)    # power level index = position + 1


def freq_mhz(band: str, channel: int) -> int:
    return FREQ_MHZ[band.upper()][channel - 1]


def request_config_frame() -> bytes:
    """MSP_VTX_CONFIG request (13 bytes)."""
    return commands.encode_msp_vtx_config_request()


def set_frames(band: str, channel: int, power_level: int, pit_mode: bool = False) -> List[bytes]:
    """MSP_SET_VTX_CONFIG frames. pit_mode False gives exactly commands.encode_msp_set_vtx_config_power()."""
    frames = commands.encode_msp_set_vtx_config_power(band, channel, power_level)
    if not pit_mode:
        return frames
    body = bytes([0x00]) + struct.pack("<HH", commands.MSP_SET_VTX_CONFIG, 4) + struct.pack(
        "<HBB", commands.band_code(band) + channel - 1, power_level, 1)
    x = 0
    for b in body:
        x ^= b
    body += bytes([x])
    out = []
    for seq, i in enumerate(range(0, len(body), 7)):
        status = 0x40 | (0x10 if seq == 0 else 0) | (seq & 0xF)
        inner = bytes([commands.TYPE_MSP_WRITE, ADDR_FC, ADDR_RADIO, status]) + body[i:i + 7]
        out.append(bytes([SYNC_FC, len(inner) + 1]) + inner + bytes([crc8(inner)]))
    return out


def scan_frame() -> bytes:
    return commands.encode_vrx_scan()


def connect_frame(band: str, channel: int) -> bytes:
    return commands.encode_vrx_connect(band, channel)


def parse_config(payload: bytes) -> Optional[dict]:
    """MSP_VTX_CONFIG reply -> dict with band_letter, channel, power_level, power_levels (count) added."""
    d = commands.decode_vtx_config(payload)
    if d is None:
        return None
    d["band_letter"] = BANDS[d["band"] - 1] if 1 <= d["band"] <= len(BANDS) else None
    return d


def power_choices(table_levels: Optional[int] = None) -> List[Tuple[int, str]]:
    """[(level index, label)]. With a known table size of n levels: n entries (mW only when n == 5, else 'level k')."""
    n = table_levels if table_levels else len(DEFAULT_POWERS_MW)
    if n == len(DEFAULT_POWERS_MW):
        return [(i + 1, f"{mw} mW") for i, mw in enumerate(DEFAULT_POWERS_MW)]
    return [(i + 1, f"level {i + 1}") for i in range(max(1, min(n, 255)))]


def parse_scan(p: bytes) -> Optional[List[Tuple[str, int, int, int]]]:
    """0xE1 payload (48 B) -> [(band letter, channel 1..8, MHz, rssi)] in band-major order (inferred layout)."""
    if p is None or len(p) != 48:
        return None
    return [(BANDS[i // 8], i % 8 + 1, FREQ_MHZ[BANDS[i // 8]][i % 8], p[i]) for i in range(48)]


def best_channel(p: bytes) -> Optional[Tuple[str, int, int, int]]:
    rows = parse_scan(p)
    if not rows or max(r[3] for r in rows) == 0:
        return None
    return max(rows, key=lambda r: r[3])
