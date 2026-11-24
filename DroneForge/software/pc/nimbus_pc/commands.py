"""CRSF command and MSP-over-CRSF frames used by NimbusOS 2.9.0 (read from the nimbus_core binary).

Confirmed from disassembly/decompilation: byte layouts, command/sub-command IDs, payloads, validation.
NOT confirmed: whether stock ExpressLRS reacts to the 0x80 (Nimbus-specific) commands; they are likely handled
only by Nimbus's custom firmware. The bind command (0x10/0x01) is an ELRS-standard command.
"""
from __future__ import annotations

import hashlib
import struct
from typing import Optional

from .crsf import ADDR_FC, ADDR_RADIO, ADDR_TX, SYNC_FC, SYNC_TX, Frame, crc8

TYPE_COMMAND = 0x32
TYPE_MSP_REQ = 0x7A
TYPE_MSP_RESP = 0x7B
TYPE_MSP_WRITE = 0x7C

CMD_RX = 0x10               # command group addressed to the receiver
SUB_RX_BIND = 0x01
CMD_NIMBUS = 0x80           # Nimbus-specific command group
SUB_VRX_SCAN = 0x40
SUB_VRX_CONNECT = 0x41
SUB_SET_BIND_UID = 0x42

MSP_BOARD_INFO = 0x04
MSP_VTX_CONFIG = 0x58
MSP_SET_VTX_CONFIG = 0x59
MSP_SET_MOTOR = 214

VRX_BANDS = "ABEFRL"        # band letters, code = index * 8


def encode_command_frame(command: int, sub_command: int, payload: bytes = b"") -> bytes:
    """EE <len=n+7> 32 EE EA <cmd> <sub> <payload> 00 <CRC>.

    The byte before the CRC is the (unused) command CRC slot, always 0 in NimbusOS; the frame CRC covers it.
    """
    body = bytes([TYPE_COMMAND, ADDR_TX, ADDR_RADIO, command, sub_command]) + bytes(payload) + b"\x00"
    if len(body) + 1 > 62:
        raise ValueError("command payload too long")
    return bytes([SYNC_TX, len(body) + 1]) + body + bytes([crc8(body)])


def encode_receiver_bind() -> bytes:
    """EncodeReceiverBindFrame: command 0x10, sub-command 0x01, no payload."""
    return encode_command_frame(CMD_RX, SUB_RX_BIND)


def encode_set_bind_uid(uid: bytes) -> bytes:
    """EncodeSetBindUidFrame: command 0x80, sub-command 0x42, 6-byte UID."""
    if len(uid) != 6:
        raise ValueError("bind UID must be 6 bytes")
    return encode_command_frame(CMD_NIMBUS, SUB_SET_BIND_UID, uid)


def uid_from_phrase(phrase: str) -> bytes:
    """ELRS bind-phrase to UID: first 6 bytes of MD5('-DMY_BINDING_PHRASE="<phrase>"'). From memory of ELRS; verify."""
    return hashlib.md5(f'-DMY_BINDING_PHRASE="{phrase}"'.encode()).digest()[:6]


def band_code(letter: str) -> int:
    i = VRX_BANDS.find(letter.upper())
    if i < 0:
        raise ValueError(f"band must be one of {VRX_BANDS}")
    return i * 8


def encode_vrx_scan() -> bytes:
    """EncodeVideoReceiverScanFrame: command 0x80, sub-command 0x40, payload [1]."""
    return encode_command_frame(CMD_NIMBUS, SUB_VRX_SCAN, bytes([1]))


def encode_vrx_connect(band: str, channel: int) -> bytes:
    """EncodeVideoReceiverConnectFrame: command 0x80, sub-command 0x41, payload [band code, channel 1..8]."""
    if not 1 <= channel <= 8:
        raise ValueError("channel must be 1..8")
    return encode_command_frame(CMD_NIMBUS, SUB_VRX_CONNECT, bytes([band_code(band), channel]))


def encode_msp_request(cmd: int) -> bytes:
    """EncodeMspRequestFrame (13 bytes): C8 0B 7A C8 EA 50 00 <cmd> 00 00 00 <xor> <crc>.

    Status byte 0x50 = MSPv2 + start flag + sequence 0. The inner checksum NimbusOS writes is the XOR of the five
    MSPv2 header bytes (not a CRC), then the usual CRSF CRC. The command's high byte is always 0 in Nimbus.
    """
    if not 0 <= cmd <= 0xFF:
        raise ValueError("Nimbus only sends 8-bit MSP commands")
    hdr = bytes([0x00, cmd, 0x00, 0x00, 0x00])
    xor = 0
    for b in hdr:
        xor ^= b
    body = bytes([TYPE_MSP_REQ, ADDR_FC, ADDR_RADIO, 0x50]) + hdr + bytes([xor])
    return bytes([SYNC_FC, len(body) + 1]) + body + bytes([crc8(body)])


def encode_msp_board_info_request() -> bytes:
    return encode_msp_request(MSP_BOARD_INFO)


def encode_msp_vtx_config_request() -> bytes:
    return encode_msp_request(MSP_VTX_CONFIG)


def encode_msp_set_vtx_config_power(band: str, channel: int, power_level: int) -> list:
    """EncodeMspSetVtxConfigPowerFrames: MSP_SET_VTX_CONFIG (0x59) written to the FC as chunked 0x7C frames.

    MSPv2 body (10 bytes): flag 00, cmd 59 00, size 04 00, payload = u16 LE (band_code + channel - 1), u8 power level,
    u8 pit mode (always 0), then an XOR of the nine bytes before it. The body is cut into chunks of at most 7 bytes; each
    chunk becomes `C8 <n+5> 7C C8 EA <status> <chunk> <crc>` with status = 0x40 (MSPv2) | 0x10 (first chunk) | seq (mod 16).
    Validation as in Nimbus: band code a multiple of 8 below 48 (A B E F R L), channel 1..8, power level non-zero.
    The power value is the 1-based power-level index of the drone's VTX table, not milliwatts.
    Betaflight decodes the u16 as band = v/8 + 1, channel = v%8 + 1 when v <= 63 (checked in its MSP source).
    """
    if not 1 <= channel <= 8:
        raise ValueError("channel must be 1..8")
    if not 1 <= power_level <= 255:
        raise ValueError("power level must be 1..255 (0 is rejected by Nimbus)")
    freq_or_bandchan = band_code(band) + channel - 1
    body = bytes([0x00]) + struct.pack("<HH", MSP_SET_VTX_CONFIG, 4) + struct.pack("<HBB", freq_or_bandchan, power_level, 0)
    xor = 0
    for b in body:
        xor ^= b
    body += bytes([xor])
    frames, seq = [], 0
    for i in range(0, len(body), 7):
        chunk = body[i:i + 7]
        status = 0x40 | (0x10 if seq == 0 else 0) | (seq & 0xF)
        inner = bytes([TYPE_MSP_WRITE, ADDR_FC, ADDR_RADIO, status]) + chunk
        frames.append(bytes([SYNC_FC, len(inner) + 1]) + inner + bytes([crc8(inner)]))
        seq = (seq + 1) & 0xF
    return frames


def encode_msp_set_motor(values) -> list:
    """MSP_SET_MOTOR (214): eight u16 LE motor values (1000 = stopped, 2000 = full), written to the FC as chunked 0x7C frames in the
    same style as encode_msp_set_vtx_config_power. Betaflight honours it only while DISARMED. Body (22 bytes): flag 00, cmd D6 00, size 10 00,
    16 payload bytes, XOR of the 21 bytes before it. Four values are accepted and padded with 1000 (motors 5..8 stay stopped)."""
    vals = [int(v) for v in values]
    if not 1 <= len(vals) <= 8:
        raise ValueError("1..8 motor values")
    vals += [1000] * (8 - len(vals))
    if any(not 1000 <= v <= 2000 for v in vals):
        raise ValueError("motor values must be 1000..2000")
    body = bytes([0x00]) + struct.pack("<HH", MSP_SET_MOTOR, 16) + struct.pack("<8H", *vals)
    xor = 0
    for b in body:
        xor ^= b
    body += bytes([xor])
    frames, seq = [], 0
    for i in range(0, len(body), 7):
        chunk = body[i:i + 7]
        status = 0x40 | (0x10 if seq == 0 else 0) | (seq & 0xF)
        inner = bytes([TYPE_MSP_WRITE, ADDR_FC, ADDR_RADIO, status]) + chunk
        frames.append(bytes([SYNC_FC, len(inner) + 1]) + inner + bytes([crc8(inner)]))
        seq = (seq + 1) & 0xF
    return frames


MSP2_DF_POS_AID = 0x3F00      # DroneForge v4.7 firmware: the PC's visual-odometry position aid for the FC sideways hold


def encode_msp_write(cmd: int, payload: bytes) -> list:
    """Any MSP write to the FC as chunked 0x7C frames, the same body and chunking as encode_msp_set_motor (proven on the drone): flag 00, cmd u16, size u16, payload, XOR."""
    body = bytes([0x00]) + struct.pack("<HH", cmd, len(payload)) + payload
    xor = 0
    for b in body:
        xor ^= b
    body += bytes([xor])
    frames, seq = [], 0
    for i in range(0, len(body), 7):
        chunk = body[i:i + 7]
        status = 0x40 | (0x10 if seq == 0 else 0) | (seq & 0xF)
        inner = bytes([TYPE_MSP_WRITE, ADDR_FC, ADDR_RADIO, status]) + chunk
        frames.append(bytes([SYNC_FC, len(inner) + 1]) + inner + bytes([crc8(inner)]))
        seq = (seq + 1) & 0xF
    return frames


def encode_msp_pos_aid(forward_m: float, right_m: float, valid: bool) -> list:
    """MSP2_DF_POS_AID (v4.7 firmware): the drone's displacement from its hold point in its own axes, from the PC's visual odometry. int16 mm, int16 mm, u8 flags (bit0 valid).
    Firmware older than v4.7 ignores the unknown command."""
    f = max(-32000, min(32000, int(round(forward_m * 1000.0)))); r = max(-32000, min(32000, int(round(right_m * 1000.0))))
    return encode_msp_write(MSP2_DF_POS_AID, struct.pack("<hhB", f, r, 1 if valid else 0))


def decode_msp_pos_aid(payload: bytes) -> Optional[dict]:
    if len(payload) < 5:
        return None
    f, r, flags = struct.unpack("<hhB", payload[:5])
    return {"forward_m": f / 1000.0, "right_m": r / 1000.0, "valid": bool(flags & 1)}


def decode_vtx_config(payload: bytes) -> Optional[dict]:
    """MSP_VTX_CONFIG reply, layout from Betaflight's msp.c: type, band, channel, power, pit mode, u16 freq (LE),
    device ready, low-power-disarm, u16 pit-mode freq, table available, bands, channels, power levels."""
    if len(payload) < 15:
        return None
    t, band, ch, power, pit = payload[:5]
    freq, ready, lpd, pitfreq = struct.unpack_from("<HBBH", payload, 5)
    avail, bands, channels, levels = payload[11:15]
    return {"vtx_type": t, "band": band, "channel": ch, "power_level": power, "pit_mode": bool(pit),
            "frequency_mhz": freq, "device_ready": bool(ready), "low_power_disarm": lpd, "pit_mode_freq_mhz": pitfreq,
            "table_available": bool(avail), "table_bands": bands, "table_channels": channels, "table_power_levels": levels}


def decode_vtx_set_config(payload: bytes) -> Optional[dict]:
    """Decode the 4-byte MSP_SET_VTX_CONFIG payload Nimbus sends (for checking and for the emulator)."""
    if len(payload) < 4:
        return None
    v, power, pit = struct.unpack("<HBB", payload[:4])
    if v > 63:
        return {"frequency_mhz": v, "power_level": power, "pit_mode": bool(pit)}
    return {"band_index": v // 8 + 1, "band_letter": VRX_BANDS[v // 8], "channel": v % 8 + 1, "power_level": power,
            "pit_mode": bool(pit)}


class MspResponseAssembler:
    """Rebuilds chunked MSP-over-CRSF responses (frame type 0x7B), after TelemetryIngestLoop::TryBuildCompleteMspResponse.

    Frame payload: dest, origin, status, data...  Status: bit4 = start chunk, bits5-6 = version (must be 0x40 = MSPv2),
    bits0-3 = sequence. The start chunk carries flag, cmd(2), size(2); the response is complete once size + 8 bytes
    (including dest/origin/status) have been collected. Continuation chunks must arrive with the expected sequence.
    """

    def __init__(self, frame_type: int = TYPE_MSP_RESP) -> None:
        self.frame_type = frame_type        # TYPE_MSP_RESP for replies; TYPE_MSP_WRITE to reassemble written requests
        self.dropped = 0
        self._reset()

    def _reset(self) -> None:
        self._buf: Optional[bytearray] = None
        self._expected = 0
        self._seq = 0

    @staticmethod
    def _build(raw: bytes) -> dict:
        flags = raw[3]
        cmd = struct.unpack_from("<H", raw, 4)[0]
        size = struct.unpack_from("<H", raw, 6)[0]
        return {"dest": raw[0], "origin": raw[1], "flags": flags, "cmd": cmd, "payload": bytes(raw[8:8 + size])}

    def feed(self, frame: Frame) -> Optional[dict]:
        p = frame.payload
        if frame.type != self.frame_type or len(p) < 3:
            return None
        status = p[2]
        if status & 0x10:                               # start chunk
            self._reset()
            if (status & 0x60) != 0x40 or len(p) < 8:
                self.dropped += 1
                return None
            total = struct.unpack_from("<H", p, 6)[0] + 8
            if len(p) >= total:
                return self._build(p[:total])
            self._buf, self._expected, self._seq = bytearray(p), total, (status + 1) & 0xF
            return None
        if self._buf is None or (status & 0x60) != 0x40 or len(p) == 3 or (status & 0xF) != self._seq:
            self.dropped += 1
            self._reset()
            return None
        self._buf += p[3:3 + (self._expected - len(self._buf))]
        self._seq = (self._seq + 1) & 0xF
        if len(self._buf) >= self._expected:
            raw = bytes(self._buf[:self._expected])
            self._reset()
            return self._build(raw)
        return None


def _lenstr(p: bytes, i: int) -> tuple[str, int]:
    n = p[i]
    return p[i + 1:i + 1 + n].decode("ascii", "replace"), i + 1 + n


def decode_board_info(payload: bytes) -> Optional[dict]:
    """MSP_BOARD_INFO: board id(4), hw rev u16, osd u8, comms u8, then length-prefixed MCU, target and manufacturer
    strings, 32-byte signature, MCU type id and any extra bytes (layout as seen on a Matek H743 running 4.5.4)."""
    if len(payload) < 9:
        return None
    try:
        board_id = payload[:4].decode("ascii", "replace")
        hw = struct.unpack_from("<H", payload, 4)[0]
        mcu, i = _lenstr(payload, 8)
        target, i = _lenstr(payload, i)
        vendor, i = _lenstr(payload, i)
    except (IndexError, struct.error):
        return {"board_id": payload[:4].decode("ascii", "replace")}
    return {"board_id": board_id, "hardware_revision": hw, "osd": payload[6], "comms": payload[7], "mcu": mcu,
            "target": target, "manufacturer": vendor, "extra": payload[i:].hex()}
