"""CRSF encode/decode helpers: RC channels frame (matches NimbusOS 2.9.0) and common telemetry frames.

Confirmed against the nimbus_core binary: RC frame layout, CRC-8 poly 0xD5, 172/992/1811 limits.
Telemetry decoders follow the public CRSF definitions and are NOT yet verified against real traffic.
"""
from __future__ import annotations

import math
import re
import struct
from dataclasses import dataclass
from typing import Optional, Sequence

CRSF_MIN, CRSF_MID, CRSF_MAX = 172, 992, 1811

SYNC_FC = 0xC8     # sync byte NimbusOS puts on RC frames
SYNC_RADIO = 0xEA
SYNC_TX = 0xEE
SYNC_RX = 0xEC
VALID_SYNC = (SYNC_FC, SYNC_RADIO, SYNC_TX, SYNC_RX)

ADDR_FC, ADDR_RADIO, ADDR_RX, ADDR_TX = 0xC8, 0xEA, 0xEC, 0xEE   # CRSF device addresses

TYPE_VARIO = 0x07
TYPE_BATTERY = 0x08
TYPE_BARO_ALT = 0x09
TYPE_LINK_STATISTICS = 0x14
TYPE_RC_CHANNELS = 0x16
TYPE_ATTITUDE = 0x1E
TYPE_FLIGHT_MODE = 0x21
TYPE_DEVICE_PING = 0x28
TYPE_DEVICE_INFO = 0x29
TYPE_OPTRANGE = 0xD3      # custom Nimbus frame: range + optical flow from the drone (10 data bytes; our FC sends them after a 00 C8 address prefix = 12-byte payload)
TYPE_GPS = 0x02
TYPE_MAG = 0x12
TYPE_RANGEFINDER = 0xD0   # custom: rangefinder distance + strength (4 bytes)
TYPE_RAW_IMU = 0xD1       # custom: raw gyro + accel (12 bytes)
TYPE_MIGHTY_CAM = 0xD2    # custom: MightyCam pose (13 bytes valid, 4 bytes invalid)
TYPE_MOTOR_PWM = 0xD4     # custom: motor PWM x4 + pole count (18 bytes), probable
TYPE_MOTOR_ERPM = 0xD5    # custom: motor eRPM x4 (16 bytes), probable
TYPE_VRX_TOP_CHANNELS = 0xE1   # custom: 48 bytes of video-receiver RSSI by frequency order
TYPE_DRONE_LINKED = 0xE2  # custom: 1 byte, drone link state
TYPE_FC_UID = 0xE3        # custom: flight controller UID, 12 bytes
TYPE_STATE = 0xD6        # PROPOSED (not in the flashed firmware): range + flow + EKF velocity + attitude in ONE frame (20 data bytes), replaces D3 + D4 + ATTITUDE on the air
TYPE_CUSTOM_E4 = 0xE4     # custom, 6 bytes, not identified


def _make_crc_table() -> bytes:
    table = bytearray(256)
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
        table[i] = crc
    return bytes(table)


_CRC = _make_crc_table()


def crc8(data: bytes, crc: int = 0) -> int:
    """CRC-8/DVB-S2 (poly 0xD5, init 0)."""
    for b in data:
        crc = _CRC[crc ^ b]
    return crc


def pack_channels(channels: Sequence[int]) -> bytes:
    """Pack 16 channels (0..2047) into 22 bytes, 11 bits each, little-endian bit order."""
    if len(channels) != 16:
        raise ValueError("CRSF RC frame needs exactly 16 channels")
    bits = 0
    for i, c in enumerate(channels):
        bits |= (int(c) & 0x7FF) << (11 * i)
    return bits.to_bytes(22, "little")


def unpack_channels(payload: bytes) -> list[int]:
    if len(payload) != 22:
        raise ValueError("RC payload must be 22 bytes")
    bits = int.from_bytes(payload, "little")
    return [(bits >> (11 * i)) & 0x7FF for i in range(16)]


def encode_frame(frame_type: int, payload: bytes, sync: int = SYNC_FC) -> bytes:
    body = bytes([frame_type]) + payload
    if len(body) + 1 > 62:
        raise ValueError("payload too long for a CRSF frame")
    return bytes([sync, len(body) + 1]) + body + bytes([crc8(body)])


def encode_rc_frame(channels: Sequence[int], sync: int = SYNC_FC) -> bytes:
    """26-byte RC_CHANNELS_PACKED frame, byte-identical to NimbusOS's EncodeControlFrame."""
    return encode_frame(TYPE_RC_CHANNELS, pack_channels(channels), sync)


def encode_device_ping(destination: int, origin: int = ADDR_RADIO) -> bytes:
    """6-byte DEVICE_PING: EE 04 28 <dest> <origin> CRC.
    Layout read from nimbus_core CrsfTransmitEncoder::EncodeDevicePingFrame (sync is always 0xEE)."""
    body = bytes([TYPE_DEVICE_PING, destination, origin])
    return bytes([SYNC_TX, len(body) + 1]) + body + bytes([crc8(body)])


@dataclass
class Frame:
    sync: int
    type: int
    payload: bytes


class FrameParser:
    """Byte-stream parser: finds sync, checks length and CRC, resynchronises after garbage."""

    def __init__(self) -> None:
        self._buf = bytearray()
        self.crc_errors = 0
        self.dropped_bytes = 0

    def feed(self, data: bytes) -> list[Frame]:
        self._buf += data
        out: list[Frame] = []
        buf = self._buf
        while len(buf) >= 4:
            if buf[0] not in VALID_SYNC:
                del buf[0]
                self.dropped_bytes += 1
                continue
            length = buf[1]
            if length < 2 or length > 62:
                del buf[0]
                self.dropped_bytes += 1
                continue
            total = length + 2
            if len(buf) < total:
                break
            body = bytes(buf[2:1 + length])
            if crc8(body) == buf[1 + length]:
                out.append(Frame(buf[0], body[0], body[1:]))
                del buf[:total]
            else:
                self.crc_errors += 1
                del buf[0]
                self.dropped_bytes += 1
        return out


def _rssi_dbm(b: int) -> int:
    """CRSF spec sends |dBm| (0..127); ELRS 4.x was seen sending the signed byte (208 -> -48 dBm). Real RSSI never exceeds 127 in magnitude."""
    return -b if b < 128 else b - 256


def decode_link_statistics(p: bytes) -> Optional[dict]:
    if len(p) < 10:
        return None
    return {"up_rssi1_dbm": _rssi_dbm(p[0]), "up_rssi2_dbm": _rssi_dbm(p[1]), "up_lq": p[2],
            "up_snr_db": struct.unpack("b", p[3:4])[0], "antenna": p[4], "rf_mode": p[5],
            "tx_power_idx": p[6], "down_rssi_dbm": _rssi_dbm(p[7]), "down_lq": p[8],
            "down_snr_db": struct.unpack("b", p[9:10])[0]}


def decode_battery(p: bytes) -> Optional[dict]:
    if len(p) < 8:
        return None
    volts, amps = struct.unpack(">HH", p[:4])
    # CRSF spec: 0.1 V units. The Matek/Betaflight build here sends 0.01 V (a 4S pack arrived as 1616, not 162).
    # No pack in this project is above 60 V, so a 0.1 V reading above 600 is taken as 0.01 V.
    v = volts / 10.0 if volts <= 600 else volts / 100.0
    return {"voltage_v": v, "current_a": amps / 10.0,
            "capacity_mah": int.from_bytes(p[4:7], "big"), "remaining_pct": p[7]}


def decode_attitude(p: bytes) -> Optional[dict]:
    if len(p) < 6:
        return None
    pitch, roll, yaw = struct.unpack(">hhh", p[:6])
    return {"pitch_rad": pitch / 10000.0, "roll_rad": roll / 10000.0, "yaw_rad": yaw / 10000.0}


_FM_MOT = re.compile(r" M(-?\d+),(-?\d+),(-?\d+),(-?\d+) T(-?\d+)")
_FM_PID = re.compile(r" P(-?\d+),(-?\d+),(-?\d+) I(-?\d+),(-?\d+),(-?\d+) D(-?\d+),(-?\d+),(-?\d+)")
_FM_GYR = re.compile(r" G(-?\d+),(-?\d+),(-?\d+) S(-?\d+),(-?\d+),(-?\d+)")


def decode_flight_mode(p: bytes) -> Optional[dict]:
    """Flight mode text, e.g. 'STAB', 'STAB HH' (FC height hold active), 'STAB*' (disarmed). Our FC firmware appends ONE diagnostic group per frame, in rotation, BEFORE the optional '*':
      ' M<m1>,<m2>,<m3>,<m4> T<thr>'          motor outputs (1000..2000) and the throttle in use (us)
      ' P<r>,<p>,<y> I<r>,<p>,<y> D<r>,<p>,<y>'   PID terms per axis (roll, pitch, yaw), about motor microseconds
      ' G<r>,<p>,<y> S<r>,<p>,<y>'            filtered gyro rate (0.1 deg/s) and the PID sum per axis
    They are split off here, so 'mode' keeps its old meaning."""
    text = p.split(b"\0", 1)[0].decode("ascii", "replace")
    star = "*" if text.endswith("*") else ""
    for rx, build in ((_FM_MOT, lambda m: {"motors": [int(m.group(i)) for i in (1, 2, 3, 4)], "thr_us": int(m.group(5))}),
                      (_FM_PID, lambda m: {"pid_p": [int(m.group(i)) for i in (1, 2, 3)], "pid_i": [int(m.group(i)) for i in (4, 5, 6)], "pid_d": [int(m.group(i)) for i in (7, 8, 9)]}),
                      (_FM_GYR, lambda m: {"gyro_dps10": [int(m.group(i)) for i in (1, 2, 3)], "pid_sum": [int(m.group(i)) for i in (4, 5, 6)]})):
        m = rx.search(text)
        if m:
            d = {"mode": text[:m.start()] + star}; d.update(build(m)); return d
    return {"mode": text}


def decode_vario(p: bytes) -> Optional[dict]:
    if len(p) < 2:
        return None
    return {"vspeed_cm_s": struct.unpack(">h", p[:2])[0]}


def decode_baro_alt(p: bytes) -> Optional[dict]:
    if len(p) < 2:
        return None
    raw = struct.unpack(">H", p[:2])[0]
    meters = float(raw & 0x7FFF) if raw & 0x8000 else (raw - 10000) / 10.0
    return {"baro_alt_m": meters}


def decode_optrange(p: bytes) -> Optional[dict]:
    """Custom frame 0xD3 -> OptrangeData, as built by nimbus_core's telemetry normalizer.

    Payload (10 bytes): distance i16 BE [mm], strength u8, precision u8, flow_x i16 BE, flow_y i16 BE,
    flow_quality u8, flow_status u8. Distance is only valid for raw 1..5999 (else 0.0), scale 1/1000 -> metres.
    Velocities: forward = distance * flow_x/100, right = +distance * flow_y/100. 2026-11-23 owner bench (drone on a chair rolled to the RIGHT, v4.7 FC):
    the raw flow_y is + to the right, as the FC's own estimator takes it (kinematic_estimator.c: flowY = right); the earlier '-' (inferred 2026-11-03) was mirrored.
    """
    if len(p) == 12 and p[0] == 0x00 and p[1] == 0xC8:   # our FC prefixes [dest 0x00][origin 0xC8] (ELRS 4.x reads types >= 0x28 as addressed)
        p = p[2:]
    if len(p) < 10:
        return None
    raw = struct.unpack(">H", p[0:2])[0]
    distance_m = (struct.unpack(">h", p[0:2])[0] / 1000.0) if 1 <= raw <= 5999 else 0.0
    flow_x, flow_y = struct.unpack(">hh", p[4:8])
    return {"distance_m": distance_m, "dist_strength": p[2], "dist_precision": p[3],
            "velocity_forward": distance_m * (flow_x / 100.0), "velocity_right": distance_m * (flow_y / 100.0),
            "flow_quality": p[8], "flow_status": p[9], "flow_x_raw": flow_x, "flow_y_raw": flow_y}



# --- Custom Nimbus frames 0xD0-0xE4. Identification: decoder case sizes + normalizer blocks + app schema + log strings.
# Confidence is noted per decoder. Scalings read from nimbus_core constants unless stated.

def decode_rangefinder(p: bytes) -> Optional[dict]:
    """0xD0 (CONFIDENT): u16 BE distance in cm (Nimbus divides by 100 -> metres), u16 BE strength."""
    if len(p) < 4:
        return None
    dist, strength = struct.unpack(">HH", p[:4])
    return {"distance_m": dist / 100.0, "strength": strength}


def decode_raw_imu(p: bytes) -> Optional[dict]:
    """0xD1 (structure confident, scaling UNVERIFIED): 6 x int16 BE = gyro xyz, accel xyz.
    Scaling read from vectorised code: gyro = raw/100 * 180/pi (deg/s), accel = raw/100."""
    if len(p) < 12:
        return None
    g = struct.unpack(">3h", p[0:6])
    a = struct.unpack(">3h", p[6:12])
    k = 180.0 / math.pi / 100.0
    return {"gyro_forward": g[0] * k, "gyro_right": g[1] * k, "gyro_down": g[2] * k,
            "accel_forward": a[0] / 100.0, "accel_right": a[1] / 100.0, "accel_down": a[2] / 100.0,
            "raw": list(g) + list(a)}


def decode_kinematic(p: bytes) -> Optional[dict]:
    """0xD4 from OUR FC firmware (kinematic_estimator.c): the FC's own IMU-rate EKF state. Payload = [dest 00][origin C8] + 9 x int16 BE:
    position x,y,z in mm, velocity x,y,z in mm/s, attitude roll,pitch,yaw in rad*10000. Position/velocity are relative to the point where the drone was armed
    (the filter resets on arm); +-32.767 m / m/s range. 0x7FFF / 0x8000 = not initialised (NaN/inf in the FC)."""
    if len(p) == 20 and p[0] == 0x00 and p[1] == 0xC8:
        p = p[2:]
    if len(p) < 18:
        return None
    v = struct.unpack(">9h", p[:18])
    invalid = any(x in (0x7FFF, -0x8000) for x in v[:6])
    return {"x_m": v[0] / 1000.0, "y_m": v[1] / 1000.0, "z_m": v[2] / 1000.0, "vx_mps": v[3] / 1000.0, "vy_mps": v[4] / 1000.0, "vz_mps": v[5] / 1000.0,
            "roll_rad": v[6] / 10000.0, "pitch_rad": v[7] / 10000.0, "yaw_rad": v[8] / 10000.0, "valid": not invalid}


STATE_PAYLOAD_BYTES = 20
STATE2_PAYLOAD_BYTES = 14      # firmware v4.8: the shorter layout of the same frame type (telemetry/crsf_state_pack.h), told apart by its length


def _s12(v: int) -> int:
    return v - 0x1000 if v & 0x800 else v


def decode_state_v48(p: bytes) -> dict:
    """The 14 data bytes of the v4.8 state frame -> the same dict, in the same units, as the 20-byte frame."""
    dist, flags, q, v0, v1, v2, vz, roll, pitch, yaw, fx, fy = struct.unpack(">hBBBBBbbbhbb", p[:STATE2_PAYLOAD_BYTES])
    return {"dist_raw": dist, "flow_x_raw": fx * 2, "flow_y_raw": fy * 2, "flow_quality": q, "ekf_valid": bool(flags & 1), "flow_status": 1 if flags & 2 else 0,
            "hold_active": bool(flags & 4), "hold_fault": bool(flags & 8), "hold_mode": (flags >> 4) & 3, "vel_body": bool(flags & 0x40), "xy_disabled": bool(flags & 0x80),
            "vx_mm": _s12((v0 << 4) | (v1 >> 4)), "vy_mm": _s12(((v1 & 0x0F) << 8) | v2), "vz_mm": vz * 10,
            "roll_e4": int(round(roll * 87.266)), "pitch_e4": int(round(pitch * 87.266)), "yaw_e4": yaw}


def encode_state(distance_raw_mm: int, flow_x: int, flow_y: int, flow_quality: int, ekf_valid: bool, flow_status: int,
                 vx_mm_s: int, vy_mm_s: int, vz_mm_s: int, roll_e4: int, pitch_e4: int, yaw_e4: int, hold_bits: int = 0, body_vel: bool = False) -> bytes:
    """The compact state frame 0xD6 as the FC would send it: [dest 00][origin C8] + 20 data bytes, big-endian, the SAME units as D3 and D4 (so the PC
    needs no new scaling): distance i16 mm (1..5999 valid, else none), flow_x i16, flow_y i16 (raw), flow quality u8, flags u8 (bit0 EKF valid, bit1 flow ok),
    vx, vy, vz i16 mm/s (FC world frame), roll, pitch, yaw i16 rad*1e4. 26 bytes on the wire against 40 (D3 16 + D4 24) -- position x,y,z is NOT carried."""
    flags = (1 if ekf_valid else 0) | (2 if flow_status else 0) | (hold_bits & 0xBC) | (0x40 if body_vel else 0)      # bit2 FC hold active, bit3 hold fault, bits 4-5 hold mode (0 hold, 1 take-off, 2 land)
    body = struct.pack(">hhhBBhhhhhh", distance_raw_mm, flow_x, flow_y, flow_quality & 0xFF, flags, vx_mm_s, vy_mm_s, vz_mm_s, roll_e4, pitch_e4, yaw_e4)
    return encode_frame(TYPE_STATE, b"\x00\xc8" + body)


def decode_state(p: bytes) -> Optional[dict]:
    """0xD6 -> one message holding both halves; split_state() turns it into the optrange + kinematic messages the rest of the program already understands."""
    if len(p) == STATE2_PAYLOAD_BYTES + 2 and p[0] == 0x00 and p[1] == 0xC8:
        return decode_state_v48(p[2:])
    if len(p) == STATE_PAYLOAD_BYTES + 2 and p[0] == 0x00 and p[1] == 0xC8:
        p = p[2:]
    if len(p) < STATE_PAYLOAD_BYTES:
        return None
    dist, fx, fy, q, flags, vx, vy, vz, r, pi, ya = struct.unpack(">hhhBBhhhhhh", p[:STATE_PAYLOAD_BYTES])
    return {"dist_raw": dist, "flow_x_raw": fx, "flow_y_raw": fy, "flow_quality": q, "ekf_valid": bool(flags & 1), "flow_status": 1 if flags & 2 else 0,
            "hold_active": bool(flags & 4), "hold_fault": bool(flags & 8), "hold_mode": (flags >> 4) & 3, "vel_body": bool(flags & 0x40), "xy_disabled": bool(flags & 0x80),
            "vx_mm": vx, "vy_mm": vy, "vz_mm": vz, "roll_e4": r, "pitch_e4": pi, "yaw_e4": ya}


def split_state(d: dict) -> list:
    """[(name, dict)] exactly as decode_optrange and decode_kinematic would give them (position is not carried: x/y/z are NaN, never a made-up number)."""
    raw = d["dist_raw"]
    dist = raw / 1000.0 if 1 <= raw <= 5999 else 0.0
    opt = {"distance_m": dist, "dist_strength": 255, "dist_precision": 1,
           "velocity_forward": dist * (d["flow_x_raw"] / 100.0), "velocity_right": dist * (d["flow_y_raw"] / 100.0),
           "flow_quality": d["flow_quality"], "flow_status": d["flow_status"], "flow_x_raw": d["flow_x_raw"], "flow_y_raw": d["flow_y_raw"]}
    nan = float("nan")
    kin = {"x_m": nan, "y_m": nan, "z_m": nan, "vx_mps": d["vx_mm"] / 1000.0, "vy_mps": d["vy_mm"] / 1000.0, "vz_mps": d["vz_mm"] / 1000.0,
           "roll_rad": d["roll_e4"] / 10000.0, "pitch_rad": d["pitch_e4"] / 10000.0, "yaw_rad": d["yaw_e4"] / 10000.0, "valid": bool(d["ekf_valid"]),
           "hold_active": d.get("hold_active", False), "hold_fault": d.get("hold_fault", False), "hold_mode": d.get("hold_mode", 0), "vel_body": d.get("vel_body", False), "xy_disabled": d.get("xy_disabled", False)}
    return [("optrange", opt), ("kinematic", kin)]


def decode_mag(p: bytes) -> Optional[dict]:
    """0x12 (CONFIDENT on scale): 3 x int16 BE, divided by 3.0 in Nimbus (constant read from the binary)."""
    if len(p) < 6:
        return None
    x, y, z = struct.unpack(">3h", p[:6])
    return {"field_forward": x / 3.0, "field_right": y / 3.0, "field_down": z / 3.0}


def decode_drone_linked(p: bytes) -> Optional[dict]:
    """0xE2 (CONFIDENT): 1 byte, non-zero = drone linked (log string 'telemetry: droneLinked changed')."""
    return {"is_linked": p[0] != 0} if p else None


def decode_fc_uid(p: bytes) -> Optional[dict]:
    """0xE3 (frame CONFIDENT, byte order inferred): 12 bytes = three u32 (little-endian) of the MCU UID."""
    if len(p) < 12:
        return None
    u = struct.unpack("<3I", p[:12])
    return {"uid0": u[0], "uid1": u[1], "uid2": u[2], "uid_hex": "-".join(f"{x:08x}" for x in u)}


def decode_vrx_top_channels(p: bytes) -> Optional[dict]:
    """0xE1 (frame CONFIDENT, layout INFERRED): 48 raw bytes copied unmodified into 'videoReceiverTopChannels'
    (schema: rssiByFreqOrder[]). 48 = 6 bands (A B E F R L) x 8 channels, so treated as 48 RSSI bytes."""
    if len(p) != 48:
        return None
    return {"rssi_by_freq_order": list(p)}


def decode_custom_raw(frame_type: int, name: str):
    def _fn(p: bytes) -> Optional[dict]:
        return {"frame_type": f"0x{frame_type:02X}", "meaning": name, "hex": p.hex(), "decoded": False}
    return _fn


def format_device_version(sw: int) -> str:
    """Nimbus (FormatDeviceVersion) prints the top three bytes of the 32-bit value as major.minor.patch."""
    return f"{(sw >> 24) & 0xFF}.{(sw >> 16) & 0xFF}.{(sw >> 8) & 0xFF}"


def decode_device_info(p: bytes) -> Optional[dict]:
    """DEVICE_INFO (0x29) extended frame payload: dest, origin, name\0, serial u32, hw u32, sw u32, params u8, proto u8."""
    if len(p) < 3:
        return None
    dest, origin = p[0], p[1]
    end = p.find(b"\0", 2)
    if end < 0 or len(p) < end + 1 + 14:
        return {"origin": origin, "dest": dest, "name": p[2:].decode("ascii", "replace")}
    serial, hw, sw = struct.unpack(">III", p[end + 1:end + 13])
    return {"origin": origin, "dest": dest, "name": p[2:end].decode("ascii", "replace"), "serial": serial,
            "hardware_id": hw, "software_id": sw, "version": format_device_version(sw),
            "param_count": p[end + 13], "param_protocol": p[end + 14] if len(p) > end + 14 else None}


_DECODERS = {TYPE_STATE: ("state_compact", decode_state), TYPE_OPTRANGE: ("optrange", decode_optrange), TYPE_RANGEFINDER: ("rangefinder", decode_rangefinder),
             TYPE_RAW_IMU: ("raw_imu", decode_raw_imu), TYPE_MAG: ("magnetometer", decode_mag),
             TYPE_DRONE_LINKED: ("drone_linked", decode_drone_linked), TYPE_FC_UID: ("fc_uid", decode_fc_uid),
             TYPE_VRX_TOP_CHANNELS: ("vrx_top_channels", decode_vrx_top_channels),
             TYPE_MIGHTY_CAM: ("custom_d2", decode_custom_raw(TYPE_MIGHTY_CAM, "MightyCam pose (not decoded)")),
             TYPE_MOTOR_PWM: ("kinematic", decode_kinematic),         # 0xD4 on OUR FC = the EKF state (Nimbus's own use of 0xD4 is unconfirmed)
             TYPE_MOTOR_ERPM: ("custom_d5", decode_custom_raw(TYPE_MOTOR_ERPM, "motor eRPM x4 (probable, not decoded)")),
             TYPE_CUSTOM_E4: ("custom_e4", decode_custom_raw(TYPE_CUSTOM_E4, "unidentified 6-byte frame")), TYPE_DEVICE_INFO: ("device_info", decode_device_info), TYPE_LINK_STATISTICS: ("link", decode_link_statistics), TYPE_BATTERY: ("battery", decode_battery),
             TYPE_ATTITUDE: ("attitude", decode_attitude), TYPE_FLIGHT_MODE: ("flight_mode", decode_flight_mode),
             TYPE_VARIO: ("vario", decode_vario), TYPE_BARO_ALT: ("baro", decode_baro_alt)}


def decode_telemetry(frame: Frame) -> Optional[tuple[str, dict]]:
    entry = _DECODERS.get(frame.type)
    if entry is None:
        return None
    name, fn = entry
    data = fn(frame.payload)
    return None if data is None else (name, data)
