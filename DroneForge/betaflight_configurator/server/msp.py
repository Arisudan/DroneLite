"""MSP v1 framing and parsing. The command allow-list is enforced in ONE place: encode_request()."""
import struct

API_VERSION, FC_VARIANT, FC_VERSION, BOARD_INFO, BUILD_INFO = 1, 2, 3, 4, 5
SET_REBOOT, STATUS, RAW_IMU, MOTOR, RC, ATTITUDE, ANALOG = 68, 101, 102, 104, 105, 108, 110
STATUS_EX, SENSOR_STATUS, UID, ACC_CALIBRATION, EEPROM_WRITE = 150, 151, 160, 205, 250

READS = {1: "API_VERSION", 2: "FC_VARIANT", 3: "FC_VERSION", 4: "BOARD_INFO", 5: "BUILD_INFO",
         101: "STATUS", 150: "STATUS_EX", 102: "RAW_IMU", 104: "MOTOR", 105: "RC",
         108: "ATTITUDE", 110: "ANALOG", 160: "UID", 151: "SENSOR_STATUS"}
ALLOWED = dict(READS)
ALLOWED.update({205: "ACC_CALIBRATION", 68: "SET_REBOOT", 250: "EEPROM_WRITE"})
# Never allowed (documented for tests): SET_MOTOR 214, SET_CF_SERIAL_CONFIG 55, SET_RAW_RC 200, RESET_CONF 208


class MspError(Exception):
    pass


class AllowListError(MspError):
    pass


def _crc(length, cmd, payload):
    c = length ^ cmd
    for b in payload:
        c ^= b
    return c


def _frame(direction, cmd, payload):
    payload = bytes(payload)
    if len(payload) > 255:
        raise MspError("payload too long for MSP v1")
    return b"$M" + direction + bytes([len(payload), cmd]) + payload + bytes([_crc(len(payload), cmd, payload)])


def encode_request(cmd, payload=b""):
    """The single allow-list gate: every byte sent to the FC as MSP goes through here."""
    if cmd not in ALLOWED:
        raise AllowListError("MSP command %s is not on the allow-list" % cmd)
    return _frame(b"<", cmd, payload)


def encode_response(cmd, payload=b"", error=False):
    """Used only by the emulator (FC side)."""
    return _frame(b"!" if error else b">", cmd, payload)


class Parser:
    """Incremental MSP v1 parser. feed() returns [(cmd, payload, direction_char)]."""

    def __init__(self):
        self.buf = bytearray()
        self.errors = 0

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(b"$M")
            if i < 0:
                if self.buf[-1:] != b"$":
                    self.buf.clear()
                else:
                    del self.buf[:-1]
                break
            del self.buf[:i]
            if len(self.buf) < 3:
                break
            if self.buf[2] not in (60, 62, 33):
                del self.buf[:2]
                continue
            if len(self.buf) < 5:
                break
            n, cmd = self.buf[3], self.buf[4]
            total = 6 + n
            if len(self.buf) < total:
                break
            payload = bytes(self.buf[5:5 + n])
            if self.buf[5 + n] == _crc(n, cmd, payload):
                out.append((cmd, payload, chr(self.buf[2])))
                del self.buf[:total]
            else:
                self.errors += 1
                del self.buf[:2]
        return out


def _s(b):
    return b.decode("ascii", "replace")


def parse_info(api=b"", variant=b"", version=b"", board=b"", build=b"", uid=b""):
    d = {}
    if len(api) >= 3:
        d["api"] = "%d.%d" % (api[1], api[2])
    d["variant"] = _s(variant[:4])
    if len(version) >= 3:
        d["version"] = "%d.%d.%d" % tuple(version[:3])
    if len(board) >= 4:
        d["board"] = _s(board[:4])
        if len(board) >= 9 and board[8] <= len(board) - 9 and board[8] > 0:
            d["board"] = _s(board[9:9 + board[8]])
    if len(build) >= 19:
        d["build"] = _s(build[:11]) + " " + _s(build[11:19])
    d["uid"] = uid[:12].hex()
    return d


def parse_status_ex(p):
    if len(p) < 15:
        raise MspError("short STATUS_EX")
    cycle, i2c, sensors, flags, profile, cpu = struct.unpack_from("<HHHIBH", p, 0)
    return {"cycle_us": cycle, "i2c_errors": i2c, "sensors": sensors, "flags": flags,
            "profile": profile, "cpu_load": cpu, "armed": bool(flags & 1)}


def parse_status(p):
    if len(p) < 11:
        raise MspError("short STATUS")
    cycle, i2c, sensors, flags = struct.unpack_from("<HHHI", p, 0)
    return {"cycle_us": cycle, "i2c_errors": i2c, "sensors": sensors, "flags": flags, "armed": bool(flags & 1)}


def parse_attitude(p):
    r, pt, y = struct.unpack_from("<hhh", p, 0)
    return {"roll": r / 10.0, "pitch": pt / 10.0, "yaw": y}


def parse_u16s(p):
    return list(struct.unpack("<%dH" % (len(p) // 2), p[:len(p) // 2 * 2]))


def parse_imu(p):
    v = struct.unpack_from("<9h", p, 0)
    return {"acc": [round(x / 512.0, 3) for x in v[0:3]], "gyro": [round(x / 16.4, 2) for x in v[3:6]]}


def parse_analog(p):
    vb, mah, rssi, amps = struct.unpack_from("<BHHh", p, 0)
    return {"vbat": vb / 10.0, "mah": mah, "rssi": round(rssi * 100 / 1023.0), "amps": amps / 100.0}
