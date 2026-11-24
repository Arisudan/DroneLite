#!/usr/bin/env python3
"""Read (and optionally write) the ELRS TX's Lua parameters over CRSF: packet rate, telemetry ratio, power, ...
usage: elrs_tx_params_list_or_set.py <adapter_port>                      read-only: list every parameter
       elrs_tx_params_list_or_set.py <adapter_port> set "<name>" <value_index>   write one text-selection parameter (changes the TX config!)
Keeps sending disarmed RC frames while talking so the TX treats the PC as a handset. Never arms.
"""
import sys, time
sys.path.insert(0, "/home/htic/Music/DroneForge/software/pc")
from nimbus_pc.crsf import ADDR_RADIO, ADDR_TX, SYNC_TX, crc8
from nimbus_pc.sender import Controls
from nimbus_pc.session import Session

TYPE_READ, TYPE_ENTRY, TYPE_WRITE = 0x2C, 0x2B, 0x2D
TYPES = {0: "u8", 1: "i8", 2: "u16", 3: "i16", 8: "float", 9: "select", 10: "string", 11: "folder", 12: "info", 13: "command"}
import queue
replies = queue.Queue()     # (field, chunks_remaining, data)


def frame(t, payload):
    body = bytes([t]) + payload
    return bytes([SYNC_TX, len(body) + 1]) + body + bytes([crc8(body)])


def on_frame(f):
    if f.type == TYPE_ENTRY and len(f.payload) >= 4 and f.payload[0] == ADDR_RADIO:
        replies.put((f.payload[2], f.payload[3], bytes(f.payload[4:])))


def fetch(sess, field, tries=4):
    data, chunk = b"", 0
    while True:
        got = None
        for _ in range(tries):
            while not replies.empty():
                replies.get_nowait()
            sess.send_frame(frame(TYPE_READ, bytes([ADDR_TX, ADDR_RADIO, field, chunk])))
            end = time.monotonic() + 0.5
            while time.monotonic() < end and got is None:
                try:
                    r = replies.get(timeout=0.05)
                except queue.Empty:
                    continue
                if r[0] == field:
                    got = r
            if got:
                break
        if got is None:
            return None
        data += got[2]
        if got[1] == 0:
            return data
        chunk += 1


def parse(data):
    parent, typ = data[0], data[1] & 0x7F
    hidden = bool(data[1] & 0x80)
    i = data.index(0, 2); name = data[2:i].decode(errors="replace"); rest = data[i + 1:]
    info = {"parent": parent, "type": TYPES.get(typ, typ), "hidden": hidden, "name": name}
    if typ == 9 and rest:
        j = rest.index(0); opts = rest[:j].decode(errors="replace").split(";")
        info.update(options=opts, value=rest[j + 1], index=rest[j + 1])
        info["text"] = opts[rest[j + 1]] if rest[j + 1] < len(opts) else "?"
    elif typ in (0, 1) and rest:
        info["value"] = rest[0] if typ == 0 else rest[0] - 256 * (rest[0] > 127)
    elif typ == 12 and rest:
        info["value"] = rest.split(b"\0")[0].decode(errors="replace")
    return info


port = sys.argv[1]
sess = Session(port, hz=100, allow_arm=False, ping=False, on_telemetry=None)
sess.reader.on_frame = lambda f: (on_frame(f), sess._on_frame(f))
sess.start()
try:
    sess.set_controls(Controls()); time.sleep(1.0)
    n = None
    sess.send_frame(bytes([SYNC_TX, 4, 0x28, ADDR_TX, ADDR_RADIO, crc8(bytes([0x28, ADDR_TX, ADDR_RADIO]))])); time.sleep(0.5)
    n = sess.telemetry.get("device_info_0xee", {}).get("param_count") or 20
    params = {}
    for field in range(1, n + 1):
        d = fetch(sess, field)
        if d:
            params[field] = parse(d)
    if len(sys.argv) >= 5 and sys.argv[2] == "set":
        want, val = sys.argv[3], int(sys.argv[4])
        fid = next((k for k, v in params.items() if v["name"].lower() == want.lower() and v["type"] == "select"), None)
        if fid is None:
            sys.exit(f"no select parameter named '{want}'")
        sess.send_frame(frame(TYPE_WRITE, bytes([ADDR_TX, ADDR_RADIO, fid, val])))
        print(f"wrote {want} = index {val} ({params[fid]['options'][val] if val < len(params[fid]['options']) else '?'})")
        time.sleep(1.0); d = fetch(sess, fid); print("readback:", parse(d) if d else None)
    else:
        for k, v in params.items():
            line = f"{k:2d} [{v['type']:7s}] {'  ' if v['parent'] else ''}{v['name']}"
            if "text" in v: line += f" = {v['text']}   (options: {', '.join(v['options'])})"
            elif "value" in v: line += f" = {v['value']}"
            print(line + ("  (hidden)" if v["hidden"] else ""))
finally:
    sess.stop()
