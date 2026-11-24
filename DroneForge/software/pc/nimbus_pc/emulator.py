"""Software stand-in for a NimbusOS-style ELRS dongle on a pseudo-terminal, so the whole PC side can be run and tested
without hardware. It parses incoming CRSF frames, answers DEVICE_PING with DEVICE_INFO, answers MSP requests with chunked
MSP responses, records commands, and streams telemetry (link stats, battery, attitude, flight mode, range/flow 0xD3).

This is NOT a model of real ELRS behaviour: it only implements what the PC code is expected to exchange.
"""
from __future__ import annotations

import os
import select
import struct
import threading
import time
from typing import Optional

from . import commands
from .crsf import (ADDR_FC, ADDR_RADIO, ADDR_RX, ADDR_TX, SYNC_FC, SYNC_RADIO, TYPE_ATTITUDE, TYPE_BATTERY,
                   TYPE_DEVICE_INFO, TYPE_DEVICE_PING, TYPE_DRONE_LINKED, TYPE_FC_UID, TYPE_FLIGHT_MODE,
                   TYPE_LINK_STATISTICS, TYPE_OPTRANGE, TYPE_RANGEFINDER, TYPE_RC_CHANNELS, Frame, FrameParser, encode_frame, unpack_channels)

VTX_CONFIG = bytes([3, 4, 2, 2, 0]) + struct.pack("<HBBH", 5800, 1, 0, 0) + bytes([1, 6, 8, 5])
BOARD_INFO = b"SH74" + struct.pack("<H", 0) + bytes([0, 0]) + bytes([9]) + b"STM32H743" + bytes([9]) + b"MATEKH743" \
    + bytes([4]) + b"MTKS" + bytes(32) + bytes([12, 1, 64, 31])


WRITE_ERROR = 2              # Betaflight telemetry/msp_shared.c: TELEMETRY_MSP_ERROR, the one payload byte of the reply to an unknown command


def _chunk_msp_response(cmd: int, payload: bytes, dest: int, origin: int, chunk: int = 20) -> list[bytes]:
    """Split an MSPv2 response into CRSF MSP_RESP frames (start chunk then continuations)."""
    body = bytes([0]) + struct.pack("<HH", cmd, len(payload)) + payload       # flag, cmd, size, payload
    frames, seq, first = [], 0, True
    while body or first:
        part, body = body[:chunk], body[chunk:]
        status = 0x40 | (seq & 0xF) | (0x10 if first else 0)
        frames.append(encode_frame(commands.TYPE_MSP_RESP, bytes([dest, origin, status]) + part, sync=SYNC_FC))
        seq, first = seq + 1, False
    return frames


class DongleEmulator:
    def __init__(self) -> None:
        self.arming_flags = 0                   # MSP_STATUS_EX arming-disable flags the emulator reports
        self.motor_values = [1000, 1000, 1000, 1000]
        self.battery_dv = 255                   # battery in 0.1 V units (25.5 V by default; tests of the hover use 160)
        self.master, self._slave = os.openpty()
        self.path = os.ttyname(self._slave)
        self.parser = FrameParser()
        self.rc_frames = 0
        self.last_channels: Optional[list[int]] = None
        self.pings: list[int] = []
        self.commands: list[bytes] = []
        self.msp_requests: list[int] = []
        self.vtx_writes: list[dict] = []
        self._write_asm = commands.MspResponseAssembler(commands.TYPE_MSP_WRITE)
        self.optrange_enabled = True           # set False to emulate a dead sensor
        self.optrange_frozen = False           # True: send identical payloads (flat-line)
        self.telemetry_enabled = True
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="dongle-emulator", daemon=True)
        self._t = 0

    def start(self) -> "DongleEmulator":
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        os.close(self.master)
        os.close(self._slave)

    def _send(self, data: bytes) -> None:
        try:
            os.write(self.master, data)
        except OSError:
            pass

    def _handle(self, f: Frame) -> None:
        if f.type == TYPE_RC_CHANNELS and len(f.payload) == 22:
            self.rc_frames += 1
            self.last_channels = unpack_channels(f.payload)
        elif f.type == TYPE_DEVICE_PING and len(f.payload) >= 2:
            dest = f.payload[0]
            self.pings.append(dest)
            info = bytes([ADDR_RADIO, dest]) + {ADDR_TX: b"ExpressLRS TX", ADDR_RX: b"ExpressLRS RX",
                                                 ADDR_FC: b"Betaflight"}.get(dest, b"Unknown") + b"\x00" \
                + b"ELRS" + bytes(4) + bytes([3, 5, 3, 0]) + bytes([10, 0])
            self._send(encode_frame(TYPE_DEVICE_INFO, info, sync=SYNC_RADIO))
        elif f.type == commands.TYPE_COMMAND:
            self.commands.append(bytes([f.type]) + f.payload)
        elif f.type == commands.TYPE_MSP_REQ and len(f.payload) >= 8:
            cmd = f.payload[4]
            self.msp_requests.append(cmd)
            reply = BOARD_INFO if cmd == commands.MSP_BOARD_INFO else VTX_CONFIG if cmd == commands.MSP_VTX_CONFIG else b""
            if cmd == 104:
                reply = struct.pack("<8H", *self.motor_values, 0, 0, 0, 0)
            elif cmd == 150:
                reply = bytes(15) + bytes([4]) + bytes(4) + bytes([1]) + struct.pack("<I", self.arming_flags)
            for fr in (_chunk_msp_response(cmd, reply, ADDR_RADIO, ADDR_FC) if self.telemetry_enabled else []):     # a silent dongle answers nothing
                self._send(fr)

    KNOWN_WRITES = {commands.MSP_SET_VTX_CONFIG, 214}             # what this FC handles; the v4.7 sim adds MSP2_DF_POS_AID

    def _handle_write(self, f: Frame) -> None:
        msg = self._write_asm.feed(f)
        if msg and msg["cmd"] == commands.MSP_SET_VTX_CONFIG:
            self.vtx_writes.append(commands.decode_vtx_set_config(msg["payload"]))
        if msg: self._reply_write(msg["cmd"])

    def _reply_write(self, cmd: int) -> None:
        """Betaflight answers every MSP write over CRSF (msp_shared.c processMspPacket): an empty reply when it handled the command, ONE byte (TELEMETRY_MSP_ERROR = 2)
        when it does not know it. The GCS tells v4.7 (has MSP2_DF_POS_AID) from older firmware by exactly this."""
        reply = b"" if cmd in self.KNOWN_WRITES else bytes([WRITE_ERROR])
        for fr in (_chunk_msp_response(cmd, reply, ADDR_RADIO, ADDR_FC) if self.telemetry_enabled else []):
            self._send(fr)

    def _telemetry(self) -> None:
        self._t += 1
        t = self._t
        if t % 5 == 0:     # 10 Hz
            self._send(encode_frame(TYPE_LINK_STATISTICS, bytes([42, 44, 98, 12, 0, 4, 3, 40, 97, 11]), sync=SYNC_RADIO))
            self._send(encode_frame(TYPE_ATTITUDE, struct.pack(">hhh", int(1000 * ((t % 40) - 20) / 20), -200, 15708)))
        if t % 20 == 0:    # 2.5 Hz
            self._send(encode_frame(TYPE_BATTERY, struct.pack(">HH", self.battery_dv, 40) + (120).to_bytes(3, "big") + bytes([88])))
            self._send(encode_frame(TYPE_FLIGHT_MODE, b"ANGL\x00"))
        if t % 20 == 0:
            self._send(encode_frame(TYPE_DRONE_LINKED, b"\x01"))
            self._send(encode_frame(TYPE_FC_UID, struct.pack("<3I", 1, 2, 3)))
            self._send(encode_frame(TYPE_RANGEFINDER, struct.pack(">HH", 150, 900)))
        if t % 10 == 0 and self.optrange_enabled:     # 5 Hz
            dist = 1500 if self.optrange_frozen else 1500 + (t % 50)
            flow = 0 if self.optrange_frozen else (t % 7) - 3
            self._send(encode_frame(TYPE_OPTRANGE, b"\x00\xc8" + struct.pack(">HBBhhBB", dist, 255, 3, flow, -flow, 64, 1)))

    def _run(self) -> None:
        while not self._stop.is_set():
            r, _, _ = select.select([self.master], [], [], 0.02)
            if r:
                try:
                    data = os.read(self.master, 4096)
                except OSError:
                    break
                for f in self.parser.feed(data):
                    self._handle(f)
                    self._handle_write(f)
            if self.telemetry_enabled:
                self._telemetry()


def main() -> None:
    emu = DongleEmulator().start()
    print(f"dongle emulator listening on {emu.path}  (Ctrl-C to stop)", flush=True)
    try:
        while True:
            time.sleep(1.0)
            print(f"rc frames {emu.rc_frames}, pings {len(emu.pings)}, commands {len(emu.commands)}, "
                  f"msp requests {emu.msp_requests}", flush=True)
    except KeyboardInterrupt:
        emu.stop()


if __name__ == "__main__":
    main()
