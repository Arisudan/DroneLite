"""pyserial connection manager: reader thread, MSP request/response, CLI text mode, reboot+reconnect."""
import os
import threading
import time

import serial

from . import msp

PROMPT_QUIET = 0.15


class LinkError(Exception):
    pass


def classify(path):
    """kind of a serial path: betaflight | radio | other. Emulator ptys count as betaflight."""
    low = path.lower()
    if "bfemu" in low:
        return "betaflight"
    if "betaflight" in low or "stm" in low:
        return "betaflight"
    if "cp210" in low or "silicon_labs" in low or "silabs" in low:
        return "radio"
    return "other"


class Link:
    def __init__(self):
        self.ser = None
        self.path = None
        self.baud = 115200
        self.lock = threading.RLock()
        self.cv = threading.Condition()
        self.mode = "msp"
        self.parser = msp.Parser()
        self.resp = {}
        self.clibuf = ""
        self.last_rx = 0.0
        self.lost = False
        self._stop = False
        self._thread = None
        self.timeouts = 0

    # ---- lifecycle
    @property
    def is_open(self):
        return self.ser is not None and not self.lost

    @property
    def kind(self):
        return classify(self.path or "")

    @property
    def packet_errors(self):
        return self.parser.errors + self.timeouts

    def open(self, path, baud=115200):
        self.close()
        try:
            ser = serial.Serial(path, baud, bytesize=8, parity="N", stopbits=1, timeout=0.05, write_timeout=2)
        except (serial.SerialException, OSError) as e:
            raise LinkError("cannot open %s: %s" % (path, e))
        with self.cv:
            self.ser, self.path, self.baud = ser, path, baud
            self.mode, self.lost, self._stop = "msp", False, False
            self.parser, self.resp, self.clibuf = msp.Parser(), {}, ""
        self._thread = threading.Thread(target=self._reader, name="link-reader", daemon=True)
        self._thread.start()

    def close(self):
        self._stop = True
        t, ser = self._thread, self.ser
        if t and t is not threading.current_thread():
            t.join(1.0)
        self._thread = None
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        self.ser = None

    def _reader(self):
        ser = self.ser
        while not self._stop:
            try:
                data = ser.read(max(1, ser.in_waiting))
            except (serial.SerialException, OSError, TypeError, ValueError):
                self.lost = True
                with self.cv:
                    self.cv.notify_all()
                return
            if not data:
                continue
            with self.cv:
                if self.mode == "cli":
                    self.clibuf += data.decode("utf-8", "replace")
                    self.last_rx = time.monotonic()
                else:
                    for cmd, payload, d in self.parser.feed(data):
                        if d in (">", "!"):
                            self.resp[cmd] = (d, payload)
                self.cv.notify_all()

    # ---- MSP
    def msp(self, cmd, payload=b"", timeout=1.0):
        frame = msp.encode_request(cmd, payload)  # allow-list gate
        with self.lock:
            if not self.is_open:
                raise LinkError("not connected")
            if self.mode != "msp":
                raise LinkError("FC is in CLI mode; MSP unavailable")
            with self.cv:
                self.resp.pop(cmd, None)
            try:
                self.ser.write(frame)
            except (serial.SerialException, OSError) as e:
                self.lost = True
                raise LinkError("write failed: %s" % e)
            with self.cv:
                ok = self.cv.wait_for(lambda: cmd in self.resp or self.lost, timeout)
                if self.lost:
                    raise LinkError("port lost")
                if not ok:
                    self.timeouts += 1
                    raise LinkError("timeout waiting for MSP %d" % cmd)
                d, payload = self.resp.pop(cmd)
            if d == "!":
                raise msp.MspError("FC rejected MSP %d" % cmd)
            return payload

    # ---- CLI
    def _wait_prompt(self, timeout):
        end = time.monotonic() + timeout
        with self.cv:
            while True:
                b = self.clibuf
                if (b == "# " or b.endswith("\n# ")) and time.monotonic() - self.last_rx >= PROMPT_QUIET:
                    return True
                if self.lost:
                    raise LinkError("port lost")
                left = end - time.monotonic()
                if left <= 0:
                    return False
                self.cv.wait(min(0.05, left))

    def cli_enter(self, timeout=4.0):
        with self.lock:
            if not self.is_open:
                raise LinkError("not connected")
            with self.cv:
                self.mode, self.clibuf, self.last_rx = "cli", "", time.monotonic()
            self.ser.write(b"#")
            if not self._wait_prompt(timeout):
                self.mode = "msp"
                raise LinkError("FC did not enter CLI")

    def cli_cmd(self, line, timeout=8.0):
        """Send one CLI line, return output (echo and prompt removed)."""
        with self.lock:
            if self.mode != "cli":
                raise LinkError("not in CLI mode")
            with self.cv:
                self.clibuf, self.last_rx = "", time.monotonic()
            self.ser.write(line.encode() + b"\n")
            if not self._wait_prompt(timeout):
                raise LinkError("timeout waiting for CLI prompt after '%s'" % line)
            text = self.clibuf.replace("\r", "")
        text = text[:-2] if text.endswith("# ") else text
        first, _, rest = text.partition("\n")
        if first.strip() == line.strip():
            text = rest
        return text.strip("\n")

    # ---- reboot
    def reboot_reconnect(self, write, timeout=30.0, settle=1.0):
        """Send `write` (CLI text or MSP frame), drop the port, wait for it to return, reopen in MSP mode."""
        path, baud = self.path, self.baud
        with self.lock:
            try:
                self.ser.write(write)
                self.ser.flush()
            except (serial.SerialException, OSError):
                pass
            time.sleep(0.3)
            self.close()
        time.sleep(settle)
        end = time.monotonic() + timeout
        while True:
            if os.path.exists(path):
                try:
                    self.open(path, baud)
                    break
                except LinkError:
                    pass
            if time.monotonic() > end:
                raise LinkError("FC port %s did not return within %.0f s" % (path, timeout))
            time.sleep(0.4)
        while time.monotonic() < end:
            try:
                self.msp(msp.API_VERSION, timeout=0.6)
                return
            except (LinkError, msp.MspError):
                time.sleep(0.3)
        raise LinkError("FC did not answer MSP after reboot")
