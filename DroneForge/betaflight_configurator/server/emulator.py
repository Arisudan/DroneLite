"""A fake Betaflight 4.5.4 FC on a pty. MSP v1 reads + CLI. `save`/`exit` reboot (port path vanishes ~2 s)."""
import math
import os
import pty
import select
import struct
import tempfile
import termios
import threading
import time
import tty

from . import msp

VERSION_LINE = "# Betaflight / STM32H743 (SH74) 4.5.4 Nov 10 2026 / 15:42:40 (a1b2c3d) MSP API: 1.46"
CONFIG_LINE = "# config: manufacturer_id: MTKS, board_name: MATEKH743, version: 7e5a1b0c, date: 2026-11-10T09:00:00Z"

MASTER_NUM = """
gyro_lpf1_static_hz 0 0 1000
gyro_lpf2_static_hz 500 0 1000
gyro_lpf1_dyn_min_hz 0 0 1000
gyro_lpf1_dyn_max_hz 0 0 1000
gyro_notch1_hz 0 0 20000
gyro_notch1_cutoff 0 0 20000
gyro_notch2_hz 0 0 20000
gyro_notch2_cutoff 0 0 20000
gyro_rpm_filter_harmonics 3 0 3
gyro_rpm_filter_min_hz 100 50 200
gyro_rpm_filter_q 500 250 3000
dyn_notch_count 3 0 5
dyn_notch_q 300 1 1000
dyn_notch_min_hz 100 20 250
dyn_notch_max_hz 600 200 1000
dterm_lpf1_static_hz 75 0 1000
dterm_lpf2_static_hz 150 0 1000
dterm_lpf1_dyn_min_hz 75 0 1000
dterm_lpf1_dyn_max_hz 150 0 1000
dterm_notch_hz 0 0 20000
dterm_notch_cutoff 0 0 20000
yaw_lowpass_hz 100 0 500
pid_process_denom 1 1 16
motor_idle 550 0 2000
dshot_idle_value 550 0 2000
motor_poles 14 4 255
motor_output_limit 100 1 100
dyn_idle_min_rpm 0 0 200
dyn_idle_p_gain 50 1 250
dyn_idle_i_gain 50 1 250
dyn_idle_d_gain 50 0 250
dyn_idle_max_increase 150 10 255
small_angle 25 0 180
vbat_scale 110 1 65535
vbat_max_cell_voltage 430 100 500
vbat_full_cell_voltage 410 100 500
vbat_min_cell_voltage 330 100 500
vbat_warning_cell_voltage 350 100 500
vbat_hysteresis 10 0 250
vbat_detect_cell_voltage 300 0 2000
ibata_scale 400 -16000 16000
ibata_offset 0 -32000 32000
battery_capacity 0 0 20000
batt_ref_v 1680 0 6000
failsafe_delay 15 0 200
failsafe_off_delay 10 0 200
failsafe_throttle 1000 1000 2000
failsafe_throttle_low_delay 100 0 300
failsafe_recovery_delay 5 1 200
rx_min_usec 885 750 2250
rx_max_usec 2115 750 2250
mid_rc 1500 1200 1700
min_check 1050 750 2250
max_check 1900 750 2250
deadband 0 0 32
yaw_deadband 0 0 100
rssi_channel 0 0 18
rssi_scale 100 1 255
rssi_offset 0 -100 100
airmode_start_throttle_percent 25 0 100
rc_smoothing_auto_factor 30 0 250
rc_smoothing_auto_factor_throttle 30 0 250
beeper_dshot_beacon_tone 1 1 5
blackbox_high_resolution 0 0 1
osd_rssi_alarm 20 0 100
osd_cap_alarm 2200 0 20000
osd_alt_alarm 100 0 10000
osd_warn_bitmask 511 0 65535
vtx_band 0 0 5
vtx_channel 0 0 8
vtx_power 0 0 7
vtx_freq 0 0 5999
"""
MASTER_LOOK = """
debug_mode NONE NONE|CYCLETIME|BATTERY|GYRO_FILTERED|ACCELEROMETER|PIDLOOP|GYRO_SCALED|RC_INTERPOLATION|ANGLERATE|ESC_SENSOR|SCHEDULER|STACK|ALTITUDE|FFT|FFT_TIME|FFT_FREQ|GYRO_RAW|RANGEFINDER|RANGEFINDER_QUALITY|CURRENT_SENSOR|ITERM_RELAX|RC_SMOOTHING|ANTI_GRAVITY|DYN_LPF|DSHOT_RPM_TELEMETRY|RPM_FILTER|D_MIN
motor_pwm_protocol DSHOT600 PWM|ONESHOT125|ONESHOT42|MULTISHOT|BRUSHED|DSHOT150|DSHOT300|DSHOT600|PROSHOT1000|DISABLED
motor_poll ON OFF|ON
dshot_bidir ON OFF|ON
gyro_lpf1_type PT1 PT1|BIQUAD|PT2|PT3
dterm_lpf1_type PT1 PT1|BIQUAD|PT2|PT3
dterm_lpf2_type PT1 PT1|BIQUAD|PT2|PT3
rc_smoothing ON OFF|ON
rc_smoothing_type FILTER INTERPOLATION|FILTER
serialrx_provider CRSF SPEK1024|SPEK2048|SBUS|SUMD|SUMH|XB-B|XB-B-RJ01|IBUS|JETIEXBUS|CRSF|SRXL|CUSTOM|FPORT|SRXL2|GHST
failsafe_procedure AUTO-LAND DROP|AUTO-LAND|GPS-RESCUE
acc_hardware AUTO AUTO|NONE|ADXL345|MPU6050|MMA8452|BMA280|LSM303DLHC|MPU6000|MPU6500|FAKE
baro_hardware AUTO AUTO|NONE|BMP085|MS5611|BMP280|LPS|QMP6988|BMP388|DPS310|2SMPB_02B|LPS22DF
mag_hardware NONE AUTO|NONE|HMC5883|AK8975|AK8963|QMC5883|LIS3MDL|MPU925X_AK8963
current_meter ADC NONE|ADC|VIRTUAL|ESC|MSP
battery_meter ADC NONE|ADC|ESC
blackbox_device SPIFLASH NONE|SPIFLASH|SDCARD|SERIAL
blackbox_mode NORMAL NORMAL|MOTOR_TEST|ALWAYS
blackbox_sample_rate 1/2 1/1|1/2|1/4|1/8|1/16
osd_units METRIC IMPERIAL|METRIC
"""
OSD = ("rssi vbat avg_cell_voltage crosshairs ah_sbar ah_pitch_pos flymode craft_name throttle_pos vtx_channel current_draw "
       "mah_drawn gps_speed gps_sats altitude roll_pids pitch_pids yaw_pids power pidrate_profile warnings avg_cell_voltage "
       "pit_ang_pidrate battery_warning disarmed nheading nvario esc_tmp esc_rpm rtc_time g_force stat_max_spd flip_arrow "
       "link_quality tim_1 tim_2 remaining_time_estimate rcchannels").split()
PROFILE_NUM = """
p_roll 45 0 250
i_roll 80 0 250
d_roll 40 0 250
f_roll 120 0 1000
p_pitch 47 0 250
i_pitch 84 0 250
d_pitch 46 0 250
f_pitch 125 0 1000
p_yaw 45 0 250
i_yaw 80 0 250
d_yaw 0 0 250
f_yaw 120 0 1000
d_min_roll 30 0 250
d_min_pitch 34 0 250
d_min_yaw 0 0 250
anti_gravity_gain 80 1 250
anti_gravity_cutoff_hz 5 2 50
anti_gravity_p_gain 100 0 250
throttle_boost 5 0 100
throttle_boost_cutoff 15 5 50
feedforward_transition 0 0 100
feedforward_smooth_factor 25 0 75
feedforward_jitter_factor 7 0 20
feedforward_boost 15 0 50
iterm_windup 85 30 100
iterm_relax_cutoff 15 1 100
abs_control_gain 0 0 20
tpa_rate 65 0 100
tpa_breakpoint 1350 750 2250
tpa_low_rate 20 0 100
tpa_low_breakpoint 1050 750 2250
level_angle_limit 55 10 90
level_strength 40 0 255
horizon_limit 35 0 125
simplified_master_multiplier 100 10 200
simplified_i_gain 100 0 200
simplified_d_gain 100 0 200
simplified_pi_gain 100 0 200
simplified_dmax_gain 100 0 200
simplified_feedforward_gain 100 0 200
thrust_linear 0 0 150
"""
PROFILE_LOOK = """
iterm_relax RP OFF|RP|RPY|RP_INC|RPY_INC
iterm_relax_type SETPOINT GYRO|SETPOINT
tpa_mode D PD|D
simplified_pids_mode RPY OFF|RP|RPY
level_race_mode OFF OFF|ON
feedforward_averaging OFF OFF|2_POINT|3_POINT|4_POINT
"""
RATE_NUM = """
roll_rc_rate 7 1 255
pitch_rc_rate 7 1 255
yaw_rc_rate 7 1 255
roll_expo 0 0 100
pitch_expo 0 0 100
yaw_expo 0 0 100
roll_srate 67 0 100
pitch_srate 67 0 100
yaw_srate 67 0 100
thr_mid 50 0 100
thr_expo 0 0 100
throttle_limit_percent 100 25 100
roll_rate_limit 1998 200 1998
pitch_rate_limit 1998 200 1998
yaw_rate_limit 1998 200 1998
"""
RATE_LOOK = """
rates_type ACTUAL BETAFLIGHT|RACEFLIGHT|KISS|ACTUAL|QUICK
throttle_limit_type OFF OFF|SCALE|CLIP
"""
DEFAULT_FEATURES = ["RX_SERIAL", "AIRMODE", "OSD", "TELEMETRY"]
ALL_FEATURES = ["RX_PPM", "INFLIGHT_ACC_CAL", "RX_SERIAL", "MOTOR_STOP", "SOFTSERIAL", "GPS", "RANGEFINDER",
                "TELEMETRY", "LED_STRIP", "OSD", "AIRMODE", "ESC_SENSOR", "DYNAMIC_FILTER"]
DEFAULT_SERIAL = ["serial 20 1 115200 57600 0 115200", "serial 0 64 115200 57600 0 115200",
                  "serial 1 0 115200 57600 0 115200", "serial 2 262144 115200 57600 0 115200",
                  "serial 3 0 115200 57600 0 115200", "serial 4 0 115200 57600 0 115200",
                  "serial 5 0 115200 57600 0 115200", "serial 6 0 115200 57600 0 115200"]
DEFAULT_AUX = ["aux 0 0 0 1700 2100 0 0", "aux 1 1 1 1300 1700 0 0", "aux 2 2 1 1700 2100 0 0",
               "aux 3 13 2 1700 2100 0 0", "aux 4 35 2 1300 1700 0 0", "aux 5 0 0 900 900 0 0",
               "aux 6 0 0 900 900 0 0", "aux 7 0 0 900 900 0 0"]
RESOURCES = ["resource BEEPER 1 D03", "resource MOTOR 1 B00", "resource MOTOR 2 B01",
             "resource MOTOR 3 A03", "resource MOTOR 4 A02", "resource SERIAL_TX 3 D08"]
OTHER = ["board_name MATEKH743", "manufacturer_id MTKS"]


def _num(block, sec):
    out = []
    for l in block.strip().splitlines():
        n, d, lo, hi = l.split()
        out.append((n, d, ("num", float(lo), float(hi)), sec))
    return out


def _look(block, sec):
    out = []
    for l in block.strip().splitlines():
        n, d, t = l.split()
        out.append((n, d, ("table", t.split("|")), sec))
    return out


def build_spec():
    spec = _num(MASTER_NUM, "master") + _look(MASTER_LOOK, "master")
    spec.append(("name", "-", ("str",), "master"))
    for o in dict.fromkeys(OSD):
        spec.append(("osd_%s_pos" % o, "0", ("num", 0.0, 65535.0), "master"))
    spec += _num(PROFILE_NUM, "profile") + _look(PROFILE_LOOK, "profile")
    spec += _num(RATE_NUM, "rateprofile") + _look(RATE_LOOK, "rateprofile")
    return spec


SPEC = build_spec()


class Emulator:
    def __init__(self, path=None, reboot_delay=2.0):
        self.master, self._slave = pty.openpty()
        tty.setraw(self._slave)
        if path is None:
            path = os.path.join(tempfile.mkdtemp(prefix="bfemu"), "ttyBFEMU")
        self.path = path
        self.reboot_delay = reboot_delay
        self.armed = False
        self.reboot_count = 0
        self.cli_log = []   # every CLI line received, in order
        self.msp_log = []   # every MSP command received
        self.meta = {n: (d, k, s) for n, d, k, s in SPEC}
        self.order = [n for n, _, _, _ in SPEC]
        defaults = {n: d for n, d, _, _ in SPEC}
        self.defaults = dict(defaults)
        self.saved = {"vals": dict(defaults), "serial": list(DEFAULT_SERIAL), "aux": list(DEFAULT_AUX),
                      "feat": list(DEFAULT_FEATURES)}
        self._load_saved()
        self.t0 = time.time()
        self.cli = False
        self._rebooting = False
        self._stop = False
        self._wlock = threading.Lock()
        self._mbuf = b""
        self._cbuf = ""
        self._last = ""
        self._link()
        self._thread = threading.Thread(target=self._run, name="emulator", daemon=True)
        self._thread.start()

    # ---- state
    def _load_saved(self):
        self.vals = dict(self.saved["vals"])
        self.serial = list(self.saved["serial"])
        self.aux = list(self.saved["aux"])
        self.feat = list(self.saved["feat"])

    def _commit(self):
        self.saved = {"vals": dict(self.vals), "serial": list(self.serial), "aux": list(self.aux), "feat": list(self.feat)}

    def _link(self):
        try:
            os.unlink(self.path)
        except OSError:
            pass
        os.symlink(os.ttyname(self._slave), self.path)

    def stop(self):
        self._stop = True
        self._thread.join(1.0)
        for fd in (self.master, self._slave):
            try:
                os.close(fd)
            except OSError:
                pass
        try:
            os.unlink(self.path)
        except OSError:
            pass

    # ---- io
    def _write(self, data):
        if isinstance(data, str):
            data = data.encode()
        with self._wlock:
            try:
                os.write(self.master, data)
            except OSError:
                pass

    def _run(self):
        while not self._stop:
            if self._rebooting:
                time.sleep(0.02)
                continue
            try:
                r, _, _ = select.select([self.master], [], [], 0.02)
                if r:
                    self._rx(os.read(self.master, 4096))
            except (OSError, ValueError):
                if self._stop:
                    return
                time.sleep(0.02)

    def _rx(self, data):
        if self.cli:
            for ch in data.decode("latin1"):
                self._cli_char(ch)
            return
        self._mbuf += data
        while self._mbuf and not self.cli:
            if self._mbuf[0:1] == b"#":
                self._mbuf = self._mbuf[1:]
                self.cli, self._cbuf, self._last = True, "", ""
                self._write("\r\nEntering CLI Mode, type 'exit' to return, or 'help'\r\n\r\n# ")
                rest, self._mbuf = self._mbuf, b""
                if rest:
                    self._rx(rest)
                return
            i = self._mbuf.find(b"$M<")
            if i < 0:
                keep = b"$M"[: 2] if self._mbuf.endswith(b"$M") else (b"$" if self._mbuf.endswith(b"$") else b"")
                j = self._mbuf.find(b"#")
                if j >= 0:
                    self._mbuf = self._mbuf[j:]
                    continue
                self._mbuf = keep
                return
            if i > 0:
                j = self._mbuf.find(b"#")
                if 0 <= j < i:
                    self._mbuf = self._mbuf[j:]
                    continue
                self._mbuf = self._mbuf[i:]
            if len(self._mbuf) < 6:
                return
            n = self._mbuf[3]
            if len(self._mbuf) < 6 + n:
                return
            cmd, payload = self._mbuf[4], self._mbuf[5:5 + n]
            ok = self._mbuf[5 + n] == (n ^ cmd ^ _xor(payload))
            self._mbuf = self._mbuf[6 + n:]
            if ok:
                self._msp(cmd, payload)

    # ---- MSP
    def _msp(self, cmd, payload):
        self.msp_log.append(cmd)
        t = time.time() - self.t0
        out = None
        if cmd == 1:
            out = bytes([0, 1, 46])
        elif cmd == 2:
            out = b"BTFL"
        elif cmd == 3:
            out = bytes([4, 5, 4])
        elif cmd == 4:
            name = b"MATEKH743"
            out = b"M7H7" + struct.pack("<HBB", 0, 0, 0) + bytes([len(name)]) + name
        elif cmd == 5:
            out = b"Nov 10 2026" + b"15:42:40" + b"a1b2c3d"
        elif cmd in (101, 150):
            flags = 1 if self.armed else 0
            out = struct.pack("<HHHIBH", 125, 0, 0x31, flags, 0, 7 + int(2 * math.sin(t))) + struct.pack("<HBB", 125, 0, 0)
            if cmd == 101:
                out = struct.pack("<HHHIB", 125, 0, 0x31, flags, 0) + struct.pack("<H", 7)
        elif cmd == 102:
            acc = [int(20 * math.sin(t)), int(20 * math.cos(t)), 512]
            gyro = [int(40 * math.sin(t * 2)), int(30 * math.cos(t * 2)), int(10 * math.sin(t))]
            out = struct.pack("<9h", *acc, *gyro, 0, 0, 0)
        elif cmd == 104:
            base = 1150 + int(60 * (1 + math.sin(t))) if self.armed else 1000
            out = struct.pack("<8H", *[base + 15 * i for i in range(4)], 0, 0, 0, 0)
        elif cmd == 105:
            ch = [1500 + int(50 * math.sin(t + i)) for i in range(4)]
            ch[2] = 1300 if self.armed else 1000
            aux = [2000 if self.armed else 1000, 1500, 1000, 1000]
            out = struct.pack("<16H", ch[0], ch[1], ch[2], ch[3], *aux, *([1500] * 8))
        elif cmd == 108:
            out = struct.pack("<hhh", int(150 * math.sin(t * 0.7)), int(100 * math.cos(t * 0.5)), int(t * 10) % 360)
        elif cmd == 110:
            out = struct.pack("<BHHh", 168, 123, 800, 15 if self.armed else 2)
        elif cmd == 160:
            out = bytes(range(0xA0, 0xAC))
        elif cmd == 151:
            out = bytes([1, 1, 0, 0, 0, 0, 1])
        elif cmd in (205, 250):
            out = b""
        elif cmd == 68:
            self._write(msp.encode_response(cmd, b""))
            self._reboot()
            return
        if out is None:
            self._write(msp.encode_response(cmd, b"", error=True))
        else:
            self._write(msp.encode_response(cmd, out))

    # ---- CLI
    def _cli_char(self, ch):
        if ch in "\r\n":
            if ch == "\n" and self._last == "\r":
                self._last = ch
                return
            self._last = ch
            line, self._cbuf = self._cbuf, ""
            self._write("\r\n")
            self._cli_line(line.strip())
        elif ch in "\x08\x7f":
            if self._cbuf:
                self._cbuf = self._cbuf[:-1]
                self._write("\x08 \x08")
            self._last = ch
        else:
            self._cbuf += ch
            self._last = ch
            self._write(ch)

    def _cli_line(self, line):
        if line:
            self.cli_log.append(line)
        parts = line.split()
        w = parts[0] if parts else ""
        out = []
        reboot = None
        try:
            if not w:
                pass
            elif w == "version":
                out = [VERSION_LINE]
            elif w in ("dump", "diff"):
                out = self._dump(diff=(w == "diff"))
            elif w == "get":
                out = self._get(parts[1] if len(parts) > 1 else "")
            elif w == "set":
                out = self._set(line)
            elif w == "serial":
                out = self._lines_cmd("serial", self.serial, parts, 2)
            elif w == "aux":
                out = self._lines_cmd("aux", self.aux, parts, 1)
            elif w == "feature":
                out = self._feature(parts[1:])
            elif w == "status":
                out = ["System Uptime: %d seconds" % int(time.time() - self.t0),
                       "Arming disable flags: CLI", "I2C Errors: 0, config size: 4210"]
            elif w == "help":
                out = ["Available commands:", "  aux, diff, dump, exit, feature, get, help, save, serial, set, status, version"]
            elif w == "save":
                self._commit()
                self._write("Saving\r\n")
                self._reboot()
                return
            elif w == "exit":
                self._write("\r\nLeaving CLI mode, unsaved changes lost.\r\n")
                self._reboot()
                return
            else:
                out = ["###ERROR IN CLI### UNKNOWN COMMAND, TRY 'HELP'"]
        except Exception as e:  # keep the fake FC alive
            out = ["###ERROR### %s" % e]
        self._write("".join(l + "\r\n" for l in out) + "\r\n# ")

    def _fmt(self, v):
        return str(v)

    def _get(self, pat):
        names = [n for n in sorted(self.vals) if pat.lower() in n.lower()]
        if not names:
            return ["Invalid name"]
        out = []
        for n in names:
            d, k, _ = self.meta[n]
            out.append("%s = %s" % (n, self.vals[n]))
            if k[0] == "num":
                out.append("Allowed range: %d - %d" % (k[1], k[2]))
            elif k[0] == "table":
                out.append("Allowed values: " + ", ".join(k[1]))
            out.append("Default value: %s" % d)
        return out

    def _set(self, line):
        body = line[3:].strip()
        if "=" not in body:
            return ["Invalid value"] if body else ["Missing = in set"]
        name, _, val = body.partition("=")
        name, val = name.strip(), val.strip()
        if name not in self.meta:
            return ["Invalid name"]
        d, k, _ = self.meta[name]
        if k[0] == "num":
            try:
                f = float(val)
            except ValueError:
                return ["Invalid value"]
            if not (k[1] <= f <= k[2]) or f != int(f):
                return ["Invalid value"]
            val = str(int(f))
        elif k[0] == "table":
            if val.upper() not in [t.upper() for t in k[1]]:
                return ["Invalid value"]
            val = [t for t in k[1] if t.upper() == val.upper()][0]
        self.vals[name] = val
        return ["%s set to %s" % (name, val)]

    def _lines_cmd(self, word, store, parts, key_idx):
        if len(parts) == 1:
            return list(store)
        if word == "serial" and len(parts) != 7:
            return ["Invalid arguments"]
        if word == "aux":
            if len(parts) == 7:
                parts = parts + ["0"]
            if len(parts) != 8 or not all(p.lstrip("-").isdigit() for p in parts[1:]):
                return ["Invalid arguments"]
        key = parts[1]
        new = " ".join(parts)
        for i, l in enumerate(store):
            if l.split()[1] == key:
                store[i] = new
                break
        else:
            store.append(new)
        return [new]

    def _feature(self, args):
        if not args:
            return ["Enabled: " + " ".join(self.feat)]
        out = []
        for a in args:
            minus = a.startswith("-")
            f = a.lstrip("-").upper()
            if f not in ALL_FEATURES:
                out.append("Invalid name")
            elif minus:
                self.feat = [x for x in self.feat if x != f]
                out.append("Disabled %s" % f)
            else:
                if f not in self.feat:
                    self.feat.append(f)
                out.append("Enabled %s" % f)
        return out

    def _dump(self, diff=False):
        o = [VERSION_LINE, CONFIG_LINE, "", "# start the command batch", "batch start", ""]
        if not diff:
            o += ["# reset configuration to default settings", "defaults nosave", ""] + OTHER + [""]
            o += ["# resources"] + RESOURCES + [""]
        else:
            o += ["# reset configuration to default settings", "defaults nosave", ""]
        o.append("# feature")
        for f in ALL_FEATURES:
            on, dflt = f in self.feat, f in DEFAULT_FEATURES
            if on and not (diff and dflt):
                o.append("feature " + f)
            elif not on and dflt:
                o.append("feature -" + f)
        o += ["", "# serial"] + [l for l in self.serial if not diff or l not in DEFAULT_SERIAL]
        o += ["", "# aux"] + [l for l in self.aux if not diff or l not in DEFAULT_AUX]
        for sec, head in (("master", ["", "# master"]), ("profile", ["", "profile 0", "", "# profile 0"]),
                          ("rateprofile", ["", "rateprofile 0", "", "# rateprofile 0"])):
            sets = ["set %s = %s" % (n, self.vals[n]) for n in self.order
                    if self.meta[n][2] == sec and (not diff or self.vals[n] != self.defaults[n])]
            if sets or not diff:
                o += head + sets
        o += ["", "# restore original profile selection", "profile 0", "rateprofile 0", "", "batch end", "",
              "# save configuration", "save"]
        return o

    # ---- reboot
    def _reboot(self):
        self._rebooting = True
        self.cli = False
        threading.Thread(target=self._reboot_thread, daemon=True).start()

    def _reboot_thread(self):
        time.sleep(0.05)
        try:
            os.unlink(self.path)
        except OSError:
            pass
        time.sleep(self.reboot_delay)
        self._load_saved()
        self.reboot_count += 1
        self._mbuf, self._cbuf = b"", ""
        try:
            termios.tcflush(self.master, termios.TCIOFLUSH)
        except Exception:
            pass
        self._link()
        self._rebooting = False


def _xor(b):
    c = 0
    for x in b:
        c ^= x
    return c
