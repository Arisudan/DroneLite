import json
import os
import threading
import time
import unittest

from helpers import Rig  # noqa: F401  (also sets sys.path)

from server import msp
from server.app import make_app
from server.cli import CliError, parse_dump, parse_get
from server.controller import Controller
from server.safety import SafetyError


class MspTests(unittest.TestCase):
    def test_roundtrip(self):
        for cmd in (1, 108, 150):
            frame = msp.encode_request(cmd, b"\x01\x02")
            self.assertEqual(frame[:3], b"$M<")
            p = msp.Parser()
            out = p.feed(b"junk" + frame[:4])
            self.assertEqual(out, [])
            out = p.feed(frame[4:])
            self.assertEqual(out, [(cmd, b"\x01\x02", "<")])
        resp = msp.encode_response(108, bytes(6))
        self.assertEqual(msp.Parser().feed(resp), [(108, bytes(6), ">")])

    def test_bad_crc(self):
        f = bytearray(msp.encode_response(1, b"\x00\x01\x2e"))
        f[-1] ^= 0xFF
        p = msp.Parser()
        self.assertEqual(p.feed(bytes(f)), [])
        self.assertEqual(p.errors, 1)

    def test_allow_list(self):
        for cmd in (214, 55, 208, 200):
            with self.assertRaises(msp.AllowListError):
                msp.encode_request(cmd)
        for cmd in (68, 205, 250, 101, 160):
            msp.encode_request(cmd)

    def test_link_msp_cannot_send_forbidden(self):
        r = Rig()
        try:
            with self.assertRaises(msp.AllowListError):
                r.link.msp(214, b"\0" * 16)
            self.assertNotIn(214, r.emu.msp_log)
        finally:
            r.close()


class ParseTests(unittest.TestCase):
    def test_dump_parse(self):
        r = Rig()
        try:
            r.cli.enter()
            d = r.cli.dump_all()
        finally:
            r.close()
        names = {s["name"]: s for s in d["settings"]}
        self.assertGreaterEqual(len(names), 150)
        for n in ("p_roll", "i_roll", "d_roll", "motor_poll", "small_angle", "throttle_boost", "anti_gravity_gain",
                  "vbat_scale", "debug_mode", "roll_rc_rate"):
            self.assertIn(n, names)
        self.assertEqual(names["p_roll"]["value"], "45")
        self.assertEqual(names["p_roll"]["group"], "pid")
        self.assertEqual(names["p_roll"]["section"], "profile")
        self.assertIn("serial 2 262144 115200 57600 0 115200", d["lines"]["serial"])
        self.assertTrue(d["lines"]["aux"])
        self.assertIn("feature RX_SERIAL", d["lines"]["feature"])
        self.assertTrue(d["version_line"].startswith("Betaflight / STM32H743"))
        self.assertEqual((d["profile"], d["rateprofile"]), (0, 0))
        self.assertNotIn("save", sum(d["lines"].values(), []))

    def test_dump_only_active_profile(self):
        t = "profile 0\nset p_roll = 1\nprofile 1\nset p_roll = 2\nprofile 0\nrateprofile 0\n"
        d = parse_dump(t)
        self.assertEqual([s["value"] for s in d["settings"]], ["1"])

    def test_get_parse(self):
        m = parse_get("p_roll = 45\nAllowed range: 0 - 250\nDefault value: 45\n"
                      "debug_mode = NONE\nAllowed values: NONE, CYCLETIME\nDefault value: NONE\n"
                      "ibata_offset = -5\nAllowed range: -32000 - 32000\nDefault value: 0")
        self.assertEqual((m["p_roll"]["min"], m["p_roll"]["max"], m["p_roll"]["default"], m["p_roll"]["type"]),
                         (0, 250, 45, "int"))
        self.assertEqual(m["debug_mode"]["table"], ["NONE", "CYCLETIME"])
        self.assertEqual(m["ibata_offset"]["min"], -32000)


class SafetyTests(unittest.TestCase):
    def setUp(self):
        self.r = Rig()
        self.r.cli.enter()
        self.d = self.r.cli.dump_all()
        self.cur = {s["name"]: s["value"] for s in self.d["settings"]}

    def tearDown(self):
        self.r.close()

    def apply(self, changes, lines=()):
        return self.r.safety.apply(self.r.cli, self.cur, self.d["lines"], changes, list(lines))

    def test_diff_only(self):
        res = self.apply({"p_roll": "45", "i_roll": 85, "d_roll": "40"})
        sets = [l for l in self.r.emu.cli_log if l.startswith("set ")]
        self.assertEqual(sets, ["set i_roll = 85"])
        self.assertEqual([a["name"] for a in res["applied"]], ["i_roll"])
        self.assertTrue(res["applied"][0]["verified"])
        self.assertEqual(res["applied"][0]["old"], "80")
        self.assertEqual(self.r.emu.vals["i_roll"], "85")

    def test_nothing_changed_sends_nothing_and_no_backup(self):
        res = self.apply({"p_roll": "45"})
        self.assertEqual(res["applied"], [])
        self.assertFalse([l for l in self.r.emu.cli_log if l.startswith(("set", "diff"))])
        self.assertFalse(os.path.exists(os.path.join(self.r.tmp, "backups")))

    def test_invalid_value_reported(self):
        res = self.apply({"p_roll": 9999})
        self.assertEqual(res["applied"], [])
        self.assertEqual(res["failed"][0]["name"], "p_roll")
        self.assertEqual(self.r.emu.vals["p_roll"], "45")

    def test_refuses_when_armed(self):
        self.r.safety.armed_fn = lambda: True
        with self.assertRaises(SafetyError):
            self.apply({"i_roll": 90})
        self.assertFalse([l for l in self.r.emu.cli_log if l.startswith("set ")])

    def test_backup_before_first_write(self):
        self.apply({"i_roll": 86})
        log = self.r.emu.cli_log
        self.assertLess(log.index("diff all"), log.index("set i_roll = 86"))
        files = os.listdir(os.path.join(self.r.tmp, "backups"))
        self.assertEqual(len(files), 1)
        self.assertTrue(files[0].endswith("_before-change.txt"))
        with open(os.path.join(self.r.tmp, "backups", files[0])) as f:
            self.assertIn("batch start", f.read())
        self.assertEqual(self.r.emu.cli_log.count("diff all"), 1)
        self.apply({"i_roll": 87})  # second write: no second backup
        self.assertEqual(self.r.emu.cli_log.count("diff all"), 1)
        self.assertEqual(len(os.listdir(os.path.join(self.r.tmp, "backups"))), 1)

    def test_serial_locked(self):
        line = "serial 3 262144 115200 57600 0 115200"
        res = self.apply({}, [line])
        self.assertEqual(res["applied"], [])
        self.assertIn("locked", res["failed"][0]["error"])
        self.assertFalse([l for l in self.r.emu.cli_log if l.startswith("serial 3")])
        self.assertIn("serial 3 0 115200 57600 0 115200", self.r.emu.serial)

    def test_serial_unlocked_and_aux(self):
        self.r.safety.serial_unlocked = True
        res = self.apply({}, ["serial 3 262144 115200 57600 0 115200", "aux 6 4 1 1300 1700 0",
                              "feature -TELEMETRY", "feature RX_SERIAL"])
        names = {a["name"]: a["verified"] for a in res["applied"]}
        self.assertEqual(len(names), 3, res)  # RX_SERIAL already on: skipped
        self.assertTrue(all(names.values()), res)
        self.assertEqual(res["failed"], [])

    def test_cli_guard(self):
        s = self.r.safety
        for bad in ("motor 1 1500", "dfu", "defaults", "bl", "resource MOTOR 1 NONE"):
            with self.assertRaises(SafetyError):
                s.check_cli(bad)
        s.check_cli("get p_roll")
        with self.assertRaises(SafetyError):
            s.check_cli("serial 1 64 115200 57600 0 115200")

    def test_cli_error_class(self):
        with self.assertRaises(CliError):
            self.r.cli.set("nonexistent", 1)


class RebootTests(unittest.TestCase):
    def test_save_reboots_and_reconnects(self):
        r = Rig(reboot_delay=0.8)
        try:
            r.cli.enter()
            r.cli.set("i_roll", 90)
            t0 = time.time()
            r.cli.save()
            self.assertGreater(time.time() - t0, 0.8)  # waited for the port to come back
            self.assertEqual(r.emu.reboot_count, 1)
            self.assertEqual(r.link.mode, "msp")
            self.assertFalse(r.emu.cli)
            self.assertEqual(len(r.link.msp(msp.RC)), 32)
            self.assertEqual(r.emu.saved["vals"]["i_roll"], "90")
        finally:
            r.close()

    def test_exit_discards_unsaved(self):
        r = Rig()
        try:
            r.cli.enter()
            r.cli.set("i_roll", 91)
            r.cli.exit()
            self.assertEqual(r.emu.vals["i_roll"], "80")
        finally:
            r.close()


class WsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import tempfile
        import uvicorn
        from server.emulator import Emulator
        cls.tmp = tempfile.mkdtemp(prefix="bftest")
        cls.emu = Emulator(path=os.path.join(cls.tmp, "ttyBFEMU"), reboot_delay=0.5)
        web = os.path.join(cls.tmp, "web")
        os.makedirs(web)
        with open(os.path.join(web, "index.html"), "w") as f:
            f.write("<title>x</title>")
        cls.ctrl = Controller(os.path.join(cls.tmp, "backups"),
                              extra_ports=[{"path": cls.emu.path, "description": "emu", "kind": "betaflight"}])
        cfg = uvicorn.Config(make_app(cls.ctrl, web_dir=web, emulator=cls.emu), host="127.0.0.1", port=0,
                             log_level="error", ws="auto")
        cls.server = uvicorn.Server(cfg)
        cls.thread = threading.Thread(target=cls.server.run, daemon=True)
        cls.thread.start()
        for _ in range(100):
            if cls.server.started:
                break
            time.sleep(0.05)
        cls.port = cls.server.servers[0].sockets[0].getsockname()[1]

    @classmethod
    def tearDownClass(cls):
        cls.server.should_exit = True
        cls.thread.join(3)
        cls.emu.stop()

    def test_protocol(self):
        from websockets.sync.client import connect
        with connect("ws://127.0.0.1:%d/ws" % self.port) as ws:
            self._protocol(ws)

    def _protocol(self, ws):
        events = []
        n = [0]

        def call(method, **params):
            n[0] += 1
            ws.send(json.dumps({"id": n[0], "method": method, "params": params}))
            while True:
                m = json.loads(ws.recv(timeout=20))
                if "event" in m:
                    events.append(m)
                    continue
                self.assertEqual(m["id"], n[0])
                return m

        try:
            ports = call("ports.list")["result"]
            self.assertEqual(ports[0]["kind"], "betaflight")
            self.assertIn("error", call("settings.dump"))  # not in settings mode yet
            info = call("fc.connect", port=ports[0]["path"])["result"]
            self.assertEqual((info["api"], info["version"], info["mode"], info["armed"]), ("1.46", "4.5.4", "live", False))
            self.assertEqual(info["board"], "MATEKH743")
            end = time.time() + 1.3
            while time.time() < end:
                try:
                    events.append(json.loads(ws.recv(timeout=0.2)))
                except TimeoutError:
                    pass
            live = [e for e in events if e["event"] == "live"]
            st = [e for e in events if e["event"] == "status"]
            self.assertGreaterEqual(len(live), 8)
            self.assertGreaterEqual(len(st), 2)
            self.assertEqual(len(live[0]["data"]["motors"]), 8)
            self.assertIn("attitude", live[0]["data"])
            # armed: settings refused
            self.emu.armed = True
            r = call("mode.settings")
            self.assertIn("ARMED", r["error"])
            self.emu.armed = False
            d = call("mode.settings")["result"]
            self.assertGreaterEqual(len(d["settings"]), 150)
            meta = call("settings.meta", name="debug_mode")["result"]
            self.assertEqual(meta["type"], "table")
            self.assertEqual(call("settings.meta", name="p_roll")["result"]["max"], 250)
            r = call("settings.apply", changes={"p_roll": "50", "i_roll": "80"}, lines=["aux 6 4 1 1300 1700 0"])["result"]
            self.assertEqual([a["name"] for a in r["applied"]], ["p_roll", "aux 6 4 1 1300 1700 0"])
            self.assertTrue(all(a["verified"] for a in r["applied"]))
            self.assertTrue(os.path.exists(r["backup"]))
            self.assertIn("set p_roll", call("settings.diff")["result"]["text"])
            self.assertIn("p_roll", call("settings.export")["result"]["text"])
            r = call("cli.send", line="get p_roll")["result"]
            self.assertIn("p_roll = 50", r["text"])
            self.assertIn("error", call("cli.send", line="motor 1 1100"))
            s = call("settings.save")["result"]
            self.assertTrue(s["saved"])
            self.assertEqual(s["info"]["mode"], "live")
            self.assertEqual(self.emu.saved["vals"]["p_roll"], "50")
            self.assertIn("error", call("cli.send", line="get p_roll"))
            self.assertEqual(call("fc.disconnect")["result"], {"connected": False})
            self.assertIn("error", call("nope"))
        finally:
            pass

    def test_static(self):
        import urllib.request
        body = urllib.request.urlopen("http://127.0.0.1:%d/" % self.port).read()
        self.assertIn(b"<title>", body)
        with self.assertRaises(Exception):
            urllib.request.urlopen("http://127.0.0.1:%d/../server/app.py" % self.port)


if __name__ == "__main__":
    unittest.main()
