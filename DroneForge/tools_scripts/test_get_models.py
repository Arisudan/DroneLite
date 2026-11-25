"""get_models.py: manifest sanity + download / resume / checksum / --from, all offline against a local HTTP server. Run: python3 -m pytest tools_scripts/test_get_models.py -q"""
import hashlib, http.server, json, os, re, sys, threading
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import get_models as G

PAYLOAD = os.urandom(300_000)


class _H(http.server.BaseHTTPRequestHandler):
    ranges = []
    def log_message(self, *a): pass
    def do_GET(self):
        if self.path != "/file.bin": self.send_response(404); self.end_headers(); return
        rng = self.headers.get("Range")
        if rng:
            _H.ranges.append(rng); start = int(re.match(r"bytes=(\d+)-", rng).group(1)); body = PAYLOAD[start:]
            self.send_response(206); self.send_header("Content-Range", f"bytes {start}-{len(PAYLOAD) - 1}/{len(PAYLOAD)}")
        else: body = PAYLOAD; self.send_response(200)
        self.send_header("Content-Length", str(len(body))); self.end_headers(); self.wfile.write(body)


def _server():
    s = http.server.ThreadingHTTPServer(("127.0.0.1", 0), _H); threading.Thread(target=s.serve_forever, daemon=True).start(); return s


def _manifest(tmp, url, sha, size=len(PAYLOAD)):
    p = tmp / "m.json"; p.write_text(json.dumps({"root": "models", "models": [{"name": "file.bin", "path": "sub/file.bin", "size": size, "sha256": sha, "url": url, "what": "t"}]})); return str(p)


def test_the_real_manifest_is_complete():
    m = json.load(open(os.path.join(HERE, "models_manifest.json"))); names = [x["name"] for x in m["models"]]
    assert len(names) == len(set(names)) >= 4 and {"da3_metric.onnx", "da3_metric.weights.safetensors", "superpoint.onnx", "superpoint_lightglue.onnx"} <= set(names)
    for x in m["models"]:
        assert x["url"].startswith("https://") and re.fullmatch(r"[0-9a-f]{64}", x["sha256"]) and x["size"] > 0 and x["path"]
    assert m["root"] == "gazebo/vision/models"


def test_download_verifies_and_check_reports(tmp_path):
    srv = _server(); url = f"http://127.0.0.1:{srv.server_port}/file.bin"; mf = _manifest(tmp_path, url, hashlib.sha256(PAYLOAD).hexdigest())
    d = str(tmp_path / "out"); assert G.main(["--manifest", mf, "--dir", d]) == 0
    assert open(os.path.join(d, "sub/file.bin"), "rb").read() == PAYLOAD
    assert G.main(["--manifest", mf, "--dir", d, "--check"]) == 0
    open(os.path.join(d, "sub/file.bin"), "ab").write(b"x")                           # corrupted: --check fails, a normal run repairs it
    assert G.main(["--manifest", mf, "--dir", d, "--check"]) == 1 and G.main(["--manifest", mf, "--dir", d]) == 0
    assert open(os.path.join(d, "sub/file.bin"), "rb").read() == PAYLOAD; srv.shutdown()


def test_an_interrupted_download_resumes(tmp_path):
    srv = _server(); url = f"http://127.0.0.1:{srv.server_port}/file.bin"; mf = _manifest(tmp_path, url, hashlib.sha256(PAYLOAD).hexdigest()); _H.ranges.clear()
    d = tmp_path / "out"; (d / "sub").mkdir(parents=True); (d / "sub/file.bin.part").write_bytes(PAYLOAD[:123_456])
    assert G.main(["--manifest", mf, "--dir", str(d)]) == 0
    assert (d / "sub/file.bin").read_bytes() == PAYLOAD and _H.ranges == ["bytes=123456-"] and not (d / "sub/file.bin.part").exists(); srv.shutdown()


def test_a_wrong_checksum_is_refused_and_leaves_nothing(tmp_path):
    srv = _server(); url = f"http://127.0.0.1:{srv.server_port}/file.bin"; mf = _manifest(tmp_path, url, "0" * 64); d = tmp_path / "out"
    assert G.main(["--manifest", mf, "--dir", str(d)]) == 1 and not (d / "sub/file.bin").exists(); srv.shutdown()


def test_from_a_folder(tmp_path):
    src = tmp_path / "usb"; (src / "sub").mkdir(parents=True); (src / "sub/file.bin").write_bytes(PAYLOAD)
    mf = _manifest(tmp_path, "https://invalid.example/none", hashlib.sha256(PAYLOAD).hexdigest()); d = str(tmp_path / "out")
    assert G.main(["--manifest", mf, "--dir", d, "--from", str(src)]) == 0 and open(os.path.join(d, "sub/file.bin"), "rb").read() == PAYLOAD
