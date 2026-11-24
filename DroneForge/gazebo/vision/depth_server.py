#!/usr/bin/env python3
"""Nimbus depth server for a GPU machine. POST a JPEG to /depth, get the Nimbus 6x8 depth grid as JSON (Depth Anything 3 metric, ONNX Runtime CUDA).
usage: python3 depth_server.py [--port 8765] [--models DIR]   (DIR holds da3_metric.onnx and da3_metric.weights.safetensors)"""
import argparse, base64, json, os, sys, time, threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import numpy as np
import nimbus_depth_core as core
try: import cv2
except Exception: cv2 = None

def main():
    p = argparse.ArgumentParser(); p.add_argument("--port", type=int, default=8765); p.add_argument("--models", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "models", "nimbus")); a = p.parse_args()
    core.MODELS = a.models
    import onnxruntime as ort
    try: ort.preload_dlls()                                  # GPU laptop: loads the CUDA / cuDNN libraries that pip installed (nvidia-*-cu12), so no LD_LIBRARY_PATH is needed
    except Exception: pass
    prov = [("CUDAExecutionProvider", {"arena_extend_strategy": "kSameAsRequested", "gpu_mem_limit": 8 << 30, "cudnn_conv_algo_search": "HEURISTIC"}) if x == "CUDAExecutionProvider" else x for x in ("CUDAExecutionProvider", "CPUExecutionProvider") if x in ort.get_available_providers()]
    da3 = core.Da3(4, providers=prov); print("providers:", da3.s.get_providers(), flush=True)
    img = np.random.randint(0, 255, (360, 640, 3), np.uint8)
    for i in range(3): t = time.time(); da3(img); print("warm-up %d: %.0f ms" % (i, (time.time() - t) * 1000), flush=True)
    lock = threading.Lock()   # one inference at a time (parallel requests exhausted the GPU memory)
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a): pass
        def do_GET(self):
            self.send_response(200); self.end_headers(); self.wfile.write(b"ok " + ",".join(da3.s.get_providers()).encode())
        def do_POST(self):
            n = int(self.headers.get("Content-Length", 0)); buf = np.frombuffer(self.rfile.read(n), np.uint8)
            t = time.time(); rgb = cv2.imdecode(buf, cv2.IMREAD_COLOR)[..., ::-1]
            with lock: d = da3(np.ascontiguousarray(rgb))
            out, *_ = core.result_json(d, time.time() - t)
            if "depth=1" in self.path:                                      # dense depth for RTAB-Map: 16-bit PNG in millimetres at half the model size
                mm = np.clip(cv2.resize(d, (252, 168), interpolation=cv2.INTER_AREA) * 1000.0, 0, 65535).astype(np.uint16)
                out["depth_png"] = base64.b64encode(cv2.imencode(".png", mm)[1].tobytes()).decode()
            body = json.dumps(out).encode(); self.send_response(200); self.send_header("Content-Type", "application/json"); self.send_header("Content-Length", str(len(body))); self.end_headers(); self.wfile.write(body)
    print("listening on 0.0.0.0:%d" % a.port, flush=True); ThreadingHTTPServer(("0.0.0.0", a.port), H).serve_forever()

if __name__ == "__main__": main()
