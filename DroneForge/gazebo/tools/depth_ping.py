#!/usr/bin/env python3
"""Checks the depth server and the link to it: provider (GPU or CPU), server time, round trip. usage: python3 tools/depth_ping.py http://172.16.100.188:8765"""
import json, sys, time, urllib.request
import cv2, numpy as np
url = (sys.argv[1] if len(sys.argv) > 1 else "http://172.16.100.188:8765").rstrip("/")
try: print("server:", urllib.request.urlopen(url + "/", timeout=10).read().decode())
except Exception as e: print("server not reachable:", e); sys.exit(1)
img = (np.random.rand(480, 640, 3) * 255).astype(np.uint8); ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 85]); rt = []
for i in range(8):
    t = time.time()
    try: o = json.loads(urllib.request.urlopen(urllib.request.Request(url + "/depth?depth=1", jpg.tobytes()), timeout=30).read()); rt.append((time.time() - t) * 1000); print("round trip %5.0f ms, server compute %s ms" % (rt[-1], o["latency_ms"]))
    except Exception as e: print("request failed:", e)
if rt: print("round trip median %.0f ms, worst %.0f ms.  Good = server compute < 60 ms (GPU) and round trip < 300 ms." % (np.median(rt), max(rt)))
