import argparse
import os

import uvicorn

from .app import make_app
from .controller import Controller

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    ap = argparse.ArgumentParser(prog="python3 -m server")
    ap.add_argument("--demo", action="store_true", help="use the built-in fake FC (no hardware)")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8765)
    a = ap.parse_args()
    emu, extra = None, None
    if a.demo:
        from .emulator import Emulator
        emu = Emulator()
        extra = [{"path": emu.path, "description": "Demo FC (emulator, Betaflight 4.5.4)", "kind": "betaflight"}]
        print("demo FC at", emu.path, "| arm/disarm: GET /demo/armed/1 or /demo/armed/0")
    ctrl = Controller(os.path.join(ROOT, "backups"), extra_ports=extra)
    uvicorn.run(make_app(ctrl, emulator=emu), host=a.host, port=a.port, log_level="warning", ws="auto")


if __name__ == "__main__":
    main()
