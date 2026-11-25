"""python3 -m nimbus_gcs [--port P] [--connect] [--screenshot out.png --seconds N]"""
import argparse
import os
import sys

# RViz (Video page) is embedded as an X11 window: the GCS itself must run on X11 (XWayland, xcb). Qt 6's xcb plugin needs libxcb-cursor0: without it the
# GCS would not start at all, so xcb is chosen only when the library is there (else Wayland: everything works except the RViz embed). QT_QPA_PLATFORM still wins.
LOCAL_XCB_CURSOR = os.path.expanduser("~/.local/lib/droneforge/libxcb-cursor.so.0")      # Ubuntu's libxcb-cursor0 unpacked without sudo (apt download + dpkg -x)


def _xcb_usable() -> bool:
    import ctypes, ctypes.util
    if not os.environ.get("DISPLAY"): return False
    if ctypes.util.find_library("xcb-cursor") is not None: return True
    if os.path.exists(LOCAL_XCB_CURSOR):
        try:
            ctypes.CDLL(LOCAL_XCB_CURSOR, mode=ctypes.RTLD_GLOBAL); return True             # loaded first: Qt's xcb plugin then finds it by its soname
        except OSError:
            return False
    return False


_qpa = os.environ.get("QT_QPA_PLATFORM", "")
if (not _qpa or _qpa.startswith("wayland")) and _xcb_usable():                 # unset, or the desktop's default ("wayland;xcb" on COSMIC): X11 for the RViz embed
    os.environ["QT_QPA_PLATFORM"] = "xcb"

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QApplication

from .main_window import MainWindow, apply_dark


def main() -> int:
    ap = argparse.ArgumentParser(prog="nimbus_gcs", description="DroneForge ground control station (props off when arming is allowed!)")
    ap.add_argument("--port", help="serial port feeding the dongle's CRSF UART (default: first CP2102)")
    ap.add_argument("--connect", action="store_true", help="connect on start")
    ap.add_argument("--screenshot", metavar="PNG", help="render the window to a PNG after --seconds and exit (also works with QT_QPA_PLATFORM=offscreen)")
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--video", metavar="URL", help="start this video feed on launch (e.g. udp://0.0.0.0:3000)")
    ap.add_argument("--sim", action="store_true", help="connected to the Gazebo simulation (sim.sh): the title and the status line say SIMULATION")
    args = ap.parse_args()
    app = QApplication(sys.argv[:1])
    apply_dark(app)
    win = MainWindow()
    if args.port:
        win.port_box.setCurrentText(args.port)
    win.show()
    if args.sim:
        win.setWindowTitle(win.windowTitle() + "  \u2014  SIMULATION")
    if args.connect:
        QTimer.singleShot(200, win.toggle_connection)
    if args.sim:
        QTimer.singleShot(900, lambda: win.say("SIMULATION: connected to the Gazebo drone over the simulated radio, not the real drone.", "ok"))
    if args.video:
        def _video():
            vp = win.video_page; vp.url.setText(args.video); vp._sync_preset("sim" if args.sim else ""); vp.start_video()
        QTimer.singleShot(600, _video)
    if args.screenshot:
        QTimer.singleShot(int(args.seconds * 1000), lambda: (win.grab().save(args.screenshot), win.close(), app.quit()))
    rc = app.exec()
    win.deleteLater(); app.processEvents(); del win          # the window goes while the application object still exists (the other order crashed Qt at exit, 2026-11-25)
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
