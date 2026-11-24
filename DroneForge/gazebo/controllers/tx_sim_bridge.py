#!/usr/bin/env python3
"""DroneForge TX & Hardware-in-Loop Bridge for Gazebo Simulation.
Bridges the physical ELRS TX dongle (/dev/ttyUSB0) and Matek H743 FC (/dev/ttyACM*)
with the Gazebo Harmonic simulation.

Features:
  1. Opens the ELRS TX dongle (/dev/ttyUSB0) and sends 100 Hz CRSF frames (AETR + AUX).
  2. Subscribes to /droneforge/fc/telemetry from Gazebo.
  3. Publishes commands to /droneforge/fc/command (arm, takeoff, pos_hold, land, disarm).
  4. Encodes and displays simulated telemetry matching CRSF standards.
  5. If physical FC is connected via USB (/dev/ttyACM*), monitors MSP status and RC channels safely.
"""

import argparse
import glob
import json
import math
import os
import select
import struct
import sys
import termios
import threading
import time

# Add PC software directory to python path
HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(HERE, "../.."))
PC_PATH = os.path.join(PROJECT_ROOT, "software/pc")
if PC_PATH not in sys.path:
    sys.path.insert(0, PC_PATH)

from nimbus_pc.crsf import (
    ADDR_FC, ADDR_RADIO, ADDR_RX, ADDR_TX,
    CRSF_MID, CRSF_MIN, CRSF_MAX,
    Frame, FrameParser, encode_frame, encode_rc_frame, encode_device_ping,
    TYPE_BATTERY, TYPE_ATTITUDE, TYPE_FLIGHT_MODE, TYPE_OPTRANGE, TYPE_LINK_STATISTICS
)
from nimbus_pc.mapper import height_to_aux5, mode_to_aux7, stick_to_crsf, throttle01_to_crsf, MODE_POS_TICKS, MODE_ALT_TICKS
from nimbus_pc.sender import Controls, SerialLink, CrsfSender
from nimbus_pc.session import Session

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MspMonitor:
    """Non-blocking MSP poller for physical Flight Controller on USB."""
    def __init__(self, port):
        self.port = port
        self.fd = None
        self.connected = False
        self.board_name = ""
        self.fc_version = ""
        self.last_rc = []
        self.arming_flags = 0
        self.try_open()

    def try_open(self):
        if not self.port or not os.path.exists(self.port):
            return False
        try:
            self.fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            a = termios.tcgetattr(self.fd)
            a[0] = a[1] = a[3] = 0
            a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            a[4] = a[5] = termios.B115200
            a[6][termios.VMIN], a[6][termios.VTIME] = 0, 0
            termios.tcsetattr(self.fd, termios.TCSANOW, a)
            termios.tcflush(self.fd, termios.TCIOFLUSH)
            self.connected = True
            self.query_board_info()
            return True
        except Exception:
            self.connected = False
            return False

    def query_board_info(self):
        if not self.connected or self.fd is None:
            return
        # MSP_BOARD_INFO (4)
        msg = b"$M<\x00\x04\x04"
        try:
            os.write(self.fd, msg)
        except Exception:
            pass

    def poll(self):
        if not self.connected or self.fd is None:
            return
        # Query MSP_STATUS (101) and MSP_RC (105)
        try:
            os.write(self.fd, b"$M<\x00\x65\x65$M<\x00\x69\x69")
            r, _, _ = select.select([self.fd], [], [], 0.01)
            if r:
                buf = os.read(self.fd, 512)
                i = 0
                while i < len(buf):
                    idx = buf.find(b"$M>", i)
                    if idx < 0 or len(buf) < idx + 6:
                        break
                    plen = buf[idx + 3]
                    cmd = buf[idx + 4]
                    if len(buf) >= idx + 6 + plen:
                        payload = buf[idx + 5:idx + 5 + plen]
                        if cmd == 101 and len(payload) >= 10:
                            # MSP_STATUS
                            flags = struct.unpack("<I", payload[6:10])[0] if len(payload) >= 10 else 0
                            self.arming_flags = flags
                        elif cmd == 105 and len(payload) >= 8:
                            # MSP_RC
                            chans = struct.unpack(f"<{len(payload)//2}H", payload)
                            self.last_rc = list(chans[:8])
                        elif cmd == 4 and len(payload) >= 4:
                            self.board_name = payload.decode(errors="replace").strip("\x00")
                        i = idx + 6 + plen
                    else:
                        break
        except Exception:
            pass

    def close(self):
        if self.fd is not None:
            try:
                os.close(self.fd)
            except Exception:
                pass
            self.fd = None
            self.connected = False


class TxSimBridgeNode(Node):
    def __init__(self, tx_port="/dev/ttyUSB0", fc_port=None):
        super().__init__("droneforge_tx_bridge")
        self.tx_port = tx_port
        self.fc_port = fc_port

        # CRSF Session for the TX dongle
        self.session = None
        self.controls = Controls()
        self.allow_arm = False
        self.init_tx()

        # Physical FC MSP monitor (if present)
        if not self.fc_port:
            candidates = sorted(glob.glob("/dev/ttyACM*"))
            if candidates:
                self.fc_port = candidates[0]
        self.msp = MspMonitor(self.fc_port) if self.fc_port else None
        if self.msp and self.msp.connected:
            self.get_logger().info(f"Physical FC detected on {self.fc_port}")

        # Simulated Telemetry State
        self.sim_mode = "DISARMED"
        self.sim_z = 0.0
        self.sim_vz = 0.0
        self.sim_vx = 0.0
        self.sim_vy = 0.0
        self.sim_roll = 0.0
        self.sim_pitch = 0.0
        self.sim_yaw = 0.0
        self.sim_vbat = 16.2
        self.sim_range = 0.0

        # ROS 2 Subscriptions & Publishers
        self.sub_telem = self.create_subscription(
            String, "/droneforge/fc/telemetry", self.on_sim_telemetry, 10
        )
        self.pub_cmd = self.create_publisher(
            String, "/droneforge/fc/command", 10
        )

        # 100 Hz Bridge Loop (CRSF update and MSP poll)
        self.timer = self.create_timer(0.01, self.bridge_loop)

        # Periodic status banner (2 Hz)
        self.status_timer = self.create_timer(0.5, self.print_status)

        self.get_logger().info(f"TX-to-Simulation Bridge Active on {self.tx_port}")
        self.start_terminal_listener()

    def init_tx(self):
        try:
            self.session = Session(self.tx_port, hz=100.0, allow_arm=True, ping=True)
            self.session.start()
            self.session.set_controls(self.controls)
            self.get_logger().info(f"Connected to ELRS TX dongle on {self.tx_port}")
        except Exception as e:
            self.get_logger().warn(f"Could not open TX port {self.tx_port}: {e}. Running in simulation-only mode.")
            self.session = None

    def on_sim_telemetry(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.sim_mode = data.get("mode", "DISARMED")
            self.sim_z = float(data.get("z", 0.0))
            self.sim_vz = float(data.get("vz", 0.0))
            self.sim_vx = float(data.get("vx", 0.0))
            self.sim_vy = float(data.get("vy", 0.0))
            self.sim_roll = float(data.get("roll", 0.0))
            self.sim_pitch = float(data.get("pitch", 0.0))
            self.sim_yaw = float(data.get("yaw", 0.0))
            self.sim_vbat = float(data.get("vbat", 16.2))
            self.sim_range = float(data.get("range", 0.0))
        except Exception:
            pass

    def bridge_loop(self):
        # 1. Update Controls on CRSF link
        if self.session is not None:
            # Map current simulation flight mode and height to AUX channels
            if self.sim_mode in ("TAKEOFF", "ALT_HOLD", "POS_HOLD"):
                self.controls.arm = True
                self.controls.aux5 = height_to_aux5(0.50)
                self.controls.aux7 = mode_to_aux7("pos_hold" if self.sim_mode == "POS_HOLD" else "alt_hold")
            else:
                self.controls.arm = False
                self.controls.aux5 = CRSF_MIN
                self.controls.aux7 = MODE_POS_TICKS

            self.session.set_controls(self.controls)

        # 2. Poll physical FC MSP
        if self.msp and self.msp.connected:
            self.msp.poll()

    def print_status(self):
        tx_stats = "N/A"
        if self.session and hasattr(self.session, "telemetry"):
            link = self.session.telemetry.get("link", {})
            lq = link.get("up_lq", 0)
            rf = link.get("rf_mode", 0)
            tx_stats = f"LQ:{lq}% RF:{rf}"

        msp_rc = "None"
        if self.msp and self.msp.last_rc:
            msp_rc = f"R:{self.msp.last_rc[0]} P:{self.msp.last_rc[1]} Y:{self.msp.last_rc[2]} T:{self.msp.last_rc[3]}"

        status_line = (
            f"\r[SIM {self.sim_mode:8s} | Alt: {self.sim_z:.3f}m | "
            f"TX: {tx_stats} | FC MSP: {msp_rc}]"
        )
        sys.stdout.write(status_line)
        sys.stdout.flush()

    def send_sim_command(self, cmd_str):
        msg = String()
        msg.data = json.dumps({"cmd": cmd_str})
        self.pub_cmd.publish(msg)
        self.get_logger().info(f"Issued Command to Gazebo FC: {cmd_str.upper()}")

    def start_terminal_listener(self):
        def listener():
            time.sleep(1.0)
            print("\n" + "="*70)
            print("  DRONEFORGE TX-TO-SIMULATION INTERACTIVE COMMANDS:")
            print("    [a] Arm  |  [t] Takeoff (0.50m)  |  [h] Pos Hold  |  [l] Land  |  [d] Disarm")
            print("="*70 + "\n")
            while True:
                try:
                    line = sys.stdin.readline()
                    if not line:
                        break
                    cmd = line.strip().lower()
                    if cmd in ("a", "arm"):
                        self.send_sim_command("arm")
                    elif cmd in ("t", "takeoff"):
                        self.send_sim_command("takeoff")
                    elif cmd in ("h", "hold"):
                        self.send_sim_command("pos_hold")
                    elif cmd in ("l", "land"):
                        self.send_sim_command("land")
                    elif cmd in ("d", "disarm"):
                        self.send_sim_command("disarm")
                except Exception:
                    break
        th = threading.Thread(target=listener, daemon=True)
        th.start()

    def destroy_node(self):
        if self.session is not None:
            try:
                self.session.stop()
            except Exception:
                pass
        if self.msp is not None:
            self.msp.close()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description="DroneForge TX-to-Simulation Bridge")
    parser.add_argument("--tx-port", default="/dev/ttyUSB0", help="Serial port for ELRS TX dongle")
    parser.add_argument("--fc-port", default=None, help="Serial port for physical Matek H743 FC")
    cli_args = parser.parse_args()

    rclpy.init()
    node = TxSimBridgeNode(tx_port=cli_args.tx_port, fc_port=cli_args.fc_port)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
