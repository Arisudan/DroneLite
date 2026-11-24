#!/usr/bin/env python3
"""One-page PDF: how the ELRS TX firmware and the PC <-> FC data flow work, in plain English.
usage: make_elrs_briefing_pdf.py <out.pdf> [font_size]   (needs reportlab; DejaVu Sans fonts)"""
import sys
from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import Paragraph, SimpleDocTemplate, Spacer, Table, TableStyle, KeepTogether
from reportlab.graphics.shapes import Drawing, Rect, String, Line, Polygon

OUT = sys.argv[1]
FS = float(sys.argv[2]) if len(sys.argv) > 2 else 8.6
D = "/usr/share/fonts/truetype/dejavu/"
pdfmetrics.registerFont(TTFont("S", D + "DejaVuSans.ttf"))
pdfmetrics.registerFont(TTFont("SB", D + "DejaVuSans-Bold.ttf"))
pdfmetrics.registerFontFamily("S", normal="S", bold="SB", italic="S", boldItalic="SB")
INK, ACC, SOFT = colors.HexColor("#1b1f24"), colors.HexColor("#0b5cad"), colors.HexColor("#eef3f9")

body = ParagraphStyle("b", fontName="S", fontSize=FS, leading=FS * 1.32, textColor=INK, alignment=TA_LEFT)
bul = ParagraphStyle("u", parent=body, leftIndent=9, bulletIndent=0, spaceAfter=0.6)
h = ParagraphStyle("h", parent=body, fontName="SB", fontSize=FS + 1.8, leading=FS + 4, textColor=ACC, spaceBefore=5, spaceAfter=1.5)
title = ParagraphStyle("t", parent=body, fontName="SB", fontSize=15, leading=18, textColor=INK)
sub = ParagraphStyle("s", parent=body, fontSize=FS - 0.6, textColor=colors.HexColor("#5b6670"))


def B(items, num=False):
    return [Paragraph(t, bul, bulletText=(f"{i}." if num else "•")) for i, t in enumerate(items, 1)]


def arrow(d, x1, x2, y, label, color):
    d.add(Line(x1, y, x2, y, strokeColor=color, strokeWidth=1.3))
    tip = 4 if x2 > x1 else -4
    d.add(Polygon([x2, y, x2 - tip, y + 2.6, x2 - tip, y - 2.6], fillColor=color, strokeColor=color))
    top, bottom = label.split("|")
    d.add(String((x1 + x2) / 2, y + 9, top, fontName="S", fontSize=FS - 1.8, fillColor=color, textAnchor="middle"))
    d.add(String((x1 + x2) / 2, y + 2.6, bottom, fontName="S", fontSize=FS - 1.8, fillColor=color, textAnchor="middle"))


def diagram(width):
    d = Drawing(width, 92)
    boxes = [("PC", "nimbus_pc"), ("TX", "ESP32-C3 + E28"), ("RX", "GEPRC 2.4 GHz"), ("FC", "Matek + Betaflight")]
    bw, bh = 74, 25
    xs = [0, (width - bw) / 3, 2 * (width - bw) / 3, width - bw]
    for rowy, tag, col in ((58, "CONTROL lane (to the drone)", ACC), (8, "TELEMETRY lane (back to the PC)", colors.HexColor("#b4561b"))):
        d.add(String(0, rowy + bh + 3, tag, fontName="SB", fontSize=FS - 0.4, fillColor=col))
        for x, (a, b) in zip(xs, boxes):
            d.add(Rect(x, rowy, bw, bh, rx=3, ry=3, fillColor=SOFT, strokeColor=col, strokeWidth=0.9))
            d.add(String(x + bw / 2, rowy + 14, a, fontName="SB", fontSize=FS + 0.4, fillColor=INK, textAnchor="middle"))
            d.add(String(x + bw / 2, rowy + 5, b, fontName="S", fontSize=FS - 1.8, fillColor=INK, textAnchor="middle"))
    ctl = ["serial 115200|100 frames/s", "radio 2.4 GHz|250 packets/s", "serial 420000|to Betaflight"]
    tel = ["serial 115200|telemetry frames", "radio 2.4 GHz|telemetry slots", "serial 420000|from Betaflight"]
    for i in range(3):
        x1, x2 = xs[i] + bw + 2, xs[i + 1] - 2
        arrow(d, x1, x2, 58 + bh / 2, ctl[i], ACC)
        arrow(d, x2, x1, 8 + bh / 2, tel[i], colors.HexColor("#b4561b"))
    return d


story = [
    Paragraph("How the drone link works: ELRS, the TX firmware and the data flow", title),
    Paragraph("One-page read-out · Nimbus replica · 3 Nov 2026 · numbers marked (bench) were measured on our setup with the props off", sub),
    Spacer(1, 3),
    Paragraph("<b>In one line:</b> the PC sends control packets to a small radio transmitter (the TX). The TX sends them through the air to a "
              "receiver on the drone, and the receiver hands them to the flight controller (FC). Status data comes back the same way.", body),
    Spacer(1, 7), diagram(527),
    Paragraph("What ELRS and the TX firmware are", h),
    *B(["<b>ELRS (ExpressLRS)</b> is an open-source radio link for drones. It moves two things: <b>stick commands up</b> to the drone and <b>status data down</b> to us.",
        "<b>The TX firmware</b> is ELRS version 4.1.0 running on a small ESP32-C3 chip with an E28 2.4 GHz radio. We did not change ELRS itself. We only gave it our wiring map and a <b>binding phrase</b> (“<BINDING_PHRASE>”).",
        "<b>Its jobs:</b> read commands from the PC, pack them into radio packets, hop between frequencies, send and receive on the radio, pass status data back to the PC, and measure link quality.",
        "<b>Binding:</b> both ends turn the same phrase into the same ID number, so they connect by themselves at power-up. No pairing button."]),
    Paragraph("How a command reaches the drone in real time (control lane)", h),
    *B(["The PC sends <b>one control frame every 10 ms</b> (100 per second). A frame is 26 bytes: a start byte, 16 channels (roll, pitch, throttle, yaw, arm switch…) and a check byte (CRC) that catches errors.",
        "The TX reads each frame from the cable and keeps the <b>newest</b> values. Old ones are simply replaced, so nothing queues up.",
        "The TX packs those values into a radio packet, <b>250 per second</b>, jumping between frequencies. Every 2nd slot is kept free for telemetry, so about 125 control packets per second go up (worked out from the settings).",
        "The RX checks each packet and writes a new frame to the FC over its serial port. Betaflight reads the channels, mixes them into four motor speeds and drives the motors.",
        "<b>Control never waits for telemetry.</b> It is its own lane, so a late status packet cannot delay a stick movement."], num=True),
    Paragraph("How telemetry comes back (status lane)", h),
    *B(["<b>Yes, it arrives as packets.</b> The TX writes a stream of small frames to the cable: start byte, length, type, data, CRC. Our PC code finds the start byte, checks the CRC, reads the type and decodes the numbers.",
        "<b>Path:</b> the FC makes frames (battery, attitude, flight mode, motors, range/flow sensor) → sends them to the RX → the RX queues them and sends one in each telemetry slot → the TX receives it and writes it to the PC. The TX itself adds the <b>link-quality</b> frame.",
        "<b>The radio has little room for this</b>, so updates are slow (bench): range/flow 10 per second, battery and attitude about 3 per second, link quality 4 per second. At the default setting range/flow came in bursts with gaps up to 3.6 s, so we raised the telemetry share to 1:2.",
        "Telemetry is <b>status, not control</b>. It can have gaps and the drone does not depend on it to fly, except for the safety watchdog below."]),
    Paragraph("Safety, in simple words", h),
    *B(["<b>PC:</b> if fresh input stops, it sends a safe frame (throttle minimum, disarmed). The arm switch only works if it was seen OFF first and the throttle is low.",
        "<b>Sensor watchdog:</b> when armed with throttle at 5% or more, if range/flow frames stop or freeze for 0.5 s, the PC forces throttle to minimum. This is why telemetry must be steady.",
        "<b>Flight controller:</b> if the radio link is lost, Betaflight cuts throttle and disarms (under 3 s, bench). Unplugging the gamepad disarms in about 0.1 s (bench)."]),
    Spacer(1, 2),
    Paragraph("<b>Not done yet:</b> real flight, a dead-sensor test, VTX control, and Nimbus’s own extra frames (0xE1–0xE3), which our stock ELRS does not send. "
              "Full notes: DroneForge/docs/elrs_tx/elrs-tx_firmware-briefing.md", sub),
]
doc = SimpleDocTemplate(OUT, pagesize=A4, leftMargin=34, rightMargin=34, topMargin=28, bottomMargin=24,
                        title="ELRS, the TX firmware and the data flow", author="DroneForge")
doc.build(story)
