"""DroneForge GCS design system ('Mission control' look, concept 2). ONE source of truth for colours, type, spacing and the global stylesheet.

Rules of the look (see docs/gcs_screens/redesign_v5/concept2_mission-control/): slate-grey canvas (brightened 2026-11-21), 1-px hairlines, three number sizes (Thin 72 / Light 40 / mono 18),
small letter-spaced Semibold labels, 8-px grid. Colour is scarce: ONE soft-blue accent (selection, live value, primary action), AMBER only for a caution (battery),
RED only for SAFE and genuine alarms. State must also read without colour (words, shape, weight, dashed = disabled).
Fonts: Lato (+ Noto Sans Mono for tabular numbers), bundled in assets/fonts (SIL OFL) and loaded by `load_fonts()`.
"""
from __future__ import annotations

import os

from PyQt6.QtGui import QColor, QFont, QFontDatabase

# ---- colour tokens ---------------------------------------------------------------------------------------------------
BG, PANEL, HAIR, HAIR2 = "#181C23", "#20252E", "#313948", "#454F61"          # brighter slate (was near-black #090A0C): lifted 2026-11-21, same layout and accent
TXT, DIM, FAINT, DISABLED = "#F6F8FB", "#B0B9C7", "#8791A1", "#657083"
ACC, AMB, RED = "#B9D6FF", "#F0B94D", "#EF5A5F"
ACC_DEEP = "#8FB5F2"          # pressed / hover companion of ACC
ON_ACC = "#0B1626"            # text on an ACC fill

# ---- spacing / shape ---------------------------------------------------------------------------------------------------
GRID = 8
SP = {"xs": 4, "s": 8, "m": 16, "l": 24, "xl": 32}
RADIUS = {"s": 4, "m": 8, "l": 12}
TOP_BAR_H, BOTTOM_STRIP_H, RIGHT_COL_W = 56, 80, 376          # concept-2 proportions at 1920x1080 (scale with the window)
HOLD_MS_DEFAULT = 1000                                          # press-and-hold time for ARM / TAKE OFF

_ASSETS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets", "fonts")
_loaded = False


def qcolor(token: str, alpha: int = 255) -> QColor:
    c = QColor(token); c.setAlpha(alpha); return c


def load_fonts() -> bool:
    """Register the bundled Lato / Noto Sans Mono (idempotent). Returns True when Lato is available (bundled or installed)."""
    global _loaded
    if not _loaded and os.path.isdir(_ASSETS):
        for f in sorted(os.listdir(_ASSETS)):
            if f.lower().endswith(".ttf"):
                QFontDatabase.addApplicationFont(os.path.join(_ASSETS, f))
        _loaded = True
    return "Lato" in QFontDatabase.families()


def _font(family: str, px: float, weight: QFont.Weight, spacing: float = 0.0) -> QFont:
    f = QFont(family if family in QFontDatabase.families() else "Sans Serif"); f.setPixelSize(int(round(px))); f.setWeight(weight)
    if spacing:
        f.setLetterSpacing(QFont.SpacingType.AbsoluteSpacing, spacing)
    f.setHintingPreference(QFont.HintingPreference.PreferNoHinting)
    return f


def f_label(px: int = 11) -> QFont:      # small letter-spaced caps labels
    return _font("Lato", px, QFont.Weight.DemiBold, 1.6)


def f_body(px: int = 13) -> QFont:
    return _font("Lato", px, QFont.Weight.Medium, 0.1)


def f_xl(px: int = 72) -> QFont:         # number size 1: the setpoint
    return _font("Lato", px, QFont.Weight.Thin)


def f_l(px: int = 40) -> QFont:          # number size 2: camera readouts
    return _font("Lato", px, QFont.Weight.Light)


def f_mono(px: int = 18) -> QFont:       # number size 3: tabular values
    return _font("Noto Sans Mono", px, QFont.Weight.Normal)


# ---- global stylesheet -------------------------------------------------------------------------------------------------
def qss() -> str:
    """Application-wide stylesheet from the tokens. Object names used by components: primary, ghost, danger (SAFE), seg (sub-tab), chip, card, hair, dim, faint, label, mono."""
    return f"""
* {{ font-family: 'Lato', 'Noto Sans', sans-serif; font-size: 13px; color: {TXT}; outline: none; }}
QMainWindow, QWidget#root, QWidget#body {{ background: {BG}; }}
QWidget {{ background: transparent; }}
QToolTip {{ background: {PANEL}; color: {TXT}; border: 1px solid {HAIR2}; padding: 6px 8px; }}
QLabel#dim {{ color: {DIM}; }} QLabel#faint {{ color: {FAINT}; }}
QLabel#label {{ color: {DIM}; font-weight: 600; font-size: 11px; letter-spacing: 1.6px; }}
QLabel#mono {{ font-family: 'Noto Sans Mono', monospace; }}
QFrame#card {{ background: {PANEL}; border: 1px solid {HAIR}; border-radius: {RADIUS['m']}px; }}
QFrame#hair {{ background: {HAIR}; border: none; max-height: 1px; min-height: 1px; }}
QPushButton {{ background: {PANEL}; border: 1px solid {HAIR2}; border-radius: {RADIUS['s']}px; padding: 8px 16px; color: {TXT}; font-weight: 600; }}
QPushButton:hover {{ border-color: {DIM}; }} QPushButton:pressed {{ background: {HAIR}; }}
QPushButton:disabled {{ color: {DISABLED}; border: 1px dashed {HAIR2}; background: transparent; }}
QPushButton#primary {{ background: {ACC}; color: {ON_ACC}; border: 1px solid {ACC}; }}
QPushButton#primary:hover {{ background: {ACC_DEEP}; }} QPushButton#primary:disabled {{ background: transparent; color: {DISABLED}; border: 1px dashed {HAIR2}; }}
QPushButton#danger {{ background: {RED}; color: #ffffff; border: 1px solid {RED}; letter-spacing: 2px; font-weight: 700; }}
QPushButton#ghost {{ background: transparent; border: 1px solid transparent; color: {DIM}; }} QPushButton#ghost:hover {{ color: {TXT}; }}
QPushButton#seg {{ background: transparent; border: none; border-bottom: 2px solid transparent; border-radius: 0; color: {DIM}; padding: 8px 12px; letter-spacing: 1.4px; }}
QPushButton#seg:checked {{ color: {TXT}; border-bottom: 2px solid {ACC}; }} QPushButton#seg:hover {{ color: {TXT}; }}
QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox, QPlainTextEdit, QTextEdit {{ background: {PANEL}; border: 1px solid {HAIR2}; border-radius: {RADIUS['s']}px; padding: 6px 8px; selection-background-color: {ACC}; selection-color: {ON_ACC}; }}
QLineEdit:focus, QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus {{ border-color: {ACC}; }}
QComboBox::drop-down {{ border: none; width: 22px; }} QComboBox QAbstractItemView {{ background: {PANEL}; border: 1px solid {HAIR2}; selection-background-color: {HAIR}; }}
QTableWidget, QListWidget, QTreeWidget {{ background: {PANEL}; border: 1px solid {HAIR}; gridline-color: {HAIR}; alternate-background-color: {BG}; }}
QHeaderView::section {{ background: {BG}; color: {DIM}; border: none; border-bottom: 1px solid {HAIR2}; padding: 6px 8px; font-weight: 600; letter-spacing: 1.2px; }}
QListWidget::item:selected, QTableWidget::item:selected {{ background: {HAIR}; color: {TXT}; }}
QScrollArea {{ border: none; background: transparent; }}
QScrollBar:vertical {{ background: transparent; width: 10px; margin: 0; }} QScrollBar::handle:vertical {{ background: {HAIR2}; border-radius: 5px; min-height: 24px; }}
QScrollBar:horizontal {{ background: transparent; height: 10px; }} QScrollBar::handle:horizontal {{ background: {HAIR2}; border-radius: 5px; min-width: 24px; }}
QScrollBar::add-line, QScrollBar::sub-line {{ width: 0; height: 0; }}
QProgressBar {{ background: {HAIR}; border: none; border-radius: 2px; max-height: 4px; text-visible: false; }} QProgressBar::chunk {{ background: {ACC}; border-radius: 2px; }}
QMenu {{ background: {PANEL}; border: 1px solid {HAIR2}; padding: 4px; }} QMenu::item {{ padding: 8px 16px; border-radius: 4px; }} QMenu::item:selected {{ background: {HAIR}; }}
"""
