"""Stick/throttle to CRSF mapping, constants read from the nimbus_core binary (CrsfMapper)."""
import math

from .crsf import CRSF_MAX, CRSF_MID, CRSF_MIN


def _lroundf(x: float) -> int:
    """C lroundf: round half away from zero (Python round() is banker's rounding)."""
    return int(math.floor(x + 0.5)) if x >= 0 else -int(math.floor(-x + 0.5))


def stick_to_crsf(v: float) -> int:
    """-1..1 -> 172..1811, centre 992 (CrsfMapper::StickToCrsf)."""
    v = max(-1.0, min(1.0, v))
    return _lroundf(max(CRSF_MIN, min(CRSF_MAX, v * 820.0 + 992.0)))


def throttle01_to_crsf(t: float) -> int:
    """0..1 -> 172..1811 (CrsfMapper::Throttle01ToCrsf)."""
    t = max(0.0, min(1.0, t))
    return _lroundf(max(CRSF_MIN, min(CRSF_MAX, t * 1639.0 + 172.0)))


def crsf_to_throttle01(v: int) -> float:
    """CrsfMapper::CrsfToThrottle01."""
    return (max(CRSF_MIN, min(CRSF_MAX, v)) - CRSF_MIN) / 1639.0


def height_to_aux5(h: float) -> int:
    """Target height (m) -> CRSF value of CH9 (AUX5), decoded by the FC height hold (flight/height_hold.h):
    us = 1150 + 320 * h (so the FC's  (us - 1150) * 2.5 / 800  gives h back; 1 tick = 0.625 us = 0.002 m), ticks = (us - 880) * 8 / 5."""
    h = max(0.2, min(2.5, h))
    return int(round((1150 + 320.0 * h - 880.0) * 8.0 / 5.0))


AUX5_LAND = 1792            # 2000 us: the FC lands by itself (descends at <=0.12 m/s, detects touchdown, throttle to idle)


def aux5_to_height(ticks: int) -> float:
    return ((5.0 * ticks / 8.0 + 880.0) - 1150.0) * 2.5 / 800.0


# FC flight mode on ch11 (AUX7, firmware v4.2): 1200 us = ALT HOLD (the sideways loop is paused), 1900 us = POS HOLD. The FC takes 1100..1399 us as ALT HOLD and everything else
# (minimum 988 us, the 1500 us idle of an unset channel, no signal, an older PC) as POS HOLD. us = 5 * ticks / 8 + 880.
MODE_ALT_TICKS = 512
MODE_POS_TICKS = 1632


def mode_to_aux7(mode: str) -> int:
    return MODE_ALT_TICKS if mode == "alt_hold" else MODE_POS_TICKS


# v4.7.2 VO position aid on RC channels (CH13 forward, CH14 right, CH15 valid flag): 1500 us = 0, 1 us = 1 mm, +-0.40 m (firmware flight/height_hold.c hhReadRcAid)
AID_MAX_M = 0.40
AID_FLAG_TICKS = 1792                       # 2000 us: the aid is valid


def us_to_ticks(us: float) -> int:
    return int(round((us - 880.0) * 8.0 / 5.0))


def aid_to_ticks(m: float) -> int:
    return us_to_ticks(1500.0 + 1000.0 * max(-AID_MAX_M, min(AID_MAX_M, m)))
