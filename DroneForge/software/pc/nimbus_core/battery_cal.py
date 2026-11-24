"""Battery voltage calibration without a multimeter: a fully charged resting LiPo is ~4.16-4.20 V/cell."""
from __future__ import annotations

LO, HI = 0.5, 2.0


def calibrate_battery_scale(raw_v: float, cells: int, full_per_cell: float = 4.16) -> float:
    """Correction factor so that raw_v reads as cells*full_per_cell, clamped to 0.5..2.0."""
    if not raw_v or raw_v <= 0 or cells <= 0:
        raise ValueError("need a positive raw voltage and cell count")
    return max(LO, min(HI, cells * full_per_cell / raw_v))
