"""When the visual-odometry position aid runs: which flight phases, and where the key frame may be set. Qt-free so it is tested without the GCS.

Default (unchanged): the key frame is set and the aid is sent only while HOLDING.
Ground key frame (opt-in: environment DF_GROUND_AID=1): the key frame is set while the drone still stands on the floor (armed, motors idle) and the aid is sent from
there through take-off and climb into the hold, so the hold point is the take-off spot. An FC up to v4.7.4R ignores the aid until its own HOLD (then it pulls back
to the take-off spot); v4.8 (not built, not flashed) also uses it in the climb. The aid's sign has never been proven in flight: keep this off until it has."""
from __future__ import annotations

import os

GROUND_PHASES = ("armed", "takeoff", "climbing", "holding")


def ground_aid_enabled(env=None) -> bool:
    return str((os.environ if env is None else env).get("DF_GROUND_AID", "")).strip() == "1"


def aid_active(phase: str, exploring: bool, ground: bool) -> bool:
    """True while the key frame may exist and the aid may be sent. Never while the explorer steers."""
    if exploring:
        return False
    return phase in (GROUND_PHASES if ground else ("holding",))
