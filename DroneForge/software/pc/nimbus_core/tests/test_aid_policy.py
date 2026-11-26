from nimbus_core.aid_policy import aid_active, ground_aid_enabled

PHASES = ("disconnected", "ready", "arming", "armed", "takeoff", "climbing", "holding", "landing", "landed", "fault")


def test_default_is_unchanged_aid_only_while_holding():
    assert [p for p in PHASES if aid_active(p, False, False)] == ["holding"]


def test_ground_key_frame_covers_floor_to_hold_but_never_landing_or_fault():
    assert [p for p in PHASES if aid_active(p, False, True)] == ["armed", "takeoff", "climbing", "holding"]


def test_never_while_exploring():
    assert not any(aid_active(p, True, g) for p in PHASES for g in (False, True))


def test_opt_in_only_with_exactly_1():
    assert ground_aid_enabled({"DF_GROUND_AID": "1"})
    assert not any(ground_aid_enabled(e) for e in ({}, {"DF_GROUND_AID": ""}, {"DF_GROUND_AID": "0"}, {"DF_GROUND_AID": "yes"}))
