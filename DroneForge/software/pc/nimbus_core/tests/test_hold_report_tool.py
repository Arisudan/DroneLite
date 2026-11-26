import importlib.util
import os

_P = os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "tools_scripts", "hold_report.py")
_S = importlib.util.spec_from_file_location("hold_report", _P); H = importlib.util.module_from_spec(_S); _S.loader.exec_module(H)


def row(phase, arm=1, sent=1500, fc=""):
    return dict(phase=phase, sent_arm=str(arm), sent_aid_valid_us=str(sent), fc_aid_valid_us=str(fc))


def test_not_a_flight():
    assert "not a flight" in H.verdict(H.report([row("ready", arm=0)]))


def test_cut_short_before_holding():
    r = H.report([row("armed"), row("takeoff"), row("climbing", sent=2000, fc=2000)])
    assert not r["reached_holding"] and r["all_armed"]["at_fc"] == 1
    assert "never reached HOLDING" in H.verdict(r)


def test_holding_cases():
    assert "never sent" in H.verdict(H.report([row("holding")]))
    assert "never showed" in H.verdict(H.report([row("holding", sent=2000, fc=1500)]))
    assert "seen at the FC in 1 of 2" in H.verdict(H.report([row("holding", sent=2000, fc=2000), row("holding")]))
