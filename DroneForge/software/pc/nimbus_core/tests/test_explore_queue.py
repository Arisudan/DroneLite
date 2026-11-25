"""EXPLORE pressed during the take-off is QUEUED and starts by itself when the FC holds the height (2026-11-25: the button is live right after TAKE OFF, not ~13 s later).
Fast logic tests (no simulator): the real HoverAutonomy with its state set directly."""
import unittest
from types import SimpleNamespace

from nimbus_core.core import NimbusCore
from nimbus_core.autonomy import HoverState
from nimbus_core.steering import SteerLink


def _au():
    core = NimbusCore(); au = core.autonomy; au.steer = SteerLink(port=0)
    return core, au


class TestExploreQueue(unittest.TestCase):
    def test_refused_on_the_ground_and_while_landing(self):
        core, au = _au()
        ok, msg = au.set_explore(True); self.assertFalse(ok); self.assertFalse(au.explore_pending); self.assertIn("TAKE OFF", msg)
        au.phase = "flying"; au.ctl.state = HoverState.LAND
        ok, _ = au.set_explore(True); self.assertFalse(ok); self.assertFalse(au.explore_pending)

    def test_during_take_off_it_is_queued_and_nothing_steers_yet(self):
        core, au = _au(); au.phase = "flying"; au.ctl.state = HoverState.TAKEOFF
        ok, msg = au.set_explore(True)
        self.assertTrue(ok); self.assertIn("queued", msg)
        self.assertTrue(au.explore_pending); self.assertFalse(au.explore); self.assertFalse(au.steer.active)       # no steering before HOLDING
        au._promote_explore(); self.assertTrue(au.explore_pending and not au.explore)                              # still climbing: stays queued
        self.assertTrue(au.status()["explore_pending"])

    def test_it_starts_by_itself_at_hold(self):
        core, au = _au(); seen = []
        from nimbus_core import topics as T
        core.bus.subscribe(T.EVENT, lambda text: seen.append(str(text)))
        au.phase = "flying"; au.ctl.state = HoverState.TAKEOFF; au.set_explore(True)
        try:
            au.ctl.state = HoverState.HOVER; au._promote_explore()
            self.assertTrue(au.explore); self.assertFalse(au.explore_pending); self.assertTrue(au.steer.active)
            self.assertTrue(any("explore started (queued)" in e for e in seen), seen)
        finally:
            au.steer.stop()

    def test_every_other_ending_drops_the_queue(self):
        for end in (HoverState.LAND, HoverState.DONE, HoverState.SENSOR_LOSS):
            core, au = _au(); au.phase = "flying"; au.ctl.state = HoverState.TAKEOFF; au.set_explore(True)
            au.ctl.state = end; au._promote_explore()
            self.assertFalse(au.explore_pending); self.assertFalse(au.explore, end)

    def test_second_click_and_land_cancel_the_queue(self):
        core, au = _au(); au.phase = "flying"; au.ctl.state = HoverState.TAKEOFF; au.set_explore(True)
        ok, msg = au.set_explore(False); self.assertTrue(ok); self.assertFalse(au.explore_pending); self.assertIn("cancelled", msg)
        au.set_explore(True); self.assertTrue(au.explore_pending); au.land("test"); self.assertFalse(au.explore_pending)
        au.set_explore(True)                                                                                    # landing: refused, nothing queued
        self.assertFalse(au.explore_pending)

    def test_already_holding_still_starts_at_once(self):
        core, au = _au(); au.phase = "flying"; au.ctl.state = HoverState.HOVER
        try:
            ok, _ = au.set_explore(True); self.assertTrue(ok); self.assertTrue(au.explore); self.assertFalse(au.explore_pending)
        finally:
            au.steer.stop()


class TestExploreButton(unittest.TestCase):
    """The Fly page button: live from take-off on, shows the queue, grey on the ground."""
    class _B:
        def __init__(s): s.enabled = None; s.text = ""; s.active = None; s.sub = ""; s.tip = ""
        def setEnabled(s, v): s.enabled = v
        def setText(s, v): s.text = v
        def set_active(s, v): s.active = v
        def set_pending(s, v): s.pending = v
        def set_sub(s, v): s.sub = v
        def setToolTip(s, v): s.tip = v

    def _page(self, explore=False, pending=False):
        au = SimpleNamespace(explore=explore, explore_pending=pending, steer=None)
        return SimpleNamespace(win=SimpleNamespace(model=SimpleNamespace(core=SimpleNamespace(autonomy=au))), btn_explore=self._B(), _explore_sig=None, _mode_sig=None,
                               mode_text=SimpleNamespace(setText=lambda v: None, setStyleSheet=lambda v: None))

    def test_states(self):
        from nimbus_gcs.fly_page import FlyPage
        for ph, pend, want in (("ready", False, False), ("armed", False, False), ("takeoff", False, True), ("climbing", False, True), ("holding", False, True), ("landing", False, False), ("climbing", True, True)):
            p = self._page(pending=pend); FlyPage._refresh_explore(p, ph); self.assertEqual(p.btn_explore.enabled, want, (ph, pend))
        p = self._page(pending=True); FlyPage._refresh_explore(p, "climbing")
        self.assertIn("EXPLORE", p.btn_explore.text); self.assertEqual(p.btn_explore.sub, "starts at HOLDING"); self.assertTrue(p.btn_explore.pending)    # state readable without colour
        p = self._page(); FlyPage._refresh_explore(p, "ready"); self.assertEqual(p.btn_explore.sub, "after take-off")


if __name__ == "__main__":
    unittest.main()
