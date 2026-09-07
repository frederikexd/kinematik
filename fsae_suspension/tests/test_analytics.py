"""Regression test for the workspace-attribution ordering bug."""
import unittest, sys
sys.path.insert(0, '.')
import suspension.analytics as ax


class TestWorkspaceBackfill(unittest.TestCase):
    """Streamlit runs the script top to bottom, and the analytics init that
    calls set_workspace() sits thousands of lines below the central widget
    instrumentation. So a run's feature events are emitted BEFORE the workspace
    is known. Production data before the fix: session_start 30/32 attributed,
    workflow_complete 3/53, and every unattributed completion timestamped
    0.2-0.5 s earlier than its own session_start."""

    def setUp(self):
        self.store = {}
        ax._store = lambda: self.store
        ax._sampled = lambda et: True
        ax._SINK._q.queue.clear()
        ax._sset("enabled", True)
        ax._sset("session_id", "sess-A")

    def _q(self):
        return list(ax._SINK._q.queue)

    def test_events_emitted_before_set_workspace_are_stamped(self):
        ax.complete("kinematics", "solve")
        self.assertTrue(all(e["workspace_id"] is None for e in self._q()))
        ax.set_workspace("ws-1")
        self.assertTrue(all(e["workspace_id"] == "ws-1" for e in self._q()),
                        "queued events were not back-filled")

    def test_events_after_set_workspace_carry_it_directly(self):
        ax.set_workspace("ws-1")
        ax.complete("pcb", "diagnose")
        self.assertEqual(self._q()[-1]["workspace_id"], "ws-1")

    def test_another_session_is_never_stamped(self):
        """The sink is process-wide. Stamping by session_id is what stops one
        team's workspace being written onto another team's event."""
        ax._sset("session_id", "sess-B")
        ax.complete("laptime", "run")
        ax._sset("session_id", "sess-A")
        ax.complete("aero", "sweep")
        ax.set_workspace("ws-1")
        for e in self._q():
            expected = "ws-1" if e["session_id"] == "sess-A" else None
            self.assertEqual(e["workspace_id"], expected)

    def test_sign_out_does_not_backfill(self):
        ax.complete("kinematics", "solve")
        ax.set_workspace(None)
        self.assertTrue(all(e["workspace_id"] is None for e in self._q()))

    def test_backfill_never_raises(self):
        """Telemetry must not be able to break a render."""
        self.assertEqual(ax._SINK.backfill_workspace(None, "ws-1"), 0)
        self.assertEqual(ax._SINK.backfill_workspace("sess-A", None), 0)


if __name__ == "__main__":
    unittest.main()
