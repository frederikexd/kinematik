# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""Tests for suspension/perf.py — the per-tab timing harness.

The harness wraps every tab in the app, so the properties that matter most are
the boring ones: it must cost nothing when off, never raise, and never
mis-attribute time. A profiler that sends you to optimise the wrong tab is
worse than no profiler, because you act on it.
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from suspension import perf                                # noqa: E402


def _busy(ms):
    """Burn wall time without sleeping, closer to what a tab body does."""
    end = time.perf_counter() + ms / 1000.0
    while time.perf_counter() < end:
        pass


def _run(store, blocks):
    """One script-run. blocks = [(feature, ms, active), ...]"""
    perf.begin_run(store)
    for feat, ms, active in blocks:
        tok = perf.enter(store, feat, active=active)
        _busy(ms)
        perf.exit(store, tok)
    perf.end_run(store)


# --- off by default --------------------------------------------------------
def test_disabled_by_default_and_stores_nothing():
    ss = {}
    assert perf.enabled(ss) is False
    _run(ss, [("brakes", 1, False)])
    assert perf.summary(ss)["runs"] == 0
    assert "_kk_perf" not in ss


def test_disabled_enter_returns_none_and_exit_tolerates_it():
    ss = {}
    assert perf.enter(ss, "brakes") is None
    perf.exit(ss, None)          # must be a no-op, not a crash


def test_disabling_drops_the_buffer():
    """A stale buffer read as current data is worse than no data."""
    ss = {}
    perf.set_enabled(ss, True)
    _run(ss, [("brakes", 1, False)])
    assert perf.summary(ss)["runs"] == 1
    perf.set_enabled(ss, False)
    assert perf.summary(ss)["runs"] == 0


# --- attribution -----------------------------------------------------------
def test_ranks_the_expensive_tab_first():
    ss = {}
    perf.set_enabled(ss, True)
    for _ in range(3):
        _run(ss, [("kinematics", 2, True), ("model3d", 20, False),
                  ("cost", 1, False)])
    rows = perf.summary(ss)["rows"]
    assert rows[0]["feature"] == "model3d"
    assert rows[0]["median_ms"] > rows[-1]["median_ms"]


def test_nested_blocks_report_self_time_not_inclusive():
    """Inclusive time makes an outer tab look like the culprit when the cost
    is really in something nested inside it."""
    ss = {}
    perf.set_enabled(ss, True)
    perf.begin_run(ss)
    outer = perf.enter(ss, "integration", active=True)
    _busy(2)
    inner = perf.enter(ss, "brakes", active=False)
    _busy(20)
    perf.exit(ss, inner)
    perf.exit(ss, outer)
    perf.end_run(ss)
    by = {r["feature"]: r["median_ms"] for r in perf.summary(ss)["rows"]}
    assert by["brakes"] > by["integration"] * 3, by


def test_multiple_blocks_for_one_feature_sum():
    """model3d owns three separate `with tab_car:` blocks; a reader thinks of
    it as one feature with one cost."""
    ss = {}
    perf.set_enabled(ss, True)
    perf.begin_run(ss)
    for _ in range(3):
        tok = perf.enter(ss, "model3d", active=False)
        _busy(5)
        perf.exit(ss, tok)
    perf.end_run(ss)
    row = perf.summary(ss)["rows"][0]
    assert row["blocks"] == 3
    assert row["median_ms"] > 12          # ~15 ms, i.e. summed not averaged


# --- the headline number ---------------------------------------------------
def test_background_share_identifies_wasted_work():
    """This is the number that decides whether lazy tabs are worth building."""
    ss = {}
    perf.set_enabled(ss, True)
    for _ in range(3):
        _run(ss, [("kinematics", 2, True), ("brakes", 20, False),
                  ("aero", 20, False)])
    s = perf.summary(ss)
    assert s["background_pct"] > 80
    assert s["background_ms"] > 0


def test_a_single_active_tab_reports_no_waste():
    ss = {}
    perf.set_enabled(ss, True)
    for _ in range(3):
        _run(ss, [("kinematics", 5, True)])
    assert perf.summary(ss)["background_pct"] == 0.0


# --- robustness ------------------------------------------------------------
def test_ring_buffer_is_bounded():
    ss = {}
    perf.set_enabled(ss, True)
    for _ in range(perf.MAX_RUNS + 15):
        _run(ss, [("brakes", 0.2, False)])
    assert perf.summary(ss)["runs"] == perf.MAX_RUNS


def test_a_raising_tab_body_does_not_corrupt_later_runs():
    """A body that throws leaves its frame open; the next run must still be
    attributed correctly rather than folding into the abandoned frame."""
    ss = {}
    perf.set_enabled(ss, True)
    perf.begin_run(ss)
    perf.enter(ss, "brakes", active=False)     # never exited
    perf.end_run(ss)
    _run(ss, [("aero", 5, True)])
    by = {r["feature"]: r for r in perf.summary(ss)["rows"]}
    assert "aero" in by and by["aero"]["median_ms"] > 0


def test_exit_without_begin_run_is_safe():
    ss = {}
    perf.set_enabled(ss, True)
    tok = perf.enter(ss, "brakes")             # no begin_run first
    perf.exit(ss, tok)
    perf.end_run(ss)
    assert perf.summary(ss)["runs"] >= 0       # no exception is the assertion


def test_summary_on_empty_store_is_well_formed():
    s = perf.summary({})
    assert s["runs"] == 0 and s["rows"] == [] and s["background_pct"] == 0.0


def test_report_text_is_readable_when_empty():
    assert "No timing data" in perf.format_report({})


def test_overhead_when_disabled_is_negligible():
    """40 tabs per rerun must not cost a measurable fraction of a frame."""
    ss = {}
    n = 20000
    t0 = time.perf_counter()
    for _ in range(n):
        perf.exit(ss, perf.enter(ss, "brakes"))
    per_tab_us = (time.perf_counter() - t0) / n * 1e6
    assert per_tab_us < 5.0, f"{per_tab_us:.2f} µs per tab when disabled"
