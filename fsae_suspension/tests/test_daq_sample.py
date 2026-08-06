# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""The worked example must keep working as an example.

A demo dataset rots differently from production code: nothing breaks, it just
quietly stops demonstrating the thing it was built to demonstrate. Tighten a
Nyquist threshold in daq_plan.py and the aliasing channel silently starts
passing — the sample still loads, still looks fine, and teaches nothing. These
tests assert each planted fault still fires.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from suspension import daq_sample as ds                    # noqa: E402
from suspension import daq_plan as dp                      # noqa: E402


def _checks(plan):
    return {f.check for f in plan.findings}


def _by_check(plan, check):
    return [f for f in plan.findings if f.check == check]


# --- it builds ------------------------------------------------------------
def test_sample_plan_builds():
    p = ds.sample_plan()
    assert len(p.sensors) == 14
    assert p.findings


def test_sample_is_not_ready_by_design():
    """It has unanswered questions and a failed check; READY would mean the
    example stopped demonstrating the hard rule."""
    assert ds.sample_plan().verdict is not dp.Verdict.READY


# --- each planted fault still fires ---------------------------------------
def test_aliasing_channel_still_aliases():
    """The headline lesson: the one acquisition error you cannot undo."""
    hits = _by_check(ds.sample_plan(), "nyquist-violation")
    assert hits, "damper_pot_fl no longer trips the Nyquist check"
    assert any("Damper position" in f.message for f in hits)


def test_missing_antialias_filter_is_flagged():
    p = ds.sample_plan()
    assert any("anti-alias" in f.message.lower() for f in p.findings)


def test_oversampled_channel_is_flagged():
    p = ds.sample_plan()
    assert any("oversampl" in f.message.lower() for f in p.findings)


def test_unisolated_tractive_channel_fails():
    hits = _by_check(ds.sample_plan(), "isolation-required")
    assert any(f.severity is dp.Severity.FAIL for f in hits)
    assert any("Tractive system current" in f.message for f in hits)


def test_unanswered_questions_are_reported():
    hits = _by_check(ds.sample_plan(), "doc-incomplete")
    assert any("Brake pressure" in f.message for f in hits)


def test_channel_already_on_the_bus_is_flagged_as_duplicate():
    """And its power/connector questions are waived rather than counted."""
    p = ds.sample_plan()
    inv = next(s for s in p.sensors if s.key == "inverter_temp")
    assert inv.available_on_existing_bus
    assert inv.not_applicable(), "the NA waiver no longer applies"


# --- the bus lesson: load and schedulability are different questions -------
def test_bus_load_looks_acceptable():
    """If this ever goes red, the demo loses its point — the whole lesson is
    that a comfortable load number hides missed deadlines."""
    b = ds.sample_plan().bus_result
    assert b.load < b.bus.load_warn, f"load {b.load:.0%} is no longer 'fine'"


def test_messages_still_miss_their_deadlines():
    b = ds.sample_plan().bus_result
    assert len(b.unschedulable) >= 4, b.unschedulable


# --- budgets --------------------------------------------------------------
def test_uart_link_budget_is_computable_and_over():
    """'uncheckable' would mean the frame was left undeclared — a different
    and less interesting failure than the one intended."""
    p = ds.sample_plan()
    assert "uart-budget-uncheckable" not in _checks(p)
    assert any("baud" in f.message and f.severity is dp.Severity.FAIL
               for f in p.findings)


def test_delta_t_pair_is_too_coarse_for_the_expected_rise():
    d = ds.sample_plan().delta_t
    assert d is not None
    assert d.relative_error > 0.2, (
        f"{d.relative_error:.0%} — the sensor pair is no longer the cautionary "
        f"example it was chosen to be")


def test_rails_and_storage_stay_within_budget():
    """Deliberately fine. An example where everything fails teaches nothing
    about what a passing check looks like."""
    p = ds.sample_plan()
    for _rail, d in (p.power.per_rail or {}).items():
        assert d["frac"] < 1.0
    assert p.storage.frac < 1.0


# --- exports --------------------------------------------------------------
def test_csv_round_trips_every_channel():
    csv = ds.sample_plan().to_csv()
    assert len(csv.splitlines()) == 15          # header + 14
    assert "damper_pot_fl" in csv


def test_text_report_names_the_planted_faults():
    txt = ds._report()
    assert "DEMO DATA" in txt
    assert "Damper position" in txt
    assert "cannot meet their own period" in txt
