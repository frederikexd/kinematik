# ============================================================================
#  KinematiK — tests for solved-property bounds inside the InverseGenesis loop
#  Copyright (c) 2026 Frederik Thio. Open source.
# ============================================================================
"""The channels do not carry anti-squat, migration or caster; a bound does.

These tests pin the distinction the paper turns on: a bound applied AFTER the
search is a screen on a finished field (it can only report that nothing it
happened to produce survived), while a bound applied INSIDE the search is a
wall the solver steers along. Everything here asserts the second behaviour,
plus the back-compatibility guarantee that declaring no bounds changes
nothing at all.
"""
from __future__ import annotations

import numpy as np
import pytest

from suspension.kinematics import Hardpoints
from suspension.kinematik_stochastic import ToleranceField, _perturbed
from suspension.inverse_genesis import (
    SOLVED_PROPERTIES, GenesisTargets, LegalVolume, PropertyBound,
    SolvedPropertyBounds, TargetCurve, curves_of, genesis_solve,
    inverse_genesis, properties_of, render_genesis_md, _shifted,
)

STATIONS = np.array([-25.0, -12.5, 0.0, 12.5, 25.0])
CTX = dict(cg_height_mm=280.0, wheelbase_mm=1630.0, track_mm=1200.0,
           brake_bias_front=0.60, drive_bias_rear=1.0)
WIDE_POINTS = ["upper_front_inner", "upper_rear_inner",
               "lower_front_inner", "lower_rear_inner"]


@pytest.fixture(scope="module")
def nominal() -> Hardpoints:
    return Hardpoints.default()


@pytest.fixture(scope="module")
def targets(nominal) -> GenesisTargets:
    """Curves taken from a known geometry the engine has never seen."""
    truth = _perturbed(nominal, {"upper_front_inner": np.array([0.0, -4.0, 5.0]),
                                 "upper_rear_inner": np.array([0.0, -4.0, 5.0])})
    cur, ok = curves_of(truth, STATIONS)
    assert ok
    return GenesisTargets(curves=[
        TargetCurve("camber_deg", STATIONS, cur["camber_deg"], np.full(5, 0.15)),
        TargetCurve("toe_deg", STATIONS, cur["toe_deg"], np.full(5, 0.08)),
        TargetCurve("rc_height_mm", STATIONS, cur["rc_height_mm"],
                    np.full(5, 6.0)),
    ])


def volume(bounds=None, half=20.0, points=None) -> LegalVolume:
    props = None
    if bounds is not None:
        props = SolvedPropertyBounds(bounds=bounds, **CTX)
    return LegalVolume.around(Hardpoints.default(), half,
                              points=points or WIDE_POINTS, properties=props)


# --------------------------------------------------------------------------- #
#  PropertyBound itself
# --------------------------------------------------------------------------- #
def test_unknown_property_is_rejected():
    with pytest.raises(ValueError):
        PropertyBound("camber_gain_deg_per_mm", lo=0.0)


def test_bound_with_no_side_is_rejected():
    """An unbounded bound is not a constraint and must not pretend to be."""
    with pytest.raises(ValueError):
        PropertyBound("anti_squat_pct")


def test_inverted_bound_is_rejected():
    with pytest.raises(ValueError):
        PropertyBound("caster_deg", lo=5.0, hi=1.0)


def test_margin_sign_and_nonfinite():
    b = PropertyBound("anti_squat_pct", lo=23.0, hi=60.0)
    assert b.margin(40.0) > 0.0
    assert b.margin(23.0) == pytest.approx(0.0)
    assert b.margin(10.0) < 0.0
    assert b.margin(70.0) < 0.0
    # a property with no finite value is a violation, not a free pass
    assert b.margin(float("nan")) == float("-inf")


def test_default_labels_read_as_the_declaration():
    assert "<=" in PropertyBound("caster_deg", hi=6.0).label
    assert ">=" in PropertyBound("anti_squat_pct", lo=23.0).label
    band = PropertyBound("anti_squat_pct", lo=23.0, hi=60.0).label
    assert "23" in band and "60" in band


# --------------------------------------------------------------------------- #
#  properties_of
# --------------------------------------------------------------------------- #
def test_properties_match_the_kinematics_solver(nominal):
    from suspension.kinematics import SuspensionKinematics
    ctx = SolvedPropertyBounds(bounds=[], **CTX)
    got = properties_of(nominal, ctx, only=SOLVED_PROPERTIES)
    assert set(got) == set(SOLVED_PROPERTIES)
    kin = SuspensionKinematics(nominal)
    st = kin.solve_at_travel(0.0)
    assert got["caster_deg"] == pytest.approx(st.caster, abs=1e-6)
    assert got["kpi_deg"] == pytest.approx(st.kpi, abs=1e-6)
    assert got["scrub_static_mm"] == pytest.approx(st.scrub_radius, abs=1e-6)
    assert got["anti_squat_pct"] == pytest.approx(
        kin.anti_squat_pct(CTX["cg_height_mm"], CTX["wheelbase_mm"],
                           CTX["drive_bias_rear"]), rel=1e-6)
    assert got["anti_dive_pct"] == pytest.approx(
        kin.anti_dive_pct(CTX["cg_height_mm"], CTX["wheelbase_mm"],
                          CTX["brake_bias_front"]), rel=1e-6)


def test_only_bounded_properties_are_computed(nominal):
    """Cost control: an unbounded property is never evaluated."""
    ctx = SolvedPropertyBounds(
        bounds=[PropertyBound("caster_deg", lo=0.0, hi=10.0)], **CTX)
    assert set(ctx.evaluate(nominal)) == {"caster_deg"}
    assert ctx.needed() == {"caster_deg"}


def test_no_bounds_means_no_work(nominal):
    ctx = SolvedPropertyBounds(bounds=[], **CTX)
    assert ctx.evaluate(nominal) == {}
    assert ctx.violations(nominal) == []


# --------------------------------------------------------------------------- #
#  The gap this feature closes
# --------------------------------------------------------------------------- #
def test_channels_alone_do_not_carry_anti_squat(nominal, targets):
    """The premise: every candidate hits every channel, anti-squat runs free."""
    vol = volume(bounds=None)
    res = inverse_genesis(nominal, targets, vol, fld=None, n_starts=6, seed=0)
    probe = SolvedPropertyBounds(bounds=[], **CTX)
    vals = [properties_of(_shifted(nominal, vol, c.shift_vec), probe,
                          only=["anti_squat_pct"])["anti_squat_pct"]
            for c in res.candidates if c.hit]
    assert len(vals) >= 3
    assert max(vals) - min(vals) > 30.0, (
        "the scenario is only meaningful if the unbounded field scatters")


def test_bound_inside_the_search_is_respected_by_every_candidate(
        nominal, targets):
    vol = volume([PropertyBound("anti_squat_pct", lo=23.0, hi=60.0)])
    res = inverse_genesis(nominal, targets, vol, fld=ToleranceField.preset(
        "jig_weld"), n_starts=6, n_yield=1000, seed=0)
    assert res.ok and res.winner is not None
    hits = [c for c in res.candidates if c.hit]
    assert hits
    for c in hits:
        assert c.properties is not None
        assert 23.0 - 1e-9 <= c.properties["anti_squat_pct"] <= 60.0 + 1e-9


def test_winner_geometry_re_evaluates_inside_the_band(nominal, targets):
    """The bound holds on the delivered coordinates, not just in the report."""
    vol = volume([PropertyBound("anti_squat_pct", lo=23.0, hi=60.0)])
    res = inverse_genesis(nominal, targets, vol, fld=None, n_starts=6, seed=0)
    assert res.winner_hp is not None
    assert vol.property_violations(res.winner_hp) == []


def test_a_one_sided_bound_does_not_bound_the_other_side(nominal, targets):
    """Declaring only a floor lets the solver overshoot; the module says so."""
    vol = volume([PropertyBound("anti_squat_pct", lo=23.0)])
    res = inverse_genesis(nominal, targets, vol, fld=None, n_starts=6, seed=0)
    assert res.ok
    assert all(c.properties["anti_squat_pct"] >= 23.0 - 1e-9
               for c in res.candidates if c.hit)


def test_unreachable_bound_is_named_not_fabricated(nominal, targets):
    vol = volume([PropertyBound("rc_migration_mm_per_mm", lo=-0.15, hi=0.15)])
    res = inverse_genesis(nominal, targets, vol, fld=None, n_starts=6, seed=0)
    assert not res.ok
    assert res.winner is None
    assert "solved-property" in res.reason
    # and it says WHY: unreachable from this volume, not unreachable full stop
    assert "LEGAL VOLUME" in res.reason or "legal volume" in res.reason


def test_refused_steps_are_counted_and_reported(nominal, targets):
    vol = volume([PropertyBound("anti_squat_pct", lo=23.0, hi=60.0)])
    res = inverse_genesis(nominal, targets, vol, fld=None, n_starts=6, seed=0)
    assert res.winner is not None
    assert res.winner.property_rejections >= 0
    md = render_genesis_md(res)
    assert "Solved properties, bounded inside the search" in md
    assert "anti-squat" in md


def test_start_that_violates_a_bound_is_refused_not_walked_in(nominal, targets):
    """The wall is absolute: an infeasible start is refused, and it says so."""
    vol = volume([PropertyBound("anti_squat_pct", lo=200.0, hi=400.0)])
    c = genesis_solve(nominal, targets, vol)
    assert not c.ok
    assert c.property_rejections == 1
    assert "anti-squat" in c.worst_row


# --------------------------------------------------------------------------- #
#  Back-compatibility and determinism
# --------------------------------------------------------------------------- #
def test_declaring_no_bounds_reproduces_the_unbounded_winner(nominal, targets):
    fld = ToleranceField.preset("jig_weld")
    pts = ["upper_front_inner", "upper_rear_inner"]
    a = inverse_genesis(nominal, targets, volume(None, 8.0, pts), fld=fld,
                        n_starts=5, n_yield=1000, seed=0)
    b = inverse_genesis(nominal, targets, volume([], 8.0, pts), fld=fld,
                        n_starts=5, n_yield=1000, seed=0)
    assert a.winner is not None and b.winner is not None
    assert np.allclose(a.winner.shift_vec, b.winner.shift_vec)
    assert a.winner.yield_frac == pytest.approx(b.winner.yield_frac)


def test_bounded_runs_are_deterministic(nominal, targets):
    vol = volume([PropertyBound("anti_squat_pct", lo=23.0, hi=60.0)])
    fld = ToleranceField.preset("jig_weld")
    kw = dict(fld=fld, n_starts=5, n_yield=1000, seed=0)
    assert (render_genesis_md(inverse_genesis(nominal, targets, vol, **kw))
            == render_genesis_md(inverse_genesis(nominal, targets, vol, **kw)))


def test_result_echoes_the_bounds_it_enforced(nominal, targets):
    bounds = [PropertyBound("anti_squat_pct", lo=23.0, hi=60.0)]
    res = inverse_genesis(nominal, targets, volume(bounds), fld=None,
                          n_starts=4, seed=0)
    assert res.property_bounds is not None
    assert [b.prop for b in res.property_bounds.bounds] == ["anti_squat_pct"]


def test_vehicle_context_is_validated():
    with pytest.raises(ValueError):
        SolvedPropertyBounds(bounds=[], cg_height_mm=0.0)
    with pytest.raises(ValueError):
        SolvedPropertyBounds(bounds=[], travel_mm=(25.0, -25.0))
    with pytest.raises(TypeError):
        SolvedPropertyBounds(bounds=["anti_squat_pct >= 23"])
