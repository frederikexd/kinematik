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


# --------------------------------------------------------------------------- #
#  Manifest round-trip — a bounded run must replay as a bounded run
# --------------------------------------------------------------------------- #
def test_manifest_carries_the_bounds(nominal, targets):
    """A manifest that forgot the bounds would replay as an unbounded run."""
    from suspension import genesis_repro as gr

    bounds = [PropertyBound("anti_squat_pct", lo=23.0, hi=60.0),
              PropertyBound("caster_deg", hi=6.0)]           # one-sided
    vol = volume(bounds)
    man = gr.GenesisManifest.build(
        "bounded", nominal, targets, vol, ToleranceField.preset("jig_weld"))
    back = gr.GenesisManifest.from_json(man.to_json())
    _, _, vol2, _ = back.objects()

    assert vol2.properties is not None
    assert vol2.has_property_bounds()
    got = {b.prop: (b.lo, b.hi) for b in vol2.properties.bounds}
    assert got["anti_squat_pct"] == (23.0, 60.0)
    # the infinity survived a JSON round-trip as an infinity, not as 0 or null
    assert got["caster_deg"][0] == float("-inf")
    assert got["caster_deg"][1] == 6.0
    # the vehicle the bounds were checked against travels with them
    p = vol2.properties
    assert p.cg_height_mm == CTX["cg_height_mm"]
    assert p.wheelbase_mm == CTX["wheelbase_mm"]
    assert p.track_mm == CTX["track_mm"]
    assert p.drive_bias_rear == CTX["drive_bias_rear"]
    assert p.travel_mm == (-25.0, 25.0)
    assert p.n_nodes == 5


def test_unbounded_manifest_bytes_are_unchanged(nominal, targets):
    """No bounds declared → no new key, so old manifests keep their hash."""
    from suspension import genesis_repro as gr

    vol = volume(None)
    d = gr.volume_to_dict(vol)
    assert "properties" not in d, (
        "an unbounded run must not gain a key, or every archived "
        "inputs_sha256 changes")
    assert gr.volume_from_dict(d).properties is None


def test_legacy_manifest_without_properties_still_loads(nominal, targets):
    """A manifest written before this feature existed must still replay."""
    from suspension import genesis_repro as gr

    man = gr.GenesisManifest.build(
        "legacy", nominal, targets, volume(None), None)
    d = man.to_json()
    assert '"properties"' not in d
    _, _, vol2, _ = gr.GenesisManifest.from_json(d).objects()
    assert vol2.properties is None
    assert not vol2.has_property_bounds()


def test_bounds_change_the_inputs_hash(nominal, targets):
    """Two runs that differ only in a bound must not share a manifest hash."""
    from suspension import genesis_repro as gr

    a = gr.GenesisManifest.build("a", nominal, targets, volume(None), None)
    b = gr.GenesisManifest.build(
        "b", nominal, targets,
        volume([PropertyBound("anti_squat_pct", lo=23.0, hi=60.0)]), None)
    assert a.inputs_sha256 != b.inputs_sha256


# --------------------------------------------------------------------------- #
#  Rear anti-lift: its own formula, its own sign
# --------------------------------------------------------------------------- #
def _rear_hp():
    from suspension.kinematics import Hardpoints
    d = dict(upper_front_inner=[-410.4, 326.8, 321.7],
             upper_rear_inner=[-127.4, 216.6, 285.7],
             lower_front_inner=[-268.8, 220.0, 58.9],
             lower_rear_inner=[5.6, 165.0, 127.6],
             tie_rod_inner=[-50.5, 310.0, 118.9],
             upper_outer=[3.0, 571.0, 300.0], lower_outer=[-5.0, 590.2, 120.0],
             tie_rod_outer=[90.0, 579.2, 150.0],
             wheel_center=[0.0, 600.0, 228.0], contact_patch=[0.0, 605.0, 0.0])
    hp = Hardpoints.default()
    for k, v in d.items():
        setattr(hp, k, np.array(v, float))
    hp.static_camber, hp.static_toe = -2.6, 0.0
    return hp


def test_anti_lift_is_opposite_sign_to_anti_dive_formula():
    """Same path slope, opposite load-transfer sense: the rear is not the front."""
    from suspension.kinematics import SuspensionKinematics
    k = SuspensionKinematics(_rear_hp())
    ad = k.anti_dive_pct(280.0, 1630.0, 0.40)
    al = k.anti_lift_pct(280.0, 1630.0, 0.40)
    assert al == pytest.approx(-ad, rel=1e-9)


def test_delivered_rear_is_pro_lift():
    """Its patch moves forward in bump (SVIC behind the patch): pro-lift."""
    from suspension.kinematics import SuspensionKinematics
    k = SuspensionKinematics(_rear_hp())
    assert k.anti_lift_pct(280.0, 1630.0, 0.40) == pytest.approx(-111.3, abs=0.2)


def test_anti_lift_is_a_boundable_property():
    from suspension.inverse_genesis import properties_of, SolvedPropertyBounds
    ctx = SolvedPropertyBounds(bounds=[], cg_height_mm=280.0,
                               wheelbase_mm=1630.0, track_mm=1210.0,
                               brake_bias_front=0.60, drive_bias_rear=1.0)
    p = properties_of(_rear_hp(), ctx, only=["anti_lift_pct"])
    assert p["anti_lift_pct"] == pytest.approx(-111.3, abs=0.2)
    PropertyBound("anti_lift_pct", lo=0.0, hi=100.0)   # accepted


def test_rear_review_matches_the_kinematic_core():
    """The review's rear branch used the front sign and misreported anti-squat."""
    from suspension import genesis_repro as gr
    from suspension.kinematics import SuspensionKinematics
    hp = _rear_hp()
    d = gr.corner_diagnostics(hp, travel_mm=25.0, track_mm=1210.0, axle="rear",
                              wheelbase_mm=1630.0, cg_height_mm=280.0,
                              brake_bias_front=0.60)
    k = SuspensionKinematics(hp)
    assert d["anti_squat_pct"] == pytest.approx(
        k.anti_squat_pct(280.0, 1630.0, 1.0), abs=1e-6)
    assert d["anti_squat_pct"] == pytest.approx(-25.7, abs=0.2)
    assert d["anti_lift_pct"] == pytest.approx(-111.3, abs=0.2)
    assert "anti_dive_pct" not in d
