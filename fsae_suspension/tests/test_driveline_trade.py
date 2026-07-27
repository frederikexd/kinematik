# ============================================================================
#  KinematiK — tests for suspension/driveline.py and suspension/trade.py
#
#  The two things worth protecting here:
#   1. the differential model degenerates EXACTLY to an open diff at TBR=1 and
#      EXACTLY to a spool as TBR grows — if those limits drift, every number
#      built on top is unanchored;
#   2. the trade study REFUSES to name a winner when it cannot see one. That
#      refusal is the feature, so it gets a test that fails loudly if some
#      future change makes the tool always produce a ranking.
# ============================================================================
import numpy as np
import pytest

from suspension.dynamics import VehicleParams, VehicleDynamics
from suspension import lapsim
from suspension import driveline as dl
from suspension import trade


def _veh(mass=280.0):
    return VehicleDynamics(VehicleParams(mass=mass, cg_height=300.0,
                                         weight_dist_front=0.46))


# ---------------------------------------------------------------- driveline
def test_open_diff_is_limited_by_the_inside_wheel():
    """The defining property of an open diff: total torque is twice whatever the
    LOW-grip wheel can take, no matter how loaded the outside wheel is."""
    v = _veh()
    r = dl.axle_traction(v, dl.DifferentialSpec("open", kind=dl.OPEN))
    assert r.t_total_nm == pytest.approx(2.0 * r.t_cap_inside_nm, rel=1e-9)
    assert r.lock_ratio == pytest.approx(0.0, abs=1e-9)


def test_spool_uses_both_wheels_fully():
    v = _veh()
    r = dl.axle_traction(v, dl.DifferentialSpec("spool", kind=dl.SPOOL))
    assert r.t_total_nm == pytest.approx(r.t_cap_inside_nm + r.t_cap_outside_nm,
                                         rel=1e-9)
    assert r.t_total_nm > 0


def test_lsd_degenerates_to_open_at_tbr_one():
    """TBR = 1 with no preload must reproduce the open diff EXACTLY — this is
    the anchor that makes the LSD branch trustworthy."""
    v = _veh()
    a = dl.axle_traction(v, dl.DifferentialSpec("open", kind=dl.OPEN))
    b = dl.axle_traction(v, dl.DifferentialSpec(
        "lsd@1", kind=dl.HELICAL_ATB, tbr=1.0, preload_nm=0.0))
    assert b.t_total_nm == pytest.approx(a.t_total_nm, rel=1e-9)


def test_lsd_degenerates_to_spool_at_huge_tbr():
    v = _veh()
    sp = dl.axle_traction(v, dl.DifferentialSpec("spool", kind=dl.SPOOL))
    big = dl.axle_traction(v, dl.DifferentialSpec(
        "lsd@1e6", kind=dl.HELICAL_ATB, tbr=1e6))
    assert big.t_total_nm == pytest.approx(sp.t_total_nm, rel=1e-9)


def test_traction_is_monotonic_in_torque_bias_ratio():
    v = _veh()
    prev = -1.0
    for tbr in (1.0, 1.5, 2.0, 2.5, 3.5, 6.0):
        r = dl.axle_traction(v, dl.DifferentialSpec(
            f"t{tbr}", kind=dl.HELICAL_ATB, tbr=tbr))
        assert r.t_total_nm >= prev - 1e-9, "more bias must never deliver less"
        prev = r.t_total_nm


def test_drive_grip_frac_is_a_usable_lapsim_knob():
    v = _veh()
    for key in ("open", "tre_mk2_center", "spool"):
        f = dl.drive_grip_frac_for(v, dl.catalog(key))
        assert 0.0 <= f <= 1.0
    assert (dl.drive_grip_frac_for(v, dl.catalog("tre_mk2_center"))
            > dl.drive_grip_frac_for(v, dl.catalog("open")))


def test_lock_penalty_is_zero_for_an_open_diff():
    assert dl.lock_lateral_derate(0.0) == pytest.approx(1.0)
    assert dl.lock_lateral_derate(1.0) < 1.0
    # the band must include "no penalty at all" — that honesty is deliberate
    assert dl.LOCK_PENALTY_K_BAND[0] == 0.0


def test_spec_exposes_itself_to_the_integration_ledger():
    s = dl.catalog("tre_mk2_center")
    i = s.to_interface()
    assert i.mass_kg == pytest.approx(4.04, abs=0.01)
    assert i.mounts_on == "chassis"
    assert i.is_estimate is True          # TBR/preload are class estimates


def test_acquisition_cost_includes_the_parts_the_listing_omits():
    s = dl.catalog("tre_mk2_center")
    assert s.sprocket_adapter_included is False
    assert s.total_acquisition_usd() > s.cost_usd
    assert s.extra_fab_hours > 0, "a missing adapter is labour, not a discount"


# -------------------------------------------------------------------- trade
def _setup():
    base = VehicleParams(mass=276.0, cg_height=290.0, weight_dist_front=0.46)
    sim = lapsim.LapSimParams(power_w=55_000.0, mass=base.mass, cl_a=2.2, cd_a=1.1)
    cond = dl.ExitCondition(lateral_g_frac=0.70, cl_a=2.2)
    return base, sim, cond


def test_identical_options_are_not_actionable():
    """Two copies of the same part must NOT produce a winner. If this ever
    passes with a ranking, the tool has started inventing differences."""
    base, sim, cond = _setup()
    a = trade.Option("A", dl.catalog("tre_mk2_center"))
    b = trade.Option("B", dl.catalog("tre_mk2_center"))
    v = trade.compare([a], baseline=b, base_params=base, sim_params=sim,
                      cond=cond, unc=trade.Uncertainty(n_draws=40))
    assert v.results[0].actionable is False
    assert v.results[0].usd_per_point is None
    assert v.best() is None
    assert "NOT SEPARABLE" in v.verdict_text


def test_a_real_difference_is_found_and_priced():
    """A locking diff versus an open diff is a large, robust effect — the tool
    must not be so cautious that it misses one."""
    base, sim, cond = _setup()
    openn = trade.Option("open", dl.catalog("open", mass_kg=3.4, cost_usd=400.0))
    atb = trade.Option("atb", dl.catalog("tre_mk2_center"))
    v = trade.compare([atb], baseline=openn, base_params=base, sim_params=sim,
                      cond=cond, unc=trade.Uncertainty(n_draws=40))
    r = v.results[0]
    assert r.actionable is True
    assert r.delta_points_median > 0
    assert r.delta_points_p05 > 0, "the whole band must stay positive"
    assert r.usd_per_point is not None and r.usd_per_point > 0


def test_usd_per_point_is_withheld_when_not_actionable():
    base, sim, cond = _setup()
    a = trade.Option("cheap", dl.catalog("tre_mk2_center", cost_usd=100.0))
    b = trade.Option("dear", dl.catalog("tre_mk2_center", cost_usd=9999.0))
    v = trade.compare([b], baseline=a, base_params=base, sim_params=sim,
                      cond=cond, unc=trade.Uncertainty(n_draws=40))
    assert all(r.usd_per_point is None for r in v.results)


def test_practical_floor_suppresses_a_trivial_but_stable_difference():
    """A 0.4-point difference can be perfectly sign-stable and still be noise
    next to a real driver. Raising the floor must suppress it."""
    base, sim, cond = _setup()
    openn = trade.Option("open", dl.catalog("open", mass_kg=3.4, cost_usd=400.0))
    atb = trade.Option("atb", dl.catalog("tre_mk2_center"))
    hard = trade.compare([atb], baseline=openn, base_params=base,
                         sim_params=sim, cond=cond,
                         practical_floor_points=1e6,
                         unc=trade.Uncertainty(n_draws=30))
    assert hard.results[0].sign_stable is True     # the model still sees it
    assert hard.results[0].actionable is False     # but it is not a decision


def test_pairwise_separability_is_reported():
    base, sim, cond = _setup()
    openn = trade.Option("open", dl.catalog("open", mass_kg=3.4, cost_usd=400.0))
    atb = trade.Option("atb", dl.catalog("tre_mk2_center"))
    adj = trade.Option("adj", dl.catalog("tre_mk2_center_adj"))
    v = trade.compare([atb, adj], baseline=openn, base_params=base,
                      sim_params=sim, cond=cond,
                      unc=trade.Uncertainty(n_draws=40))
    assert ("atb", "adj") in v.pairwise
    # same physics at nominal preload: the adjuster cannot be justified on points
    assert v.pairwise[("atb", "adj")]["separable"] is False


def test_same_seed_is_reproducible():
    base, sim, cond = _setup()
    openn = trade.Option("open", dl.catalog("open", mass_kg=3.4, cost_usd=400.0))
    atb = trade.Option("atb", dl.catalog("tre_mk2_center"))
    kw = dict(base_params=base, sim_params=sim, cond=cond,
              unc=trade.Uncertainty(n_draws=25, seed=7))
    a = trade.compare([atb], baseline=openn, **kw)
    b = trade.compare([atb], baseline=openn, **kw)
    assert a.results[0].delta_points_median == pytest.approx(
        b.results[0].delta_points_median)


# --------------------------------------------------------------- cost event
def test_cost_event_points_are_withheld_without_the_years_references():
    assert trade.cost_event_points(20_000.0) is None
    assert trade.cost_event_delta(20_000.0, 2_600.0) is None


def test_cost_event_points_score_when_references_are_given():
    p = trade.cost_event_points(20_000.0, min_cost_usd=15_000.0,
                                max_cost_usd=30_000.0)
    assert p == pytest.approx(100.0 * (30_000 - 20_000) / 15_000)
    d = trade.cost_event_delta(20_000.0, 2_200.0, 15_000.0, 30_000.0)
    assert d < 0 and abs(d) < 20, "one part should move Cost points only a little"


def test_provenance_declares_the_refusal_rule():
    for mod in (dl, trade):
        assert "hard_rule" in mod.PROVENANCE
        assert mod.PROVENANCE["estimate_flagged"]
