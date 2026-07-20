# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  tests/test_inverse_genesis.py — the stochastic inverse engine, pinned.
# ============================================================================
"""What these tests guard, in the module's own order of claims:

* the forward curve map solves, interpolates, and refuses cliffs;
* the intent objects validate (zero bands refused, duplicate channels refused);
* the boundary filter: box clamping names its clamps, KeepOutBox signed
  distance is exact inside/outside/probe, keep-out is a wall (no winner may
  violate it — ever);
* the reverse gradients recover a hidden geometry from its curves alone,
  deterministically;
* the co-optimizer: yields in [0,1], the winner is never out-yielded by
  another hit, knife-edges are named, the fit residual genuinely spends band
  headroom (a worse fit can only lower a same-geometry yield);
* honesty: unsatisfiable intent returns ok=False with the binding constraint
  named, no field degrades to fit-only with the warning printed, and the
  report is byte-identical across runs.
"""

import numpy as np
import pytest

from suspension.kinematics import Hardpoints
from suspension.kinematik_stochastic import ToleranceField, _perturbed
from suspension import inverse_genesis as ig


# --------------------------------------------------------------------------- #
#  Shared fixtures — one nominal, one hidden truth, one drawn intent.
# --------------------------------------------------------------------------- #
STATIONS = np.array([-25.0, -12.5, 0.0, 12.5, 25.0])
TRUTH_SHIFT = {"upper_front_inner": np.array([0.0, -4.0, 5.0]),
               "upper_rear_inner":  np.array([0.0, -4.0, 5.0])}


@pytest.fixture(scope="module")
def hp():
    return Hardpoints.default()


@pytest.fixture(scope="module")
def targets(hp):
    truth, ok = ig.curves_of(_perturbed(hp, TRUTH_SHIFT), STATIONS)
    assert ok
    return ig.GenesisTargets(curves=[
        ig.TargetCurve("camber_deg", STATIONS, truth["camber_deg"],
                       np.full(5, 0.15)),
        ig.TargetCurve("toe_deg", STATIONS, truth["toe_deg"],
                       np.full(5, 0.08)),
        ig.TargetCurve("rc_height_mm", STATIONS, truth["rc_height_mm"],
                       np.full(5, 6.0)),
    ])


@pytest.fixture(scope="module")
def volume(hp):
    return ig.LegalVolume.around(
        hp, 8.0, points=["upper_front_inner", "upper_rear_inner"])


# --------------------------------------------------------------------------- #
#  The forward map
# --------------------------------------------------------------------------- #
def test_curves_of_solves_and_matches_stations(hp):
    vals, ok = ig.curves_of(hp, STATIONS)
    assert ok
    for ch in ig.CHANNELS:
        assert vals[ch].shape == STATIONS.shape
        assert np.all(np.isfinite(vals[ch]))


def test_curves_of_refuses_garbage_geometry(hp):
    broken = hp.copy()
    broken.upper_outer = np.asarray(broken.lower_outer, float).copy()
    # coincident ball joints: the validator/solver must refuse, not emit NaNs
    vals, ok = ig.curves_of(broken, STATIONS)
    assert not ok


def test_camber_curve_agrees_with_direct_solver(hp):
    from suspension.kinematics import SuspensionKinematics
    vals, ok = ig.curves_of(hp, STATIONS, n_sweep=41)
    assert ok
    states = SuspensionKinematics(hp).sweep(-25.0, 25.0, 41)
    direct = {round(s.travel, 3): s.camber for s in states}
    for t in (-25.0, 0.0, 25.0):     # stations lying ON the sweep grid
        i = int(np.where(STATIONS == t)[0][0])
        assert vals["camber_deg"][i] == pytest.approx(direct[t], abs=1e-6)


# --------------------------------------------------------------------------- #
#  Intent validation
# --------------------------------------------------------------------------- #
def test_zero_band_refused():
    with pytest.raises(ValueError, match="band"):
        ig.TargetCurve("camber_deg", [0.0], [1.0], [0.0])


def test_duplicate_channel_refused():
    c = ig.TargetCurve("toe_deg", [0.0], [0.0], [0.1])
    with pytest.raises(ValueError, match="twice"):
        ig.GenesisTargets(curves=[c, c])


def test_unknown_channel_refused():
    with pytest.raises(ValueError, match="channel"):
        ig.TargetCurve("wheelbase_mm", [0.0], [0.0], [1.0])


def test_residual_is_band_weighted(hp, targets):
    r, ok = targets.residual(_perturbed(hp, TRUTH_SHIFT))
    assert ok
    assert np.max(np.abs(r)) < 0.5   # the truth sits deep inside its own bands


# --------------------------------------------------------------------------- #
#  The boundary filter
# --------------------------------------------------------------------------- #
def test_legal_volume_rejects_unknown_point(hp):
    with pytest.raises(ValueError, match="designable"):
        ig.LegalVolume(boxes={"contact_patch": (np.zeros(3), np.ones(3))})


def test_clamp_names_its_clamps(hp, volume):
    big = np.full(6, 50.0)           # way past every ±8 mm face
    clamped_vec, names = volume.clamp(hp, big)
    lo, hi = volume.bounds_vec(hp)
    assert np.all(clamped_vec <= hi + 1e-9)
    assert len(names) == 6           # every coordinate pinned, every one named


def test_keepout_box_signed_distance():
    box = ig.KeepOutBox(np.zeros(3), np.full(3, 10.0))
    out = box.clearances(np.array([[15.0, 5.0, 5.0]]))[0]
    assert out == pytest.approx(5.0)
    inside = box.clearances(np.array([[5.0, 5.0, 5.0]]))[0]
    assert inside == pytest.approx(-5.0)     # 5 mm to the nearest face
    probed = box.clearances(np.array([[15.0, 5.0, 5.0]]), 2.0)[0]
    assert probed == pytest.approx(3.0)      # the probe sphere spends 2 mm


def test_keepout_is_a_wall_not_a_penalty(hp, targets):
    """Wall off the truth position; whatever comes back must be violation-free."""
    tgt = np.asarray(_perturbed(hp, TRUTH_SHIFT).upper_front_inner, float)
    vol = ig.LegalVolume.around(
        hp, 8.0, points=["upper_front_inner", "upper_rear_inner"],
        keep_out=[ig.KeepOutBox(tgt - 3.0, tgt + 3.0, label="test header")],
        min_clearance_mm=1.0)
    c = ig.genesis_solve(hp, targets, vol)
    assert c.ok
    hp_c = ig._shifted(hp, vol, c.shift_vec)
    assert vol.keepout_violations(hp_c) == []


# --------------------------------------------------------------------------- #
#  The reverse gradients
# --------------------------------------------------------------------------- #
def test_inverse_solve_recovers_hidden_geometry(hp, targets, volume):
    c = ig.genesis_solve(hp, targets, volume)
    assert c.ok and c.hit
    assert c.max_band_frac <= 1.0
    assert c.iterations >= 1


def test_inverse_solve_is_deterministic(hp, targets, volume):
    a = ig.genesis_solve(hp, targets, volume)
    b = ig.genesis_solve(hp, targets, volume)
    assert np.array_equal(a.shift_vec, b.shift_vec)
    assert a.residual.tolist() == b.residual.tolist()


def test_nominal_already_inside_bands_needs_no_shift(hp, volume):
    nom, ok = ig.curves_of(hp, STATIONS)
    assert ok
    easy = ig.GenesisTargets(curves=[
        ig.TargetCurve("camber_deg", STATIONS, nom["camber_deg"],
                       np.full(5, 0.5))])
    c = ig.genesis_solve(hp, easy, volume)
    assert c.hit and c.shifts == {}


# --------------------------------------------------------------------------- #
#  The co-optimizer
# --------------------------------------------------------------------------- #
@pytest.fixture(scope="module")
def result(hp, targets, volume):
    fld = ToleranceField.preset("hand_weld", weld_pull_mm=1.0, pull_axis="z")
    return ig.inverse_genesis(hp, targets, volume, fld=fld,
                              n_starts=5, n_yield=2000, n_verify_full=40,
                              seed=0)


def test_winner_exists_and_yield_bounded(result):
    assert result.winner is not None
    y = result.winner.yield_frac
    assert y is not None and 0.0 <= y <= 1.0


def test_winner_is_never_out_yielded_by_another_hit(result):
    for c in result.candidates:
        if c.hit and c.yield_frac is not None:
            assert result.winner.yield_frac >= c.yield_frac - 1e-12


def test_every_candidate_carries_a_verdict(result):
    assert all(c.verdict in ("RESILIENT", "TEMPERED", "KNIFE_EDGE", "NO_FIT")
               for c in result.candidates)


def test_fit_residual_spends_band_headroom(hp, targets):
    """Same geometry, same field: a fabricated worse fit can only cost yield —
    the coupling that makes the co-optimizer honest."""
    fld = ToleranceField.preset("hand_weld")
    hp_t = _perturbed(hp, TRUTH_SHIFT)
    r0, ok = targets.residual(hp_t)
    assert ok
    y_center, _ = ig.build_yield(hp_t, targets, fld, r0, n=2000, seed=0)
    y_edge, _ = ig.build_yield(hp_t, targets, fld,
                               np.full_like(r0, 0.9), n=2000, seed=0)
    assert y_center is not None and y_edge is not None
    assert y_edge <= y_center + 1e-12


def test_cnc_shop_out_yields_hand_weld(hp, targets, volume):
    """A tighter field can never lower the winning yield — the control case."""
    res_hand = ig.inverse_genesis(hp, targets, volume,
                                  fld=ToleranceField.preset("hand_weld"),
                                  n_starts=3, n_yield=1500, seed=0)
    res_cnc = ig.inverse_genesis(hp, targets, volume,
                                 fld=ToleranceField.preset("cnc"),
                                 n_starts=3, n_yield=1500, seed=0)
    assert res_cnc.winner.yield_frac >= res_hand.winner.yield_frac - 1e-12


def test_verification_is_priced(result):
    assert result.verify_yield is not None
    assert result.verify_agreement is not None
    assert 0.0 <= result.verify_agreement <= 1.0


# --------------------------------------------------------------------------- #
#  Honesty
# --------------------------------------------------------------------------- #
def test_unsatisfiable_intent_is_named(hp, targets, volume):
    impossible = ig.GenesisTargets(curves=[
        ig.TargetCurve("camber_deg", STATIONS,
                       targets.curves[0].target + 25.0, np.full(5, 0.1))])
    res = ig.inverse_genesis(hp, impossible, volume,
                             fld=ToleranceField.preset("cnc"),
                             n_starts=3, n_yield=300, seed=0)
    assert not res.ok and res.winner is None
    assert "unsatisfiable" in res.reason
    assert res.best_fit is not None          # the closest legal car is offered


def test_no_field_degrades_honestly(hp, targets, volume):
    res = ig.inverse_genesis(hp, targets, volume, fld=None,
                             n_starts=3, seed=0)
    assert res.ok and res.winner is not None
    assert res.winner.yield_frac is None
    assert any("buildability question was not asked" in w
               for w in res.warnings)


def test_report_is_byte_identical_across_runs(hp, targets, volume):
    fld = ToleranceField.preset("hand_weld", weld_pull_mm=0.8, pull_axis="z")
    kw = dict(fld=fld, n_starts=4, n_yield=1500, n_verify_full=30, seed=0)
    a = ig.render_genesis_md(ig.inverse_genesis(hp, targets, volume, **kw))
    b = ig.render_genesis_md(ig.inverse_genesis(hp, targets, volume, **kw))
    assert a == b


def test_report_names_the_scope(result):
    md = ig.render_genesis_md(result)
    assert "NEIGHBOURING assemblies" in md
    assert "Ghost Topology" in md
