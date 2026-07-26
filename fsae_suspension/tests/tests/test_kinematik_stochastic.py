# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  tests/test_kinematik_stochastic.py — the Stochastic Inversion suite.
#  Pins: determinism (byte-identical yields and markdown), the tolerance
#  field's closed-form moments, the sensitivity Jacobian against direct
#  solves, the yield's monotonicity in the field, the self-pricing of the
#  linearisation, every verdict boundary, the robust nudge (asymmetric field
#  re-centred, centred field honestly refused, the goalpost fix), and the
#  alignment prescription (known injected error cancelled, quantisation,
#  unreachable metrics named, tampered inputs refused).
# ============================================================================
import numpy as np
import pytest

from suspension.kinematics import Hardpoints
from suspension import kinematik_stochastic as ks


@pytest.fixture(scope="module")
def hp():
    return Hardpoints.default()


@pytest.fixture(scope="module")
def small_field():
    """A modest hand-shop field — big enough to matter, small enough to stay
    well inside the solver's linear-ish neighbourhood."""
    return ks.ToleranceField.preset("jig_weld")


@pytest.fixture(scope="module")
def sens(hp, small_field):
    return ks.sensitivity(hp, small_field)


# --------------------------------------------------------------------------- #
#  The tolerance field.
# --------------------------------------------------------------------------- #
def test_spec_rejects_inverted_bounds():
    with pytest.raises(ValueError):
        ks.ToleranceSpec(lo=np.array([1.0, 0, 0]), hi=np.zeros(3))


def test_spec_moments_uniform():
    s = ks.ToleranceSpec(lo=np.array([-1.0, -2.0, 0.0]),
                         hi=np.array([3.0, 2.0, 0.0]))
    assert np.allclose(s.mean, [1.0, 0.0, 0.0])
    # Var of U(a,b) = (b-a)^2/12; a zero-span axis has zero variance
    assert np.allclose(s.var, [16.0 / 12.0, 16.0 / 12.0, 0.0])


def test_sampler_respects_bounds_and_bias():
    s = ks.ToleranceSpec(lo=np.array([-0.3, -0.3, -0.3]),
                         hi=np.array([1.5, 0.3, 0.3]))
    fld = ks.ToleranceField({"upper_front_inner": s})
    x = fld.sample(4000, seed=7)
    assert x.shape == (4000, 3)
    assert np.all(x[:, 0] >= -0.3 - 1e-12) and np.all(x[:, 0] <= 1.5 + 1e-12)
    # empirical mean tracks the closed-form mean of the asymmetric axis
    assert abs(np.mean(x[:, 0]) - 0.6) < 0.05


def test_sampler_deterministic():
    fld = ks.ToleranceField.preset("hand_weld")
    a = fld.sample(500, seed=3)
    b = fld.sample(500, seed=3)
    assert np.array_equal(a, b)
    assert not np.array_equal(a, fld.sample(500, seed=4))


def test_field_rejects_unknown_point():
    with pytest.raises(ValueError):
        ks.ToleranceField({"flux_capacitor": ks.ToleranceSpec.symmetric(1.0)})


def test_preset_classes_are_physical():
    """Welded tabs carry the weld-class bound; the rack and the machined
    upright points carry the machining class — the tie rod inner is NOT a
    weld tab."""
    fld = ks.ToleranceField.preset("hand_weld")
    assert fld.specs["upper_front_inner"].hi[0] == pytest.approx(1.5)
    assert fld.specs["tie_rod_inner"].hi[0] == pytest.approx(0.2)


# --------------------------------------------------------------------------- #
#  Metrics and sensitivity.
# --------------------------------------------------------------------------- #
def test_metrics_solve_and_are_finite(hp):
    m, ok = metrics = ks.metrics_of(hp)
    assert ok and np.all(np.isfinite(m)) and m.shape == (len(ks.METRICS),)


def test_sensitivity_matches_direct_solve(hp, sens):
    """J's tie-rod-inner z column must predict the actual metric change of a
    small real perturbation — the Jacobian is a derivative, not a vibe."""
    j = sens.coords.index(("tie_rod_inner", 2))
    d = 0.2
    m1, ok = ks.metrics_of(ks._perturbed(hp, {"tie_rod_inner":
                                              np.array([0, 0, d])}))
    assert ok
    pred = sens.nominal + sens.J[:, j] * d
    assert np.allclose(m1, pred, atol=5e-3)


def test_bump_steer_feels_the_tie_rod(sens):
    """Physics sanity: bump steer must be sensitive to the tie-rod inner
    height — the classic bump-steer lever."""
    i = ks.METRICS.index("bump_steer_deg")
    j = sens.coords.index(("tie_rod_inner", 2))
    assert abs(sens.J[i, j]) > 0.01


# --------------------------------------------------------------------------- #
#  The sweep.
# --------------------------------------------------------------------------- #
def test_sweep_deterministic(hp, small_field, sens):
    r1 = ks.stochastic_sweep(hp, small_field, n=800, seed=1, sens=sens,
                             n_verify=10)
    r2 = ks.stochastic_sweep(hp, small_field, n=800, seed=1, sens=sens,
                             n_verify=10)
    assert r1.yield_frac == r2.yield_frac
    assert np.array_equal(r1.fail_frac_per_metric, r2.fail_frac_per_metric)
    md1 = ks.render_stochastic_md(r1)
    md2 = ks.render_stochastic_md(r2)
    assert md1 == md2


def test_zero_field_yields_unity(hp):
    fld = ks.ToleranceField({"upper_front_inner":
                             ks.ToleranceSpec.symmetric(0.0)})
    r = ks.stochastic_sweep(hp, fld, n=200, n_verify=5)
    assert r.yield_frac == pytest.approx(1.0)
    assert r.verdict == "ROBUST"


def test_yield_monotone_in_field(hp):
    """A bigger error field can never raise the yield (same seed, same
    bands)."""
    y = []
    for r_mm in (0.2, 0.8, 2.0):
        fld = ks.ToleranceField({p: ks.ToleranceSpec.symmetric(r_mm)
                                 for p in ks.PERTURBABLE_POINTS})
        y.append(ks.stochastic_sweep(hp, fld, n=1500, seed=0,
                                     n_verify=8).yield_frac)
    assert y[0] >= y[1] >= y[2]


def test_linearisation_is_priced(hp, small_field, sens):
    r = ks.stochastic_sweep(hp, small_field, n=600, sens=sens, n_verify=25)
    assert r.mode == "linear"
    assert r.verify_agreement is not None
    assert r.verify_agreement >= 0.9         # jig field is near-linear
    assert r.verify_worst_err is not None
    assert np.all(np.isfinite(r.verify_worst_err))


def test_full_mode_agrees_with_linear(hp, small_field, sens):
    rl = ks.stochastic_sweep(hp, small_field, n=250, seed=2, sens=sens,
                             n_verify=10)
    rf = ks.stochastic_sweep(hp, small_field, n=250, seed=2, sens=sens,
                             mode="full")
    assert rf.verify_agreement is None       # full mode prices nothing extra
    assert abs(rl.yield_frac - rf.yield_frac) < 0.05


def test_attribution_rows_sum_to_one(hp, small_field, sens):
    r = ks.stochastic_sweep(hp, small_field, n=400, sens=sens, n_verify=5)
    sums = np.sum(r.attribution, axis=1)
    assert np.allclose(sums, 1.0, atol=1e-9)


def test_verdict_boundaries(hp, small_field, sens):
    """Verdicts sit exactly at the documented thresholds — driven through a
    tight band spec so the boundary is exercised, not simulated."""
    r = ks.stochastic_sweep(hp, small_field, n=1200, sens=sens, n_verify=6)
    th = ks.StochasticThresholds()
    if r.yield_frac >= th.robust_yield:
        assert r.verdict == "ROBUST"
    # squeeze the bands until it must go FRAGILE
    tight = ks.YieldSpec(camber_bump_deg=1e-4, bump_steer_deg=1e-4,
                         rc_height_mm=1e-3, scrub_mm=1e-3, caster_deg=1e-4)
    rf = ks.stochastic_sweep(hp, small_field, tight, n=1200, sens=sens,
                             n_verify=6)
    assert rf.verdict == "FRAGILE"
    assert rf.yield_frac < th.marginal_yield


def test_empty_field_refused(hp):
    with pytest.raises(ValueError):
        ks.stochastic_sweep(hp, ks.ToleranceField({}), n=10)


def test_bad_mode_refused(hp, small_field):
    with pytest.raises(ValueError):
        ks.stochastic_sweep(hp, small_field, n=10, mode="vibes")


# --------------------------------------------------------------------------- #
#  The robust nudge.
# --------------------------------------------------------------------------- #
def test_nudge_refuses_centred_field(hp, small_field, sens):
    r = ks.stochastic_sweep(hp, small_field, n=600, sens=sens, n_verify=5)
    nud = ks.robust_nudge(hp, small_field, r)
    assert not nud.ok
    assert "centred" in nud.reason
    assert nud.shifts == {}


def test_nudge_recentres_asymmetric_field(hp):
    """A z weld draw biases the population; the nudge must raise the linear
    yield and cancel most of the bias."""
    fld = ks.ToleranceField.preset("hand_weld", weld_pull_mm=1.2,
                                   pull_axis="z")
    r = ks.stochastic_sweep(hp, fld, n=2500, seed=0, n_verify=15)
    nud = ks.robust_nudge(hp, fld, r, freedom_mm=3.0)
    assert nud.ok
    assert nud.predicted_yield > nud.baseline_yield + 0.02
    # the residual expected bias after the nudge is a fraction of the raw one
    x = np.zeros(len(r.sens.coords))
    for j, (p, a) in enumerate(r.sens.coords):
        if p in nud.shifts:
            x[j] = nud.shifts[p][a]
    resid = r.sens.J @ (fld.mean_vec() + x)
    raw = r.sens.J @ fld.mean_vec()
    assert np.linalg.norm(resid / r.bands) < 0.5 * np.linalg.norm(raw / r.bands)


def test_nudge_verification_judges_original_intent(hp):
    """The goalpost fix: the full-solve verification of the nudged nominal
    must judge against the ORIGINAL design intent. If it judged against the
    shifted nominal's own metrics, the bias would be silently re-introduced
    and the verified yield would collapse back to the baseline."""
    fld = ks.ToleranceField.preset("hand_weld", weld_pull_mm=1.2,
                                   pull_axis="z")
    r = ks.stochastic_sweep(hp, fld, n=2000, seed=0, n_verify=10)
    nud = ks.robust_nudge(hp, fld, r, freedom_mm=3.0, n_verify_full=120)
    assert nud.verified_yield is not None
    assert abs(nud.verified_yield - nud.predicted_yield) < 0.08
    assert nud.verified_yield > nud.baseline_yield


def test_nudge_freedom_clamps_are_named(hp):
    fld = ks.ToleranceField.preset("hand_weld", weld_pull_mm=2.5,
                                   pull_axis="z")
    r = ks.stochastic_sweep(hp, fld, n=800, seed=0, n_verify=5)
    nud = ks.robust_nudge(hp, fld, r, freedom_mm=0.05)
    assert nud.ok
    assert len(nud.clamped) > 0
    for lab in nud.clamped:
        p, ax = lab.rsplit(".", 1)
        assert abs(nud.shifts[p][{"x": 0, "y": 1, "z": 2}[ax]]) \
            == pytest.approx(0.05)


# --------------------------------------------------------------------------- #
#  The alignment prescription.
# --------------------------------------------------------------------------- #
_ADJ = [ks.Adjuster("tie_rod_inner", "z", -3, 3, 0.25, "tie-rod inner shim"),
        ks.Adjuster("upper_rear_inner", "x", -3, 3, 0.5, "upper aft shims"),
        ks.Adjuster("lower_rear_inner", "x", -3, 3, 0.5, "lower aft shims")]


def test_prescription_restores_injected_error(hp):
    """Inject a known weld pull; the prescription must bring every metric
    back inside its band, verified by the full re-solve it reports."""
    ab = {"upper_front_inner":
          np.asarray(hp.upper_front_inner, float) + np.array([1.4, 0, 0])}
    rx = ks.alignment_prescription(hp, ab, _ADJ)
    assert rx.verdict == "RESTORED"
    assert np.all(np.abs(rx.delta_after) <= rx.bands)
    # residuals shrank (or stayed) on every metric
    assert np.all(np.abs(rx.delta_after) <= np.abs(rx.delta_before) + 1e-9)


def test_prescription_moves_are_quantised_and_clamped(hp):
    ab = {"upper_front_inner":
          np.asarray(hp.upper_front_inner, float) + np.array([0.9, 0, 0.6])}
    rx = ks.alignment_prescription(hp, ab, _ADJ)
    for adj, mv in zip(rx.adjusters, rx.moves_mm):
        assert adj.lo - 1e-9 <= mv <= adj.hi + 1e-9
        if adj.step:
            assert abs(mv / adj.step - round(mv / adj.step)) < 1e-9


def test_prescription_names_unreachable_metric(hp):
    """An adjuster set that cannot touch scrub radius must say so when the
    as-built error lives there — never round the residual away."""
    ab = {"lower_outer":
          np.asarray(hp.lower_outer, float) + np.array([0.0, 8.0, 0.0])}
    only_tie = [ks.Adjuster("tie_rod_inner", "z", -3, 3, 0.25)]
    rx = ks.alignment_prescription(hp, ab, only_tie,
                                   ks.YieldSpec(scrub_mm=1.0))
    assert rx.verdict in ("UNSHIMMABLE", "PARTIAL")
    assert abs(rx.delta_after[ks.METRICS.index("scrub_mm")]) > 1.0


def test_prescription_refuses_garbage_inputs(hp):
    with pytest.raises(ValueError):
        ks.alignment_prescription(hp, {"flux_capacitor": np.zeros(3)}, _ADJ)
    with pytest.raises(ValueError):
        ks.alignment_prescription(hp, {}, [])
    # an as-built sheet in metres (a units slip) must be refused, not shimmed
    ab_m = {"upper_front_inner":
            np.asarray(hp.upper_front_inner, float) / 1000.0}
    with pytest.raises(ValueError):
        ks.alignment_prescription(hp, ab_m, _ADJ)


def test_adjuster_direction_normalised_and_validated():
    a = ks.Adjuster("tie_rod_inner", np.array([0.0, 3.0, 4.0]))
    assert np.allclose(np.linalg.norm(a.direction()), 1.0)
    with pytest.raises(ValueError):
        ks.Adjuster("tie_rod_inner", np.zeros(3)).direction()


# --------------------------------------------------------------------------- #
#  Reports.
# --------------------------------------------------------------------------- #
def test_reports_render_and_are_deterministic(hp, small_field, sens):
    r = ks.stochastic_sweep(hp, small_field, n=400, sens=sens, n_verify=5)
    nud = ks.robust_nudge(hp, small_field, r)
    md = ks.render_stochastic_md(r, nud, title="test corner")
    assert "Stochastic Inversion" in md and "Verdict" in md
    assert md == ks.render_stochastic_md(r, nud, title="test corner")

    ab = {"upper_front_inner":
          np.asarray(hp.upper_front_inner, float) + np.array([1.0, 0, 0])}
    rx = ks.alignment_prescription(hp, ab, _ADJ)
    pmd = ks.render_prescription_md(rx, title="test corner")
    assert "Alignment Prescription" in pmd and "full nonlinear re-solve" in pmd
