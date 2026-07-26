# ============================================================================
#  KinematiK — tests for suspension/proof_engine.py
#
#  The proof engine sells three promises, and each has a test pinning it:
#    1. determinism — same ledger in, same plan out, always;
#    2. honesty     — grades never silently upgrade, staleness never shrinks,
#                     re-proving the already-proven is never recommended;
#    3. pre-registration — a sealed contract cannot be judged after tampering,
#                     and the DISCREPANT verdict fires exactly when the run
#                     and the ledger disagree beyond the plausibility envelope.
# ============================================================================

import datetime as dt

import pytest

from suspension.interfaces import SubsystemInterface, blank_ledger
from suspension.proof_engine import (
    EvidenceGrade, Quantity, ValidationContract, Verdict,
    DEFAULT_ACTIONS, DEFAULT_OBJECTIVES,
    analyze_objective, aggregate, build_uncertainty_ledger, create_contract,
    effective_rel_unc, judge_result, plan_proofs,
    render_contract_brief_md, render_proof_plan_md,
)

TODAY = dt.date(2026, 7, 17)


def _ledger():
    led = blank_ledger()
    led.set(SubsystemInterface(name="chassis", mass_kg=45.0, cg_z_mm=300.0,
                               is_estimate=True))
    led.set(SubsystemInterface(name="powertrain", mass_kg=60.0, cg_z_mm=320.0,
                               peak_power_kw=68.0, heat_reject_w=3200.0,
                               is_estimate=True))
    led.set(SubsystemInterface(name="cooling", mass_kg=6.0,
                               cooling_airflow_cms=0.14, is_estimate=False))
    return led


def _laptime():
    return next(o for o in DEFAULT_OBJECTIVES if o.key == "laptime_s")


# --------------------------------------------------------------------------- #
#  Grades & staleness
# --------------------------------------------------------------------------- #
def test_grade_uncertainty_ordering():
    """Better evidence must always mean a tighter band."""
    uncs = [EvidenceGrade.GUESS.base_rel_unc, EvidenceGrade.ESTIMATE.base_rel_unc,
            EvidenceGrade.MODELLED.base_rel_unc, EvidenceGrade.MEASURED.base_rel_unc,
            EvidenceGrade.VERIFIED.base_rel_unc]
    assert uncs == sorted(uncs, reverse=True)


def test_staleness_inflates_and_caps():
    fresh = effective_rel_unc(EvidenceGrade.MEASURED, 0)
    halflife = effective_rel_unc(EvidenceGrade.MEASURED, 180)
    ancient = effective_rel_unc(EvidenceGrade.MEASURED, 100_000)
    assert halflife == pytest.approx(2 * fresh)
    assert ancient == EvidenceGrade.GUESS.base_rel_unc  # never worse than a guess
    # a guess does not rot — it is already maximally uncertain
    assert effective_rel_unc(EvidenceGrade.GUESS, 10_000) == \
        EvidenceGrade.GUESS.base_rel_unc


def test_quantity_age_uses_measured_on():
    q = Quantity("chassis.mass_kg", "chassis", "mass_kg", "chassis mass",
                 45.0, "kg", EvidenceGrade.MEASURED, measured_on="2026-01-17")
    assert q.age_days(TODAY) == pytest.approx(181, abs=1)
    assert q.rel_unc(TODAY) > EvidenceGrade.MEASURED.base_rel_unc


# --------------------------------------------------------------------------- #
#  Ledger seeding
# --------------------------------------------------------------------------- #
def test_seeding_never_claims_measured():
    """is_estimate=False seeds MODELLED — a checkbox is not an instrument."""
    qs = build_uncertainty_ledger(_ledger())
    grades = {q.key: q.grade for q in qs}
    assert grades["chassis.mass_kg"] == EvidenceGrade.ESTIMATE
    assert grades["cooling.mass_kg"] == EvidenceGrade.MODELLED
    assert all(g.rank <= EvidenceGrade.MODELLED.rank for g in grades.values())


def test_overrides_apply():
    ov = {"chassis.mass_kg": {"grade": "measured", "measured_on": "2026-07-01",
                              "source": "corner scales"}}
    qs = build_uncertainty_ledger(_ledger(), overrides=ov)
    q = next(x for x in qs if x.key == "chassis.mass_kg")
    assert q.grade == EvidenceGrade.MEASURED
    assert q.source == "corner scales"


# --------------------------------------------------------------------------- #
#  Aggregation & attribution
# --------------------------------------------------------------------------- #
def test_aggregate_sums_and_mass_weights():
    qs = build_uncertainty_ledger(_ledger())
    car = aggregate(qs)
    assert car["mass_kg"] == pytest.approx(111.0)
    # mass-weighted CG of chassis(45@300) + powertrain(60@320); cooling has no cg
    assert car["cg_z_mm"] == pytest.approx((45 * 300 + 60 * 320) / 105.0)


def test_attribution_is_deterministic_and_normalised():
    qs = build_uncertainty_ledger(_ledger())
    r1 = analyze_objective(_laptime(), qs, today=TODAY)
    r2 = analyze_objective(_laptime(), qs, today=TODAY)
    assert r1.total_unc == r2.total_unc
    assert [a.delta_out for a in r1.attributions] == \
        [a.delta_out for a in r2.attributions]
    assert sum(a.share for a in r1.attributions) == pytest.approx(1.0)
    # attributions arrive sorted, dominant first
    deltas = [a.delta_out for a in r1.attributions]
    assert deltas == sorted(deltas, reverse=True)


def test_better_evidence_tightens_the_objective():
    qs = build_uncertainty_ledger(_ledger())
    loose = analyze_objective(_laptime(), qs, today=TODAY).total_unc
    for q in qs:
        q.grade = EvidenceGrade.MEASURED
        q.measured_on = TODAY.isoformat()
    tight = analyze_objective(_laptime(), qs, today=TODAY).total_unc
    assert tight < loose


# --------------------------------------------------------------------------- #
#  Planning
# --------------------------------------------------------------------------- #
def test_plan_ranks_by_value_per_hour_and_hits_floor():
    qs = build_uncertainty_ledger(_ledger())
    plan = plan_proofs(_laptime(), qs, today=TODAY)
    vals = [it.value_per_hour for it in plan.items]
    assert vals == sorted(vals, reverse=True)
    assert plan.unc_floor <= plan.unc_now
    # with a ±20% power estimate feeding a lap-time objective, the dyno pull
    # must outrank the flow bench (which can't touch lap time at all)
    keys = [it.action_key for it in plan.items]
    assert keys.index("dyno_pull") < keys.index("flow_bench")


def test_plan_never_recommends_reproving_the_proven():
    """A channel already VERIFIED gains nothing from a MEASURED-grade action."""
    qs = build_uncertainty_ledger(_ledger())
    for q in qs:
        if q.key == "powertrain.peak_power_kw":
            q.grade = EvidenceGrade.VERIFIED
            q.measured_on = TODAY.isoformat()
    plan = plan_proofs(_laptime(), qs, today=TODAY)
    dyno = next((it for it in plan.items if it.action_key == "dyno_pull"), None)
    assert dyno is None or "powertrain.peak_power_kw" not in dyno.affected


# --------------------------------------------------------------------------- #
#  Contracts — pre-registration
# --------------------------------------------------------------------------- #
def _quantity():
    return Quantity("suspension.mount_load_n", "suspension", "mount_load_n",
                    "suspension peak mount load", 4000.0, "N",
                    EvidenceGrade.MODELLED, measured_on=TODAY.isoformat())


def _action():
    return next(a for a in DEFAULT_ACTIONS if a.key == "ansys_static")


def test_contract_seal_roundtrip_and_inverted_band_refused():
    c = create_contract(_action(), _quantity(), 3200, 4800,
                        "FoS ≥ 1.5 on the M8 bracket at this load", "lead",
                        today=TODAY)
    assert c.verify_seal()
    # survives persistence
    c2 = ValidationContract.from_dict(c.as_dict())
    assert c2.verify_seal()
    with pytest.raises(ValueError):
        create_contract(_action(), _quantity(), 4800, 3200, "inverted",
                        today=TODAY)


def test_tampered_contract_refuses_judgment():
    c = create_contract(_action(), _quantity(), 3200, 4800, "band", today=TODAY)
    c.pass_hi = 99_999.0            # moving the goalpost after sealing
    assert not c.verify_seal()
    with pytest.raises(ValueError):
        judge_result(c, 5000.0, today=TODAY)


def test_three_way_verdict():
    q = _quantity()                  # 4000 ± 10% → plausibility 2800..5200
    c = create_contract(_action(), q, 3200, 4800, "band", today=TODAY)
    assert judge_result(c, 4100.0, today=TODAY).status == Verdict.PASS.value
    assert judge_result(c, 5000.0, today=TODAY).status == Verdict.FAIL.value
    d = judge_result(c, 9000.0, today=TODAY)
    assert d.status == Verdict.DISCREPANT.value
    assert "disagree" in d.judgment_note
    # judging never breaks the seal — the goalposts provably never moved
    assert d.verify_seal()


def test_judgment_does_not_mutate_input_contract():
    c = create_contract(_action(), _quantity(), 3200, 4800, "band", today=TODAY)
    judge_result(c, 4100.0, today=TODAY)
    assert c.status == Verdict.OPEN.value and c.result_value is None


# --------------------------------------------------------------------------- #
#  Exports
# --------------------------------------------------------------------------- #
def test_markdown_exports_carry_the_promises():
    qs = build_uncertainty_ledger(_ledger())
    rep = analyze_objective(_laptime(), qs, today=TODAY)
    plan = plan_proofs(_laptime(), qs, today=TODAY)
    md = render_proof_plan_md(plan, rep, frame_note="ISO 8855 (charter v2)")
    assert "Proof Plan" in md and "ranked by certainty" in md
    assert "ISO 8855" in md and "deterministic" in md

    c = create_contract(_action(), _quantity(), 3200, 4800,
                        "FoS ≥ 1.5", today=TODAY)
    brief = render_contract_brief_md(c, _action(), frame_note="ISO 8855")
    assert c.seal[:16] in brief
    assert "Pre-registered acceptance band" in brief
    assert "DISCREPANT" in brief
