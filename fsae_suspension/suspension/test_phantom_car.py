# ============================================================================
#  KinematiK — tests for suspension/phantom_car.py
#
#  The Phantom Car sells five promises, and each has tests pinning it:
#    1. determinism    — same charter + declarations + ledger in, same docket
#                        out, byte for byte;
#    2. verdicts       — a hedge to exactly the charter percentile is ALIGNED,
#                        nominal is NAKED, far past is STACKED with the excess
#                        priced as releasable envelope, a favourable value is
#                        ANTI-HEDGED — each at its documented σ threshold;
#    3. the two cars   — two consumers assuming the same quantity >1σ apart is
#                        flagged contradictory, with both consumers named;
#    4. β & three cars — the FORM index is √(Σ z²) over stacked worst cases,
#                        and the phantom car is the union of adverse extremes;
#    5. honesty & seal — unresolved keys and unaudited consumers are reported
#                        not hidden, and an edited charter refuses to judge,
#                        same as a tampered validation contract.
# ============================================================================

import datetime as dt

import pytest

from suspension.interfaces import SubsystemInterface, blank_ledger
from suspension.proof_engine import build_uncertainty_ledger
from suspension.phantom_car import (
    MarginCharter, MarginDeclaration, audit, car_quantities, create_charter,
    demo_declarations, phi, render_docket_md, seed_declarations,
    z_from_percentile, ALIGN_TOL_SIGMA, NAKED_SIGMA, TWO_CARS_SIGMA,
)

TODAY = dt.date(2026, 7, 17)


def _full_ledger():
    """A deck with every channel the demo/consumption maps reach declared."""
    led = blank_ledger()
    led.set(SubsystemInterface(name="chassis", mass_kg=45.0, cg_z_mm=300.0,
                               mount_load_n=4200.0, is_estimate=True))
    led.set(SubsystemInterface(name="powertrain", mass_kg=60.0, cg_z_mm=320.0,
                               peak_power_kw=68.0, peak_torque_nm=180.0,
                               heat_reject_w=3200.0, is_estimate=True))
    led.set(SubsystemInterface(name="accumulator", mass_kg=55.0,
                               peak_current_a=180.0, power_draw_w=350.0,
                               is_estimate=True))
    led.set(SubsystemInterface(name="cooling", mass_kg=6.0,
                               cooling_airflow_cms=0.14, is_estimate=True))
    return led


def _quantities():
    return build_uncertainty_ledger(_full_ledger())


def _charter(pct=95.0):
    return create_charter(pct, note="one phantom", author="test", today=TODAY)


def _mass():
    return car_quantities(_quantities(), TODAY)["car.mass_kg"]


def _one(quantity_key, assumed, factor=1.0, adverse="high"):
    """Audit a single declaration and return the sole finding."""
    qs = _quantities()
    a = audit(_charter(), [MarginDeclaration("consumer", quantity_key, adverse,
                                             assumed, factor, "")], qs,
              today=TODAY)
    return a.findings[0]


# --------------------------- 1 · determinism -------------------------------

def test_audit_is_deterministic():
    qs = _quantities()
    ch = _charter()
    decs = demo_declarations(qs, TODAY)
    a = audit(ch, decs, qs, today=TODAY)
    b = audit(ch, decs, qs, today=TODAY)
    assert [(f.consumer, f.verdict, round(f.z_total, 6)) for f in a.findings] \
        == [(f.consumer, f.verdict, round(f.z_total, 6)) for f in b.findings]


def test_docket_markdown_is_byte_identical():
    qs = _quantities()
    ch = _charter()
    decs = demo_declarations(qs, TODAY)
    assert render_docket_md(audit(ch, decs, qs, today=TODAY)) \
        == render_docket_md(audit(ch, decs, qs, today=TODAY))


def test_charter_seal_is_deterministic():
    assert _charter().seal == _charter().seal


# --------------------------- 2 · verdicts ----------------------------------

def test_hedge_to_the_charter_percentile_is_aligned():
    mk = _mass()
    f = _one("car.mass_kg", mk.value + _charter().z * mk.sigma)
    assert f.verdict == "ALIGNED"
    assert f.coverage_pct == pytest.approx(95.0, abs=0.1)


def test_consuming_the_nominal_value_is_naked():
    mk = _mass()
    f = _one("car.mass_kg", mk.value)
    assert f.verdict == "NAKED"
    assert f.z_total == pytest.approx(0.0, abs=1e-9)
    # NAKED names the evidence grade it is naked on
    assert f.worst_grade == mk.worst_grade


def test_far_past_the_charter_is_stacked_with_releasable_envelope():
    mk = _mass()
    f = _one("car.mass_kg", mk.value + 4.0 * mk.sigma)
    assert f.verdict == "STACKED"
    # the excess over the charter car, priced in the quantity's own units
    assert f.releasable > 0.0
    assert f.unit == mk.unit


def test_a_favourable_assumption_is_anti_hedged():
    mk = _mass()
    # assuming a LIGHTER car than nominal, while adverse is "high"
    f = _one("car.mass_kg", mk.value - 1.5 * mk.sigma)
    assert f.verdict == "ANTI-HEDGED"
    assert f.z_assumed < 0.0


def test_an_explicit_factor_counts_as_cover():
    mk = _mass()
    # nominal assumed value but a fat factor on top should still buy cover
    plain = _one("car.mass_kg", mk.value, factor=1.0)
    hedged = _one("car.mass_kg", mk.value, factor=1.5)
    assert hedged.z_factor > plain.z_factor
    assert hedged.z_total > plain.z_total


def test_verdict_thresholds_are_the_documented_constants():
    # guard the named constants against silent drift into folklore
    assert ALIGN_TOL_SIGMA == 0.5
    assert NAKED_SIGMA == 0.25
    assert TWO_CARS_SIGMA == 1.0


# --------------------------- 3 · the two cars ------------------------------

def test_two_consumers_more_than_one_sigma_apart_are_contradictory():
    mk = _mass()
    qs = _quantities()
    decls = [
        MarginDeclaration("brakes", "car.mass_kg", "high",
                          mk.value + 2.0 * mk.sigma, 1.0, "heavy car"),
        MarginDeclaration("energy", "car.mass_kg", "high",
                          mk.value - 1.5 * mk.sigma, 1.0, "target car"),
    ]
    a = audit(_charter(), decls, qs, today=TODAY)
    two = [q for q in a.quantity_coverage if q.contradictory]
    assert len(two) == 1
    q = two[0]
    assert q.spread_sigma > TWO_CARS_SIGMA
    assert {q.hi_consumer, q.lo_consumer} == {"brakes", "energy"}


def test_consumers_within_one_sigma_are_not_contradictory():
    mk = _mass()
    qs = _quantities()
    decls = [
        MarginDeclaration("a", "car.mass_kg", "high",
                          mk.value + 0.3 * mk.sigma, 1.0, ""),
        MarginDeclaration("b", "car.mass_kg", "high",
                          mk.value + 0.6 * mk.sigma, 1.0, ""),
    ]
    a = audit(_charter(), decls, qs, today=TODAY)
    assert not any(q.contradictory for q in a.quantity_coverage)


def test_the_demo_pathology_describes_more_than_one_car():
    qs = _quantities()
    a = audit(_charter(), demo_declarations(qs, TODAY), qs, today=TODAY)
    contradictory = [q for q in a.quantity_coverage if q.contradictory]
    # brakes (+2σ heavy) vs energy budget (−1.5σ light) on car mass
    assert any(q.quantity_key == "car.mass_kg" for q in contradictory)


# --------------------------- 4 · β & three cars ----------------------------

def test_beta_is_the_root_sum_square_of_stacked_worst_cases():
    mk = _mass()
    qs = _quantities()
    # one consumer stacking +2σ on mass: β should be 2.0
    decls = [MarginDeclaration("brakes", "car.mass_kg", "high",
                               mk.value + 2.0 * mk.sigma, 1.0, "")]
    a = audit(_charter(), decls, qs, today=TODAY)
    p = next(p for p in a.consumer_phantoms if p.consumer == "brakes")
    assert p.beta == pytest.approx(2.0, abs=0.05)
    assert p.n_hedged == 1


def test_favourable_assumptions_do_not_inflate_beta():
    mk = _mass()
    qs = _quantities()
    decls = [MarginDeclaration("energy", "car.mass_kg", "high",
                               mk.value - 1.5 * mk.sigma, 1.0, "")]
    a = audit(_charter(), decls, qs, today=TODAY)
    p = next(p for p in a.consumer_phantoms if p.consumer == "energy")
    assert p.beta == pytest.approx(0.0, abs=1e-9)
    assert p.n_hedged == 0


def test_the_phantom_car_is_the_union_of_adverse_extremes():
    qs = _quantities()
    a = audit(_charter(), demo_declarations(qs, TODAY), qs, today=TODAY)
    mass_gap = next(g for g in a.objective_gaps
                    if "mass" in g.label.lower())
    # the phantom (heaviest anyone assumed) is heavier than nominal
    assert mass_gap.phantom > mass_gap.nominal
    # the coherent charter car sits between nominal and the phantom
    assert mass_gap.nominal <= mass_gap.coherent <= mass_gap.phantom


def test_over_defence_is_reported_per_objective():
    qs = _quantities()
    a = audit(_charter(), demo_declarations(qs, TODAY), qs, today=TODAY)
    assert a.objective_gaps
    # the mass objective's gap is the phantom-minus-charter over-defence
    for g in a.objective_gaps:
        assert g.unit  # every gap is priced in a real unit


# --------------------------- 5 · honesty & seal ----------------------------

def test_a_tampered_charter_fails_its_own_seal_check():
    ch = _charter(95.0)
    ch.percentile = 50.0          # move the sealed field after sealing
    assert not ch.verify_seal()


def test_the_audit_refuses_to_judge_a_broken_seal():
    ch = _charter(95.0)
    good_seal = ch.seal
    ch.percentile = 50.0
    ch.seal = good_seal           # keep the old seal over a changed percentile
    a = audit(ch, demo_declarations(_quantities(), TODAY), _quantities(),
              today=TODAY)
    assert a.refused
    assert a.refusal_reason
    assert a.findings == []       # nothing else is computed on refusal


def test_unresolved_keys_are_reported_not_silently_dropped():
    qs = _quantities()
    decls = [MarginDeclaration("ghost", "car.nonexistent_key", "high",
                               1.0, 1.0, "")]
    a = audit(_charter(), decls, qs, today=TODAY)
    assert "car.nonexistent_key" in a.unresolved
    assert not a.findings


def test_seeds_start_at_nominal_so_a_fresh_audit_is_not_flattered():
    qs = _quantities()
    cars = car_quantities(qs, TODAY)
    for d in seed_declarations(qs, TODAY):
        cq = cars.get(d.quantity_key)
        if cq is not None:
            assert d.assumed_value == pytest.approx(cq.value)
            assert d.design_factor == 1.0


def test_unaudited_consumers_are_named_not_absorbed():
    qs = _quantities()
    # audit a single consumer; the rest of the consumption map is unaudited
    decls = [MarginDeclaration("brakes", "car.mass_kg", "high",
                               _mass().value, 1.0, "")]
    a = audit(_charter(), decls, qs, today=TODAY)
    assert a.unaudited_consumers
    # each is a named sizing path, not a count folded into a green board
    assert all("consumer" in u and "quantity_key" in u
               for u in a.unaudited_consumers)


def test_helper_math_matches_the_standard_normal():
    assert phi(0.0) == pytest.approx(0.5, abs=1e-9)
    assert z_from_percentile(95.0) == pytest.approx(1.6449, abs=1e-3)
    assert z_from_percentile(97.72) == pytest.approx(2.0, abs=1e-2)
    # designing to the median car (or worse) is refused, by design
    with pytest.raises(ValueError):
        z_from_percentile(50.0)
