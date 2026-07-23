# ============================================================================
#  KinematiK — tests for suspension/worthwhile.py
#  Verifies the worthwhileness verdict: the no-go gate that WITHHOLDS points on
#  a blocking contradiction, the reconciled-vehicle handoff, and the paper-vs-
#  real points delta that makes a heavier assembled car actually score less.
# ============================================================================
import numpy as np
import pytest

from suspension.interfaces import (
    SubsystemInterface, blank_ledger, Severity,
)
from suspension.worthwhile import (
    assess, Assumption, check_assumptions, vehicle_from_ledger,
    WorthwhileVerdict, PROVENANCE,
)
from suspension.dynamics import VehicleParams


def _full_ledger(pt_mass=40.0, driver_kg=70.0, target=300.0):
    """A complete, buildable ledger with all canonical subsystems declared."""
    led = blank_ledger()
    led.target_mass_kg = target
    led.includes_driver_kg = driver_kg
    rows = [
        ("suspension", 35, 180), ("chassis", 32, 300),
        ("powertrain", pt_mass, 270), ("aerodynamics", 12, 400),
        ("brakes", 8, 200), ("cooling", 6, 250),
        ("electrics", 10, 220), ("data-acquisition", 3, 240),
    ]
    for name, m, z in rows:
        led.set(SubsystemInterface(name=name, mass_kg=m, cg_x_mm=900,
                                   cg_y_mm=0, cg_z_mm=z, is_estimate=False))
    return led


# ---- the reconciled-vehicle handoff (the missing link) -----------------
def test_vehicle_from_ledger_uses_reconciled_mass_and_cg():
    led = _full_ledger(pt_mass=40)
    base = VehicleParams(mass=999.0, cg_height=999.0)
    veh = vehicle_from_ledger(led, base=base)
    roll = led.mass_rollup()
    expected_mass = roll["total_kg"] + led.includes_driver_kg
    assert veh.p.mass == pytest.approx(expected_mass, abs=0.1)
    assert veh.p.cg_height == pytest.approx(roll["cg_mm"][2], abs=0.1)


# ---- assumption contradiction check ------------------------------------
def test_assumption_contradiction_fires_and_names_both_sides():
    led = _full_ledger(pt_mass=90)   # very heavy powertrain
    a = Assumption(by="suspension", field="total_mass_kg", value=110.0, tol=5.0)
    findings = check_assumptions(led, [a])
    fails = [f for f in findings if f.severity == Severity.FAIL]
    assert fails, "a contradicted assumption must FAIL"
    subs = fails[0].subsystems
    assert "suspension" in subs                 # the assumer
    assert "powertrain" in subs                 # the largest contributor


def test_assumption_within_tolerance_does_not_fire():
    led = _full_ledger(pt_mass=40)
    roll = led.mass_rollup()
    a = Assumption(by="suspension", field="total_mass_kg",
                   value=roll["total_kg"], tol=5.0)   # assume the real value
    findings = check_assumptions(led, [a])
    assert not [f for f in findings if f.severity == Severity.FAIL]


def test_uncheckable_assumption_is_missing_not_fail():
    led = blank_ledger()   # nothing declared
    a = Assumption(by="suspension", field="total_mass_kg", value=210.0)
    findings = check_assumptions(led, [a])
    assert any(f.severity == Severity.MISSING for f in findings)
    assert not any(f.severity == Severity.FAIL for f in findings)


# ---- THE HARD RULE: points withheld on a blocking contradiction --------
def test_no_go_gate_withholds_points_on_contradiction():
    led = _full_ledger(pt_mass=95)   # heavy enough to break the assumption
    a = Assumption(by="suspension", field="total_mass_kg", value=100.0, tol=5.0)
    v = assess(led, assumptions=[a], paper_baseline=VehicleParams())
    assert v.buildable is False
    assert v.points_delta is None            # WITHHELD, not faked
    assert v.real_points is None
    assert "NOT BUILDABLE" in v.verdict_text


def test_no_go_gate_withholds_points_when_physics_input_missing():
    led = blank_ledger()   # no masses => cannot roll up => cannot sim
    v = assess(led, paper_baseline=VehicleParams())
    assert v.buildable is False
    assert v.points_delta is None
    assert any(f.check == "physics-input-missing" for f in v.blocking)


def test_envelope_fail_blocks_build():
    led = _full_ledger()
    led.chassis_envelope_mm = (500, 400, 300)
    # a subsystem that cannot fit
    led.set(SubsystemInterface(name="accumulator-box", mass_kg=5,
                               env_x_mm=900, env_y_mm=400, env_z_mm=300))
    # note: only canonical subsystems are envelope-checked; use a canonical one
    led.set(SubsystemInterface(name="powertrain", mass_kg=40, cg_x_mm=900,
                               cg_y_mm=0, cg_z_mm=270,
                               env_x_mm=900, env_y_mm=400, env_z_mm=300,
                               is_estimate=False))
    v = assess(led, paper_baseline=VehicleParams())
    assert v.buildable is False


# ---- the worthwhileness delta: heavier real car scores less ------------
def test_heavier_reconciled_car_loses_points():
    led = _full_ledger(pt_mass=100)  # +60 kg over a light paper baseline
    paper = VehicleParams(mass=186.0, cg_height=270.0)
    v = assess(led, paper_baseline=paper)
    assert v.buildable is True
    assert v.points_delta is not None
    assert v.total_delta() < 0.0             # a heavier build must cost points
    # and the loss should be spread across events, worst where mass hurts most
    assert v.points_delta["acceleration"] <= 0.0


def test_on_budget_car_has_near_zero_delta():
    # paper baseline equal to the reconciled all-up mass => ~no delta
    led = _full_ledger(pt_mass=40)
    roll = led.mass_rollup()
    allup = roll["total_kg"] + led.includes_driver_kg
    paper = VehicleParams(mass=allup, cg_height=roll["cg_mm"][2])
    v = assess(led, paper_baseline=paper)
    assert v.buildable is True
    assert abs(v.total_delta()) < 1.0        # matching inputs => matching points


def test_estimate_flag_propagates_to_verdict():
    led = _full_ledger()
    # mark one subsystem as an estimate
    it = led.get("aerodynamics")
    it.is_estimate = True
    led.set(it)
    paper = VehicleParams(mass=250.0, cg_height=270.0)
    v = assess(led, paper_baseline=paper)
    if v.buildable:
        assert v.any_estimate is True
        assert "ESTIMATE" in v.verdict_text


# ---- verdict object is well-formed -------------------------------------
def test_verdict_shape_and_provenance():
    led = _full_ledger()
    v = assess(led, paper_baseline=VehicleParams(mass=250.0, cg_height=270.0))
    assert isinstance(v, WorthwhileVerdict)
    assert isinstance(v.findings, list) and v.findings
    assert "hard_rule" in PROVENANCE
    assert "withheld" in PROVENANCE["hard_rule"].lower()


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
