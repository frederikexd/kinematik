# ============================================================================
#  KinematiK — tests for pickup-stiffness elastokinematics
#  Copyright (c) 2026 Frederik Thio. Open source.
# ============================================================================
"""Stiffness field, compliance Jacobian, load-stepped solve, and the compliance
properties as walls inside InverseGenesis."""
import numpy as np
import pytest

from suspension import elastokinematics as ek
from suspension import inverse_genesis as ig
from suspension import genesis_repro as gr
from suspension.kinematics import Hardpoints

HP = Hardpoints.default()
LOAD = dict(Fy=1918.0, Fz=1235.0)


def _spec(k=30000.0, steps=5):
    return ek.ElastoSpec(stiffness=ek.StiffnessField(k), n_steps=steps, **LOAD)


def test_zero_load_moves_nothing():
    r = ek.solve_elastokinematic(HP, ek.ElastoSpec())
    assert all(abs(v) < 1e-9 for v in r.change.values())


def test_converges_and_matches_the_jacobian():
    r = ek.solve_elastokinematic(HP, _spec())
    assert r.converged
    assert all(e < 1e-3 for e in r.linearity_error.values())


def test_load_steps_agree():
    a = ek.solve_elastokinematic(HP, _spec(steps=1)).change["toe"]
    b = ek.solve_elastokinematic(HP, _spec(steps=8)).change["toe"]
    assert a == pytest.approx(b, abs=1e-4)


def test_change_scales_inversely_with_stiffness():
    a = ek.solve_elastokinematic(HP, _spec(30000.0)).change["toe"]
    b = ek.solve_elastokinematic(HP, _spec(60000.0)).change["toe"]
    assert a / b == pytest.approx(2.0, rel=0.01)


def test_jacobian_matches_a_single_displacement():
    J = ek.compliance_jacobian(HP, points=("upper_rear_inner",))
    d = np.array([0.0, 0.05, 0.0])
    from suspension.kinematics import SuspensionKinematics
    s0 = SuspensionKinematics(HP).solve_at_travel(0.0)
    s1 = SuspensionKinematics(HP, pickup_deltas={"upper_rear_inner": d}).solve_at_travel(0.0)
    assert (s1.camber - s0.camber) == pytest.approx(J[0] @ d, abs=1e-4)


def test_stiffness_matrix_is_validated():
    with pytest.raises(ValueError):
        ek.StiffnessField(per_point={"upper_rear_inner": -np.eye(3)})
    with pytest.raises(ValueError):
        ek.StiffnessField(per_point={"not_a_point": 1000.0})


def test_pull_tests_build_a_measured_field():
    f = ek.StiffnessField.from_pull_tests(HP, {"tie_rod_inner": (1200.0, 0.04)})
    assert f.provenance == "measured"
    K = f.matrix("tie_rod_inner")
    u = np.asarray(HP.tie_rod_outer, float) - np.asarray(HP.tie_rod_inner, float)
    u /= np.linalg.norm(u)
    assert u @ K @ u == pytest.approx(30000.0, rel=1e-9)


def test_anisotropic_node_changes_the_answer():
    soft_y = ek.StiffnessField(per_point={"upper_rear_inner": np.diag([30000.0, 3000.0, 30000.0])})
    a = ek.solve_elastokinematic(HP, _spec()).change["camber"]
    b = ek.solve_elastokinematic(HP, ek.ElastoSpec(stiffness=soft_y, **LOAD)).change["camber"]
    assert abs(b) > abs(a)


def test_compliance_bound_needs_a_load_case():
    with pytest.raises(ValueError):
        ig.SolvedPropertyBounds(bounds=[ig.PropertyBound("compliance_toe_deg", lo=-0.03, hi=0.03)])


def test_compliance_is_a_solved_property():
    ctx = ig.SolvedPropertyBounds(bounds=[ig.PropertyBound("compliance_toe_deg", lo=-0.5, hi=0.5)],
                                  elasto=_spec())
    p = ig.properties_of(HP, ctx)
    assert "compliance_toe_deg" in p and abs(p["compliance_toe_deg"]) > 0.0


def test_compliance_bound_is_a_wall():
    """A bound tighter than the geometry's compliance steer is a violation."""
    r = ek.solve_elastokinematic(HP, _spec()).change["toe"]
    tight = abs(r) / 2.0
    ctx = ig.SolvedPropertyBounds(bounds=[ig.PropertyBound("compliance_toe_deg", lo=-tight, hi=tight)],
                                  elasto=_spec())
    vol = ig.LegalVolume.around(HP, 10.0, points=["tie_rod_inner"], properties=ctx)
    assert vol.property_violations(HP)


def test_manifest_round_trip_keeps_the_load_case():
    ctx = ig.SolvedPropertyBounds(bounds=[ig.PropertyBound("compliance_toe_deg", lo=-0.1, hi=0.1)],
                                  elasto=ek.ElastoSpec(stiffness=ek.StiffnessField(
                                      per_point={"tie_rod_inner": 45000.0}, provenance="measured"),
                                      n_steps=4, **LOAD))
    back = gr._property_bounds_from_dict(gr._property_bounds_to_dict(ctx))
    assert back.elasto.n_steps == 4 and back.elasto.Fy == 1918.0
    assert back.elasto.stiffness.provenance == "measured"
    assert back.elasto.stiffness.matrix("tie_rod_inner")[0, 0] == pytest.approx(45000.0)


def test_frame_twist_toe_reproduces_the_paper():
    t = ek.frame_twist_toe(618.0, 2000.0, 1630.0, 250.0, 200.0, 0.6766)
    assert t == pytest.approx(0.1106, abs=0.002)
