"""Independent checks of the topology-agnostic compliance estimator (suspension/generic_compliance.py).

The member force of each link is the virtual work of the wheel load through that link's stretch, so it is checked
against a finite difference of the solved mechanism taken here with a different step; the reported toe and camber are
checked by re-solving the mechanism at the reported deflections; the small-rotation read-out against exact rotations;
and the bookkeeping (labels, restored lengths, the no-link and failing-stiffness paths, the summary) against what the
docstrings promise.
"""
import numpy as np
import pytest

import suspension.generic_compliance as gc
import suspension.loadpath as lp
import suspension.topologies as T
from suspension import GenericKinematics, MemberStiffness
from suspension.joints import JointCompliance
from suspension.topology import Link

pytestmark = pytest.mark.filterwarnings("ignore")
LOAD = lp.WheelLoad(Fx=200.0, Fy=-2800.0, Fz=1600.0)


def _tube(label, a, b):
    return MemberStiffness(material="Aluminium 6061", od_mm=12.7, wall_mm=0.9,
                           joint_in=JointCompliance.rubber_bushing(k_radial=40000.0))


def _rot(axis, a):
    c, s = np.cos(a), np.sin(a)
    return {0: np.array([[1, 0, 0], [0, c, -s], [0, s, c]]),
            1: np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]),
            2: np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])}[axis]


def test_the_rotation_read_out_is_the_axis_times_the_sine():
    for axis in range(3):
        w = gc._rotvec(_rot(axis, 0.03))
        want = np.zeros(3)
        want[axis] = np.sin(0.03)
        assert w == pytest.approx(want, abs=1e-15)
    assert gc._v([1, 2, 3]).dtype == np.float64 and gc._v([[1], [2], [3]]).shape == (3,)


def test_links_are_named_by_label_or_by_their_ends():
    class Mech:
        constraints = [Link(a="p", b="q", label="toe link"), Link(a="r", b="s"), object()]
    names = [(n, a, b) for n, a, b, _ in gc._link_constraints(Mech())]
    assert names == [("toe link", "p", "q"), ("r->s", "r", "s")]


def _kin():
    k = GenericKinematics(T.example("multilink"))
    k.mech.compile()
    return k


def test_member_forces_are_the_virtual_work_of_the_load_through_each_stretch():
    k = _kin()
    mech = k.mech
    r = gc.solve_generic_compliance(k, LOAD, lambda *a: None)
    F = LOAD.force()
    links = gc._link_constraints(mech)
    assert set(r.member_forces) == {n for n, *_ in links}
    for name, a, b, c in links:
        L, h = c.length, 0.05
        c.length = L + h
        up = np.asarray(mech.solve(0.0)["positions"][mech.contact_patch], float)
        c.length = L - h
        dn = np.asarray(mech.solve(0.0)["positions"][mech.contact_patch], float)
        c.length = L
        assert r.member_forces[name] == pytest.approx(F @ ((up - dn) / (2 * h)), rel=2e-3, abs=1e-6)
    mech.solve(0.0)
    # no stiffness: nothing deflects, and the corner reads zero compliance
    assert all(d == 0.0 for d in r.member_deflection.values()) and r.member_stiffness == {}
    assert (r.compliance_toe, r.compliance_camber, r.contact_patch_lateral_shift_mm) == pytest.approx((0, 0, 0),
                                                                                                       abs=1e-12)
    assert r.converged and r.iterations == 2


def test_the_reported_compliance_is_the_mechanism_solved_at_the_reported_give():
    k = _kin()
    mech = k.mech
    r = gc.solve_generic_compliance(k, LOAD, _tube, tol_deg=1e-7)
    assert r.converged and r.well_conditioned and r.iterations >= 3
    links = gc._link_constraints(mech)
    base = mech.solve(0.0)
    R0 = np.asarray(base["carrier_R"], float)
    cp0 = np.asarray(base["positions"][mech.contact_patch], float)
    lengths = {n: c.length for n, *_, c in links}
    for n, *_, c in links:
        c.length = lengths[n] + r.member_deflection[n]
    d = mech.solve(0.0)
    for n, *_, c in links:
        c.length = lengths[n]
    mech.solve(0.0)
    w = gc._rotvec(np.asarray(d["carrier_R"], float) @ R0.T)
    assert r.compliance_camber == pytest.approx(np.degrees(w[0]), abs=1e-5)
    assert r.compliance_toe == pytest.approx(np.degrees(w[2]), abs=1e-5)
    assert r.contact_patch_lateral_shift_mm == pytest.approx(
        (np.asarray(d["positions"][mech.contact_patch], float) - cp0)[1], abs=1e-5)
    # each give is the member's own law at its force and its current length
    pos = d["positions"]
    for n, a, b, _ in links:
        L = float(np.linalg.norm(np.asarray(pos[a], float) - np.asarray(pos[b], float)))
        ms = _tube(n, a, b)
        assert r.member_deflection[n] == pytest.approx(ms.axial_deflection(r.member_forces[n], L), rel=1e-4)
        assert r.member_stiffness[n] == pytest.approx(ms.tangent_axial_stiffness(r.member_forces[n], L), rel=1e-4)
        assert r.member_joint_deflection[n] == pytest.approx(ms.deflection_breakdown(r.member_forces[n], L),
                                                               rel=1e-4, abs=1e-12)
    assert r.compliance_toe != 0.0 and r.compliance_camber != 0.0


def test_the_defaults_are_a_quarter_millimetre_step_twelve_passes_and_a_ten_thousandth_degree():
    a = gc.solve_generic_compliance(_kin(), LOAD, _tube)
    b = gc.solve_generic_compliance(_kin(), LOAD, _tube, eps_mm=0.25, max_iter=12, tol_deg=1e-4)
    assert (a.compliance_toe, a.compliance_camber, a.iterations) == (b.compliance_toe, b.compliance_camber,
                                                                     b.iterations)
    assert a.member_forces == b.member_forces


def test_a_pass_limit_that_stops_the_loop_early_is_not_convergence():
    # pass 1 reads the unloaded pose; only a second pass that agrees with it can call the loop settled
    for load in (LOAD, lp.WheelLoad()):
        one = gc.solve_generic_compliance(_kin(), load, _tube, max_iter=1)
        assert one.iterations == 1 and one.converged is False and one.well_conditioned is True
    still = gc.solve_generic_compliance(_kin(), lp.WheelLoad(), _tube, max_iter=2)
    assert still.iterations == 2 and still.converged is True
    two = gc.solve_generic_compliance(_kin(), LOAD, _tube, max_iter=2, tol_deg=0.0)
    assert two.iterations == 2 and two.converged is False


def test_a_member_whose_stiffness_fails_gives_no_deflection():
    def bad(label, a, b):
        return MemberStiffness()                  # no link and no joint: axial_deflection raises
    r = gc.solve_generic_compliance(_kin(), LOAD, bad)
    assert all(d == 0.0 for d in r.member_deflection.values()) and r.member_stiffness == {}
    assert r.member_joint_deflection == {}


def test_a_mechanism_with_no_links_says_it_cannot_estimate():
    class Mech:
        _compiled = False
        constraints = []
        contact_patch = "cp"
        compiled = []

        def compile(self):
            self.compiled.append(True)
            self._compiled = True

        def solve(self, travel):
            return {"positions": {"cp": [0.0, 0.0, 0.0]}, "carrier_R": np.eye(3)}

    class Kin:
        mech = Mech()
    load = lp.WheelLoad(Fx=3.0, Fy=4.0, Fz=12.0)
    r = gc.solve_generic_compliance(Kin(), load, _tube)
    assert Kin.mech.compiled == [True]
    assert r.load is load and (r.compliance_toe, r.compliance_camber, r.contact_patch_lateral_shift_mm) == (0, 0, 0)
    assert (r.converged, r.well_conditioned, r.iterations, r.residual_N) == (False, False, 0, 13.0)
    assert (r.member_forces, r.member_deflection, r.member_stiffness) == ({}, {}, {})


def test_a_failed_solve_still_restores_the_link_length():
    k = _kin()
    links = gc._link_constraints(k.mech)
    name, a, b, c = links[0]
    L = c.length
    real = k.mech.solve
    calls = []

    def solve(t):
        calls.append(c.length)
        if len(calls) == 2:
            raise RuntimeError("solver blew up")
        return real(t)
    k.mech.solve = solve
    R0 = np.asarray(real(0.0)["carrier_R"], float)
    with pytest.raises(RuntimeError):
        gc._pose_sensitivities(k.mech, links[:1], k.mech.contact_patch, R0, 0.25)
    assert c.length == L and calls == [L + 0.25, L - 0.25]


def test_the_summary_carries_every_headline_number():
    r = gc.solve_generic_compliance(_kin(), LOAD, _tube)
    s = r.summary()
    assert s == {"compliance_toe_deg": r.compliance_toe, "compliance_camber_deg": r.compliance_camber,
                 "contact_patch_lateral_shift_mm": r.contact_patch_lateral_shift_mm,
                 "member_forces_N": r.member_forces, "member_deflection_mm": r.member_deflection,
                 "residual_N": 0.0, "iterations": r.iterations, "well_conditioned": True, "converged": True}
    assert all(type(v) is float for v in s["member_forces_N"].values())
    assert type(s["iterations"]) is int and type(s["converged"]) is bool


def test_rotation_sensitivities_are_the_finite_difference_of_the_carrier_rotation():
    k = _kin()
    mech = k.mech
    links = gc._link_constraints(mech)
    R0 = np.asarray(mech.solve(0.0)["carrier_R"], float)
    b, r = gc._pose_sensitivities(mech, links, mech.contact_patch, R0, 0.25)
    for name, *_, c in links:
        L, h = c.length, 0.02
        c.length = L + h
        up = gc._rotvec(np.asarray(mech.solve(0.0)["carrier_R"], float) @ R0.T)
        c.length = L - h
        dn = gc._rotvec(np.asarray(mech.solve(0.0)["carrier_R"], float) @ R0.T)
        c.length = L
        assert r[name] == pytest.approx((up - dn) / (2 * h), rel=2e-3, abs=1e-9)
    mech.solve(0.0)


def test_compilation_only_when_needed_and_the_stiffness_is_asked_per_link():
    class Mech:
        def __init__(self, compiled):
            if compiled is not None:
                self._compiled = compiled
            self.compiled_calls = 0
            self.constraints, self.contact_patch = [], "cp"

        def compile(self):
            self.compiled_calls += 1

        def solve(self, travel):
            return {"positions": {"cp": [0.0, 0.0, 0.0]}, "carrier_R": np.eye(3)}
    for compiled, calls in ((True, 0), (False, 1), (None, 1)):
        m = Mech(compiled)
        gc.solve_generic_compliance(type("K", (), {"mech": m})(), LOAD, _tube)
        assert m.compiled_calls == calls
    seen = []
    k = _kin()
    gc.solve_generic_compliance(k, LOAD, lambda *a: seen.append(a) or _tube(*a), max_iter=1)
    assert seen == [(n, a, b) for n, a, b, _ in gc._link_constraints(k.mech)]


def test_a_member_the_axial_model_cannot_carry_is_left_out_and_flagged(monkeypatch):
    k = _kin()
    real = gc._pose_sensitivities

    def tilted(mech, links, cp, R0, eps):
        b, r = real(mech, links, cp, R0, eps)
        first = links[0][0]
        r[first] = r[first] / max(np.linalg.norm(r[first]), 1e-12) * 1.5      # 1.5 rad/mm: beyond the 1.0 guard
        return b, r
    monkeypatch.setattr(gc, "_pose_sensitivities", tilted)
    r = gc.solve_generic_compliance(k, LOAD, _tube)
    assert r.well_conditioned is False and r.converged is False
    first = gc._link_constraints(k.mech)[0][0]
    assert r.member_deflection[first] != 0.0                 # reported, but kept out of the deflected pose


def test_no_pass_at_all_reports_the_unloaded_corner():
    r = gc.solve_generic_compliance(_kin(), LOAD, _tube, max_iter=0)
    assert (r.iterations, r.well_conditioned, r.converged) == (0, True, False)
    assert (r.compliance_toe, r.compliance_camber, r.contact_patch_lateral_shift_mm) == (0.0, 0.0, 0.0)
    assert r.load is LOAD and r.residual_N == 0.0


def test_each_pass_solves_the_pose_and_both_sides_of_every_link_then_restores(monkeypatch):
    k = _kin()
    mech = k.mech
    n = len(gc._link_constraints(mech))
    base = {p: v.copy() for p, v in mech.solve(0.0)["positions"].items()}
    calls = []
    real = mech.solve
    monkeypatch.setattr(mech, "solve", lambda t: calls.append(t) or real(t))
    two = gc.solve_generic_compliance(k, LOAD, _tube, max_iter=2, tol_deg=0.0)
    assert len(calls) == 1 + 2 * (1 + 2 * n) + 1 and set(calls) == {0.0}
    three = gc.solve_generic_compliance(k, LOAD, _tube, max_iter=3, tol_deg=0.0)
    assert two.compliance_toe != three.compliance_toe
    for p, v in base.items():
        assert np.asarray(mech.points[p].pos) == pytest.approx(v, abs=1e-9)
    calls.clear()
    d = gc.solve_generic_compliance(k, LOAD, _tube, tol_deg=0.0)
    assert d.iterations == 12 and len(calls) == 1 + 12 * (1 + 2 * n) + 1
