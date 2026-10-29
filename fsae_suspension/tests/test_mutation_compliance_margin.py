"""suspension/compliance_margin.py: the end-stiffness series sum, the wall read from a manifest, and the bisection
for the stiffness at which a wall breaks, each checked against its closed form. The corner solve itself is
tests/test_review_checks.py's; here it is replaced by a toe law whose answer is known exactly."""
import math
from types import SimpleNamespace

import pytest

from suspension import compliance as cm
from suspension import compliance_margin as cmg
from suspension import elastokinematics as ek


def _spec(**tabs):
    members = {m: cm.MemberStiffness(material="Steel 4130", od_mm=19.05, wall_mm=0.9, k_tab=tabs.get(m))
               for m in ek.MEMBER_PICKUP}
    return ek.ElastoSpec(members=members, Fy=1500.0, Fz=1200.0, n_steps=2)


def test_rigid_ends_leave_the_spec_as_declared():
    s = _spec()
    assert cmg.with_end_stiffness(s, None) is s and cmg.with_end_stiffness(s, math.inf) is s


def test_two_ends_in_series_with_any_declared_tab():
    s = cmg.with_end_stiffness(_spec(TR=6000.0), 20000.0)
    assert s.members["UF"].k_tab == pytest.approx(10000.0)                     # two 20 kN/mm ends in series
    assert s.members["TR"].k_tab == pytest.approx(1.0 / (1.0 / 6000.0 + 1.0 / 10000.0))
    assert (s.Fy, s.Fz, s.n_steps) == (1500.0, 1200.0, 2) and s.members["UF"].od_mm == 19.05
    assert _spec().members["UF"].k_tab is None                                  # the original is not changed


def test_every_link_must_be_declared():
    s = ek.ElastoSpec(members={"TR": cm.MemberStiffness(k_direct=1.0)})
    with pytest.raises(ValueError) as e:
        cmg.with_end_stiffness(s, 1000.0)
    assert str(e.value) == ("compliance_margin needs every link declared as a member (material, od, wall); "
                            "missing UF, UR, LF, LR")


def test_toe_is_the_solved_change_or_nan_when_the_solve_did_not_settle(monkeypatch):
    seen = []

    def fake(hp, spec, jacobian=True):
        seen.append(jacobian)
        return SimpleNamespace(converged=spec.Fy > 0, change={"toe": 0.125})
    monkeypatch.setattr(cmg.ek, "solve_elastokinematic", fake)
    assert cmg.toe("hp", ek.ElastoSpec(Fy=1.0)) == 0.125
    assert math.isnan(cmg.toe("hp", ek.ElastoSpec(Fy=-1.0)))
    assert seen == [False, False]


def _manifest(bounds, elasto="corner-spec", braking="brake-spec"):
    pb = SimpleNamespace(bounds=[SimpleNamespace(prop=p, lo=lo, hi=hi) for p, lo, hi in bounds], elasto=elasto)
    if braking is not None:
        pb.elasto_braking = braking
    return SimpleNamespace(objects=lambda: (None, None, SimpleNamespace(properties=pb)))


def test_the_wall_is_the_larger_finite_bound_of_each_declared_toe():
    m = _manifest([("compliance_toe_deg", -0.05, 0.08), ("compliance_toe_braking_deg", -math.inf, 0.03),
                   ("camber_gain", -1.0, 1.0)])
    assert cmg.walls(m) == {"corner": ("corner-spec", 0.08), "braking": ("brake-spec", 0.03)}
    m = _manifest([("compliance_toe_deg", -0.2, math.inf), ("compliance_toe_braking_deg", -0.1, 0.1)], braking=None)
    assert cmg.walls(m) == {"corner": ("corner-spec", 0.2)}
    assert cmg.walls(_manifest([("compliance_toe_deg", -0.2, 0.1)], elasto=None)) == {}
    assert cmg.walls(_manifest([("camber_gain", -1.0, 1.0)])) == {}


def _law(monkeypatch, c, rigid=0.01):
    """toe = rigid + c / k_end: the wall at `limit` breaks at k = c / (limit - rigid)."""
    monkeypatch.setattr(cmg, "_corner", lambda manifest: "hp")
    monkeypatch.setattr(cmg, "with_end_stiffness", lambda spec, k: (spec, k))

    def toe(hp, spec):
        k = spec[1] if isinstance(spec, tuple) else None
        return rigid + (0.0 if k is None or not math.isfinite(k) else c / k)
    monkeypatch.setattr(cmg, "toe", toe)


def test_the_sweep_reports_each_end_stiffness_and_rigid(monkeypatch):
    _law(monkeypatch, 100.0)
    m = _manifest([("compliance_toe_deg", -0.05, 0.05)], braking=None)
    r = cmg.end_stiffness_sweep(m)
    assert r == {"corner": {"limit_deg": 0.05, "toe_deg": {
        "rigid": 0.01, 100000.0: 0.011, 50000.0: 0.012, 20000.0: 0.015, 10000.0: 0.02}}}
    assert cmg.end_stiffness_sweep(m, (2000.0,))["corner"]["toe_deg"] == {2000.0: pytest.approx(0.06)}


def test_the_required_stiffness_is_where_the_toe_meets_the_wall(monkeypatch):
    _law(monkeypatch, 400.0)
    m = _manifest([("compliance_toe_deg", -0.05, 0.05)], braking=None)
    k, rigid, limit = cmg.required_end_stiffness(m)["corner"]
    exact = 400.0 / (0.05 - 0.01)                                        # 10 000 N/mm
    assert (rigid, limit) == (0.01, 0.05)
    assert exact <= k <= exact * math.exp(0.005)                         # the side that holds, within the tolerance
    k2 = cmg.required_end_stiffness(m, rel_tol=1e-6)["corner"][0]
    assert k2 == pytest.approx(exact, rel=2e-6)
    assert cmg.required_end_stiffness(m, lo=20000.0)["corner"][0] == 20000.0    # holds even at lo
    assert cmg.required_end_stiffness(m, lo=500.0, hi=11000.0, rel_tol=1e-6)["corner"][0] == pytest.approx(exact)
    assert cmg.required_end_stiffness(m, lo=1001.0)["corner"][0] >= exact      # the default lo is 1000
    _law(monkeypatch, 400.0, rigid=0.06)
    assert cmg.required_end_stiffness(m)["corner"] == (math.inf, 0.06, 0.05)
    _law(monkeypatch, 40.0)
    assert cmg.required_end_stiffness(m)["corner"][0] == 1000.0          # 40/1000 + 0.01 = 0.05: holds at lo
    _law(monkeypatch, 3e5)
    assert cmg.required_end_stiffness(m)["corner"][0] == pytest.approx(3e5 / 0.04, rel=0.006)  # under the 1e7 cap
    _law(monkeypatch, 5e5)
    assert cmg.required_end_stiffness(m)["corner"][0] == pytest.approx(1e7)    # needs more than hi: reports hi


def test_the_lash_deadband_is_read_for_a_tenth_of_a_millimetre_everywhere(monkeypatch):
    monkeypatch.setattr(cmg, "_corner", lambda manifest: "hp")
    seen = {}

    def fake(hp, lash):
        seen.update(hp=hp, lash=lash)
        return {"toe": 0.0123, "camber": 9.0}
    monkeypatch.setattr(cmg.ek, "lash_deadband", fake)
    assert cmg.lash_toe_per_0p1mm("manifest") == 0.0123
    assert seen == {"hp": "hp", "lash": {m: 0.1 for m in ek.MEMBER_PICKUP}}


def _strict(monkeypatch, m, laws):
    """Stubs that only answer for this manifest's corner and the declared specs: toe = rigid + c / k_end per spec."""
    monkeypatch.setattr(cmg, "_corner", lambda manifest: "hp" if manifest is m else None)
    monkeypatch.setattr(cmg, "with_end_stiffness", lambda spec, k: (spec, k) if spec in laws else None)

    def toe(hp, spec):
        assert hp == "hp"
        base, k = spec if isinstance(spec, tuple) else (spec, None)
        rigid, c = laws[base]
        return rigid + (0.0 if k is None or not math.isfinite(k) else c / k)
    monkeypatch.setattr(cmg, "toe", toe)


def test_each_case_is_solved_on_this_corner_with_its_own_spec(monkeypatch):
    m = _manifest([("compliance_toe_deg", -0.05, 0.05), ("compliance_toe_braking_deg", -0.04, 0.04)])
    _strict(monkeypatch, m, {"corner-spec": (0.01, 100.0), "brake-spec": (0.02, 200.0)})
    r = cmg.end_stiffness_sweep(m, (math.inf, 10000.0))
    assert r == {"corner": {"limit_deg": 0.05, "toe_deg": {"rigid": 0.01, 10000.0: pytest.approx(0.02)}},
                 "braking": {"limit_deg": 0.04, "toe_deg": {"rigid": 0.02, 10000.0: pytest.approx(0.04)}}}
    req = cmg.required_end_stiffness(m, rel_tol=1e-7)
    assert req["corner"][0] == pytest.approx(100.0 / 0.04, rel=1e-6) and req["braking"][0] == pytest.approx(
        200.0 / 0.02, rel=1e-6)
    _strict(monkeypatch, m, {"corner-spec": (0.06, 100.0), "brake-spec": (0.02, 200.0)})
    req = cmg.required_end_stiffness(m, rel_tol=1e-7)                 # the first case fails rigid; the next still runs
    assert req["corner"] == (math.inf, 0.06, 0.05) and req["braking"][0] == pytest.approx(1e4, rel=1e-6)
    _strict(monkeypatch, m, {"corner-spec": (0.01, 1.0), "brake-spec": (0.02, 200.0)})
    req = cmg.required_end_stiffness(m, rel_tol=1e-7)                 # the first holds at lo; the next still runs
    assert req["corner"] == (1000.0, 0.01, 0.05) and req["braking"][0] == pytest.approx(1e4, rel=1e-6)
    _strict(monkeypatch, m, {"corner-spec": (0.05, 100.0), "brake-spec": (0.02, 200.0)})
    assert cmg.required_end_stiffness(m)["corner"][0] == pytest.approx(1e7)      # on the wall rigid: not a failure


def test_the_corner_comes_from_the_steering_margin_winner(monkeypatch):
    from suspension import steering_margin as sm
    monkeypatch.setattr(sm, "corner", lambda manifest: ("corner of", manifest))
    assert cmg._corner("M") == ("corner of", "M")
    seen = {}
    monkeypatch.setattr(cmg.ek, "lash_deadband", lambda hp, lash: seen.update(hp=hp) or {"toe": 0.5})
    assert cmg.lash_toe_per_0p1mm("M") == 0.5 and seen["hp"] == ("corner of", "M")


def test_toe_hands_the_solver_this_corner_and_spec(monkeypatch):
    seen = []
    monkeypatch.setattr(cmg.ek, "solve_elastokinematic", lambda hp, spec, jacobian=True:
                        seen.append((hp, spec)) or SimpleNamespace(converged=True, change={"toe": 0.2}))
    spec = ek.ElastoSpec(Fy=3.0)
    assert cmg.toe("HP", spec) == 0.2 and seen == [("HP", spec)]
