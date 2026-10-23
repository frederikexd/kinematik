# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Property-based checks of the kinematics and statics on randomly perturbed corners.

The other suites check hand-picked geometries. These draw geometries at random around the published corners
(every pickup moved independently by up to a few millimetres) and random wheel loads, and check laws the solver must
obey whatever the geometry: rigid links keep their length, the answer does not depend on where the corner sits in
space or on the path taken to reach a travel, the statics are linear in the load, the forces on the chassis balance
the wheel load, and the pushrod force matches virtual work. Each runs a fixed, derandomized set of examples so a
failure reproduces exactly.
Lengths mm, forces N, moments N*mm, angles deg.
"""
import sys
from pathlib import Path

import numpy as np
import pytest

hypothesis = pytest.importorskip("hypothesis")
from hypothesis import HealthCheck, assume, given, settings, strategies as st   # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension import loadpath as lp                                          # noqa: E402
from suspension.genesis_repro import GenesisManifest, hp_from_dict              # noqa: E402
from suspension.kinematics import SuspensionKinematics                         # noqa: E402

MAN = ROOT / "manifests"
SETTINGS = settings(max_examples=100, deadline=None, derandomize=True,
                    suppress_health_check=[HealthCheck.too_slow, HealthCheck.filter_too_much])

POINTS = ["upper_front_inner", "upper_rear_inner", "lower_front_inner", "lower_rear_inner", "upper_outer",
          "lower_outer", "tie_rod_inner", "tie_rod_outer", "wheel_center", "contact_patch"]
ROCKER = ["pushrod_outer", "rocker_pivot", "rocker_pushrod", "rocker_spring", "spring_inner"]
INNER = ["upper_front_inner", "upper_rear_inner", "lower_front_inner", "lower_rear_inner", "tie_rod_inner"]
LINKS = [("upper_front_inner", "upper_outer"), ("upper_rear_inner", "upper_outer"),
         ("lower_front_inner", "lower_outer"), ("lower_rear_inner", "lower_outer"),
         ("tie_rod_inner", "tie_rod_outer")]


def _base(name):
    return hp_from_dict(GenesisManifest.from_json((MAN / f"{name}.recorded.genesis.json").read_text())
                        .recorded["winner_hardpoints"])


BASES = {n: _base(n) for n in ("rear_v11_final", "front_v11_final")}

shift = st.lists(st.floats(-1.0, 1.0, allow_nan=False), min_size=3, max_size=3)


@st.composite
def corners(draw, max_mm=4.0, attach=False):
    """A published corner with every inner pickup and the tie-rod outer moved independently by up to max_mm; with
    attach, the pushrod rides on the upright, the lower wishbone or the upper wishbone."""
    hp = BASES[draw(st.sampled_from(sorted(BASES)))].copy()
    for name in INNER + ["tie_rod_outer"]:
        setattr(hp, name, np.asarray(getattr(hp, name), float) + max_mm * np.asarray(draw(shift)))
    if attach:
        hp.pushrod_attach = draw(st.sampled_from(["upright", "lower", "upper"]))
    return hp


vec = lambda lim: st.tuples(*(st.floats(-lim, lim),) * 3)
loads = st.builds(lp.WheelLoad,
                  Fx=st.floats(-2500.0, 2500.0), Fy=st.floats(-2500.0, 2500.0), Fz=st.floats(300.0, 3000.0),
                  Mz=st.floats(-60000.0, 60000.0), Fx_wc=st.floats(-2000.0, 2000.0),
                  F_wc_extra=vec(600.0), M_wc_extra=vec(10000.0))
travel = st.floats(-25.0, 25.0)


def _solved(kin, t):
    s = kin.solve_at_travel(t)
    assume(s.converged)
    return s


def _pos(s, name):
    return np.asarray({"upper_outer": s.upper_outer, "lower_outer": s.lower_outer,
                       "tie_rod_outer": s.tie_rod_outer}[name], float)


@SETTINGS
@given(hp=corners(attach=True), t=travel)
def test_rigid_links_and_upright_keep_their_length(hp, t):
    """Every link, and the upright's three joints, are rigid: lengths at any travel equal the static ones, and the
    pushrod's outer end rides rigidly on the body it is bolted to."""
    kin = SuspensionKinematics(hp)
    s = _solved(kin, t)
    for inner, outer in LINKS:
        L0 = np.linalg.norm(np.asarray(getattr(hp, inner), float) - np.asarray(getattr(hp, outer), float))
        assert np.linalg.norm(np.asarray(getattr(hp, inner), float) - _pos(s, outer)) == pytest.approx(L0, abs=1e-6)
    for a, b in (("upper_outer", "lower_outer"), ("upper_outer", "tie_rod_outer"), ("lower_outer", "tie_rod_outer")):
        L0 = np.linalg.norm(np.asarray(getattr(hp, a), float) - np.asarray(getattr(hp, b), float))
        assert np.linalg.norm(_pos(s, a) - _pos(s, b)) == pytest.approx(L0, abs=1e-6)
    assert s.lower_outer[2] - hp.lower_outer[2] == pytest.approx(t, abs=1e-6)
    body = {"upright": ["upper_outer", "lower_outer", "tie_rod_outer"],
            "lower": ["lower_outer", "lower_front_inner", "lower_rear_inner"],
            "upper": ["upper_outer", "upper_front_inner", "upper_rear_inner"]}[hp.pushrod_attach]
    now = lambda n: _pos(s, n) if n in ("upper_outer", "lower_outer", "tie_rod_outer") else np.asarray(getattr(hp, n), float)
    for n in body:
        d0 = np.linalg.norm(np.asarray(hp.pushrod_outer, float) - np.asarray(getattr(hp, n), float))
        assert np.linalg.norm(np.asarray(s.pushrod_outer, float) - now(n)) == pytest.approx(d0, abs=1e-6)


@SETTINGS
@given(hp=corners(), t=travel, d=st.lists(st.floats(-500.0, 500.0), min_size=3, max_size=3))
def test_moving_the_whole_corner_changes_no_angle(hp, t, d):
    """Translating every point of the corner moves the solution with it and leaves every angle and the scrub
    unchanged: the answer does not depend on where the corner sits in the car's frame."""
    d = np.asarray(d, float)
    moved = hp.copy()
    for name in POINTS + ROCKER:
        if getattr(moved, name, None) is not None:
            setattr(moved, name, np.asarray(getattr(moved, name), float) + d)
    a, b = _solved(SuspensionKinematics(hp), t), _solved(SuspensionKinematics(moved), t)
    for q in ("camber", "toe", "caster", "kpi", "scrub_radius"):
        assert getattr(b, q) == pytest.approx(getattr(a, q), abs=1e-6)
    for q in ("upper_outer", "lower_outer", "tie_rod_outer", "wheel_center", "contact_patch"):
        assert np.allclose(np.asarray(getattr(b, q)) - d, np.asarray(getattr(a, q)), atol=1e-6)


@SETTINGS
@given(hp=corners(), t=travel)
def test_the_answer_does_not_depend_on_the_path(hp, t):
    """Solving straight from static and stepping there 1 mm at a time, each step seeded by the last, land on the
    same configuration."""
    kin = SuspensionKinematics(hp)
    direct = _solved(kin, t)
    seed, s = None, None
    for x in np.linspace(0.0, t, max(2, int(abs(t)) + 1))[1:]:
        s = kin.solve_at_travel(float(x), seed=seed)
        assume(s.converged)
        seed = np.concatenate([s.lower_outer, s.upper_outer, s.tie_rod_outer])
    for q in ("camber", "toe", "caster", "kpi"):
        assert getattr(s, q) == pytest.approx(getattr(direct, q), abs=1e-6)


@SETTINGS
@given(hp=corners(attach=True), t=st.floats(-15.0, 15.0), L1=loads, L2=loads, a=st.floats(-2.0, 2.0),
       b=st.floats(-2.0, 2.0))
def test_member_forces_are_linear_in_the_load(hp, t, L1, L2, a, b):
    """Rigid statics are linear: the member forces for a*L1 + b*L2 are a*F(L1) + b*F(L2)."""
    kin = SuspensionKinematics(hp)
    s = _solved(kin, t)
    lin = lambda u, v: tuple(a * np.asarray(u, float) + b * np.asarray(v, float))
    comb = lp.WheelLoad(Fx=a * L1.Fx + b * L2.Fx, Fy=a * L1.Fy + b * L2.Fy, Fz=a * L1.Fz + b * L2.Fz,
                        Mz=a * L1.Mz + b * L2.Mz, Fx_wc=a * L1.Fx_wc + b * L2.Fx_wc,
                        F_wc_extra=lin(L1.F_wc_extra, L2.F_wc_extra), M_wc_extra=lin(L1.M_wc_extra, L2.M_wc_extra))
    f1, f2, fc = (lp.solve_member_forces(kin, s, L).forces for L in (L1, L2, comb))
    scale = 1.0 + max(abs(v) for v in list(f1.values()) + list(f2.values()))
    for m in fc:
        assert fc[m] == pytest.approx(a * f1[m] + b * f2[m], abs=1e-7 * scale)


@SETTINGS
@given(hp=corners(attach=True), t=st.floats(-15.0, 15.0), L=loads)
def test_forces_on_the_chassis_balance_the_wheel_load(hp, t, L):
    """Whatever the geometry and load, the forces the corner puts on the chassis (inner pickups and the pushrod via
    the rocker) sum to the wheel load, and their moment about the patch is the load's moment."""
    kin = SuspensionKinematics(hp)
    s = _solved(kin, t)
    mf = lp.solve_member_forces(kin, s, L)
    assume(mf.condition < 1e8)
    assert mf.residual <= 1e-6 * (1.0 + np.abs(L.force()).sum())
    cp = np.asarray(s.contact_patch, float)
    F = sum(mf.pickup_forces.values()) - mf.forces["PR"] * mf.axes["PR"]
    M = sum(np.cross(np.asarray(getattr(hp, k), float) - cp, v) for k, v in mf.pickup_forces.items())
    M = M + np.cross(np.asarray(hp.rocker_pushrod, float) - cp, -mf.forces["PR"] * mf.axes["PR"])
    scale = 1.0 + np.abs(L.force()).sum()
    assert np.allclose(F, L.force(), atol=1e-6 * scale)
    assert np.allclose(M, lp._applied_moment(s, L, cp), atol=1e-3 * scale)


def _rigid_velocity(kin, s0, h=0.02):
    """Velocity per mm of travel of the upright's material points, from the rigid transform between solved states."""
    sp, sm = kin.solve_at_travel(h), kin.solve_at_travel(-h)
    pts = lambda s: np.array([s.upper_outer, s.lower_outer, s.tie_rod_outer], float)
    def fit(A, B):
        ca, cb = A.mean(0), B.mean(0)
        U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))
        D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
        R = Vt.T @ D @ U.T
        return R, cb - R @ ca
    Rp, tp = fit(pts(s0), pts(sp)); Rm, tm = fit(pts(s0), pts(sm))
    W = (Rp - Rm) / (2 * h)
    return (lambda p: ((Rp @ np.asarray(p, float) + tp) - (Rm @ np.asarray(p, float) + tm)) / (2 * h),
            np.array([W[2, 1], W[0, 2], W[1, 0]]))


@SETTINGS
@given(hp=corners(attach=True), L=loads)
def test_pushrod_force_equals_virtual_work(hp, L):
    """With rigid links and one compliant member, the pushrod force times its length rate equals the power of the
    wheel load on the moving wheel, for any geometry and load."""
    kin = SuspensionKinematics(hp)
    s0 = _solved(kin, 0.0)
    vel, omega = _rigid_velocity(kin, s0)
    h = 0.02
    Lp = lambda s: np.linalg.norm(np.asarray(s.pushrod_outer) - np.asarray(hp.rocker_pushrod))
    dL = (Lp(kin.solve_at_travel(h)) - Lp(kin.solve_at_travel(-h))) / (2 * h)
    assume(abs(dL) > 0.05)
    vwc = vel(s0.wheel_center)
    P = (np.dot([L.Fx, L.Fy, L.Fz], vel(s0.contact_patch)) + L.Fx_wc * vwc[0] + L.Mz * omega[2]
         + np.dot(L.F_wc_extra, vwc) + np.dot(L.M_wc_extra, omega))
    mf = lp.solve_member_forces(kin, s0, L)
    assert mf.forces["PR"] == pytest.approx(P / dL, rel=2e-3, abs=0.5)


@SETTINGS
@given(hp=corners(), t=st.floats(-15.0, 15.0), L=loads)
def test_a_corner_without_a_rocker_balances_the_load_through_the_strut(hp, t, L):
    """Without a pushrod and rocker the solver carries the vertical path on a vertical strut proxy at the lower ball
    joint. That strut can take only a vertical force through that joint, so the links alone must balance the load's
    horizontal force and its whole moment about the joint."""
    for name in ROCKER + ["rocker_axis"]:
        setattr(hp, name, None)
    kin = SuspensionKinematics(hp)
    s = _solved(kin, t)
    mf = lp.solve_member_forces(kin, s, L)
    assert not mf.has_pushrod and mf.forces["PR"] == 0.0
    assume(mf.condition < 1e8)
    assert mf.residual <= 1e-6 * (1.0 + np.abs(L.force()).sum())
    links = [m for m in mf.forces if m != "PR"]
    cp, lo = np.asarray(s.contact_patch, float), np.asarray(s.lower_outer, float)
    F = sum(mf.forces[m] * np.asarray(mf.axes[m], float) for m in links) + L.force()
    M = sum(np.cross(np.asarray(mf.outboard[m], float) - lo, mf.forces[m] * np.asarray(mf.axes[m], float))
            for m in links) + lp._applied_moment(s, L, cp) + np.cross(cp - lo, L.force())
    scale = 1.0 + np.abs(L.force()).sum()
    assert np.allclose(F[:2], 0.0, atol=1e-6 * scale)
    assert np.allclose(M, 0.0, atol=1e-3 * scale)


def _rodrigues(p, pivot, k, th):
    v = np.asarray(p, float) - pivot
    return pivot + v * np.cos(th) + np.cross(k, v) * np.sin(th) + k * np.dot(k, v) * (1.0 - np.cos(th))


@SETTINGS
@given(hp=corners(attach=True), t=st.floats(-20.0, 20.0))
def test_spring_length_matches_an_independent_rocker_solve(hp, t):
    """The installed spring length, found here by rotating the rocker until the pushrod has its static length (a
    one-variable root solve that shares no code with the solver), equals the solver's; and the motion ratio is the
    spring's length change per mm of wheel-centre travel."""
    from scipy.optimize import brentq
    kin = SuspensionKinematics(hp)
    s = _solved(kin, t)
    piv = np.asarray(hp.rocker_pivot, float)
    k = np.asarray(hp.rocker_axis, float) / np.linalg.norm(hp.rocker_axis)
    Lp = np.linalg.norm(np.asarray(hp.pushrod_outer, float) - np.asarray(hp.rocker_pushrod, float))
    f = lambda th: np.linalg.norm(_rodrigues(hp.rocker_pushrod, piv, k, th) - np.asarray(s.pushrod_outer)) - Lp
    th = brentq(f, -0.6, 0.6) if f(-0.6) * f(0.6) < 0 else None
    assume(th is not None)
    L_ind = np.linalg.norm(_rodrigues(hp.rocker_spring, piv, k, th) - np.asarray(hp.spring_inner, float))
    L_kin, _, ok = kin.spring_length_at(s)
    assume(ok)
    assert L_kin == pytest.approx(L_ind, abs=1e-6)
    up, dn = kin.solve_at_travel(5.0), kin.solve_at_travel(-5.0)
    lu, thu, _ = kin.spring_length_at(up)
    ld, _, _ = kin.spring_length_at(dn, seed=thu)
    mr = abs(lu - ld) / (up.wheel_center[2] - dn.wheel_center[2])
    assert kin.motion_ratio() == pytest.approx(mr, rel=1e-9)
    assert kin.wheel_rate(30.0) == pytest.approx(30.0 * mr ** 2, rel=1e-9)


@SETTINGS
@given(hp=corners(), t=travel, which=st.sampled_from(["upper_f", "upper_r", "lower_f", "lower_r", "tie"]),
       dl=st.floats(-1.5, 1.5))
def test_a_link_length_change_is_carried_exactly(hp, t, which, dl):
    """The compliance layer stretches one link: at any travel that link is longer by exactly the stretch and the
    others keep their length."""
    kin = SuspensionKinematics(hp, length_deltas={which: dl})
    s = _solved(kin, t)
    names = dict(zip(["upper_f", "upper_r", "lower_f", "lower_r", "tie"], LINKS))
    for key, (inner, outer) in names.items():
        L0 = np.linalg.norm(np.asarray(getattr(hp, inner), float) - np.asarray(getattr(hp, outer), float))
        L = np.linalg.norm(np.asarray(getattr(hp, inner), float) - _pos(s, outer))
        assert L == pytest.approx(L0 + (dl if key == which else 0.0), abs=1e-6)


@SETTINGS
@given(hp=corners(), d=st.lists(st.floats(-500.0, 500.0), min_size=2, max_size=2))
def test_side_view_geometry_does_not_depend_on_where_the_corner_sits(hp, d):
    """Anti-dive, anti-squat and the side-view swing arm are unchanged when the whole corner moves fore-aft or
    sideways."""
    moved = hp.copy()
    shift = np.array([d[0], d[1], 0.0])
    for name in POINTS + ROCKER:
        if getattr(moved, name, None) is not None:
            setattr(moved, name, np.asarray(getattr(moved, name), float) + shift)
    a, b = SuspensionKinematics(hp), SuspensionKinematics(moved)
    for f in (lambda k: k.anti_dive_pct(280.0, 1530.0, 0.7), lambda k: k.anti_squat_pct(280.0, 1530.0),
              lambda k: k.anti_lift_pct(280.0, 1530.0, 0.3), lambda k: k.side_view_swing_arm_length()):
        va, vb = f(a), f(b)
        assert (np.isinf(va) and np.isinf(vb)) or vb == pytest.approx(va, rel=1e-6, abs=1e-6)


@SETTINGS
@given(hp=corners(attach=True))
def test_hardpoints_survive_a_round_trip_through_a_saved_file(hp):
    """Writing the hardpoints to a project dict and reading them back gives the same corner."""
    back = type(hp).from_dict(hp.as_dict())
    for name in POINTS + ROCKER + ["rocker_axis"]:
        assert np.allclose(np.asarray(getattr(back, name), float), np.asarray(getattr(hp, name), float))
    assert (back.pushrod_attach, back.static_camber, back.static_toe) == (hp.pushrod_attach, hp.static_camber,
                                                                         hp.static_toe)


@SETTINGS
@given(hp=corners(), h=st.floats(200.0, 400.0), wb=st.floats(1500.0, 1700.0), bias=st.floats(0.2, 0.6))
def test_inboard_anti_lift_is_anti_squat_times_the_brake_bias(hp, h, wb, bias):
    """Inboard rear brakes and an inboard drive both put their force through the links at wheel-centre height, so
    anti-lift is anti-squat scaled by the rear brake bias; outboard brakes use the patch's path instead."""
    kin = SuspensionKinematics(hp)
    sq = kin.anti_squat_pct(h, wb, drive_bias_rear=1.0)
    assert kin.anti_lift_pct(h, wb, brake_bias_rear=bias, inboard_brakes=True) == pytest.approx(sq * bias, rel=1e-9,
                                                                                               abs=1e-9)
    assert kin.anti_lift_pct(h, wb, bias) == pytest.approx(-kin.anti_dive_pct(h, wb, bias), rel=1e-9, abs=1e-9)


@SETTINGS
@given(Fz=st.floats(100.0, 3000.0), mu_y=st.floats(0.0, 2.0), mu_x=st.floats(-2.0, 2.0),
       sign=st.sampled_from([-1.0, 1.0]), drive=st.booleans(), brakes=st.booleans())
def test_a_corner_load_keeps_every_newton_and_routes_shaft_torque_to_the_wheel_centre(Fz, mu_y, mu_x, sign, drive,
                                                                                       brakes):
    """The convenience load builder keeps the friction fractions exactly and sends traction (with an inboard drive) or
    braking (with inboard brakes) to the wheel centre, never both places."""
    w = lp.wheel_load_from_corner(Fz, mu_lateral=mu_y, mu_long=mu_x, lateral_sign=sign, inboard_drive=drive,
                                  inboard_brakes=brakes)
    assert (w.Fz, w.Fy) == (pytest.approx(Fz), pytest.approx(sign * mu_y * Fz))
    assert w.Fx + w.Fx_wc == pytest.approx(mu_x * Fz)
    shaft = (mu_x < 0 and drive) or (mu_x > 0 and brakes)
    assert (w.Fx_wc != 0.0) == (shaft and mu_x != 0.0) and (w.Fx == 0.0 or not shaft)
    assert np.allclose(w.force(), [mu_x * Fz, sign * mu_y * Fz, Fz])


@SETTINGS
@given(hp=corners(attach=True), L=loads)
def test_the_reported_member_forces_are_the_solved_ones(hp, L):
    """tension() and the saved report give back exactly the solved member forces and diagnostics."""
    kin = SuspensionKinematics(hp)
    s = _solved(kin, 0.0)
    mf = lp.solve_member_forces(kin, s, L)
    d = mf.as_dict()
    assert all(mf.tension(m) == mf.forces[m] == d["forces_N"][m] for m in mf.forces)
    assert mf.tension("not a member") == 0.0
    assert (d["condition"], d["residual_N"], d["has_pushrod"], d["note"]) == (mf.condition, mf.residual,
                                                                             mf.has_pushrod, mf.note)
