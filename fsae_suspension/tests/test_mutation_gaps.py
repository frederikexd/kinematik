"""Tests aimed at the mutants the kinematics and statics suites let through.

The monthly mutation run (.github/workflows/mutation.yml) breaks suspension/kinematics.py and
suspension/loadpath.py on purpose and counts how many breaks the tests notice. These are the checks
whose absence let the largest groups of breaks through:

* the pushrod-on-wishbone solution was only checked for its pushrod force, never for the wishbone's
  own balance or for how the pivot reactions are split, so sign errors in that part went unseen;
* compliance link-length shims were never exercised for the upright and tie-rod links;
* the default corner was not pinned, so moving any pickup by a millimetre passed;
* the save/load round trip and the validation messages were not checked field by field;
* the motion-ratio, swing-arm and anti-geometry functions were called only at static, with defaults.

Each check is an independent calculation (a free-body balance, a finite difference of solved
positions, a closed form), not a re-run of the code under test.
"""
import inspect

import numpy as np
import pytest

from suspension import loadpath as lp
from suspension.kinematics import Hardpoints, SuspensionKinematics

pytestmark = pytest.mark.filterwarnings("ignore")


@pytest.fixture(scope="module")
def kin():
    return SuspensionKinematics(Hardpoints.default())


def _unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


LOADS = [
    lp.WheelLoad(Fz=1500.0),
    lp.WheelLoad(Fx=1200.0, Fy=-1800.0, Fz=1500.0, Mz=20000.0),
    lp.WheelLoad(Fz=1500.0, Fx_wc=-900.0),
    lp.WheelLoad(Fy=900.0, Fz=1500.0, F_wc_extra=(50.0, -30.0, 20.0), M_wc_extra=(1e4, -2e4, 5e3)),
]


def _external(state, load):
    """Force and moment about the contact patch of a WheelLoad, from its definition."""
    cp, wc = np.asarray(state.contact_patch), np.asarray(state.wheel_center)
    F = np.array([load.Fx + load.Fx_wc, load.Fy, load.Fz]) + np.asarray(load.F_wc_extra)
    M = (np.array([0.0, 0.0, load.Mz])
         + np.cross(wc - cp, np.array([load.Fx_wc, 0.0, 0.0]) + np.asarray(load.F_wc_extra))
         + np.asarray(load.M_wc_extra))
    return F, M


# --------------------------------------------------- pushrod on a wishbone
def _pullrod_on_upper():
    hp = Hardpoints.default()
    hp.pushrod_attach = "upper"
    hp.pushrod_outer = np.array([5.0, 480.0, 295.0])
    hp.rocker_pivot = np.array([-5.0, 330.0, 120.0])
    hp.rocker_pushrod = np.array([-5.0, 372.0, 140.0])
    hp.rocker_spring = np.array([-5.0, 300.0, 90.0])
    hp.spring_inner = np.array([-5.0, 200.0, 120.0])
    return hp


def _plan_angled():
    """Pivot axes with a lateral component: every entry of the cross-product matrix matters."""
    hp = Hardpoints.default()
    hp.upper_rear_inner = hp.upper_rear_inner + [0.0, 40.0, 0.0]
    hp.lower_rear_inner = hp.lower_rear_inner + [0.0, -30.0, 6.0]
    return hp


def _plan_angled_pullrod():
    hp = _pullrod_on_upper()
    hp.upper_rear_inner = hp.upper_rear_inner + [0.0, 35.0, -4.0]
    return hp


def _mount_past_the_ball_joint():
    """Pushrod mount outboard of the ball joint: the bending arm is capped at each leg's length."""
    hp = Hardpoints.default()
    hp.pushrod_outer = np.array([-5.0, 620.0, 108.0])
    return hp


def _mount_inboard_of_the_pivots():
    """Pushrod mount inboard of both pivots: the bending arm is floored at zero."""
    hp = Hardpoints.default()
    hp.pushrod_outer = np.array([-5.0, 100.0, 120.0])
    return hp


@pytest.mark.parametrize("hp_fn,body,legs", [
    (Hardpoints.default, "lower", ("lower_front_inner", "lower_rear_inner")),
    (_pullrod_on_upper, "upper", ("upper_front_inner", "upper_rear_inner")),
    (_plan_angled, "lower", ("lower_front_inner", "lower_rear_inner")),
    (_plan_angled_pullrod, "upper", ("upper_front_inner", "upper_rear_inner")),
    (_mount_past_the_ball_joint, "lower", ("lower_front_inner", "lower_rear_inner")),
    (_mount_inboard_of_the_pivots, "lower", ("lower_front_inner", "lower_rear_inner"))])
@pytest.mark.parametrize("load", LOADS)
def test_the_wishbone_carrying_the_pushrod_balances_on_its_own(hp_fn, body, legs, load):
    hp = hp_fn()
    k = SuspensionKinematics(hp)
    st = k.static
    mf = lp.solve_member_forces(k, st, load)
    assert mf.model == "wishbone-pushrod" and mf.has_pushrod and body in mf.note
    assert mf.residual < 1e-6
    F, M = _external(st, load)
    cp = np.asarray(st.contact_patch)
    P, u, T = np.asarray(st.pushrod_outer), mf.axes["PR"], mf.forces["PR"]
    assert u == pytest.approx(_unit(np.asarray(hp.rocker_pushrod) - P))
    pick = mf.pickup_forces
    # the whole corner: tyre load + every chassis reaction + the pushrod
    sumF = F - sum(pick.values()) + T * u
    sumM = M + sum(np.cross(np.asarray(getattr(hp, n)) - cp, -f) for n, f in pick.items()) \
        + np.cross(P - cp, T * u)
    assert np.linalg.norm(sumF) < 1e-6 * np.linalg.norm(F) + 1e-9
    assert np.linalg.norm(sumM) < 1e-6 * np.linalg.norm(F) * 1000
    # the upright alone fixes the ball-joint force B; then the wishbone must balance with it
    bj = np.asarray(st.lower_outer if body == "lower" else st.upper_outer)
    others = [n for n in pick if n not in legs]
    B = -(F - sum(pick[n] for n in others))
    fi, ri = (np.asarray(getattr(hp, n)) for n in legs)
    RF, RR = -pick[legs[0]], -pick[legs[1]]
    assert np.linalg.norm(-B + T * u + RF + RR) < 1e-6 * np.linalg.norm(F)
    Mw = np.cross(bj - fi, -B) + np.cross(P - fi, T * u) + np.cross(ri - fi, RR)
    assert np.linalg.norm(Mw) < 1e-6 * np.linalg.norm(F) * 1000
    # the split along the pivot axis is the one with the least force across the legs
    ax = _unit(ri - fi)
    eF, eR = _unit(fi - bj), _unit(ri - bj)
    PF, PR = np.eye(3) - np.outer(eF, eF), np.eye(3) - np.outer(eR, eR)
    lam = np.linspace(-50, 50, 11)
    cost = [np.linalg.norm(PF @ (RF + x * ax)) ** 2 + np.linalg.norm(PR @ (RR - x * ax)) ** 2
            for x in lam]
    assert int(np.argmin(cost)) == 5
    assert ax @ PF @ RF - ax @ PR @ RR == pytest.approx(0.0, abs=1e-6 * np.linalg.norm(F))
    names = {"lower_front_inner": "LF", "lower_rear_inner": "LR",
             "upper_front_inner": "UF", "upper_rear_inner": "UR"}
    nF, nR = names[legs[0]], names[legs[1]]
    assert mf.forces[nF] == pytest.approx(RF @ eF) and mf.forces[nR] == pytest.approx(RR @ eR)
    assert mf.transverse == pytest.approx({nF: np.linalg.norm(PF @ RF), nR: np.linalg.norm(PR @ RR)})
    arm = {nF: np.clip((P - fi) @ _unit(bj - fi), 0, np.linalg.norm(bj - fi)),
           nR: np.clip((P - ri) @ _unit(bj - ri), 0, np.linalg.norm(bj - ri))}
    assert mf.bending_arm_mm == pytest.approx(arm)
    assert mf.axes[nF] == pytest.approx(eF) and mf.outboard[nF] == pytest.approx(bj)
    assert mf.outboard["PR"] == pytest.approx(P)


def test_pushrod_on_the_upright_is_a_six_link_two_force_solve(kin):
    hp = Hardpoints.default()
    hp.pushrod_attach = "upright"
    hp.pushrod_outer = np.array([-5.0, 520.0, 150.0])
    k = SuspensionKinematics(hp)
    st = k.static
    for load in LOADS:
        mf = lp.solve_member_forces(k, st, load)
        assert mf.model == "two-force" and mf.has_pushrod and mf.transverse is None
        F, M = _external(st, load)
        cp = np.asarray(st.contact_patch)
        sumF, sumM = F.copy(), M.copy()
        for m, f in mf.forces.items():
            sumF += f * mf.axes[m]
            sumM += np.cross(np.asarray(mf.outboard[m]) - cp, f * mf.axes[m])
        assert np.linalg.norm(sumF) < 1e-6 * np.linalg.norm(F)
        assert np.linalg.norm(sumM) < 1e-3 * np.linalg.norm(F)
        assert mf.pickup_forces["tie_rod_inner"] == pytest.approx(-mf.forces["TR"] * mf.axes["TR"])
    notes = lp.solve_member_forces(k, st, LOADS[2]).note
    assert "-900 N of longitudinal force applied at the wheel centre" in notes
    assert "Hub force/moment" in lp.solve_member_forces(k, st, LOADS[3]).note


def test_without_a_rocker_a_vertical_strut_carries_what_the_links_do_not():
    hp = Hardpoints.default()
    hp.pushrod_outer = None
    k = SuspensionKinematics(hp)
    st = k.static
    mf = lp.solve_member_forces(k, st, LOADS[1])
    assert not mf.has_pushrod and mf.forces["PR"] == 0.0 and "STRUT" not in mf.forces
    assert mf.note.startswith("No pushrod/rocker defined")
    F, M = _external(st, LOADS[1])
    links = sum(f * mf.axes[m] for m, f in mf.forces.items() if m != "PR")
    strut = -(F + links)                          # what is left must be vertical
    assert abs(strut[0]) < 1e-6 * np.linalg.norm(F) and abs(strut[1]) < 1e-6 * np.linalg.norm(F)
    assert mf.axes["STRUT"] == pytest.approx([0, 0, 1])
    assert mf.outboard["STRUT"] == pytest.approx(st.lower_outer)


def test_wheel_load_from_corner():
    w = lp.wheel_load_from_corner(1000.0, 1.4, 0.5)
    assert (w.Fx, w.Fy, w.Fz, w.Fx_wc) == (500.0, -1400.0, 1000.0, 0.0)
    plain = lp.wheel_load_from_corner(1000.0)
    assert (plain.Fx, plain.Fy, plain.Fx_wc) == (0.0, 0.0, 0.0)
    assert lp.wheel_load_from_corner(1000.0, mu_long=-0.5, inboard_drive=True).Fx_wc == -500.0
    assert lp.wheel_load_from_corner(1000.0, mu_long=-0.5, inboard_brakes=True).Fx_wc == 0.0
    assert lp.wheel_load_from_corner(1000.0, mu_long=0.5, inboard_brakes=True).Fx_wc == 500.0
    assert lp.wheel_load_from_corner(1000.0, mu_long=0.5, inboard_drive=True).Fx_wc == 0.0
    assert lp.wheel_load_from_corner(1000.0, 1.0, lateral_sign=1.0).Fy == 1000.0
    assert lp.WheelLoad(Fx=1, Fx_wc=2, F_wc_extra=(0, 3, 0)).force() == pytest.approx([3, 3, 0])


# -------------------------------------------------- compliance link lengths
LINKS = {
    "upper_f": ("upper_outer", "upper_front_inner"), "upper_r": ("upper_outer", "upper_rear_inner"),
    "lower_f": ("lower_outer", "lower_front_inner"), "lower_r": ("lower_outer", "lower_rear_inner"),
    "upright": ("upper_outer", "lower_outer"), "tie": ("tie_rod_outer", "tie_rod_inner"),
    "tro_lo": ("tie_rod_outer", "lower_outer"), "tro_uo": ("tie_rod_outer", "upper_outer"),
}
MOVING = {"upper_outer", "lower_outer", "tie_rod_outer"}


def _length(hp, st, a, b):
    pa = np.asarray(getattr(st, a) if a in MOVING else getattr(hp, a))
    pb = np.asarray(getattr(st, b) if b in MOVING else getattr(hp, b))
    return float(np.linalg.norm(pa - pb))


@pytest.mark.parametrize("key", sorted(LINKS))
def test_a_length_shim_lengthens_that_link_and_only_that_link(key):
    hp = Hardpoints.default()
    st = SuspensionKinematics(hp, length_deltas={key: 1.5, "bogus": 9.0}).static
    for k, (a, b) in LINKS.items():
        nominal = float(np.linalg.norm(np.asarray(getattr(hp, a)) - np.asarray(getattr(hp, b))))
        assert _length(hp, st, a, b) == pytest.approx(nominal + (1.5 if k == key else 0.0), abs=1e-6)


def test_a_pickup_shift_keeps_the_unloaded_link_length():
    hp = Hardpoints.default()
    shift = np.array([0.0, 0.0, 2.0])
    k = SuspensionKinematics(hp, pickup_deltas={"lower_front_inner": shift, "not_a_point": shift})
    assert k.hp.lower_front_inner == pytest.approx(hp.lower_front_inner + shift)
    assert Hardpoints.default().lower_front_inner == pytest.approx(hp.lower_front_inner)  # untouched
    st = k.static
    nominal = np.linalg.norm(hp.lower_outer - hp.lower_front_inner)
    assert np.linalg.norm(st.lower_outer - k.hp.lower_front_inner) == pytest.approx(nominal, abs=1e-6)
    assert st.lower_outer[2] == pytest.approx(hp.lower_outer[2])     # same ride height


# ------------------------------------------------------ the default corner
def test_the_default_corner_is_pinned():
    """The documented examples all come from this geometry; a 1 mm change has to be deliberate."""
    hp = Hardpoints.default()
    expect = dict(
        upper_front_inner=[-100.0, 240.0, 299.2], upper_rear_inner=[130.0, 240.0, 280.8],
        lower_front_inner=[-110.0, 200.0, 117.5], lower_rear_inner=[140.0, 200.0, 122.5],
        upper_outer=[12.0, 540.0, 300.0], lower_outer=[-5.0, 575.0, 110.0],
        tie_rod_inner=[100.0, 230.0, 160.0], tie_rod_outer=[90.0, 560.0, 150.0],
        wheel_center=[0.0, 600.0, 228.0], contact_patch=[0.0, 605.0, 0.0],
        pushrod_outer=[-5.0, 408.0, 120.0], rocker_pivot=[-5.0, 360.0, 330.0],
        rocker_axis=[1.0, 0.0, 0.0], rocker_pushrod=[-5.0, 402.0, 308.0],
        rocker_spring=[-5.0, 335.0, 360.0], spring_inner=[-5.0, 196.7, 244.8])
    for name, v in expect.items():
        assert getattr(hp, name).tolist() == v, name
        assert getattr(hp, name).dtype == np.float64
    assert (hp.pushrod_attach, hp.static_camber, hp.static_toe) == ("lower", -1.5, 0.0)
    k = SuspensionKinematics(hp)
    s = k.static
    assert (s.camber, s.toe, s.caster, s.kpi, s.scrub_radius) == pytest.approx(
        (-1.5, 0.0, 5.112849779882466, 10.43747535111818, 9.736842105263122), abs=1e-9)
    assert s.instant_center == pytest.approx([-2712.935779816514, 194.17115596330274], rel=1e-9)
    assert k.motion_ratio() == pytest.approx(0.5222357223003841, rel=1e-9)
    assert k.anti_dive_pct(280, 1550) == pytest.approx(28.092810702134045, rel=1e-9)
    assert k.anti_squat_pct(280, 1550) == pytest.approx(18.33749470207674, rel=1e-9)
    assert k.side_view_swing_arm_length() == pytest.approx(1908.6, rel=1e-9)


# ------------------------------------------------------- saving and loading
@pytest.mark.parametrize("rocker", [True, False])
def test_every_field_survives_a_save_and_reload(rocker):
    hp = Hardpoints.default()
    if not rocker:
        hp.pushrod_outer = hp.rocker_pivot = None
    hp.pushrod_attach, hp.static_camber, hp.static_toe = "upper", -2.25, 0.4
    d = hp.as_dict()
    assert all(isinstance(v, list) for k, v in d.items()
               if isinstance(getattr(hp, k), np.ndarray))
    back = Hardpoints.from_dict({"from_another_version": 1, **d})     # unknown key first
    for name in Hardpoints.__dataclass_fields__:
        a, b = getattr(hp, name), getattr(back, name)
        if a is None:
            assert b is None, name
        elif isinstance(a, np.ndarray):
            assert isinstance(b, np.ndarray) and b.dtype == np.float64 and b.tolist() == a.tolist(), name
        else:
            assert b == a, name
    c = hp.copy()
    c.upper_outer[0] += 1.0
    assert hp.upper_outer[0] == 12.0                                  # a copy, not a view
    ints = Hardpoints.from_dict({**d, "upper_outer": [12, 540, 300]})
    assert ints.upper_outer.dtype == np.float64


# --------------------------------------------------------------- validation
@pytest.mark.parametrize("change,message", [
    (dict(upper_outer=[1.0, 2.0]), r"Hardpoint 'upper_outer' must be a 3D point \[x, y, z\]; got shape \(2,\)\."),
    (dict(tie_rod_inner=[np.nan, 0.0, 0.0]), r"Hardpoint 'tie_rod_inner' contains non-finite values"),
    (dict(upper_outer=[-5.0, 575.0, 110.0]), r"^Upper and lower ball joints are coincident — the upright would have zero length\.$"),
    (dict(upper_front_inner=[12.0, 540.0, 300.0]), r"^The upper front wishbone has zero length \(inner and outer points coincide\)\.$"),
    (dict(upper_rear_inner=[12.0, 540.0, 300.0]), r"^The upper rear wishbone has zero length"),
    (dict(lower_front_inner=[-5.0, 575.0, 110.0]), r"^The lower front wishbone has zero length"),
    (dict(lower_rear_inner=[-5.0, 575.0, 110.0]), r"^The lower rear wishbone has zero length"),
])
def test_bad_geometry_is_refused_with_a_message_that_names_it(change, message):
    hp = Hardpoints.default()
    for k, v in change.items():
        setattr(hp, k, np.array(v, float))
    with pytest.raises(ValueError, match=message):
        SuspensionKinematics(hp)


# ----------------------------------------------------- the rocker and its ratio
def test_the_rocker_rotates_by_the_right_hand_rule(kin):
    piv = np.asarray(kin.hp.rocker_pivot)
    assert kin._rotate_about_axis(piv + [0, 1, 0], np.pi / 2) == pytest.approx(piv + [0, 0, 1])
    assert kin._rotate_about_axis(piv + [0, 0, 2], np.pi / 2) == pytest.approx(piv + [0, -2, 0])
    assert kin._rotate_about_axis(piv + [3, 0, 0], 1.0) == pytest.approx(piv + [3, 0, 0])


def test_spring_length_follows_the_pushrod_through_the_rocker(kin):
    st = kin.solve_at_travel(12.0)
    L, theta, ok = kin.spring_length_at(st)
    assert ok
    # the rocker's pushrod point sits one pushrod length from the pushrod outer
    rp = kin._rotate_about_axis(kin.hp.rocker_pushrod, theta)
    assert np.linalg.norm(rp - st.pushrod_outer) == pytest.approx(
        np.linalg.norm(kin.hp.rocker_pushrod - kin.hp.pushrod_outer), abs=1e-6)
    rs = kin._rotate_about_axis(kin.hp.rocker_spring, theta)
    assert L == pytest.approx(np.linalg.norm(rs - kin.hp.spring_inner))
    from dataclasses import replace
    assert kin.spring_length_at(replace(st, pushrod_outer=None))[0] == pytest.approx(L, abs=1e-9)
    assert kin.spring_length_at(st, seed=0.3)[1] == pytest.approx(theta, abs=1e-6)
    hp = Hardpoints.default()
    hp.pushrod_outer = None
    with pytest.raises(ValueError, match="^No pushrod/rocker geometry defined on these hardpoints.$"):
        SuspensionKinematics(hp).spring_length_at(SuspensionKinematics(hp).static)


def _fd_ratio(k, t, h):
    su, sd = k.solve_at_travel(t + h), k.solve_at_travel(t - h)
    return abs((k.spring_length_at(su)[0] - k.spring_length_at(sd)[0])
               / (su.wheel_center[2] - sd.wheel_center[2]))


def test_motion_ratio_is_the_spring_over_wheel_travel(monkeypatch):
    k = SuspensionKinematics(Hardpoints.default())
    calls = []
    real = k._compute_motion_ratio
    monkeypatch.setattr(k, "_compute_motion_ratio",
                        lambda *a: calls.append(a) or real(*a))
    mr = k.motion_ratio()
    assert mr == pytest.approx(_fd_ratio(k, 0.0, 5.0), rel=1e-9)
    assert k.motion_ratio() == mr and len(calls) == 1                 # cached
    assert k.motion_ratio(d=2.0) == pytest.approx(_fd_ratio(k, 0.0, 2.0), rel=1e-9)
    assert len(calls) == 2 and calls[1] == (None, None, None, 2.0)    # not cached
    k.motion_ratio(spring_inner=[0, 0, 0])
    k.motion_ratio(rocker_pivot=[0, 0, 0])
    k.motion_ratio(push_outer_local=[0, 0, 0])
    assert len(calls) == 5 and k.motion_ratio_is_real()
    assert k.wheel_rate(30.0) == pytest.approx(30.0 * mr * mr)


def test_the_motion_ratio_curve_is_a_central_difference_at_each_travel(kin):
    t, mr = kin.motion_ratio_curve(-20, 20, 5)
    assert t == pytest.approx([-20, -10, 0, 10, 20])
    assert mr == pytest.approx([_fd_ratio(kin, x, 1.0) for x in t], rel=1e-9)
    assert mr[0] != pytest.approx(mr[-1], rel=1e-3)                   # it does vary


def test_the_direct_acting_proxy_projects_the_ball_joint_on_the_spring():
    hp = Hardpoints.default()
    hp.pushrod_outer = None                                           # no linkage, proxy points left
    k = SuspensionKinematics(hp)
    assert not k.motion_ratio_is_real()
    su, sd = k.solve_at_travel(5.0), k.solve_at_travel(-5.0)
    axis = _unit(np.asarray(hp.spring_inner) - np.asarray(hp.rocker_pivot))
    want = abs((su.lower_outer - sd.lower_outer) @ axis / (su.wheel_center[2] - sd.wheel_center[2]))
    assert k.motion_ratio() == pytest.approx(want, rel=1e-9)
    other = np.array([0.0, 0.0, 400.0])
    axis2 = _unit(other - np.asarray(hp.rocker_pivot))
    want2 = abs((su.lower_outer - sd.lower_outer) @ axis2 / (su.wheel_center[2] - sd.wheel_center[2]))
    assert k.motion_ratio(spring_inner=other) == pytest.approx(want2, rel=1e-9)
    assert k.motion_ratio_curve(-5, 5, 3) == ([-5.0, 0.0, 5.0], [k.motion_ratio()] * 3)
    hp.spring_inner = None
    assert SuspensionKinematics(hp).motion_ratio() == 1.0
    hp.spring_inner, hp.rocker_pivot = np.array([0.0, 0.0, 400.0]), None
    assert SuspensionKinematics(hp).motion_ratio() == 1.0


# ----------------------------------------------- sweep and side-view geometry
def test_sweep_returns_each_travel_solved_on_the_static_branch(kin):
    travels = [-30, -15, 0, 15, 30]
    states = kin.sweep(-30, 30, 5)
    assert [s.travel for s in states] == pytest.approx(travels)
    for s, t in zip(states, travels):
        cold = kin.solve_at_travel(t)
        assert s.lower_outer == pytest.approx(cold.lower_outer, abs=1e-7)
        assert s.upper_outer == pytest.approx(cold.upper_outer, abs=1e-7)
        assert s.lower_outer[2] == pytest.approx(kin.hp.lower_outer[2] + t, abs=1e-9)
    assert len(kin.sweep()) == 41 and kin.sweep(-4, 4, 3)[1].travel == 0.0


def _svic_by_finite_difference(k, travel, h=1e-3):
    """Side-view centre from the SOLVED motion of the ball joints, not from the pivot axes."""
    up, dn = k.solve_at_travel(travel + h), k.solve_at_travel(travel - h)
    st = k.solve_at_travel(travel)
    lines = []
    for j in ("upper_outer", "lower_outer"):
        v = np.asarray(getattr(up, j)) - np.asarray(getattr(dn, j))
        p = np.asarray(getattr(st, j))
        lines.append((np.array([p[0], p[2]]), np.array([-v[2], v[0]])))
    (p1, d1), (p2, d2) = lines
    t = np.linalg.solve(np.column_stack([d1, -d2]), p2 - p1)
    return p1 + t[0] * d1


@pytest.mark.parametrize("plan_angle", [False, True])
@pytest.mark.parametrize("travel", [0.0, 18.0])
def test_side_view_centre_matches_the_solved_motion(plan_angle, travel):
    hp = Hardpoints.default()
    if plan_angle:
        hp.upper_rear_inner = hp.upper_rear_inner + [0.0, 40.0, 0.0]
        hp.lower_rear_inner = hp.lower_rear_inner + [0.0, -30.0, 0.0]
    k = SuspensionKinematics(hp)
    st = k.solve_at_travel(travel)
    svic = k._side_view_swing_arm(st if travel else None)
    assert svic == pytest.approx(_svic_by_finite_difference(k, travel), rel=1e-5)
    assert k.side_view_swing_arm_length(st if travel else None) == pytest.approx(
        svic[0] - st.contact_patch[0], rel=1e-9)
    if plan_angle and not travel:
        assert svic[0] == pytest.approx(1852.44351528, rel=1e-6)      # no longer the hand line


def test_anti_geometry_uses_the_state_and_the_biases_it_is_given(kin):
    st = kin.solve_at_travel(15.0)

    def slope(point, t, d=0.5):
        up, dn = kin.solve_at_travel(t + d), kin.solve_at_travel(t - d)
        pu, pd = getattr(up, point), getattr(dn, point)
        return (pu[0] - pd[0]) / (pu[2] - pd[2])
    s_cp, s_wc = slope("contact_patch", 15.0), slope("wheel_center", 15.0)
    assert kin.anti_dive_pct(300, 1600, 0.6, state=st) == pytest.approx(-s_cp * 1600 / 300 * 0.6 * 100)
    assert kin.anti_squat_pct(300, 1600, 0.8, state=st) == pytest.approx(s_wc * 1600 / 300 * 0.8 * 100)
    assert kin.anti_lift_pct(300, 1600, 0.35, state=st) == pytest.approx(s_cp * 1600 / 300 * 0.35 * 100)
    assert kin.anti_lift_pct(300, 1600, 0.35, state=st, inboard_brakes=True) == pytest.approx(
        s_wc * 1600 / 300 * 0.35 * 100)
    assert kin.anti_dive_pct(300, 1600) == kin.anti_dive_pct(300, 1600, 0.65)
    assert kin.anti_squat_pct(300, 1600) == kin.anti_squat_pct(300, 1600, 1.0)
    assert kin.anti_lift_pct(300, 1600) == kin.anti_lift_pct(300, 1600, 0.40)
    assert kin._path_slope_xz("contact_patch", st) == pytest.approx(s_cp, rel=1e-9)
    assert kin._path_slope_xz("contact_patch") == pytest.approx(slope("contact_patch", 0.0), rel=1e-9)


# ------------------------------------------------------------ exact notes
def test_member_force_notes_say_exactly_what_was_assumed():
    hp = Hardpoints.default()
    hp.pushrod_attach = "upright"
    hp.pushrod_outer = np.array([-5.0, 520.0, 150.0])
    k = SuspensionKinematics(hp)
    assert lp.solve_member_forces(k, k.static, lp.WheelLoad(Fz=1000.0)).note == ""
    hub = " Hub force/moment at the wheel centre included (halfshaft)."
    for extra in (dict(F_wc_extra=(0.0, 5.0, 0.0)), dict(M_wc_extra=(0.0, 0.0, 7.0))):
        note = lp.solve_member_forces(k, k.static, lp.WheelLoad(Fz=1000.0, **extra)).note
        assert note == hub.strip()
    both = lp.solve_member_forces(k, k.static, lp.WheelLoad(Fz=1000.0, Fx_wc=250.0,
                                                            F_wc_extra=(1.0, 0.0, 0.0))).note
    assert both == (hub + " +250 N of longitudinal force applied at the wheel centre "
                    "(torque reacted by a shaft, not the upright).").strip()
    bare = Hardpoints.default()
    bare.pushrod_outer = None
    kb = SuspensionKinematics(bare)
    assert lp.solve_member_forces(kb, kb.static, lp.WheelLoad(Fz=1000.0, Fx_wc=-90.0)).note == (
        "No pushrod/rocker defined — a direct-acting vertical strut proxy was used for the "
        "vertical load path. Define the rocker for the real pushrod force. -90 N of "
        "longitudinal force applied at the wheel centre (torque reacted by a shaft, not the "
        "upright).")


def test_a_near_singular_linkage_is_flagged(monkeypatch):
    hp = Hardpoints.default()
    hp.pushrod_outer = None
    k = SuspensionKinematics(hp)
    real = np.linalg.cond
    monkeypatch.setattr(lp.np.linalg, "cond", lambda A: 3e9)
    mf = lp.solve_member_forces(k, k.static, lp.WheelLoad(Fz=1000.0))
    monkeypatch.setattr(lp.np.linalg, "cond", real)
    good = lp.solve_member_forces(k, k.static, lp.WheelLoad(Fz=1000.0))
    assert mf.note.endswith(" Equilibrium matrix is poorly conditioned (cond≈3.0e+09); the "
                            "linkage may be near a singular configuration — treat member loads "
                            "with caution.")
    assert mf.forces == pytest.approx(good.forces)            # still solved, by least squares
    assert "poorly conditioned" not in good.note


# ------------------------------------------------- what the algorithms promise
def test_sweep_marches_out_from_static_warm_starting_each_step(monkeypatch):
    k = SuspensionKinematics(Hardpoints.default())
    calls = []
    real = k.solve_at_travel

    def spy(t, seed=None):
        st = real(t, seed=seed)
        calls.append((t, None if seed is None else np.array(seed), st))
        return st
    monkeypatch.setattr(k, "solve_at_travel", spy)
    k.sweep(-20, 20, 5)
    order = [c[0] for c in calls]
    assert order == [0.0, 10.0, 20.0, -10.0, -20.0]
    pos = {c[0]: np.concatenate([c[2].lower_outer, c[2].upper_outer, c[2].tie_rod_outer])
           for c in calls}
    assert calls[0][1] is None
    for (t, seed, _), prev in zip(calls[1:], (0.0, 10.0, 0.0, -10.0)):
        assert seed == pytest.approx(pos[prev])


def test_the_motion_ratio_curve_warm_starts_the_rocker(monkeypatch):
    k = SuspensionKinematics(Hardpoints.default())
    seeds = []
    real = k.spring_length_at

    def spy(state, seed=0.0):
        out = real(state, seed=seed)
        seeds.append((seed, out[1]))
        return out
    monkeypatch.setattr(k, "spring_length_at", spy)
    k.motion_ratio_curve(-10, 10, 3)
    # calls alternate bump (t + 1) and droop (t - 1); each bump solve seeds from the last
    # bump angle, and its droop partner seeds from that bump solve's own angle
    assert seeds[0][0] == 0.0 and len(seeds) == 6
    for i in range(1, 6):
        assert seeds[i][0] == (seeds[i - 1][1] if i % 2 else seeds[i - 2][1])
    seeds.clear()
    k._compute_motion_ratio(None, None, None, 5.0)
    assert seeds[0][0] == 0.0 and seeds[1][0] == seeds[0][1]


def test_the_motion_ratio_cache_is_only_for_the_default_call(monkeypatch):
    k = SuspensionKinematics(Hardpoints.default())
    calls = []
    monkeypatch.setattr(k, "_compute_motion_ratio", lambda *a: calls.append(a) or 0.5)
    k.motion_ratio()
    k.motion_ratio(spring_inner=[1, 2, 3], rocker_pivot=[4, 5, 6], push_outer_local=[7, 8, 9], d=3.0)
    assert calls == [(None, None, None, 5.0), ([1, 2, 3], [4, 5, 6], [7, 8, 9], 3.0)]


# ------------------------------------------ thresholds are absolute, not relative
def test_a_corner_drawn_in_metres_solves_the_same_as_in_millimetres():
    """Every 'is this zero' threshold in the solvers sits far below any real dimension."""
    mm = Hardpoints.default()
    m = Hardpoints.from_dict({k: (np.asarray(v) * 1e-3 if isinstance(v, list) and k != "rocker_axis"
                                  else v) for k, v in mm.as_dict().items()})
    a, b = SuspensionKinematics(mm), SuspensionKinematics(m)
    for t in (0.0, 15.0):
        sa, sb = a.solve_at_travel(t), b.solve_at_travel(t * 1e-3)
        assert (sb.camber, sb.toe, sb.caster, sb.kpi) == pytest.approx(
            (sa.camber, sa.toe, sa.caster, sa.kpi), abs=1e-6)
        assert sb.scrub_radius == pytest.approx(sa.scrub_radius * 1e-3, rel=1e-6)
        assert sb.instant_center == pytest.approx(sa.instant_center * 1e-3, rel=1e-6)
        assert b._side_view_swing_arm(sb) == pytest.approx(a._side_view_swing_arm(sa) * 1e-3, rel=1e-6)
    kin_unit = __import__("suspension.kinematics", fromlist=["_unit"])._unit
    for unit in (kin_unit, lp._unit):
        assert unit(np.array([0.3, 0.4, 0.0])) == pytest.approx([0.6, 0.8, 0.0])
        assert unit(np.zeros(3)) == pytest.approx(np.zeros(3))


def test_one_square_and_one_staggered_arm_still_meet():
    """One pivot axis along x makes one projected velocity component exactly zero; that is not degenerate."""
    hp = Hardpoints.default()
    hp.upper_rear_inner = np.array([130.0, 240.0, hp.upper_front_inner[2]])    # upper axis along x
    k = SuspensionKinematics(hp)
    sv = k._side_view_swing_arm()
    assert np.all(np.isfinite(sv))
    assert sv == pytest.approx(_svic_by_finite_difference(k, 0.0), rel=1e-5)
    flat = Hardpoints.default()
    flat.lower_front_inner = np.array([-110.0, 200.0, 110.0])
    flat.lower_rear_inner = np.array([140.0, 200.0, 110.0])          # lower arm level, along x
    st = SuspensionKinematics(flat).static
    assert np.all(np.isfinite(st.instant_center))


def test_the_solved_state_is_marked():
    k = SuspensionKinematics(Hardpoints.default())
    st = k.solve_at_travel(10.0)
    assert np.isnan(st.roll_center_height) and isinstance(st.roll_center_height, float)
    assert st.converged is True and st.travel == 10.0


def test_shaft_reaction_needs_a_nonzero_longitudinal_force():
    for flags in (dict(inboard_drive=True), dict(inboard_brakes=True)):
        assert lp.wheel_load_from_corner(1000.0, mu_long=0.0, **flags).Fx_wc == 0.0


def test_the_bending_arm_is_clipped_to_the_leg():
    for hp_fn, expect in ((_mount_past_the_ball_joint, "length"), (_mount_inboard_of_the_pivots, "zero")):
        hp = hp_fn()
        k = SuspensionKinematics(hp)
        mf = lp.solve_member_forces(k, k.static, LOADS[1])
        bj = np.asarray(k.static.lower_outer)
        for n, leg in (("LF", hp.lower_front_inner), ("LR", hp.lower_rear_inner)):
            want = np.linalg.norm(bj - np.asarray(leg)) if expect == "length" else 0.0
            assert mf.bending_arm_mm[n] == pytest.approx(want)


# ------------------------------------------------ conversions, flags and seeds
def test_points_are_stored_and_converted_as_floats():
    from suspension import kinematics as km
    for vec in (km._vec, lp._vec):
        assert vec([1, 2, 3]).dtype == np.float64
    ints = {k: ([int(round(x)) for x in v] if isinstance(v, list) else v)
            for k, v in Hardpoints.default().as_dict().items()}
    hp = Hardpoints.from_dict(ints)
    for k, v in vars(hp).items():
        if isinstance(v, np.ndarray):
            assert v.dtype == np.float64, k


def test_a_rocker_that_does_not_close_gives_no_motion_ratio(monkeypatch):
    k = SuspensionKinematics(Hardpoints.default())
    real = k.spring_length_at
    for bad in (0, 1):                     # first (bump) or second (droop) rocker solve fails
        calls = []

        def fake(state, seed=0.0):
            length, theta, _ = real(state, seed=seed)
            calls.append(None)
            return length, theta, len(calls) - 1 != bad
        monkeypatch.setattr(k, "spring_length_at", fake)
        assert np.isnan(k._compute_motion_ratio(None, None, None, 5.0))
        calls.clear()
        assert np.isnan(k.motion_ratio_curve(-5.0, 5.0, 1)[1][0])


def test_spring_length_passes_its_seed_and_flags_a_rocker_that_does_not_close(monkeypatch):
    k = SuspensionKinematics(Hardpoints.default())
    seeds = []
    monkeypatch.setattr(k, "_solve_rocker_angle", lambda pro, seed: seeds.append(seed) or (0.0, 0.5))
    assert k.spring_length_at(k.static, seed=0.3)[2] is False
    k.spring_length_at(k.static)
    assert seeds == [0.3, 0.0]


def test_solve_at_travel_starts_from_its_seed_and_reports_a_failed_solve(monkeypatch):
    # patch the solver where solve_at_travel looks it up (another test may have reloaded the module); unwrap first,
    # because under mutmut the method is a decorator defined in mutmut's own module
    module = inspect.unwrap(SuspensionKinematics.solve_at_travel).__globals__
    k = SuspensionKinematics(Hardpoints.default())
    st = k.static
    seed = np.concatenate([st.lower_outer, st.upper_outer, st.tie_rod_outer]) + 0.5
    starts = []
    real = module["least_squares"]

    def spy(fun, x0, **kw):
        starts.append(np.array(x0))
        return real(fun, x0, **kw)
    monkeypatch.setitem(module, "least_squares", spy)
    k.solve_at_travel(3.0, seed=seed)
    assert starts[-1] == pytest.approx(seed)

    class Unsolved:
        x, fun = seed, np.full(9, 5.0)
    monkeypatch.setitem(module, "least_squares", lambda *a, **kw: Unsolved)
    assert k.solve_at_travel(3.0).converged is False


def test_the_default_ranges():
    k = SuspensionKinematics(Hardpoints.default())
    assert [s.travel for s in k.sweep()] == pytest.approx(np.linspace(-30.0, 30.0, 41))
    assert k.motion_ratio_curve()[0] == pytest.approx(np.linspace(-25.0, 25.0, 21))


def test_a_rocker_car_state_without_a_pushrod_point_falls_back_to_the_strut():
    import dataclasses
    k = SuspensionKinematics(Hardpoints.default())
    st = dataclasses.replace(k.static, pushrod_outer=None)
    mf = lp.solve_member_forces(k, st, LOADS[1])
    assert not mf.has_pushrod and "PR" not in mf.axes and "STRUT" in mf.axes
    assert all(np.isfinite(f) for f in mf.forces.values())


def test_notes_add_up_rather_than_replace_each_other(monkeypatch):
    bare = Hardpoints.default()
    bare.pushrod_outer = None
    k = SuspensionKinematics(bare)
    real = lp._member_geometry

    def seven(kk, st):
        g = real(kk, st)
        g["UF2"] = g["UF"]
        return g
    monkeypatch.setattr(lp, "_member_geometry", seven)
    note = lp.solve_member_forces(k, k.static, lp.WheelLoad(Fz=1000.0, F_wc_extra=(0.0, 5.0, 0.0))).note
    assert note.startswith("No pushrod/rocker defined")
    assert "(Expected 6 force members for a determinate solve, got 7.)" in note
    assert "Hub force/moment at the wheel centre included (halfshaft)." in note
