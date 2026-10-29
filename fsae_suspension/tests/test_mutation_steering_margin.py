"""suspension/steering_margin.py on the default corner with a stand-in manifest: the hand torque is rebuilt from
steering_feel's aligning curves, each bisection is checked at its root, the spindle offset moves only the wheel, the
wall and band verdicts follow their declared logic, and the lock check's arms and jacking are recomputed from the
solved steered corner by virtual work."""
import math
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from suspension import inverse_genesis as ig
from suspension import steering_feel as sf
from suspension import steering_margin as sm
from suspension.kinematics import Hardpoints, SuspensionKinematics
from suspension.tiremodel import default_tire

HP = Hardpoints.default()
TIRE = default_tire()
SE = ig.SteerEffort()


def _manifest(se=SE, recorded=None, start=HP, props=None, tg=None, vol_kw=None):
    vol = SimpleNamespace(properties=props if props is not None else SimpleNamespace(steer_effort=se), **(vol_kw or {}))
    return SimpleNamespace(objects=lambda: (start, tg, vol, None), recorded=recorded)


def _hand(trail, ratio, fo, fi, t0=None, amax=22.0):
    t0o = sf.brush_trail_mm(fo) if t0 is None else t0
    co = sf.aligning_curve(TIRE, fo, trail, t0o, alpha_max_deg=amax, n=441)
    ci = sf.aligning_curve(TIRE, fi, trail, sf.brush_trail_mm(fi), alpha_max_deg=amax, n=441)
    return float(np.max(co["M_Nm"] + ci["M_Nm"])) / ratio


def test_mechanical_trail_and_the_declared_effort():
    assert sm.mechanical_trail(HP) == pytest.approx(ig.mechanical_trail_mm(SuspensionKinematics(HP).static))
    se = replace(SE, effort_limit_nm=7.5)
    assert sm.steer_effort(_manifest(se)) is se
    assert sm.steer_effort(_manifest(props=SimpleNamespace())) == ig.SteerEffort()
    assert sm.steer_effort(_manifest(props=SimpleNamespace(steer_effort=None))) == ig.SteerEffort()


def test_hand_torque_from_both_wheels_aligning_curves():
    assert sm.hand_torque(TIRE, 20.0, 5.0, 1250.0, 165.0) == pytest.approx(_hand(20.0, 5.0, 1250.0, 165.0))
    assert sm.hand_torque(TIRE, 20.0, 5.0, 1250.0, 165.0, 35.0) == pytest.approx(_hand(20.0, 5.0, 1250.0, 165.0, 35.0))
    assert sm.hand_torque(TIRE, 20.0, 5.0, 1250.0, 165.0, alpha_max_deg=10.0) == pytest.approx(
        _hand(20.0, 5.0, 1250.0, 165.0, amax=10.0))


def test_the_trail_limits_are_roots_of_the_effort_wall():
    lim = sm.trail_limit(TIRE, 15.0, 4.0, 1250.0, 165.0, effort_limit_nm=12.0)
    assert 2.0 < lim < 80.0
    assert _hand(15.0, 4.0, 1250.0, 165.0, lim) < 12.0 <= _hand(15.0, 4.0, 1250.0, 165.0, lim + 1e-6)
    d = sm.trail_limit(TIRE, 15.0, 4.0, 1250.0, 165.0)
    assert _hand(15.0, 4.0, 1250.0, 165.0, d) < 10.0 <= _hand(15.0, 4.0, 1250.0, 165.0, d + 1e-6)
    assert sm.trail_limit(TIRE, 15.0, 4.0, 1250.0, 165.0, effort_limit_nm=1e6) == pytest.approx(80.0, abs=1e-9)
    assert sm.trail_limit(TIRE, 15.0, 4.0, 1250.0, 165.0, effort_limit_nm=1e-6) == pytest.approx(2.0, abs=1e-9)
    t = sm.trail_for_effort(TIRE, 4.0, 1250.0, 165.0, 40.0, 30.0, effort_limit_nm=12.0)
    assert -20.0 < t < 30.0
    assert _hand(t, 4.0, 1250.0, 165.0, 40.0) < 12.0 <= _hand(t + 1e-6, 4.0, 1250.0, 165.0, 40.0)
    t10 = sm.trail_for_effort(TIRE, 4.0, 1250.0, 165.0, None, 30.0)
    assert _hand(t10, 4.0, 1250.0, 165.0) < 10.0 <= _hand(t10 + 1e-6, 4.0, 1250.0, 165.0) or t10 == pytest.approx(30.0)
    assert sm.trail_for_effort(TIRE, 4.0, 1250.0, 165.0, None, 30.0, effort_limit_nm=1e6) == pytest.approx(30.0)


def test_the_spindle_offset_moves_only_the_wheel():
    o = sm.spindle_offset(HP, -6.0)
    assert np.asarray(o.wheel_center) == pytest.approx(np.asarray(HP.wheel_center) + [-6, 0, 0])
    assert np.asarray(o.contact_patch) == pytest.approx(np.asarray(HP.contact_patch) + [-6, 0, 0])
    assert np.asarray(o.upper_outer) == pytest.approx(HP.upper_outer)
    assert sm.mechanical_trail(o) == pytest.approx(sm.mechanical_trail(HP) - 6.0, abs=1e-6)
    dx = sm.offset_for_trail(HP, 12.0)
    assert sm.mechanical_trail(sm.spindle_offset(HP, dx)) == pytest.approx(12.0, abs=1e-6)
    assert sm.offset_for_trail(HP, 500.0) == pytest.approx(40.0, abs=1e-9)
    assert sm.offset_for_trail(HP, -500.0) == pytest.approx(-40.0, abs=1e-9)


def test_the_corner_is_the_recorded_winner_or_the_start():
    from suspension.genesis_repro import hp_from_dict
    other = replace(HP, upper_outer=np.array([12.0, 541.0, 300.0]))
    assert sm.corner(_manifest(recorded={})) is HP and sm.corner(_manifest(recorded=None)) is HP
    rec = {"winner_hardpoints": other.as_dict()}
    assert np.asarray(sm.corner(_manifest(recorded=rec)).upper_outer) == pytest.approx(
        np.asarray(hp_from_dict(other.as_dict()).upper_outer))


class _Tg:
    def __init__(self, band, ok=True):
        self.band, self.ok = band, ok

    def residual(self, h):
        return np.array([0.2, -self.band]), self.ok


def _vol(bounds=True, props=None, keepout=(), viol=(), margin=2.0):
    # a wall asked about anything but a corner fails, so the checks must be handed the offset corner
    return dict(has_property_bounds=lambda: bounds, keepout_violations=lambda h: ["?"] if h is None else list(keepout),
                property_violations=lambda h: ["?"] if h is None else list(viol), envelope_margin=lambda h: margin)


def test_the_offset_check_verdicts():
    props = SimpleNamespace(evaluate=lambda h: {"trail_mm": sm.mechanical_trail(h)}, steer_effort=SE)
    m = _manifest(props=props, tg=_Tg(0.8), vol_kw=_vol())
    r = sm.offset_check(m, -4.0)
    assert r["dx_mm"] == -4.0 and r["walls_hold"] is True and r["bands_hold"] is True
    assert r["trail_mm"] == pytest.approx((sm.mechanical_trail(HP), sm.mechanical_trail(sm.spindle_offset(HP, -4.0))))
    assert r["before"] == pytest.approx({"trail_mm": r["trail_mm"][0], "band": 0.8, "clearance_mm": 2.0})
    assert r["after"]["trail_mm"] == pytest.approx(r["trail_mm"][1])
    for kw, walls in ((dict(keepout=("x",)), False), (dict(viol=("x",)), False), (dict(margin=-0.1), False),
                      (dict(margin=None), True), (dict(bounds=False, viol=("x",)), True), (dict(margin=0.0), True)):
        m = _manifest(props=props, tg=_Tg(0.8), vol_kw=_vol(**kw))
        assert sm.offset_check(m, -4.0)["walls_hold"] is walls, kw
    assert sm.offset_check(_manifest(props=props, tg=_Tg(1.2), vol_kw=_vol()), 1.0)["bands_hold"] is False
    assert sm.offset_check(_manifest(props=props, tg=_Tg(1.0), vol_kw=_vol()), 1.0)["bands_hold"] is True
    nan = sm.offset_check(_manifest(props=props, tg=_Tg(0.5, ok=False), vol_kw=_vol()), 1.0)
    assert math.isnan(nan["after"]["band"]) and nan["bands_hold"] is False
    nb = sm.offset_check(_manifest(props=props, tg=_Tg(0.5), vol_kw=_vol(bounds=False, margin=None)), 1.0)
    assert set(nb["before"]) == {"band", "clearance_mm"} and nb["before"]["clearance_mm"] is None
    other = replace(HP, upper_outer=np.array([20.0, 540.0, 300.0]))
    assert abs(sm.mechanical_trail(other) - sm.mechanical_trail(HP)) > 1.0
    h = sm.offset_check(_manifest(props=props, tg=_Tg(0.5), vol_kw=_vol()), 1.0, hp=other)
    assert h["trail_mm"][0] == pytest.approx(sm.mechanical_trail(other))


def test_the_margin_report():
    m = _manifest(se=replace(SE, effort_limit_nm=11.0))
    r = sm.margin(m, {"a": TIRE}, t0_assumed_mm=40.0)
    se = replace(SE, effort_limit_nm=11.0)
    ratio, trail = se.kinematic_ratio(HP), sm.mechanical_trail(HP)
    assert (r["trail_mm"], r["ratio"]) == pytest.approx((trail, ratio))
    assert (r["fz_outer_n"], r["fz_inner_n"], r["effort_limit_nm"], r["t0_assumed_mm"]) == (1247.7, 164.9, 11.0, 40.0)
    t = r["tires"]["a"]
    assert t["hand_brush_Nm"] == pytest.approx(_hand(trail, ratio, 1247.7, 164.9))
    assert t["t0_limit_mm"] == pytest.approx(sm.trail_limit(TIRE, trail, ratio, 1247.7, 164.9, 11.0))
    assert t["hand_assumed_Nm"] == pytest.approx(_hand(trail, ratio, 1247.7, 164.9, 40.0))
    assert t["trail_for_limit_mm"] == pytest.approx(sm.trail_for_effort(TIRE, ratio, 1247.7, 164.9, 40.0, trail,
                                                                        11.0))
    plain = sm.margin(_manifest(), {"a": TIRE})["tires"]["a"]
    assert set(plain) == {"hand_brush_Nm", "t0_limit_mm"}


def test_the_lateral_arm_about_the_steering_axis():
    st = SuspensionKinematics(HP).static
    U, L, cp = (np.asarray(v, float) for v in (st.upper_outer, st.lower_outer, st.contact_patch))
    k = (U - L) / np.linalg.norm(U - L)
    t = math.radians(st.toe)
    assert sm._lateral_arm(st) == pytest.approx(abs(np.cross(cp - L, [math.sin(t), math.cos(t), 0.0]) @ k))


def test_the_lock_check_by_virtual_work():
    r = sm.lock_check(_manifest())
    c = SE.rack_c_mm_per_rev
    x = SE.lock_sw_deg(HP) * c / 360.0
    st = lambda rk: SuspensionKinematics(HP, pickup_deltas={"tie_rod_inner": np.array([0.0, rk, 0.0])}).solve_at_travel(0.0)
    sgn = 1.0 if st(1.0).toe > st(0.0).toe else -1.0
    xi, xo = sgn * x, -sgn * x
    dz = lambda rk: (st(rk + 0.25).contact_patch[2] - st(rk - 0.25).contact_patch[2]) / 0.5
    dzi, dzo = sgn * dz(xi), -sgn * dz(xo)
    lever = c / (2 * math.pi) / 1000
    jack = lambda fi, fo: -(fi * dzi + fo * dzo)
    assert r["rack_mm"] == pytest.approx(x)
    assert (r["steer_inner_deg"], r["steer_outer_deg"]) == pytest.approx((st(xi).toe, st(xo).toe))
    a0, ai, ao = sm._lateral_arm(st(0.0)), sm._lateral_arm(st(xi)), sm._lateral_arm(st(xo))
    assert (r["arm_straight_mm"], r["arm_inner_mm"], r["arm_outer_mm"]) == pytest.approx((a0, ai, ao))
    jd = jack(SE.fz_inner_n, SE.fz_outer_n)
    s = 0.5 * (SE.fz_inner_n + SE.fz_outer_n)
    js = jack(s, s)
    assert r["jacking_design"] == pytest.approx({"rack_N": jd, "hand_Nm": jd * lever})
    assert r["jacking_static"] == pytest.approx({"rack_N": js, "hand_Nm": js * lever})
    base = _hand(sm.mechanical_trail(HP), SE.kinematic_ratio(HP), SE.fz_outer_n, SE.fz_inner_n, SE.t0_outer_mm)
    assert r["hand_straight_Nm"] == pytest.approx(base)
    assert r["hand_lock_Nm"] == pytest.approx(base * max(ai, ao) / a0 + max(jd * lever, js * lever, 0.0))
    fine = sm.lock_check(_manifest(), hp=HP, tire=TIRE, h_mm=0.1)
    assert fine["jacking_design"]["rack_N"] == pytest.approx(jd, rel=1e-2)


def test_standstill_scrub_torque():
    a = math.sqrt(1200.0 / (0.083 * math.pi))
    assert sm.standstill_scrub_torque(1200.0, 1.6) == pytest.approx(2 / 3 * 1.6 * 1200 * a / 1000)
    a2 = math.sqrt(1200.0 / (0.07 * math.pi))
    assert sm.standstill_scrub_torque(1200.0, 1.6, inflation_kpa=70.0) == pytest.approx(2 / 3 * 1.6 * 1200 * a2 / 1000)


def test_a_volume_without_properties_declares_the_default_effort():
    m = SimpleNamespace(objects=lambda: (HP, None, SimpleNamespace(), None), recorded=None)
    assert sm.steer_effort(m) == ig.SteerEffort()


def test_the_margin_passes_its_corner_and_loads_through(monkeypatch):
    other = replace(HP, upper_outer=np.array([20.0, 540.0, 300.0]))
    se = replace(SE, effort_limit_nm=9.0, fz_outer_n=1300.0, fz_inner_n=150.0)
    monkeypatch.setattr(sm, "hand_torque", lambda *a: ("hand",) + a)
    monkeypatch.setattr(sm, "trail_limit", lambda *a: ("lim",) + a)
    monkeypatch.setattr(sm, "trail_for_effort", lambda *a: ("eff",) + a)
    r = sm.margin(_manifest(se=se), {"a": TIRE}, t0_assumed_mm=40.0, hp=other)
    trail, ratio = sm.mechanical_trail(other), se.kinematic_ratio(other)
    assert r["trail_mm"] == pytest.approx(trail) and r["ratio"] == pytest.approx(ratio)
    t = r["tires"]["a"]
    assert t["hand_brush_Nm"] == ("hand", TIRE, r["trail_mm"], r["ratio"], 1300.0, 150.0)
    assert t["t0_limit_mm"] == ("lim", TIRE, r["trail_mm"], r["ratio"], 1300.0, 150.0, 9.0)
    assert t["hand_assumed_Nm"] == ("hand", TIRE, r["trail_mm"], r["ratio"], 1300.0, 150.0, 40.0)
    assert t["trail_for_limit_mm"] == ("eff", TIRE, r["ratio"], 1300.0, 150.0, 40.0, r["trail_mm"], 9.0)


def test_the_lateral_arm_on_a_steered_skew_axis():
    st = SimpleNamespace(upper_outer=(14.0, 530.0, 310.0), lower_outer=(-6.0, 570.0, 105.0),
                         contact_patch=(9.0, 612.0, 3.0), toe=17.0)
    U, L, cp = (np.asarray(v, float) for v in (st.upper_outer, st.lower_outer, st.contact_patch))
    k = (U - L) / np.linalg.norm(U - L)
    t = math.radians(17.0)
    assert sm._lateral_arm(st) == pytest.approx(abs(np.cross(cp - L, [math.sin(t), math.cos(t), 0.0]) @ k), rel=1e-12)


@pytest.mark.parametrize("a, b", [(0.5, -0.01), (-0.1, 0.01)])
def test_the_lock_check_on_a_rack_that_steers_positively(monkeypatch, a, b):
    """A stand-in steered corner: toe grows with the rack (sgn +1), the patch height is a*x + b*x^2 (exact under the
    central difference) and moves out along x. (a, b) = (0.5, -0.01) puts the design jacking above a positive static
    one; (-0.1, 0.01) makes both resist negatively, so the floor at zero shows."""
    other = replace(HP, upper_outer=np.array([20.0, 540.0, 300.0]))

    def fake(hp, r):
        assert hp is other
        return SimpleNamespace(toe=3.0 * r, contact_patch=(5.0 + 0.4 * r, 610.0, a * r + b * r * r),
                               upper_outer=(14.0, 530.0, 310.0), lower_outer=(-6.0, 570.0, 105.0))
    calls = []
    monkeypatch.setattr(sm, "_steered", fake)
    monkeypatch.setattr(sm, "hand_torque", lambda *args: calls.append(args) or 4.0)
    se = SimpleNamespace(rack_c_mm_per_rev=80.0, lock_sw_deg=lambda hp: 90.0, kinematic_ratio=lambda hp: 5.0,
                         fz_inner_n=200.0, fz_outer_n=1200.0, t0_outer_mm=33.0)
    tire = object()
    r = sm.lock_check(_manifest(se=se, start=HP), hp=other, tire=tire)
    x = 20.0
    assert calls == [(tire, sm.mechanical_trail(other), 5.0, 1200.0, 200.0, 33.0)]
    assert r["rack_mm"] == pytest.approx(x)
    assert (r["steer_inner_deg"], r["steer_outer_deg"]) == pytest.approx((60.0, -60.0))
    dz = lambda xr: a + 2 * b * xr
    dzi, dzo = dz(x), -dz(-x)
    lever = 80.0 / (2 * math.pi) / 1000
    jd = -(200.0 * dzi + 1200.0 * dzo)
    js = -(700.0 * dzi + 700.0 * dzo)
    assert r["jacking_design"] == pytest.approx({"rack_N": jd, "hand_Nm": jd * lever}, rel=1e-9)
    assert r["jacking_static"] == pytest.approx({"rack_N": js, "hand_Nm": js * lever}, rel=1e-9)
    arms = [sm._lateral_arm(fake(other, v)) for v in (0.0, x, -x)]
    assert (r["arm_straight_mm"], r["arm_inner_mm"], r["arm_outer_mm"]) == pytest.approx(arms)
    assert r["hand_straight_Nm"] == 4.0
    want = 4.0 * max(arms[1:]) / arms[0] + max(jd * lever, js * lever, 0.0)
    assert r["hand_lock_Nm"] == pytest.approx(want, rel=1e-12)
    assert (jd > js > 0) if a > 0 else (js < 0 and jd < 0)


def test_the_lock_check_defaults_to_the_default_tire(monkeypatch):
    seen = []
    monkeypatch.setattr(sm, "hand_torque", lambda tire, *a: seen.append(tire) or 4.0)
    monkeypatch.setattr(sm, "_steered", lambda hp, r: SimpleNamespace(
        toe=3.0 * r, contact_patch=(5.0, 610.0, 0.1 * r), upper_outer=(14.0, 530.0, 310.0),
        lower_outer=(-6.0, 570.0, 105.0)))
    sm.lock_check(_manifest())
    assert seen[0].__dict__ == default_tire().__dict__ if hasattr(seen[0], "__dict__") else seen[0] == default_tire()
