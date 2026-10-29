"""suspension/robustness.py against what each function promises: the neutral share is where the two axles' capacities
match, the clevis effect is the linkage re-solved with the pickup moved along its link, the member margins follow
Euler and Perry-Robertson by hand on loadpath's forces, the hand roll centre agrees with the kinematic solver's own
instant centre, and the shock cases and brake requirement are their declared formulas."""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from suspension import loadpath as lp
from suspension import robustness as rb
from suspension import target_derivation as td
from suspension.kinematics import Hardpoints, SuspensionKinematics

HP = Hardpoints.default()


def _cap(share, a_lat, kf, kr, wfrac=0.48):
    v = td.Vehicle()
    lt = v.mass_kg * 9.81 * a_lat * v.cg_height_mm / v.track_mm
    wf, wr = v.mass_kg * 9.81 * wfrac / 2, v.mass_kg * 9.81 * (1 - wfrac) / 2
    cf = kf * (td.mu_of_load(wf + share * lt) * (wf + share * lt) + td.mu_of_load(wf - share * lt) * (wf - share * lt))
    cr = kr * (td.mu_of_load(wr + (1 - share) * lt) * (wr + (1 - share) * lt)
               + td.mu_of_load(wr - (1 - share) * lt) * (wr - (1 - share) * lt))
    return (cf / (2 * wf)) / (cr / (2 * wr))


def test_the_capacity_ratio():
    v = td.Vehicle()
    for args in ((0.5, 1.5, 1.0, 1.0), (0.55, 1.2, 0.95, 1.0), (0.45, 1.5, 1.0, 1.05, 0.5)):
        assert rb._capacity_ratio(v, *args) == pytest.approx(_cap(*args))
    assert rb.G0 == 9.81


def test_the_neutral_share_and_what_restores_the_margin():
    r = rb.balance_under_tire_change(front_mu_scale=0.95)
    assert _cap(r["neutral_share"], 1.5, 0.95, 1.0) == pytest.approx(1.0, abs=1e-9)
    assert r["target_share"] == pytest.approx(r["neutral_share"] + 0.02)
    now = td.front_lltd_share(58.5, 39.5, 454.0 / (454.0 + 405.0))
    s = td.stiffness_share_for_lltd(r["target_share"], 58.5, 39.5)
    assert r == pytest.approx({"neutral_share": r["neutral_share"], "target_share": r["target_share"],
                               "current_share": now, "share_error_points": 100 * (now - r["target_share"]),
                               "front_rc_change_mm": (r["target_share"] - now) / td.share_per_mm_front_rc(td.Vehicle()),
                               "front_bar_change_nm_deg": s / (1 - s) * 405.0 - 454.0})
    base = rb.balance_under_tire_change()
    assert base["neutral_share"] > r["neutral_share"]                  # hot fronts move neutral down
    o = rb.balance_under_tire_change(rear_mu_scale=0.97, margin=0.03, a_lat_g=1.2, k_rear_nm_deg=380.0,
                                     k_front_nm_deg=500.0, rc_front_mm=50.0, rc_rear_mm=45.0)
    assert _cap(o["neutral_share"], 1.2, 1.0, 0.97) == pytest.approx(1.0, abs=1e-9)
    assert o["target_share"] == pytest.approx(o["neutral_share"] + 0.03)
    assert o["current_share"] == pytest.approx(td.front_lltd_share(50.0, 45.0, 500.0 / 880.0))
    s = td.stiffness_share_for_lltd(o["target_share"], 50.0, 45.0)
    assert o["front_bar_change_nm_deg"] == pytest.approx(s / (1 - s) * 380.0 - 500.0)


def _curves(deltas=None, travel=25.0):
    k = SuspensionKinematics(HP, pickup_deltas=deltas or {})
    toes = [k.solve_at_travel(t).toe for t in np.linspace(-travel, travel, 11)]
    return {"camber": k.solve_at_travel(0.0).camber, "toe": k.solve_at_travel(0.0).toe,
            "bump_steer": max(toes) - min(toes),
            "camber_gain": (k.solve_at_travel(travel).camber - k.solve_at_travel(-travel).camber) / (2 * travel)}


def test_the_curves_and_the_clevis_sensitivity():
    assert rb._curves(HP) == pytest.approx(_curves())
    assert rb._curves(HP, travel=10.0) == pytest.approx(_curves(travel=10.0))
    assert rb.LINKS == {"upper_front_inner": "upper_outer", "upper_rear_inner": "upper_outer",
                        "lower_front_inner": "lower_outer", "lower_rear_inner": "lower_outer",
                        "tie_rod_inner": "tie_rod_outer"}
    r = rb.clevis_sensitivity(HP, deflection_mm=0.5)
    base = _curves()
    for inner, outer in rb.LINKS.items():
        u = np.asarray(getattr(HP, outer), float) - np.asarray(getattr(HP, inner), float)
        c = _curves({inner: u / np.linalg.norm(u) * 0.5})
        assert r[inner] == pytest.approx({k: c[k] - base[k] for k in base})
    one = rb.clevis_sensitivity(HP)
    u = np.asarray(HP.tie_rod_outer, float) - np.asarray(HP.tie_rod_inner, float)
    c = _curves({"tie_rod_inner": u / np.linalg.norm(u)})
    assert one["tie_rod_inner"]["toe"] == pytest.approx(c["toe"] - base["toe"])


def test_a_preset_spare_tie_rod():
    r = rb.preset_tie_rod_toe(120.0)
    flat = 25.4 / 24 / 6
    assert r == pytest.approx({"preset_error_deg": math.degrees(math.atan(0.05 / 120.0)),
                               "half_flat_error_deg": math.degrees(math.atan(0.5 * flat / 120.0)), "band_deg": 0.08})
    r = rb.preset_tie_rod_toe(100.0, length_tol_mm=0.1, tpi=28.0, flats=4)
    assert r["half_flat_error_deg"] == pytest.approx(math.degrees(math.atan(0.5 * 25.4 / 28 / 4 / 100.0)))
    assert r["preset_error_deg"] == pytest.approx(math.degrees(math.atan(0.1 / 100.0)))


def _margins(loads, od=15.875, wall=0.889, E=205000.0, sy=460.0, crook=1e-3):
    A = math.pi / 4 * (od ** 2 - (od - 2 * wall) ** 2)
    I = math.pi / 64 * (od ** 4 - (od - 2 * wall) ** 4)
    c = od / 2
    kin = SuspensionKinematics(HP)
    st = kin.static
    out = {}
    for case, ld in loads.items():
        mf = lp.solve_member_forces(kin, st, ld)
        for m, F in mf.forces.items():
            a, b = rb._MEMBER_ENDS[m]
            L = float(np.linalg.norm(np.asarray(getattr(HP, a)) - np.asarray(getattr(HP, b))))
            pcr = math.pi ** 2 * E * I / L ** 2
            Mb = (mf.transverse or {}).get(m, 0.0) * (mf.bending_arm_mm or {}).get(m, 0.0)
            if F < 0:
                amp = 1 / (1 + F / pcr)
                fb, fy = pcr / -F, sy / (-F / A * (1 + crook * L * c / (I / A) * amp) + Mb / (I / c) * amp)
            else:
                fb, fy = math.inf, sy / max(F / A + Mb / (I / c), 1e-9)
            r = out.setdefault(m, {"length_mm": L, "p_cr_n": pcr, "worst_n": 0.0, "worst_case": "",
                                   "fos_buckling": math.inf, "fos_yield": math.inf})
            if abs(F) > abs(r["worst_n"]):
                r["worst_n"], r["worst_case"] = F, case
            r["fos_buckling"], r["fos_yield"] = min(r["fos_buckling"], fb), min(r["fos_yield"], fy)
    return out


def test_member_margins_by_euler_and_perry_robertson():
    loads = {"corner": lp.WheelLoad(Fy=-2500.0, Fz=1800.0), "brake": lp.WheelLoad(Fx=2000.0, Fz=1500.0)}
    r = rb.member_margins(HP, loads)
    want = _margins(loads)
    assert set(r) == set(want) == {"UF", "UR", "LF", "LR", "TR", "PR"}
    for m in want:
        assert r[m] == pytest.approx(want[m])
    assert {r[m]["worst_case"] for m in r} <= {"corner", "brake"}
    alt = rb.member_margins(HP, loads, od_mm=19.05, wall_mm=1.2, e_mpa=70000.0, sy_mpa=300.0, crook=2e-3)
    want = _margins(loads, 19.05, 1.2, 70000.0, 300.0, 2e-3)
    for m in want:
        assert alt[m] == pytest.approx(want[m])
    huge = rb.member_margins(HP, {"x": lp.WheelLoad(Fy=-4e6, Fz=3e6)}, od_mm=6.0, wall_mm=0.5)
    comp = [m for m, v in huge.items() if v["worst_n"] < -v["p_cr_n"]]
    assert comp and all(huge[m]["fos_yield"] == 0.0 for m in comp)      # past buckling: no yield margin left


def test_the_overtravel_check(monkeypatch):
    r = rb.overtravel_check(HP, bump_mm=30.0, droop_mm=20.0, n=11)
    from suspension.actuation import actuation_metrics
    m = actuation_metrics(HP, travel_mm=(-20.0, 30.0), n=11)
    mr, trv, ta = np.asarray(m.mr), np.asarray(m.travel), np.asarray(m.transmission_deg)
    used = np.concatenate([[0.0], np.cumsum(0.5 * (mr[1:] + mr[:-1]) * np.diff(trv))])
    assert r["ok"] is True and r["stroke_used_mm"] == pytest.approx(used[-1])
    assert (r["mr_min"], r["mr_max"]) == pytest.approx((mr.min(), mr.max()))
    assert (r["transmission_min_deg"], r["transmission_max_deg"]) == pytest.approx((np.nanmin(ta), np.nanmax(ta)))
    assert r["toggle_margin_deg"] == pytest.approx(min(np.nanmin(ta), 180 - np.nanmax(ta)))
    assert r["stroke_available_mm"] == 57.0 and r["damper_bottoms_at_mm"] is None and r["all_steps_solved"] is True
    assert r["monotonic"] == bool(np.all(np.diff(mr) <= 1e-9) or np.all(np.diff(mr) >= -1e-9))
    short = rb.overtravel_check(HP, bump_mm=30.0, droop_mm=20.0, stroke_mm=10.0, n=11)
    assert short["damper_bottoms_at_mm"] == pytest.approx(float(np.interp(10.0, used, trv)))
    ext = rb.overtravel_check(HP, bump_mm=30.0, droop_mm=20.0, n=11, damper_extended_mm=m.spring_length_mm[0] + 5.0)
    assert ext["stroke_available_mm"] == pytest.approx(57.0 - 5.0)
    monkeypatch.setattr("suspension.actuation.actuation_metrics",
                        lambda *a, **k: SimpleNamespace(ok=False, reason="binds"))
    assert rb.overtravel_check(HP) == {"ok": False, "reason": "binds"}


def test_the_hand_roll_centre_agrees_with_the_solver():
    r = rb.hand_roll_centre(HP)
    ic = SuspensionKinematics(HP).static.instant_center
    assert (r["ic_y_mm"], r["ic_z_mm"]) == pytest.approx(tuple(ic), rel=1e-9)
    cp = np.asarray(HP.contact_patch, float)
    assert r["rc_height_mm"] == pytest.approx(cp[2] + (ic[1] - cp[2]) / (ic[0] - cp[1]) * (0 - cp[1]))


def test_the_shock_cases():
    v = td.Vehicle()
    st = 300.0 * 9.81 * 0.48 / 2
    fx = 0.73 * 300.0 * 9.81 * 1.8 / 2
    f = rb.shock_cases("front")
    fz = td.corner_loads(v, a_lat_g=0.0, a_long_g=1.8)["total"]["FL"]
    assert (f["braking 1.8 g"].Fx, f["braking 1.8 g"].Fz) == pytest.approx((fx, fz))
    k = f["curb strike 3 g + kick"]
    assert (k.Fx, k.Fy, k.Fz) == pytest.approx((0.3 * 3 * st, 0.3 * 3 * st, 3 * st))
    assert f["curb strike 3 g + inboard kick"].Fy == pytest.approx(-0.3 * 3 * st)
    assert (f["curb strike while braking"].Fx, f["curb strike while braking"].Fz) == pytest.approx((fx, 3 * st))
    r = rb.shock_cases("rear")
    sr = 300.0 * 9.81 * 0.52 / 2
    fzb = td.corner_loads(v, a_lat_g=0.0, a_long_g=1.8)["total"]["RL"]
    b = r["braking 1.8 g"]
    assert (b.Fx, b.Fx_wc, b.Fz) == pytest.approx((0.0, 0.27 * 300.0 * 9.81 * 1.8 / 2, fzb))
    t = r["traction 1.06 g + curb"]
    assert (t.Fx, t.Fx_wc, t.Fz) == pytest.approx((0.0, -300.0 * 9.81 * 1.06 / 2, 3 * sr))
    assert r["curb strike 3 g + inboard kick"].Fy == pytest.approx(-0.9 * sr)
    o = rb.shock_cases("rear", brake_front=0.7, kick=0.2, mass_kg=250.0, inboard_drive=False, inboard_brakes=False)
    assert (o["braking 1.8 g"].Fx, o["braking 1.8 g"].Fx_wc) == pytest.approx((0.3 * 250 * 9.81 * 1.8 / 2, 0.0))
    assert (o["traction 1.06 g + curb"].Fx, o["traction 1.06 g + curb"].Fx_wc) == pytest.approx(
        (-250 * 9.81 * 1.06 / 2, 0.0))
    assert o["curb strike 3 g + kick"].Fx == pytest.approx(0.2 * 3 * 250 * 9.81 * 0.52 / 2)
    ff = rb.shock_cases("front", brake_front=0.6, kick=0.25, mass_kg=280.0)
    assert ff["braking 1.8 g"].Fx == pytest.approx(0.6 * 280 * 9.81 * 1.8 / 2)
    assert ff["curb strike 3 g + kick"].Fz == pytest.approx(3 * 280 * 9.81 * 0.48 / 2)


def test_the_upright_brake_requirement():
    t = 0.73 * 300 * 9.81 * 1.8 / 2 * 0.228
    assert rb.upright_brake_requirement() == pytest.approx({"torque_Nm": t, "caster_stiffness_Nm_per_deg": t / 0.1,
                                                            "toe_limit_deg": 0.027})
    t = 0.6 * 250 * 9.81 * 1.5 / 2 * 0.203
    assert rb.upright_brake_requirement(0.6, 1.5, 250.0, 203.0, 0.2, 0.03) == pytest.approx(
        {"torque_Nm": t, "caster_stiffness_Nm_per_deg": t / 0.2, "toe_limit_deg": 0.03})


def test_every_shock_case_in_full():
    v, g = td.Vehicle(), 9.81
    st, fx = 300 * g * 0.48 / 2, 0.73 * 300 * g * 1.8 / 2
    fz = td.corner_loads(v, a_long_g=1.8)["total"]["FL"]
    assert rb.shock_cases("front") == {
        "braking 1.8 g": lp.WheelLoad(Fx=fx, Fz=fz), "curb strike 3 g + kick": lp.WheelLoad(Fx=0.3 * 3 * st, Fy=0.3 * 3 * st,
                                                                                            Fz=3 * st),
        "curb strike 3 g + inboard kick": lp.WheelLoad(Fx=0.3 * 3 * st, Fy=-0.3 * 3 * st, Fz=3 * st),
        "curb strike while braking": lp.WheelLoad(Fx=fx, Fz=3 * st)}
    st, fxb, fxt = 300 * g * 0.52 / 2, 0.27 * 300 * g * 1.8 / 2, -300 * g * 1.06 / 2
    fzb = td.corner_loads(v, a_long_g=1.8)["total"]["RL"]
    curb = {"curb strike 3 g + kick": lp.WheelLoad(Fx=0.3 * 3 * st, Fy=0.3 * 3 * st, Fz=3 * st),
            "curb strike 3 g + inboard kick": lp.WheelLoad(Fx=0.3 * 3 * st, Fy=-0.3 * 3 * st, Fz=3 * st)}
    assert rb.shock_cases("rear") == {"braking 1.8 g": lp.WheelLoad(Fx_wc=fxb, Fz=fzb),
                                      "traction 1.06 g + curb": lp.WheelLoad(Fx_wc=fxt, Fz=3 * st), **curb}
    assert rb.shock_cases("rear", inboard_drive=False, inboard_brakes=False) == {
        "braking 1.8 g": lp.WheelLoad(Fx=fxb, Fz=fzb), "traction 1.06 g + curb": lp.WheelLoad(Fx=fxt, Fz=3 * st), **curb}


def test_the_hand_roll_centre_with_a_raised_contact_patch():
    hp = HP.copy()
    hp.contact_patch = np.asarray(HP.contact_patch, float) + [0.0, 0.0, 12.0]
    a, b = rb.hand_roll_centre(hp), rb.hand_roll_centre(HP)
    assert (a["ic_y_mm"], a["ic_z_mm"]) == pytest.approx((b["ic_y_mm"], b["ic_z_mm"]))
    cp = np.asarray(hp.contact_patch, float)
    slope = (a["ic_z_mm"] - cp[2]) / (a["ic_y_mm"] - cp[1])
    assert a["rc_height_mm"] == pytest.approx(cp[2] - slope * cp[1])


def test_member_margins_on_declared_forces(monkeypatch):
    od, wall = 15.875, 0.889
    A = math.pi / 4 * (od ** 2 - (od - 2 * wall) ** 2); I = math.pi / 64 * (od ** 4 - (od - 2 * wall) ** 4)
    forces = {"c1": {"UF": -800.0, "LF": 0.6, "TR": 300.0, "PR": -0.4}, "c2": {"UF": 800.0, "LF": -0.6, "TR": 0.2,
                                                                             "PR": -0.4}}
    monkeypatch.setattr(lp, "solve_member_forces", lambda kin, st, ld: SimpleNamespace(
        forces=forces[ld], transverse={"UF": 50.0} if ld == "c1" else None, bending_arm_mm={"UF": 120.0, "TR": 30.0}))
    out = rb.member_margins(HP, {"c1": "c1", "c2": "c2"})
    L = {m: float(np.linalg.norm(np.asarray(getattr(HP, a), float) - np.asarray(getattr(HP, b), float)))
         for m, (a, b) in rb._MEMBER_ENDS.items()}
    pcr = {m: math.pi ** 2 * 205000.0 * I / L[m] ** 2 for m in L}
    c, r2, Z = od / 2, I / A, I / (od / 2)
    amp = 1 / (1 - 800 / pcr["UF"])
    s_uf = 800 / A * (1 + 1e-3 * L["UF"] * c / r2 * amp) + 50 * 120 / Z * amp
    assert out["UF"] == pytest.approx({"length_mm": L["UF"], "p_cr_n": pcr["UF"], "worst_n": -800.0, "worst_case": "c1",
                                       "fos_buckling": pcr["UF"] / 800, "fos_yield": min(460 / s_uf, 460 / (800 / A))})
    assert out["LF"]["worst_n"] == 0.6 and out["LF"]["worst_case"] == "c1"            # ties keep the first case
    assert out["TR"]["fos_yield"] == pytest.approx(460 / (300 / A)) and out["TR"]["worst_case"] == "c1"
    assert out["PR"]["worst_n"] == -0.4 and out["PR"]["fos_buckling"] == pytest.approx(pcr["PR"] / 0.4)
    tiny = rb.member_margins(HP, {"c2": "c2"})
    assert tiny["TR"]["fos_yield"] == pytest.approx(460 / (0.2 / A)) and tiny["LF"]["worst_n"] == -0.6


def test_overtravel_on_declared_metrics(monkeypatch):
    from suspension import actuation
    seen = {}

    def metrics(hp, travel_mm, n):
        seen.update(travel=travel_mm, n=n)
        return SimpleNamespace(ok=True, transmission_deg=[30.0, 95.0, 150.0], mr=seen["mr"], travel=[-25.0, 10.0, 50.0],
                               spring_length_mm=[220.0, 200.0, 180.0])
    monkeypatch.setattr(actuation, "actuation_metrics", metrics)
    seen["mr"] = [0.9, 0.8, 0.7]
    r = rb.overtravel_check(HP)
    used = [0.0, 0.85 * 35.0, 0.85 * 35.0 + 0.75 * 40.0]
    assert seen == {"travel": (-25.0, 50.0), "n": 31, "mr": [0.9, 0.8, 0.7]}
    assert r == {"ok": True, "transmission_min_deg": 30.0, "transmission_max_deg": 150.0, "mr_min": 0.7, "mr_max": 0.9,
                 "monotonic": True, "toggle_margin_deg": 30.0, "all_steps_solved": True,
                 "stroke_used_mm": pytest.approx(used[-1]), "stroke_available_mm": 57.0,
                 "damper_bottoms_at_mm": pytest.approx(float(np.interp(57.0, used, [-25.0, 10.0, 50.0])))}
    seen["mr"] = [0.7, 0.8, 0.8 + 5e-10]
    assert rb.overtravel_check(HP, n=9)["monotonic"] is True and seen["n"] == 9
    seen["mr"] = [0.7, 0.9, 0.8]
    assert rb.overtravel_check(HP)["monotonic"] is False
    seen["mr"] = [0.9, 0.9 + 5e-10, 0.7]
    assert rb.overtravel_check(HP)["monotonic"] is True
    seen["mr"] = [0.9, 0.8, 0.7]
    assert rb.overtravel_check(HP, stroke_mm=used[-1])["damper_bottoms_at_mm"] == pytest.approx(50.0)
    assert rb.overtravel_check(HP, stroke_mm=used[1] + 1.0)["damper_bottoms_at_mm"] is not None
    assert rb.overtravel_check(HP, stroke_mm=used[-1] + 0.01)["damper_bottoms_at_mm"] is None
    r = rb.overtravel_check(HP, damper_extended_mm=250.0, stroke_mm=50.0)
    assert r["stroke_available_mm"] == 20.0
    monkeypatch.setattr(actuation, "actuation_metrics", lambda hp, travel_mm, n: SimpleNamespace(
        ok=True, transmission_deg=[40.0, 170.0], mr=[0.8, 0.8], travel=[0.0, 1.0], spring_length_mm=[1.0, 1.0]))
    assert rb.overtravel_check(HP)["toggle_margin_deg"] == 10.0


def test_the_curves_sample_eleven_points():
    k = SuspensionKinematics(HP)
    toes = [k.solve_at_travel(t).toe for t in np.linspace(-20.0, 20.0, 11)]
    assert rb._curves(HP, travel=20.0)["bump_steer"] == pytest.approx(max(toes) - min(toes), rel=1e-9)
    assert 0.3 < rb.balance_under_tire_change()["neutral_share"] < 0.8


def test_bending_in_tension_a_missing_arm_and_an_idle_member(monkeypatch):
    od, wall = 15.875, 0.889
    A = math.pi / 4 * (od ** 2 - (od - 2 * wall) ** 2); I = math.pi / 64 * (od ** 4 - (od - 2 * wall) ** 4)
    Z = I / (od / 2)
    monkeypatch.setattr(lp, "solve_member_forces", lambda kin, st, ld: SimpleNamespace(
        forces={"UF": 500.0, "LR": 0.0, "TR": 200.0}, transverse={"UF": 40.0, "TR": 25.0},
        bending_arm_mm={"UF": 110.0}))
    out = rb.member_margins(HP, {"c": None})
    assert out["UF"]["fos_yield"] == pytest.approx(460 / (500 / A + 40 * 110 / Z))
    assert out["TR"]["fos_yield"] == pytest.approx(460 / (200 / A))                  # no arm: no moment
    assert (out["LR"]["worst_n"], out["LR"]["worst_case"]) == (0.0, "")


def test_the_damper_bottoms_inside_the_first_step(monkeypatch):
    from suspension import actuation
    monkeypatch.setattr(actuation, "actuation_metrics", lambda hp, travel_mm, n: SimpleNamespace(
        ok=True, transmission_deg=[30.0, 95.0, 150.0], mr=[0.9, 0.8, 0.7], travel=[-25.0, 10.0, 50.0],
        spring_length_mm=[220.0, 200.0, 180.0]))
    r = rb.overtravel_check(HP, stroke_mm=10.0)
    assert r["damper_bottoms_at_mm"] == pytest.approx(-25.0 + 35.0 * 10.0 / (0.85 * 35.0))
