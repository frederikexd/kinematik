"""suspension/target_derivation.py through identities its functions must satisfy with each other (the camber gain
that puts the tire at its optimum leaves zero loaded error; the anti floor at a frequency gives back that frequency as
the minimum; the stiffness share for an LLTD target gives back the target; corner loads sum to the normal load) and
the closed forms of the gas law, the series roll stiffness, aero heave and the steer warp's roll moment."""
import math
from dataclasses import replace

import numpy as np
import pytest

from suspension import target_derivation as td
from suspension.kinematics import Hardpoints, SuspensionKinematics

V = td.Vehicle()
T = td.Tire()


def test_the_vehicle_tire_and_reference_accelerations():
    assert (V.mass_kg, V.cg_height_mm, V.wheelbase_mm, V.track_mm, V.jounce_budget_mm, V.sprung_corner_front_kg,
            V.sprung_corner_rear_kg, V.roll_gradient_deg_per_g, V.ride_front_hz, V.ride_rear_hz,
            V.tire_rate_n_per_mm, V.aero_heave_mm) == (300.0, 280.0, 1630.0, 1210.0, 17.5, 60.1, 66.1, 0.78, 2.6,
                                                       3.0, None, 0.0)
    assert (T.camber_opt_deg, T.peak_mu, T.camber_sensitivity_pct_per_deg) == (-1.83, 1.55, 0.36)
    assert (td.MU_REF, td.LAT_REF_G, td.COMB_REF_G, td.STRAIGHT_REF_G, td.G0) == (1.55, 1.5, 1.06, 1.06, 9.81)
    assert td.design_accelerations(T) == pytest.approx({"lateral": 1.5, "combined": 1.06, "straight": 1.06})
    assert td.design_accelerations(replace(T, peak_mu=1.705)) == pytest.approx(
        {"lateral": 1.65, "combined": 1.166, "straight": 1.166})


def test_roll_and_the_tire_deflection_it_adds():
    phi, z = td.roll(V, 1.5)
    assert phi == pytest.approx(1.17) and z == pytest.approx(605.0 * math.tan(math.radians(1.17)))
    assert td.tire_roll_deg(V, 1.5) == 0.0 and td.roll_to_road(V, 1.5) == (phi, z)
    soft = replace(V, tire_rate_n_per_mm=130.0)
    M = 300 * 9.81 * 1.5 * 0.28
    k = 130e3 * 1.21 ** 2
    assert td.tire_roll_deg(soft, 1.5) == pytest.approx(math.degrees(M / k))
    assert td.roll_to_road(soft, 1.5) == pytest.approx((phi + math.degrees(M / k), z))


def test_series_roll_stiffness():
    r = td.series_roll_stiffness(450.0, 400.0, 130.0, 1210.0)
    kt = 130e3 * 1.21 ** 2 / 2 * math.pi / 180
    f, rr = 450 * kt / (450 + kt), 400 * kt / (400 + kt)
    assert r == pytest.approx({"tire_axle_Nm_per_deg": kt, "front": f, "rear": rr, "total": f + rr,
                               "share_rigid": 450 / 850, "share_series": f / (f + rr)})


def test_aero_heave():
    assert td.aero_heave_mm(3.0, 60.0, 30.0) == pytest.approx(0.5 * 1.225 * 3 * (60 / 3.6) ** 2 * 0.5 / 2 / 30)
    assert td.aero_heave_mm(3.0, 60.0, 30.0, axle_share=0.4, rho=1.1) == pytest.approx(
        0.5 * 1.1 * 3 * (60 / 3.6) ** 2 * 0.4 / 2 / 30)


def test_the_camber_target_functions_agree_with_each_other():
    for veh in (V, replace(V, tire_rate_n_per_mm=120.0)):
        g = td.camber_gain_needed(veh, T, -2.5)
        assert td.loaded_camber_error(veh, T, -2.5, g) == pytest.approx(0.0, abs=1e-12)
        assert td.static_camber_needed(veh, T, g) == pytest.approx(-2.5)
        phi, z = td.roll_to_road(veh, 1.5)
        assert g == pytest.approx((T.camber_opt_deg - phi + 2.5) / z)
        assert td.loaded_camber_error(veh, T, -2.0, g) == pytest.approx(0.5)


def test_the_ride_floor_and_the_anti_floor_are_inverses():
    for axle in ("front", "rear"):
        f = td.f_min(V, 30.0, 1.06, 1.06, axle)
        m_s = V.sprung_corner_front_kg if axle == "front" else V.sprung_corner_rear_kg
        dW = 300 * 9.81 * 1.06 * 280 / (2 * 1630)
        room = 17.5 - td.roll(V, 1.06)[1]
        assert f == pytest.approx(math.sqrt(0.7 * dW / (room / 1000) / m_s) / (2 * math.pi))
        assert td.anti_floor_pct(V, f, 1.06, 1.06, axle) == pytest.approx(30.0)
    tight = replace(V, aero_heave_mm=17.5)
    assert td.f_min(tight, 0.0, 1.0, 1.0, "front") == math.inf
    assert td.anti_floor_pct(tight, 3.0, 1.0, 1.0, "rear") == math.inf
    some = replace(V, aero_heave_mm=5.0)
    assert td.f_min(some, 0.0, 1.0, 0.0, "front") == pytest.approx(
        math.sqrt(300 * 9.81 * 280 / 3260 / 0.0125 / 60.1) / (2 * math.pi))


def test_the_targets_for_one_tire():
    t = td.targets()
    a = td.design_accelerations(T)
    assert t == pytest.approx({"camber_gain_front_deg_per_mm": td.camber_gain_needed(V, T, -2.5),
                               "camber_gain_rear_deg_per_mm": td.camber_gain_needed(V, T, -2.6),
                               "anti_dive_floor_pct": td.anti_floor_pct(V, 2.6, a["combined"], a["combined"], "front"),
                               "anti_squat_floor_pct": td.anti_floor_pct(V, 3.0, a["combined"], a["combined"], "rear")})
    t2 = td.targets(V, T, static_front_deg=-2.0, static_rear_deg=-1.5)
    assert t2["camber_gain_front_deg_per_mm"] == pytest.approx(td.camber_gain_needed(V, T, -2.0))
    assert t2["camber_gain_rear_deg_per_mm"] == pytest.approx(td.camber_gain_needed(V, T, -1.5))


def test_the_tire_sensitivity_table():
    corners = [td.Corner("F", -2.5, 0.02, 25.0, "front"), td.Corner("R", -2.6, 0.018, 20.0, "rear")]
    rows = td.tire_sensitivity(corners)
    assert [r["case"] for r in rows] == ["reference", "camber optimum x0.75", "peak mu x0.75",
                                         "camber optimum x1.25", "peak mu x1.25", "camber sensitivity x0.5",
                                         "camber sensitivity x5"]
    assert rows[1]["tire"].camber_opt_deg == pytest.approx(-1.83 * 0.75)
    assert rows[4]["tire"].peak_mu == pytest.approx(1.55 * 1.25)
    assert rows[6]["tire"].camber_sensitivity_pct_per_deg == pytest.approx(0.36 * 5)
    for r in rows:
        t = r["tire"]
        for c in corners:
            err = td.loaded_camber_error(V, t, c.static_camber_deg, c.camber_gain_deg_per_mm)
            floor = r["anti_dive_floor_pct"] if c.axle == "front" else r["anti_squat_floor_pct"]
            assert r[c.name] == pytest.approx({"camber_error_deg": err,
                                               "static_to_restore_deg": td.static_camber_needed(
                                                   V, t, c.camber_gain_deg_per_mm),
                                               "grip_cost_pct": abs(err) * t.camber_sensitivity_pct_per_deg,
                                               "anti_floor_pct": floor, "anti_margin_pct": c.anti_pct - floor})
        assert r["anti_dive_floor_pct"] == pytest.approx(td.targets(V, t)["anti_dive_floor_pct"])
    assert [r["case"] for r in td.tire_sensitivity([], span=0.1)][1] == "camber optimum x0.90"


def test_the_derived_ride_frequency_and_roll_gradient():
    f = td.ride_frequency_derived(V, T, 25.0, "front")
    a = td.design_accelerations(replace(T, peak_mu=1.55 * 1.1))
    assert f == pytest.approx(td.f_min(V, 25.0, a["combined"], a["combined"], "front"))
    f0 = td.ride_frequency_derived(V, T, 25.0, "rear", mu_allowance=0.0)
    assert f0 == pytest.approx(td.f_min(V, 25.0, 1.06, 1.06, "rear"))
    g = td.roll_gradient_ceiling(V, T, 25.0, "front", 2.6)
    ok = lambda x: td.f_min(replace(V, roll_gradient_deg_per_g=x), 25.0, 1.06, 1.06, "front") < 2.6
    assert 0.05 < g < 3.0 and ok(g) and not ok(g + 1e-9)
    gh = td.roll_gradient_ceiling(V, T, 25.0, "front", 2.6, mu_allowance=0.1)
    assert gh < g
    assert td.roll_gradient_for_friction(V, T, 25.0, "front", 2.6, 1.55 * 1.1) == pytest.approx(gh)
    # a stiff enough spring never binds: the ceiling is where roll alone uses the whole jounce budget
    assert td.roll_gradient_ceiling(V, T, 25.0, "front", 50.0) == pytest.approx(
        math.degrees(math.atan(17.5 / 605.0)) / 1.06, rel=5e-3)
    assert td.roll_gradient_ceiling(V, T, 25.0, "front", 0.1) == pytest.approx(0.05, abs=1e-9)


def test_springs_only_roll_gradient():
    r = td.springs_only_roll_gradient(V, 60.0, 40.0)
    kf = (2 * math.pi * 2.6) ** 2 * 60.1 / 1000
    kr = (2 * math.pi * 3.0) ** 2 * 66.1 / 1000
    c = 1210 ** 2 / 2 * math.pi / 180 / 1000
    h = 60.0 + (40.0 - 60.0) * 0.52
    M = 300 * 9.81 * (280 - h) / 1000
    assert r == pytest.approx({"front_Nm_per_deg": kf * c, "rear_Nm_per_deg": kr * c, "moment_Nm_per_g": M,
                               "gradient_deg_per_g": M / (kf * c + kr * c)})
    r2 = td.springs_only_roll_gradient(V, 60.0, 40.0, weight_front=0.5)
    assert r2["moment_Nm_per_g"] == pytest.approx(300 * 9.81 * (280 - 50.0) / 1000)


def test_the_share_per_mm_and_the_lltd_pair():
    assert td.share_per_mm_front_rc(V) == pytest.approx((120.2 - 0.522 * 0.48 * 252.4) / (300 * 280))
    assert td.share_per_mm_front_rc(V, 0.5, 0.5) == pytest.approx((120.2 - 0.25 * 252.4) / (300 * 280))
    s = td.front_lltd_share(58.5, 39.5, 0.52)
    h = 58.5 + (39.5 - 58.5) * 0.52
    assert s == pytest.approx((0.48 * 58.5 + 0.52 * (280 - h)) / 280)
    assert td.stiffness_share_for_lltd(s, 58.5, 39.5) == pytest.approx(0.52)
    s2 = td.front_lltd_share(50.0, 30.0, 0.6, cg_height_mm=300.0, weight_front=0.45)
    assert td.stiffness_share_for_lltd(s2, 50.0, 30.0, cg_height_mm=300.0, weight_front=0.45) == pytest.approx(0.6)
    h2 = 50.0 + (30.0 - 50.0) * 0.55
    assert s2 == pytest.approx((0.45 * 50.0 + 0.6 * (300 - h2)) / 300)
    # the linearised share per mm matches a finite difference of the exact one at its stiffness share
    d = (td.front_lltd_share(59.5, 39.5, 0.522) - td.front_lltd_share(57.5, 39.5, 0.522)) / 2
    assert d == pytest.approx((0.48 - 0.522 * 0.48) / 280)


def test_the_synthetic_mu_and_the_neutral_share():
    assert td.mu_of_load(550.0) == pytest.approx(1.66) and td.mu_of_load(1650.0) == pytest.approx(1.44)
    assert "uncalibrated" in td.TIRE_PROVENANCE
    n = td.neutral_front_share(V)
    assert td.axle_capacity_ratio(V, n) == pytest.approx(1.0, abs=1e-12)
    lt = 300 * 9.81 * 1.5 * 280 / 1210
    wf, wr = 300 * 9.81 * 0.48 / 2, 300 * 9.81 * 0.52 / 2
    cf = sum(td.mu_of_load(f) * f for f in (wf + 0.5 * lt, wf - 0.5 * lt)) / (2 * wf)
    cr = sum(td.mu_of_load(f) * f for f in (wr + 0.5 * lt, wr - 0.5 * lt)) / (2 * wr)
    assert td.axle_capacity_ratio(V, 0.5) == pytest.approx(cf / cr)
    assert td.axle_capacity_ratio(V, 0.5, 1.2, 0.5) != td.axle_capacity_ratio(V, 0.5)
    n12 = td.neutral_front_share(V, 1.2)
    assert td.axle_capacity_ratio(V, n12, 1.2) == pytest.approx(1.0, abs=1e-12)


def test_hot_pressure_and_the_rate_it_moves():
    assert td.hot_pressure_kpa(83.0, 20.0, 60.0) == pytest.approx((83 + 101.325) * 333.15 / 293.15 - 101.325)
    assert td.hot_pressure_kpa(83.0, 20.0, 60.0, p_atm_kpa=95.0) == pytest.approx((83 + 95) * 333.15 / 293.15 - 95)
    assert td.tire_rate_at_pressure(120.0, 83.0, 100.0) == pytest.approx(120 * (0.3 + 0.7 * 100 / 83))
    assert td.tire_rate_at_pressure(120.0, 83.0, 100.0, carcass_share=0.25) == pytest.approx(
        120 * (0.25 + 0.75 * 100 / 83))


def test_corner_loads_term_by_term():
    r = td.corner_loads(V, a_lat_g=1.2, a_long_g=0.8, bank_deg=5.0, grade_deg=-3.0, downforce_n=600.0,
                        aero_front_share=0.45, front_lltd=0.55, steer_warp_n=40.0, weight_front=0.47)
    m, g = 300.0, 9.81
    b, gr = math.radians(5.0), math.radians(-3.0)
    normal = m * (g * math.cos(b) * math.cos(gr) + 1.2 * g * math.sin(b))
    lat = m * (1.2 * g * math.cos(b) - g * math.sin(b))
    lon = m * 0.8 * g - m * g * math.sin(gr)
    dlon, dlat = lon * 0.28 / 1.63 / 2, lat * 0.28 / 1.21
    assert r["longitudinal"] == pytest.approx({"FL": dlon, "FR": dlon, "RL": -dlon, "RR": -dlon})
    assert r["lateral"] == pytest.approx({"FL": -0.55 * dlat, "FR": 0.55 * dlat, "RL": -0.45 * dlat,
                                          "RR": 0.45 * dlat})
    assert r["downforce"] == pytest.approx({"FL": 135.0, "FR": 135.0, "RL": 165.0, "RR": 165.0})
    assert r["steer_warp"] == {"FL": 40.0, "FR": -40.0, "RL": -40.0, "RR": 40.0}
    st = {"FL": normal * 0.235, "FR": normal * 0.235, "RL": normal * 0.265, "RR": normal * 0.265}
    flat = {"FL": m * g * 0.235, "FR": m * g * 0.235, "RL": m * g * 0.265, "RR": m * g * 0.265}
    assert r["static_flat"] == pytest.approx(flat)
    assert r["bank_grade"] == pytest.approx({c: st[c] - flat[c] for c in st})
    for c in st:
        assert r["total"][c] == pytest.approx(st[c] + r["longitudinal"][c] + r["lateral"][c] + r["downforce"][c]
                                              + r["steer_warp"][c])
    assert r["check_sum_N"] == pytest.approx(normal + 600.0) and r["normal_N"] == pytest.approx(normal + 600.0)
    d = td.corner_loads(V)
    assert d["total"] == pytest.approx({"FL": 706.32, "FR": 706.32, "RL": 765.18, "RR": 765.18})
    lt = td.corner_loads(V, a_lat_g=1.0)["lateral"]
    assert lt["FR"] == pytest.approx(0.53 * 300 * 9.81 * 0.28 / 1.21)


def test_the_steer_warp_moment():
    for hp in (Hardpoints.default(), replace(Hardpoints.default(), pushrod_attach="upright",
                                             pushrod_outer=np.array([-5.0, 520.0, 150.0]))):
        r = td.steer_warp(hp, 12.0, 450.0, 400.0, 1210.0, 1180.0)
        z = lambda rack: SuspensionKinematics(hp, pickup_deltas={"tie_rod_inner": np.array([0.0, rack, 0.0])}
                                              ).solve_at_travel(0.0).contact_patch[2]
        gR, gL = z(12.0) - z(0.0), z(-12.0) - z(0.0)
        assert (r["patch_dz_right_mm"], r["patch_dz_left_mm"]) == pytest.approx((gR, gL))
        assert r["dz_right_mm"] == pytest.approx(gR + r["spring_dz_right_mm"])
        assert r["dz_left_mm"] == pytest.approx(gL + r["spring_dz_left_mm"])
        phi = (r["dz_right_mm"] - r["dz_left_mm"]) / 1210.0
        kf, kr = 450 * 180 / math.pi, 400 * 180 / math.pi
        M = phi * kf * kr / (kf + kr)
        assert r["phi_deg"] == pytest.approx(math.degrees(phi)) and r["warp_moment_Nm"] == pytest.approx(M)
        assert r["left_steer_N"] == pytest.approx({"FL": M / 1.21, "FR": -M / 1.21, "RL": -M / 1.18, "RR": M / 1.18})
        assert (r["front_pair_N"], r["rear_pair_N"]) == pytest.approx((M / 1.21, M / 1.18))
        assert r["lift_common_mm"] == pytest.approx(0.5 * (r["dz_right_mm"] + r["dz_left_mm"]))
    k = SuspensionKinematics(r_hp := replace(Hardpoints.default(), pushrod_attach="upright",
                                             pushrod_outer=np.array([-5.0, 520.0, 150.0])))
    st0 = k.solve_at_travel(0.0)
    L0 = k.spring_length_at(st0, seed=0.0)[0]
    kk = SuspensionKinematics(r_hp, pickup_deltas={"tie_rod_inner": np.array([0.0, 12.0, 0.0])})
    L = kk.spring_length_at(kk.solve_at_travel(0.0), seed=0.0)[0]
    mr0 = float(np.interp(0.0, *k.motion_ratio_curve(-1.0, 1.0, 3)))
    assert td.steer_warp(r_hp, 12.0, 450.0, 400.0, 1210.0, 1180.0)["spring_dz_right_mm"] == pytest.approx(
        -(L0 - L) / mr0)
    bare = Hardpoints.default()
    bare.pushrod_outer = None
    assert td.steer_warp(bare, 12.0, 450.0, 400.0, 1210.0, 1180.0)["spring_dz_left_mm"] == 0.0


def test_no_travel_room_and_a_sliver_of_it():
    v = td.Vehicle()
    none = replace(v, jounce_budget_mm=v.aero_heave_mm)
    assert td.f_min(none, 30.0, 1.0, 0.0, "front") == math.inf and td.anti_floor_pct(none, 3.0, 1.0, 0.0, "rear") == math.inf
    thin = replace(v, jounce_budget_mm=v.aero_heave_mm + 0.5)
    dW = v.mass_kg * td.G0 * 1.0 * v.cg_height_mm / (2.0 * v.wheelbase_mm)
    k = (1 - 0.3) * dW / 0.0005
    assert td.f_min(thin, 30.0, 1.0, 0.0, "front") == pytest.approx(math.sqrt(k / v.sprung_corner_front_kg) / (2 * math.pi))
    kr = (2 * math.pi * 3.0) ** 2 * v.sprung_corner_rear_kg
    assert td.anti_floor_pct(thin, 3.0, 1.0, 0.0, "rear") == pytest.approx(100 * (1 - kr * 0.0005 / dW))
    with pytest.raises(KeyError):
        td.f_min(v, 30.0, 1.0, 0.0, "Rear")


def test_downforce_splits_evenly_by_default():
    v = td.Vehicle()
    assert td.corner_loads(v, downforce_n=600.0) == td.corner_loads(v, downforce_n=600.0, aero_front_share=0.5)
    assert td.corner_loads(v, downforce_n=600.0) != td.corner_loads(v, downforce_n=600.0, aero_front_share=0.6)
