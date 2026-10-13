# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Every number the paper and supplement quote from this round, recomputed.

If a change to the code moves one of these, the paper must change with it.
"""
import math
from pathlib import Path

import pytest

from suspension import actuation as act, contact_patch as cp, damper_synthesis as ds
from suspension import genesis_analysis as ga, genesis_repro as gr, robustness as rb
from suspension.genesis_repro import GenesisManifest, hp_from_dict
from suspension.halfshaft import goodman_min_diameter

MAN = Path(__file__).resolve().parents[1] / "manifests"
TABLE_S5_1 = {"front_final": "5220385cbcd2", "front_steering_walls": "bbc4d8297765", "front_wheel_lock": "9e9257a82fe5",
              "rear_final": "aa9928e583fe", "rear_antilift": "24bc81a7538c", "rear_v11": "71b6c9fe45bd",
              "front_v11_25deg": "ae5cef6d5757", "front_v11_29deg": "2673492a63c5",
              "front_v11_29deg_walls": "4f088b0e4609",
              "front_v11_25deg_walls": "5e0ab207749b",
              "front_v11_hairpin_walls": "7f1436d93a63",
              "front_v11_curves_only": "c320eac46d0f",
              "rear_v11_walls": "f567eaf8bce3",
              "front_v11_29deg_b60_brake": "113a757c3ce0",
              "front_v11_29deg_b73_6001": "115b45a62748",
              "front_v11_29deg_b73_tube035": "af71aa072573",
              "front_v11_final": "2e7072ce2268",
              "rear_v11_final": "8818c4b890d1",
              "front_v11_from_85mm": "3bcacea5c507",
              "front_v11_from_delivered": "3079657a19c5", "front_v11_joints50": "b6399630caba"}


def _manifest(name):
    return GenesisManifest.from_json((MAN / f"{name}.recorded.genesis.json").read_text())


def _winner(name):
    return hp_from_dict(_manifest(name).recorded["winner_hardpoints"])


@pytest.mark.parametrize("name,prefix", TABLE_S5_1.items())
def test_published_manifests_carry_the_table_s5_1_hashes(name, prefix):
    assert _manifest(name).inputs_sha256.startswith(prefix)


# the delivered and released corners with their pushrods on the lower wishbone, on the corrected statics (S14)
@pytest.mark.parametrize("name,s0,c30,c60", [("front_steering_walls", 0.60, 7.9, 6.9), ("rear_final", 0.76, 6.2, 5.8)])
def test_rocker_bearings_on_the_synthesised_actuation(name, s0, c30, c60):
    res = act.synthesize_actuation(_winner(name)); hp = getattr(res, "hp", None) or getattr(res, "hardpoints", None)
    cases = gr.vehicle_load_cases()
    assert min(r["s0"] for r in act.rocker_bearing_loads(hp, cases)) == pytest.approx(s0, abs=0.01)
    rows = act.required_rating_vs_spacing(hp, cases, [30.0, 60.0])
    assert rows[0]["required_C0_N"] / 1000 == pytest.approx(c30, abs=0.05)
    assert rows[1]["required_C0_N"] / 1000 == pytest.approx(c60, abs=0.05)


def test_front_lower_fore_bracket_closure():
    # the delivered front's lower-front leg on the corrected statics: 8.66 kN in steady 1.5 g braking, 10.51 kN in the
    # curb strike while braking at 73% bias
    from suspension import genesis_walls as gw
    from suspension.kinematics import SuspensionKinematics
    from suspension.loadpath import solve_member_forces
    hp = _winner("front_steering_walls"); kin = SuspensionKinematics(hp); st = kin.solve_at_travel(0.0)
    steady = max(solve_member_forces(kin, st, gw._wheel_load(gw._load_dict(c))).forces["LF"]
                 for c in gr.vehicle_load_cases(axle="front", brake_bias_front=0.73))
    stop = gw.LinkStructure.from_shock_cases("front", 0.73).margins(hp)["LF"]["worst_n"]
    assert (round(steady / 1000, 1), round(stop / 1000, 2)) == (8.7, 10.51)
    assert ga.bracket_fos(steady, 27.9, 30, 5)["fos"] == pytest.approx(0.37, abs=0.005)      # steady case, as built
    assert ga.bracket_fos(stop, 27.9, 30, 5)["fos"] == pytest.approx(0.31, abs=0.005)        # curb strike while braking
    assert ga.bracket_fos(stop, 13.0, 50, 6.35)["fos"] == pytest.approx(1.77, abs=0.005)     # moved to its node


@pytest.mark.parametrize("kw,d", [({}, 23.2), ({"ke": 0.897}, 24.0), ({"ke": 0.897, "kf": 1.3}, 26.1), ({"ke": 0.897, "kf": 1.5}, 27.4)])
def test_halfshaft_diameters(kw, d):
    assert goodman_min_diameter(356.0, -201.0, kc=1.0, **kw)["d_min_mm"] == pytest.approx(d, abs=0.06)


def test_upright_brake_torque_requirement():
    r = rb.upright_brake_requirement()
    assert r["torque_Nm"] == pytest.approx(441, abs=1) and r["caster_stiffness_Nm_per_deg"] == pytest.approx(4408, abs=5)


def test_front_lower_legs_at_73_percent_bias():
    # the delivered front, pushrod on the lower wishbone: on the corrected statics the pushrod bends the bare legs, so
    # yield, not buckling, governs, and upsizing the walls alone does not recover them
    m = rb.member_margins(_winner("front_steering_walls"), rb.shock_cases("front", 0.73), sy_mpa=435.0)
    assert m["LR"]["fos_buckling"] == pytest.approx(1.22, abs=0.01) and m["LR"]["fos_yield"] == pytest.approx(0.07, abs=0.01)
    assert m["LF"]["fos_yield"] == pytest.approx(0.32, abs=0.01)
    lr = rb.member_margins(_winner("front_steering_walls"), rb.shock_cases("front", 0.73), wall_mm=1.651, sy_mpa=435.0)["LR"]
    lf = rb.member_margins(_winner("front_steering_walls"), rb.shock_cases("front", 0.73), wall_mm=1.245, sy_mpa=435.0)["LF"]
    assert lr["fos_buckling"] == pytest.approx(1.95, abs=0.01) and lr["fos_yield"] == pytest.approx(0.27, abs=0.01)
    assert lf["fos_yield"] == pytest.approx(0.42, abs=0.01)


def _qc(f_hz, ms=60.1, mu=11.9, zeta=0.3, kappa=0.0):
    k = ms * (2 * math.pi * f_hz) ** 2
    return cp.QuarterCar(ms, mu, k, 2 * zeta * math.sqrt(k * ms), 120e3, kappa=kappa)


def test_dynamics_at_the_2_6_hz_set_up():
    kappa = cp.kappa_from_anti(57.3, 280, 1630, 0.6, 1.5)
    assert cp.load_variation(_qc(2.6), 15.0)["ratio"] == pytest.approx(0.244, abs=0.0006)
    assert cp.load_variation(_qc(2.6, kappa=kappa), 15.0)["ratio"] == pytest.approx(0.260, abs=0.0006)
    assert cp.load_variation(_qc(2.6, mu=9.0), 15.0)["ratio"] == pytest.approx(0.229, abs=0.0006)
    assert cp.load_variation(_qc(2.6, mu=15.0), 15.0)["ratio"] == pytest.approx(0.256, abs=0.0006)
    assert ds.grip_optimal_zeta(_qc(2.6)) == pytest.approx(0.49, abs=0.005)
    cf = 2 * 0.65 * math.sqrt(60.1 * (2 * math.pi * 2.6) ** 2 * 60.1); cr = 2 * 0.65 * math.sqrt(66.1 * (2 * math.pi * 3.0) ** 2 * 66.1)
    assert ds.transient_front_share(cf, cr, 1210, 1210) == pytest.approx(0.441, abs=0.0006)       # declared 1210 mm track, both axles
    assert ds.front_rate_for_share(0.53, cr, 1210, 1210) / cf - 1 == pytest.approx(0.43, abs=0.005)


def test_steering_lag_is_computed_on_the_tire_derived_car():
    from dataclasses import replace
    from suspension import steering_response as sr
    car = sr.tire_derived_single_track()
    assert car.cf_n_per_rad == pytest.approx(41512, rel=0.002) and car.cr_n_per_rad == pytest.approx(43953, rel=0.002)
    b = sr.lag_budget(car)
    assert b["total_ms"] == pytest.approx(58.3, abs=0.1) and b["relaxation_ms"] == pytest.approx(46.6, abs=0.1)
    assert b["chassis_ms"] == pytest.approx(11.7, abs=0.1)
    ym = sr.yaw_mode(car, (15.0,))[0]
    assert ym.get("fn_hz", ym.get("f_hz")) == pytest.approx(3.0, abs=0.02) and ym["zeta"] == pytest.approx(1.0, abs=0.02)
    lo = sr.lag_budget(replace(car, sigma_m=0.376))["total_ms"]; hi = sr.lag_budget(replace(car, sigma_m=0.564))["total_ms"]
    assert lo == pytest.approx(46.6, abs=0.1) and hi == pytest.approx(71.6, abs=0.1)
    assert sr.lag_budget(sr.SingleTrack())["total_ms"] == pytest.approx(63.1, abs=0.1)      # placeholder default, not the paper's car


def test_joint_swing_over_travel_and_lock():
    from suspension import inverse_genesis as ig
    r = ig.joint_swing(_winner("front_steering_walls"), travel_mm=(-25.0, 25.0), rack_travel_mm=25.4)
    assert r["tie_rod.outer"] == pytest.approx(22.8, abs=0.05)
    for j in ("upper_front", "upper_rear", "lower_front", "lower_rear"):
        assert 21.0 < r[f"{j}.outer"] < 21.8 and r[f"{j}.inner"] < 4.4


def test_camber_shim_couples_into_toe():
    import numpy as np
    from suspension.kinematics import SuspensionKinematics
    hp = _winner("front_steering_walls"); s0 = SuspensionKinematics(hp).solve_at_travel(0.0)
    d = {"upper_front_inner": np.array([0, 0.5, 0]), "upper_rear_inner": np.array([0, 0.5, 0])}
    s = SuspensionKinematics(hp, pickup_deltas=d).solve_at_travel(0.0)
    assert s.camber - s0.camber == pytest.approx(0.163, abs=0.001) and s.toe - s0.toe == pytest.approx(0.088, abs=0.001)


def test_compliance_matrix_and_roll_centre_shifts():
    import numpy as np
    from suspension.kinematics import SuspensionKinematics
    from suspension.ghost_topology import _rc_height_mm
    hp = _winner("front_steering_walls"); k0 = SuspensionKinematics(hp); s0 = k0.solve_at_travel(0.0)
    exp = {"lower_front_inner": ("lower_outer", (1.682, 0.212, 0.098), 0.778), "tie_rod_inner": ("tie_rod_outer", (-0.433, -0.030, 0.024), -0.825)}
    for a, (b, dwc, dtoe) in exp.items():
        pa, pb = np.asarray(getattr(hp, a), float), np.asarray(getattr(hp, b), float); u = (pb - pa) / np.linalg.norm(pb - pa)
        s = SuspensionKinematics(hp, pickup_deltas={a: u}).solve_at_travel(0.0)
        assert np.asarray(s.wheel_center) - np.asarray(s0.wheel_center) == pytest.approx(np.array(dwc), abs=0.002)
        assert s.toe - s0.toe == pytest.approx(dtoe, abs=0.002)
    rc0 = _rc_height_mm(s0, 1210.0)
    play = {p: np.array([0, 0.025, 0]) for p in ("upper_front_inner", "upper_rear_inner", "lower_front_inner", "lower_rear_inner")}
    assert abs(_rc_height_mm(SuspensionKinematics(hp, pickup_deltas=play).solve_at_travel(0.0), 1210.0) - rc0) < 0.005
    tab = abs(_rc_height_mm(SuspensionKinematics(hp, pickup_deltas={"lower_front_inner": np.array([0, 0.17, 0])}).solve_at_travel(0.0), 1210.0) - rc0)
    assert tab == pytest.approx(0.145, abs=0.003)


def test_peak_mu_thresholds_and_floors():
    from dataclasses import replace
    from suspension import target_derivation as td, tire_field as tf
    v = td.Vehicle(); base = td.Tire(); v26 = replace(v, ride_front_hz=2.6)
    assert tf.mu_threshold_for("anti_squat_floor_pct", 34.28) == pytest.approx(1.671, abs=0.002)
    assert tf.mu_threshold_for("anti_squat_floor_pct", 43.3) == pytest.approx(1.785, abs=0.002)
    assert tf.mu_threshold_for("anti_dive_floor_pct", 57.25, veh=v26) == pytest.approx(1.709, abs=0.002)
    up = td.targets(v, replace(base, peak_mu=base.peak_mu * 1.1)); dn = td.targets(v, replace(base, peak_mu=base.peak_mu * 0.9))
    assert up["anti_squat_floor_pct"] == pytest.approx(37.1, abs=0.1) and dn["anti_squat_floor_pct"] == pytest.approx(6.1, abs=0.1)
    assert td.targets(v26, replace(base, peak_mu=base.peak_mu * 1.1))["anti_dive_floor_pct"] == pytest.approx(57.0, abs=0.1)
    assert td.targets(v, replace(base, peak_mu=1.85))["anti_squat_floor_pct"] == pytest.approx(48.0, abs=0.1)


def _axle_torque(trail_mm):
    import numpy as np
    from suspension import steering_feel as sf
    from suspension.tiremodel import default_tire
    t = default_tire(); co = sf.aligning_curve(t, 1248.0, trail_mm); ci = sf.aligning_curve(t, 165.0, trail_mm)
    a = np.asarray(co["alpha_deg"]); M = np.asarray(co["M_Nm"]) + np.interp(a, ci["alpha_deg"], ci["M_Nm"])
    return float(M.max()), float(np.interp(co["alpha_peak_deg"], a, M))


def test_steering_feel_on_the_re_run_front():
    import numpy as np
    from suspension import steering_feel as sf
    from suspension.kinematics import SuspensionKinematics
    hp = _winner("front_steering_walls"); s = SuspensionKinematics(hp).solve_at_travel(0.0)
    U, L, cp = np.asarray(hp.upper_outer, float), np.asarray(hp.lower_outer, float), np.asarray(s.contact_patch, float)
    k = U - L; g = L + (-L[2] / k[2]) * k; trail = cp[0] - g[0]
    assert trail == pytest.approx(12.67, abs=0.01) and cp[1] - g[1] == pytest.approx(6.0, abs=0.01)
    pk, lim = _axle_torque(trail)
    assert pk == pytest.approx(40.5, abs=0.06) and lim == pytest.approx(27.4, abs=0.06) and 1 - lim / pk == pytest.approx(0.32, abs=0.005)
    assert (pk - lim) / 5.91 == pytest.approx(2.2, abs=0.05) and pk / 5.91 == pytest.approx(6.85, abs=0.05)
    t1 = sf.mechanical_trail_mm(1.0) + (trail - sf.mechanical_trail_mm(s.caster))      # same geometry, +1.0 deg caster
    pk1, lim1 = _axle_torque(t1)
    assert pk1 == pytest.approx(20.3, abs=0.1) and lim1 == pytest.approx(4.3, abs=0.05)


def test_gust_roll_and_torque_split_values():
    import numpy as np
    from suspension import steering_response as sr, target_derivation as td, torque_distribution as tdist
    g = sr.lateral_accel_split(sr.SingleTrack(), delta_deg=0.0, gust_n=300.0)
    assert abs(np.asarray(g["ay"])[1]) / 9.81 == pytest.approx(0.10, abs=0.005)            # F/m at onset, zero yaw rate
    roll, bump = td.roll(td.Vehicle(), 1.5)
    assert roll == pytest.approx(1.17, abs=0.005) and bump == pytest.approx(12.4, abs=0.05)
    for R in (4.5, 9.125, 15.0, 30.0):
        lims = [tdist.steady_limit(R, o)["ay_g"] for o in ("open", "lsd2", "lsd3", "tv")]
        assert max(lims) - min(lims) <= 0.0075


TABLE_7 = {"table7_control": ("a5c4712165ac", "RESILIENT", 1.0), "table7_as40": ("2ac627c01835", "TEMPERED", 0.86),
           "table7_as40_60": ("8cbc5d8e9368", None, None), "table7_mig10": ("7ca0044f0866", None, None),
           "table7_as40_mig10": ("e6684459d9c8", "RESILIENT", 0.983)}


@pytest.mark.parametrize("name,expected", TABLE_7.items())
def test_table_7_manifests_are_published_with_their_outcomes(name, expected):
    m = _manifest(name); prefix, verdict, yld = expected
    assert m.inputs_sha256.startswith(prefix)
    assert m.recorded["winner_verdict"] == verdict
    assert (m.recorded["winner_yield"] is None) if yld is None else m.recorded["winner_yield"] == pytest.approx(yld, abs=1e-3)


def test_roll_centre_share_resolution_matches_lltd_derivative():
    from suspension import target_derivation as td
    v = td.Vehicle(); h = 1e-3
    assert td.share_per_mm_front_rc(v) == pytest.approx(0.000678, abs=2e-6)      # 0.068 points/mm, ±18 mm = ±1.2
    total = (td.front_lltd_share(58.5 + h, 39.5, 0.522) - td.front_lltd_share(58.5, 39.5, 0.522)) / h
    assert total == pytest.approx(0.48 * (1 - 0.522) / 280.0, rel=1e-6)          # same b/L roll-axis derivative


def test_hand_derived_pitch_chain():
    import math
    dF = 300 * 9.81 * 1.5 * 0.280 / (2 * 1.630)
    kf = 60.1 * (2 * math.pi * 2.6) ** 2 / 1000; kr = 66.1 * (2 * math.pi * 3.0) ** 2 / 1000
    dive = dF * (1 - 0.573) / kf; rise = dF * (1 - 0.137) / kr
    assert dive == pytest.approx(10.1, abs=0.05) and rise == pytest.approx(13.9, abs=0.05)
    assert math.degrees(math.atan((dive + rise) / 1630)) == pytest.approx(0.84, abs=0.005)


def test_footprint_lengths_and_the_trail_limit():
    # [29] Tables 3-4: 145 x 71 mm at 525 N and 175 x 78 mm at 881 N (width x length). The lengths fit inside the
    # chord the measured 100.2 N/mm vertical rate allows on a 228.6 mm radius; the widths could not be lengths.
    import numpy as np
    from suspension import steering_feel as sf
    from suspension.tiremodel import default_tire
    for fz, w, l in ((525.0, 145.0, 71.0), (881.0, 175.0, 78.0)):
        d = fz / 100.173
        chord = 2 * math.sqrt(2 * 228.6 * d - d * d)
        assert l < chord < w
    assert sf.brush_trail_mm(1248.0) == pytest.approx(13.9, abs=0.05)
    t = default_tire()
    def peak(t0):
        co = sf.aligning_curve(t, 1248.0, 12.6667, t0_mm=t0); ci = sf.aligning_curve(t, 165.0, 12.6667)
        a = np.asarray(co["alpha_deg"]); return float((np.asarray(co["M_Nm"]) + np.interp(a, ci["alpha_deg"], ci["M_Nm"])).max())
    assert peak(19.6) < 10.0 * 120.0 / 25.6 < peak(19.8)          # the t0 limit at the 4.69:1 window edge, 19.7 mm
    assert peak(78.0 / 6.0) < peak(85.2 / 6.0) < 10.0 * 120.0 / 25.6


def test_lower_fore_bracket_bound_and_resolved_load():
    # bracket_fos puts the whole leg load across both plates (thin axis); resolved along and across the bolt the
    # final front's node bracket reaches 4.16 and the delivered bracket as built 0.66
    import numpy as np
    assert ga.bracket_fos(8736, 13.0, 50, 6.35)["fos"] == pytest.approx(2.13, abs=0.005)
    def split(name, F, r, w, t):
        hp = _winner(name)
        A, B, O = (np.asarray(getattr(hp, p), float) for p in ("lower_front_inner", "lower_rear_inner", "lower_outer"))
        c = abs(float(((O - A) / np.linalg.norm(O - A)) @ ((B - A) / np.linalg.norm(B - A))))
        fa, fp = F * c, F * math.sqrt(1 - c * c)
        s = 1.28 * (fa / 2 * r) / (w * t * t / 6) + 1.28 * (fp / 2 * r) / (t * w * w / 6) + 1.28 * (fp / 2) / (w * t)
        return math.degrees(math.acos(c)), 460.0 / s
    ang, fos = split("front_v11_final", 8736.0, 13.0, 50.0, 6.35)
    assert (round(ang), round(fos, 2)) == (72, 4.16)
    assert round(split("front_steering_walls", 10506.0, 27.9, 30.0, 5.0)[1], 2) == 0.66
    assert round(split("front_steering_walls", 10506.0, 13.0, 50.0, 6.35)[1], 2) == 3.69
