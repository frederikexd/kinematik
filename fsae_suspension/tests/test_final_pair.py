# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The numbers the paper quotes for its final pair, recomputed from the published manifests.

Final front: front_v11_final, the pickups of front_v11_hairpin_walls (the 4.5 m hairpin on the axle's two-track mean
steer, 25.6 deg, at >= 75% Ackermann; joint misalignment and cornering compliance toe walled, the links in series;
81 mm rack) with the pushrod on the upright, the rocker and spring re-placed for it, the compliance walls on the
frame-derived pickup flexibility, the lower-front pickup moved 41.5 mm forward along the lower pivot axis for the
braking compliance wall and the spring mount on the damper axis for the declared damper (scripts/final_corners_v3.py).
Final rear: rear_v11_final, the pickups of rear_v11_walls declared the same way (damper fit only). front_v11_29deg_walls (ALT) and front_v11_25deg_walls (PAR) are
superseded; on the corrected code the walled corners' recorded winners are refused (test_superseded_walls_corners_...).
Every assertion is a value printed in the paper or its supplement; if the code moves one, this fails.
"""
import json
import math
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from suspension import (actuation as act, dynamic_ackermann as da, elastokinematics as ek,
                        genesis_analysis as ga, genesis_walls as gw, inverse_genesis as ig,
                        robustness as rb, target_derivation as td, tire_field as tf)
from suspension import steering_feel as sf
from suspension.genesis_repro import GenesisManifest, hp_from_dict, vehicle_load_cases
from suspension.compliance import MemberStiffness
from suspension.ghost_topology import _rc_height_mm
from suspension.kinematics import SuspensionKinematics
from suspension.tiremodel import default_tire

MAN = Path(__file__).resolve().parents[1] / "manifests"
TRACK = 1210.0
FRONT, ALT, PAR, REAR = "front_v11_final", "front_v11_29deg_walls", "front_v11_25deg_walls", "rear_v11_final"
HAIRPIN = "front_v11_hairpin_walls"
UPSIZED = {"LR": [15.875, 1.651], "LF": [15.875, 1.245]}


def _m(name):
    return GenesisManifest.from_json((MAN / f"{name}.recorded.genesis.json").read_text())


def _w(name):
    return hp_from_dict(_m(name).recorded["winner_hardpoints"])


def _corner(hp):
    tr = np.linspace(-25, 25, 11)
    k = SuspensionKinematics(hp)
    sts = [k.solve_at_travel(float(t)) for t in tr]
    rc = np.array([_rc_height_mm(s, TRACK) for s in sts])
    cam = np.array([s.camber for s in sts]); toe = np.array([s.toe for s in sts])
    return {"rc": float(rc[5]), "mig": float(np.polyfit(tr, rc, 1)[0]), "gain": float(np.polyfit(tr, cam, 1)[0]),
            "bump_steer": float(toe.max() - toe.min())}


def _anti(hp, prop, **kw):
    b = ig.SolvedPropertyBounds(bounds=[ig.PropertyBound(prop, lo=-1e9)], **kw)
    return ig.properties_of(hp, b, only=[prop])[prop]


@pytest.fixture(scope="module")
def pair():
    hf, hr = _w(FRONT), _w(REAR)
    F, R = _corner(hf), _corner(hr)
    F["ad73"] = _anti(hf, "anti_dive_pct", brake_bias_front=0.73)
    F["ad60"] = _anti(hf, "anti_dive_pct", brake_bias_front=0.60)
    R["as"] = _anti(hr, "anti_squat_pct")
    R["al_in"] = _anti(hr, "anti_lift_pct", brake_bias_front=0.73, rear_brakes_inboard=True)
    R["al_out"] = _anti(hr, "anti_lift_pct", brake_bias_front=0.73, rear_brakes_inboard=False)
    return hf, hr, F, R


def _pitch(ad, al, a_g=1.5):
    dF = 300 * 9.81 * a_g * 0.280 / (2 * 1.630)
    kf = 60.1 * (2 * math.pi * 2.6) ** 2 / 1000; kr = 66.1 * (2 * math.pi * 3.0) ** 2 / 1000
    dive = dF * (1 - ad / 100) / kf; rise = dF * (1 - al / 100) / kr
    return dive, rise, math.degrees(math.atan((dive + rise) / 1630)), dF * (1 - al / 100)


def test_final_rear_holds_every_wall():
    m = _m(REAR); _, _, vol, _ = m.objects(); hp = _w(REAR)
    assert m.recorded["winner_verdict"] == "RESILIENT" and m.inputs_sha256.startswith("8818c4b890d1")
    assert m.recorded["winner_hardpoints"] == json.loads(m.to_json())["hardpoints"]    # returns its start
    assert vol.wall_violations(hp) == []
    v = vol.properties.evaluate(hp)
    assert v["brake_decel_g"] == pytest.approx(1.57, abs=0.005)
    assert v["link_fos_min"] == pytest.approx(2.24, abs=0.005)
    assert v["rocker_bearing_sf"] == pytest.approx(2.08, abs=0.005)
    assert vol.envelope_margin(hp) == pytest.approx(1.19, abs=0.01)
    assert vol.envelope_margin(_w("rear_v11")) == pytest.approx(-2.75, abs=0.01)     # 0.25 mm from the tire


def test_final_front_holds_every_wall():
    m = _m(FRONT); _, _, vol, _ = m.objects(); hp = _w(FRONT)
    assert m.recorded["winner_verdict"] == "RESILIENT" and m.inputs_sha256.startswith("2e7072ce2268")
    assert vol.wall_violations(hp) == []
    v = vol.properties.evaluate(hp)
    assert v["ackermann_lock_pct"] == pytest.approx(75.27, abs=0.005)
    assert v["joint_misalignment_lock_deg"] == pytest.approx(4.41, abs=0.005)
    assert v["compliance_toe_deg"] == pytest.approx(0.0335, abs=0.0005)           # derived frame, links in series
    assert v["compliance_toe_braking_deg"] == pytest.approx(0.230, abs=0.0005)    # the braking wall, +/-0.25 deg
    assert v["anti_dive_pct"] == pytest.approx(68.94, abs=0.005)
    assert (round(v["link_fos_min"], 2), round(v["rocker_bearing_sf"], 2), round(v["brake_decel_g"], 2)) == (2.85, 2.44, 1.57)
    assert vol.wheel_envelope.rack_travel_mm == pytest.approx(27.0)
    assert m.recorded["winner_hardpoints"] == json.loads(m.to_json())["hardpoints"]    # the genesis_seed start, returned unchanged
    assert (m.context["seed_search_stage1"]["evaluations"], m.context["seed_search"]["evaluations"]) == (1274, 3)
    assert vol.properties.elasto.stiffness.provenance == "fea"                          # the frame-derived flexibility
    assert getattr(hp, "pushrod_attach") == "upright"
    hh = _w(HAIRPIN)
    for p in ("upper_front_inner", "upper_rear_inner", "lower_rear_inner", "tie_rod_inner",
              "tie_rod_outer", "upper_outer", "lower_outer"):                          # the pickups of the hairpin front
        assert np.allclose(getattr(hp, p), getattr(hh, p), atol=1e-9)
    a, b = np.asarray(hh.lower_front_inner, float), np.asarray(hh.lower_rear_inner, float)
    u = (b - a) / np.linalg.norm(b - a); d = np.asarray(hp.lower_front_inner, float) - a
    assert np.linalg.norm(np.cross(d, u)) < 1e-9 and d @ u == pytest.approx(-41.48, abs=0.01)   # forward along the axis
    assert np.linalg.norm(np.asarray(hp.lower_rear_inner) - np.asarray(hp.lower_front_inner)) == pytest.approx(166.9, abs=0.05)


def test_moving_the_lower_front_pickup_along_its_axis_changes_no_curve():
    hp, hh = _w(FRONT), _w(HAIRPIN)
    k0, k1 = SuspensionKinematics(hh), SuspensionKinematics(hp)
    for t in (-25.0, 0.0, 25.0):
        s0, s1 = k0.solve_at_travel(t), k1.solve_at_travel(t)
        assert abs(s0.camber - s1.camber) < 1e-9 and abs(s0.toe - s1.toe) < 1e-9
        assert np.allclose(s0.wheel_center, s1.wheel_center, atol=1e-9)


@pytest.mark.parametrize("name,link,bearing", [(HAIRPIN, 0.24, 1.61), (PAR, 0.34, 1.97), (ALT, 0.18, 1.58),
                                               ("rear_v11_walls", 0.13, 1.94)])
def test_superseded_walls_corners_are_refused_on_the_corrected_code(name, link, bearing):
    # re-run on the corrected load cases and statics these manifests return no winner: the start each recorded as its
    # winner (pushrod on the lower wishbone) now fails the link wall and the 6204 bearing wall
    m = _m(name); _, _, vol, _ = m.objects(); hp = _w(name)
    assert m.recorded["winner_hardpoints"] == json.loads(m.to_json())["hardpoints"]
    assert [p for p, _, _ in vol.property_violations(hp)] == ["link_fos_min", "rocker_bearing_sf"]
    v = vol.properties.evaluate(hp)
    assert (round(v["link_fos_min"], 2), round(v["rocker_bearing_sf"], 2)) == (link, bearing)


def test_final_fronts_clearance_and_fit():
    for name, fit, clear in ((FRONT, 0.347, 0.49), (PAR, 0.295, 2.16), (ALT, 0.359, 2.01)):
        m = _m(name); _, _, vol, _ = m.objects()
        assert m.recorded["candidates"][0]["max_band_frac"] == pytest.approx(fit, abs=0.0005)
        assert vol.envelope_margin(_w(name)) == pytest.approx(clear, abs=0.005)


def test_corners_and_balance(pair):
    hf, hr, F, R = pair
    assert (round(F["rc"], 1), round(R["rc"], 1)) == (52.8, 41.6)
    assert td.front_lltd_share(F["rc"], R["rc"], 0.522) == pytest.approx(0.525, abs=0.0005)
    assert (round(F["gain"], 3), round(R["gain"], 3)) == (-0.035, -0.029)
    assert (round(F["bump_steer"], 3), round(R["bump_steer"], 3)) == (0.028, 0.033)
    assert (round(F["ad73"], 1), round(F["ad60"], 1), round(R["as"], 1)) == (68.9, 56.7, 46.7)
    assert (round(R["al_in"], 1), round(R["al_out"], 1)) == (12.6, -86.8)
    assert td.stiffness_share_for_lltd(0.53, 39.5, 58.5) == pytest.approx(0.561, abs=0.0005)


def test_floors_thresholds_and_derived_frequencies(pair):
    _, _, F, R = pair
    v = td.Vehicle(); t = td.Tire(); a = td.design_accelerations(t)
    assert td.f_min(v, F["ad73"], a["straight"], 0.0, "front") == pytest.approx(1.42, abs=0.005)
    assert td.f_min(v, F["ad73"], a["combined"], a["combined"], "front") == pytest.approx(2.00, abs=0.005)
    assert td.f_min(v, R["as"], a["straight"], 0.0, "rear") == pytest.approx(1.77, abs=0.005)
    assert td.f_min(v, R["as"], a["combined"], a["combined"], "rear") == pytest.approx(2.50, abs=0.005)
    assert td.ride_frequency_derived(v, t, F["ad73"], "front") == pytest.approx(2.21, abs=0.005)
    assert td.ride_frequency_derived(v, t, R["as"], "rear") == pytest.approx(2.76, abs=0.005)
    assert tf.mu_threshold_for("anti_dive_floor_pct", F["ad73"], veh=replace(v, ride_front_hz=2.6)) == pytest.approx(1.95, abs=0.005)
    assert tf.mu_threshold_for("anti_squat_floor_pct", R["as"]) == pytest.approx(1.83, abs=0.005)     # the rear binds
    t10 = replace(t, peak_mu=t.peak_mu * 1.1); a10 = td.design_accelerations(t10)
    assert td.anti_floor_pct(v, 2.6, a10["combined"], a10["combined"], "front") == pytest.approx(57.0, abs=0.05)
    assert td.roll_gradient_ceiling(v, t, R["as"], "rear", 3.0) == pytest.approx(1.02, abs=0.005)
    assert td.roll_gradient_ceiling(v, t, R["as"], "rear", 3.0, mu_allowance=0.10) == pytest.approx(0.88, abs=0.005)


def test_pitch_and_share_shift_in_braking(pair):
    _, _, F, R = pair
    dive, rise, deg, unload = _pitch(F["ad73"], R["al_in"])
    assert (round(dive, 1), round(rise, 1), round(deg, 2), round(unload)) == (7.3, 14.1, 0.75, 331)
    assert _pitch(F["ad73"], R["al_in"], 1.8)[2] == pytest.approx(0.90, abs=0.005)
    s0 = td.front_lltd_share(F["rc"], R["rc"], 0.522)
    body = td.front_lltd_share(F["rc"] + F["mig"] * dive, R["rc"] - R["mig"] * rise, 0.522)
    cg = 280.0 - (dive - 0.52 * (dive + rise))
    ground = td.front_lltd_share(F["rc"] + F["mig"] * dive - dive, R["rc"] - R["mig"] * rise + rise, 0.522, cg_height_mm=cg)
    assert 100 * (body - s0) == pytest.approx(-0.09, abs=0.005)
    assert 100 * (ground - s0) == pytest.approx(-2.04, abs=0.01)
    kr = 66.1 * (2 * math.pi * 3.0) ** 2 / 1000
    assert 379.16 * (1 - R["al_out"] / 100) / kr == pytest.approx(30.2, abs=0.05)    # outboard rear extension


def test_steering_on_the_final_front():
    hp = _w(FRONT); _, _, vol, _ = _m(FRONT).objects(); se = vol.properties.steer_effort
    assert (se.lock_basis, se.rack_c_mm_per_rev, se.lock_road_wheel_deg) == ("mean", 81.0, 25.6)
    rc = se.kinematic_ratio(hp); sw = se.lock_sw_deg(hp)
    assert (round(rc, 2), round(sw, 1), round(sw / 25.6, 2)) == (4.99, 118.9, 4.64)
    t = default_tire()
    co = sf.aligning_curve(t, 1248.0, 12.6667); ci = sf.aligning_curve(t, 165.0, 12.6667)
    a = np.asarray(co["alpha_deg"]); M = np.asarray(co["M_Nm"]) + np.interp(a, ci["alpha_deg"], ci["M_Nm"])
    pk, lim = float(M.max()), float(np.interp(co["alpha_peak_deg"], a, M))
    assert (round(pk, 1), round(lim, 1)) == (40.5, 27.4)
    assert (round(pk / rc, 1), round(lim / rc, 1), round((pk - lim) / rc, 1)) == (8.1, 5.5, 2.6)
    r = da.lock_report(hp, se.rack_c_mm_per_rev, 25.6, lock_basis="mean")
    assert (round(r["mean_deg"], 2), round(r["inner_deg"], 2), round(r["outer_deg"], 2)) == (25.6, 28.74, 22.46)
    assert [round(x, 1) for x in r["ackermann_pct_lo_and_lock"]] == [63.2, 75.3]
    assert (round(r["camber_at_lock_outside_deg"], 2), round(r["camber_at_lock_inside_deg"], 2)) == (-3.61, 0.18)
    rack = sw * se.rack_c_mm_per_rev / 360.0
    assert rack == pytest.approx(26.75, abs=0.005)
    assert round(ig.joint_swing(hp, rack_travel_mm=rack)["_worst"][1], 1) == 32.9          # an upper bound only
    jm = ig.joint_misalignment(hp, rack_travel_mm=rack)
    assert (round(jm["tie_rod_outer"], 2), round(jm["upper_ball"], 2), round(jm["lower_ball"], 2)) == (4.41, 4.41, 2.77)
    inner = da.lock_report(hp, se.rack_c_mm_per_rev, 25.6, lock_basis="inner")
    assert inner["mean_deg"] < 25.6                                   # an inner-wheel lock would leave the mean short
    r0 = da.lock_report(_w("front_v11_25deg"), 76.2, 25.0)
    assert round(r0["mean_deg"], 1) == 23.8                           # the released 25 deg front, inner-wheel lock


def test_table_8_on_the_final_corners():
    f = _m(FRONT).objects()[2].properties.structure.margins(_w(FRONT))
    assert (round(f["LR"]["worst_n"]), round(f["LR"]["fos_buckling"], 2), round(f["LR"]["fos_yield"], 2)) == (-7744, 2.85, 3.46)
    assert (round(f["LF"]["worst_n"]), round(f["LF"]["fos_yield"], 2)) == (8736, 2.85)
    assert min(min(v["fos_buckling"], v["fos_yield"]) for k, v in f.items() if k not in ("LF", "LR")) == pytest.approx(6.67, abs=0.005)
    s = gw.LinkStructure.from_shock_cases("front", 0.73).margins(_w(FRONT))
    assert (round(s["LR"]["fos_buckling"], 2), round(s["LR"]["fos_yield"], 2), round(s["LF"]["fos_yield"], 2)) == (1.78, 1.85, 2.08)
    r = _m(REAR).objects()[2].properties.structure.margins(_w(REAR))
    assert (round(r["UR"]["worst_n"]), round(r["UR"]["fos_buckling"], 2), round(r["UR"]["fos_yield"], 2)) == (-2894, 6.41, 5.62)
    assert (round(r["LF"]["worst_n"]), round(r["LF"]["fos_buckling"], 2), round(r["LF"]["fos_yield"], 2)) == (-7833, 2.24, 3.25)
    assert (round(r["LR"]["worst_n"]), round(r["LR"]["fos_buckling"], 2), round(r["LR"]["fos_yield"], 2)) == (9356, 7.03, 2.66)
    assert ga.bracket_fos(8736, 13.0, 50, 6.35)["fos"] == pytest.approx(2.13, abs=0.005)


def test_bearings_on_the_final_corners():
    for name, axle, s01, label, c0, s0, c30, c60 in ((FRONT, "front", 0.88, "6204", 6550.0, 2.44, 5.4, 4.4),
                                                     (REAR, "rear", 0.63, "6205", 7800.0, 2.08, 7.5, 6.1)):
        hp = _w(name)
        kw = {"traction_g": 1.06, "inboard_drive": True, "inboard_brakes": True} if axle == "rear" else {}
        cases = list(vehicle_load_cases(axle=axle, brake_bias_front=0.73, **kw))
        assert min(r["s0"] for r in act.rocker_bearing_loads(hp, cases)) == pytest.approx(s01, abs=0.005)
        assert min(r["s0"] for r in act.rocker_bearing_loads(hp, cases, act.RockerBearing(label=label, C0_N=c0))) == pytest.approx(s0, abs=0.005)
        req = act.required_rating_vs_spacing(hp, cases, [30.0, 60.0])
        assert (round(req[0]["required_C0_N"] / 1000, 1), round(req[1]["required_C0_N"] / 1000, 1)) == (c30, c60)


def test_compliance_of_the_final_front():
    # the compliance wall's own stiffness: the frame-derived 15 x 15 pickup flexibility, links (3/4 x 0.083 in tie rod,
    # upsized lower legs) in series
    hp = _w(FRONT); el = _m(FRONT).objects()[2].properties.elasto
    out, links = {}, {}
    for c in vehicle_load_cases(axle="front", brake_bias_front=0.73):
        kw = dict(Fx=c.Fx, Fy=c.Fy, Fz=c.Fz, Mz=c.mz_Nmm, Fx_wc=getattr(c, "Fx_wc", 0.0))
        out[c.name] = ek.solve_elastokinematic(hp, ek.ElastoSpec(stiffness=el.stiffness, members=dict(el.members), **kw),
                                               jacobian=False).change
        links[c.name] = ek.solve_elastokinematic(hp, ek.ElastoSpec(stiffness=ek.StiffnessField(1e9), members=dict(el.members), **kw),
                                                 jacobian=False).change
    assert out["1.5g corner"]["toe"] == pytest.approx(0.0335, abs=0.0005)     # the compliance wall, 42% of +/-0.08
    assert links["1.5g corner"]["toe"] == pytest.approx(0.007, abs=0.0005)
    assert out["1.5g braking"]["toe"] == pytest.approx(0.230, abs=0.0005)     # the braking wall, +/-0.25
    assert out["1.5g braking"]["caster"] == pytest.approx(-1.02, abs=0.005)
    assert links["1.5g braking"]["toe"] == pytest.approx(0.185, abs=0.0005)
    assert out["combined 1.06g"]["toe"] == pytest.approx(0.205, abs=0.0005)
    lf = ek.solve_elastokinematic(hp, ek.ElastoSpec(stiffness=ek.StiffnessField(1e9), members={"LF": el.members["LF"]},
                                                    **_case("front", "1.5g braking")), jacobian=False).change
    assert lf["toe"] == pytest.approx(0.20, abs=0.005)                       # the lower-front leg's own stretch


def _case(axle, name):
    kw = {"traction_g": 1.06, "inboard_drive": True, "inboard_brakes": True} if axle == "rear" else {}
    c = [c for c in vehicle_load_cases(axle=axle, brake_bias_front=0.73, **kw) if c.name == name][0]
    return dict(Fx=c.Fx, Fy=c.Fy, Fz=c.Fz, Mz=c.mz_Nmm, Fx_wc=getattr(c, "Fx_wc", 0.0))


def test_compliance_of_the_final_rear_at_the_declared_rate():
    # Table S6.36, first column: 30 kN/mm pickups, the rear's links (lower legs 5/8 x 0.065 and 0.049 in) in series
    hp = _w(REAR)
    mem = {k: MemberStiffness(material="Steel 4130", od_mm=15.875, wall_mm={"LF": 1.651, "LR": 1.245}.get(k, 0.889))
           for k in ("UF", "UR", "LF", "LR", "TR")}

    def ch(name):
        return ek.solve_elastokinematic(hp, ek.ElastoSpec(stiffness=ek.StiffnessField(30000.0), members=mem, **_case("rear", name)),
                                        jacobian=False).change
    assert ch("1.5g corner")["camber"] == pytest.approx(0.088, abs=0.0005)
    assert ch("1.06g traction")["toe"] == pytest.approx(-0.144, abs=0.0005)
    assert ch("combined 1.06g traction")["toe"] == pytest.approx(-0.096, abs=0.0005)


def test_twist_adjustment_and_warp():
    hp = _w(FRONT)
    J = ek.compliance_jacobian(hp, points=("tie_rod_inner",))
    u = np.asarray(hp.tie_rod_outer, float) - np.asarray(hp.tie_rod_inner, float); u /= np.linalg.norm(u)
    tpm = float(abs(J[1] @ u))
    assert tpm == pytest.approx(0.91, abs=0.005)
    assert ek.frame_twist_toe(618.0, 2000.0, 1630.0, 250.0, 200.0, tpm) == pytest.approx(0.150, abs=0.0005)
    assert ek.frame_twist_toe(51.9, 2000.0, 1630.0, 250.0, 200.0, tpm) == pytest.approx(0.0126, abs=0.0005)
    assert 0.176 * tpm == pytest.approx(0.160, abs=0.0005)                    # one flat of the 5/16-24 toe thread
    w = td.steer_warp(hp, 26.752, 454.0, 405.0, TRACK, TRACK)
    assert (round(w["patch_dz_right_mm"], 2), round(w["patch_dz_left_mm"], 2)) == (0.76, -0.96)
    assert (round(w["spring_dz_right_mm"], 1), round(w["spring_dz_left_mm"], 1)) == (4.7, 1.2)   # pushrod turns with the upright
    assert round(w["front_pair_N"]) == 44
    assert [round(w["left_steer_N"][k]) for k in ("FR", "FL", "RR", "RL")] == [-44, 44, 44, -44]   # Table 10, row 13
    loads = max(td.corner_loads(td.Vehicle(), a_lat_g=1.5, bank_deg=2.0, grade_deg=-2.0, downforce_n=442.0,
                                steer_warp_n=sg * w["front_pair_N"])["total"]["FR"] for sg in (1.0, -1.0))
    assert round(loads) == 1434


def test_actuation_of_the_final_corners():
    for name, mr, spread, tmin, tmax, stroke in ((FRONT, 0.804, 0.077, 60.5, 111.5, 40.0), (REAR, 0.821, 0.034, 73.5, 104.9, 40.8)):
        m = act.actuation_metrics(_w(name))
        assert (round(m.mr_static, 3), round(m.spread, 3), round(m.stroke_used_mm, 1)) == (mr, spread, stroke)
        assert (round(m.transmission_deg[0], 1), round(m.transmission_deg[1], 1)) == (tmin, tmax)
    ot = rb.overtravel_check(_w(FRONT))
    assert (round(ot["transmission_min_deg"], 1), round(ot["transmission_max_deg"], 1)) == (37.9, 124.0)
    assert (round(ot["mr_max"], 2), round(ot["mr_min"], 2), round(ot["stroke_used_mm"], 1)) == (0.81, 0.58, 56.9)
    assert rb.overtravel_check(_w(FRONT), bump_mm=60.0)["all_steps_solved"]
    assert rb.overtravel_check(_w(REAR))["damper_bottoms_at_mm"] == pytest.approx(46.5, abs=0.05)   # 57 mm from full droop
    # the declared damper (200 mm eye to eye extended, 57 mm stroke): its margin to top-out at full droop is not stroke
    assert rb.overtravel_check(_w(FRONT), damper_extended_mm=200.0)["damper_bottoms_at_mm"] == pytest.approx(45.6, abs=0.05)
    assert rb.overtravel_check(_w(REAR), damper_extended_mm=200.0)["damper_bottoms_at_mm"] == pytest.approx(43.1, abs=0.05)


def test_the_declared_damper_fits_both_final_corners():
    t = act.ActuationTargets.derived(damper_extended_mm=200.0, stroke_mm=57.0)
    for name, lengths in ((FRONT, (197.2, 176.8, 157.2)), (REAR, (197.6, 177.0, 156.8))):
        m = act.actuation_metrics(_w(name))
        assert tuple(round(v, 1) for v in m.spring_length_mm) == lengths
        assert m.spring_length_mm[0] <= t.damper_extended_mm                                   # no top-out at full droop
        assert m.spring_length_mm[2] >= t.damper_extended_mm - t.usable_stroke * t.stroke_mm   # inside 80% at full bump
        assert act._residual(m, t)[-2:].tolist() == [0.0, 0.0]
    old = hp_from_dict(json.loads((MAN / "superseded" / "front_v11_final.cea1a7c48229.recorded.genesis.json").read_text())
                       ["recorded"]["winner_hardpoints"])
    m = act.actuation_metrics(old)                    # the spring line before the damper was declared: 106.7 mm
    assert round(m.spring_length_mm[1], 1) == 106.7 and act._residual(m, t)[-1] > 0
