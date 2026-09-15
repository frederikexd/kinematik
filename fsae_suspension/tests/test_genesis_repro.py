# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  tests/test_genesis_repro.py — manifests, transforms, spacing constraints,
#  diagnostics and the declared-car mode, pinned against published numbers.
# ============================================================================
import json
import math

import numpy as np
import pytest

from suspension.kinematics import Hardpoints
from suspension.kinematik_stochastic import ToleranceField, _perturbed
from suspension import inverse_genesis as ig
from suspension import genesis_repro as gr
from suspension import inverse_genesis_fullcar as fc

A = np.array
ST = A([-25.0, -12.5, 0.0, 12.5, 25.0])

# Table 13 of Thio, "Inverse Kinematic Synthesis of a Formula SAE
# Double-Wishbone Suspension…" (corner frame, mm)
FRONT = dict(
    upper_front_inner=A([-120.0, 288.1, 280.5]),
    upper_rear_inner=A([100.0, 270.2, 267.0]),
    lower_front_inner=A([-260.0, 170.8, 103.9]),
    lower_rear_inner=A([42.8, 148.9, 133.7]),
    tie_rod_inner=A([43.0, 193.8, 147.8]),
    upper_outer=A([6.5, 560.5, 300.0]), lower_outer=A([-5.0, 583.6, 120.0]),
    tie_rod_outer=A([90.0, 572.0, 150.0]),
    wheel_center=A([0.0, 600.0, 228.0]), contact_patch=A([0.0, 605.0, 0.0]))
REAR = dict(
    upper_front_inner=A([-290.4, 357.6, 351.9]),
    upper_rear_inner=A([-118.2, 200.0, 297.1]),
    lower_front_inner=A([-230.0, 260.0, 140.0]),
    lower_rear_inner=A([-69.4, 205.0, 127.3]),
    tie_rod_inner=A([-53.9, 270.0, 164.2]),
    upper_outer=A([3.0, 571.0, 300.0]), lower_outer=A([-5.0, 590.2, 120.0]),
    tie_rod_outer=A([90.0, 579.2, 150.0]),
    wheel_center=A([0.0, 600.0, 228.0]), contact_patch=A([0.0, 605.0, 0.0]))


@pytest.fixture(scope="module")
def front():
    return Hardpoints(**FRONT, static_camber=-1.5, static_toe=0.0)


@pytest.fixture(scope="module")
def rear():
    return Hardpoints(**REAR, static_camber=-1.0, static_toe=0.0)


# ---- frames ---------------------------------------------------------------- #
def test_cad_corner_round_trip_and_handedness():
    p = A([[12.0, -18.5, 1000.0], [-300.0, 250.0, -600.0]])
    c = gr.cad_to_corner(p, 950.0, -50.0)
    assert np.allclose(c[0], [-50.0, 12.0, 31.5])
    assert np.allclose(gr.corner_to_cad(c, 950.0, -50.0), p)
    M = np.array([gr.cad_to_corner(e, 0.0, 0.0) for e in np.eye(3)])
    assert np.isclose(np.linalg.det(M), -1.0)


# ---- geometry primitives --------------------------------------------------- #
def test_segment_distance_known_cases():
    assert math.isclose(gr.seg_seg_dist([0, 0, 0], [1, 0, 0],
                                        [0, 1, 0], [1, 1, 0]), 1.0)
    assert math.isclose(gr.seg_seg_dist([0, 0, 0], [1, 0, 0],
                                        [0.5, -1, 1], [0.5, 1, 1]), 1.0)
    assert math.isclose(gr.seg_seg_dist([0, 0, 0], [1, 0, 0],
                                        [3, 0, 0], [4, 0, 0]), 2.0)


def test_capsule_clearance_is_signed():
    cap = gr.CapsuleObstacle([[0, 0, 0]], [[100, 0, 0]], [10.0])
    cl = cap.clearances([[50, 30, 0], [50, 5, 0], [-20, 0, 0]], 2.0)
    assert np.allclose(cl, [18.0, -7.0, 8.0])


# ---- spacing constraints --------------------------------------------------- #
def test_point_spacing_is_a_wall(front):
    sp = ig.PointSpacing("lower_front_inner", "lower_rear_inner", "x", 150.0)
    assert math.isclose(sp.gap(front), 42.8 - (-260.0))
    vol = ig.LegalVolume(
        boxes=gr.boxes_about(front, {"lower_rear_inner": 40.0}),
        spacings=[ig.PointSpacing("lower_front_inner", "lower_rear_inner",
                                  "x", 400.0)])
    assert vol.keepout_violations(front)          # 302.8 < 400 → refused
    ok = ig.LegalVolume(boxes=vol.boxes, spacings=[sp])
    assert not ok.keepout_violations(front)
    with pytest.raises(ValueError):
        ig.PointSpacing("lower_front_inner", "nope", "x", 1.0)


def test_free_axis_is_finite_in_json(front):
    b = gr.boxes_about(front, {"upper_outer": [math.inf, 0.0, 0.0]})
    lo, hi = b["upper_outer"]
    assert lo[0] < -1e5 and hi[0] > 1e5 and lo[1] == hi[1]
    json.dumps(gr.volume_to_dict(ig.LegalVolume(boxes=b)))


# ---- manifest --------------------------------------------------------------- #
def _small_manifest():
    hp = Hardpoints.default()
    truth, _ = ig.curves_of(_perturbed(hp, {
        "upper_front_inner": A([0.0, -4.0, 5.0]),
        "upper_rear_inner": A([0.0, -4.0, 5.0])}), ST)
    tg = ig.GenesisTargets(curves=[
        ig.TargetCurve("camber_deg", ST, truth["camber_deg"],
                       np.full(5, 0.15)),
        ig.TargetCurve("rc_height_mm", ST, truth["rc_height_mm"],
                       np.full(5, 6.0))])
    vol = ig.LegalVolume(
        boxes=gr.boxes_about(hp, {"upper_front_inner": 8.0,
                                  "upper_rear_inner": 8.0}),
        keep_out=[ig.KeepOutBox([0, 0, -50], [10, 10, -40], "box"),
                  gr.CapsuleObstacle([[0, 0, -100]], [[1, 0, -100]], [1.0])],
        spacings=[ig.PointSpacing("upper_front_inner", "upper_rear_inner",
                                  "x", 0.0)])
    fld = gr.field_with_overrides("jig_weld", {"tie_rod_inner": 0.5})
    return gr.GenesisManifest.build(
        "small", hp, tg, vol, fld,
        gr.SearchSettings(seed=3, n_starts=2, n_yield=300, max_iter=12),
        {"axle": "front"})


@pytest.fixture(scope="module")
def ran():
    m = _small_manifest()
    res = m.run()
    m.record(res)
    return m, res


def test_manifest_round_trip_preserves_inputs(ran):
    m, _ = ran
    m2 = gr.GenesisManifest.from_json(m.to_json())
    assert m2.inputs_sha256 == m.inputs_sha256
    hp, tg, vol, fld = m2.objects()
    assert fld.specs["tie_rod_inner"].hi[0] == 0.5
    assert len(vol.spacings) == 1 and len(vol.keep_out) == 2
    assert hp.static_camber == Hardpoints.default().static_camber


def test_manifest_rerun_is_byte_identical(ran):
    m, res = ran
    m2 = gr.GenesisManifest.from_json(m.to_json())
    same, diffs = m2.verify(m2.run())
    assert same, diffs


def test_manifest_detects_tampering(ran):
    m, _ = ran
    d = json.loads(m.to_json())
    d["search"]["seed"] = 99
    with pytest.raises(ValueError, match="sha256"):
        gr.GenesisManifest.from_json(json.dumps(d))
    d.pop("inputs_sha256")
    assert gr.GenesisManifest.from_json(json.dumps(d)).search.seed == 99


def test_verify_reports_a_changed_input(ran):
    m, _ = ran
    m2 = gr.GenesisManifest.from_json(m.to_json())
    m2.search.n_yield = 200
    same, diffs = m2.verify(m2.run())
    assert not same and diffs


# ---- published numbers (Table 12 / 9b / 7b of the paper) ------------------ #
def test_table12_delivered_front(front):
    d = gr.corner_diagnostics(front, track_mm=1210, axle="front",
                              wheelbase_mm=1630, cg_height_mm=280,
                              brake_bias_front=0.60)
    assert round(d["camber_gain_deg_per_mm"], 4) == -0.0367
    assert round(d["toe_change_deg"], 3) == 0.046
    assert round(d["rc_height_static_mm"], 1) == 58.5
    assert round(d["rc_migration_chassis_mm_per_mm"], 3) == 0.044
    assert round(d["kpi_deg"], 2) == 7.31
    assert round(d["scrub_static_mm"], 1) == 6.0
    assert round(d["contact_patch_rise_per_mm"], 2) == 1.01
    assert abs(d["side_view_ic_x_mm"] - 1196) < 1
    assert abs(d["side_view_ic_z_mm"] - 236) < 1
    assert round(d["anti_dive_pct"]) == 69


def test_table12_delivered_rear(rear):
    d = gr.corner_diagnostics(rear, track_mm=1210, axle="rear",
                              wheelbase_mm=1630, cg_height_mm=280)
    assert round(d["camber_gain_deg_per_mm"], 4) == -0.0287
    assert round(d["rc_migration_chassis_mm_per_mm"], 3) == -0.604
    assert round(d["rc_migration_ground_mm_per_mm"], 2) == -1.61
    assert round(d["rc_above_ground_min_mm"], 2) == -0.85
    assert round(d["caster_deg"], 2) == 2.54
    assert abs(d["side_view_ic_x_mm"] - 1142) < 1
    assert round(d["anti_squat_pct"]) == -102


def test_table7b_camber_to_road():
    f = gr.camber_to_road(-1.5, -0.0367, 1.18, 1210, -1.83)
    r = gr.camber_to_road(-1.0, -0.0287, 1.18, 1210, -1.83)
    assert round(f["camber_to_road_deg"], 2) == -0.78
    assert round(r["camber_to_road_deg"], 2) == -0.18
    assert round(f["gain_required_deg_per_mm"], 3) == -0.121
    assert round(r["gain_required_deg_per_mm"], 3) == -0.161


def test_front_worst_case_W(front):
    tg = gr.linear_targets(ST, static_camber=-1.5, camber_gain=-0.035,
                           toe=0.0, rc_height=55.0, track_mm=1210)
    yb = gr.yield_breakdown(front, tg, ToleranceField.preset("jig_weld"),
                            n=1500)
    assert yb["yield"] == 1.0
    assert round(yb["worst_case_W"], 2) == 1.07
    assert yb["worst_case_row"].startswith("toe")
    assert round(yb["governing_headroom_sigma"], 1) == 3.9


def test_discordant_builds_exact_p():
    a = np.array([True] * 10)
    b = np.array([True] * 6 + [False] * 4)
    d = gr.discordant_builds(a, b)
    assert d["discordant"] == 4 and math.isclose(d["p_two_sided"], 0.125)


# ---- swept volume ---------------------------------------------------------- #
def test_swept_clearance_finds_a_tube_through_a_link(front):
    mid = 0.5 * (FRONT["lower_rear_inner"] + FRONT["lower_outer"])
    cap = gr.CapsuleObstacle([mid + [0, 0, -100]], [mid + [0, 0, 100]],
                             [5.0], ["probe tube"])
    sw = gr.swept_clearance(front, cap, n_stations=5)
    assert sw["ok"] and sw["min_clearance_mm"] < 0
    far = gr.CapsuleObstacle([[0, 0, 2000]], [[1, 0, 2000]], [5.0])
    assert gr.swept_clearance(front, far, n_stations=5)[
        "min_clearance_mm"] > 1000


# ---- FullCar ---------------------------------------------------------------- #
def test_rear_track_legacy_default_and_override():
    assert fc.DesignSpace().rear_track() == pytest.approx(1176.0)
    assert fc.DesignSpace(track_rear_mm=1210).rear_track() == 1210


def test_declared_intent_matches_paper_targets(front):
    tg = fc.kinematic_intent_for(
        None, fc.DesignSpace(), hp=front,
        declared=dict(static_camber=-1.5, camber_gain=-0.035, toe=0.0,
                      rc_height=55.0),
        bands=dict(camber_deg=0.30, toe_deg=0.08, rc_height_mm=18.0))
    cam = tg.curves[0]
    assert np.allclose(cam.target, -1.5 - 0.035 * ST)
    assert np.allclose(cam.band, 0.30)


def test_roll_gradient_is_an_input(front):
    a = fc.kinematic_intent_for(None, fc.DesignSpace(), hp=front,
                                peak_lat_g=1.5, roll_gradient_deg_per_g=1.2)
    b = fc.kinematic_intent_for(None, fc.DesignSpace(), hp=front,
                                peak_lat_g=1.5, roll_gradient_deg_per_g=0.8)
    assert not np.allclose(a.curves[0].target, b.curves[0].target)


def test_declared_car_uses_mf52_and_real_corners(front, rear):
    car = fc.DeclaredCar(front_hp=front, rear_hp=rear,
                         roll_stiffness_front=458, roll_stiffness_rear=420)
    s = car.summary(1.5)
    assert s["grip_model"] == "Pacejka MF5.2"
    assert round(s["rc_front_mm"], 1) == 58.5
    assert round(s["rc_rear_mm"], 1) == 39.5
    assert s["motion_ratio"]["front"].startswith("PROXY")
    assert 1.4 < s["max_lateral_g"] < 1.6
    sens = fc.lap_sensitivity(car, "cg_height_mm", [240, 320])
    assert sens["track_length_m"] == 334.0
    assert sens["spread_s"] > 0
