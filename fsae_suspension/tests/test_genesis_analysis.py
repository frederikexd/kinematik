# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  tests/test_genesis_analysis.py — the InverseGenesis corner/vehicle
#  analyses, pinned to reference values, and the STEP tube reader on a
#  synthetic B-rep.
# ============================================================================
import math
import os

import numpy as np
import pytest

from suspension import genesis_analysis as pt
from suspension.kinematics import Hardpoints

LB_IN = 0.1751268


# ---- synthetic STEP --------------------------------------------------------- #
class _Step:
    def __init__(self):
        self.lines, self.n = [], 0

    def add(self, s):
        self.n += 1
        self.lines.append(f"#{self.n}={s};")
        return self.n

    def point(self, p):
        return self.add("CARTESIAN_POINT('',(%r,%r,%r))" % tuple(map(float, p)))

    def vertex(self, p):
        return self.add(f"VERTEX_POINT('',#{self.point(p)})")

    def placement(self, o, z):
        po = self.point(o)
        d = self.add("DIRECTION('',(%r,%r,%r))" % tuple(map(float, z)))
        return self.add(f"AXIS2_PLACEMENT_3D('',#{po},#{d},$)")

    def face(self, surf, verts):
        oes = []
        for a, b in zip(verts, verts[1:] + verts[:1]):
            line = self.add("LINE('',#1,#1)")
            ec = self.add(f"EDGE_CURVE('',#{a},#{b},#{line},.T.)")
            oes.append(self.add(f"ORIENTED_EDGE('',*,*,#{ec},.T.)"))
        loop = self.add("EDGE_LOOP('',(" + ",".join(f"#{o}" for o in oes) + "))")
        bound = self.add(f"FACE_OUTER_BOUND('',#{loop},.T.)")
        return self.add(f"ADVANCED_FACE('',(#{bound}),#{surf},.T.)")

    def tube(self, a, b, R=12.7, r_in=None):
        a, b = np.asarray(a, float), np.asarray(b, float)
        d = (b - a) / np.linalg.norm(b - a)
        perp = np.cross(d, [0, 0, 1.0])
        if np.linalg.norm(perp) < 1e-6:
            perp = np.cross(d, [0, 1.0, 0])
        perp /= np.linalg.norm(perp)
        for sign in (1, -1):                    # two half-cylinder faces
            pl = self.placement(a, d)           # separate surface entities
            s = self.add(f"CYLINDRICAL_SURFACE('',#{pl},{R})")
            vs = [self.vertex(a + sign * R * perp), self.vertex(b + sign * R * perp)]
            self.face(s, vs)
        if r_in:
            pl = self.placement(a, d)
            s = self.add(f"CYLINDRICAL_SURFACE('',#{pl},{r_in})")
            self.face(s, [self.vertex(a + r_in * perp), self.vertex(b + r_in * perp)])

    def text(self):
        head = ("ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n"
                "#9000=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));\n")
        return head + "\n".join(self.lines) + "\nENDSEC;\nEND-ISO-10303-21;\n"


@pytest.fixture(scope="module")
def step_text():
    s = _Step()
    s.point((0, 0, 0))                          # #1 used by LINE placeholders
    s.tube((0, 0, 0), (500, 0, 0), r_in=11.05)                     # 1.65 wall
    s.tube((500, 0, 0), (500, 400, 0), r_in=10.29)                 # 2.41 wall
    s.tube((0, 0, 0), (0, 0, 300))                                  # no bore
    # a neighbour's vertex sits inside tube 1's radius past its end — the
    # envelope rule would over-extend tube 1 to reach it; own-faces must not
    s.vertex((520, 5, 0))
    # 90° bend, major radius 80, minor 12.7, centred at (1000, 0, 0), axis z
    pl = s.placement((1000, 0, 0), (0, 0, 1))
    tor = s.add(f"TOROIDAL_SURFACE('',#{pl},80.0,12.7)")
    s.face(tor, [s.vertex((1080, 0, 0)), s.vertex((1000, 80, 0))])
    return s.text()


def test_step_axes_are_bounded_by_their_own_faces(step_text):
    r = pt.parse_step_tubes(step_text)
    assert len(r.axes) == 3
    lengths = sorted(round(float(np.linalg.norm(b - a)), 6) for a, b, _ in r.axes)
    assert lengths == [300.0, 400.0, 500.0]      # not 520: envelope rule refused
    walls = sorted(w for _, _, w in r.axes if w is not None)
    assert walls == [1.65, 2.41]
    assert "milli" in r.units_note


def test_step_bend_arc(step_text):
    r = pt.parse_step_tubes(step_text)
    assert len(r.bends) == 1
    major, ang = r.bends[0]
    assert major == 80.0 and round(ang, 6) == 90.0


def test_frame_stats_counts_and_lengths(step_text):
    r = pt.parse_step_tubes(step_text)
    s = pt.frame_stats(r.axes, r.bends, cluster_tols_mm=(1, 50),
                       vertices=r.vertices)
    assert s["tubes"] == 3
    assert s["straight_m"] == pytest.approx(1.2)
    assert s["bend_arc_m"] == pytest.approx(80 * math.pi / 2 / 1000)
    assert s["nodes"][1.0]["nodes"] == 4         # 6 ends → 4 joints
    assert s["walls_mm"] == {1.65: 1, 2.41: 1, None: 1}
    assert s["vertices"]["in_tube_envelope"] >= 6


def test_mixed_bend_total():
    bends = [(64, 90)] * 2 + [(80, a) for a in
                              (43.6, 141.6, 90, 90, 90, 90, 80)]
    s = pt.frame_stats([], bends, cluster_tols_mm=())
    assert s["bends"] == 9 and 0.5 < s["bend_arc_m"] < 1.2


# ---- declarations ------------------------------------------------------------ #
def test_wheelbase_and_clearance():
    assert pt.wheelbase_check(950, -600)["margin_mm"] == 25
    assert pt.wheelbase_check(950, -680)["wheelbase_mm"] == 1630
    assert pt.static_clearance(-18.5, -50)["clearance_mm"] == 31.5


def test_tyre_table_reference_values():
    rows = {r["Fz_N"]: r for r in pt.tyre_table()}
    assert round(rows[550.0]["mu_peak"], 2) == 1.66
    assert round(rows[1100.0]["mu_peak"], 2) == 1.55
    assert round(rows[1650.0]["mu_peak"], 2) == 1.44
    assert rows[1100.0]["alpha_peak_deg"] == pytest.approx(8.1)
    assert round(rows[1100.0]["optimal_camber_deg"], 1) == -1.8


# ---- steering ------------------------------------------------------------------ #
def test_steering_torque_reference_case():
    s = pt.steering_torque(3.67, 228, 1262, 150, 1.55, 50, 6.0)
    assert round(s["trail_mm"], 1) == 14.6
    assert round(s["fy_outer_N"]) == 1956
    assert round(s["trail_torque_outer_Nm"], 1) == 28.6
    assert round(s["road_wheel_total_Nm"]) == 88
    got = {r["ratio"]: round(r["steering_wheel_Nm"], 1) for r in s["rows"]}
    assert got == {4.0: 22.0, 5.0: 17.6, 6.0: 14.7, 8.0: 11.0}


# ---- actuation, ride, roll ------------------------------------------------- #
def test_roll_stiffness_placeholder_error():
    f = pt.roll_stiffness_from_spring(295 * LB_IN, 0.60, 1200)
    assert round(f["roll_stiffness_Nm_deg"]) == 234
    assert round(f["placeholder_Nm_deg"]) == 649
    assert round(f["placeholder_error_pct"]) == 178
    r = pt.roll_stiffness_from_spring(349 * LB_IN, 0.62, 1180)
    assert round(r["roll_stiffness_Nm_deg"]) == 285
    assert round(r["placeholder_Nm_deg"]) == 743
    assert round(r["placeholder_error_pct"]) == 160


def test_ride_frequency_spread():
    f = [pt.ride_frequency_hz(295 * LB_IN, m, 60.1) for m in (0.585, 0.600, 0.596)]
    assert [round(x, 2) for x in f] == [2.73, 2.80, 2.78]


def test_actuation_summary_on_default_rocker():
    a = pt.actuation_summary(Hardpoints.default(), spring_rate_N_mm=50.0,
                             sprung_corner_mass_kg=60.0, damper_stroke_mm=57)
    assert a["ok"]
    assert 0.2 < a["mr_static"] < 1.5
    assert a["damper_static_mm"] == pytest.approx(180.0, abs=0.5)
    assert a["damper_travel_used_mm"] > 0
    assert 0 < a["attachment_fraction"] <= 1.2
    assert a["ride_hz_static"] > 0


def test_actuation_summary_refuses_without_rocker():
    hp = Hardpoints.default()
    hp.rocker_pivot = None
    assert pt.actuation_summary(hp)["ok"] is False


# ---- brackets and compliance ----------------------------------------------------------- #
def test_bracket_reference_cases():
    assert round(pt.bracket_fos(4799, 27.9, 30, 5)["fos"], 2) == 0.67
    assert round(pt.bracket_fos(4799, 27.9, 40, 6)["fos"], 2) == 1.29
    assert round(pt.bracket_fos(4799, 10.0, 30, 5)["fos"], 1) == 1.9


def test_axial_budget_reference_case():
    b = pt.axial_budget(4799, 485, reported_extension_mm=0.60)
    assert round(b["axial_strain_mm"], 2) == 0.27
    assert b["lash_mm"] == pytest.approx(0.05)
    f = pt.band_fractions({"camber": 0.116, "toe": 0.074},
                          {"camber": 0.30, "toe": 0.08})
    assert round(100 * f["camber"]) == 39
    assert 100 * f["toe"] == pytest.approx(92.5)    # reported as 93 %


def test_ball_joint_envelope():
    e = pt.ball_joint_envelope(Hardpoints.default(), 115.0)
    assert set(e) >= {"upper_above_wc_mm", "lower_below_wc_mm", "inside"}


# ---- design review ---------------------------------------------------------- #
def test_design_review_statuses():
    rows = {r["key"]: r for r in pt.design_review({
        "caster_deg": 5.0,            # inside 2..8
        "kpi_deg": 10.4,              # 0.4 over a 10-wide span -> watch
        "scrub_mm": 80.0,             # far outside -> fail
        "joints_in_rim": 0.0,         # boolean -> fail
        "worst_fos": 1.45,            # one-sided >= 1.5, small miss -> watch
    })}
    assert rows["caster_deg"]["status"] == "pass"
    assert rows["kpi_deg"]["status"] == "watch"
    assert rows["scrub_mm"]["status"] == "fail"
    assert rows["joints_in_rim"]["status"] == "fail"
    assert rows["worst_fos"]["status"] == "watch"
    assert rows["bump_steer_abs"]["status"] == "n/a"


def test_design_review_custom_limits():
    lim = [{"key": "x", "check": "x", "unit": "", "lo": 0.0, "hi": 1.0,
            "why": "", "fix": ""}]
    assert pt.design_review({"x": 0.5}, lim)[0]["status"] == "pass"
    assert pt.design_review({"x": 3.0}, lim)[0]["status"] == "fail"


def test_every_default_limit_explains_itself():
    for r in pt.DEFAULT_REVIEW_LIMITS:
        assert r["lo"] <= r["hi"]
        assert len(r["why"]) > 20 and len(r["fix"]) > 10


def test_declared_motion_ratio_curve_reference_values():
    f = pt.declared_mr_summary(0.585, 0.600, 0.596, 295 * LB_IN, 60.1)
    assert round(f["wheel_rate_spread_pct"], 1) == 5.2
    assert [round(f[k], 2) for k in ("ride_hz_droop", "ride_hz_static",
                                     "ride_hz_bump")] == [2.73, 2.80, 2.78]
    r = pt.declared_mr_summary(0.603, 0.620, 0.630, 349 * LB_IN, 66.1)
    assert r["rate_character"] == "rising"
    assert round(r["mr_spread_pct"], 1) == 4.5
    assert round(r["wheel_rate_spread_pct"], 1) == 9.2
    assert [round(r[k], 2) for k in ("ride_hz_droop", "ride_hz_static",
                                     "ride_hz_bump")] == [2.92, 3.00, 3.05]


def test_vehicle_level_lap_sensitivity_reference():
    """Linear grip, default aero, no corners, 55 % split, 1200/1180 tracks:
    26.126 s baseline, 0.127 s roll-split spread with its best at 54 %,
    0.315 s CG spread (240–320 mm)."""
    from suspension import inverse_genesis_fullcar as fc
    car = fc.DeclaredCar(tire_model="linear", cla=2.6, cda=1.1,
                         track_front_mm=1200, track_rear_mm=1180,
                         roll_stiffness_front=357.5,
                         roll_stiffness_rear=292.5)
    shares = [round(0.40 + 0.02 * i, 2) for i in range(16)]
    a = fc.lap_sensitivity(car, "front_roll_share", shares)
    b = fc.lap_sensitivity(car, "cg_height_mm", [240, 260, 280, 300, 320])
    assert round(a["baseline_s"], 3) == 26.126
    assert round(a["spread_s"], 3) == 0.127
    assert a["best_value"] == 0.54
    assert round(b["spread_s"], 3) == 0.315


# ---- a real CAD-kernel STEP file (OpenCascade export) ---------------------- #
_MIXED = os.path.join(os.path.dirname(__file__), "data",
                      "tube_frame_mixed.step")


def test_real_step_detects_sizes_lengths_walls_and_bend():
    """Four straight 25.4/19.05 mm tubes, a bent 19.05 mm tube (200 mm +
    64 mm × 90° + 200 mm) and a bolted plate that must not read as a tube."""
    r = pt.parse_step_tubes(open(_MIXED).read())
    assert r.tube_radii_mm == [9.525, 12.7]
    assert "millimetre" in r.units_note
    s = pt.frame_stats(r.axes, r.bends, (5,), r.vertices, od_mm=r.od_mm)
    assert s["tubes"] == 6
    assert s["straight_m"] == pytest.approx(2.0, abs=1e-6)
    assert s["sizes_mm"] == {(19.05, 0.889): 4, (25.4, 1.65): 1,
                             (25.4, 2.41): 1}
    assert len(r.bends) == 1
    assert r.bends[0][0] == pytest.approx(64.0)
    assert r.bends[0][1] == pytest.approx(90.0, abs=1e-6)
    assert s["vertices"]["outside_envelopes"] == 8      # the plate corners


def test_real_step_to_frame_graph_keeps_tube_sizes():
    r = pt.parse_step_tubes(open(_MIXED).read())
    g = pt.frame_graph_from_step(r, cluster_tol_mm=5.0)
    assert len(g.tubes) == 6
    ods = sorted(g.spec_of(t).od_mm for t in g.tubes)
    assert ods == [19.05] * 4 + [25.4] * 2


def test_step_single_radius_and_unit_scaling():
    txt = open(_MIXED).read()
    only_big = pt.parse_step_tubes(txt, 12.7)
    assert len(only_big.axes) == 2
    metres = txt.replace("SI_UNIT(.MILLI.,.METRE.)", "SI_UNIT($,.METRE.)")
    r = pt.parse_step_tubes(metres, [9525.0, 12700.0])   # radii now in mm
    assert r.unit_scale == 1000.0 and len(r.axes) == 6
