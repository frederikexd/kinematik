# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""suspension/fit_forecast.py against exact geometry and arithmetic: a box's volume, edge counts of open and
non-manifold meshes, principal extents of a rotated part, slot fit and clearance by hand, the CG shift and fastener
load, and every message and deduction the forecast can produce. Lengths mm, mass kg, force N."""
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension import fit_forecast as ff                                     # noqa: E402

CUBE_F = np.array([[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5], [0, 4, 5], [0, 5, 1],
                   [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]])


def _box(dims=(200.0, 150.0, 100.0), rot_z_deg=0.0):
    v = np.array([[x, y, z] for x in (0, 1) for y in (0, 1) for z in (0, 1)], float)
    v = (v - 0.5) * np.asarray(dims, float)
    a = np.radians(rot_z_deg)
    R = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
    return dict(verts=(v @ R.T).tolist(), faces=CUBE_F.tolist())


REP = ff.mesh_report(_box())


# --------------------------------------------------------------------------- #
#  Mesh health
# --------------------------------------------------------------------------- #
def test_a_closed_box_is_watertight_with_its_exact_volume_and_extents():
    r = REP
    assert r["watertight"] and r["open_edges"] == 0 and r["nonmanifold_edges"] == 0 and r["degenerate_tris"] == 0
    assert r["volume_mm3"] == pytest.approx(200 * 150 * 100) and r["solidity"] == pytest.approx(1.0)
    assert r["bbox_mm"] == [200.0, 150.0, 100.0] and r["pca_mm"] == pytest.approx([200.0, 150.0, 100.0])
    assert r["symmetry"] == pytest.approx(1.0) and r["triangles"] == 12
    assert r["diag_mm"] == pytest.approx(np.sqrt(200 ** 2 + 150 ** 2 + 100 ** 2))


def test_principal_extents_see_through_a_rotated_export():
    """A part exported 30 degrees off axis has a misleading axis-aligned box; its principal extents are still its
    true size."""
    r = ff.mesh_report(_box(rot_z_deg=30.0))
    assert r["pca_mm"] == pytest.approx([200.0, 150.0, 100.0], rel=1e-9)
    assert r["bbox_mm"][0] == pytest.approx(200 * np.cos(np.pi / 6) + 150 * np.sin(np.pi / 6))
    assert r["volume_mm3"] == pytest.approx(200 * 150 * 100)


def test_missing_and_extra_faces_are_counted_as_open_and_non_manifold_edges():
    p = _box()
    open_ = ff.mesh_report(dict(p, faces=CUBE_F[:-2].tolist()))     # one square side removed
    assert not open_["watertight"] and open_["open_edges"] == 4 and open_["nonmanifold_edges"] == 0
    extra = ff.mesh_report(dict(p, faces=CUBE_F.tolist() + [[0, 1, 3]]))
    assert not extra["watertight"] and extra["nonmanifold_edges"] == 3


def test_a_zero_area_sliver_is_counted():
    p = _box()
    verts = p["verts"] + [[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [20.0, 0.0, 0.0]]       # three collinear points
    r = ff.mesh_report(dict(verts=verts, faces=CUBE_F.tolist() + [[8, 9, 10]]))
    assert r["degenerate_tris"] == 1


def test_a_handed_part_scores_low_on_mirror_symmetry():
    """A right triangle in plan, extruded: its mirror about the mid-plane lands far from any of its own points."""
    v = np.array([[0, -50, 0], [400, -50, 0], [0, 50, 0], [0, -50, 20], [400, -50, 20], [0, 50, 20]], float)
    v -= (v.max(0) + v.min(0)) / 2
    f = [[0, 2, 1], [3, 4, 5], [0, 1, 4], [0, 4, 3], [1, 2, 5], [1, 5, 4], [2, 0, 3], [2, 3, 5]]
    r = ff.mesh_report(dict(verts=v.tolist(), faces=f))
    assert r["watertight"] and r["symmetry"] < 0.5


def test_a_degenerate_point_cloud_falls_back_to_its_box():
    r = ff.mesh_report(dict(verts=[[0, 0, 0], [30, 20, 10]], faces=np.empty((0, 3), int)))
    assert r["pca_mm"] == [30.0, 20.0, 10.0] and r["volume_mm3"] == 0.0


# --------------------------------------------------------------------------- #
#  Slot and neighbours
# --------------------------------------------------------------------------- #
def test_fit_matches_the_parts_sorted_size_to_the_slots():
    f = ff.slot_fit(REP, (120, 250, 180), (0, 0, 0))
    assert f["fits"] and f["fill"] == pytest.approx(150 / 180)                # the tightest axis
    assert f["placed_mm"] == pytest.approx([100.0, 200.0, 150.0])              # longest along the slot's longest (y), and so on
    assert f["ratios"] == pytest.approx([200 / 250, 150 / 180, 100 / 120]) and not f["unit_suspect"]
    tight = ff.slot_fit(REP, (190, 160, 110), (0, 0, 0))
    assert not tight["fits"] and tight["overhang_mm"] == pytest.approx([10.0, 0.0, 0.0], abs=1e-9)


@pytest.mark.parametrize("slot, suspect", [((40, 30, 20), True), ((1000, 750, 500), True), ((210, 160, 110), False)])
def test_a_part_five_times_off_scale_is_flagged_as_a_unit_mixup(slot, suspect):
    assert ff.slot_fit(REP, slot, (0, 0, 0))["unit_suspect"] is suspect


def test_clearance_overlap_and_containment_by_hand():
    """Part 200 x 150 x 100 at the origin. A box whose near face is 25 mm away is clearance 25; a small box at the
    centre overlaps by its own volume; a body that swallows the part is nesting, not interference."""
    nb = {"gap": dict(centre=[0, 75 + 25 + 50, 0], size=[100, 100, 100]),
          "core": dict(centre=[0, 0, 0], size=[100, 80, 60]),
          "pod": dict(centre=[0, 0, 0], size=[800, 600, 400]),
          "bad": dict(centre=[0, 0, 0]),
          "skip": dict(centre=[0, 0, 0], size=[10, 10, 10])}
    f = ff.slot_fit(REP, (250, 180, 120), (0, 0, 0), nb, exclude=("skip",))
    assert dict(f["tightest_neighbors"])["gap"] == pytest.approx(25.0)
    assert f["interferences"] == [("core", pytest.approx(100 * 80 * 60 / 1000.0))]
    assert f["contained_in"] == ["pod"]


def test_the_nearest_copy_of_a_mirrored_neighbour_sets_the_clearance():
    nb = {"Tire": [dict(centre=[0, 500, 0], size=[400, 200, 400]), dict(centre=[0, -450, 0], size=[400, 200, 400])],
          "Pod": [dict(centre=[0, 0, 0], size=[900, 700, 500]), dict(centre=[0, 0, 0], size=[900, 700, 500])]}
    f = ff.slot_fit(REP, (250, 180, 120), (0, 0, 0), nb)
    assert dict(f["tightest_neighbors"])["Tire"] == pytest.approx(450 - 75 - 100)
    assert f["contained_in"] == ["Pod"]


def test_the_slot_is_the_dummys_own_box_or_its_container_when_the_dummy_is_a_plate():
    inst = {"Damper": [dict(centre=[0, -300, 200], size=[60, 60, 200]), dict(centre=[0, 300, 200], size=[60, 60, 200])],
            "Radiator core": dict(centre=[100, 300, 200], size=[8, 90, 130]),
            "Sidepod": [dict(centre=[90, 300, 210], size=[520, 110, 160])],
            "Huge": [dict(centre=[0, 0, 0], size=[5000, 5000, 5000])],
            "Disc": [dict(centre=[0, 900, 0], size=[10, 200, 200])]}
    d = ff.resolve_slot("Damper", inst)
    assert d["centre"] == [0.0, 300.0, 200.0] and d["env"] == [60.0, 60.0, 200.0] and d["container"] is None
    r = ff.resolve_slot("Radiator core", inst)
    assert r["container"] == "Sidepod" and r["env"] == [520.0, 110.0, 160.0]       # the smallest that contains it
    lone = ff.resolve_slot("Disc", {"Disc": inst["Disc"]})
    assert lone["container"] is None and lone["env"] == [10.0, 200.0, 200.0]
    fb = ff.resolve_slot("Missing", inst, fallback_env=(100, 50, 25), fallback_centre=(1, 2, 3))
    assert fb == dict(env=[100.0, 50.0, 25.0], centre=[1.0, 2.0, 3.0], container=None)
    assert ff.resolve_slot(None, None) == dict(env=[1.0, 1.0, 1.0], centre=[0.0, 0.0, 0.0], container=None)


# --------------------------------------------------------------------------- #
#  SolidWorks messages and the score
# --------------------------------------------------------------------------- #
def _texts(rep, fit):
    return {(m["severity"], m["text"].split(":")[0].split(" ")[0]) for m in ff.solidworks_forecast(rep, fit)}


def test_a_clean_part_gets_a_single_all_clear():
    fit = ff.slot_fit(REP, (300, 250, 200), (0, 0, 0))
    msgs = ff.solidworks_forecast(REP, fit)
    assert len(msgs) == 1 and msgs[0]["severity"] == "ok" and "No red flags" in msgs[0]["text"]
    s = ff.realism_score(REP, fit)
    assert s == dict(score=100.0, grade="assembles cleanly", deductions=[])


def test_every_problem_produces_its_message_and_its_deduction():
    bad = dict(REP, watertight=False, open_edges=120, nonmanifold_edges=2, degenerate_tris=3, symmetry=0.2)
    nb = {"core": dict(centre=[0, 0, 0], size=[100, 80, 60]), "near": dict(centre=[0, 75 + 10 + 50, 0],
                                                                          size=[100, 100, 100]),
          "pod": dict(centre=[0, 0, 0], size=[900, 700, 500])}
    fit = ff.slot_fit(bad, (1000, 900, 800), (0, 0, 0), nb)                  # 5x too small for its space
    sev = [m["severity"] for m in ff.solidworks_forecast(bad, fit)]
    text = " ".join(m["text"] for m in ff.solidworks_forecast(bad, fit))
    assert "too small" in text and "Interference Detection" in text and "Only 10 mm" in text
    assert "SURFACE bodies" in text and "sliver" in text and "correct SIDE" in text and "nested by design" in text
    assert sev.count("stop") == 2 and "ok" in sev
    s = ff.realism_score(bad, fit)
    ic = 100 * 80 * 60 / 1000.0
    expect = 45 + min(25.0, 8.0 + ic / 25.0) + 5 + min(18.0, 6.0 + 120 / 60.0) + 3 + 6
    assert s["score"] == pytest.approx(max(0.0, 100.0 - expect)) and s["grade"] == "will fight you in SolidWorks"


def test_oversize_and_near_full_fits_are_told_apart():
    over = ff.slot_fit(REP, (190, 140, 90), (0, 0, 0))
    m = ff.solidworks_forecast(REP, over)
    assert m[0]["severity"] == "stop" and "Oversize" in m[0]["text"] and "10/10/10" in m[0]["text"]
    s = ff.realism_score(REP, over)
    assert s["deductions"][0][0] == round(min(35.0, 100.0 * (over["fill"] - 1.0) * 1.2 + 10.0))
    full = ff.slot_fit(REP, (205, 155, 105), (0, 0, 0))
    m = ff.solidworks_forecast(REP, full)
    assert m[0]["severity"] == "warn" and "Fills 98%" in m[0]["text"]
    s = ff.realism_score(REP, full)
    assert s["score"] == 94.0 and s["grade"] == "assembles cleanly"
    big = ff.slot_fit(REP, (40, 30, 20), (0, 0, 0))
    assert "~5× too big" in ff.solidworks_forecast(REP, big)[0]["text"]
    assert ff.realism_score(dict(REP, symmetry=0.2, degenerate_tris=1), full)["grade"] == "assembles with rework" \
        or ff.realism_score(dict(REP, symmetry=0.2, degenerate_tris=1), full)["score"] >= 65


# --------------------------------------------------------------------------- #
#  Performance
# --------------------------------------------------------------------------- #
def test_mass_cg_shift_and_fastener_load_by_hand():
    p = ff.performance_forecast(REP, material="Steel 4130", load_case="1.5 g braking", n_mounts=3,
                                base_mass_kg=220.0, base_cg_mm=(0, 0, 280), part_centre_mm=(1000, 0, 280))
    m = 0.003 * 7850.0
    assert p["mass_kg"] == pytest.approx(m) and "good estimate" in p["mass_note"]
    assert p["cg_shift_mm"] == pytest.approx([1000 * m / (220 + m), 0.0, 0.0])
    F = m * 9.81 * np.hypot(1.5, 1.0)
    assert p["inertial_force_n"] == pytest.approx(F) and p["per_mount_n"] == pytest.approx(F / 3)
    assert p["fastener_util"] == pytest.approx(F / 3 / 6000.0)
    assert p["lap_delta_s"] == pytest.approx(0.035 * m) and p["capacity_proxy"] is None


def test_unknown_inputs_fall_back_and_an_open_mesh_is_a_lower_bound():
    p = ff.performance_forecast(dict(REP, watertight=False), material="Unobtainium", load_case="?", n_mounts=0)
    assert p["density_kg_m3"] == 2760.0 and p["g_vector"] == [0.0, 1.5, 1.0] and p["n_mounts"] == 0
    assert p["per_mount_n"] == pytest.approx(p["inertial_force_n"]) and "lower bound" in p["mass_note"]
    z = ff.performance_forecast(dict(REP, volume_mm3=0.0), base_mass_kg=0.0)
    assert z["mass_kg"] == 0.0 and z["cg_shift_mm"] == [0.0, 0.0, -280.0]


def test_the_subsystem_proxies_are_their_stated_formulas():
    aero = ff.performance_forecast(REP, subsys="aero")["capacity_proxy"]
    area = 200 * 150 * 1e-6 * max(0.3, REP["solidity"])
    assert aero["value"] == f"{0.5 * 1.225 * 25 ** 2 * 0.9 * area:.0f} N"
    cool = ff.performance_forecast(REP, subsys="cooling")["capacity_proxy"]
    assert cool["value"] == f"{200 * 150 * 1e-6 * 90.0:.1f} kW"
    assert "APDL" in ff.performance_forecast(REP, subsys="chassis")["capacity_proxy"]["value"]


def test_the_full_forecast_runs_every_stage_on_the_slot_centre():
    out = ff.full_forecast(_box(), (300, 250, 200), (500, 0, 250), material="Steel 4130")
    assert set(out) == {"mesh", "fit", "solidworks", "performance", "realism"}
    assert out["performance"]["cg_shift_mm"][0] > 0 and out["realism"]["score"] == 100.0


@pytest.mark.filterwarnings("ignore::RuntimeWarning")
def test_the_modules_own_self_test_passes(capsys):
    import runpy
    runpy.run_module("suspension.fit_forecast", run_name="__main__")
    assert "self-test OK" in capsys.readouterr().out
