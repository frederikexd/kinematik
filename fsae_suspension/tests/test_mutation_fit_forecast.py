"""suspension/fit_forecast.py against hand-built meshes and boxes: volumes, extents, edge counts and symmetry from
their definitions, slot fits and box overlaps worked out here, and every forecast message, deduction and grade
checked word for word, including the thresholds at their exact edges."""
import math

import numpy as np
import pytest

from suspension import fit_forecast as ff

CUBE_F = [[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5], [0, 4, 5], [0, 5, 1],
          [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]]


def box(dx, dy, dz, centre=(0.0, 0.0, 0.0)):
    v = np.array([[x, y, z] for x in (0, 1) for y in (0, 1) for z in (0, 1)], float)
    v = (v - 0.5) * np.array([dx, dy, dz]) + np.asarray(centre, float)
    return dict(verts=v.tolist(), faces=[list(f) for f in CUBE_F])


def test_the_tables():
    assert ff.MATERIAL_DENSITY == {"Steel 4130": 7850.0, "Aluminium 6061/7075": 2760.0, "Titanium Ti-6Al-4V": 4430.0,
                                   "Carbon fibre laminate": 1550.0, "GFRP / fibreglass": 1900.0,
                                   "Nylon / printed": 1150.0}
    assert ff.LOAD_CASES == {"1.5 g cornering": (0.0, 1.5, 1.0), "1.5 g braking": (1.5, 0.0, 1.0),
                             "3 g vertical bump": (0.0, 0.0, 3.0), "2 g brake + 1 g bump": (2.0, 0.0, 2.0)}
    assert (ff._M6_ALLOW_N, ff._G) == (6000.0, 9.81)


# ------------------------------------------------------------------ mesh report
def test_a_closed_box():
    r = ff.mesh_report(box(200, 150, 100, centre=(10.0, 0.0, 5.0)))
    assert r["bbox_mm"] == [200.0, 150.0, 100.0] and r["pca_mm"] == pytest.approx([200.0, 150.0, 100.0])
    assert r["diag_mm"] == pytest.approx(math.sqrt(200 ** 2 + 150 ** 2 + 100 ** 2))
    assert (r["triangles"], r["watertight"], r["open_edges"], r["nonmanifold_edges"], r["degenerate_tris"]) == (
        12, True, 0, 0, 0)
    assert r["volume_mm3"] == pytest.approx(200 * 150 * 100) and r["solidity"] == pytest.approx(1.0)
    assert r["symmetry"] == 1.0
    assert set(r) == {"bbox_mm", "pca_mm", "diag_mm", "triangles", "watertight", "open_edges", "nonmanifold_edges",
                      "degenerate_tris", "volume_mm3", "solidity", "symmetry"}


def _sym(V, cap=1500):
    V = np.asarray(V, float)
    diag = float(np.linalg.norm(V.max(0) - V.min(0))) or 1.0
    idx = np.linspace(0, len(V) - 1, min(cap, len(V))).astype(int)
    S = V[idx]
    M = S * [1, -1, 1]
    nn = np.sqrt(((S[:, None] - M[None]) ** 2).sum(-1).min(1)).mean()
    return float(np.clip(1 - nn / (0.05 * diag), 0, 1))


def test_open_nonmanifold_degenerate_and_handed_meshes():
    b = box(100, 60, 40, centre=(0.0, 30.0, 0.0))                 # off the y=0 plane: handed
    b["faces"] = b["faces"][:-2]
    r = ff.mesh_report(b)
    assert (r["watertight"], r["open_edges"], r["nonmanifold_edges"]) == (False, 4, 0)
    assert r["symmetry"] == pytest.approx(_sym(b["verts"]), rel=1e-12) and r["symmetry"] < 0.5
    nm = box(100, 60, 40)
    nm["faces"] = nm["faces"] + [[0, 1, 3]]
    r = ff.mesh_report(nm)
    assert (r["watertight"], r["open_edges"], r["nonmanifold_edges"]) == (False, 0, 3)
    deg = box(100, 60, 40)
    deg["verts"].append([0.0, 0.0, 0.0])
    deg["faces"] = deg["faces"] + [[8, 8, 0], [0, 1, 1]]
    r = ff.mesh_report(deg)
    assert r["degenerate_tris"] == 2
    assert ff.mesh_report(deg, sample_cap=3)["symmetry"] == pytest.approx(_sym(deg["verts"], 3), rel=1e-12)
    half = box(100, 60, 40, centre=(0.0, 1.0, 0.0))
    assert ff.mesh_report(half)["symmetry"] == pytest.approx(_sym(half["verts"]), rel=1e-12)


def test_flat_and_degenerate_extents():
    flat = dict(verts=[[0, 0, 0], [100, 0, 0], [0, 50, 0], [100, 50, 0]], faces=[[0, 1, 3], [0, 3, 2]])
    r = ff.mesh_report(flat)
    assert r["volume_mm3"] == 0.0 and r["solidity"] == 0.0 and r["bbox_mm"] == [100.0, 50.0, 0.0]
    dot = dict(verts=[[5, 5, 5]], faces=[[0, 0, 0]])
    r = ff.mesh_report(dot)
    assert r["diag_mm"] == 1.0 and r["degenerate_tris"] == 1 and r["pca_mm"] == [0.0, 0.0, 0.0]
    two = dict(verts=[[0, 0, 0], [30, 40, 0]], faces=[[0, 1, 1]])
    assert ff.mesh_report(two)["pca_mm"] == [40.0, 30.0, 0.0]               # fewer than 3 points: the bbox
    tri = dict(verts=[[0, 0, 0], [30, 40, 0], [-40, 30, 0]], faces=[[0, 1, 2]])
    p = ff.mesh_report(tri)["pca_mm"]
    C = np.array(tri["verts"], float) - np.mean(tri["verts"], axis=0)
    _, _, vt = np.linalg.svd(C, full_matrices=False)
    P = C @ vt.T
    assert p == pytest.approx(sorted((P.max(0) - P.min(0)).tolist(), reverse=True), abs=1e-9)
    assert p[0] != pytest.approx(70.0)                                      # not the bbox
    big = dict(verts=[[0, 0, 0], [1000, 0, 0], [0, 1000, 0], [0, 0, 1000]], faces=[[0, 1, 2], [0, 2, 3], [0, 3, 1],
                                                                                   [1, 3, 2]])
    r = ff.mesh_report(big)
    assert r["volume_mm3"] == pytest.approx(1e9 / 6) and r["solidity"] == pytest.approx(1 / 6)


def test_a_rotated_box_has_pca_extents():
    b = box(300, 100, 50)
    c, s = math.cos(0.5), math.sin(0.5)
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    b["verts"] = (np.array(b["verts"]) @ R.T).tolist()
    r = ff.mesh_report(b)
    assert r["pca_mm"] == pytest.approx([300.0, 100.0, 50.0], rel=1e-9)
    assert r["bbox_mm"][0] > 300.0


# ------------------------------------------------------------------ slot resolution
def test_resolve_slot():
    out = ff.resolve_slot(None, None)
    assert out == dict(env=[1.0, 1.0, 1.0], centre=[0.0, 0.0, 0.0], container=None)
    out = ff.resolve_slot("x", {}, fallback_env=(10, 20, 30), fallback_centre=(1, 2, 3))
    assert out == dict(env=[10.0, 20.0, 30.0], centre=[1.0, 2.0, 3.0], container=None)
    inst = {"Bin": [dict(centre=[0, -100, 0], size=[50, 50, 50]), dict(centre=[0, 100, 0], size=[40, 40, 40]),
                    dict(centre=[0, 120, 0], size=[30, 30, 30])]}
    assert ff.resolve_slot("Bin", inst) == dict(env=[40.0, 40.0, 40.0], centre=[0.0, 100.0, 0.0], container=None)
    one = {"Bin": dict(centre=[5, -1, 0], size=[60, 30, 30])}
    assert ff.resolve_slot("Bin", one)["env"] == [60.0, 30.0, 30.0]
    zero_y = {"Bin": [dict(centre=[0, 0, 0], size=[30, 30, 30]), dict(centre=[0, -5, 0], size=[90, 90, 90])]}
    assert ff.resolve_slot("Bin", zero_y)["centre"] == [0.0, 0.0, 0.0]


def test_a_plate_takes_the_smallest_containing_body():
    inst = {"Core": [dict(centre=[100, 300, 200], size=[8, 90, 130])],
            "Sidepod": [dict(centre=[90, 300, 210], size=[520, 110, 160])],
            "Body": dict(centre=[0, 0, 0], size=[4000, 2000, 2000]),
            "Tiny": [dict(centre=[100, 300, 200], size=[10, 10, 10])],
            "Far": [dict(centre=[5000, 0, 0], size=[9000, 9000, 9000])],
            "Bad": [dict(centre=[100, 300, 200])]}
    out = ff.resolve_slot("Core", inst)
    assert out == dict(env=[520.0, 110.0, 160.0], centre=[90.0, 300.0, 210.0], container="Sidepod")
    edge = {"Core": [dict(centre=[0, 0, 0], size=[24.9, 90, 130])],
            "Shell": [dict(centre=[61.0, 0, 0], size=[120, 200, 200])]}
    assert ff.resolve_slot("Core", edge)["container"] == "Shell"            # 1 mm containment slack
    edge["Shell"][0]["centre"] = [61.01, 0, 0]
    assert ff.resolve_slot("Core", edge)["container"] is None
    thick = {"Core": [dict(centre=[0, 0, 0], size=[25.0, 90, 130])],
             "Shell": [dict(centre=[0, 0, 0], size=[120, 200, 200])]}
    assert ff.resolve_slot("Core", thick)["container"] is None
    self_only = {"Core": [dict(centre=[0, 0, 0], size=[8, 90, 130]), dict(centre=[0, 0, 0], size=[9, 99, 199])]}
    assert ff.resolve_slot("Core", self_only)["container"] is None
    same = {"Core": [dict(centre=[0, 0, 0], size=[8, 100, 100])],
            "Equal": [dict(centre=[0, 0, 0], size=[8, 100, 100])]}
    assert ff.resolve_slot("Core", same)["container"] is None             # not strictly bigger


# ------------------------------------------------------------------ slot fit
REP = dict(pca_mm=[100.0, 60.0, 40.0], watertight=True, open_edges=0, nonmanifold_edges=0, degenerate_tris=0,
           symmetry=1.0, volume_mm3=240000.0, bbox_mm=[100.0, 60.0, 40.0], solidity=1.0)


def test_the_fit_against_the_slot():
    f = ff.slot_fit(REP, (50, 200, 100), (0, 0, 0))
    assert f["placed_mm"] == [40.0, 100.0, 60.0] and f["slot_mm"] == [50.0, 200.0, 100.0]
    assert f["ratios"] == [0.5, 0.6, 0.8] and f["fill"] == 0.8 and f["fits"] is True
    assert f["overhang_mm"] == [0.0, 0.0, 0.0] and f["scale_ratio"] == 0.5 and f["unit_suspect"] is False
    assert (f["tightest_neighbors"], f["interferences"], f["contained_in"]) == ([], [], [])
    exact = ff.slot_fit(REP, (100, 60, 40), (0, 0, 0))
    assert exact["fits"] is True and exact["fill"] == 1.0
    over = ff.slot_fit(REP, (90, 70, 30), (0, 0, 0))
    assert over["fits"] is False and over["overhang_mm"] == [10.0, 0.0, 10.0]
    assert over["ratios"] == pytest.approx([100 / 90, 60 / 70, 40 / 30])
    huge = ff.slot_fit(REP, (5, 5, 5), (0, 0, 0))
    assert huge["fill"] == 10.0 and huge["unit_suspect"] is True
    flat = ff.slot_fit(REP, (100, 60, 0), (0, 0, 0))
    assert flat["ratios"][2] == pytest.approx(40 / 1e-6)
    assert ff.slot_fit(REP, (20, 12, 8), (0, 0, 0))["scale_ratio"] == 5.0
    assert ff.slot_fit(REP, (20, 12, 8), (0, 0, 0))["unit_suspect"] is True
    assert ff.slot_fit(REP, (20.01, 12, 8), (0, 0, 0))["unit_suspect"] is False
    assert ff.slot_fit(REP, (500, 300, 200), (0, 0, 0))["unit_suspect"] is True       # 0.2 exactly
    assert ff.slot_fit(REP, (499, 300, 200), (0, 0, 0))["unit_suspect"] is False
    zero = dict(REP, pca_mm=[0.0, 0.0, 0.0])
    assert ff.slot_fit(zero, (10, 10, 10), (0, 0, 0))["scale_ratio"] == 1e-9


def test_neighbours_clearances_overlaps_and_nesting():
    nb = {"Far": dict(centre=[0, 300, 0], size=[10, 10, 10]),
          "Touch": dict(centre=[0, 0, 80], size=[100, 100, 100]),
          "Twins": [dict(centre=[200, 0, 0], size=[20, 20, 20]), dict(centre=[0, -100, 0], size=[20, 20, 20])],
          "Hit": dict(centre=[30, 0, 0], size=[60, 40, 40]),
          "Pod": [dict(centre=[0, 0, 0], size=[300, 300, 300]), dict(centre=[0, 0, 0], size=[310, 310, 310])],
          "Skip": dict(centre=[0, 0, 0], size=[5, 5, 5]),
          "Broken": dict(size=[1, 1, 1])}
    f = ff.slot_fit(REP, (40, 100, 60), (0, 0, 0), nb, exclude=("Skip",))
    assert f["placed_mm"] == [40.0, 100.0, 60.0]
    assert f["tightest_neighbors"] == [("Touch", 0.0), ("Twins", 40.0), ("Far", 245.0)]
    assert f["interferences"] == [("Hit", pytest.approx(20 * 40 * 40 / 1000.0))]
    assert f["contained_in"] == ["Pod"]
    many = {f"N{i}": dict(centre=[0, 100 + i, 0], size=[10, 10, 10]) for i in range(7)}
    g = ff.slot_fit(REP, (40, 100, 60), (0, 0, 0), many)
    assert [n for n, _ in g["tightest_neighbors"]] == ["N0", "N1", "N2", "N3", "N4"]
    two = {"A": dict(centre=[10, 0, 0], size=[10, 10, 10]), "B": dict(centre=[0, 0, 0], size=[30, 30, 30])}
    assert [n for n, _ in ff.slot_fit(REP, (40, 100, 60), (0, 0, 0), two)["interferences"]] == ["B", "A"]


def test_the_containment_thresholds_at_their_edges():
    rep = dict(REP, pca_mm=[10.0, 10.0, 10.0])
    # part box 10^3 = 1000; neighbour 3000 is NOT more than 3x
    eq = {"N": dict(centre=[0, 0, 0], size=[30, 10, 10])}
    f = ff.slot_fit(rep, (10, 10, 10), (0, 0, 0), eq)
    assert f["contained_in"] == [] and f["interferences"] == [("N", pytest.approx(1.0))]
    big = {"N": dict(centre=[0, 0, 0], size=[30.01, 10, 10])}
    assert ff.slot_fit(rep, (10, 10, 10), (0, 0, 0), big)["contained_in"] == ["N"]
    # overlap of exactly 60% of the part box, in a body far larger than it
    sixty = {"N": dict(centre=[999, 0, 0], size=[2000, 400, 400])}
    f = ff.slot_fit(rep, (10, 10, 10), (0, 0, 0), sixty)
    assert f["contained_in"] == ["N"]
    under = {"N": dict(centre=[999.01, 0, 0], size=[2000, 400, 400])}
    f = ff.slot_fit(rep, (10, 10, 10), (0, 0, 0), under)
    assert f["contained_in"] == [] and f["interferences"][0][0] == "N"


# ------------------------------------------------------------------ the SolidWorks forecast
def _fit(**kw):
    base = dict(fits=True, fill=0.5, overhang_mm=[0.0, 0.0, 0.0], interferences=[], tightest_neighbors=[],
                contained_in=[], unit_suspect=False, scale_ratio=0.5)
    base.update(kw)
    return base


def test_a_clean_part_gets_one_ok():
    assert ff.solidworks_forecast(REP, _fit()) == [dict(severity="ok", text=(
        "No red flags: closed solid, fits its space with margin, no predicted interference. Expect clean mates on "
        "the first insert."))]


def test_every_forecast_message():
    rep = dict(REP, watertight=False, open_edges=6, nonmanifold_edges=2, degenerate_tris=3, symmetry=0.4249)
    fit = _fit(unit_suspect=True, scale_ratio=25.4, fits=False, overhang_mm=[12.4, 3.6, 0.0],
               interferences=[("A", 12.6), ("B", 2.0), ("C", 1.0), ("D", 0.5), ("E", 0.1)],
               tightest_neighbors=[("P", 0.0), ("Q", 14.4), ("R", 15.0), ("S", 3.0)],
               contained_in=["Pod", "Shell", "Body"])
    out = ff.solidworks_forecast(rep, fit)
    want = [
        ("stop", "Scale is off: ~25× too big for its space in every orientation. That magnitude is almost always a "
                 "unit-conversion mixup (mm / cm / inch / m) in the export — fix the units before reading anything "
                 "else in this forecast."),
        ("stop", "Oversize in every orientation: overhangs its space by up to 12 mm (per sorted axis: 12/4/0 mm). "
                 "Mates will solve, but Interference Detection will flag it against whatever borders that space — "
                 "the part needs a redesign or the neighbours need to move."),
        ("stop", "Interference Detection: expect a hit against “A” (~13 cm³ of box-level overlap at the slot "
                 "centre)."),
        ("stop", "Interference Detection: expect a hit against “B” (~2 cm³ of box-level overlap at the slot "
                 "centre)."),
        ("stop", "Interference Detection: expect a hit against “C” (~1 cm³ of box-level overlap at the slot "
                 "centre)."),
        ("stop", "Interference Detection: expect a hit against “D” (~0 cm³ of box-level overlap at the slot "
                 "centre)."),
        ("warn", "Only 0 mm to “P” — inside typical FSAE minimum clearance. Clearance Verification will flag it; "
                 "decide the gap deliberately (heat shield? service access?)."),
        ("warn", "Only 14 mm to “Q” — inside typical FSAE minimum clearance. Clearance Verification will flag it; "
                 "decide the gap deliberately (heat shield? service access?)."),
        ("warn", "Not a closed solid: 6 open and 2 non-manifold edges. SolidWorks will import this as SURFACE "
                 "bodies — Knit + Thicken before mating, and treat the mass estimate below as a lower bound."),
        ("warn", "3 sliver/zero-area facets — usually a too-coarse export tolerance; harmless visually, but "
                 "re-export if the STEP won't knit."),
        ("warn", "Strongly handed part (y-mirror symmetry 42%). Confirm it's the correct SIDE — mirrored handoffs "
                 "are the classic silent assembly killer; the left and right versions mate without complaint and "
                 "are both wrong on one corner."),
        ("ok", "Sits inside “Pod” — nested by design (Interference Detection will report the overlap; mark that "
               "pair as ignored/coincident in the assembly rather than chasing it)."),
        ("ok", "Sits inside “Shell” — nested by design (Interference Detection will report the overlap; mark that "
               "pair as ignored/coincident in the assembly rather than chasing it)."),
    ]
    assert [(d["severity"], d["text"]) for d in out] == want


def test_the_forecast_edges():
    small = ff.solidworks_forecast(REP, _fit(unit_suspect=True, scale_ratio=0.04))
    assert small[0]["text"].startswith("Scale is off: ~25× too small for its space")
    unknown = ff.solidworks_forecast(REP, {"unit_suspect": True, "fits": True, "fill": 0.5})
    assert unknown[0]["text"].startswith("Scale is off: ~1000000000× too small")
    one = ff.solidworks_forecast(REP, _fit(unit_suspect=True, scale_ratio=1.0))
    assert one[0]["text"].startswith("Scale is off: ~1× too big")
    full = ff.solidworks_forecast(REP, _fit(fill=0.9451))
    assert full == [dict(severity="warn", text=(
        "Fills 95% of its space in the best orientation — assembles, but with almost no tolerance stack margin. "
        "One weld distortion or bracket shim and it touches."))]
    assert ff.solidworks_forecast(REP, _fit(fill=0.94))[0]["severity"] == "ok"
    assert ff.solidworks_forecast(dict(REP, symmetry=0.5), _fit())[0]["severity"] == "ok"
    bare = {"fits": True, "fill": 0.1}
    assert ff.solidworks_forecast(REP, bare)[0]["severity"] == "ok"


# ------------------------------------------------------------------ the performance forecast
def test_the_performance_numbers():
    p = ff.performance_forecast(REP)
    m = 240000e-9 * 2760.0
    assert p["material"] == "Aluminium 6061/7075" and p["density_kg_m3"] == 2760.0
    assert p["mass_kg"] == pytest.approx(m, rel=1e-12)
    assert p["mass_note"] == "enclosed volume × density (closed solid — good estimate)"
    assert p["cg_shift_mm"] == pytest.approx([0.0, 0.0, (280 * 220 + 250 * m) / (220 + m) - 280], rel=1e-12)
    assert p["load_case"] == "1.5 g cornering" and p["g_vector"] == [0.0, 1.5, 1.0]
    F = m * 9.81 * math.sqrt(1.5 ** 2 + 1.0)
    assert p["inertial_force_n"] == pytest.approx(F, rel=1e-12) and p["n_mounts"] == 4
    assert p["per_mount_n"] == pytest.approx(F / 4, rel=1e-12)
    assert p["fastener_util"] == pytest.approx(F / 4 / 6000.0, rel=1e-12)
    assert p["fastener_note"] == "vs one M6 8.8 in single shear @ 6 kN design allowable"
    assert p["lap_delta_s"] == pytest.approx(m * 0.035, rel=1e-12)
    assert p["lap_note"] == "~0.035 s/lap per kg — FSAE autocross rule of thumb, not a sim"
    assert p["capacity_proxy"] is None
    q = ff.performance_forecast(dict(REP, watertight=False), material="Unobtainium", load_case="??", n_mounts=0,
                                base_mass_kg=100.0, base_cg_mm=(10.0, 20.0, 30.0), part_centre_mm=(110.0, 20.0, 0.0))
    m2 = 240000e-9 * 2760.0
    assert q["density_kg_m3"] == 2760.0 and q["g_vector"] == [0.0, 1.5, 1.0] and q["n_mounts"] == 0
    assert q["per_mount_n"] == pytest.approx(q["inertial_force_n"], rel=1e-12)
    assert q["cg_shift_mm"] == pytest.approx([100 * m2 / (100 + m2), 0.0, -30 * m2 / (100 + m2)], rel=1e-12)
    assert q["mass_note"] == ("enclosed volume × density on an OPEN mesh — lower bound; knit the model or weigh the "
                              "part for a real number")
    s = ff.performance_forecast(REP, material="Steel 4130", load_case="2 g brake + 1 g bump", n_mounts=3.7)
    assert s["density_kg_m3"] == 7850.0 and s["n_mounts"] == 3
    assert s["inertial_force_n"] == pytest.approx(240000e-9 * 7850 * 9.81 * math.sqrt(8.0), rel=1e-12)
    assert s["per_mount_n"] == pytest.approx(s["inertial_force_n"] / 3, rel=1e-12)
    void = ff.performance_forecast(dict(REP, volume_mm3=0.0), base_mass_kg=0.0, base_cg_mm=(1.0, 2.0, 3.0))
    assert void["cg_shift_mm"] == [-1.0, -2.0, -3.0]


def test_the_capacity_proxies():
    rep = dict(REP, bbox_mm=[60.0, 400.0, 300.0], solidity=0.2)
    for s in ("aerodynamics", "aero", "bodywork"):
        p = ff.performance_forecast(rep, subsys=s)["capacity_proxy"]
        area = 400 * 300 * 1e-6 * 0.3
        assert p == dict(label="Drag at 25 m/s (flat-plate Cd 0.9 on projected area)",
                         value=f"{0.5 * 1.225 * 625 * 0.9 * area:.0f} N",
                         note="direction-only surrogate — panel method / tunnel for real numbers")
    solid = ff.performance_forecast(dict(rep, solidity=0.8), subsys="aero")["capacity_proxy"]
    assert solid["value"] == f"{0.5 * 1.225 * 625 * 0.9 * 400 * 300 * 1e-6 * 0.8:.0f} N"
    c = ff.performance_forecast(rep, subsys="cooling")["capacity_proxy"]
    assert c == dict(label="Heat rejection capacity (core face × 90 kW/m²)", value=f"{0.12 * 90:.1f} kW",
                     note="rule-of-thumb core flux — duct design changes this a lot")
    ch = ff.performance_forecast(rep, subsys="chassis")["capacity_proxy"]
    assert ch == dict(label="Torsional stiffness", value="run the Frame Planner audit + APDL deck",
                      note="a mesh bbox can't predict frame stiffness — the BEAM188 torsion deck in Integration "
                           "exports it properly")
    assert ff.performance_forecast(rep, subsys="brakes")["capacity_proxy"] is None


# ------------------------------------------------------------------ realism
def test_a_clean_part_scores_100():
    assert ff.realism_score(REP, _fit()) == dict(score=100.0, grade="assembles cleanly", deductions=[])


def test_every_deduction():
    rep = dict(REP, watertight=False, open_edges=300, degenerate_tris=2, symmetry=0.3)
    fit = _fit(unit_suspect=True, fits=False, fill=1.1, interferences=[("A", 100.0), ("B", 50.0)],
               tightest_neighbors=[("P", 0.0), ("Q", 14.9), ("R", 15.0), ("S", -1.0)])
    r = ff.realism_score(rep, fit)
    assert r["deductions"] == [(45, "probable unit (inch/mm) mixup"),
                               (22, "oversize for its space in every orientation"),
                               (14, "predicted interference with neighbours"),
                               (5, "2 neighbour(s) inside 15 mm clearance"),
                               (11, "open / non-manifold mesh — surface-body import likely"),
                               (3, "sliver facets in the tessellation"),
                               (6, "strongly handed — mirrored-part risk on handoff")]
    assert r["score"] == 0.0 and r["grade"] == "will fight you in SolidWorks"
    capped = ff.realism_score(dict(REP, watertight=False, open_edges=6000),
                              _fit(fits=False, fill=5.0, interferences=[("A", 9000.0)]))
    assert capped["deductions"] == [(35, "oversize for its space in every orientation"),
                                    (25, "predicted interference with neighbours"),
                                    (18, "open / non-manifold mesh — surface-body import likely")]
    assert capped["score"] == 22.0


def test_the_realism_grades_and_edges():
    tight = ff.realism_score(REP, _fit(fill=0.95))
    assert tight == dict(score=94.0, grade="assembles cleanly", deductions=[(6, "fits with <6% margin — "
                                                                                "tolerance-stack risk")])
    assert ff.realism_score(REP, _fit(fill=0.94))["deductions"] == []
    edge85 = ff.realism_score(dict(REP, symmetry=0.4, degenerate_tris=1),
                              _fit(fill=0.95, tightest_neighbors=[("P", 1.0)]))
    assert edge85["score"] == 80.0 and edge85["grade"] == "assembles with rework"
    at85 = ff.realism_score(dict(REP, degenerate_tris=1), _fit(fill=0.95, interferences=[("A", 0.0)],
                                                               tightest_neighbors=[("P", 3.0)]))
    assert at85["score"] == 86.0
    eighty_five = ff.realism_score(dict(REP, symmetry=0.4, degenerate_tris=1), _fit(tightest_neighbors=[("P", 3.0)]))
    assert eighty_five["score"] == 86.0
    g = ff.realism_score(dict(REP, symmetry=0.4), _fit(fill=0.95, interferences=[("A", 25.0)]))
    assert g["score"] == 100 - 6 - 9 - 6 and g["grade"] == "assembles with rework"
    exact85 = ff.realism_score(dict(REP, watertight=False, open_edges=180), _fit(fill=0.95))
    assert exact85["score"] == 85.0 and exact85["grade"] == "assembles cleanly"
    exact65 = ff.realism_score(dict(REP, watertight=False, open_edges=180),
                               _fit(fill=0.95, interferences=[("A", 300.0)]))
    assert exact65["score"] == 65.0 and exact65["grade"] == "assembles with rework"
    low = ff.realism_score(dict(REP, watertight=False, open_edges=240),
                           _fit(fill=0.95, interferences=[("A", 300.0)]))
    assert low["score"] == 64.0 and low["grade"] == "will fight you in SolidWorks"
    assert ff.realism_score(dict(REP, symmetry=0.5), {"fits": True, "fill": 0.2})["score"] == 100.0
    fit_one = ff.realism_score(REP, _fit(fits=False, fill=1.0))
    assert fit_one["deductions"] == [(10, "oversize for its space in every orientation")]


# ------------------------------------------------------------------ all at once
def test_the_full_forecast():
    pay = box(200, 150, 100)
    nb = {"Radiator core": dict(centre=[0, 0, 250], size=[100, 80, 60])}
    out = ff.full_forecast(pay, (250, 180, 120), (0, 0, 250), nb, exclude=("Nothing",), material="Steel 4130",
                           n_mounts=2)
    rep = ff.mesh_report(pay)
    fit = ff.slot_fit(rep, (250, 180, 120), (0, 0, 250), nb)
    assert out["mesh"] == rep and out["fit"] == fit
    assert out["solidworks"] == ff.solidworks_forecast(rep, fit)
    assert out["performance"] == ff.performance_forecast(rep, part_centre_mm=(0, 0, 250), material="Steel 4130",
                                                         n_mounts=2)
    assert out["realism"] == ff.realism_score(rep, fit)
    ex = ff.full_forecast(pay, (250, 180, 120), (0, 0, 250), nb, exclude=("Radiator core",))
    assert ex["fit"]["interferences"] == []


def test_more_edges():
    from tests._golden import check
    res = {}
    res["box_odd"] = ff.mesh_report(box(30.0, 70.0, 110.0, centre=(5.0, -3.0, 2.0)))
    nb = {"Skip": dict(centre=[0, 0, 0], size=[5, 5, 5]),
          "Broken": [dict(size=[1, 1, 1]), dict(centre=[0, 400, 0], size=[10, 10, 10])],
          "Pair": [dict(centre=[0, 150, 0], size=[20, 20, 20]), dict(centre=[0, 300, 0], size=[20, 20, 20])],
          "Graze": dict(centre=[0, 0, 79.5], size=[100, 100, 100])}
    res["fit_off"] = ff.slot_fit(REP, (40, 100, 60), (10.0, 20.0, -5.0), nb, exclude=("Skip",))
    res["fit_origin"] = ff.slot_fit(REP, (40, 100, 60), (0.0, 0.0, 0.0), nb, exclude=("Skip",))
    res["iter"] = list(ff._iter_instances(nb))
    res["slot_none"] = [ff.resolve_slot(None, {"": dict(centre=[0, 0, 0], size=[1, 2, 3])}),
                        ff.resolve_slot("", {"": dict(centre=[0, 0, 0], size=[1, 2, 3])})]
    res["perf_light"] = ff.performance_forecast(REP, base_mass_kg=0.2, base_cg_mm=(10.0, 20.0, 30.0),
                                                part_centre_mm=(110.0, 20.0, 0.0))
    res["sw_one_degen"] = ff.solidworks_forecast(dict(REP, degenerate_tris=1), _fit())
    res["realism"] = {f"{f:.4f}": ff.realism_score(REP, _fit(fill=f)) for f in (1.0375, 1.0125, 1.0625, 1.104,
                                                                             1.1875)}
    res["realism_ic1"] = ff.realism_score(REP, _fit(interferences=[("A", 1.0)]))
    res["realism_ic_half"] = ff.realism_score(REP, _fit(interferences=[("A", 0.5)]))
    check("fit_forecast_edges", res)
