# ============================================================================
#  KinematiK — tests for the simulation handoff layer
# ============================================================================
"""What these guard, in order of how much it would hurt to get wrong:

  1. The UNIT CONTRACT. Handoff geometry is millimetres, always, whatever the
     UI is displaying. The old exporter scaled to inches with the display
     setting, which is invisible to a human reading a drawing and a silent
     25.4x error to anything automated.
  2. DXF WELL-FORMEDNESS. Balanced sections, a declared APPID, and XDATA that
     actually parses — because a malformed DXF fails at the far end of the
     handoff, in someone else's tool, hours later.
  3. ROLE CLASSIFICATION. A bolt hole must come out a bolt hole and a bore must
     not, or a downstream bolt-automation step pretensions a motor register.
  4. MESH SIZING. Global size follows material thinness; hole diameter drives
     only local refinement. Conflating them is how a 3 mm hole silently
     dictates a 200k-element mesh on a plate that needed 5 mm elements.
  5. HONESTY. A missing load stays null and marked required. Starter values
     stay separated from measured ones. Neither may quietly become a number.
"""
import io
import json
import math
import zipfile

import pytest

from suspension import sim_handoff as sh


# --------------------------------------------------------------------------- #
#  Fixtures shaped exactly like the app's own candidate dicts
# --------------------------------------------------------------------------- #
def _rect(w, h, x0=0.0, y0=0.0):
    return [(x0, y0), (x0 + w, y0), (x0 + w, y0 + h), (x0, y0 + h)]


def _bolt_circle(pcd, n, d):
    r = pcd / 2.0
    return [{"c": (r * math.cos(2 * math.pi * i / n),
                   r * math.sin(2 * math.pi * i / n)), "r": d / 2.0}
            for i in range(n)]


def _plate_candidate(pcd=92.0, n=3, hole=8.4, plate=120.0):
    return {
        "label": f"Upright mount plate blank — {n} bolts on {pcd:g} mm PCD",
        "meta": {"Bolt count": n, "PCD (mm)": pcd, "Hole ⌀ (mm)": hole,
                 "Plate size (mm)": plate, "Peak load (N)": 4200.0,
                 "Source": "hardpoints"},
        "dxf_kwargs": {
            "polylines": [{"pts": _rect(plate, plate, -plate / 2, -plate / 2),
                           "closed": True}],
            "circles": _bolt_circle(pcd, n, hole)},
        "notes": ["PCD is REAL", "Outline is a STARTER blank"],
    }


def _flange_candidate():
    return {
        "label": "Motor mount flange — 4 bolts on 150 mm PCD",
        "meta": {"Bolt count": 4, "PCD (mm)": 150.0, "Bore ⌀ (mm)": 75.0,
                 "Peak torque (N·m)": 230.0, "Source": "powertrain tab"},
        "dxf_kwargs": {
            "polylines": [{"pts": _rect(190, 190, -95, -95), "closed": True}],
            "circles": [{"c": (0, 0), "r": 37.5}] + _bolt_circle(150.0, 4, 9.0)},
    }


def _box_candidate():
    return {
        "label": "Accumulator segment box — 320×140 mm",
        "meta": {"Outer W (mm)": 320.0, "Outer H (mm)": 140.0, "Wall (mm)": 8.0,
                 "Config": "96s3p", "Source": "accumulator tab"},
        "dxf_kwargs": {"polylines": [
            {"pts": _rect(320, 140), "closed": True},
            {"pts": _rect(304, 124, 8, 8), "closed": True}]},
    }


def _dxf_pairs(dxf_bytes):
    """Walk a DXF into (group_code, value) pairs — the whole format is pairs."""
    lines = dxf_bytes.decode("ascii").split("\n")
    assert len(lines) % 2 == 0, "DXF must be an even number of lines"
    return [(lines[i].strip(), lines[i + 1]) for i in range(0, len(lines), 2)]


# --------------------------------------------------------------------------- #
#  1. The unit contract
# --------------------------------------------------------------------------- #
def test_geometry_is_millimetres_regardless_of_display_units():
    """The plate is 120 mm across. It must be 120 in the DXF, not 4.72."""
    sec = sh.section_from_candidate("suspension", _plate_candidate(plate=120.0))
    x0, y0, x1, y1 = sec.bbox
    assert x1 - x0 == pytest.approx(120.0)
    assert y1 - y0 == pytest.approx(120.0)

    dxf = sh.section_dxf(sec)
    pairs = _dxf_pairs(dxf)
    insunits = [v for (c, v) in pairs if c == "70"]
    # $INSUNITS is the first code-70 in the header and must say 4 = millimetres
    assert insunits[0].strip() == "4"

    man = sh.handoff_manifest(sec)
    assert man["units"]["length"] == "mm"
    assert man["units"]["canonical"] is True
    assert man["geometry"]["bbox_mm"][2] == pytest.approx(60.0)


def test_manifest_declares_every_unit_it_uses():
    sec = sh.section_from_candidate("suspension", _plate_candidate())
    man = sh.handoff_manifest(sec)
    for key in ("length", "force", "stress", "moment", "mass", "temperature",
                "angle"):
        assert man["units"][key]


# --------------------------------------------------------------------------- #
#  2. DXF well-formedness
# --------------------------------------------------------------------------- #
def test_dxf_sections_balance_and_terminate():
    sec = sh.section_from_candidate("suspension", _plate_candidate())
    pairs = _dxf_pairs(sh.section_dxf(sec))
    depth, seen = 0, []
    for code, val in pairs:
        if code == "0" and val == "SECTION":
            depth += 1
        elif code == "0" and val == "ENDSEC":
            depth -= 1
        elif code == "2" and depth == 1 and val in ("HEADER", "TABLES",
                                                    "ENTITIES"):
            seen.append(val)
    assert depth == 0, "unbalanced SECTION/ENDSEC"
    assert seen == ["HEADER", "TABLES", "ENTITIES"]
    assert pairs[-1] == ("0", "EOF")


def test_dxf_declares_the_appid_it_writes_xdata_under():
    """XDATA under an undeclared APPID is invalid and silently dropped by some
    readers — the roles would vanish without anything erroring."""
    sec = sh.section_from_candidate("suspension", _plate_candidate())
    pairs = _dxf_pairs(sh.section_dxf(sec))
    appids = [v for i, (c, v) in enumerate(pairs)
              if c == "2" and i and pairs[i - 1] == ("0", "APPID")]
    assert sh.XDATA_APPID in appids
    assert any(c == "1001" and v == sh.XDATA_APPID for c, v in pairs)


def test_every_entity_role_is_readable_from_layers():
    sec = sh.section_from_candidate("suspension", _plate_candidate())
    pairs = _dxf_pairs(sh.section_dxf(sec))
    layers = {v for c, v in pairs if c == "8"}
    assert sh.LAYER_OUTER in layers
    assert sh.LAYER_HOLE in layers
    assert sh.LAYER_ANNOTATION in layers


def test_manifest_survives_inside_the_dxf_alone():
    """The sidecar gets separated from the DXF constantly — emailed on its own,
    dropped into a shared folder. The DXF has to be self-sufficient."""
    sec = sh.section_from_candidate("suspension", _plate_candidate())
    dxf = sh.section_dxf(sec)
    body, grab = [], False
    for code, val in _dxf_pairs(dxf):
        if code != "999":
            continue
        if val.strip() == "BEGIN KK-MANIFEST-JSON":
            grab = True
        elif val.strip() == "END KK-MANIFEST-JSON":
            grab = False
        elif grab:
            body.append(val)
    man = json.loads("\n".join(body))
    assert man["schema"] == sh.SCHEMA
    assert man["part"]["geometry_digest"] == sec.digest()


def test_dxf_is_pure_ascii():
    """R12 is a 7-bit format in practice, and the app's strings are full of ⌀,
    · and em dashes."""
    for key, cand in (("suspension", _plate_candidate()),
                      ("powertrain", _flange_candidate()),
                      ("electrics", _box_candidate())):
        sec = sh.section_from_candidate(key, cand)
        sh.section_dxf(sec).decode("ascii")     # raises if anything slipped


def test_digest_changes_with_geometry_and_not_with_time():
    a = sh.section_from_candidate("suspension", _plate_candidate(pcd=92.0))
    b = sh.section_from_candidate("suspension", _plate_candidate(pcd=92.0))
    c = sh.section_from_candidate("suspension", _plate_candidate(pcd=96.0))
    assert a.digest() == b.digest()
    assert a.digest() != c.digest()


# --------------------------------------------------------------------------- #
#  3. Role classification
# --------------------------------------------------------------------------- #
def test_bolt_circle_is_recognised_with_the_right_pcd():
    sec = sh.section_from_candidate("suspension", _plate_candidate(pcd=92.0, n=3))
    assert len(sec.fastener_groups) == 1
    g = sec.fastener_groups[0]
    assert g.pattern == "circular"
    assert g.count == 3
    assert g.pcd_mm == pytest.approx(92.0, abs=0.05)
    assert all(h.role == sh.ROLE_HOLE_BOLT for h in sec.holes)


def test_central_bore_is_not_treated_as_a_bolt():
    """A motor register must never be handed a pretension."""
    sec = sh.section_from_candidate("powertrain", _flange_candidate())
    bores = [h for h in sec.holes if h.role == sh.ROLE_HOLE_BORE]
    bolts = [h for h in sec.holes if h.role == sh.ROLE_HOLE_BOLT]
    assert len(bores) == 1 and bores[0].d_mm == pytest.approx(75.0)
    assert len(bolts) == 4
    assert bores[0].group is None


def test_rectangular_pattern_does_not_claim_a_pcd():
    """Four corner holes lie on a circle, but no drawing dimensions them that
    way, so quoting a PCD would be a number nobody could check."""
    cand = {"label": "bracket", "meta": {},
            "dxf_kwargs": {"polylines": [{"pts": _rect(80, 50), "closed": True}],
                           "circles": [{"c": (6, 6), "r": 1.6},
                                       {"c": (74, 6), "r": 1.6},
                                       {"c": (6, 44), "r": 1.6},
                                       {"c": (74, 44), "r": 1.6}]}}
    sec = sh.section_from_candidate("data-acquisition", cand)
    g = sec.fastener_groups[0]
    assert g.pattern == "rectangular"
    assert g.pcd_mm == 0.0
    assert g.pitch_x_mm == pytest.approx(68.0)
    assert g.pitch_y_mm == pytest.approx(38.0)


def test_cavity_is_a_wall_not_a_hole():
    sec = sh.section_from_candidate("electrics", _box_candidate())
    assert sec.outer is not None
    assert len(sec.cavities) == 1
    assert sec.outer.area_mm2 > sec.cavities[0].area_mm2
    assert sec.wall_mm == pytest.approx(8.0, abs=0.01)


def test_clearance_hole_reads_back_to_the_right_thread():
    assert sh.infer_bolt(8.4)["nominal"] == "M8"
    assert sh.infer_bolt(8.4)["fit"] == "close"
    assert sh.infer_bolt(9.0)["nominal"] == "M8"
    assert sh.infer_bolt(9.0)["fit"] == "medium"
    assert sh.infer_bolt(6.6)["nominal"] == "M6"
    assert sh.infer_bolt(3.2)["nominal"] == "M3"
    # a bore is not a bolt hole and must not be given a thread
    assert sh.infer_bolt(75.0).get("nominal") is None


def test_pretension_matches_the_torque_it_quotes():
    """T = K·F·d, and if the two disagree the number a member torques to is
    not the number the model was preloaded with."""
    b = sh.infer_bolt(8.4, grade="10.9", preload_fraction=0.70, nut_factor=0.20)
    assert b["pretension_n"] == pytest.approx(0.70 * 830.0 * 36.6, rel=1e-3)
    expected = b["nut_factor"] * b["pretension_n"] * (b["nominal_d_mm"] / 1000.0)
    assert b["install_torque_nm"] == pytest.approx(expected, rel=1e-3)


# --------------------------------------------------------------------------- #
#  4. Mesh sizing
# --------------------------------------------------------------------------- #
def test_global_size_follows_material_thinness_not_hole_diameter():
    """A small hole must refine locally, not everywhere. Shrinking only the
    holes must not shrink the global element size."""
    coarse = sh.section_from_candidate(
        "suspension", _plate_candidate(pcd=92.0, hole=8.4, plate=200.0))
    fine_holes = sh.section_from_candidate(
        "suspension", _plate_candidate(pcd=92.0, hole=3.2, plate=200.0))
    m_coarse = sh.mesh_shortlist(coarse)
    m_fine = sh.mesh_shortlist(fine_holes)

    g_c = [lv for lv in m_coarse.levels if lv.name == "production"][0]
    g_f = [lv for lv in m_fine.levels if lv.name == "production"][0]
    # smaller holes leave MORE material, so the global size may not get finer
    assert g_f.global_size_mm >= g_c.global_size_mm
    # but the local size at the hole must follow the hole down
    assert g_f.local_size_mm < g_c.local_size_mm


def test_levels_are_monotonically_finer_and_more_expensive():
    sec = sh.section_from_candidate("suspension", _plate_candidate())
    lv = sh.mesh_shortlist(sec).levels
    assert [x.name for x in lv] == ["screening", "production", "convergence"]
    for a, b in zip(lv, lv[1:]):
        assert b.global_size_mm <= a.global_size_mm
        assert b.local_size_mm <= a.local_size_mm
        assert b.est_dof > a.est_dof
        assert b.growth_rate <= a.growth_rate
        assert b.quality["skewness_max"] <= a.quality["skewness_max"]


def test_thin_feature_gets_enough_elements_across_it():
    sec = sh.section_from_candidate("electrics", _box_candidate())
    mesh = sh.mesh_shortlist(sec)
    wall = sec.wall_mm
    for lv in mesh.levels:
        across = wall / lv.global_size_mm
        assert across >= lv.elems_across_thin - 1e-6, (
            f"{lv.name}: only {across:.2f} elements across an {wall} mm wall")


def test_a_bolted_part_is_never_shelled():
    """A shell has no through-thickness, so it cannot carry a pretension, a
    bearing stress at a hole, or a washer footprint."""
    thin_bolted = _plate_candidate(pcd=92.0, n=6, hole=6.6, plate=300.0)
    sec = sh.section_from_candidate("suspension", thin_bolted)
    sec.extrude_mm = 2.0                      # deliberately very thin
    basis = sh.mesh_shortlist(sec).basis
    assert "shell" not in basis["method"]
    assert "bolted" in basis["method_rationale"]


def test_sizing_scales_with_the_part():
    """The same shape ten times bigger should get roughly ten times the element
    size — the ladder is a ratio, not a fixed millimetre value."""
    # the holes have to scale too, or the two plates are not the same shape and
    # the test is asking the wrong question
    small = sh.section_from_candidate(
        "suspension", _plate_candidate(pcd=46.0, hole=3.2, plate=60.0))
    big = sh.section_from_candidate(
        "suspension", _plate_candidate(pcd=460.0, hole=32.0, plate=600.0))
    gs = [lv for lv in sh.mesh_shortlist(small).levels
          if lv.name == "production"][0].global_size_mm
    gb = [lv for lv in sh.mesh_shortlist(big).levels
          if lv.name == "production"][0].global_size_mm
    assert gb / gs == pytest.approx(10.0, rel=0.25)


def test_convergence_pair_is_declared():
    sec = sh.section_from_candidate("suspension", _plate_candidate())
    conv = sh.mesh_shortlist(sec).convergence
    assert conv["compare"] == ["production", "convergence"]
    assert conv["accept_change_pct"] > 0
    assert "singularity" in json.dumps(conv).lower()


# --------------------------------------------------------------------------- #
#  5. Honesty
# --------------------------------------------------------------------------- #
def test_a_missing_load_stays_null_and_marked_required():
    cand = _plate_candidate()
    cand["meta"].pop("Peak load (N)")
    sec = sh.section_from_candidate("suspension", cand)
    study = sh.study_spec(sec)
    corner = [lc for lc in study.load_cases if lc["name"] == "corner_peak_load"][0]
    assert corner["magnitude_n"] is None
    assert corner["required"] is True


def test_a_declared_load_is_carried_through_untouched():
    sec = sh.section_from_candidate("suspension", _plate_candidate())
    study = sh.study_spec(sec)
    corner = [lc for lc in study.load_cases if lc["name"] == "corner_peak_load"][0]
    assert corner["magnitude_n"] == pytest.approx(4200.0)
    assert not corner.get("required")


def test_starter_values_are_separated_from_measured_ones():
    sec = sh.section_from_candidate("suspension", _plate_candidate())
    t = sh.trust_map(sec)
    assert "PCD (mm)" in t["measured"]
    assert "Peak load (N)" in t["measured"]
    assert "Plate size (mm)" in t["starter"]
    # an assumed extrusion depth is a starter and must say so
    assert "extrude_mm" in t["starter"]


def test_a_declared_extrusion_is_not_marked_as_a_starter():
    cand = {"label": "Radiator core face", "meta": {"Core depth (mm)": 40.0},
            "dxf_kwargs": {"polylines": [{"pts": _rect(280, 220),
                                          "closed": True}]}}
    sec = sh.section_from_candidate("cooling", cand)
    assert sec.extrude_mm == pytest.approx(40.0)
    assert sec.extrude_source.startswith("declared")
    assert "extrude_mm" not in sh.trust_map(sec)["starter"]


def test_manifest_carries_the_screening_disclaimer():
    sec = sh.section_from_candidate("suspension", _plate_candidate())
    man = sh.handoff_manifest(sec)
    assert "screening" in man["disclaimer"].lower()


# --------------------------------------------------------------------------- #
#  6. One vocabulary across all four artefacts
# --------------------------------------------------------------------------- #
def test_every_boundary_condition_names_a_real_entity():
    """A constraint pointing at a name nothing defines is the exact failure this
    whole layer exists to prevent."""
    for key, cand in (("suspension", _plate_candidate()),
                      ("powertrain", _flange_candidate()),
                      ("electrics", _box_candidate())):
        sec = sh.section_from_candidate(key, cand)
        ns = set(sh.named_selections(sec))
        study = sh.study_spec(sec)
        targets = []
        for c in study.constraints:
            targets += list(c.get("targets", []))
        for lc in study.load_cases:
            targets += list(lc.get("targets", []))
        for t in targets:
            if t in ("ALL",) or t.startswith("KK_OUTER_EDGE"):
                continue          # documented placeholders the member picks
            assert t in ns, f"{key}: '{t}' is not a defined named selection"


def test_bundle_is_complete_and_internally_consistent():
    sec = sh.section_from_candidate("suspension", _plate_candidate())
    z = zipfile.ZipFile(io.BytesIO(sh.bundle_bytes(sec)))
    names = set(z.namelist())
    assert {"manifest.json", "study.json", "mesh_shortlist.md",
            "README.txt"} <= names
    dxf_name = [n for n in names if n.endswith(".dxf")][0]

    man = json.loads(z.read("manifest.json"))
    assert man["part"]["dxf"] == dxf_name
    assert man["part"]["geometry_digest"] == sec.digest()

    # the digest in the DXF must match the one in the manifest beside it
    assert sec.digest().encode() in z.read(dxf_name)

    # study.json and the manifest's study block must be the same study
    assert json.loads(z.read("study.json"))["analysis_type"] == \
        man["study"]["analysis_type"]


def test_handoff_for_candidate_returns_everything_the_ui_needs():
    out = sh.handoff_for_candidate("suspension", _plate_candidate())
    assert set(out) >= {"section", "mesh", "study", "manifest", "dxf",
                        "dxf_name", "bundle", "bundle_name"}
    assert out["dxf"].startswith(b"999")
    assert out["bundle"][:2] == b"PK"
    assert out["bundle_name"].endswith("_handoff.zip")
    assert sh.mesh_rows(out["mesh"])


# --------------------------------------------------------------------------- #
#  7. It must not fall over on the shapes the app really produces
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("subsystem", [
    "suspension", "aerodynamics", "powertrain", "electrics", "brakes",
    "cooling", "chassis", "data-acquisition", "something-new",
])
def test_every_subsystem_produces_a_usable_handoff(subsystem):
    out = sh.handoff_for_candidate(subsystem, _plate_candidate())
    assert out["manifest"]["schema"] == sh.SCHEMA
    assert out["study"].analysis_type
    assert len(out["mesh"].levels) == 3
    assert out["dxf"]


def test_empty_and_degenerate_candidates_do_not_raise():
    for cand in ({}, {"label": "x"}, {"label": "x", "dxf_kwargs": {}},
                 {"label": "x", "dxf_kwargs": {"polylines": [{"pts": []}]}},
                 {"label": "x", "dxf_kwargs": {"circles": [{"c": (0, 0),
                                                            "r": 0}]}},
                 {"label": "x", "dxf_kwargs": {
                     "polylines": [{"pts": [(0, 0), (1, 1)], "closed": True}]}}):
        sec = sh.section_from_candidate("suspension", cand)
        sh.mesh_shortlist(sec)
        sh.handoff_manifest(sec)
        sh.section_dxf(sec)


def test_legacy_profile_mm_candidates_still_work():
    """Older callers pass a bare point list. They must keep working — nothing
    upstream should have to change to get a handoff."""
    sec = sh.section_from_candidate(
        "brakes", {"label": "rotor half-section",
                   "profile_mm": _rect(120, 30), "meta": {}})
    assert sec.outer is not None
    assert len(sec.outer.pts) == 4
    assert sh.section_dxf(sec)
