"""Characterization suite for suspension/tubeframe.py (mutation gate); see tests/_golden.py. The demo side frame
(open bay and mid-span landing) before and after its fixes, a CSV frame, every tube size against every member class
plus off-catalog candidates, the audits at several tolerances, the BOM and spec consolidation, and the panel,
harness and seat screens across their branches."""
import dataclasses

import pytest

from suspension import tubeframe as T
from tests._golden_lazy import check_lazy
from tests._golden import plain as _plain


def plain(x):
    """tests._golden.plain, with sets sorted (their repr follows the hash seed)."""
    if dataclasses.is_dataclass(x) and not isinstance(x, type):
        return {f.name: plain(getattr(x, f.name)) for f in dataclasses.fields(x)}
    if isinstance(x, (set, frozenset)):
        return sorted((plain(v) for v in x), key=repr)
    if isinstance(x, dict):
        return {str(k): plain(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [plain(v) for v in x]
    return _plain(x)


def _try(fn, *a, **kw):
    try:
        return plain(fn(*a, **kw))
    except (ValueError, KeyError, TypeError) as e:
        return {"raised": type(e).__name__, "message": str(e)}


NODES = """ID,X,Y,Z,Label
a,0,0,0,corner
b,500,0,0,
c,500,0,400,top
d,0,0,400
e,250,0,0,mid bottom
f,250,0,200
"""
TUBES = """name,a,b,class,size,primary
ab,a,b,side_impact,B,1
bc,b,c,main_hoop,A,yes
cd,c,d,Main hoop,A,true
da,a,d,front_hoop,A,
ef,e,f,non_structural,C,0
"""


def _tubes(g):
    return list(g.tubes.values()) if isinstance(g.tubes, dict) else list(g.tubes)


NODES2 = """id,x,y,z
a,0,0,0
b,400,0,0
c,400,0,400
d,0,0,400
e,200,0,8.6
f,0,0,0
"""
TUBES2 = """name,a,b,class,size,primary,extra
ab,a,b,,B,0
bc,b,c,Weird thing,B,false
cd,c,d,main hoop,B,no
da,d,a,Front Hoop,B,n
ac,a,c,side_impact,,N,surplus,more
ae,a,e,,C,yes
"""


def _box():
    """A 600 x 400 x 300 box: a planar open face, a braced face, a warped face, a secondary face, an end landing
    off-centre on a primary tube and a zero-length tube."""
    g = T.FrameGraph()
    for nid, xyz in (("A", (0, 0, 0)), ("B", (600, 0, 0)), ("C", (600, 400, 0)), ("D", (0, 400, 0)),
                     ("E", (0, 0, 300)), ("F", (600, 0, 300)), ("G", (600, 400, 340)), ("H", (0, 400, 300)),
                     ("M", (300, 8.8, 0)), ("Z", (0, 0, 0))):
        g.add_node(nid, xyz)
    edges = [("AB", "A", "B", "side_impact", "B", True), ("BC", "B", "C", "main_hoop", "A", True),
             ("CD", "C", "D", "front_hoop", "A", True), ("DA", "D", "A", "side_impact", "B", True),
             ("EF", "E", "F", "side_impact", "B", True), ("FG", "F", "G", "main_hoop", "A", True),
             ("GH", "G", "H", "front_hoop", "A", True), ("HE", "H", "E", "side_impact", "B", False),
             ("AE", "A", "E", "side_impact", "B", True), ("BF", "B", "F", "side_impact", "B", True),
             ("CG", "C", "G", "roll_hoop_bracing", "B", True), ("DH", "D", "H", "front_bulkhead_support", "B", False),
             ("AF", "A", "F", "side_impact", "B", True), ("MC", "M", "C", "non_structural", "C", True),
             ("AZ", "A", "Z", "non_structural", "C", True)]
    for name, a, b, cls, size, prim in edges:
        g.add_tube(name, a, b, member_class=cls, size=size, is_primary=prim)
    return g


def _graph(g):
    out = {"dict": plain(g.as_dict()), "tri": plain(g.triangles()), "tri_all": plain(g.triangles(primary_only=False)),
           "tri_nodes": plain(g.triangulated_nodes()), "tri_nodes_all": plain(g.triangulated_nodes(False)),
           "bom": plain(g.bom_by_spec()), "table": plain(g.tube_table()), "ledger": plain(T.frame_summary_for_ledger(g)),
           "buckle": [g.euler_buckling_kN(tb) for tb in _tubes(g)], "len": [g.length_mm(tb) for tb in _tubes(g)]}
    for tol in (8.0, 0.5, 200.0):
        out[f"mid.{tol}"] = plain(g.midspan_landings(tol_mm=tol))
        out[f"mid.{tol}.hoops"] = plain(g.midspan_landings(tol_mm=tol, exempt_hoop_hosts=False))
    for tol in (60.0, 1.0):
        out[f"quads.{tol}"] = plain(g.untriangulated_quads(planarity_tol_mm=tol))
        out[f"quads.{tol}.all"] = plain(g.untriangulated_quads(planarity_tol_mm=tol, primary_only=False))
    rt = T.FrameGraph.from_dict(g.as_dict())
    out["roundtrip"] = plain(rt.as_dict()) == out["dict"]
    return out


def _everything():
    table = T.default_size_table()
    yield "table", {k: plain(v) | {"id": v.id_mm, "area": v.area_mm2, "mpm": v.mass_per_m_kg, "m": v.mass_kg(750.0),
                                   "cost": v.cost_usd(750.0), "rt": plain(T.TubeSpec.from_dict(v.as_dict()))}
                    for k, v in table.items()}
    cands = dict(table, big=T.TubeSpec("big", 31.75, 1.65), thin=T.TubeSpec("thin", 28.6, 1.1),
                 small=T.TubeSpec("small", 22.2, 2.6), wall2=T.TubeSpec("wall2", 25.4, 2.0),
                 a_plus=T.TubeSpec("A+", 25.4, 2.41), b_minus=T.TubeSpec("B-", 25.0, 1.65))
    for k, c in cands.items():
        for cls in T.MEMBER_CLASS_MIN_SIZE:
            yield f"size.{k}.{cls}", [_try(T.size_meets_minimum, c, cls), _try(T.equivalency_check, c, cls)]
    yield "size.unknown_class", [_try(T.size_meets_minimum, table["A"], "nope"), _try(T.equivalency_check,
                                                                                     table["A"], "nope")]
    g = T.demo_frame()
    yield "demo", _graph(g)
    yield "demo.path", plain(g.load_path_audit(T.DEMO_PATH_FROM, T.DEMO_PATH_TO))
    yield "demo.path.tight", plain(g.load_path_audit(T.DEMO_PATH_FROM, T.DEMO_PATH_TO, midspan_tol_mm=0.1))
    yield "demo.path.none", _try(g.load_path_audit, "BADT", "FBL")
    yield "demo.path.unknown", _try(g.load_path_audit, "MHS", "nope")
    fixed = T.demo_frame()
    fixed.add_tube("diag", "SIL", "MHS", member_class="side_impact", size="B")
    fixed.tubes = [t for t in _tubes(fixed) if t.name != "bad_support"]
    yield "fixed", _graph(fixed)
    yield "fixed.path", plain(fixed.load_path_audit(T.DEMO_PATH_FROM, T.DEMO_PATH_TO))
    cons = T.demo_frame()
    yield "consolidate", plain(cons.consolidate_spec("C", "B"))
    yield "consolidate.n", cons.apply_consolidation("C", "B")
    yield "consolidate.after", plain(cons.bom_by_spec())
    yield "consolidate.bad", [_try(cons.consolidate_spec, "B", "C"), _try(cons.consolidate_spec, "Z", "B")]
    yield "consolidate.none", cons.apply_consolidation("C", "B")
    yield "errors", [_try(g.add_tube, "x", "FBL", "nope"), _try(g.add_tube, "x", "FBL", "FBL"),
                     _try(g.spec_of, T.FrameTube("q", "FBL", "FBU", size="ZZ")), _try(g.p, "nope")]
    csv = T.FrameGraph.from_csv(NODES, TUBES)
    yield "csv", _graph(csv)
    big_table = dict(T.default_size_table(), C=T.TubeSpec("C", 30.0, 2.0))
    csv2 = T.FrameGraph.from_csv(NODES2, TUBES2, size_table=big_table)
    yield "csv2", _graph(csv2)
    yield "csv2.path", plain(csv2.load_path_audit("a", "c"))
    yield "csv.errors", [_try(T.FrameGraph.from_csv, "id,x,y,z\na,1,2,zz\n", TUBES2),
                         _try(T.FrameGraph.from_csv, b"\xff\xfe\x00bad", TUBES2),
                         _try(T.FrameGraph.from_csv, NODES2, "name,a,b\nab,a,nope\n")]
    box = _box()
    yield "box", _graph(box)
    yield "box.path", plain(box.load_path_audit("A", "G"))
    yield "box.path.default_tol", plain(box.load_path_audit("A", "C"))
    yield "box.path.secondary", plain(box.load_path_audit("A", "H"))
    for tol in (0.2, 8.5, 9.5, 30.0):
        yield f"box.mid.{tol}", plain(box.midspan_landings(tol_mm=tol))
        yield f"box.path.mid.{tol}", plain(box.load_path_audit("A", "G", midspan_tol_mm=tol))
    for tol in (None, 9.5):
        kw = {} if tol is None else dict(midspan_tol_mm=tol)
        yield f"box.path.AB.{tol}", plain(box.load_path_audit("A", "B", **kw))
    box2 = _box()
    box2.add_tube("DE", "D", "E", member_class="side_impact", size="B", is_primary=False)
    box2.add_tube("EG", "E", "G", member_class="side_impact", size="B")
    yield "box2", _graph(box2)
    yield "box2.paths", [plain(box2.load_path_audit(a, b)) for a, b in (("A", "H"), ("D", "G"), ("H", "C"))]
    csv3 = T.FrameGraph.from_csv(NODES2, "name,a,b\nab,a,b\nbc,b,c\nac,a,c\ncd,c,d,\n")
    yield "csv3", _graph(csv3)
    csv4 = T.FrameGraph.from_csv(NODES2, "name,a,b,class,size,primary\nab,a,b,side impact,,\nbc,b,c,Side Impact,B,1\n")
    yield "csv4", _graph(csv4)
    yield "csv.nul", [_try(T.FrameGraph.from_csv, "id,x,y,z\na,1,2,3\x00\n", "name,a,b\n"),
                      _try(T.FrameGraph.from_csv, NODES2, "name,a,b\nab,a,\x00b\n")]
    short = _box()
    short.add_node("T", (0.5, 0.0, 0.0))
    short.add_node("U", (300.0, 0.0, 0.4))
    short.add_tube("AT", "A", "T")
    short.add_tube("UB", "U", "B", member_class="side_impact", size="B")
    yield "short", _graph(short)
    yield "short.mid", [plain(short.midspan_landings(tol_mm=t)) for t in (0.3, 0.45, 1.0)]
    yield "add_node.bad", [_try(T.FrameGraph().add_node, "n", (1.0, 2.0)), _try(T.FrameGraph().add_node, "n",
                                                                              (1.0, 2.0, float("nan"))),
                           _try(T.FrameGraph().add_node, "n", (1.0, 2.0, 3.0, 4.0))]
    yield "equiv.empty_table", [_try(T.equivalency_check, T.TubeSpec("x", 26.0, 1.9), cls, {}) for cls in
                                ("main_hoop", "side_impact", "non_structural")]
    cons2 = _box()
    yield "cons2", [_try(cons2.consolidate_spec, "B", "A"), _try(cons2.consolidate_spec, "C", "B"),
                    _try(cons2.consolidate_spec, "A", "C"), _try(cons2.consolidate_spec, "C", "ZZ"),
                    _try(cons2.consolidate_spec, "B", "B")]
    yield "panel.defaults", [_try(T.plan_panel_attachment, "aero", 600.0, 250.0, 2.0, "GFRP / fibreglass", 60.0,
                                  pressure_kPa=4.0),
                             _try(T.plan_panel_attachment, "aero", 20.0, 20.0, 1.0, "Polycarbonate", 0.5,
                                  speed_kph=60.0),
                             _try(T.plan_panel_attachment, "aero", 300.0, 200.0, 1.0, "Polycarbonate", 150.0,
                                  speed_kph=0.0)]
    yield "dict.rt", plain(T.FrameGraph.from_dict(dict(box.as_dict(), size_table={
        "B": T.TubeSpec("B", 26.0, 1.8).as_dict()})).as_dict())
    yield "dict.bare", plain(T.FrameGraph.from_dict({}).as_dict())
    yield "dict.nolabel", plain(T.FrameGraph.from_dict({"nodes": [{"nid": "q", "xyz_mm": [1, 2, 3]}]}).as_dict())
    yield "dict.bad", [_try(T.FrameGraph.from_dict, {"nodes": [{"nid": "q", "xyz_mm": [1, 2]}]}),
                       _try(T.FrameGraph.from_dict, {"nodes": [{"nid": "q", "xyz_mm": [1, 2, float("inf")]}]}),
                       _try(T.FrameNode.from_dict, {"nid": "z", "xyz_mm": [0, 0, 0], "label": None})]
    yield "csv.bad", [_try(T.FrameGraph.from_csv, "id,x\na,1\n", TUBES), _try(T.FrameGraph.from_csv, NODES,
                                                                               "name,a\nab,a\n")]
    for kind in sorted(T.PANEL_KIND_NOTES) + ["other"]:
        for mat in T.PANEL_MATERIALS:
            yield f"panel.{kind}.{mat}", _try(T.plan_panel_attachment, kind, 400.0, 300.0, 1.5, mat, 100.0,
                                              speed_kph=110.0)
    yield "panel.pressure", _try(T.plan_panel_attachment, "aero", 600.0, 250.0, 2.0, "GFRP / fibreglass", 60.0,
                                 pressure_kPa=4.0, deflection_limit_mm=2.0, g_load=5.0, fos_target=2.0)
    yield "panel.coarse", _try(T.plan_panel_attachment, "aero", 900.0, 900.0, 0.8, "Polycarbonate", 400.0,
                               speed_kph=140.0, cp=1.5, deflection_limit_mm=1.0)
    yield "panel.noload", _try(T.plan_panel_attachment, "aero", 200.0, 200.0, 1.0, "Polycarbonate", 50.0)
    yield "panel.badmat", _try(T.plan_panel_attachment, "aero", 200.0, 200.0, 1.0, "Wood", 50.0, speed_kph=50.0)
    yield "q", [T.dynamic_pressure_kPa(v, cp) for v in (0.0, 60.0, 120.0) for cp in (1.0, 1.2)]
    yield "harness", plain(T.harness_attachment_loads())
    yield "harness.heavy", plain(T.harness_attachment_loads(95.0, 25.0, 0.7, 25.0, 40.0, 1, 2, 1))
    yield "seat", plain(T.seat_mount_check(4.0, 77.0))
    yield "seat.weak", plain(T.seat_mount_check(12.0, 110.0, n_mounts=2, g_vertical=6.0, g_lateral=4.0,
                                                g_longitudinal=5.0, fastener="M5 rivnut", fos_target=5.0))
    yield "seat.badfast", _try(T.seat_mount_check, 4.0, 77.0, fastener="glue")
    yield "consts", [T.RULES_YEAR_TRANSCRIBED, T.RULES_DISCLAIMER, plain(T.MEMBER_CLASS_LABELS)]


def _quad(lift=0.0, size_table=None, secondary_first=True):
    """A 1000 mm square bay, one corner lifted, its closing edge (Q4-Q1) the strictest class; a secondary tube first."""
    g = T.FrameGraph(size_table=size_table)
    for nid, xyz in (("Q1", (0, 0, 0)), ("Q2", (1000, 0, 0)), ("Q3", (1000, 1000, lift)), ("Q4", (0, 1000, 0)),
                     ("S1", (0, 0, 2000)), ("S2", (10, 0, 2000))):
        g.add_node(nid, xyz)
    if secondary_first:
        g.add_tube("A_sec", "S1", "S2", is_primary=False)
    for name, a, b, cls in (("Q12", "Q1", "Q2", "non_structural"), ("Q23", "Q2", "Q3", "side_impact"),
                            ("Q34", "Q3", "Q4", "non_structural"), ("Q41", "Q4", "Q1", "main_hoop")):
        g.add_tube(name, a, b, member_class=cls)
    return g


def _landings():
    """A zero-length tube listed first, a host 0.9 mm long with an end landing on it, and an end 8.5 mm off a host."""
    g = T.FrameGraph()
    for nid, xyz in (("Z1", (5, 5, 5)), ("Z2", (5, 5, 5)), ("S1", (0, 0, 0)), ("S2", (0.9, 0, 0)), ("R", (0.45, 0.05, 0)),
                     ("R2", (0.45, 50, 0)), ("H1", (0, 0, 100)), ("H2", (100, 0, 100)), ("P", (50, 8.5, 100)),
                     ("P2", (50, 100, 100))):
        g.add_node(nid, xyz)
    for name, a, b in (("A_zero", "Z1", "Z2"), ("S", "S1", "S2"), ("RR", "R", "R2"), ("H", "H1", "H2"),
                       ("PP", "P", "P2")):
        g.add_tube(name, a, b, member_class="side_impact")
    return g


def _more():
    cand = T.TubeSpec("x", 26.0, 1.9)
    yield "equiv.table", [_try(T.equivalency_check, cand, "side_impact", {"B": T.TubeSpec("B", 30.0, 3.0)}),
                          _try(T.equivalency_check, cand, "side_impact", {"A": T.TubeSpec("A", 30.0, 3.0)})]
    yield "csv.more", [_try(T.FrameGraph.from_csv, "id,x,y,z\n" + "a" * 140000 + ",1,2,3\n", "name,a,b\n"),
                       _try(lambda: plain(T.FrameGraph.from_csv(NODES2, "name,a,b\nab,a,a\n").as_dict())),
                       _try(lambda: plain(T.FrameGraph.from_csv(NODES2, "name,a,b\nab,a,b\n").as_dict()))]
    g = _landings()
    yield "landings", [plain(g.midspan_landings()), plain(g.midspan_landings(tol_mm=0.1)),
                       plain(g.midspan_landings(tol_mm=9.0))]
    big_a = dict(T.default_size_table(), A=T.TubeSpec("A", 30.0, 3.0))
    for name, q in (("flat", _quad()), ("lift", _quad(240.0)), ("lift_more", _quad(241.0)), ("big_a", _quad(0.0, big_a)),
                    ("only_c", _quad(0.0, {"C": T.default_size_table()["C"]})), ("noSec", _quad(secondary_first=False))):
        yield f"quad.{name}", [plain(q.untriangulated_quads()), plain(q.untriangulated_quads(planarity_tol_mm=0.0)),
                               plain(q.untriangulated_quads(planarity_tol_mm=61.0))]
    tbl = dict(T.default_size_table(), B=T.TubeSpec("B", 40.0, 4.0), C=T.TubeSpec("C", 30.0, 2.6))
    cg = _quad(0.0, tbl)
    cg.add_tube("Q13", "Q1", "Q3", member_class="side_impact", size="B")
    yield "consolidate.table", plain(cg.consolidate_spec("B", "C"))
    yield "q.default_cp", T.dynamic_pressure_kPa(100.0)
    pol = "Polycarbonate"
    zero = T.plan_panel_attachment("aero", 300.0, 200.0, 1.0, pol, 80.0, g_load=0.0)
    four = T.plan_panel_attachment("aero", 100.0, 100.0, 1.0, pol, 100.0, pressure_kPa=2.0, g_load=0.0)
    cap = min(T.FASTENER_OPTIONS[0]["shear_N"], T.FASTENER_OPTIONS[0]["tension_N"])
    exact = T.plan_panel_attachment("aero", 100.0, 100.0, 1.0, pol, 100.0, pressure_kPa=2.0, g_load=0.0,
                                    fos_target=cap / four.load_per_fastener_N)
    mid = T.plan_panel_attachment("aero", 100.0, 100.0, 1.0, pol, 100.0, pressure_kPa=cap / 8.75, g_load=0.0)
    hard = T.plan_panel_attachment("aero", 900.0, 900.0, 0.5, pol, 600.0, pressure_kPa=50.0, fos_target=1e6)
    yield "panel.more", [plain(p) for p in (zero, four, exact, mid, hard)]
    base = T.seat_mount_check(4.0, 77.0)
    per = base["load_per_mount_N"]
    yield "seat.more", [plain(T.seat_mount_check(4.0, 77.0, n_mounts=1)),
                        plain(T.seat_mount_check(4.0, 77.0, g_vertical=0.0, g_lateral=0.0, g_longitudinal=0.0)),
                        plain(T.seat_mount_check(0.01, 0.0, g_vertical=1.0, g_lateral=0.0, g_longitudinal=0.0)),
                        plain(T.seat_mount_check(4.0, 77.0, fos_target=cap / per))]
    yield "harness.no_antisub", plain(T.harness_attachment_loads(n_antisub=0))


def test_every_frame_audit_and_screen():
    check_lazy("tubeframe", _everything())


def test_every_bay_landing_and_screen_edge():
    check_lazy("tubeframe_more", _more())


def test_a_tube_needs_two_known_ends():
    g = T.FrameGraph()
    g.add_node("a", (0, 0, 0))
    with pytest.raises(KeyError):
        g.add_tube("t", "a", "b")
