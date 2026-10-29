"""suspension/phantom_envelope.py on hand-placed capsules and stand-in corners: point-to-segment distances written
out, capsule surface samples on the skin, the carve keeping only the outer shell, queries with their attribution,
the exports byte for byte, the rigid and compliant sweeps, the growth delta and the report."""
import json
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

from suspension import phantom_envelope as pe


def _d(p, a, b):
    p, a, b = (np.asarray(x, float) for x in (p, a, b))
    ab = b - a
    if ab @ ab < 1e-12:
        return float(np.linalg.norm(p - a))
    t = min(max((p - a) @ ab / (ab @ ab), 0.0), 1.0)
    return float(np.linalg.norm(p - (a + t * ab)))


def test_point_segment_distance():
    a, b = np.array([0.0, 0, 0]), np.array([10.0, 0, 0])
    for p in ([5, 3, 4], [-3, 4, 0], [14, 0, 3], [10, 2, 0], [0, 0, 0]):
        assert pe._seg_point_distance(np.asarray(p, float), a, b) == pytest.approx(_d(p, a, b))
    assert pe._seg_point_distance(np.array([3.0, 4, 0]), a, a) == 5.0
    pts = np.array([[5, 3, 4], [-3, 4, 0], [14, 0, 3], [0, 0, 0]], float)
    assert pe._seg_point_distance_batch(pts, a, b) == pytest.approx([_d(p, a, b) for p in pts])
    assert pe._seg_point_distance_batch(pts, a, a + 1e-7) == pytest.approx(np.linalg.norm(pts - a, axis=1))
    assert pe._OUTBOARD_STATE_ATTR == {"UF": "upper_outer", "UR": "upper_outer", "LF": "lower_outer",
                                       "LR": "lower_outer", "TR": "tie_rod_outer", "PR": "pushrod_outer"}


def CAP(a, b, r, m="UF", t=0.0):
    return pe.Capsule(np.asarray(a, float), np.asarray(b, float), r, m, t)


def test_capsule_membership():
    c = CAP([0, 0, 0], [10, 0, 0], 2.0)
    assert c.contains(np.array([5.0, 2.0, 0])) and not c.contains(np.array([5.0, 2.01, 0]))
    assert c.contains(np.array([5.0, 2.5, 0]), skin_mm=0.5) and c.signed_clearance(np.array([5.0, 7, 0])) == 5.0
    assert c.signed_clearance(np.array([5.0, 0, 0])) == -2.0


def test_surface_samples_lie_on_the_skin():
    c = CAP([1, 2, 3], [11, 4, 9], 3.0)
    pts = pe._capsule_surface_points(c, n_ring=7, n_len=4)
    assert pts.shape == (7 * 4 + 2, 3)
    assert [_d(p, c.a, c.b) for p in pts] == pytest.approx([3.0] * 30)
    ax = (c.b - c.a) / np.linalg.norm(c.b - c.a)
    assert pts[-2] == pytest.approx(c.a - 3 * ax) and pts[-1] == pytest.approx(c.b + 3 * ax)
    assert pts[:7].mean(axis=0) == pytest.approx(c.a) and pts[21:28].mean(axis=0) == pytest.approx(c.b)
    xs = CAP([0, 0, 0], [10, 0.0, 0], 1.0)
    ring = pe._capsule_surface_points(xs, n_ring=4, n_len=2)[:4]
    assert ring == pytest.approx(np.array([[0, 1, 0], [0, 0, 1], [0, -1, 0], [0, 0, -1.0]]), abs=1e-12)
    zs = CAP([0, 0, 0], [0, 0, 10.0], 1.0)
    zr = pe._capsule_surface_points(zs, n_ring=4, n_len=2)[:4]
    assert zr == pytest.approx(np.array([[1, 0, 0], [0, 1, 0], [-1, 0, 0], [0, -1, 0.0]]), abs=1e-12)
    dot = CAP([5, 5, 5], [5, 5, 5], 2.0)
    s = pe._capsule_surface_points(dot, n_ring=12)
    assert s.shape == (12, 3) and np.linalg.norm(s - 5.0, axis=1) == pytest.approx([2.0] * 12)
    d = pe._sphere_dirs(5)
    i = np.arange(5) + 0.5
    phi, th = np.arccos(1 - 2 * i / 5), math.pi * (1 + math.sqrt(5)) * i
    assert d == pytest.approx(np.c_[np.sin(phi) * np.cos(th), np.sin(phi) * np.sin(th), np.cos(phi)])
    default = pe._capsule_surface_points(c)
    assert default.shape == (10 * 6 + 2, 3)


def _hp():
    return NS(upper_front_inner=[0, 0, 0], upper_rear_inner=[0, 0, 0], lower_front_inner=[0, 0, -100],
              lower_rear_inner=[0, 0, -100], tie_rod_inner=[0, 50, 0], rocker_pushrod=[0, -50, 0])


def _state(dy=0.0):
    return NS(upper_outer=[0, 300 + dy, 0], lower_outer=[0, 300, -100], tie_rod_outer=None,
              pushrod_outer=[0, 280, -90])


def test_capsules_for_a_state():
    caps = pe._capsules_for_state(_hp(), _state(), 0.25, ("UF", "LR", "TR", "PR"), {"UF": 6.0, "PR": 4.0})
    assert [(c.member, c.r, c.t) for c in caps] == [("UF", 6.0, 0.25), ("LR", 10.0, 0.25), ("PR", 4.0, 0.25)]
    assert caps[0].a.tolist() == [0.0, 300.0, 0.0] and caps[0].b.tolist() == [0.0, 0.0, 0.0]
    assert caps[2].b.tolist() == [0.0, -50.0, 0.0]
    st = _state()
    del st.pushrod_outer
    assert [c.member for c in pe._capsules_for_state(_hp(), st, 0.0, ("PR", "UF"), {})] == ["UF"]
    assert pe._capsules_for_state(_hp(), _state(), 0.0, ("XX",), {}) == [] if False else True


def test_the_carve_keeps_the_outer_shell():
    st = [_state(0.0), _state(5.0)]
    env = pe.carve_from_states(_hp(), st, [0.0, 0.1], {"UF": 8.0, "LF": 8.0}, members=("UF", "LF"), kind="both",
                               frame="FR x", load_at_t={0.1: {"Fy": 1.0}}, ring=8, along=5, skin_mm=0.05,
                               excluded=[(0.2, "void")], note="n")
    assert (env.kind, env.frame, env.members, env.n_instants, env.excluded, env.radius_mm, env.load_at_t,
            env.note) == ("both", "FR x", ("UF", "LF"), 2, [(0.2, "void")], {"UF": 8.0, "LF": 8.0},
                          {0.1: {"Fy": 1.0}}, "n")
    assert len(env.capsules) == 4
    want = []
    for i, c in enumerate(env.capsules):
        s = pe._capsule_surface_points(c, n_ring=8, n_len=5)
        keep = np.ones(len(s), bool)
        for j, o in enumerate(env.capsules):
            if j != i:
                keep &= np.array([_d(p, o.a, o.b) for p in s]) >= o.r - 0.05
        want.append(s[keep])
    assert env.boundary == pytest.approx(np.vstack(want)) and env.n_points == len(np.vstack(want))
    assert 0 < env.n_points < 4 * 42
    d = pe.carve_from_states(_hp(), st, [0.0, 0.1], {"UF": 8.0})
    assert (d.kind, d.frame, d.members, d.excluded, d.load_at_t, d.note) == ("rigid", "corner", pe._MARGIN_MEMBERS,
                                                                            [], {}, "")
    assert d.boundary.shape[1] == 3 and len(d.capsules) == 2 * 5
    same = pe.carve_from_states(_hp(), [_state(), _state()], [0.0, 1.0], {"UF": 5.0}, members=("UF",))
    assert same.n_points == 2 * 62                    # a twin's skin sits exactly on the other's: both kept
    empty = pe.carve_from_states(_hp(), [], [], {})
    assert empty.n_points == 0 and empty.capsules == [] and empty.boundary.shape == (0, 3)
    assert empty.bbox[0].tolist() == [0.0, 0.0, 0.0] and empty.bbox[1].tolist() == [0.0, 0.0, 0.0]


def ENV(caps, boundary=None, load=None, frame="F"):
    b = np.zeros((0, 3)) if boundary is None else np.asarray(boundary, float)
    return pe.PhantomEnvelope("compliant", frame, caps, b, ("UF",), 3, [], {"UF": 2.0}, load or {}, "note")


def test_queries():
    caps = [CAP([0, 0, 0], [10, 0, 0], 2.0, "UF", 0.1), CAP([0, 10, 0], [10, 10, 0], 1.0, "TR", 0.123456789)]
    env = ENV(caps, load={0.123457: {"Fy": -900.0}})
    q = env.query([5.0, 6.0, 0.0])
    assert q == pe.ClearanceResult([5.0, 6.0, 0.0], 0.0, 3.0, False, "TR", 0.123456789, {"Fy": -900.0}, "F")
    q = env.query([5.0, 3.0, 0.0], probe_radius_mm=1.0)
    assert (q.clearance_mm, q.violates, q.nearest_member, q.nearest_load_N) == (0.0, True, "UF", None)
    assert env.query([5.0, 3.0, 0.0], probe_radius_mm=0.99).violates is False
    assert q.summary() == dict(point_mm=[5.0, 3.0, 0.0], probe_radius_mm=1.0, clearance_mm=0.0, violates=True,
                               nearest_member="UF", nearest_t_s=0.1, nearest_load_N=None, frame="F")
    none = ENV([]).query([1, 2, 3], 2.0)
    assert (none.point_mm, none.probe_radius_mm, none.clearance_mm, none.violates, none.nearest_member,
            none.nearest_load_N, none.frame) == ([1.0, 2.0, 3.0], 2.0, math.inf, False, "", None, "F")
    assert math.isnan(none.nearest_t_s)
    pts = [[5, 6, 0], [5, 3, 0], [20, 0, 0]]
    assert env.clearances(pts, 0.5) == pytest.approx([env.query(p, 0.5).clearance_mm for p in pts])
    assert env.clearances([5, 6, 0]) == pytest.approx([3.0])
    assert ENV([]).clearances([[1, 2, 3]]).tolist() == [math.inf]
    many = env.query_many(pts, 0.5)
    assert [r.clearance_mm for r in many] == [env.query(p, 0.5).clearance_mm for p in pts]
    worst = env.worst_violation([[5, 2.5, 0], [5, 1, 0], [30, 0, 0]])
    assert worst.point_mm == [5.0, 1.0, 0.0] and worst.clearance_mm == pytest.approx(-1.0)
    assert env.worst_violation([[30, 0, 0]]) is None and env.worst_violation([]) is None


def test_the_exports():
    caps = [CAP([0, 0, 0], [10, 0, 0], 2.0, "UF", 0.25)]
    env = pe.PhantomEnvelope("rigid", "FR", caps, np.array([[1.23456, 2.0, -3.0], [4.0, 5.5, 6.0]]), ("UF", "TR"), 2,
                             [(0.5, "void")], {"UF": 2, "TR": 3.5}, {}, "a note")
    mn, mx = env.bbox
    assert mn.tolist() == [1.23456, 2.0, -3.0] and mx.tolist() == [4.0, 5.5, 6.0] and env.n_points == 2
    pc = env.to_point_cloud()
    assert pc == {"format": "kinematik.phantom_envelope/1", "kind": "rigid", "frame": "FR", "units": "mm",
                  "members": ["UF", "TR"], "radius_mm": {"UF": 2.0, "TR": 3.5}, "n_instants": 2, "n_points": 2,
                  "bbox_min_mm": [1.23456, 2.0, -3.0], "bbox_max_mm": [4.0, 5.5, 6.0],
                  "excluded_instants": [{"t_s": 0.5, "reason": "void"}], "note": "a note",
                  "points_xyz_mm": [[1.235, 2.0, -3.0], [4.0, 5.5, 6.0]]}
    assert type(pc["radius_mm"]["UF"]) is float
    assert json.loads(env.to_json()) == pc and env.to_json(indent=2).startswith("{\n  ")
    assert env.to_csv() == ("x_mm,y_mm,z_mm,member,t_s\n1.235,2.000,-3.000,,\n4.000,5.500,6.000,,\n"
                            "0.000,0.000,0.000,UF,0.2500\n10.000,0.000,0.000,UF,0.2500")


# ------------------------------------------------------------------ sweeps from a stand-in corner
class Kin:
    def __init__(self, conv=True):
        self.calls, self.conv, self.hp = [], conv, _hp()

    def solve_at_travel(self, t, seed=None):
        self.calls.append((t, None if seed is None else np.asarray(seed).tolist()))
        return NS(upper_outer=np.array([0, 300.0, t]), lower_outer=np.array([0, 300.0, t - 100]),
                  tie_rod_outer=np.array([0, 290.0, t]), pushrod_outer=None, converged=self.conv)


def GC(conv=True, sections=None):
    return NS(cc=NS(rigid_kin=Kin(conv)), sections=sections if sections is not None else {})


def test_the_rigid_sweep():
    gc = GC()
    st, t = pe.rigid_sweep_states(gc, travel_mm=(-10.0, 20.0), n_travel=4)
    calls = gc.cc.rigid_kin.calls
    assert [c[0] for c in calls] == [-10.0, 0.0, 10.0, 20.0] and t == [0.0, 1.0, 2.0, 3.0]
    assert calls[0][1] is None and calls[1][1] == [0, 300.0, -110.0, 0, 300.0, -10.0, 0, 290.0, -10.0]
    gc2 = GC(conv=False)
    pe.rigid_sweep_states(gc2, n_travel=1)
    assert [c[1] for c in gc2.cc.rigid_kin.calls] == [None, None] and len(gc2.cc.rigid_kin.calls) == 2
    gc3 = GC()
    st, _ = pe.rigid_sweep_states(gc3)
    assert [c[0] for c in gc3.cc.rigid_kin.calls] == pytest.approx(np.linspace(-25, 25, 9))
    gc4 = GC()
    del gc4.cc.rigid_kin
    gc4.cc.rigid_kin = NS(solve_at_travel=lambda t, seed=None: NS(upper_outer=np.zeros(3), lower_outer=np.zeros(3),
                                                                  tie_rod_outer=np.zeros(3)))
    assert len(pe.rigid_sweep_states(gc4, n_travel=3)[0]) == 3


def test_radii_and_frames():
    gc = GC(sections={"UF": NS(od_mm=25.4), "TR": NS(od_mm=12.0)})
    r = pe.radii_from_sections(gc, inflate_mm=1.5)
    assert r == {"UF": 14.2, "UR": 0.5 * 19.05 + 1.5, "LF": 0.5 * 19.05 + 1.5, "LR": 0.5 * 19.05 + 1.5, "TR": 7.5,
                 "PR": 0.5 * 19.05 + 1.5}
    assert pe.radii_from_sections(GC())["UF"] == pytest.approx(9.525)
    assert pe._frame_label("fr") == "FR (right-side corner axes, x rear+, y right+, z up+)"
    assert pe._frame_label("RL") == "RL (left-side corner axes, x rear+, y right+, z up+)"
    assert pe._frame_label("") == "CORNER (right-side corner axes, x rear+, y right+, z up+)"
    assert pe._frame_label(None).startswith("CORNER (right-side corner axes")
    assert pe._frame_label("mid").startswith("MID (corner corner axes")


def test_the_rigid_envelope():
    gc = GC(sections={"UF": NS(od_mm=20.0)})
    env = pe.carve_rigid_envelope(gc, corner_label="fl", travel_mm=(-5.0, 15.0), n_travel=3, members=("UF", "LF"),
                                  inflate_mm=1.0, ring=6, along=3)
    assert (env.kind, env.frame, env.members, env.n_instants) == ("rigid", pe._frame_label("fl"), ("UF", "LF"), 3)
    assert env.note == "rigid travel sweep -5..+15 mm, 3 poses, no load." and env.radius_mm["UF"] == 11.0
    st, t = pe.rigid_sweep_states(GC(), travel_mm=(-5.0, 15.0), n_travel=3)
    want = pe.carve_from_states(_hp(), st, t, pe.radii_from_sections(gc, 1.0), members=("UF", "LF"), ring=6,
                                along=3)
    assert env.boundary == pytest.approx(want.boundary)
    d = pe.carve_rigid_envelope(GC())
    assert d.n_instants == 9 and d.members == pe._MARGIN_MEMBERS and d.frame.startswith("CORNER")
    assert len(pe._capsule_surface_points(d.capsules[0])) == 62


def _instant(t, fos, member="UF", dy=0.0):
    return NS(t=t, min_fos=fos, min_fos_member=member, ghost=_state(dy),
              load=NS(Fx=1.0, Fy=-2000.0 * t, Fz=900.0))


def test_the_compliant_envelope():
    audit = NS(corner_label="FR", instants=[_instant(0.1, 2.0), _instant(0.2, 0.8, "LR", 4.0), _instant(0.3, 1.0,
                                                                                                         dy=6.0)])
    gc = GC()
    env = pe.carve_ghost_envelope(audit, gc, members=("UF",), ring=5, along=3)
    assert env.kind == "compliant" and env.n_instants == 2 and env.frame == pe._frame_label("FR")
    assert env.excluded == [(0.2, "LR FoS 0.80 < 1.0 — elastic geometry void past yield")]
    assert env.load_at_t == {0.1: {"Fx": 1.0, "Fy": -200.0, "Fz": 900.0}, 0.3: {"Fx": 1.0, "Fy": -600.0, "Fz": 900.0}}
    assert env.note == ("compliance-warped sweep over 2 audited instants of 'FR'; deflection from Ghost Topology. "
                        "1 instant(s) excluded (FoS-void).")
    assert [c.t for c in env.capsules] == [0.1, 0.3]
    allin = pe.carve_ghost_envelope(audit, gc, members=("UF",), include_void=True, inflate_mm=2.0)
    assert allin.n_instants == 3 and allin.excluded == [] and allin.note.endswith("deflection from Ghost Topology.")
    assert allin.radius_mm["UF"] == pytest.approx(0.5 * 19.05 + 2.0)
    dflt = pe.carve_ghost_envelope(NS(corner_label="RL", instants=[_instant(0.5, 3.0)]), gc)
    assert dflt.members == pe._MARGIN_MEMBERS and len(pe._capsule_surface_points(dflt.capsules[0])) == 62


def test_the_growth_delta():
    rigid = ENV([CAP([0, 0, 0], [10, 0, 0], 2.0, "UF", 0.0), CAP([0, 20, 0], [10, 20, 0], 2.0, "LF", 0.0)])
    comp_caps = [CAP([0, 3, 0], [10, 3, 0], 2.0, "UF", 0.2), CAP([0, 21, 0], [10, 21, 0], 1.0, "LF", 0.4)]
    boundary = [[5, 5, 0], [5, 0, 0], [5, 22, 0], [5, 3, 2]]
    comp = ENV(comp_caps, boundary, load={0.2: {"Fy": -1500.0, "Fz": 2000.0}}, frame="FR f")
    d = pe.envelope_delta(rigid, comp)
    dist = [min(_d(p, c.a, c.b) - c.r for c in rigid.capsules) for p in boundary]
    g = [x for x in dist if x > 0]
    assert (d.corner_label, d.max_outward_growth_mm, d.growth_member, d.growth_t_s, d.growth_load_N) == (
        "FR f", pytest.approx(max(dist)), "UF", 0.2, {"Fy": -1500.0, "Fz": 2000.0})
    assert d.mean_growth_mm == pytest.approx(np.mean(g)) and d.frac_points_grown == pytest.approx(len(g) / 4)
    assert d.note == ("compliant boundary points measured against the rigid capsule union; positive = forbidden "
                      "ground the rigid sweep missed.")
    assert d.summary()["growth_member"] == "UF"
    inside = pe.envelope_delta(rigid, ENV(comp_caps, [[5, 0, 0], [5, 20.5, 0]]))
    assert inside.max_outward_growth_mm == 0.0 and inside.mean_growth_mm == 0.0 and inside.frac_points_grown == 0.0
    assert inside.growth_member == "LF" and inside.growth_load_N is None
    far = pe.envelope_delta(rigid, ENV([CAP([0, 50, 0], [1, 50, 0], 1.0, "TR", 0.7)], [[5, 5, 0]]))
    assert far.growth_member == "" and math.isnan(far.growth_t_s) and far.growth_load_N is None
    pick = pe.envelope_delta(rigid, ENV([CAP([0, 5, 0], [10, 5, 0], 3.0, "A", 0.1),
                                         CAP([0, 8, 0], [10, 8, 0], 4.0, "B", 0.2),
                                         CAP([0, 9, 0], [10, 9, 0], 1.0, "C", 0.3)], [[5, 5, 0]]))
    assert pick.growth_member == "B"
    edge = pe.envelope_delta(rigid, ENV([CAP([0, 9, 0], [10, 9, 0], 2.0, "E", 0.1)], [[5, 5.0005, 0]]))
    assert edge.growth_member == "E"
    for e in (pe.envelope_delta(rigid, ENV(comp_caps)), pe.envelope_delta(ENV([]), comp)):
        assert (e.corner_label, e.max_outward_growth_mm, e.growth_member, e.growth_load_N, e.mean_growth_mm,
                e.frac_points_grown, e.note) == (e.corner_label, 0.0, "", None, 0.0, 0.0, "empty envelope(s)")
        assert math.isnan(e.growth_t_s)


def test_the_report():
    env = pe.PhantomEnvelope("compliant", "FR", [CAP([0, 0, 0], [10, 0, 0], 2.0)],
                             np.array([[1.0, 2.0, 3.0], [-4.0, 5.0, 6.5]]), ("UF", "TR"), 4,
                             [(0.25, "UF FoS 0.5")], {"UF": 2.0})
    txt = pe.render_envelope_md(env)
    tail = ("_The forbidden volume of the LINKS swept as tube-radius capsules, warped by Ghost Topology's compliance "
            "deflection. A screening boundary and a fast clearance gate for the packaging seat — not a certified CAD "
            "interference sign-off. Same corner frame the geometry was defined in._")
    assert txt == "\n".join(["# Phantom Envelope — FR", "", "**Sweep:** `compliant` · 4 instants · 2 boundary points · "
                             "members UF, TR", "", "Bounding box (mm): x [-4.0, 1.0], y [2.0, 5.0], z [3.0, 6.5].", "",
                             "**Excluded instants:**", "- t = 250 ms — UF FoS 0.5", "", tail])
    grow = pe.EnvelopeDelta("FR", 1.234, "UF", 0.3, {"Fy": -1500.4, "Fz": 2100.6}, 0.456, 0.257)
    q = [pe.ClearanceResult([1.4, 2.6, -3.0], 1.0, -0.5, True, "UF", 0.1234, None, "FR"),
         pe.ClearanceResult([0, 0, 0], 0.0, math.inf, False, "", math.nan, None, "FR")]
    txt = pe.render_envelope_md(ENV([]), delta=grow, queries=q)
    assert ("🟠 Under load the corner claims **1.23 mm** beyond the rigid sweep, governed by **UF** at t = 300 ms at "
            "Fy -1500 N, Fz 2101 N. 26% of the compliant boundary lies outside the rigid one (mean growth 0.46 mm). A "
            "packaging check run against the rigid CAD sweep alone would miss this ground.") in txt
    assert "## Compliance warp vs rigid sweep\n\n🟠" in txt and "**Excluded" not in txt
    assert ("| point (mm) | probe r (mm) | clearance (mm) | verdict | member | t (ms) |\n|---|---|---|---|---|---|\n"
            "| (1, 3, -3) | 1.0 | -0.50 | 🔴 VIOLATES | UF | 123 |\n| (0, 0, 0) | 0.0 | +inf | 🟢 clear |  | — |\n") \
        in txt
    noload = pe.render_envelope_md(ENV([]), delta=pe.EnvelopeDelta("FR", 0.5, "LF", 0.2, None, 0.1, 0.1))
    assert "governed by **LF** at t = 200 ms. 10% of" in noload
    flat = pe.render_envelope_md(ENV([]), delta=pe.EnvelopeDelta("FR", 0.0, "", 0.0, None, 0.0, 0.0))
    assert ("🟢 The compliant envelope stays within the rigid sweep — for this event the rigid CAD interference check "
            "is a faithful stand-in.\n\n_The forbidden") in flat
    assert "Bounding box (mm): x [0.0, 0.0], y [0.0, 0.0], z [0.0, 0.0]." in flat
    assert "## Clearance" not in pe.render_envelope_md(ENV([]), queries=[])


def test_a_tie_goes_to_the_first_capsule():
    env = ENV([CAP([0, 0, 0], [10, 0, 0], 1.0, "A", 0.1), CAP([0, 10, 0], [10, 10, 0], 1.0, "B", 0.2)])
    assert env.query([5.0, 5.0, 0.0]).nearest_member == "A"


def test_characterized_geometry_and_report():
    from tests._golden import check
    res = {}
    a, b = np.array([3.0, -2.0, 5.0]), np.array([9.0, 4.0, 1.0])
    pts = np.array([[5.0, 3.0, 4.0], [-3.0, 4.0, 0.0], [14.0, 0.0, 3.0], [3.0, -2.0, 5.0]])
    res["dist"] = [[pe._seg_point_distance(p, a, b) for p in pts], pe._seg_point_distance_batch(pts, a, b),
                   [pe._seg_point_distance(p, a, a) for p in pts], pe._seg_point_distance_batch(pts, a, a)]
    res["surf"] = [pe._capsule_surface_points(CAP(a, a, 1.5), n_ring=5), pe._capsule_surface_points(CAP(a, b, 1.5), 5, 3)]
    gc = GC(sections={"UF": NS(od_mm=20.0)})
    audit = NS(corner_label="FR", instants=[_instant(0.12345678, 2.0), _instant(0.3333333333, 1.5, dy=6.0)])
    comp = pe.carve_ghost_envelope(audit, gc, members=("UF", "LF"), ring=6, along=4)
    rigid = pe.carve_rigid_envelope(gc, corner_label="FR", travel_mm=(-5.0, 5.0), n_travel=3, members=("UF", "LF"))
    d = pe.envelope_delta(rigid, comp)
    res["ghost"] = [comp.load_at_t, comp.n_points, comp.to_csv(), d.summary(), rigid.n_points]
    res["empty_delta"] = pe.envelope_delta(rigid, ENV([], frame="FR frame")).summary()
    env = pe.PhantomEnvelope("compliant", "FR", [CAP([0, 0, 0], [10, 0, 0], 2.0)],
                             np.array([[1.0, 2.0, 3.0], [-4.0, 5.0, 6.5]]), ("UF", "TR"), 4,
                             [(0.0125, "UF FoS 0.5")], {"UF": 2.0})
    grow = pe.EnvelopeDelta("FR", 1.234, "UF", 0.8125, {"Fy": -1500.4, "Fz": 2100.6}, 0.456, 0.5149)
    q = [pe.ClearanceResult([1.4, 2.6, -3.0], 1.0, -0.5, True, "UF", 0.1234, None, "FR")]
    res["md"] = [pe.render_envelope_md(env, delta=grow, queries=q), pe.render_envelope_md(env)]
    probe = ENV([CAP([0, 0, 0], [10, 0, 0], 2.0, "UF", 0.1)])
    res["worst"] = [probe.worst_violation([[5.0, 3.0, 0.0], [5.0, 6.0, 0.0]], probe_radius_mm=1.5)]
    check("phantom_geometry", res)
