"""suspension/topology.py on mechanisms small enough to solve by hand: a pendulum arm swinging about a ground pivot
(closed-form position, carried points rotated rigidly), a rack-steered point, every constraint residual written out,
the body frame and its collinear fallbacks, the compile step's frozen lengths and offsets, the warm-started sweep and
the builder."""
import math

import numpy as np
import pytest

from suspension import topology as tp


def test_vector_helpers():
    assert tp._unit([3.0, 4.0, 0.0]) == pytest.approx([0.6, 0.8, 0.0])
    z = tp._unit([1e-13, 0.0, 0.0])
    assert z.tolist() == [1e-13, 0.0, 0.0]
    assert tp._vec([[1, 2, 3]]).tolist() == [1.0, 2.0, 3.0]
    with pytest.raises(ValueError) as e:
        tp._vec([1, 2])
    assert str(e.value) == "expected a 3-vector, got shape (2,)"


def test_points():
    p = tp.Point("a", [1, 2, 3])
    assert (p.role, p.carrier, p.pos.tolist()) == ("free", None, [1.0, 2.0, 3.0])
    assert tp.Point("g", (0, 0, 0), role="ground").role == "ground"
    assert tp.Point("c", (0, 0, 0), role="carried", carrier="b").carrier == "b"
    with pytest.raises(ValueError) as e:
        tp.Point("x", (0, 0, 0), role="floating")
    assert str(e.value) == "point x: bad role 'floating'"


def test_the_body_frame_and_its_fallbacks():
    R, o = tp.Body._frame_from_points([np.array([1.0, 1, 1]), np.array([4.0, 5, 1]), np.array([1.0, 1, 7])])
    x = np.array([0.6, 0.8, 0.0])
    zz = np.cross(x, [0, 0, 6.0])
    zz /= np.linalg.norm(zz)
    assert o.tolist() == [1.0, 1.0, 1.0] and R[:, 0] == pytest.approx(x) and R[:, 2] == pytest.approx(zz)
    assert R[:, 1] == pytest.approx(np.cross(zz, x)) and R.T @ R == pytest.approx(np.eye(3))
    R, _ = tp.Body._frame_from_points([np.zeros(3), np.array([1.0, 0, 0]), np.array([2.0, 0, 0])])
    assert R[:, 2] == pytest.approx(tp._unit(np.cross([1.0, 0, 0], [0, 0, 1.0])))
    R, _ = tp.Body._frame_from_points([np.zeros(3), np.array([0, 0, 2.0]), np.array([0, 0, 5.0])])
    assert R[:, 2] == pytest.approx(tp._unit(np.cross([0, 0, 1.0], [0, 1.0, 0]))) and R[:, 0] == pytest.approx(
        [0, 0, 1.0])
    pts = [np.zeros(3), np.array([1.0, 0, 0]), np.array([0, 1.0, 0])]
    o_in = pts[0]
    tp.Body._frame_from_points(pts)
    assert o_in.tolist() == [0.0, 0.0, 0.0]


def _rot(axis, ang):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K


def test_a_body_carries_points_rigidly():
    world = {"a": np.array([10.0, 0, 0]), "b": np.array([30.0, 5, 2]), "c": np.array([12.0, 20, -4])}
    b = tp.Body("k", ["a", "b", "c"])
    carried = {"w": np.array([40.0, -7.0, 13.0]), "q": np.array([0.0, 0.0, 0.0])}
    b.freeze(world.__getitem__, carried)
    Rm, t = _rot([1, 2, 3], 0.7), np.array([5.0, -3.0, 8.0])
    moved = {k: Rm @ v + t for k, v in world.items()}
    R, o = b.pose(moved.__getitem__)
    for nm, p in carried.items():
        assert b.carry(nm, R, o) == pytest.approx(Rm @ p + t, abs=1e-9)
    R0, o0 = b._R0, b._o0
    assert o0.tolist() == [10.0, 0.0, 0.0] and b._local["w"] == pytest.approx(R0.T @ (carried["w"] - o0))


class Ctx:
    def __init__(self, pts):
        self.P = lambda n: np.asarray(pts[n], float)
        self.body_pose = None


def test_constraint_residuals():
    c = Ctx({"a": [3.0, 4.0, 0.0], "b": [0.0, 0.0, 0.0], "p": [2.0, 3.0, 5.0], "base": [1.0, 1.0, 1.0]})
    assert tp.Constraint().n_eq == 1 and tp.Constraint().members() == []
    ln = tp.Link(a="a", b="b", length=4.0)
    assert ln.n_eq == 1 and ln.residual(c).tolist() == [1.0]
    assert ln.members() == [("a->b", "a", "b")] and tp.Link(a="a", b="b", label="L").members() == [("L", "a", "b")]
    co = tp.Coincident(a="a", b="b")
    assert co.n_eq == 3 and co.residual(c).tolist() == [3.0, 4.0, 0.0] and co.members() == []
    ol = tp.OnLine(p="p", base="base", axis=np.array([0.0, 0.0, 2.0]))
    assert ol.n_eq == 2
    e1, e2 = ol._frame()
    d = np.array([0, 0, 1.0])
    assert e1 == pytest.approx(tp._unit(np.cross(d, [1.0, 0, 0]))) and e2 == pytest.approx(np.cross(d, e1))
    v = np.array([1.0, 2.0, 4.0])
    assert ol.residual(c) == pytest.approx([v @ e1, v @ e2])
    olx = tp.OnLine(p="p", base="base", axis=np.array([3.0, 0.1, 0.0]))
    e1, _ = olx._frame()
    dx = tp._unit([3.0, 0.1, 0.0])
    assert e1 == pytest.approx(tp._unit(np.cross(dx, [0, 1.0, 0])))
    edge = tp.OnLine(p="p", base="base", axis=np.array([0.9, math.sqrt(1 - 0.81), 0.0]))
    assert edge._frame()[0] == pytest.approx(tp._unit(np.cross(edge.axis, [1.0, 0, 0])))
    ip = tp.InPlane(p="p", base="base", normal=np.array([0.0, 2.0, 0.0]))
    assert ip.n_eq == 1 and ip.residual(c).tolist() == [2.0]
    ip.offset = 0.5
    assert ip.residual(c).tolist() == [1.5]
    rv = tp.Revolute(p="p", base="base", axis=np.array([0.0, 0.0, 3.0]), radius=2.0)
    assert rv.n_eq == 2 and rv.residual(c) == pytest.approx([math.sqrt(21) - 2.0, 4.0])
    rv.offset = 1.0
    assert rv.residual(c) == pytest.approx([math.sqrt(21) - 2.0, 3.0])
    dz = tp.DriveZ(p="p", z0=2.0, travel=1.5)
    assert dz.n_eq == 1 and dz.residual(c).tolist() == [1.5] and tp.DriveZ().travel == 0.0
    rk = tp.RackTranslation(p="p", base_pos=np.array([2.0, 1.0, 5.0]), delta=0.5)
    assert rk.n_eq == 3 and rk.residual(c).tolist() == [0.0, 1.5, 0.0] and tp.RackTranslation().delta == 0.0
    rk2 = tp.RackTranslation(p="p", base_pos=np.array([2.0, 3.0, 5.0]), axis=np.array([2.0, 0, 0]), delta=-1.0)
    assert rk2.residual(c).tolist() == [1.0, 0.0, 0.0]
    ax = tp.AxleRoll(end_a="a", end_b="b", length=2.0)
    assert ax.n_eq == 1 and ax.residual(c).tolist() == [3.0]


# ------------------------------------------------------------------ a pendulum arm
L = 100.0


def _arm(spin=None, cam=0.0, toe=0.0, steer=False):
    b = tp.MechanismBuilder("arm")
    b.ground("G", [0, 0, 0]).ground("Y", [0, 50, 0]).free("P", [L, 0, 0])
    b.body("arm", ["G", "P", "Y"])
    b.carried("W", "arm", [L, 0, 30]).carried("C", "arm", [L, 10, -20]).carried("S", "arm", [L, 40, 30])
    b.link("P", "G").in_plane("P", "G", normal=[0, 1, 0])
    if steer:
        b.free("R", [0, 200, 50])
    return b.finish(carrier="arm", wheel_center="W", contact_patch="C", drive_point="P", spin_axis_pts=spin,
                    static_camber=cam, static_toe=toe, steer_point="R" if steer else None,
                    steer_axis=np.array([0, 0, 1.0]) if steer else None)


def _expect(t, p0):
    x = math.sqrt(L * L - t * t)
    c, s = x / L, t / L
    R = np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])
    return R @ np.asarray(p0, float), R


def test_the_compile_step():
    m = _arm()
    assert m._compiled and m.label == "arm" and m.meta == {} and m._free_names == ["P"]
    link = next(c for c in m.constraints if isinstance(c, tp.Link))
    assert link.length == L and link.label == "P->G"
    ip = next(c for c in m.constraints if isinstance(c, tp.InPlane))
    assert ip.offset == 0.0
    assert isinstance(m.constraints[-1], tp.DriveZ) and m._drive.z0 == 0.0 and m._rack is None
    assert sum(isinstance(c, tp.DriveZ) for c in m.compile().constraints) == 1
    cam, toe = math.radians(-2.0), math.radians(0.5)
    m2 = _arm(cam=-2.0, toe=0.5)
    spin0 = tp._unit(np.array([math.sin(toe), math.cos(toe) * math.cos(cam), -math.sin(cam)]))
    assert m2.bodies["arm"]._R0 @ m2._spin_local == pytest.approx(spin0)
    m3 = _arm(spin=("W", "S"))
    assert m3.bodies["arm"]._R0 @ m3._spin_local == pytest.approx([0, 1.0, 0])
    b = tp.MechanismBuilder()
    b.ground("g", [0, 0, 0]).free("p", [0, 30, 40]).ground("h", [5, 0, 0]).free("q", [1, 2, 3])
    b.body("k", ["p", "g", "h"]).carried("w", "k", [0, 31, 40])
    b.revolute("p", "g", axis=[0, 0, 2]).axle_roll("p", "q").on_line("q", "g", axis=[0, 0, 1])
    b.constraint(tp.DriveZ(p="q", z0=99.0)).coincident("q", "q")
    m4 = b.finish(carrier="k", wheel_center="w", contact_patch="w", drive_point="p")
    rv = next(c for c in m4.constraints if isinstance(c, tp.Revolute))
    assert (rv.radius, rv.offset) == (50.0, 40.0)
    ar = next(c for c in m4.constraints if isinstance(c, tp.AxleRoll))
    assert ar.length == pytest.approx(math.sqrt(1 + 28 ** 2 + 37 ** 2))
    ol = next(c for c in m4.constraints if isinstance(c, tp.OnLine))
    assert ol._e1 is not None and ol._e1 == pytest.approx(ol._frame()[0])
    drives = [c for c in m4.constraints if isinstance(c, tp.DriveZ)]
    assert len(drives) == 1 and drives[0].p == "p" and drives[0].z0 == 40.0 and m4.label == "custom"


def test_the_arm_solves_in_closed_form():
    m = _arm()
    for t in (0.0, 25.0, -40.0, 70.0):
        r = m.solve(t)
        P, Rw = _expect(t, [L, 0, 0])
        assert r["positions"]["P"] == pytest.approx(P, abs=1e-7)
        assert r["wheel_center"] == pytest.approx(_expect(t, [L, 0, 30])[0], abs=1e-6)
        assert r["contact_patch"] == pytest.approx(_expect(t, [L, 10, -20])[0], abs=1e-6)
        assert r["carrier_o"].tolist() == [0.0, 0.0, 0.0] and r["carrier_R"] @ [1, 0, 0] == pytest.approx(P / L,
                                                                                                         abs=1e-9)
        assert r["spin_axis"] == pytest.approx([0, 1.0, 0], abs=1e-9)
        assert r["max_residual"] < 1e-8 and r["converged"] is True
        assert (r["travel"], r["steer"]) == (t, 0.0) and r["free"] == pytest.approx(P, abs=1e-7)
        assert r["positions"]["W"] is not m.points["W"].pos
    tilted = _arm(cam=-3.0).solve(40.0)
    cam = math.radians(-3.0)
    spin0 = np.array([0.0, math.cos(cam), -math.sin(cam)])
    assert tilted["spin_axis"] == pytest.approx(_expect(40.0, spin0)[1] @ spin0, abs=1e-9)


def test_an_impossible_travel_is_not_converged():
    r = _arm().solve(160.0)
    assert r["converged"] is False and r["max_residual"] > 10.0


def test_an_underdetermined_mechanism_still_solves():
    b = tp.MechanismBuilder("loose")
    b.ground("G", [0, 0, 0]).ground("Y", [0, 50, 0]).free("P", [L, 0, 0])
    b.body("arm", ["G", "P", "Y"]).carried("W", "arm", [L, 0, 30])
    b.link("P", "G")
    m = b.finish(carrier="arm", wheel_center="W", contact_patch="W", drive_point="P")
    r = m.solve(30.0)
    p = r["positions"]["P"]
    assert r["converged"] and np.linalg.norm(p) == pytest.approx(L, abs=1e-6) and p[2] == pytest.approx(30.0)


def test_the_rack_moves_the_steer_point():
    m = _arm(steer=True)
    assert m._rack is not None and m._rack.p == "R" and m._rack.axis.tolist() == [0.0, 0.0, 1.0]
    assert m._rack.base_pos.tolist() == [0.0, 200.0, 50.0] and m._free_names == ["P", "R"]
    r = m.solve(10.0, steer=7.5)
    assert r["positions"]["R"] == pytest.approx([0.0, 200.0, 57.5], abs=1e-8) and r["steer"] == 7.5
    assert sum(isinstance(c, tp.RackTranslation) for c in m.compile().constraints) == 1
    b = tp.MechanismBuilder()
    b.ground("G", [0, 0, 0]).ground("Y", [0, 50, 0]).free("P", [L, 0, 0]).free("R", [5, 5, 5])
    b.body("arm", ["G", "P", "Y"]).carried("W", "arm", [L, 0, 30])
    b.link("P", "G").in_plane("P", "G", normal=[0, 1, 0])
    m2 = b.finish(carrier="arm", wheel_center="W", contact_patch="W", drive_point="P", steer_point="R")
    assert m2._rack.axis.tolist() == [0.0, 1.0, 0.0]
    assert m2.solve(0.0, steer=2.0)["positions"]["R"] == pytest.approx([5.0, 7.0, 5.0], abs=1e-8)


def test_solve_compiles_on_demand_and_takes_a_seed():
    raw = tp.Mechanism(points={"G": tp.Point("G", [0, 0, 0], "ground"), "Y": tp.Point("Y", [0, 50, 0], "ground"),
                               "P": tp.Point("P", [L, 0, 0]),
                               "W": tp.Point("W", [L, 0, 30], "carried", carrier="arm")},
                       bodies={"arm": tp.Body("arm", ["G", "P", "Y"])},
                       constraints=[tp.Link(a="P", b="G"), tp.InPlane(p="P", base="G", normal=np.array([0, 1.0, 0]))],
                       carrier_body="arm", wheel_center="W", contact_patch="W", drive_point="P")
    assert raw._compiled is False and raw.label == "custom" and raw.steer_point is None
    r = raw.solve(20.0)
    assert raw._compiled and r["positions"]["P"] == pytest.approx(_expect(20.0, [L, 0, 0])[0], abs=1e-7)
    flip = raw.solve(20.0, seed=np.array([-L, 0.0, 0.0]))
    assert flip["positions"]["P"][0] < 0                                   # the seed picks the branch
    raw2 = tp.Mechanism(points=raw.points, bodies=raw.bodies, constraints=list(raw.constraints), carrier_body="arm",
                        wheel_center="W", contact_patch="W", drive_point="P")
    raw2.points["P"].pos = np.array([L, 0.0, 0.0])
    s = raw2.sweep(-20, 20, 5)
    assert raw2._compiled and len(s) == 5


def test_the_sweep():
    m = _arm()
    out = m.sweep(-60.0, 60.0, 7)
    assert [r["travel"] for r in out] == pytest.approx([-60, -40, -20, 0, 20, 40, 60])
    for r in out:
        assert r["positions"]["P"] == pytest.approx(_expect(r["travel"], [L, 0, 0])[0], abs=1e-6)
        assert r["converged"]
    assert m.points["P"].pos == pytest.approx([L, 0, 0], abs=1e-8)
    even = m.sweep(-30.0, 30.0, 4)
    assert [r["travel"] for r in even] == pytest.approx([-30, -10, 10, 30])
    assert all(r["positions"]["P"][0] > 0 for r in even)
    deep = m.sweep(-95.0, 95.0, 39)
    assert all(r["positions"]["P"][0] > 0 for r in deep)                    # warm starts hold the branch
    st = _arm(steer=True).sweep(-10.0, 10.0, 3, steer=4.0)
    assert all(r["steer"] == 4.0 and r["positions"]["R"][2] == pytest.approx(54.0, abs=1e-8) for r in st)
    one_side = m.sweep(10.0, 30.0, 3)
    assert [r["travel"] for r in one_side] == pytest.approx([10, 20, 30])
    assert one_side[0]["positions"]["P"] == pytest.approx(_expect(10.0, [L, 0, 0])[0], abs=1e-6)
    defaults = m.sweep()
    assert len(defaults) == 41 and defaults[0]["travel"] == -30.0 and defaults[-1]["travel"] == 30.0


def test_force_members_point_outboard_first():
    b = tp.MechanismBuilder()
    b.ground("g1", [0, 0, 0]).ground("g2", [1, 0, 0]).free("f1", [0, 1, 0]).free("f2", [0, 0, 1])
    b.ground("Y", [0, 5, 0]).body("k", ["f1", "f2", "g1"]).carried("w", "k", [0, 1, 1])
    b.link("f1", "g1", label="A").link("g2", "f2", label="B").link("f1", "f2").link("g1", "g2", label="D")
    b.coincident("f1", "f1")
    m = b.finish(carrier="k", wheel_center="w", contact_patch="w", drive_point="f2")
    assert m.force_members() == [("A", "f1", "g1"), ("B", "f2", "g2"), ("f1->f2", "f1", "f2"), ("D", "g1", "g2")]
    bw = tp.MechanismBuilder()
    bw.ground("g", [0, 0, 0]).carried("c", "k", [0, 1, 0]).free("f", [1, 0, 0]).free("h", [0, 0, 1])
    bw.body("k", ["f", "h", "g"]).link("g", "c", label="E").link("c", "f", label="F")
    mw = bw.finish(carrier="k", wheel_center="c", contact_patch="c", drive_point="f")
    assert mw.force_members() == [("E", "c", "g"), ("F", "c", "f")]


def test_the_builder():
    b = tp.MechanismBuilder("x")
    assert (b.label, b.points, b.bodies, b.constraints) == ("x", {}, {}, [])
    assert tp.MechanismBuilder().label == "custom"
    for meth, args in (("ground", ("a", [0, 0, 0])), ("free", ("b", [1, 0, 0])), ("carried", ("c", "k", [0, 1, 0])),
                       ("body", ("k", ("a", "b", "c"))), ("link", ("a", "b")), ("coincident", ("a", "b")),
                       ("on_line", ("b", "a", [1, 0, 0])), ("in_plane", ("b", "a", [0, 0, 1])),
                       ("revolute", ("b", "a", [0, 0, 1])), ("axle_roll", ("a", "b")),
                       ("constraint", (tp.Link(a="a", b="c"),))):
        assert getattr(b, meth)(*args) is b
    assert [p.role for p in b.points.values()] == ["ground", "free", "carried"] and b.points["c"].carrier == "k"
    assert b.bodies["k"].defining_points == ["a", "b", "c"] and isinstance(b.bodies["k"].defining_points, list)
    kinds = [type(c).__name__ for c in b.constraints]
    assert kinds == ["Link", "Coincident", "OnLine", "InPlane", "Revolute", "AxleRoll", "Link"]
    lk, co, ol, ip, rv, ar, lk2 = b.constraints
    assert (lk.a, lk.b, lk.label) == ("a", "b", "a->b") and (co.a, co.b) == ("a", "b")
    assert (ol.p, ol.base, ol.axis.tolist()) == ("b", "a", [1.0, 0.0, 0.0])
    assert (ip.p, ip.base, ip.normal.tolist()) == ("b", "a", [0.0, 0.0, 1.0])
    assert (rv.p, rv.base, rv.axis.tolist()) == ("b", "a", [0.0, 0.0, 1.0]) and (ar.end_a, ar.end_b) == ("a", "b")
    assert b.link("b", "c", label="named").constraints[-1].label == "named"
    m = _arm(spin=("W", "S"), cam=1.0, toe=2.0)
    assert (m.carrier_body, m.wheel_center, m.contact_patch, m.drive_point, m.spin_axis_pts, m.static_camber,
            m.static_toe) == ("arm", "W", "C", "P", ("W", "S"), 1.0, 2.0)
    b2 = tp.MechanismBuilder("base")
    b2.ground("G", [0, 0, 0]).ground("Y", [0, 50, 0]).free("P", [L, 0, 0]).body("arm", ["G", "P", "Y"])
    b2.carried("W", "arm", [L, 0, 30]).link("P", "G").in_plane("P", "G", normal=[0, 1, 0])
    m2 = b2.finish(carrier="arm", wheel_center="W", contact_patch="W", drive_point="P", label="named",
                   meta={"k": 1})
    assert m2.label == "named" and m2.meta == {"k": 1} and m2._compiled


def test_characterized_builder_and_sweep():
    from tests._golden import check
    res = {}
    m = _arm(steer=True)
    res["names"] = {n: [p.name, p.role, p.carrier] for n, p in m.points.items()}
    res["bodies"] = {n: [b.name, b.defining_points] for n, b in m.bodies.items()}
    sw = m.sweep(-10.0, 10.0, 5, steer=3.0)
    res["sweep"] = [[r["travel"], r["steer"], r["positions"]["P"], r["positions"]["R"], r["spin_axis"]] for r in sw]
    res["after"] = {n: p.pos for n, p in m.points.items()}
    plain = tp.MechanismBuilder("plain")
    plain.ground("G", [0, 0, 0]).ground("Y", [0, 50, 0]).free("P", [L, 0, 0])
    plain.body("arm", ["G", "P", "Y"]).carried("W", "arm", [L, 0, 30])
    plain.link("P", "G").in_plane("P", "G", normal=[0, 1, 0])
    pm = plain.finish(carrier="arm", wheel_center="W", contact_patch="W", drive_point="P")
    res["default_spin"] = [pm._spin_local, pm.solve(5.0)["spin_axis"]]
    res["frames"] = [tp.Body._frame_from_points([np.zeros(3), np.array([0, 0, 2.0]), np.array([0, 0, 5.0])]),
                     tp.Body._frame_from_points([np.zeros(3), np.array([3.0, 0, 0]), np.array([6.0, 0, 0])])]
    res["members"] = m.force_members()
    check("topology_builder", res)


class _BodyHeight(tp.Constraint):
    """A custom constraint that reads a body pose through the solve context."""
    def __init__(self, body, z):
        self.n_eq, self.body, self.z = 1, body, z

    def residual(self, ctx):
        R, o = ctx.body_pose(self.body)
        return np.array([o[2] + 0.1 * R[0, 0] - self.z])


def test_characterized_two_bodies_and_custom_constraints():
    from tests._golden import check
    b = tp.MechanismBuilder("two")
    b.ground("G", [10, 20, 30]).ground("Y", [10, 70, 30]).ground("H", [40, 20, 90]).ground("K", [45, 75, 95])
    b.free("P", [10 + L, 20, 30]).free("Q", [40 + L, 25, 92]).free("M", [70, 40, 60])
    b.body("arm", ["G", "P", "Y"]).body("bar", ["H", "Q", "K"])
    b.carried("W", "arm", [10 + L, 20, 60]).carried("C", "arm", [10 + L, 30, 10]).carried("E", "bar", [40 + L, 30, 120])
    b.link("P", "G").in_plane("P", "G", normal=[0, 1, 0]).link("Q", "H").in_plane("Q", "H", normal=[0, 1, 0.2])
    b.axle_roll("W", "E").coincident("M", "M").revolute("M", "G", axis=[0, 0, 1])
    b.constraint(_BodyHeight("arm", 31.0))
    m = b.finish(carrier="arm", wheel_center="W", contact_patch="C", drive_point="P")
    res = {"static": {n: p.pos for n, p in m.points.items()}}
    res["solves"] = [[r["positions"], r["max_residual"] < 1e9] for r in (m.solve(4.0), m.solve(-3.0, steer=0.0))]
    res["resid"] = m._residuals(m._get_free() + 0.01)
    res["dflt_sweep"] = [len(_arm().sweep()), [r["travel"] for r in _arm().sweep()][:3]]
    check("topology_two_bodies", res)
