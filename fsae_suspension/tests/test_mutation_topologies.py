"""Characterization suites for suspension/topologies.py and suspension/adapter.py (mutation gate); see
tests/_golden.py. Every template's example, built and solved in bump, droop and steer, with its labels, metadata,
points, bodies, constraints and force members; the builders' own defaults and options (both solid-axle locators, a
bad one, spring perches, a steered multilink and one without a steer link, a free-form mechanism); and the generic
kinematics on every example: states, sweeps, anti-dive and anti-squat, swing arm, motion ratio, wheel rate,
render segments, named points and the hardpoint shim."""
import math

import numpy as np
import pytest

from suspension import topologies as T
from suspension.adapter import GenericKinematics
from tests._golden import plain
from tests._golden_lazy import check_lazy

REL = 1e-6          # the generic kernel solves iteratively


def _mech(m):
    out = {k: plain(getattr(m, k)) for k in ("label", "carrier_body", "wheel_center", "contact_patch", "drive_point",
                                             "spin_axis_pts", "static_camber", "static_toe", "steer_point",
                                             "steer_axis", "meta")}
    out["points"] = {n: [p.role, plain(p.pos), getattr(p, "body", None)] for n, p in sorted(m.points.items())}
    out["bodies"] = {n: plain(getattr(b, "defining", getattr(b, "defining_points", None))) for n, b in
                     sorted(m.bodies.items())}
    out["constraints"] = [[type(c).__name__, plain({k: v for k, v in vars(c).items()})] for c in m.constraints]
    out["members"] = plain(m.force_members())
    return out


def _solve(m):
    out = {}
    for t, s in ((0.0, 0.0), (15.0, 0.0), (-15.0, 0.0), (0.0, 3.0)):
        r = m.solve(t, steer=s)
        out[f"{t}.{s}"] = {k: plain(v) for k, v in sorted(r.items()) if k != "positions"} | {
            "positions": {n: plain(p) for n, p in sorted(r["positions"].items())}}
    return out


def _free_form(**kw):
    return T.from_links(
        chassis_points={"a": [0, 200, 100], "b": [200, 200, 100], "c": [0, 220, 300], "d": [200, 220, 300]},
        free_points={"lo": [0, 560, 110], "uo": [0, 540, 320]},
        carried_points={"wc": ("knuckle", [0, 600, 228]), "cp": ("knuckle", [0, 605, 0]),
                        "tro": ("knuckle", [100, 560, 170]), "x": ("other", [0, 0, 0])},
        links=[("a", "lo"), ("b", "lo", "lower rear"), ("c", "uo"), ("d", "uo"), ("lo", "uo", "kingpin")],
        carrier_body="knuckle", carrier_defining=["lo", "uo", "tro"], wheel_center="wc", contact_patch="cp",
        drive_point="lo", **({"label": "free"} | kw))


def _finish_calls():
    from suspension import topology as TP
    calls = []
    real = TP.MechanismBuilder.finish

    def spy(self, **kw):
        calls.append([self.label, {k: plain(v) for k, v in sorted(kw.items())}])
        return real(self, **kw)
    TP.MechanismBuilder.finish = spy
    try:
        for name in T.list_templates():
            if name != "from_links":
                T.example(name)
        yield "finish.examples", calls[:]
    finally:
        TP.MechanismBuilder.finish = real


def _topologies():
    yield from _finish_calls()
    yield "v.nested", list(T._v([[1, 2, 3]]).shape)
    three = [([-60, 210, 130], [-10, 560, 120]), ([-40, 250, 330], [0, 545, 320]), ([130, 240, 200], [120, 560, 190])]
    m3 = T.multilink(three, [0, 1, 2], [0, 600, 228], [0, 605, 0], label="three")
    yield "multilink.three", [m3.drive_point, m3.label, sorted(m3.points)]
    low_last = [([-60, 210, 330], [-10, 560, 320]), ([150, 215, 328], [60, 565, 318]), ([-40, 250, 130], [0, 545, 120]),
                ([160, 255, 132], [70, 548, 122]), ([130, 240, 200], [120, 560, 190])]
    yield "multilink.drive", T.multilink(low_last, [0, 2, 4], [0, 600, 228], [0, 605, 0]).drive_point
    for bad in ((three[:2], [0, 1, 1]), (three, [0, 1, 1]), (three, [0, 1, 7])):
        with pytest.raises((ValueError, IndexError)) as e:
            T.multilink(bad[0], bad[1], [0, 600, 228], [0, 605, 0])
        yield f"multilink.bad.{len(bad[0])}.{bad[1][-1]}", str(e.value)
    yield "list", T.list_templates()
    yield "v", plain(T._v([1, 2, 3]))
    for name in T.list_templates():
        if name == "from_links":
            with pytest.raises(ValueError) as e:
                T.example(name)
            yield "example.from_links", str(e.value)
            continue
        m = T.example(name)
        yield f"example.{name}", _mech(m)
        yield f"solve.{name}", _solve(m.compile())
    links = [([-60, 210, 130], [-10, 560, 120]), ([150, 215, 128], [60, 565, 118]), ([-40, 250, 330], [0, 545, 320]),
             ([160, 255, 332], [70, 548, 322]), ([130, 240, 200], [120, 560, 190])]
    yield "multilink.nosteer", _mech(T.multilink(links, [0, 2, 4], [0, 600, 228], [0, 605, 0], static_camber=-2.0,
                                                 label="ml"))
    yield "watts", _mech(T.solid_axle([0, 600, 230], [0, -600, 230], links[:3],
                                       ("watts", [-50, 0, 280], [-50, 500, 300], [-50, -500, 260]),
                                       [0, 620, 230], [0, 625, 0], label="w", static_toe=0.1))
    with pytest.raises(ValueError) as e:
        T.solid_axle([0, 600, 230], [0, -600, 230], links[:3], ("leaf", 1), [0, 620, 230], [0, 625, 0])
    yield "solid.bad", str(e.value)
    yield "truck.perches", _mech(T.truck_steer_linkage([0, 900, 560], [0, 920, 360], [0, 980, 470], [160, 870, 380],
                                                       [160, 400, 380], [160, 300, 600], [0, 1010, 470],
                                                       [0, 1015, 0], spring_perches=([0, 700, 500], [0, 700, 700])))
    nolabel = T.from_links(
        chassis_points={"a": [0, 200, 100], "b": [200, 200, 100], "c": [0, 220, 300], "d": [200, 220, 300]},
        free_points={"lo": [0, 560, 110], "uo": [0, 540, 320]}, carried_points=None,
        links=[("a", "lo"), ("b", "lo"), ("c", "uo"), ("d", "uo"), ("lo", "uo")],
        carrier_body="knuckle", carrier_defining=["lo", "uo", "a"], wheel_center="lo", contact_patch="uo",
        drive_point="lo")
    yield "free.nolabel", [nolabel.label, nolabel.meta, sorted(nolabel.bodies)]
    from suspension import topology as TP
    extra = _free_form(extra_constraints=[TP.Link(a="a", b="uo")], steer_axis=("a", "b"), steer_point="tro",
                       static_toe=0.2, static_camber=-1.0)
    yield "free.extra", _mech(extra)
    ff = _free_form()
    yield "free", _mech(ff)
    yield "free.solve", _solve(ff.compile())
    yield "templates", sorted(T.TEMPLATES)


def _kin(k):
    st = k.static
    out = {"label": k.label, "static": plain(st), "sweep": plain(k.sweep(-20.0, 20.0, 5)),
           "sweep.steer": plain(k.sweep(-10.0, 10.0, 3, steer=2.0)), "solve": plain(k.solve_at_travel(12.0)),
           "solve.steer": plain(k.solve_at_travel(0.0, steer=-3.0)),
           "mr": [k.motion_ratio(), k.motion_ratio(2.0), k.motion_ratio_is_real(), k.wheel_rate(30.0)],
           "segments": plain(k.render_segments()), "segments.bump": plain(k.render_segments(k.solve_at_travel(10.0))),
           "points": {n: plain(p) for n, p in sorted(k.named_points().items())}, "hp": k.hp.as_dict()}
    for fn, args in (("anti_dive_pct", (280.0, 1530.0)), ("anti_squat_pct", (280.0, 1530.0)),
                     ("side_view_swing_arm_length", ())):
        try:
            out[fn] = plain(getattr(k, fn)(*args))
        except Exception as e:                                      # noqa: BLE001
            out[fn] = type(e).__name__
    out["anti_dive.bias"] = plain(k.anti_dive_pct(300.0, 1600.0, brake_bias_front=0.5, state=k.solve_at_travel(5.0)))
    out["anti_squat.bias"] = plain(k.anti_squat_pct(300.0, 1600.0, drive_bias_rear=0.6))
    out["svsa.state"] = plain(k._side_view_swing_arm(k.solve_at_travel(-5.0), -5.0))
    names = list(k.hp.as_dict())
    out["shim"] = [plain(getattr(k.hp, names[0])), k.hp.as_dict() == {n: p.pos.tolist() for n, p in k.mech.points.items()}]
    try:
        k.hp.no_such_point
    except AttributeError as e:
        out["shim.missing"] = str(e)
    return out


def _adapter():
    for name in T.list_templates():
        if name == "from_links":
            continue
        yield f"kin.{name}", _kin(GenericKinematics(T.example(name)))
    yield "kin.free", _kin(GenericKinematics(_free_form()))
    yield "kin.compiled", GenericKinematics(T.example("trailing_arm").compile()).label
    sp = GenericKinematics(T.example("double_wishbone"))
    pts = sp.mech.points
    sp.mech.meta["spring_points"] = ("lo", "ufi") if {"lo", "ufi"} <= set(pts) else tuple(sorted(pts)[:2])
    yield "kin.spring", [sp.motion_ratio(), sp.motion_ratio(1.0), sp.motion_ratio_is_real(), sp.wheel_rate(40.0),
                         sp.mech.meta["spring_points"]]
    k = GenericKinematics(T.example("trailing_arm"))
    yield "helpers", [plain(k._camber(np.array([0.0, 1.0, 0.1]))), plain(k._toe(np.array([0.1, 1.0, 0.0]))),
                      plain(GenericKinematics._caster_kpi(np.array([0.1, -0.2, 1.0]))),
                      plain(GenericKinematics._ic_from_velocity(np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0]),
                                                                np.array([0.0, 100.0, 0.0]), np.array([0.0, 0.0, 2.0]))),
                      plain(GenericKinematics._ic_from_velocity(np.zeros(3), np.array([0.0, 0.0, 1.0]),
                                                                np.array([0.0, 100.0, 0.0]),
                                                                np.array([0.0, 0.0, 1.0]))),
                      math.isnan(float(np.nan))]
    G = GenericKinematics
    yield "kingpin", [plain(k._kingpin_axis({"carrier_R": np.diag([1.0, 1.0, -1.0])})),
                      plain(k._kingpin_axis({"carrier_R": np.array([[1.0, 0, 0], [0, 1.0, 0], [0, 0, 0]])}))]
    r0 = {"contact_patch": np.array([0.0, 600.0, 0.0]), "wheel_center": np.array([0.0, 600.0, 230.0])}
    yield "scrub", [k._scrub(np.array([0.0, 0.1, 1e-9]), r0), k._scrub(np.array([0.0, 0.1, 2e-9]), r0)]
    yield "cvp", [plain(k._carrier_velocity_point((np.array([1.0, 2.0, 3.0]), np.array([0.0, 0.0, 1.0])), 0.0, d=2.0)),
                  plain(k._carrier_velocity_point((np.array([1.0, 2.0, 3.0]), np.array([0.0, 0.0, 1.0])), 0.0))]
    z = np.zeros(2)
    yield "ic", [plain(G._ic_from_velocity(z, z, np.array([0.0, 100.0]), np.array([0.0, 1.0]))),
                 plain(G._ic_from_velocity(z, np.array([1e-9, 0.0]), np.array([0.0, 100.0]), np.array([0.0, 1.0]))),
                 plain(G._ic_from_velocity(z, np.array([0.0, 1.0]), np.array([0.0, 100.0]), np.array([1e-9, 0.0]))),
                 plain(G._ic_from_velocity(z, np.array([0.0, 1e-4]), np.array([0.0, 100.0]),
                                           np.array([1e-6, 1.0]))),
                 plain(G._ic_from_velocity(z, np.array([0.0, 1.0]), np.array([0.0, 100.0]),
                                           np.array([1e-10, 1e-4])))]
    m = T.example("trailing_arm").compile()
    m.solve(25.0)
    yield "precompiled_moved", plain(GenericKinematics(m).static)
    yield "rocker", [k._has_rocker, len(k.sweep()), k.sweep()[0].travel, k.sweep()[-1].travel]
    r = k.mech.solve(0.0)
    r = dict(r, positions=dict(r["positions"], pushrod_outer=np.array([1.0, 2.0, 3.0])))
    yield "pushrod", plain(k._to_state(r, 0.0).pushrod_outer)
    dw = GenericKinematics(T.example("double_wishbone"))
    dw5 = dw.solve_at_travel(12.0)
    yield "state_args.dw", [plain(dw.anti_squat_pct(300.0, 1600.0, state=dw5)), plain(dw.side_view_swing_arm_length(dw5))]
    fmd = dw.mech.force_members()
    dw.mech.force_members = lambda: fmd[:1] + fmd[:1] + fmd[1:]
    yield "segments.dupes_dw", [x[2] for x in dw.render_segments()]
    st5 = k.solve_at_travel(5.0)
    yield "state_args", [plain(k.anti_squat_pct(300.0, 1600.0, state=st5)), plain(k.side_view_swing_arm_length(st5)),
                         plain(k.anti_dive_pct(300.0, 1600.0, state=st5))]
    from dataclasses import replace
    for x in (0.5, 1e-9, 2e-9):
        k._side_view_swing_arm = (lambda x: lambda state=None, travel=0.0: np.array([x, 50.0]))(x)
        cp_st = replace(st5, contact_patch=np.array([0.0, 600.0, 0.0]), wheel_center=np.array([0.0, 600.0, 230.0]))
        yield f"lsva.{x}", [plain(k.anti_dive_pct(300.0, 1600.0, state=cp_st)),
                            plain(k.anti_squat_pct(300.0, 1600.0, state=replace(cp_st, wheel_center=np.array(
                                [2 * x, 600.0, 230.0])))),
                            plain(k.side_view_swing_arm_length(replace(cp_st, contact_patch=np.array([5.0, 600.0,
                                                                                                         0.0]))))]
    k2 = GenericKinematics(T.example("trailing_arm"))
    real = k2.mech.solve
    k2.mech.solve = lambda t, **kw: real(t, **kw) if t == 0.0 else (_ for _ in ()).throw(RuntimeError("x"))
    yield "vel_fail", [plain(k2._instant_center(real(0.0), 0.0)), plain(k2._side_view_swing_arm(None, 0.0))]
    k2.mech.solve = real
    fm = k2.mech.force_members()
    k2.mech.force_members = lambda: fm[:1] + fm[:1] + [("ghost", "nope", fm[0][2])] + fm[1:]
    yield "segments.dupes", [len(k2.render_segments()), [x[2] for x in k2.render_segments()]]
    cpn = k2.mech.contact_patch
    k2.mech.solve = lambda t, **kw: (lambda r: dict(r, positions={n: q for n, q in r["positions"].items()
                                                                  if n != cpn}))(real(t, **kw))
    yield "segments.nocp", [x[2] for x in k2.render_segments()]
    k2.mech.solve = real


def test_every_topology_built_and_solved():
    check_lazy("topologies", _topologies(), rel=REL, abs_=1e-9)


def test_generic_kinematics_on_every_topology():
    check_lazy("adapter", _adapter(), rel=REL, abs_=1e-9)
