"""Characterization suite for suspension/inverse_genesis.py (mutation gate); see tests/_golden.py.

Split into one test per area so a mutant only runs the tests that reach it. The search, the co-optimizer, the
report and the band sweep are driven mostly through stand-ins (an analytic residual, duck-typed walls, a scripted
kinematic solver), so their branches are reached in milliseconds; the real solver is used where the module talks to
it directly (curves, residuals, steering, joint swing and misalignment, the wheel envelope, one small search)."""
import math

import numpy as np
import pytest

from suspension import inverse_genesis as G
from suspension.kinematics import Hardpoints
from suspension.kinematik_stochastic import ToleranceField, ToleranceSpec
from tests._golden import plain
from tests._golden_lazy import check_lazy

REL = 1e-6
ABS = 1e-8


def _try(fn, *a, **k):
    try:
        return plain(fn(*a, **k))
    except (ValueError, TypeError, KeyError) as e:
        return {"raised": type(e).__name__, "message": str(e)}


def _a(x, n=8):
    return plain(np.round(np.asarray(x, float), n))


@pytest.fixture(scope="module")
def hp():
    return Hardpoints.default()


class Named:
    def __repr__(self):
        return type(self).__name__


# --------------------------------------------------------------------------- #
#  A scripted solver: states whose numbers are chosen, not solved.
# --------------------------------------------------------------------------- #
class St(Named):
    def __init__(self, travel, converged=True, **kw):
        self.travel, self.converged = float(travel), converged
        self.camber, self.toe, self.caster, self.kpi = -1.0 - travel / 50, travel / 100, 5.0, 8.0
        self.scrub_radius = 10.0 + travel / 10
        self.lower_outer = np.array([0.0, 560.0, 120.0])
        self.upper_outer = np.array([-12.0, 540.0, 330.0])
        self.contact_patch = np.array([0.0, 600.0, 0.0])
        self.instant_center = np.array([-1500.0, 40.0 + travel])
        self.wheel_center = np.array([0.0, 600.0, 230.0])
        self.tie_rod_outer = np.array([70.0, 555.0, 150.0])
        self.__dict__.update(kw)


def fake_kin(kind="ok", **state_kw):
    class K:
        def __init__(self, hp, pickup_deltas=None):
            if kind == "raise":
                raise RuntimeError("no")

        def sweep(self, travel_min, travel_max, n):
            if kind == "empty":
                return []
            tr = np.linspace(travel_min, travel_max, n)
            out = [St(t, converged=not (kind == "unconv" and i == 1),
                      **({"camber": math.nan} if kind == "nan" else {}),
                      **({"instant_center": np.array([math.nan, 0.0])} if kind == "rcnan" else {}),
                      **state_kw) for i, t in enumerate(tr)]
            if kind == "noattr":
                for x in out:
                    del x.converged
            return out

        def solve_at_travel(self, t):
            return St(0.0, **state_kw)

        def anti_dive_pct(self, cg, wb, bias, state):
            if kind == "pitchraise":
                raise RuntimeError("pitch")
            return cg / 10 + wb / 1000 + bias + state.travel

        def anti_lift_pct(self, cg, wb, bias, state, inboard_brakes=False):
            return cg / 20 + bias * 3 + (100.0 if inboard_brakes else 0.0) + state.travel

        def anti_squat_pct(self, cg, wb, bias, state):
            return cg / 5 + bias * 7 + wb / 100 + state.travel
    return K


# --------------------------------------------------------------------------- #
def _curves_cases(hp, mp):
    st = np.array([-20.0, 0.0, 15.0])
    yield "consts", [list(G.CHANNELS), G._CHANNEL_LABELS, list(G.DESIGNABLE_POINTS), list(G.PERTURBABLE_OR_FIXED),
                     list(G._AXES)]
    for name, args in (("three", (st,)), ("one", (np.array([5.0]),)), ("track", (st, 1400.0)),
                       ("nsweep", (st, 1200.0, 5)), ("four", (np.array([-10.0, -5.0, 5.0, 10.0]),))):
        v, ok = G.curves_of(hp, *args)
        yield f"curves.{name}", [ok, {k: _a(x) for k, x in v.items()}]
    calls = []

    class Spy(G.SuspensionKinematics):
        def sweep(self, travel_min, travel_max, n):
            calls.append((travel_min, travel_max, n))
            return super().sweep(travel_min=travel_min, travel_max=travel_max, n=n)
    mp.setattr(G, "SuspensionKinematics", Spy)
    G.curves_of(hp, np.array([5.0]))
    G.curves_of(hp, np.arange(10.0))
    G.curves_of(hp, st, n_sweep=4)
    G.curves_of(hp, np.array([0.0, 0.5]))
    G.curves_of(hp, np.array([0.0, 1e-9]))
    yield "curves.sweep_args", calls
    for kind in ("raise", "empty", "unconv", "nan", "rcnan", "ok", "noattr"):
        mp.setattr(G, "SuspensionKinematics", fake_kin(kind))
        v, ok = G.curves_of(hp, st)
        yield f"curves.fake.{kind}", [ok, {k: _a(x) for k, x in v.items()}]
    mp.undo()
    T = G.TargetCurve
    yield "target.errors", [_try(T, "caster_deg", [0], [0], [1]), _try(T, "toe_deg", [0, 1], [0], [1]),
                            _try(T, "toe_deg", [0], [0, 1], [1]), _try(T, "toe_deg", [], [], []),
                            _try(T, "toe_deg", [0, 1], [0, 0], [1, 0]), _try(T, "toe_deg", [0], [0], [-1])]
    c = T("camber_deg", [10, -10, 0], [1.0, 2.0, 3.0], [0.1, 0.2, 0.3])
    yield "target.sorted", plain(c)
    yield "targets.errors", [_try(G.GenesisTargets, []),
                             _try(G.GenesisTargets, [T("toe_deg", [0], [0], [1]), T("toe_deg", [1], [0], [1])])]
    tg = G.GenesisTargets([c, T("rc_height_mm", [5, -10], [30.0, 20.0], [4.0, 5.0]),
                           T("scrub_mm", [0], [9.0], [2.0])], track_mm=1300.0)
    yield "targets.layout", [tg.rows(), _a(tg.stations()), _a(tg.target_vec()), _a(tg.band_vec()), tg.row_labels()]
    r, ok = tg.residual(hp)
    yield "targets.residual", [ok, _a(r, 6)]
    yield "targets.residual_default_track", _a(G.GenesisTargets([T("rc_height_mm", [0, 10], [30.0, 31.0],
                                                                    [1.0, 1.0])]).residual(hp)[0], 6)
    mp.setattr(G, "SuspensionKinematics", fake_kin("raise"))
    yield "targets.residual_fail", plain(tg.residual(hp))
    mp.undo()


def test_curves_and_targets(hp, monkeypatch):
    check_lazy("inverse_genesis.curves", _curves_cases(hp, monkeypatch), rel=REL, abs_=ABS)


# --------------------------------------------------------------------------- #
def _walls_cases(hp):
    K = G.KeepOutBox
    yield "box.errors", [_try(K, [0, 0, 0], [1, 1, 0]), _try(K, [0, 0, 0], [1, -1, 1], "x")]
    b = K([0, 0, 0], [10, 20, 30], label="hdr")
    pts = [[5, 5, 5], [1, 10, 15], [-3, 5, 5], [15, 25, 35], [10, 20, 30], [0, 0, 0], [5, 19, 15], [-1, -2, -2]]
    yield "box.clear", [_a(b.clearances(pts)), _a(b.clearances(pts, 2.0)), _a(b.clearances([5, 5, 5])),
                        _a(b.clearances(np.array([12.0, 10.0, 15.0]), 1.0)), b.label, K([0, 0, 0], [1, 1, 1]).label]
    S = G.PointSpacing
    yield "spacing.errors", [_try(S, "nope", "lower_rear_inner"), _try(S, "lower_front_inner", "wheel_centre"),
                             _try(S, "lower_front_inner", "lower_rear_inner", "w")]
    sps = [S("lower_front_inner", "lower_rear_inner", "x", 150), S("upper_outer", "lower_outer", "z", "-200"),
           S("tie_rod_inner", "tie_rod_outer", "dist", 10, "toe link"), S("wheel_center", "contact_patch", "y"),
           S("upper_front_inner", "upper_rear_inner", "dist")]
    yield "spacing", [[s.label, s.min_gap_mm, _a(s.gap(hp))] for s in sps]
    P = G.PropertyBound
    yield "bound.errors", [_try(P, "nope", 0, 1), _try(P, "caster_deg", 2, 1), _try(P, "caster_deg"),
                           _try(P, "caster_deg", -math.inf, math.inf)]
    bs = [P("caster_deg", 3, 7), P("anti_squat_pct", lo=23), P("kpi_deg", hi=9.5), P("caster_deg", 1, 1, "fixed"),
          P("rc_migration_mm_per_mm", "-0.1", "0.2")]
    yield "bound.labels", [[b.prop, b.lo, b.hi, b.label] for b in bs]
    yield "bound.margin", [[b.margin(v) for v in (0.0, 1.0, 3.0, 5.0, 7.0, 9.5, 30.0, math.nan, math.inf, -math.inf)]
                           for b in bs]
    yield "props.consts", [list(G.SOLVED_PROPERTIES), list(G.COMPLIANCE_PROPERTIES), sorted(G._STEER_PROPERTIES),
                           list(G.STEER_PROPERTIES), list(G.VEHICLE_PROPERTIES), G._VEHICLE_SPEC,
                           list(G.BOUNDABLE_PROPERTIES), G._PROPERTY_LABELS, sorted(G._PITCH_PROPERTIES),
                           sorted(G._COMPLIANCE_PROPERTIES), G._MAX_START_DRAWS_PER_START, G._ENVELOPE_LINKS,
                           list(G._OUTBOARD_PICKUPS), G._PIVOT_PARTNER, G._VERDICT_ICON]


def test_boxes_spacings_bounds(hp):
    check_lazy("inverse_genesis.walls", _walls_cases(hp), rel=REL, abs_=ABS)


# --------------------------------------------------------------------------- #
class FakeSE(Named):
    rack_c_mm_per_rev, lock_road_wheel_deg, rack_travel_max_mm, lock_basis = 80.0, 22.0, 55.0, "mean"

    def __init__(self, sw=400.0, kr_raise=False):
        self.sw, self.kr_raise, self.calls = sw, kr_raise, 0

    def ratio_min(self, trail):
        return 3.0 + trail / 10

    def kinematic_ratio(self, hp):
        if self.kr_raise:
            raise RuntimeError("kr")
        return 5.5 + hp.upper_outer[0] / 1000

    def lock_sw_deg(self, hp):
        self.calls += 1
        return self.sw + hp.upper_outer[0] / 100


class FakeElasto(Named):
    def __init__(self, lash=None, twist=None, mag=1000.0):
        self.joint_lash_mm = {"lo": 0.1} if lash is None else lash
        self.twist, self.mag = twist, mag

    def wheel_load_magnitude(self):
        return self.mag


class FakeTwist(Named):
    def __init__(self, raise_=False):
        self.raise_ = raise_

    def toe_deg(self, hp):
        if self.raise_:
            raise RuntimeError("tw")
        return 0.042 + hp.upper_outer[0] / 1e4


class EKR(Named):
    def __init__(self, converged=True, toe=0.01, members=None, all_members=None):
        self.converged = converged
        self.change = {"camber": -0.05, "toe": toe, "caster": 0.2, "kpi": 0.1}
        self.member_forces = members if members is not None else {"a": 300.0, "b": -2500.0}
        self.all_member_forces = all_members


class Vehicle(Named):
    def __init__(self, raise_=False):
        self.raise_ = raise_

    def decel_g(self, bias):
        return 1.4 + bias

    def fos_min(self, hp):
        if self.raise_:
            raise RuntimeError("fos")
        return 2.25 + hp.upper_outer[0] / 1e4

    def s0_min(self, hp):
        if self.raise_:
            raise RuntimeError("s0")
        return 3.5 + hp.upper_outer[0] / 1e4


def _ctx(props, **kw):
    """A SolvedPropertyBounds built without its validation, so every combination of specs can be offered."""
    c = G.SolvedPropertyBounds()
    for k, v in kw.items():
        setattr(c, k, v)
    c.bounds = [G.PropertyBound(p, lo=-1e9) for p in props]
    return c


def _props_cases(hp, mp):
    import suspension.dynamic_ackermann as DA
    import suspension.elastokinematics as EK
    mp.setattr(G, "SuspensionKinematics", fake_kin("ok"))
    js_calls = []

    def fake_swing(h, travel, n, rack):
        js_calls.append(("swing", travel, n, rack, h is hp))
        return {"_worst": ("x", 12.5, "w")}

    def fake_mis(h, travel, n, rack):
        js_calls.append(("mis", travel, n, rack, h is hp))
        return {"_worst": ("y", 7.25, "w")}
    mp.setattr(G, "joint_swing", fake_swing)
    mp.setattr(G, "joint_misalignment", fake_mis)
    lr_calls = []

    def fake_lr(h, c, lock, wheelbase_mm, kingpin_track_mm, rack_max_mm, lock_basis):
        lr_calls.append([c, lock, wheelbase_mm, kingpin_track_mm, rack_max_mm, lock_basis, h is hp])
        return {"reachable": True, "ackermann_pct_lo_and_lock": (10.0, 64.5)}
    mp.setattr(DA, "lock_report", fake_lr)
    ek_calls = []

    def fake_solve(h, spec, jacobian=True):
        ek_calls.append(("solve", getattr(spec, "tag", "e"), jacobian, h is hp))
        return spec.result if hasattr(spec, "result") else EKR()
    mp.setattr(EK, "solve_elastokinematic", fake_solve)
    mp.setattr(EK, "lash_deadband", lambda h, lash: {"toe": 0.03 * len(lash) + h.upper_outer[0] / 1e4,
                                                     "camber": 0.02})
    el = FakeElasto(twist=FakeTwist())
    br = FakeElasto()
    br.tag, br.result = "brake", EKR(toe=-0.07)
    se = FakeSE()
    full = dict(steer_effort=se, elasto=el, elasto_braking=br, brake=Vehicle(), structure=Vehicle(),
                bearing=Vehicle(), cg_height_mm=300.0, wheelbase_mm=1550.0, track_mm=1250.0, brake_bias_front=0.65,
                drive_bias_rear=0.9, travel_mm=(-10.0, 20.0), n_nodes=4)
    allp = list(G.BOUNDABLE_PROPERTIES)
    yield "all", plain(G.properties_of(hp, _ctx(allp, **full)))
    yield "all.calls", plain([js_calls, lr_calls, ek_calls, se.calls])
    yield "empty", [G.properties_of(hp, _ctx([])), G.properties_of(hp, _ctx(allp, **full), only=[])]
    yield "only", plain(G.properties_of(hp, _ctx([], **full), only=["caster_deg", "kpi_deg"]))
    yield "inboard", plain(G.properties_of(hp, _ctx(["anti_lift_pct"], rear_brakes_inboard=True)))
    yield "nnodes", plain(G.properties_of(hp, _ctx(["rc_migration_mm_per_mm", "anti_dive_pct"], n_nodes=1,
                                                   travel_mm=(-3.0, 9.0))))
    yield "ptp", [plain(G.properties_of(hp, _ctx(["rc_migration_mm_per_mm"], travel_mm=t))) for t in (
        (0.0, 1.0), (0.0, 1e-9), (0.0, 2e-9))]
    with mp.context() as m2:
        m2.setattr(G, "SuspensionKinematics", fake_kin("noattr"))
        yield "kin.noattr", plain(G.properties_of(hp, _ctx(["caster_deg"])))
    e3 = FakeElasto()
    e3.result = EKR(converged=False)
    yield "comp.lash_only", plain(G.properties_of(hp, _ctx(["lash_toe_deg"], elasto=e3)))
    yield "comp.small_load", plain(G.properties_of(hp, _ctx(["member_force_ratio"], elasto=FakeElasto(mag=0.5))))
    for p in allp:
        yield f"one.{p}", plain(G.properties_of(hp, _ctx([p], **full)))
    yield "steer.no_se", [plain(G.properties_of(hp, _ctx([p]))) for p in sorted(G._STEER_PROPERTIES)]
    yield "steer.kr_raise", [plain(G.properties_of(hp, _ctx([p], steer_effort=FakeSE(kr_raise=True))))
                             for p in ("steer_ratio", "steer_effort_margin", "steer_ratio_min")]
    yield "steer.sw_inf", [plain(G.properties_of(hp, _ctx([p], steer_effort=FakeSE(sw=math.inf))))
                           for p in ("lock_sw_deg", "joint_swing_lock_deg", "joint_misalignment_lock_deg")]

    def boom(*a, **k):
        raise RuntimeError("boom")
    with mp.context() as m2:
        m2.setattr(G, "joint_swing", boom)
        m2.setattr(G, "joint_misalignment", boom)
        m2.setattr(DA, "lock_report", boom)
        yield "steer.raise", [plain(G.properties_of(hp, _ctx([p], steer_effort=FakeSE())))
                              for p in ("joint_swing_lock_deg", "joint_misalignment_lock_deg", "ackermann_lock_pct")]
    with mp.context() as m2:
        m2.setattr(DA, "lock_report", lambda *a, **k: {"reachable": False})
        yield "steer.unreachable", plain(G.properties_of(hp, _ctx(["ackermann_lock_pct"], steer_effort=FakeSE())))
    for kind in ("raise", "empty", "unconv"):
        with mp.context() as m2:
            m2.setattr(G, "SuspensionKinematics", fake_kin(kind))
            yield f"kin.{kind}", plain(G.properties_of(hp, _ctx(["caster_deg"])))
    with mp.context() as m2:
        m2.setattr(G, "SuspensionKinematics", fake_kin("rcnan"))
        yield "kin.rcnan", plain(G.properties_of(hp, _ctx(["rc_migration_mm_per_mm"])))
    with mp.context() as m2:
        m2.setattr(G, "SuspensionKinematics", fake_kin("ok"))
        yield "kin.flat", plain(G.properties_of(hp, _ctx(["rc_migration_mm_per_mm"], travel_mm=(0.0, 1e-12))))
        flat_kpi = fake_kin("ok", upper_outer=np.array([-12.0, 540.0, 120.0]))
        m2.setattr(G, "SuspensionKinematics", flat_kpi)
        yield "kin.no_trail", [plain(G.properties_of(hp, _ctx(["mech_trail_mm"]))),
                               plain(G.properties_of(hp, _ctx(["steer_ratio_min"], steer_effort=FakeSE())))]
    with mp.context() as m2:
        m2.setattr(G, "SuspensionKinematics", fake_kin("pitchraise"))
        yield "kin.pitchraise", [plain(G.properties_of(hp, _ctx([p]))) for p in sorted(G._PITCH_PROPERTIES)]
    yield "comp.no_spec", plain(G.properties_of(hp, _ctx([]), only=["compliance_toe_deg", "caster_deg"]))
    with mp.context() as m2:
        m2.setattr(EK, "solve_elastokinematic", boom)
        m2.setattr(EK, "lash_deadband", boom)
        yield "comp.raise", [plain(G.properties_of(hp, _ctx([p], elasto=el, elasto_braking=br)))
                             for p in ("compliance_camber_deg", "member_force_ratio", "lash_toe_deg",
                                       "lash_camber_deg", "compliance_toe_braking_deg")]
    for name, res in (("unconv", EKR(converged=False)), ("allm", EKR(all_members={"z": -7000.0})),
                      ("nom", EKR(members={})), ("allm_empty", EKR(members={"q": 50.0}, all_members={}))):
        e2 = FakeElasto()
        e2.result = res
        yield f"comp.{name}", [plain(G.properties_of(hp, _ctx(["member_force_ratio", "compliance_kpi_deg"],
                                                              elasto=e2))),
                               plain(G.properties_of(hp, _ctx(["compliance_toe_braking_deg"], elasto=e2,
                                                              elasto_braking=e2)))]
    yield "comp.zero_load", plain(G.properties_of(hp, _ctx(["member_force_ratio"], elasto=FakeElasto(mag=0.0))))
    yield "comp.twist_raise", plain(G.properties_of(hp, _ctx(["twist_toe_deg"],
                                                             elasto=FakeElasto(twist=FakeTwist(True)))))
    yield "vehicle.raise", [plain(G.properties_of(hp, _ctx([p], brake=Vehicle(), structure=Vehicle(True),
                                                           bearing=Vehicle(True))))
                            for p in ("link_fos_min", "rocker_bearing_sf", "brake_decel_g")]
    mp.undo()
    real = G.SolvedPropertyBounds(bounds=[G.PropertyBound(p, lo=-1e9) for p in (
        "caster_deg", "kpi_deg", "scrub_static_mm", "rc_migration_mm_per_mm", "anti_dive_pct", "anti_lift_pct",
        "anti_squat_pct", "mech_trail_mm")], n_nodes=3)
    yield "real", plain(G.properties_of(hp, real))
    yield "real.evaluate", plain(real.evaluate(hp))
    st0 = St(0.0)
    yield "trail", [G.mechanical_trail_mm(st0), G.mechanical_trail_mm(St(0.0, upper_outer=np.array([5.0, 0, 120.0]))),
                    G.mechanical_trail_mm(St(0.0, contact_patch=np.array([3.0, 0.0, -10.0]),
                                             lower_outer=np.array([1.0, 0.0, 100.0]),
                                             upper_outer=np.array([-20.0, 0.0, 300.0]))),
                    G.mechanical_trail_mm(St(0.0, lower_outer=np.zeros(3), upper_outer=np.array([1.0, 0.0, 0.5]))),
                    G.mechanical_trail_mm(St(0.0, lower_outer=np.zeros(3), upper_outer=np.array([1.0, 0.0, 1e-9]))),
                    G.mechanical_trail_mm(St(0.0, lower_outer=np.zeros(3), upper_outer=np.array([1.0, 0.0, 2e-9])))]


def test_properties_of(hp, monkeypatch):
    check_lazy("inverse_genesis.props", _props_cases(hp, monkeypatch), rel=REL, abs_=ABS)


# --------------------------------------------------------------------------- #
def _steer_cases(hp, mp):
    S = G.SteerEffort
    yield "errors", _try(S, lock_basis="outer")
    se = S()
    yield "dict", [se.to_dict(), S(t0_outer_mm=12.0, rack_c_mm_per_rev=90.0, lock_road_wheel_deg=25.0,
                                   rack_travel_max_mm=40.0, lock_basis="mean", fz_outer_n=1000.0, fz_inner_n=100.0,
                                   effort_limit_nm=8.0).to_dict()]
    yield "toe", [se._toe_at(hp, x) for x in (-1.0, 0.0, 2.0)]
    yield "ratio", [se.kinematic_ratio(hp), se.kinematic_ratio(hp, h=1.0), S(rack_c_mm_per_rev=50.0).kinematic_ratio(hp)]
    yield "ratio_min", [se.ratio_min(t) for t in (5.0, 20.0)] + [S(t0_outer_mm=30.0, effort_limit_nm=5.0).ratio_min(15.0)]
    yield "lock", [se.lock_sw_deg(hp), S(lock_basis="mean").lock_sw_deg(hp), S(lock_road_wheel_deg=-15.0).lock_sw_deg(hp),
                   S(lock_road_wheel_deg=80.0).lock_sw_deg(hp), S(rack_travel_max_mm=2.0).lock_sw_deg(hp)]

    class Flat(S):
        def _toe_at(self, hp, rack_mm):
            return 0.25
    yield "flat", [Flat().kinematic_ratio(hp), Flat().lock_sw_deg(hp)]

    class Down(S):
        def _toe_at(self, hp, rack_mm):
            return 1.0 - 0.5 * rack_mm
    yield "down", [Down().lock_sw_deg(hp), Down(lock_basis="mean").lock_sw_deg(hp), Down().kinematic_ratio(hp)]

    class Lopsided(S):
        def _toe_at(self, hp, rack_mm):
            return 0.8 * rack_mm if rack_mm > 0 else 0.2 * rack_mm
    yield "lopsided", [Lopsided().lock_sw_deg(hp), Lopsided(lock_basis="mean").lock_sw_deg(hp)]

    class Breaks(S):
        def _toe_at(self, hp, rack_mm):
            if abs(rack_mm) > 50.0:
                raise RuntimeError("far")
            if 20.0 < abs(rack_mm) < 30.0:
                raise RuntimeError("mid")
            return 0.5 * rack_mm
    yield "breaks", [Breaks().lock_sw_deg(hp), Breaks(rack_travel_max_mm=45.0).lock_sw_deg(hp)]

    def toe(fn):
        return type("T", (S,), {"_toe_at": lambda self, hp, x: fn(x)})
    lin = toe(lambda x: x)
    yield "exact", [repr(x) for x in (
        toe(lambda x: 1e-9 if x > 0 else 0.0)().kinematic_ratio(hp),
        toe(lambda x: 2e-9 if x > 0 else 0.0)().kinematic_ratio(hp),
        toe(lambda x: x if x < 1.5 else -x)().lock_sw_deg(hp),
        toe(lambda x: 0.0 if abs(x - 1.0) < 1e-12 else -x)().lock_sw_deg(hp),
        toe(lambda x: 40.0 * x)().lock_sw_deg(hp),
        lin(lock_road_wheel_deg=20.0, rack_travel_max_mm=20.0).lock_sw_deg(hp),
        lin(lock_road_wheel_deg=30.0).lock_sw_deg(hp),
        lin(lock_road_wheel_deg=30.0, lock_basis="mean").lock_sw_deg(hp),
        lin().lock_sw_deg(hp),
        toe(lambda x: (x if x < 1.5 else -x) if x >= 0 else 0.1 * x)().lock_sw_deg(hp),
        toe(lambda x: 0.0 if x == 1.0 else (x if x > 0 else 0.1 * x))().lock_sw_deg(hp))]

    def gap(x):
        if 0.2 < abs(x) < 0.4:
            raise RuntimeError("gap")
        return x
    yield "exact.gap", [repr(toe(gap)(lock_road_wheel_deg=0.5).lock_sw_deg(hp)),
                        repr(toe(gap)(lock_road_wheel_deg=0.25).lock_sw_deg(hp))]


def test_steer_effort(hp, monkeypatch):
    check_lazy("inverse_genesis.steer", _steer_cases(hp, monkeypatch), rel=REL, abs_=ABS)


# --------------------------------------------------------------------------- #
def _bounds_cases(hp):
    P, B = G.PropertyBound, G.SolvedPropertyBounds
    yield "errors", [
        _try(B, bounds=["x"]),
        _try(B, bounds=[P("compliance_toe_deg", hi=1)]),
        _try(B, bounds=[P("compliance_toe_braking_deg", hi=1)], elasto=FakeElasto()),
        _try(B, bounds=[P("steer_ratio", hi=1)]),
        _try(B, bounds=[P("lash_toe_deg", hi=1)], elasto=FakeElasto(lash={})),
        _try(B, bounds=[P("lash_camber_deg", hi=1)], elasto=type("E", (), {})()),
        _try(B, bounds=[P("twist_toe_deg", hi=1)], elasto=FakeElasto()),
        _try(B, bounds=[P("twist_toe_deg", hi=1)], elasto=type("E", (), {})()),
        _try(B, bounds=[P("brake_decel_g", lo=1)]), _try(B, bounds=[P("link_fos_min", lo=1)]),
        _try(B, bounds=[P("rocker_bearing_sf", lo=1)]),
        _try(B, cg_height_mm=0), _try(B, wheelbase_mm=-1), _try(B, track_mm=0.0),
        _try(B, travel_mm=(5, 5)), _try(B, travel_mm=(5, -5)), _try(lambda: B(cg_height_mm=0.5, wheelbase_mm=0.75,
                                                                                  track_mm=0.9).track_mm)]
    ok = [B(bounds=[P("compliance_toe_braking_deg", hi=1)], elasto=FakeElasto(), elasto_braking=FakeElasto()),
          B(bounds=[P("lash_toe_deg", hi=1), P("twist_toe_deg", hi=1)], elasto=FakeElasto(twist=FakeTwist())),
          B(bounds=[P("lock_sw_deg", hi=1)], steer_effort=FakeSE()),
          B(bounds=[P("brake_decel_g", lo=1), P("link_fos_min", lo=1), P("rocker_bearing_sf", lo=1)], brake=1,
            structure=2, bearing=3),
          B(bounds=(P("caster_deg", 1, 2),), travel_mm=[-3, "4"]), B(elasto=FakeElasto(lash={}))]
    yield "ok", [[sorted(b.needed()), list(b.travel_mm), type(b.bounds).__name__] for b in ok]
    b = B(bounds=[P("caster_deg", 4.0, 4.5), P("kpi_deg", hi=50.0), P("scrub_static_mm", lo=1e6)], n_nodes=3)
    yield "violations", [plain(b.violations(hp)), B().violations(hp)]
    edge = B(bounds=[P("caster_deg", lo=1e-12), P("kpi_deg", hi=-1e-12)])
    edge.evaluate = lambda h: {"caster_deg": 0.0, "kpi_deg": 0.0}
    yield "violations.edge", plain(edge.violations(hp))
    yield "violations.exact", plain(B(bounds=[P("caster_deg", lo=b.evaluate(hp)["caster_deg"])], n_nodes=3)
                                    .violations(hp))


def test_solved_property_bounds(hp, monkeypatch):
    check_lazy("inverse_genesis.bounds", _bounds_cases(hp), rel=REL, abs_=ABS)
    monkeypatch.setattr(G, "SuspensionKinematics", fake_kin("raise"))
    b = G.SolvedPropertyBounds(bounds=[G.PropertyBound("caster_deg", 1, 2)])
    assert b.violations(Hardpoints.default()) == [("(sweep)", "geometry does not solve over the bound evaluation "
                                                   "range", -math.inf)]


# --------------------------------------------------------------------------- #
def _joint_cases(hp):
    yield "spin", [_a(G.spin_axis(c, t)) for c, t in ((0, 0), (-2, 0.5), (3, -1), (-1.5, 2.0))]
    p0 = np.array([[0.0, 0, 0], [1, 0, 0], [0, 2, 0], [0, 0, 3]])
    rz = np.array([[0.0, -1, 0], [1, 0, 0], [0, 0, 1]])
    yield "kabsch.shift", _a(G._kabsch(p0, p0 @ rz.T + np.array([3.0, -70.0, 110.0])))
    yield "angles", [G._angle_deg(np.array([1.0, 0, 0]), np.array([1.0, 0, 0])),
                     G._angle_deg(np.array([1.0, 0, 0]), np.array([0.0, 1, 0])),
                     G._angle_deg(np.array([1.0, 0, 0]), np.array([-1.0, 0, 0])),
                     G._angle_deg(np.array([0.6, 0.8, 0]), np.array([0.8, 0.6, 0])),
                     G._tilt_deg(np.array([0.0, 0, 1]), np.array([1.0, 0, 0])),
                     G._tilt_deg(np.array([0.0, 0, 1]), np.array([0.0, 0, 1])),
                     G._tilt_deg(np.array([0.0, 0, 1]), np.array([0.0, -0.6, -0.8]))]
    yield "kabsch", [_a(G._kabsch(p0, p0 @ rz.T + 5.0)), _a(G._kabsch(p0, p0)),
                     _a(G._kabsch(p0, p0 * np.array([1.0, 1.0, -1.0])))]
    for name, kw in (("default", {}), ("rack", dict(rack_travel_mm=-6.0)), ("links", dict(links=["tie_rod"])),
                     ("short", dict(travel_mm=(-5.0, 10.0), n_travel=3, rack_travel_mm=4.0))):
        yield f"swing.{name}", plain({k: (v if isinstance(v, tuple) else round(v, 6))
                                      for k, v in G.joint_swing(hp, **kw).items()})
    for name, kw in (("default", {}), ("rack", dict(rack_travel_mm=-6.0)),
                     ("short", dict(travel_mm=(-5.0, 10.0), n_travel=3, rack_travel_mm=4.0))):
        yield f"mis.{name}", plain({k: (v if isinstance(v, tuple) else round(v, 6))
                                    for k, v in G.joint_misalignment(hp, **kw).items()})


def _joint_spy_cases(hp, mp):
    calls = []

    class Spy(G.SuspensionKinematics):
        def __init__(self, h, pickup_deltas=None):
            calls.append(["init", None if pickup_deltas is None else plain(pickup_deltas)])
            super().__init__(h, pickup_deltas=pickup_deltas)

        def sweep(self, travel_min, travel_max, n):
            calls.append(["sweep", travel_min, travel_max, n])
            return super().sweep(travel_min=travel_min, travel_max=travel_max, n=n)
    mp.setattr(G, "SuspensionKinematics", Spy)
    for fn in (G.joint_swing, G.joint_misalignment):
        for kw in ({}, dict(rack_travel_mm=3.0, travel_mm=(-2.0, 2.0), n_travel=2)):
            fn(hp, **kw)
            yield f"{fn.__name__}.{len(kw)}", calls[:]
            calls.clear()
        with mp.context() as m2:
            m2.setattr(G, "SuspensionKinematics", fake_kin("ok"))
            yield f"{fn.__name__}.still", plain(fn(hp, n_travel=2))
        tiny = fn(hp, travel_mm=(-0.25, 0.25), n_travel=2)
        yield f"{fn.__name__}.tiny", plain({k: (v if isinstance(v, tuple) else round(v, 9)) for k, v in tiny.items()})
        calls.clear()


def test_joint_swing_and_misalignment(hp, monkeypatch):
    check_lazy("inverse_genesis.joints", _joint_cases(hp), rel=REL, abs_=1e-6)
    check_lazy("inverse_genesis.joints_spy", _joint_spy_cases(hp, monkeypatch), rel=REL, abs_=1e-9)


# --------------------------------------------------------------------------- #
def _node_tool_cases(hp):
    N, T = G.NodeAttachment, G.ToolAccess
    ufi = list(hp.upper_front_inner)
    yield "node.errors", [_try(N, []), _try(N, [ufi], points=["wheel_center"]), _try(N, [ufi], max_offset_mm=0),
                          _try(N, [ufi], max_offset_mm=-1)]
    n = N([ufi, [0, 0, 0], np.array(hp.lower_rear_inner) + [0, 0, 25.0]])
    near = np.array(hp.lower_outer, float) + [0.0, 0.0, 25.5]
    yield "node.edges", [N([[0, 0, 0]], max_offset_mm=0.5).label,
                         plain(N([near], points=("lower_outer",)).violations(hp))]
    yield "node", [n.label, plain(n.nodes), n.points, plain(n.nearest(hp)), plain(n.violations(hp)),
                   plain(N([[0, 0, 0]], points=("tie_rod_inner",), max_offset_mm=5).violations(hp))]
    yield "tool.errors", [_try(T, points=["upper_outer"]), _try(T, reach_mm=0), _try(T, tool_radius_mm=-1),
                          _try(T, start_offset_mm=-0.1)]
    t = T()
    yield "tool", [t.label, t.samples, T(samples=0).samples, T(samples=7.9).samples, T(start_offset_mm=0).label,
                   T(reach_mm=0.5, tool_radius_mm=0.5).label]
    c = np.array(hp.upper_front_inner, float)
    a = c - np.array(hp.upper_rear_inner, float)
    a /= np.linalg.norm(a)
    obs = [G.KeepOutBox(c + 40 * a - 10, c + 40 * a + 10, "front"), G.KeepOutBox([-900, -900, -900],
                                                                                  [-800, -800, -800])]
    yield "tool.clear", [plain(t.clearance(hp, obs)), plain(t.clearance(hp, [])), plain(t.clearance(hp, obs[1:])),
                         plain(T(points=("lower_rear_inner",), reach_mm=50, tool_radius_mm=3, start_offset_mm=0,
                                 samples=5).clearance(hp, obs))]
    same = hp.copy()
    same.upper_rear_inner = np.array(same.upper_front_inner, float)
    yield "tool.degenerate", plain(t.clearance(same, obs))
    end = c + 125 * a
    far = [G.KeepOutBox(end - 3, end + 3, "end")]
    b_ = np.array(hp.upper_rear_inner, float)
    both = [G.KeepOutBox(c + 134 * a - 4, c + 134 * a + 4, "fore"), G.KeepOutBox(b_ - 131 * a - 4, b_ - 131 * a + 4,
                                                                                  "aft")]
    yield "tool.both", [plain(t.clearance(hp, both)), plain(T(samples=5).clearance(hp, both))]
    yield "tool.end", [plain(t.clearance(hp, far)), plain(T(samples=3, reach_mm=200.0).clearance(hp, far))]
    for gap_ in (0.5, 1e-9, 2e-9):
        h2 = hp.copy()
        h2.upper_front_inner = np.array([0.0, 0.0, 0.0])
        h2.upper_rear_inner = np.array([gap_, 0.0, 0.0])
        yield f"tool.short.{gap_}", plain(T(points=("upper_front_inner",)).clearance(h2, [G.KeepOutBox(
            [50.0, -5, -5], [60.0, 5, 5])]))


def test_nodes_and_tool_access(hp):
    check_lazy("inverse_genesis.nodetool", _node_tool_cases(hp), rel=REL, abs_=ABS)


# --------------------------------------------------------------------------- #
def _wheel_cases(hp, mp):
    W, WS = G.WheelEnvelope, G.WheelSector
    yield "sector.errors", [_try(WS, "a", -1, 5, 0, 1, 0, 1), _try(WS, "a", 5, 5, 0, 1, 0, 1),
                            _try(WS, "a", 0, 5, 1, 1, 0, 1), _try(WS, "a", 0, 5, 0, 1, 1, 1)]
    sec = WS("caliper", 90.0, 130.0, 30.0, 90.0, -20.0, 30.0)
    yield "sector.depth", [sec.depth(r, th, ax, pad) for r, th, ax, pad in (
        (100, 60, 0, 0), (100, 30, 0, 0), (100, 0, 0, 0), (100, 200, 0, 0), (100, 359, 0, 2), (85, 60, 0, 10),
        (140, 60, 0, 0), (100, 60, -25, 0), (100, 60, 35, 6), (0.0, 60, 0, 0), (100, 95, 0, 3), (100, 420, 0, 0),
        (100, -300, 0, 0), (200, 85, 0, 0), (200, 80, 0, 0), (0.5, 60, 0, 0), (0.5, 89, 0, 0), (0.5, 100, 0, 0),
        (128, 60, 0, 3), (100, 60, -21, 3), (100, 31, 0, 0.5), (100, 25, 0, 0.5), (100, 89.5, 5, 0))]
    hub = WS("hub", 0.0, 10.0, 30.0, 90.0, -5.0, 5.0)
    yield "sector.hub", [hub.depth(r, th, 0.0, 0.0) for r, th in ((0.5, 60), (0.5, 200), (2e-7, 60), (2e-7, 200),
                                                                   (3.0, 40))]
    errs = [dict(pickup_radius_mm=0), dict(rim_radius_mm=-1), dict(tire_radius_mm=0), dict(rim_half_width_mm=0),
            dict(tire_half_width_mm=0), dict(tire_radius_mm=127), dict(tire_half_width_mm=80),
            dict(pickup_radius_mm=128), dict(travel_mm=(5, 5)), dict(links=("upper_front", "bad")),
            dict(tire_profile=((120, 90), (200, 95))), dict(tire_profile=((130, 90), (230, 95))),
            dict(sectors=("x",))]
    yield "env.errors", [_try(W, **e) for e in errs]
    e1 = W(n_travel=1, samples_per_link=2, rack_travel_mm=-5, link_radius_mm=-3, rod_end_radius_mm=-1,
           tire_profile=[(200, 95), (127, 89)], sectors=[sec], joint_swing_limit_deg=12.5, travel_mm=[-10, "10"],
           links=["tie_rod"], pickup_radius_mm=127.0, tire_half_width_mm=89.0)
    yield "env.norm", [e1.n_travel, e1.samples_per_link, e1.rack_travel_mm, e1.link_radius_mm, e1.rod_end_radius_mm,
                       plain(e1.tire_profile), list(e1.travel_mm), list(e1.links), e1.label, W().label,
                       W(rack_travel_mm=3).label, W(tire_profile=[(127, 89), (228, 95)]).label]
    yield "env.edges", [W(pickup_radius_mm=0.5).label, W(rim_half_width_mm=0.5, tire_half_width_mm=0.75).label,
                        _try(W, tire_profile=((126.5, 90), (200, 95))), _try(W, tire_profile=((127, 89), (150, 92),
                                                                                              (228.5, 95))),
                        W(tire_profile=((127, 89), (150, 92), (228, 95))).label,
                        [r for r, h, st in W(rack_travel_mm=0.5, n_travel=2)._states(hp)]]
    e0 = W()
    yield "env.halfwidth", [[e._half_width(r) for r in (100, 127, 150, 177.5, 178, 200, 228, 300)]
                            for e in (e0, e1, W(rim_radius_mm=127, tire_radius_mm=127.5))]
    yield "env.default", [plain(e0.check(hp)), plain(e0.violations(hp)), e0.margin(hp)]
    tight = W(pickup_radius_mm=60.0, rim_radius_mm=80.0, tire_radius_mm=300.0, rim_half_width_mm=150.0,
              tire_half_width_mm=400.0, clearance_mm=2.0, n_travel=2, samples_per_link=4, rack_travel_mm=5.0,
              sectors=(WS("hub", 0.0, 60.0, -180.0, 180.0, -50.0, 50.0), sec), joint_swing_limit_deg=0.5,
              wheel_offset_mm=10.0)
    yield "env.tight", [plain(tight.check(hp)), plain(tight.violations(hp))]
    yield "env.offset", plain(W(wheel_offset_mm=-40.0, n_travel=2, samples_per_link=4, link_radius_mm=0,
                                rod_end_radius_mm=0, links=("lower_front", "tie_rod")).check(hp))
    yield "env.states", [[r, plain(h.tie_rod_inner), len(s)] for r, h, s in tight._states(hp)]
    pds = []

    class Spy(G.SuspensionKinematics):
        def __init__(self, h, pickup_deltas=None):
            pds.append(None if pickup_deltas is None else plain(pickup_deltas))
            super().__init__(h, pickup_deltas=pickup_deltas)
    with mp.context() as m2:
        m2.setattr(G, "SuspensionKinematics", Spy)
        W(rack_travel_mm=2.0, n_travel=2)._states(hp)
        W(n_travel=2)._states(hp)
    yield "env.states_spy", pds
    for kind in ("raise", "unconv", "empty"):
        with mp.context() as m2:
            m2.setattr(G, "SuspensionKinematics", fake_kin(kind))
            yield f"env.fake.{kind}", plain(W(rack_travel_mm=2.0, n_travel=2).check(hp))

    def boom(*a, **k):
        raise RuntimeError("b")
    with mp.context() as m2:
        m2.setattr(G, "joint_swing", boom)
        yield "env.swing_raise", plain(W(joint_swing_limit_deg=1.0, n_travel=2).violations(hp))
    calls = []
    with mp.context() as m2:
        m2.setattr(G, "joint_swing", lambda *a: calls.append(a[1:]) or {"a.inner": 3.0, "b.outer": 3.0 + 1e-10,
                                                                         "c.outer": 3.1, "_worst": ("c", 9.0, "")})
        yield "env.swing_fake", [plain(W(joint_swing_limit_deg=3.0, n_travel=3, rack_travel_mm=2.0,
                                         links=("tie_rod",)).violations(hp)), plain(calls)]
        m2.setattr(G, "joint_swing", lambda *a: {"a.inner": 3.0 + 1e-9, "_worst": ("a", 3.0 + 1e-9, "")})
        yield "env.swing_edge", plain(W(joint_swing_limit_deg=3.0, n_travel=2).violations(hp))
    with mp.context() as m2:
        m2.setattr(G, "SuspensionKinematics", fake_kin("noattr"))
        yield "env.fake.noattr", plain(W(n_travel=2, links=("tie_rod",)).check(hp))
    yield "env.halfwidth.fine", [W(rim_radius_mm=127, tire_radius_mm=127.5)._half_width(r) for r in (127.05, 127.1,
                                                                                                       127.2)]
    # the rod-end barrel term governs: pickup envelope wider than rim - rod end
    for off in (-60.0, -20.0, 0.0, 35.0, 60.0, 80.0):
        yield f"env.barrel.{off}", plain(W(pickup_radius_mm=120.0, rim_radius_mm=125.0, clearance_mm=1.0,
                                           wheel_offset_mm=off, n_travel=2, links=()).check(hp))
    yield "env.small_violation", plain(W(rod_end_radius_mm=10.0, pickup_radius_mm=125.0, n_travel=2, links=("lower_front",)).check(hp))
    yield "env.small_tire", plain(W(pickup_radius_mm=50.0, rim_radius_mm=60.0, tire_radius_mm=80.0, n_travel=2,
                                    clearance_mm=1.5, link_radius_mm=3.0).check(hp))
    # a sector straddling the upper ball joint, so the sector angle decides its margin
    s0 = G.SuspensionKinematics(hp).sweep(travel_min=-25.0, travel_max=25.0, n=2)[0]
    ax = G.spin_axis(s0.camber, s0.toe)
    up = np.array([0.0, 0.0, 1.0]) - ax[2] * ax
    up /= np.linalg.norm(up)
    rear = np.cross(up, ax)
    rear = -rear if rear[0] < 0 else rear
    d = np.asarray(s0.upper_outer, float) - np.asarray(s0.wheel_center, float)
    a_ = float(d @ ax)
    rv = d - a_ * ax
    th = float(np.degrees(np.arctan2(rv @ rear, rv @ up)))
    rad = float(np.linalg.norm(rv))
    sec2 = WS("ball", rad - 20, rad + 20, th - 4, th + 4, a_ - 25, a_ + 25)
    yield "env.sector_ball", [round(th, 6), plain(W(sectors=(sec2,), n_travel=2, clearance_mm=1.0,
                                                    links=("upper_front", "upper_rear")).check(hp))]


def test_wheel_envelope(hp, monkeypatch):
    check_lazy("inverse_genesis.wheel", _wheel_cases(hp, monkeypatch), rel=REL, abs_=1e-6)


# --------------------------------------------------------------------------- #
class Wall(Named):
    """A duck-typed wall: properties, wheel envelope or node attachment, scripted by a predicate."""

    def __init__(self, bad, label="wall", rack=0.0, bounds=None, margin=1.5):
        self.bad, self.label, self.rack_travel_mm, self._m = bad, label, rack, margin
        self.bounds = bounds if bounds is not None else [G.PropertyBound("caster_deg", 1, 9)]
        self.calls = 0

    def violations(self, hp):
        self.calls += 1
        x = float(hp.upper_front_inner[0])
        return [("p", self.label, round(-0.5 - x / 1e4, 9))] if self.bad(hp) else []

    def margin(self, hp):
        return round(self._m + float(hp.upper_front_inner[0]) / 1e4, 9)

    def evaluate(self, hp):
        return {"caster_deg": round(float(hp.upper_front_inner[0]), 6), "kpi_deg": 7.5}


def _volume_cases(hp):
    L = G.LegalVolume
    yield "errors", [_try(L, {}), _try(L, {"wheel_center": ([0, 0, 0], [1, 1, 1])}),
                     _try(L, {"upper_outer": ([0, 0, 0], [1, -1, 1])})]
    v = L.around(hp, 5.0, points=["upper_rear_inner", "upper_front_inner"])
    yield "around", [v.points(), v.coords(), v.coord_labels(), plain(v.boxes), plain(v.bounds_vec(hp))]
    yield "around.dict", plain(L.around(hp, {"lower_outer": -3.0, "tie_rod_inner": 2}).boxes)
    yield "around.all", L.around(hp, 1.0).points()
    nested = L({"lower_outer": ([[0, 0, 0]], [[1, 2, 3]]), "upper_outer": ([5, 5, 5], [5, 6, 5])})
    yield "nested", [plain(nested.boxes), plain(nested.bounds_vec(hp)),
                     plain(G.KeepOutBox([[0, 0, 0]], [[1, 2, 3]]).lo), plain(G.KeepOutBox([[0, 0, 0]], [[1, 2, 3]]).hi)]
    lo_, hi_ = v.bounds_vec(hp)
    yield "clamp.edges", [plain(v.clamp(hp, lo_)), plain(v.clamp(hp, hi_)), plain(v.clamp(hp, hi_ + 0.5)),
                          plain(v.clamp(hp, lo_ - 0.5))]
    yield "around.kw", plain(L.around(hp, {"lower_outer": 1}, points=["lower_outer"], probe_radius_mm=4.0,
                                      min_clearance_mm=2.0))
    yield "clamp", [plain(v.clamp(hp, np.array([1.0, -6.0, 5.0, 5.0 + 1e-13, -5.0 - 1e-11, 0.0]))),
                    plain(v.clamp(hp, np.zeros(6)))]
    c = np.array(hp.upper_front_inner, float)
    ko = [G.KeepOutBox(c - 1, c + 1, "near"), G.KeepOutBox(c + 3, c + 9)]

    class Env:
        kind = "phantom"

        def clearances(self, pts, probe):
            return np.full(len(pts), 1.0 - probe)

    class Bare:
        def clearances(self, pts, probe):
            return [2.0 - probe] * len(pts)
    vk = L.around(hp, 5.0, points=["upper_front_inner", "upper_rear_inner"], keep_out=ko + [Env(), Bare()],
                  probe_radius_mm=0.5, min_clearance_mm=1.0,
                  spacings=[G.PointSpacing("upper_front_inner", "upper_rear_inner", "dist", 1e4),
                            G.PointSpacing("upper_front_inner", "upper_rear_inner", "dist", 1.0),
                            G.PointSpacing("upper_front_inner", "upper_rear_inner", "dist",
                                           G.PointSpacing("upper_front_inner", "upper_rear_inner", "dist").gap(hp)
                                           + 0.5)],
                  tool_access=G.ToolAccess(points=("upper_front_inner",)))
    yield "keepout", plain(vk.keepout_violations(hp))
    yield "keepout.notool_without_obstacles", plain(L.around(hp, 1.0, points=["upper_outer"],
                                                             tool_access=G.ToolAccess()).keepout_violations(hp))
    exact = L.around(hp, 1.0, points=["upper_outer"], keep_out=[Bare()], probe_radius_mm=1.0, min_clearance_mm=1.0,
                     spacings=[G.PointSpacing("upper_outer", "lower_outer", "dist",
                                              G.PointSpacing("upper_outer", "lower_outer", "dist").gap(hp))])
    yield "keepout.exact", plain(exact.keepout_violations(hp))
    blocked = G.KeepOutBox(c - 500, c + 500, "all")
    ta = G.ToolAccess(points=("lower_rear_inner",))
    cl0 = ta.clearance(hp, [blocked])["lower_rear_inner"]
    for dc in (0.5, -0.5):
        ko2 = G.KeepOutBox(c - 500, c + 500, "all")
        ko2.clearances = (lambda f, k: (lambda pts, r: f(pts, r) - k))(ko2.clearances, cl0 - dc)
        yield f"keepout.tool_small.{dc}", plain(L.around(hp, 1.0, points=["upper_outer"], keep_out=[ko2],
                                                        min_clearance_mm=-1e4, tool_access=ta).keepout_violations(hp))
    yield "keepout.tool", plain(L.around(hp, 1.0, points=["upper_outer"], keep_out=[blocked], min_clearance_mm=-1e4,
                                         tool_access=G.ToolAccess(points=("lower_rear_inner",)))
                                .keepout_violations(hp))
    pw = Wall(lambda h: True, "prop")
    walls = L.around(hp, 1.0, points=["upper_outer"], properties=pw, wheel_envelope=Wall(lambda h: True, "env"),
                     node_attachment=Wall(lambda h: True, "node"))
    yield "walls", [walls.has_property_bounds(), walls.has_wheel_envelope(), walls.has_hard_walls(),
                    plain(walls.property_violations(hp)), plain(walls.envelope_violations(hp)),
                    walls.envelope_margin(hp), plain(walls.node_violations(hp)), plain(walls.wall_violations(hp)),
                    plain(walls.evaluate_properties(hp))]
    none = L.around(hp, 1.0, points=["upper_outer"])
    empty = L.around(hp, 1.0, points=["upper_outer"], properties=Wall(lambda h: True, bounds=[]))
    only_node = L.around(hp, 1.0, points=["upper_outer"], node_attachment=Wall(lambda h: False))
    only_env = L.around(hp, 1.0, points=["upper_outer"], wheel_envelope=Wall(lambda h: False))
    yield "walls.none", [[x.has_property_bounds(), x.has_wheel_envelope(), x.has_hard_walls(),
                          x.property_violations(hp), x.envelope_violations(hp), x.envelope_margin(hp),
                          x.node_violations(hp), x.wall_violations(hp), x.evaluate_properties(hp)]
                         for x in (none, empty, only_node, only_env)]
    yield "unflatten", plain(G._unflatten(np.arange(6.0) + 1, v.coords()))
    yield "unflatten.partial", plain(G._unflatten(np.array([2.0, 3.0]), [("lower_outer", 2), ("upper_outer", 0)]))
    s = G._shifted(hp, v, np.array([1.0, 2.0, 3.0, -1.0, -2.0, -3.0]))
    yield "shifted", [plain(s.upper_front_inner), plain(s.upper_rear_inner), plain(s.lower_outer)]


def test_legal_volume(hp):
    check_lazy("inverse_genesis.volume", _volume_cases(hp), rel=REL, abs_=ABS)


# --------------------------------------------------------------------------- #
class LinTargets(Named):
    """An analytic residual on two points: r = band-scaled offsets from a chosen target, with a gentle
    nonlinearity so the search needs several steps, and an optional cliff where it does not solve."""

    def __init__(self, t1, t2z, band=1.0, cliff=None):
        self.t1, self.t2z, self.band, self.cliff = np.asarray(t1, float), float(t2z), band, cliff

    def rows(self):
        return [("camber_deg", -10.0), ("toe_deg", 0.0), ("rc_height_mm", 10.0), ("scrub_mm", 0.0)]

    def row_labels(self):
        return [f"{c} @ {t:+.1f} mm" for c, t in self.rows()]

    def residual(self, hp):
        if self.cliff is not None and self.cliff(hp):
            return np.full(4, np.nan), False
        d = np.asarray(hp.upper_front_inner, float) - self.t1
        e = float(hp.upper_rear_inner[2]) - self.t2z
        r = np.array([d[0] + 0.05 * d[0] ** 2 + 0.01 * d[0] ** 3, d[1] - 0.02 * d[1] * d[2], d[2], e]) / self.band
        return r, True


def _cand(c):
    d = plain(c)
    d["shifts"] = {k: _a(v, 6) for k, v in sorted(c.shifts.items())}
    d["shift_vec"] = _a(c.shift_vec, 6)
    d["residual"] = _a(c.residual, 6)
    return d


def _solve_cases(hp, mp):
    ufi = np.array(hp.upper_front_inner, float)
    uriz = float(hp.upper_rear_inner[2])
    pts = ["upper_front_inner", "upper_rear_inner"]
    L = G.LegalVolume
    v = L.around(hp, 6.0, points=pts)
    tg = LinTargets(ufi + [3.0, -2.0, 4.0], uriz - 2.5, band=0.5)
    yield "jac", [_a(G._jacobian(hp, tg, v.coords()), 6), _a(G._jacobian(hp, tg, v.coords(), step_mm=1.0), 6)]
    yield "jac.cliff", G._jacobian(hp, LinTargets(ufi, uriz, cliff=lambda h: h.upper_front_inner[1] > ufi[1]),
                                   v.coords())
    yield "jac.cliff_minus", G._jacobian(hp, LinTargets(ufi, uriz, cliff=lambda h: h.upper_front_inner[1] < ufi[1]),
                                         v.coords())
    yield "plain", _cand(G.genesis_solve(hp, tg, v))
    yield "plain.args", _cand(G.genesis_solve(hp, tg, v, start_shift=[1.0, 1.0, 1.0, 0.0, 0.0, 9.0], max_iter=2,
                                              step_mm=0.5, lam0=10.0))
    yield "already", _cand(G.genesis_solve(hp, LinTargets(ufi, uriz), v))
    yield "edge_hit", _cand(G.genesis_solve(hp, LinTargets(ufi + [1.0, 0, 0], uriz, band=1.05), v))
    yield "clamped", _cand(G.genesis_solve(hp, LinTargets(ufi + [30.0, 0, 0], uriz), v))
    yield "start_cliff", _cand(G.genesis_solve(hp, LinTargets(ufi, uriz, cliff=lambda h: True), v))
    yield "cliff_mid", _cand(G.genesis_solve(hp, LinTargets(ufi + [5.0, 0, 0], uriz,
                                                            cliff=lambda h: h.upper_front_inner[0] > ufi[0] + 2.0), v))
    yield "cliff_jac", _cand(G.genesis_solve(hp, LinTargets(ufi + [5.0, 0, 0], uriz,
                                                            cliff=lambda h: h.upper_front_inner[0] > ufi[0] + 4.0
                                                            and h.upper_front_inner[0] < ufi[0] + 4.3), v))
    box = G.KeepOutBox(ufi + [2.0, -30, -30], ufi + [3.0, 30, 30], "wall")
    vk = L.around(hp, 6.0, points=pts, keep_out=[box])
    yield "keepout", _cand(G.genesis_solve(hp, LinTargets(ufi + [5.0, 0, 0], uriz), vk))
    yield "keepout.start", _cand(G.genesis_solve(hp, tg, vk, start_shift=[2.5, 0, 0, 0, 0, 0]))
    nodes = Wall(lambda h: abs(h.upper_front_inner[2] - ufi[2]) > 1.0, "node")
    vn = L.around(hp, 6.0, points=pts, node_attachment=nodes)
    yield "node", _cand(G.genesis_solve(hp, tg, vn))
    yield "node.start", _cand(G.genesis_solve(hp, tg, vn, start_shift=[0, 0, 3.0, 0, 0, 0]))
    env = Wall(lambda h: h.upper_front_inner[1] < ufi[1] - 0.5, "env", margin=-0.25)
    ve = L.around(hp, 6.0, points=pts, wheel_envelope=env)
    yield "env", _cand(G.genesis_solve(hp, tg, ve))
    yield "env.start", _cand(G.genesis_solve(hp, tg, ve, start_shift=[0, -2.0, 0, 0, 0, 0]))
    prop = Wall(lambda h: float(h.upper_rear_inner[2]) < uriz - 1.0, "caster wall")
    vp = L.around(hp, 6.0, points=pts, properties=prop)
    yield "prop", _cand(G.genesis_solve(hp, tg, vp))
    yield "prop.start", _cand(G.genesis_solve(hp, tg, vp, start_shift=[0, 0, 0, 0, 0, -2.0]))
    two = [("p", "first", -0.1), ("p", "second", -3.0), ("q", "third", -1.0)]

    class Many(Wall):
        def violations(self, hp):
            return list(two)
    yield "start.worst", [_cand(G.genesis_solve(hp, tg, L.around(hp, 6.0, points=pts, node_attachment=Many(None)))),
                          _cand(G.genesis_solve(hp, tg, L.around(hp, 6.0, points=pts, wheel_envelope=Many(None)))),
                          _cand(G.genesis_solve(hp, tg, L.around(hp, 6.0, points=pts, properties=Many(None))))]
    yield "noprop_eval", _cand(G.genesis_solve(hp, tg, L.around(hp, 6.0, points=pts, properties=Wall(
        lambda h: False, bounds=[]))))
    fails = {"n": 0}
    real_solve = np.linalg.solve

    def flaky(a, b):
        fails["n"] += 1
        if fails["n"] <= 2:
            raise np.linalg.LinAlgError("sing")
        return real_solve(a, b)
    with mp.context() as m2:
        m2.setattr(G.np.linalg, "solve", flaky)
        yield "linalg", _cand(G.genesis_solve(hp, tg, v, max_iter=3))
    with mp.context() as m2:
        m2.setattr(G.np.linalg, "solve", lambda a, b: (_ for _ in ()).throw(np.linalg.LinAlgError("x")))
        yield "linalg.always", _cand(G.genesis_solve(hp, tg, v, max_iter=2))
    stuck = LinTargets(ufi + [3.0, 0, 0], uriz)
    with mp.context() as m2:
        m2.setattr(G, "_jacobian", lambda *a, **k: -np.eye(4, 6))
        yield "uphill", _cand(G.genesis_solve(hp, stuck, v, max_iter=4))
    calls = []
    with mp.context() as m2:
        m2.setattr(G, "_jacobian", lambda h, t, c, step_mm: calls.append(step_mm) or np.eye(4, 6))
        G.genesis_solve(hp, tg, v, max_iter=1, step_mm=0.7)
        G.genesis_solve(hp, tg, v, max_iter=1)
        yield "jac.step", calls
    real = G.GenesisTargets([G.TargetCurve("camber_deg", [-10.0, 10.0], [-1.6, -0.6], [0.05, 0.05])])
    yield "real", _cand(G.genesis_solve(hp, real, L.around(hp, 4.0, points=["upper_front_inner"]), max_iter=3))
    yield "zero_iter", [_cand(G.genesis_solve(hp, LinTargets(ufi + [0, 0, 1.0], uriz), v)),
                        _cand(G.genesis_solve(hp, LinTargets(ufi + [0, 0, 1.5], uriz), v, max_iter=0)),
                        _cand(G.genesis_solve(hp, LinTargets(ufi + [0, 0, 1.0], uriz), v, max_iter=0)),
                        _cand(G.genesis_solve(hp, LinTargets(ufi, uriz), v, start_shift=[1e-9, 0, 0, 0, 0, 0],
                                              max_iter=0))]
    # a wall the start sits exactly on: every step toward the target is refused, the whole ladder
    probe = G.KeepOutBox(ufi + [1.0, -30, -30], ufi + [9.0, 30, 30], "flush")
    flush = float(probe.clearances(ufi)[0])
    vf = L.around(hp, 6.0, points=pts, keep_out=[probe], min_clearance_mm=flush)
    yield "keepout.flush", _cand(G.genesis_solve(hp, LinTargets(ufi + [5.0, 0, 0], uriz), vf, max_iter=2))
    # a wall short of the target: the long step is refused, shorter ones land
    vs = L.around(hp, 9.0, points=pts, keep_out=[G.KeepOutBox(ufi + [3.5, -30, -30], ufi + [7.0, 30, 30], "short")])
    yield "keepout.short", _cand(G.genesis_solve(hp, LinTargets(ufi + [5.0, 0, 0], uriz), vs, max_iter=4))

    class Scripted(LinTargets):
        """Residual 1.2 at the start; anything moved lands at ``moved`` in every row."""

        def __init__(self, moved):
            super().__init__(ufi, uriz)
            self.moved = moved

        def residual(self, hp):
            x = float(hp.upper_front_inner[0]) - ufi[0]
            if abs(x) < 1e-12:
                return np.array([1.2, 0.0, 0.0, 0.0]), True
            return np.full(4, self.moved), True
    with mp.context() as m2:
        m2.setattr(G, "_jacobian", lambda *a, **k: np.eye(4, 6))
        yield "inband", [_cand(G.genesis_solve(hp, Scripted(1.0), v, max_iter=2)),
                         _cand(G.genesis_solve(hp, Scripted(1.5), v, max_iter=2))]

    class Overshoot(LinTargets):
        """A residual that punishes a long step: GN's first try overshoots, a damped one lands."""

        def residual(self, hp):
            x = float(hp.upper_front_inner[0]) - ufi[0]
            r0 = 3.0 - x if x <= 2.0 else 1.0 + 10.0 * (x - 2.0)
            return np.array([r0, 0.0, 0.0, 0.0]), True
    with mp.context() as m2:
        m2.setattr(G, "_jacobian", lambda *a, **k: -np.eye(4, 6))
        yield "overshoot", [_cand(G.genesis_solve(hp, Overshoot(ufi, uriz), v, max_iter=1)),
                            _cand(G.genesis_solve(hp, Overshoot(ufi, uriz), v, max_iter=3, lam0=0.5))]


def test_genesis_solve(hp, monkeypatch):
    check_lazy("inverse_genesis.solve", _solve_cases(hp, monkeypatch), rel=REL, abs_=1e-6)


# --------------------------------------------------------------------------- #
def _yield_cases(hp, mp):
    ufi = np.array(hp.upper_front_inner, float)
    uriz = float(hp.upper_rear_inner[2])
    tg = LinTargets(ufi + [0.2, 0.0, 0.0], uriz, band=1.0)
    S = ToleranceSpec
    fld = ToleranceField({"upper_front_inner": S([-0.5, -0.4, -0.3], [0.9, 0.4, 0.3]),
                          "upper_rear_inner": S.symmetric(0.6)})
    r0 = tg.residual(hp)[0]
    yield "build", [plain(G.build_yield(hp, tg, fld, r0, n=300)), plain(G.build_yield(hp, tg, fld, r0, n=200, seed=4,
                                                                                      step_mm=0.5)),
                    plain(G.build_yield(hp, tg, fld, r0 + 0.55, n=300))]
    yield "build.cliff", plain(G.build_yield(hp, LinTargets(ufi, uriz, cliff=lambda h: h.upper_front_inner[0] > ufi[0]),
                                             fld, r0, n=10))
    J = G._jacobian(hp, tg, fld.coords())
    yield "verify", [plain(G._verify_yield_full(hp, tg, fld, r0, J, 20, 0)),
                     plain(G._verify_yield_full(hp, tg, fld, r0 + 0.5, J, 15, 3)),
                     plain(G._verify_yield_full(hp, tg, fld, r0, None, 10, 1)),
                     plain(G._verify_yield_full(hp, LinTargets(ufi, uriz, cliff=lambda h: h.upper_rear_inner[0]
                                                               > hp.upper_rear_inner[0]), fld, r0, J, 12, 2))]
    yield "thresholds", plain(G.GenesisThresholds())
    edge = r0 + 0.8
    steps = []
    with mp.context() as m2:
        m2.setattr(G, "_jacobian", lambda h, t, c, step_mm=0.25: steps.append([step_mm, t is tg, c]) or np.zeros((4, 6)))
        G.build_yield(hp, tg, fld, r0, n=3, step_mm=0.7)
        G.build_yield(hp, tg, fld, r0, n=3)
    yield "build.steps", steps
    tg2 = LinTargets(ufi + [0.0, 0.0, 1.5], uriz)
    yield "verify.out_of_band", plain(G._verify_yield_full(hp, tg2, fld, r0, J, 6, 0))
    yield "build.defaults", [plain(G.build_yield(hp, tg, fld, edge)), plain(G.build_yield(hp, tg, fld, edge, n=400,
                                                                                         seed=1)),
                             plain(G.build_yield(hp, tg, fld, edge, n=400, step_mm=2.0))]
    still = ToleranceField({"upper_front_inner": S([0, 0, 0], [0, 0, 0])})
    one = np.array([1.0, -1.0, 0.5, 0.0])
    yield "build.still", [plain(G.build_yield(hp, tg, still, one, n=5)), plain(G.build_yield(hp, tg, still, one + 1e-9,
                                                                                            n=5))]

    class Exact(LinTargets):
        def residual(self, hp):
            return one.copy(), True

    class Refuses(LinTargets):
        def residual(self, hp):
            return np.zeros(4), False
    Js = G._jacobian(hp, tg, still.coords())
    yield "verify.still", [plain(G._verify_yield_full(hp, Exact(ufi, uriz), still, one, Js, 4, 0)),
                           plain(G._verify_yield_full(hp, Refuses(ufi, uriz), still, np.zeros(4), Js, 3, 0)),
                           plain(G._verify_yield_full(hp, tg, fld, r0 + 1.5, J, 12, 5))]


def test_build_yield(hp, monkeypatch):
    check_lazy("inverse_genesis.yield", _yield_cases(hp, monkeypatch), rel=REL, abs_=1e-6)


# --------------------------------------------------------------------------- #
def _C(ok=True, hit=True, shift=(0, 0, 0), frac=0.5, **kw):
    sv = np.array(shift, float)
    base = dict(ok=ok, hit=hit, shifts={"upper_front_inner": sv} if np.any(sv) else {}, shift_vec=sv,
                residual=np.array([frac, -frac / 2]), max_band_frac=frac, worst_row="camber @ +0.0 mm", iterations=3,
                clamped=[], keepout_rejections=0)
    base.update(kw)
    return G.Candidate(**base)


def _res(r):
    d = plain(r)
    d["winner_hp"] = None if r.winner_hp is None else _a(r.winner_hp.upper_front_inner, 6)
    d["candidates"] = [_cand(c) for c in r.candidates]
    for k in ("winner", "best_fit"):
        d[k] = None if getattr(r, k) is None else _cand(getattr(r, k))
    d["winner_is_best"] = r.winner is r.best_fit
    return d


def _flow_cases(hp, mp):
    pts = ["upper_front_inner"]
    L = G.LegalVolume
    ufi = np.array(hp.upper_front_inner, float)
    uriz = float(hp.upper_rear_inner[2])

    def vol(**kw):
        kw.setdefault("properties", Wall(lambda h: False, "pw"))
        kw.setdefault("wheel_envelope", Wall(lambda h: False, "ew"))
        return L.around(hp, 4.0, points=pts, **{k: x for k, x in kw.items() if x is not None})
    v = vol()
    tg = LinTargets(ufi, uriz)
    fld = ToleranceField({"upper_front_inner": ToleranceSpec.symmetric(0.3)})
    ig = G.inverse_genesis
    yield "nominal_fails", _res(ig(hp, LinTargets(ufi, uriz, cliff=lambda h: True), v, seed=3,
                                   thresholds=G.GenesisThresholds(0.5, 0.2, 0.9)))

    def script(cands, yields=None, volume=None):
        it = iter(cands)
        seen = []

        def solve(h, t, vol_, start_shift=None, max_iter=30, step_mm=0.25):
            seen.append([_a(start_shift, 6), max_iter, step_mm, h is hp, t is tg, vol_ is (volume or v)])
            return next(it)

        def by(h, t, f, r, n=4000, seed=0, step_mm=0.25):
            seen.append(["yield", n, seed, step_mm, _a(h.upper_front_inner, 6), t is tg, f is fld, _a(r, 6)])
            y = yields.pop(0)
            return (y, ["w"] if y is None else [])
        mp.setattr(G, "genesis_solve", solve)
        mp.setattr(G, "build_yield", by)
        return seen
    E = dict(ok=False, hit=False)
    bad = [_C(envelope_rejections=1, **E), _C(envelope_rejections=1, **E), _C(property_rejections=1, **E),
           _C(property_rejections=1, **E), _C(keepout_rejections=1, **E), _C(**E)]
    propw = Wall(lambda h: False, "anti", bounds=[G.PropertyBound("anti_squat_pct", 20, 60),
                                                  G.PropertyBound("kpi_deg", hi=9)])
    for name, kw, cs in (
            ("none.env", {}, bad), ("none.prop", dict(properties=propw), bad[2:]),
            ("none.keep", {}, bad[4:]), ("none.env_undeclared", dict(wheel_envelope=None), bad[:1]),
            ("none.prop_undeclared", dict(properties=None), bad[2:3]),
            ("none.prop_empty", dict(properties=Wall(lambda h: False, bounds=[])), bad[2:3])):
        vv = vol(**kw)
        seen = script(cs, volume=vv)
        yield f"flow.{name}", [_res(ig(hp, tg, vv, n_starts=len(cs), seed=1, max_iter=7, step_mm=0.3)), seen]
    nohit = [_C(hit=False, frac=3.0, shift=(1, 0, 0), clamped=["upper_front_inner.x"], keepout_rejections=2,
                node_rejections=1, envelope_rejections=4, property_rejections=5),
             _C(hit=False, frac=2.0, shift=(1.02, 0, 0)), _C(hit=False, frac=4.0, shift=(-1, 0, 0))]
    for name, kw, cs in (("nohit.all", dict(properties=propw), nohit),
                         ("nohit.prop_undeclared", dict(properties=None), [_C(hit=False, frac=2.0,
                                                                               property_rejections=1)]),
                         ("nohit.linkage", {}, [_C(hit=False, frac=2.5), _C(hit=False, frac=1.5, shift=(2, 0, 0))])):
        vv = vol(**kw)
        seen = script(cs, volume=vv)
        yield f"flow.{name}", [_res(ig(hp, tg, vv, n_starts=len(cs))), seen]
    fitonly = [_C(frac=0.8), _C(frac=0.3, shift=(1, 0, 0)), _C(hit=False, frac=1.4, shift=(2, 0, 0)),
               _C(hit=False, frac=1.2, shift=(3, 0, 0)), _C(frac=0.9, shift=(0.04, 0, 0)),
               _C(frac=0.95, shift=(1.05, 0, 0))]
    for name, f in (("fit.none", None), ("fit.empty", ToleranceField())):
        seen = script(list(fitonly))
        yield f"flow.{name}", [_res(ig(hp, tg, v, fld=f, n_starts=6)), seen]
    seen = script([_C(frac=0.3 + i / 10, shift=(i, 0, 0)) for i in range(7)])
    yield "flow.default_starts", [_res(ig(hp, tg, v)), len(seen)]
    for name, cs, ys, kw in (
            ("co.resilient_premium", [_C(frac=0.2), _C(frac=0.6, shift=(1, 0, 0)), _C(hit=False, frac=2, shift=(2, 0, 0))],
             [0.5, 0.97], {}),
            ("co.same", [_C(frac=0.2), _C(frac=0.6, shift=(1, 0, 0))], [0.96, 0.85], dict(n_yield=99, seed=5)),
            ("co.tie", [_C(frac=0.6), _C(frac=0.2, shift=(1, 0, 0))], [0.9, 0.9], {}),
            ("co.tempered", [_C(frac=0.2)], [0.8], {}), ("co.knife", [_C(frac=0.2), _C(frac=0.3, shift=(1, 0, 0))],
                                                         [0.79, None], {}),
            ("co.th", [_C(frac=0.2)], [0.5], dict(thresholds=G.GenesisThresholds(0.5, 0.4))),
            ("co.th2", [_C(frac=0.2)], [0.45], dict(thresholds=G.GenesisThresholds(0.5, 0.45))),
            ("co.none_y", [_C(frac=0.2)], [None], {}),
            ("co.zero_premium", [_C(frac=0.2), _C(frac=0.6, shift=(1, 0, 0))], [0.9, 0.9 + 1e-10], {}),
            ("co.nonhit_first", [_C(hit=False, frac=3.0), _C(frac=0.2, shift=(1, 0, 0)), _C(frac=0.4, shift=(2, 0, 0)),
                                 _C(hit=False, frac=2.0, shift=(3, 0, 0))], [0.97, 0.99], dict(step_mm=0.6)),
            ("co.none_then_hit", [_C(frac=0.2), _C(frac=0.5, shift=(1, 0, 0))], [None, 0.97], {}),
            ("co.zero_yield", [_C(frac=0.2)], [0.0], dict(thresholds=G.GenesisThresholds(0.0, 0.0))),
            ("co.zero_best", [_C(frac=0.2), _C(frac=0.6, shift=(1, 0, 0))], [0.0, 0.5],
             dict(thresholds=G.GenesisThresholds(0.0, 0.0)))):
        seen = script(cs, list(ys))
        yield f"flow.{name}", [_res(ig(hp, tg, v, fld=fld, n_starts=len(cs), **kw)), seen]
    mp.undo()
    calls = []

    def vfull(h, t, f, r, J, n, seed):
        calls.append([n, seed, J, _a(h.upper_front_inner, 6), t is tg, f is fld, _a(r, 6)])
        return (0.5, 0.97)
    for name, th, nv in (("verify.low", None, 7), ("verify.ok", G.GenesisThresholds(verify_agreement=0.97), 7),
                         ("verify.one", None, 1)):
        with mp.context() as m2:
            script([_C(frac=0.2, shift=(0.5, 0, 0))], [0.99])
            m2.setattr(G, "_verify_yield_full", vfull)
            m2.setattr(G, "_jacobian", lambda h, t, c, step_mm=0.25: f"J@{step_mm}:{h.upper_front_inner[0]:.3f}:{t is tg}:"
                                                                      f"{c}")
            yield f"flow.{name}", [_res(ig(hp, tg, v, fld=fld, n_starts=1, n_verify_full=nv, seed=2, thresholds=th,
                                           step_mm=0.4)), plain(calls)]
        calls.clear()
        mp.undo()
    walls = L.around(hp, 4.0, points=pts, node_attachment=Wall(lambda h: h.upper_front_inner[0] > ufi[0] + 1.0))
    seen = []
    with mp.context() as m2:
        m2.setattr(G, "genesis_solve", lambda h, t, vol, start_shift=None, **k: seen.append(_a(start_shift, 6))
                   or _C(hit=False, frac=2.0, shift=start_shift))
        yield "starts.walls", [_res(ig(hp, tg, walls, n_starts=4, seed=9)), seen[:]]
        seen.clear()
        yield "starts.plain", [_res(ig(hp, tg, L.around(hp, 4.0, points=pts), n_starts=3, seed=9)), seen[:]]
        seen.clear()
        yield "starts.zero", [_res(ig(hp, tg, v, n_starts=0)), seen[:]]
        seen.clear()
        for n in (1, 2, 3):
            never = L.around(hp, 4.0, points=pts, node_attachment=Wall(lambda h: True))
            yield f"starts.never.{n}", [_res(ig(hp, tg, never, n_starts=n, seed=1)), len(seen),
                                        never.node_attachment.calls]
            seen.clear()
        never_two = L.around(hp, 4.0, points=pts, node_attachment=Wall(lambda h: h.upper_front_inner[0] < ufi[0] + 3.9))
        yield "starts.rare", [_res(ig(hp, tg, never_two, n_starts=2, seed=0)), len(seen),
                              never_two.node_attachment.calls]
        seen.clear()
        twin = [_C(frac=0.5), _C(frac=0.5, shift=(0.05, 0, 0)), _C(frac=0.5, shift=(0.0, 0.03, 0.0))]
        it = iter(twin)
        m2.setattr(G, "genesis_solve", lambda *a, **k: next(it))
        yield "starts.dedupe", [len(_res(ig(hp, tg, L.around(hp, 4.0, points=pts), n_starts=3))["candidates"])]
    mp.undo()
    real = G.GenesisTargets([G.TargetCurve("camber_deg", [-10.0, 10.0], [-1.6, -0.6], [0.06, 0.06])])
    rv = L.around(hp, 3.0, points=["upper_front_inner"])
    rr = ig(hp, real, rv, fld=fld, n_starts=2, n_yield=200, n_verify_full=3, seed=0, max_iter=3)
    yield "real", _res(rr)
    yield "real.md", G.render_genesis_md(rr)


def test_inverse_genesis_flow(hp, monkeypatch):
    check_lazy("inverse_genesis.flow", _flow_cases(hp, monkeypatch), rel=REL, abs_=1e-6)


# --------------------------------------------------------------------------- #
def _md_cases(hp):
    th = G.GenesisThresholds()
    pb = G.SolvedPropertyBounds(bounds=[G.PropertyBound("caster_deg", 3, 7), G.PropertyBound("kpi_deg", hi=9),
                                        G.PropertyBound("anti_dive_pct", lo=10)])

    def R(**kw):
        base = dict(ok=True, reason="fine", candidates=[], winner=None, winner_hp=None, best_fit=None,
                    resilience_premium=None, n_starts=4, seed=2, verify_yield=None, verify_agreement=None,
                    thresholds=th, warnings=[])
        base.update(kw)
        return G.GenesisResult(**base)
    w = _C(frac=0.42, shift=(1.234, -0.5, 0.0), verdict="RESILIENT", yield_frac=0.9731, keepout_rejections=2,
           property_rejections=3, envelope_rejections=1, envelope_margin_mm=4.567, clamped=["upper_front_inner.x", "upper_front_inner.z"],
           properties={"caster_deg": 5.5, "kpi_deg": math.nan, "scrub_static_mm": 11.25, "anti_squat_pct": 30.0})
    bf = _C(frac=0.11, verdict="KNIFE_EDGE", yield_frac=0.5)
    nf = _C(hit=False, frac=2.5, verdict="NO_FIT")
    bare = _C(frac=0.7, verdict="ODD")
    env = G.WheelEnvelope(rack_travel_mm=5.0)
    yield "full", G.render_genesis_md(R(winner=w, best_fit=bf, candidates=[w, bf, nf], resilience_premium=0.4731,
                                        verify_yield=0.91, verify_agreement=0.955, warnings=["careful", "again"],
                                        property_bounds=pb, wheel_envelope=env))
    yield "failed", G.render_genesis_md(R(ok=False, reason="nope"))
    yield "plain_winner", G.render_genesis_md(R(winner=bare, best_fit=bare, candidates=[bare], resilience_premium=0.0))
    yield "env_no_rack", G.render_genesis_md(R(winner=_C(envelope_margin_mm=-1.0, verdict="TEMPERED"),
                                               wheel_envelope=G.WheelEnvelope()))
    yield "env_unknown", G.render_genesis_md(R(winner=_C(envelope_margin_mm=2.0, verdict="TEMPERED")))
    yield "props_no_bounds", G.render_genesis_md(R(winner=_C(properties={"kpi_deg": 8.0}, verdict="TEMPERED")))
    yield "premium_without_best", G.render_genesis_md(R(winner=w, resilience_premium=0.1, candidates=[w]))
    yield "premium_no_best_yield", G.render_genesis_md(R(winner=w, best_fit=_C(frac=0.1, verdict="NO_FIT"),
                                                         resilience_premium=0.2, candidates=[w]))
    yield "premium_none", G.render_genesis_md(R(winner=w, best_fit=bf, resilience_premium=None, candidates=[w, bf]))
    yield "two_no_winner", G.render_genesis_md(R(ok=False, reason="x", candidates=[nf, _C(hit=False, frac=3.0)],
                                                 property_bounds=G.SolvedPropertyBounds()))


def test_render_md(hp):
    check_lazy("inverse_genesis.md", _md_cases(hp), rel=REL, abs_=ABS)


# --------------------------------------------------------------------------- #
def _sweep_cases(hp, mp):
    T = G.TargetCurve
    tg = G.GenesisTargets([T("camber_deg", [0, 10], [-1.0, -1.2], [0.1, 0.2]), T("toe_deg", [5], [0.0], [0.05])],
                          track_mm=1234.0)
    calls = []
    base = Hardpoints.default()

    class Res:
        def __init__(self, winner, hp_):
            self.winner, self.winner_hp = winner, hp_

    def fake_ig(h, t, vol, fld=None, **kw):
        calls.append([[(c.channel, _a(c.band), _a(c.target), _a(c.travel_mm)) for c in t.curves], t.track_mm,
                      fld is not None, sorted(kw.items()), h is hp, vol is v])
        bands = sum(float(np.sum(c.band)) for c in t.curves)
        if bands > 0.45:
            return Res(None, None)
        h2 = base.copy()
        h2.upper_outer = np.array(base.upper_outer, float) + [bands * 10, 0.0, 0.0]
        h2.tie_rod_outer = np.array(base.tie_rod_outer, float) + [0.0, -bands * 100, 0.0]
        return Res(_C(frac=bands, verdict="TEMPERED", yield_frac=bands / 2), h2)
    mp.setattr(G, "inverse_genesis", fake_ig)
    v = G.LegalVolume.around(hp, 1.0, points=["upper_outer"])
    yield "default", [plain(G.band_sweep(hp, tg, v)), calls[:]]
    calls.clear()
    yield "args", [plain(G.band_sweep(hp, tg, v, scales=(0.5, 1.0, 1.0 + 1e-13, 3.0), channels=["toe_deg"],
                                      fld=ToleranceField(), n_starts=2, seed=4)), calls[:]]
    calls.clear()
    wide = G.GenesisTargets([T("camber_deg", [0, 10], [-1.0, -1.2], [0.3, 0.3])])
    yield "ref_fails", [plain(G.band_sweep(hp, wide, v, scales=(0.5, 2.0))), len(calls)]


def test_band_sweep(hp, monkeypatch):
    check_lazy("inverse_genesis.sweep", _sweep_cases(hp, monkeypatch), rel=REL, abs_=ABS)
