"""Characterization suite for suspension/genesis_walls.py (mutation gate); see tests/_golden.py. Load records in
every shape (full, partial, Mz or mz_Nmm, nothing), the deceleration wall (single biases, the balance search on
default and custom ranges), the link-structure wall (validation, per-member sections, margins, an empty table,
round trips from full and partial dicts, the shock-case constructor) and the rocker-bearing wall (named and
unnamed cases, rows, an empty table, round trips, the vehicle-case constructor)."""
from types import SimpleNamespace


from suspension import genesis_walls as gw
from suspension.kinematics import Hardpoints
from tests._golden import plain
from tests._golden_lazy import check_lazy


def _try(fn, *a, **k):
    try:
        return plain(fn(*a, **k))
    except (ValueError, KeyError, TypeError) as e:
        return {"raised": type(e).__name__, "message": str(e)}


def _wl(w):
    return [w.Fx, w.Fy, w.Fz, w.Mz, w.Fx_wc]


CASE = {"Fx": 1200.0, "Fy": -3400.0, "Fz": 2500.0, "Mz": 15000.0, "Fx_wc": 300.0}


def _cases(mp):
    gw._max_decel_g.cache_clear()
    N = SimpleNamespace
    yield "load", [gw._load_dict(N(**CASE)), gw._load_dict(N(Fz=900.0)), gw._load_dict(N(mz_Nmm=77.0, Fy=5.0)),
                   gw._load_dict(N(Mz=11.0, mz_Nmm=77.0)), gw._load_dict(N()), gw._load_dict(N(Fx=1, Fx_wc=2))]
    yield "wheel", [_wl(gw._wheel_load(CASE)), _wl(gw._wheel_load({})), _wl(gw._wheel_load({"Fy": 7.0, "Mz": 8.0})),
                    _wl(gw._wheel_load({"Fx": 1.0, "Fz": 2.0, "Fx_wc": 3.0}))]
    bd = gw.BrakeDecel()
    yield "decel", [bd.decel_g(0.6), bd.decel_g(0.6000004), gw.BrakeDecel(mass_kg=250.0, cg_height_mm=300.0,
                                                                         wheelbase_mm=1550.0, track_mm=1200.0
                                                                         ).decel_g(0.7)]
    yield "balanced.extra", [plain(bd.balanced()), plain(bd.balanced(0.123456, 0.123456, 0.01))]
    yield "balanced", [plain(bd.balanced(0.6, 0.7, 0.05)), plain(bd.balanced(lo=0.55, hi=0.56, step=0.01)),
                       plain(gw.BrakeDecel(cg_height_mm=600.0).balanced(0.5, 0.9, 0.1))]
    yield "brake.dict", [bd.to_dict(), plain(gw.BrakeDecel.from_dict({"mass_kg": "310", "track_mm": 1180}))]
    L = gw.LinkStructure
    yield "link.errors", [_try(L), _try(L, cases={"a": CASE}, per_member={"XX": [1, 2]})]
    ls = L(cases={"brake": CASE, "obj": N(Fz=3000.0, mz_Nmm=9.0)}, per_member={"TR": ("12.7", 1), "LF": [19.05, 1.2]})
    yield "link.norm", [ls.cases, ls.per_member]
    hp = Hardpoints.default()
    yield "link.margins", plain(ls.margins(hp))
    yield "link.fos", [ls.fos_min(hp), L(cases={"c": CASE}, od_mm=20.0, wall_mm=1.5, sy_mpa=500.0, e_mpa=200000.0,
                                         crook=0.002).fos_min(hp)]
    yield "link.dict", [ls.to_dict(), L(cases={"c": CASE}).to_dict()]
    rt = L.from_dict({"cases": {"c": CASE}})
    yield "link.from_partial", [rt.od_mm, rt.wall_mm, rt.per_member, rt.sy_mpa, rt.e_mpa, rt.crook]
    yield "link.from_full", L.from_dict(ls.to_dict()).to_dict() == ls.to_dict()
    odd = L(cases={"c": CASE}, od_mm=19.05, wall_mm=1.245, sy_mpa=460.0, e_mpa=200000.0, crook=0.002)
    yield "link.from_odd", L.from_dict(odd.to_dict()).to_dict()
    import suspension.robustness as RB
    seen = []
    mp.setattr(RB, "member_margins", lambda hp, loads, **kw: seen.append(sorted(kw.items())) or {})
    yield "link.empty", [ls.fos_min(hp), ls.margins(hp), plain(seen)]
    mp.setattr(RB, "shock_cases", lambda axle, brake_front, inboard_drive, inboard_brakes: seen.append(
        [axle, brake_front, inboard_drive, inboard_brakes]) or {"k": N(**CASE)})
    seen.clear()
    a = L.from_shock_cases("rear", 0.62)
    b = L.from_shock_cases("front", 0.7, inboard_drive=False, inboard_brakes=False, od_mm=19.05)
    yield "link.shock", [plain(seen), a.cases, b.od_mm]
    mp.undo()
    R = gw.RockerBearingWall
    yield "rocker.errors", _try(R)
    rw = R(cases=[dict(CASE, name="corner"), N(Fz=800.0, mz_Nmm=5.0), N(Fz=500.0, name="named")])
    yield "rocker.norm", rw.cases
    yield "rocker.rows", [plain(rw.rows(hp)), rw.s0_min(hp)]
    yield "rocker.custom", plain(R(cases=[dict(CASE, name="c")], label="L", C0_N=3000.0, spacing_mm=40.0, X0=0.5,
                                   Y0=0.4).rows(hp))
    yield "rocker.dict", [rw.to_dict(), plain(R.from_dict({"cases": [dict(CASE, name="c")], "C0_N": 1000})),
                          R.from_dict(rw.to_dict()).to_dict() == rw.to_dict()]
    rodd = R(cases=rw.cases, label="L2", C0_N=1500.0, spacing_mm=25.0, X0=1.3, Y0=0.45)
    yield "rocker.odd", [plain(rodd.rows(hp)), R.from_dict(rodd.to_dict()).to_dict()]
    yield "rocker.from_missing", _try(R.from_dict, {"cases": [dict(CASE, name="c")]})
    import suspension.actuation as AC
    mp.setattr(AC, "rocker_bearing_loads", lambda hp, cs, b: [])
    yield "rocker.empty", rw.s0_min(hp)
    mp.undo()
    import suspension.genesis_repro as GR
    seen.clear()
    mp.setattr(GR, "vehicle_load_cases", lambda axle, brake_bias_front: seen.append([axle, brake_bias_front]) or [
        dict(CASE, name="v1")])
    v = R.from_vehicle_cases("front", 0.66, C0_N=4000.0)
    yield "rocker.vehicle", [plain(seen), v.cases, v.C0_N]
    mp.undo()


def test_every_wall(monkeypatch):
    check_lazy("genesis_walls", _cases(monkeypatch), rel=1e-7, abs_=1e-9)
