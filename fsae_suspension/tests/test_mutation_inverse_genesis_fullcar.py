"""Characterization suite for suspension/inverse_genesis_fullcar.py (mutation gate); see tests/_golden.py.

One test per area so a mutant only runs the tests that reach it. The forward evaluation runs for real on a few
configurations (it is cheap); the pack-thermal integration, the staged search and the corner hand-off are driven
through stand-ins so their branches are reached in milliseconds."""
import math

import numpy as np

from suspension import inverse_genesis as IG
from suspension import inverse_genesis_fullcar as F
from suspension.kinematics import Hardpoints
from suspension.laptime import LapResult
from tests._golden import plain
from tests._golden_lazy import check_lazy

REL = 1e-6
ABS = 1e-8


def _try(fn, *a, **k):
    try:
        return plain(fn(*a, **k))
    except (ValueError, TypeError, KeyError, IndexError) as e:
        return {"raised": type(e).__name__, "message": str(e)}


def _sc(s):
    d = plain(s)
    d.pop("thermal", None)
    d["thermal_peak"] = None if s.thermal is None else plain(s.thermal.hottest_peak_c)
    return d


# --------------------------------------------------------------------------- #
def _rules_cases():
    c = F.CellSpec()
    yield "cell", [c.energy_kwh(), F.CellSpec(capacity_ah=5.0, nominal_v=3.7).energy_kwh(), plain(c.thermal.calibrated)]
    r = F.RuleMatrix()
    yield "rules", plain(r)
    for name, args in (("legal", (112, 5, c, 1550.0)), ("volts", (143, 5, c, 1550.0)), ("volts_edge", (142, 5, c, 1525.0)),
                       ("wb", (112, 5, c, 1524.0)), ("both", (150, 4, c, 1400.0))):
        yield f"viol.{name}", [r.violations(*args), r.segments_needed(*args[:3])]
    hot = F.CellSpec(max_v=130.0)
    yield "viol.cellv", [r.violations(10, 2, hot, 1600.0), r.segments_needed(10, 2, hot)]
    fat = F.CellSpec(capacity_ah=100.0)
    yield "viol.energy", [r.violations(100, 6, fat, 1600.0), r.segments_needed(100, 6, fat)]
    yield "viol.custom", [F.RuleMatrix(max_ts_voltage=400.0, max_segment_voltage=60.0, max_segment_energy_mj=3.0,
                                       min_wheelbase_mm=1600.0).violations(96, 4, c, 1599.0),
                          F.RuleMatrix(max_segment_energy_mj=3.0).segments_needed(96, 7, c),
                          F.RuleMatrix().segments_needed(1, 1, c), F.RuleMatrix().segments_needed(29, 1, c)]
    exact = F.CellSpec(max_v=4.0)
    unit = F.CellSpec(capacity_ah=1.0, nominal_v=1.0, max_v=1.0)
    yield "seg.integer", [F.RuleMatrix(max_segment_energy_mj=0.036).segments_needed(40, 1, unit),
                          F.RuleMatrix(max_segment_energy_mj=0.03599999).segments_needed(40, 1, unit),
                          F.RuleMatrix(max_segment_energy_mj=0.036).violations(40, 1, unit, 1600.0),
                          F.RuleMatrix(max_segment_energy_mj=0.0359).violations(9, 1, unit, 1600.0)]
    tiny = F.CellSpec(capacity_ah=1e-4, nominal_v=1.0, max_v=1.0)
    yield "seg.tiny", [F.RuleMatrix(max_segment_energy_mj=1e-6).violations(10, 1, tiny, 1600.0),
                       F.RuleMatrix(max_segment_energy_mj=1e-6).segments_needed(10, 1, tiny)]
    big = F.CellSpec(capacity_ah=1005000.9 / 3600.0, nominal_v=1.0, max_v=1.0)
    yield "seg.message", F.RuleMatrix(max_segment_energy_mj=1.0).violations(10, 1, big, 1600.0)
    yield "viol.exact", [F.RuleMatrix(max_ts_voltage=400.0).violations(100, 3, exact, 1525.0),
                         F.RuleMatrix(max_segment_voltage=4.0).violations(2, 1, exact, 1600.0),
                         F.RuleMatrix(max_segment_voltage=3.99).violations(2, 1, exact, 1600.0)]
    sp = F.DesignSpace()
    yield "space", [sp.rear_track(), F.DesignSpace(track_rear_mm=1180).rear_track(), sp.series_options(),
                    sp.parallel_options(), F.DesignSpace(series_range=(10, 13), series_step=0).series_options(),
                    F.DesignSpace(series_range=(10.9, 12.2), parallel_range=(2.7, 4.2)).series_options(),
                    F.DesignSpace(parallel_range=(2.7, 4.2)).parallel_options(), sp.ENV_CHANNEL, plain(sp)]
    yield "consts", [list(F._ARCHITECTURES), F._ARCH_LABEL, F._ARCH_MASS_KG, F._ARCH_DRIVE, F._ARCH_GRIP_FRAC,
                     F._ARCH_TV_YAW_FRAC, F._PHI, list(F._EVENT_KEYS), F._EVENT_LABEL, F._EVENT_FAMILY,
                     F._VERDICT_ICON, F.LBF_IN_TO_N_MM]
    for arch in F._ARCHITECTURES:
        cfg = F.FullCarConfig(100, 4, arch, 3.6789)
        yield f"derive.{arch}", [cfg.derive(sp, r), cfg.label()]
    yield "derive.capped", F.FullCarConfig(140, 7, "four_tv", 4.0).derive(F.DesignSpace(pack_overhead_frac=0.3,
                                                                                           pack_usable_frac=0.8),
                                                                          F.RuleMatrix(max_power_kw=50.0))
    yield "ref", [plain(F.PointsReference()), plain(F.PointsReference(1, 2, 3, 4))]
    yield "trapz", [F._trapz([0, 1, 2], [0, 1, 2]), F._trapz(np.array([1.0, 1.0]), np.array([0.0, 3.0]))]
    lap = LapResult(10.0, 5.0, 8.0, 2.0, 50.0, s=[0.0, 10.0, 20.0, 30.0], v=[2.0, 4.0, 5.0, 3.0])
    ta = F._TraceAdapter(lap)
    yield "trace", [plain(ta.distance), plain(ta.speed), plain(np.round(ta.long_g, 9)),
                    plain(np.round(F._TraceAdapter(lap, g=1.0).long_g, 9))]
    empty = F._TraceAdapter(LapResult(0.0, 0.0, 0.0, 0.0, 0.0))
    yield "trace.empty", [plain(empty.distance), plain(empty.speed), plain(empty.long_g)]
    one = F._TraceAdapter(LapResult(0.0, 0.0, 0.0, 0.0, 0.0, s=[3.0], v=[1.0]))
    yield "trace.one", [plain(one.distance), plain(one.speed), plain(one.long_g)]
    rep = F._TraceAdapter(LapResult(1.0, 1.0, 1.0, 1.0, 1.0, s=[0.0, 0.0, 5.0], v=[1.0, 2.0, 2.0]))
    yield "trace.repeat", plain(np.round(rep.long_g, 9))
    two = F._TraceAdapter(LapResult(1.0, 1.0, 1.0, 1.0, 1.0, s=[0.0, 4.0], v=[1.0, 3.0]))
    yield "trace.two", [plain(two.distance), plain(two.speed), plain(np.round(two.long_g, 9))]
    neg = F._TraceAdapter(LapResult(1.0, 1.0, 1.0, 1.0, 1.0, s=[0.0, 0.0, 5.0], v=[2.0, 1.0, 1.0]))
    yield "trace.neginf", plain(np.round(neg.long_g, 9))
    sh = F._LapParamsShim(300.0, 1.2, 1.1)
    yield "shim", [sh.__dict__, F._LapParamsShim(1, 2, 3, rolling_g=0.02).rolling_g]


def test_rules_space_and_config():
    check_lazy("inverse_genesis_fullcar.rules", _rules_cases(), rel=REL, abs_=ABS)


# --------------------------------------------------------------------------- #
def _veh(v):
    p = v.params if hasattr(v, "params") else v.p
    return [plain({k: getattr(p, k) for k in ("mass", "cg_height", "wheelbase", "track_front", "track_rear",
                                               "weight_dist_front", "roll_stiffness_front", "roll_stiffness_rear")}),
            v.grip_model_name(), round(float(v.max_lateral_g()), 9)]


def _eval_cases(mp):
    sp, r = F.DesignSpace(), F.RuleMatrix()
    cfg = F.FullCarConfig(112, 5, "single_diff", 3.8)
    d = cfg.derive(sp, r)
    yield "vehicle", [_veh(F._vehicle_for(sp, d)),
                      _veh(F._vehicle_for(F.DesignSpace(roll_stiffness_front=420.0, roll_stiffness_rear=310.0,
                                                        track_rear_mm=1150.0, tire_model="mf52_generic"), d)),
                      _veh(F._vehicle_for(F.DesignSpace(roll_stiffness_rear=200.0), d)),
                      _veh(F._vehicle_for(F.DesignSpace(cg_height_mm=320.0, wheelbase_mm=1600.0, track_mm=1250.0,
                                                        weight_dist_front=0.5), d))]
    yield "tire", [F._tire_for(sp), type(F._tire_for(F.DesignSpace(tire_model="mf52_generic"))).__name__,
                   _try(F._tire_for, F.DesignSpace(tire_model="mf61"))]
    for arch in F._ARCHITECTURES:
        pt = F._powertrain_for(F.FullCarConfig(112, 5, arch, 4.1), F.DesignSpace(cla=3.0, cda=1.3, wheel_radius_m=0.23,
                                                                                 motor_redline_rpm=7000.0,
                                                                                 motor_peak_torque_nm=150.0,
                                                                                 inverter_motor_eff=0.85),
                               dict(d, power_kw=55.0))
        mm = pt.motor_map
        yield f"pt.{arch}", [pt.power_kw, pt.drivetrain_eff, pt.cda, pt.cla, pt.drive, pt.max_tractive_n,
                             plain({k: getattr(mm, k) for k in vars(mm) if not k.startswith("_")})]
    tr, laps = F._endurance_track(r)
    L = tr.total_length()
    yield "track", [laps, round(L, 6), F._endurance_track(F.RuleMatrix(endurance_km=0.0))[1],
                    F._endurance_track(F.RuleMatrix(endurance_km=1.0))[1],
                    F._endurance_track(F.RuleMatrix(endurance_km=(3 * L - 0.5) / 1000.0))[1]]
    for name, c, s in (("feasible", cfg, sp), ("rwd_twin", F.FullCarConfig(112, 5, "twin_axle", 3.2), sp),
                       ("tv", F.FullCarConfig(100, 6, "four_tv", 4.5), sp),
                       ("short", F.FullCarConfig(88, 3, "single_diff", 4.0), sp),
                       ("killed", F.FullCarConfig(150, 5, "single_diff", 4.0), sp),
                       ("crash", F.FullCarConfig(112, 5, "single_diff", 4.0), F.DesignSpace(tire_model="nope"))):
        yield f"eval.{name}", _sc(F.evaluate_config(c, s, r))

    class FakeLap:
        def __init__(self, t, ok=True, warning=""):
            self.lap_time_s, self.ok, self.warning = t, ok, warning
            self.s, self.v = [0.0, 100.0, 200.0], [10.0, 20.0, 15.0]

        def total_length(self):
            return 200.0
    got = []
    with mp.context() as m2:
        m2.setattr(F, "acceleration_time", lambda v, p, distance_m: got.append(["accel", distance_m, p.power_kw,
                                                                                 v.p.mass]) or FakeLap(4.5, False,
                                                                                                       "slip"))
        m2.setattr(F, "skidpad_time", lambda v, p: got.append(["skid", p.power_kw]) or FakeLap(5.1, False, "wide"))
        m2.setattr(F, "simulate_lap", lambda v, t, p: FakeLap(55.0, False, "lap"))
        yield "eval.warnings", [_sc(F.evaluate_config(cfg, sp, r, run_thermal=True)), plain(got)]
        m2.setattr(F, "simulate_lap", lambda v, t, p: FakeLap(math.nan, True))
        yield "eval.nan", _sc(F.evaluate_config(cfg, sp, r))
        m2.setattr(F, "skidpad_time", lambda v, p: FakeLap(math.inf))
        m2.setattr(F, "acceleration_time", lambda v, p, distance_m: FakeLap(4.0))
        m2.setattr(F, "simulate_lap", lambda v, t, p: FakeLap(50.0))
        yield "eval.inf_skid", _sc(F.evaluate_config(cfg, sp, r))
    calls = []
    with mp.context() as m2:
        m2.setattr(F, "_thermal_gate", lambda *a: calls.append([type(x).__name__ for x in a]) or (None, 7))
        yield "eval.overheat", [_sc(F.evaluate_config(cfg, sp, r, run_thermal=True)), calls[:]]
        m2.setattr(F, "_thermal_gate", lambda *a: (None, None))
        yield "eval.cool", _sc(F.evaluate_config(cfg, sp, r, run_thermal=True))
        m2.setattr(F, "_thermal_gate", lambda *a: (None, 3))
        yield "eval.short_and_hot", _sc(F.evaluate_config(F.FullCarConfig(88, 3, "single_diff", 4.0), sp, r,
                                                          run_thermal=True))
    pc = []

    def fake_trace(adapter, shim, pack_nominal_v, inverter_motor_eff, regen_eff, regen_max_g, regen_enabled):
        pc.append([pack_nominal_v, inverter_motor_eff, regen_eff, regen_max_g, regen_enabled, shim.mass, shim.cd_a,
                   shim.rho])
        return np.array([0.0, 1.0, 2.0, 3.0]), np.array([100.0, -40.0, 60.0, -500.0])
    with mp.context() as m2:
        m2.setattr(F, "pack_current_trace", fake_trace)
        yield "eval.energy", [_sc(F.evaluate_config(cfg, F.DesignSpace(inverter_motor_eff=0.8, regen_eff=0.4,
                                                                       regen_max_g=0.2, cda=1.4), r)), pc[:]]
        m2.setattr(F, "pack_current_trace", lambda *a, **k: (np.array([0.0, 1.0]), np.array([1e9, 1e9])))
        yield "eval.derate", _sc(F.evaluate_config(cfg, sp, r))
        m2.setattr(F, "pack_current_trace", lambda *a, **k: (np.array([0.0, 1.0]), np.array([0.0, 0.0])))
        yield "eval.zero_energy", _sc(F.evaluate_config(cfg, F.DesignSpace(pack_usable_frac=0.0), r))
        m2.setattr(F, "pack_current_trace", lambda *a, **k: (np.array([0.0, 1.0]), np.array([20.0, 20.0])))
        yield "eval.small_energy", _sc(F.evaluate_config(cfg, F.DesignSpace(pack_usable_frac=0.0), r))
        m2.setattr(F, "pack_current_trace", lambda *a, **k: (np.array([0.0, 1.0]), np.array([-5.0, -5.0])))
        yield "eval.regen_only", _sc(F.evaluate_config(cfg, sp, r))
        m2.setattr(F, "pack_current_trace", lambda *a, **k: (np.array([0.0, 1.0]), np.array([np.inf, np.inf])))
        yield "eval.inf_energy", _sc(F.evaluate_config(cfg, sp, r))


def test_evaluate_config(monkeypatch):
    check_lazy("inverse_genesis_fullcar.eval", _eval_cases(monkeypatch), rel=1e-10, abs_=1e-9)


# --------------------------------------------------------------------------- #
class Res:
    def __init__(self, ok=True, breached=False, ttl=None, idx=1, peak=58.0):
        self.ok, self.any_cell_breached_limit, self.hottest_cell_index, self.hottest_peak_c = ok, breached, idx, peak
        self.time_to_limit_s = np.array([math.nan, math.nan if ttl is None else ttl, 5.0])

    def __repr__(self):
        return "Res"


def _thermal_cases(mp):
    seen = []

    class Model:
        def __init__(self, layout, fans, airflow):
            seen.append(["model", layout.rows, layout.cols, layout.series, layout.parallel, layout.ambient_c,
                         layout.cell.temp_limit_c, layout.cell.r_internal_ohm, fans, airflow])
            self.layout = layout

        def simulate(self, t, cur, init_temp_c, n_laps):
            seen.append(["sim", plain(t), plain(cur), init_temp_c, n_laps])
            return out.pop(0)

    def trace(adapter, shim, pack_nominal_v, inverter_motor_eff, regen_eff, regen_max_g, regen_enabled):
        seen.append(["trace", round(pack_nominal_v, 9), inverter_motor_eff, regen_eff, regen_max_g, regen_enabled,
                     adapter, type(shim).__name__])
        return (np.array([2.0, 2.25, 2.5]) if short else np.array([2.0, 12.0, 32.0])), np.array([50.0, -10.0, 30.0])
    mp.setattr(F, "PackThermalModel", Model)
    mp.setattr(F, "pack_current_trace", trace)
    sp = F.DesignSpace(thermal_rows=3, thermal_cols=4, ambient_c=27.0, inverter_motor_eff=0.85, regen_eff=0.5,
                       regen_max_g=0.3)
    rules = F.RuleMatrix(cell_temp_limit_c=55.0)
    cfg = F.FullCarConfig(96, 4, "twin_axle", 4.0)
    out = [Res(), Res(breached=True, ttl=95.0), Res(breached=True, ttl=1e6), Res(breached=True, ttl=math.nan),
           Res(ok=False, breached=True, ttl=1.0), Res(breached=True, ttl=0.0), Res(breached=True, ttl=30.0),
           Res(breached=True, ttl=29.999)]
    short = False
    for i in range(8):
        r, lap = F._thermal_gate(cfg, sp, rules, "adapter", F._LapParamsShim(1, 1, 1), {}, 12)
        yield f"gate.{i}", [lap, seen[:]]
        seen.clear()
    short = True
    out = [Res(breached=True, ttl=0.3), Res(breached=True, ttl=1.7)]
    yield "gate.short", [F._thermal_gate(cfg, sp, rules, "a", F._LapParamsShim(1, 1, 1), {}, 12)[1] for _ in range(2)]
    seen.clear()
    orig_limit = sp.cell.thermal.temp_limit_c
    yield "gate.cell_untouched", orig_limit == F.DesignSpace().cell.thermal.temp_limit_c


def test_thermal_gate(monkeypatch):
    check_lazy("inverse_genesis_fullcar.thermal", _thermal_cases(monkeypatch), rel=REL, abs_=ABS)


# --------------------------------------------------------------------------- #
def _S(series=100, parallel=4, arch="single_diff", fd=3.5, ok=True, times=(4.0, 5.0, 50.0, 1100.0), verdict="FEASIBLE",
       margin=1.0, tot=None, kill=None, overheat=None, thermal=None):
    cfg = F.FullCarConfig(series, parallel, arch, fd)
    s = F.ConfigScore(config=cfg, ok=ok, derived=cfg.derive(F.DesignSpace(), F.RuleMatrix()),
                      accel_s=times[0], skidpad_s=times[1], autocross_s=times[2], endurance_s=times[3],
                      energy_margin_kwh=margin, verdict=verdict, kill_reasons=list(kill or []), overheat_lap=overheat,
                      thermal=thermal)
    if tot is not None:
        s.total_points = tot
    return s


def _search_cases(mp):
    field = [_S(times=(4.0, 5.0, 50.0, 1100.0)), _S(times=(4.4, 5.2, 52.0, 1000.0), verdict="THERMAL_DNF"),
             _S(times=(3.9, math.nan, 51.0, 1050.0), verdict="ENERGY_SHORT"), _S(ok=False),
             _S(times=(0.0, 5.0, 49.0, math.inf))]
    F._score_field(field, F.PointsReference())
    yield "score.relative", [[s.points, s.total_points] for s in field]
    F._score_field(field, F.PointsReference(accel_s=3.5, skidpad_s=-1.0, autocross_s=None, endurance_s=math.nan))
    yield "score.declared", [[s.points, s.total_points] for s in field]
    nan_ev = [_S(times=(math.nan, 5.0, 50.0, 1000.0)), _S(times=(math.inf, 5.5, 51.0, 1001.0))]
    F._score_field(nan_ev, F.PointsReference())
    yield "score.all_nan", [[s.points, s.total_points] for s in nan_ev]
    edge = [_S(times=(4.0, 5.0, 50.0, 1000.0))]
    F._score_field(edge, F.PointsReference(accel_s=0.5, skidpad_s=0.0))
    yield "score.edge", [edge[0].points, edge[0].total_points]
    F._score_field([_S(ok=False)], F.PointsReference())
    yield "score.none", F._score_field([], F.PointsReference())
    d = F.SearchDiagnostics(10, 3, 40, 8, 2, 160, 12.345)
    yield "diag", [d.summary(), d.timing(), F.SearchDiagnostics().summary(), F.SearchDiagnostics().timing()]
    calls = []

    def fake_eval(cfg, space, rules, run_thermal=False):
        calls.append([round(cfg.final_drive, 9), run_thermal, space is gear_space, rules is gear_rules])
        fd = cfg.final_drive
        t = 4.0 + (fd - script["best"]) ** 2
        return _S(cfg.series, cfg.parallel, cfg.architecture, fd, times=(t, 5.0, 50.0 + t, 1000.0 + t))
    mp.setattr(F, "evaluate_config", fake_eval)
    gear_space, gear_rules = F.DesignSpace(), F.RuleMatrix()
    full = F.PointsReference(3.9, 4.9, 49.0, 990.0)
    for name, best, iters, ref in (("mid", 3.9, 8, full), ("low", 2.6, 3, full),
                                   ("high", 5.2, 5, F.PointsReference(accel_s=4.0)), ("zero", 3.0, 0, full),
                                   ("flat", 0.0, 2, F.PointsReference()), ("c2", 4.2, 1, full),
                                   ("c1", 3.6, 1, full), ("relative", 3.9, 3, F.PointsReference())):
        script = {"best": best}
        diag = F.SearchDiagnostics()
        w, s = F._gear_refine(F.FullCarConfig(100, 4, "twin_axle", 0.0), gear_space, gear_rules, ref, diag,
                              iters=iters)
        yield f"gear.{name}", [round(w.final_drive, 9), w.label(), round(s.total_points, 6), plain(diag), calls[:]]
        calls.clear()
    script = {"best": 3.0}
    diag = F.SearchDiagnostics()
    gear_space = F.DesignSpace(final_drive_range=(2.0, 3.0))
    F._gear_refine(F.FullCarConfig(100, 4, "twin_axle", 0.0), gear_space, gear_rules, full, diag)
    yield "gear.default_iters", [plain(diag), calls[:]]
    calls.clear()

    class Th:
        def __init__(self, peak):
            self.hottest_peak_c, self.ok = peak, True

        def __repr__(self):
            return "Th"

    def gated(cfg, space_, rules_, run_thermal=False):
        calls.append([cfg.label(), run_thermal, space_ is space, isinstance(rules_, F.RuleMatrix)])
        key = (cfg.series, cfg.parallel, cfg.architecture)
        v = plan.get(key, "FEASIBLE") if run_thermal else "FEASIBLE"
        base = 4.0 + cfg.series / 1000 + cfg.parallel / 100 + {"single_diff": 0.3, "twin_axle": 0.1,
                                                                "four_tv": 0.2}[cfg.architecture]
        base += (cfg.final_drive - 3.5) ** 2
        if key in flat_times:
            base = 4.5
        if key in broken:
            return _S(cfg.series, cfg.parallel, cfg.architecture, cfg.final_drive, ok=False, verdict="FAILED",
                      kill=["nope"])
        return _S(cfg.series, cfg.parallel, cfg.architecture, cfg.final_drive,
                  times=(base, 5.0, 50.0 * base / 4, 1000.0 * base / 4), verdict=v,
                  kill=["too hot"] if v == "THERMAL_DNF" else (["short"] if v == "ENERGY_SHORT" else []),
                  overheat=4 if v == "THERMAL_DNF" else None, thermal=Th(52.5) if run_thermal else None,
                  margin=0.5 if v != "ENERGY_SHORT" else -0.3)
    mp.setattr(F, "evaluate_config", gated)
    space = F.DesignSpace(series_range=(96, 104), series_step=8, parallel_range=(4, 5),
                          architectures=("single_diff", "twin_axle"))
    for name, kw, pl, br in (
            ("default", {}, {}, set()),
            ("hot_top", dict(n_finalists=3, gear_iters=1), {(96, 4, "twin_axle"): "THERMAL_DNF"}, set()),
            ("none_finish", dict(n_finalists=2, gear_iters=1), {(96, 4, "twin_axle"): "THERMAL_DNF",
                                                                (104, 4, "twin_axle"): "ENERGY_SHORT"}, set()),
            ("one_finalist", dict(n_finalists=0, gear_iters=1), {}, set()),
            ("tie", dict(n_finalists=2, gear_iters=1), {(96, 4, "single_diff"): "ODD"}, set()),
            ("all_broken", dict(gear_iters=1), {}, {(96, 4, "single_diff"), (96, 4, "twin_axle"), (96, 5, "single_diff"),
                                                    (96, 5, "twin_axle"), (104, 4, "single_diff"),
                                                    (104, 4, "twin_axle"), (104, 5, "single_diff"),
                                                    (104, 5, "twin_axle")})):
        plan, broken = pl, br
        flat_times = {(96, 4, "single_diff"), (96, 5, "single_diff")} if name == "tie" else set()
        rules = F.RuleMatrix()
        ref = F.PointsReference() if name != "default" else None
        r = F.synthesize_fullcar(space, rules, ref, **kw)
        yield f"synth.{name}", [r.ok, r.reason, None if r.winner is None else _sc(r.winner),
                                [_sc(s) for s in r.finalists], [[s.config.label(), s.total_points] for s in r.ranked],
                                [_sc(s) for s in r.rule_killed], {k: v for k, v in plain(r.diagnostics).items()
                                                                   if k != "elapsed_s"}, r.relative_scoring,
                                r.warnings, plain(calls), 0 <= r.diagnostics.elapsed_s < 60, r.space is space,
                                r.rules is rules, plain(r.points_ref), ref is None or r.points_ref is ref]
        calls.clear()
    plan, broken, flat_times = {}, set(), set()
    cal = F.DesignSpace(series_range=(96, 96), parallel_range=(4, 4), architectures=("twin_axle",))
    cal.cell.thermal.calibrated = True
    r = F.synthesize_fullcar(cal, None, F.PointsReference(4.0, 5.0, 50.0, 1000.0), gear_iters=1)
    yield "synth.absolute_calibrated", [r.ok, r.reason, r.warnings, r.relative_scoring, r.space is cal,
                                        plain(r.rules), plain(r.points_ref)]
    yield "synth.partial_ref", F.synthesize_fullcar(cal, None, F.PointsReference(endurance_s=900.0),
                                                    gear_iters=1).relative_scoring
    ks, kr, kref = (F.DesignSpace(series_range=(150, 158), series_step=8, parallel_range=(4, 4),
                                  architectures=("four_tv",)), F.RuleMatrix(), F.PointsReference(accel_s=4.0))
    killed = F.synthesize_fullcar(ks, kr, kref, gear_iters=1)
    yield "synth.killed", [killed.ok, killed.reason, [_sc(s) for s in killed.rule_killed], plain(killed.diagnostics.n_grid),
                           killed.diagnostics.n_rule_killed, killed.warnings, killed.ranked, killed.finalists,
                           killed.winner, killed.space is ks, killed.rules is kr, killed.points_ref is kref,
                           killed.relative_scoring, 0 <= killed.diagnostics.elapsed_s < 60]
    broken = {(96, 4, "twin_axle")}
    one = F.DesignSpace(series_range=(96, 96), parallel_range=(4, 4), architectures=("twin_axle",))
    br_rules, br_ref = F.RuleMatrix(), F.PointsReference(accel_s=4.0)
    nob = F.synthesize_fullcar(one, br_rules, br_ref, gear_iters=1)
    yield "synth.broken_fields", [nob.ok, nob.space is one, nob.rules is br_rules, nob.points_ref is br_ref,
                                  nob.relative_scoring, nob.warnings, 0 <= nob.diagnostics.elapsed_s < 60]
    broken = set()
    w = _S(tot=80.0, margin=0.75, thermal=Th(51.2))
    hot = _S(102, tot=80.0, verdict="THERMAL_DNF", kill=["too hot on lap 9"])
    hot_low = _S(102, tot=79.999998, verdict="THERMAL_DNF", kill=["x"])
    noreason = _S(102, tot=81.0, verdict="ENERGY_SHORT")
    yield "reason.win", [F._winner_reason(w, [w], True), F._winner_reason(w, [hot, w], False),
                         F._winner_reason(w, [w, hot_low], True), F._winner_reason(w, [_S(103, tot=79.0, verdict="THERMAL_DNF",
                                                                                         kill=["y"]), w], True),
                         F._winner_reason(w, [noreason, hot, w], True), F._winner_reason(w, [w, _S(104, tot=90.0)], True)]
    yield "reason.none", [F._no_survivor_reason([hot, noreason, hot_low]), F._no_survivor_reason([noreason])]


def test_search(monkeypatch):
    check_lazy("inverse_genesis_fullcar.search", _search_cases(monkeypatch), rel=REL, abs_=ABS)


# --------------------------------------------------------------------------- #
def _corner_cases(mp):
    sp = F.DesignSpace(track_mm=1250.0)
    hp = Hardpoints.default()
    sc = _S()
    sc.peak_lat_g = 1.6

    def tg(t):
        return [[c.channel, plain(np.round(c.travel_mm, 9)), plain(np.round(c.target, 6)), plain(c.band)]
                for c in t.curves] + [t.track_mm]
    nan_sc = _S()
    yield "intent.default", tg(F.kinematic_intent_for(sc, sp))
    yield "intent.args", tg(F.kinematic_intent_for(nan_sc, sp, hp=hp, stations_mm=[-10.0, 0.0, 20.0],
                                                   roll_gradient_deg_per_g=2.0, bands={"camber_deg": 0.5}))
    yield "intent.none", tg(F.kinematic_intent_for(None, F.DesignSpace(), stations_mm=[5.0, 15.0]))
    yield "intent.peak", tg(F.kinematic_intent_for(sc, sp, peak_lat_g="1.1", stations_mm=[0.0, 7.0]))
    yield "intent.single", tg(F.kinematic_intent_for(sc, sp, stations_mm=[3.0]))
    yield "intent.declared", [tg(F.kinematic_intent_for(sc, sp, declared={"static_camber": -2.0, "camber_gain": -0.04,
                                                                          "toe": 0.05, "rc_height": 50.0, "scrub": 9.0},
                                                        bands={"toe_deg": 0.2, "scrub_mm": 4.5})),
                              tg(F.kinematic_intent_for(sc, sp, declared={"camber_gain": 0.0, "rc_height": 40.0},
                                                        stations_mm=[-5.0, 5.0]))]
    with mp.context() as m2:
        m2.setattr(IG, "curves_of", lambda h, s: ({}, False))
        yield "intent.fallback", tg(F.kinematic_intent_for(sc, sp, stations_mm=[-10.0, 10.0]))
    with mp.context() as m2:
        m2.setattr(IG, "curves_of", lambda h, s: ({c: np.array([]) for c in IG.CHANNELS}, True))
        yield "intent.empty", _try(lambda: tg(F.kinematic_intent_for(sc, sp, stations_mm=[-10.0, 10.0])))
    seen = []

    def fake_ig(h, t, vol, fld=None, **kw):
        seen.append([plain(np.round(h.upper_front_inner, 6)), tg(t), vol.points(), plain(vol.boxes), fld,
                     sorted(kw.items())])
        return "result"
    mp.setattr(IG, "inverse_genesis", fake_ig)
    yield "hp.default", [F.synthesize_hardpoints(sc, sp), seen[:]]
    seen.clear()
    h2 = hp.copy()
    h2.upper_front_inner = np.array(h2.upper_front_inner, float) + 1.0
    vol = IG.LegalVolume.around(h2, 2.0, points=["lower_outer"])
    yield "hp.args", [F.synthesize_hardpoints(sc, sp, hp=h2, volume=vol, fld="F", intent_kw={"stations_mm": [0.0, 9.0]},
                                              n_starts=2, seed=7), seen[:]]
    mp.undo()
    lc = F.load_case_for(sc, sp)
    yield "load", [lc.fz_n, lc.mu_lateral, {k: round(v, 6) for k, v in lc.member_forces.items()}, round(lc.condition, 6),
                   lc.note]
    lc2 = F.load_case_for(_S(), F.DesignSpace(weight_dist_front=0.5), hp=h2)
    yield "load.nan", [lc2.fz_n, lc2.mu_lateral, {k: round(v, 6) for k, v in lc2.member_forces.items()}]
    bare = F.ConfigScore(config=F.FullCarConfig(1, 1, "single_diff", 1.0), ok=False, derived={})
    yield "load.bare", [F.load_case_for(bare, sp).fz_n, F.load_case_for(bare, sp).mu_lateral]
    import suspension.elastokinematics as EK
    got = []

    def fake_solve(h, spec):
        got.append([spec.Fx, spec.Fy, spec.Fz, spec.Mz, spec.n_steps, type(spec.stiffness).__name__,
                    plain(np.round(h.upper_front_inner, 6))])
        return "ek"
    with mp.context() as m2:
        m2.setattr(EK, "solve_elastokinematic", fake_solve)
        yield "ek", [F.elastokinematic_check(sc, sp), F.elastokinematic_check(bare, F.DesignSpace(weight_dist_front=0.4),
                                                                               hp=h2, stiffness=EK.StiffnessField(),
                                                                               n_steps=3), got[:]]
    tw = []
    with mp.context() as m2:
        m2.setattr(EK, "frame_twist_toe", lambda *a: tw.append(a) or 0.123)
        yield "twist", [F.frame_twist_toe_budget(sc, sp, 1500.0, 300.0),
                        F.frame_twist_toe_budget(bare, sp, 2000.0, 250.0, lever_mm=150.0, toe_per_mm=0.5, hp=h2,
                                                 cg_height_mm=300.0, wheelbase_mm=1550.0), plain(tw)]


def test_corner_handoff(monkeypatch):
    check_lazy("inverse_genesis_fullcar.corner", _corner_cases(monkeypatch), rel=REL, abs_=1e-6)


# --------------------------------------------------------------------------- #
def _declared_cases(mp):
    D = F.DeclaredCar
    car = D()
    hp = Hardpoints.default()
    hp.static_camber = -2.0
    yield "vehicle", _veh(car.vehicle())
    yield "vehicle.over", _veh(car.vehicle(mass_kg=250.0, tire_model="linear", track_rear_mm=1180.0, arb_rate_front=5.0))
    rig = D(front_hp=hp, rear_hp=hp, use_spring_rates=True, spring_rate_front=40.0, spring_rate_rear=30.0,
            arb_rate_front=2.0, arb_rate_rear=1.0, weight_dist_front=0.46, cg_height_mm=300.0, wheelbase_mm=1580.0,
            track_front_mm=1220.0, track_rear_mm=1190.0, roll_stiffness_front=400.0, roll_stiffness_rear=330.0)
    v = rig.vehicle()
    yield "vehicle.rig", [_veh(v), v.p.use_spring_rates, v.p.spring_rate_front, v.p.spring_rate_rear,
                          v.p.arb_rate_front, v.p.arb_rate_rear, v.p.static_camber_front,
                          v.p.static_camber_rear, v.front_kin is not None, v.rear_kin is not None,
                          D().vehicle().p.static_camber_front, D().vehicle().p.static_camber_rear]
    pt = car.powertrain(power_kw=60.0, cla=2.0)
    yield "powertrain", [pt.power_kw, pt.cla, pt.cda, pt.drive, D(drive="awd", cda=1.3).powertrain().drive,
                         D(cda=1.3).powertrain().cda]
    yield "lap", [round(car.lap().lap_time_s, 6), round(car.lap(mass_kg=260.0).lap_time_s, 6)]
    yield "summary", plain(car.summary())
    yield "summary.rig", plain(rig.summary(lateral_g=1.2))
    direct = hp.copy()
    direct.actuation = "direct"
    proxy = hp.copy()
    proxy.rocker_pivot = None
    yield "summary.mr", plain([D(front_hp=direct, rear_hp=proxy).summary()["motion_ratio"],
                               D(front_hp=proxy, rear_hp=direct).summary()["motion_ratio"]])
    yield "summary.liftoff", [repr(D().summary()["inside_front_liftoff_g"]),
                              repr(D(cg_height_mm=900.0, track_front_mm=800.0, track_rear_mm=800.0)
                                   .summary()["inside_front_liftoff_g"])]
    yield "summary.equal_roll", plain(D(roll_stiffness_front=320.0, roll_stiffness_rear=320.0).summary()[
        "front_roll_share"])
    yield "summary.zero_roll", plain(D(roll_stiffness_front=0.0, roll_stiffness_rear=0.0).summary()["front_roll_share"])
    from suspension.laptime import default_autocross
    yield "sens.share", plain(F.lap_sensitivity(car, "front_roll_share", [0.4, 0.6]))
    yield "sens.mass", plain(F.lap_sensitivity(D(use_spring_rates=True), "mass_kg", [280.0, 300.0, 320.0],
                                               track=default_autocross()))
    yield "sens.bad", _try(F.lap_sensitivity, car, "bogus", [1])
    from suspension.laptime import Track
    short = default_autocross()
    short = Track(name="short", segments=short.segments[:6])
    yield "sens.track", plain(F.lap_sensitivity(car, "mass_kg", [290.0], track=short))
    yield "sens.share_springs", plain(F.lap_sensitivity(D(use_spring_rates=True, front_hp=hp), "front_roll_share",
                                                        [0.5]))


def test_declared_car(monkeypatch):
    check_lazy("inverse_genesis_fullcar.declared", _declared_cases(monkeypatch), rel=REL, abs_=1e-6)


# --------------------------------------------------------------------------- #
def _export_cases():
    hp = Hardpoints.default()
    yield "csv", F.export_hardpoints_csv(hp)
    h2 = hp.copy()
    h2.wheel_center = None
    yield "csv.missing", F.export_hardpoints_csv(h2)

    class Bare:
        pass
    b = Bare()
    for k in ("upper_front_inner", "lower_outer"):
        setattr(b, k, getattr(hp, k))
    yield "csv.bare", F.export_hardpoints_csv(b)
    sp, r = F.DesignSpace(regen_max_g=0.312, regen_eff=0.61), F.RuleMatrix(cell_temp_limit_c=58.5)
    for arch in F._ARCHITECTURES:
        s = _S(112, 5, arch, 3.87654)
        yield f"c.{arch}", F.export_flash_constants_c(s, sp, r)
        yield f"py.{arch}", F.export_flash_constants_py(s, sp, r)
    low = _S(10, 1, "single_diff", 2.0)
    low.derived = dict(low.derived, pack_nominal_v=0.5)
    yield "lowv", [F.export_flash_constants_c(low, sp, r), F.export_flash_constants_py(low, sp, r)]

    class Th:
        def __init__(self, ok=True):
            self.ok, self.hottest_peak_c = ok, 57.6

        def __repr__(self):
            return "Th"
    diag = F.SearchDiagnostics(12, 4, 30, 6, 2, 120, 3.0)

    def R(**kw):
        base = dict(ok=True, reason="won", winner=None, finalists=[], ranked=[], rule_killed=[], diagnostics=diag,
                    space=sp, rules=r, points_ref=F.PointsReference(), relative_scoring=True, warnings=[])
        base.update(kw)
        return F.FullCarResult(**base)
    w = _S(112, 5, "four_tv", 3.9, tot=321.4, margin=0.4567, thermal=Th())
    w.points = {"accel": 70.1, "skidpad": 60.4, "autocross": 100.5}
    w.tv_yaw_note = "torque vectoring: note"
    hotw = _S(104, 5, "twin_axle", 4.1, tot=300.0, thermal=Th(), overheat=12, verdict="THERMAL_DNF", kill=["hot"])
    badth = _S(100, 5, "single_diff", 4.0, thermal=Th(ok=False))
    ranked = [w, hotw, badth, _S(96, 4, margin=math.nan, kill=["k" * 60], verdict="ENERGY_SHORT"),
              _S(92, 4, verdict="ODD", margin=math.inf)] + [_S(80 + i, 3) for i in range(10)]
    killed = [_S(150 + i, 5, ok=False, verdict="RULE_KILLED", kill=[f"rule {i}"]) for i in range(8)]
    yield "md.full", F.render_fullcar_md(R(winner=w, ranked=ranked, rule_killed=killed, warnings=["a", "b"]),
                                         include_exports=True)
    yield "md.abs", F.render_fullcar_md(R(winner=hotw, ranked=[hotw], relative_scoring=False, ok=False),
                                        include_exports=True)
    yield "md.nothermal", F.render_fullcar_md(R(winner=badth, rule_killed=killed[:6]))
    yield "md.noexport_flag", F.render_fullcar_md(R(winner=w))
    yield "md.empty", F.render_fullcar_md(R(ok=False, reason="none"), include_exports=True)
    yield "md.nothermal_none", F.render_fullcar_md(R(winner=_S(tot=10.0)))
    yield "md.odd", F.render_fullcar_md(R(winner=_S(tot=10.0, verdict="ODD")))


def test_exports_and_report():
    check_lazy("inverse_genesis_fullcar.export", _export_cases(), rel=REL, abs_=ABS)
