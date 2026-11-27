"""Characterization suite for suspension/trade.py (mutation gate); see tests/_golden.py. A few paired draws (a real
study uses hundreds) through every verdict: not separable, separable and negative, a winner with and without a
price or fabrication hours, a winner tied with a cheaper option and one tied with a dearer one; the nominal run with
and without a draw (a spool's infinite bias ratio included), the field reference and scoring with missing and zero
times, the bands, best(), and the Cost-event screens with and without the reference costs."""
import math

from suspension import driveline as D
from suspension import trade as T
from suspension.dynamics import VehicleParams
from suspension import lapsim
from tests._golden import plain
from tests._golden_lazy import check_lazy

UNC = T.Uncertainty(n_draws=5, seed=7)


def _v(v):
    return {"baseline": v.baseline, "results": plain(v.results), "decidable": v.decidable,
            "resolution": v.resolution_points, "text": v.verdict_text,
            "pairwise": {f"{a}|{b}": plain(x) for (a, b), x in v.pairwise.items()}, "cost_event": v.cost_event,
            "provenance": v.provenance, "best": None if v.best() is None else v.best().label}


def _cases():
    open_ = T.Option("open", D.catalog("open", mass_kg=3.4, cost_usd=400.0))
    same = T.Option("same", D.catalog("open", mass_kg=3.4, cost_usd=400.0))
    lsd = T.Option("lsd", D.DifferentialSpec("lsd", tbr=3.0, preload_nm=20.0, mass_kg=4.0, cost_usd=900.0,
                                             extra_fab_hours=6.0))
    lsd2 = T.Option("lsd2", D.DifferentialSpec("lsd2", tbr=3.1, preload_nm=21.0, mass_kg=4.0, cost_usd=600.0))
    lsd3 = T.Option("lsd3", D.DifferentialSpec("lsd3", tbr=3.0, preload_nm=20.0, mass_kg=4.0, cost_usd=1500.0))
    free = T.Option("free", D.DifferentialSpec("free", tbr=3.0, preload_nm=20.0, mass_kg=4.0))
    heavy = T.Option("heavy", D.catalog("open", mass_kg=3.4, cost_usd=400.0), mass_offset_kg=60.0, notes="ballast")
    spool = T.Option("spool", D.DifferentialSpec("spool", kind=D.SPOOL, tbr=math.inf, mass_kg=2.0, cost_usd=150.0))
    yield "option", [open_.acquisition_usd(), lsd.fab_hours(), free.acquisition_usd(), plain(heavy)]
    import numpy as np
    yield "sample", [T.Uncertainty().sample(np.random.default_rng(1)), plain(T.Uncertainty())]
    base = VehicleParams()
    sim = lapsim.LapSimParams(mass=base.mass)
    cond = D.ExitCondition(cl_a=sim.cl_a)
    for name, opt in (("open", open_), ("lsd", lsd), ("spool", spool), ("heavy", heavy)):
        yield f"eval.{name}", [plain(T.evaluate_option(opt, base, sim, cond)),
                               plain(T.evaluate_option(opt, base, sim, cond, dict(mu_scale=0.9, tbr_rel=1.2,
                                                                                  preload_rel=0.5, lock_k=0.3,
                                                                                  exit_lateral_frac=0.6)))]
    yield "eval.nofrac", plain(T.evaluate_option(lsd, base, sim, D.ExitCondition(lateral_g_frac=None, cl_a=sim.cl_a)))
    yield "ref", [T._field_reference([{"a": 10.0, "b": None}, {"a": 9.0, "b": None, "c": 4.0}]), T._field_reference([])]
    yield "score", [T._score_against({"a": 10.0, "b": None, "c": 5.0}, {"a": 9.0, "c": 0.0}),
                    T._score_against({"a": 9.0}, {"a": 9.0})]
    for name, opts, bl, kw in (("tie", [same], open_, {}), ("winner", [lsd], open_, {}),
                               ("negative", [heavy], open_, {}), ("free", [free], open_, {}),
                               ("ties", [lsd, lsd2, lsd3], open_, {}), ("floor", [lsd], open_,
                                                                         {"practical_floor_points": 1000.0}),
                               ("spool", [spool, lsd], open_, {"endurance_laps": 2})):
        yield f"compare.{name}", _v(T.compare(opts, bl, unc=UNC, **kw))
    yield "compare.defaults", _v(T.compare([lsd], open_, base_params=VehicleParams(mass=280.0),
                                           unc=T.Uncertainty(n_draws=3)))
    yield "best.none", T.TradeVerdict("b", [], False, 0.0, "").best()
    r = [T.OptionResult("a", 3.0, 1.0, 4.0, True, None, 0.0, None, actionable=True),
         T.OptionResult("b", 9.0, 1.0, 12.0, True, None, 0.0, None, actionable=True),
         T.OptionResult("c", 20.0, 1.0, 30.0, True, None, 0.0, None, actionable=False),
         T.OptionResult("d", None, None, None, False, None, 0.0, None, actionable=True),
         T.OptionResult("e", -5.0, -6.0, -4.0, True, None, 0.0, None, actionable=True)]
    yield "best", T.TradeVerdict("b", r, True, 0.0, "").best().label
    yield "cost", [T.cost_event_points(c, lo, hi, p) for c, lo, hi, p in (
        (30000.0, 10000.0, 50000.0, 100.0), (5000.0, 10000.0, 50000.0, 100.0), (60000.0, 10000.0, 50000.0, 100.0),
        (30000.0, None, 50000.0, 100.0), (30000.0, 10000.0, None, 100.0), (math.nan, 1.0, 2.0, 100.0),
        (1.0, 2.0, 2.0, 100.0), (1.0, 3.0, 2.0, 100.0), (20000.0, 10000.0, 50000.0, 50.0))] + [
        T.cost_event_points(30000.0, 10000.0, 50000.0)]
    yield "cost_delta", [T.cost_event_delta(30000.0, 500.0, 10000.0, 50000.0),
                         T.cost_event_delta(30000.0, -500.0, 10000.0, 50000.0, pts_max=80.0),
                         T.cost_event_delta(30000.0, 500.0), T.cost_event_delta(30000.0, 500.0, 10000.0, None)]


def test_every_trade_verdict():
    check_lazy("trade", _cases())


class _R:
    def __init__(self, **kw):
        self.__dict__.update(kw)

    def __repr__(self):
        return "R"


def _fake_cases(mp):
    seen = []

    def sim(veh, params, endurance_laps):
        seen.append([veh, params, endurance_laps])
        return {"a": _R(ok=True, event_time=5.0), "b": _R(ok=False, event_time=5.0), "c": _R(ok=True, event_time=None),
                "d": _R(ok=True, event_time=float("nan")), "e": _R(ok=True, event_time=0.0),
                "f": _R(ok=True, event_time=0.5), "g": _R(event_time=4.0), "h": _R(ok=True),
                "i": _R(ok=True, event_time=float("inf")), "j": _R(ok=True, event_time=-2.0)}
    mp.setattr(T.lapsim, "simulate_events", sim)
    yield "times", [T._event_times("veh", "lp", 3), seen[:]]
    mp.undo()
    pts = T._score_against({"skidpad": 5.0, "accel": None, "autocross": 50.0, "endurance": 1000.0, "x": 3.0},
                           {"skidpad": 4.9, "accel": 3.0, "autocross": 0.5, "endurance": -1.0, "x": 0.0})
    yield "score", pts
    yield "score.noref", T._score_against({"skidpad": 5.0}, {})
    yield "cost_delta.inf", T.cost_event_delta(30000.0, math.inf, 10000.0, 50000.0)
    log = []

    class VD:
        def __init__(self, p, front_kin=None, rear_kin=None, tire=None):
            log.append(["vd", p.mass, round(p.mu_peak, 9), front_kin, rear_kin, tire])
            self.p = p

    def trac(veh, spec, c):
        log.append(["trac", spec.tbr, spec.preload_nm, spec.mass_kg, c.lateral_g, c.lateral_g_frac, c.cl_a])
        return _R(drive_grip_frac=0.8, lock_ratio=0.4, t_total_nm=900.0, fz_inside_n=1000.0, fz_outside_n=1500.0,
                  lateral_util=0.7, notes=["n1"])

    def derate(p, lock, k):
        log.append(["derate", lock, k])
        from dataclasses import replace
        return replace(p, mass=p.mass + 1.0)

    def times(veh, lp, laps):
        log.append(["times", lp.mass, lp.drive_grip_frac, laps])
        return {"skidpad": 5.0}
    mp.setattr(T, "VehicleDynamics", VD)
    mp.setattr(T, "axle_traction", trac)
    mp.setattr(T, "apply_lock_derate", derate)
    mp.setattr(T, "_event_times", times)
    base = VehicleParams()
    sim = lapsim.LapSimParams(mass=base.mass, cl_a=3.3)
    opt = T.Option("lsd", D.DifferentialSpec("lsd", tbr=3.0, preload_nm=20.0, mass_kg=4.0, cost_usd=900.0),
                   mass_offset_kg=2.5)
    nomass = T.Option("nm", D.DifferentialSpec("nm", tbr=math.inf, preload_nm=10.0, mass_kg=None))
    for name, o, cond, draw in (
            ("default", opt, D.ExitCondition(cl_a=1.0, lateral_g_frac=0.55, lateral_g=1.2), None),
            ("nofrac", opt, D.ExitCondition(cl_a=1.0, lateral_g_frac=None), None),
            ("draw", opt, D.ExitCondition(cl_a=1.0), dict(mu_scale=0.9, tbr_rel=1.2, preload_rel=0.5, lock_k=0.3,
                                                         exit_lateral_frac=0.6)),
            ("nomass", nomass, D.ExitCondition(cl_a=1.0), None)):
        r = T.evaluate_option(o, base, sim, cond, draw, front_kin="FK", rear_kin="RK", tire="TY", endurance_laps=4)
        yield f"eval.{name}", [plain(r), plain(log)]
        log.clear()
    mp.undo()
    calls = []

    def fake_eval(o, base_params, sim_params, cond, d, front_kin, rear_kin, tire, endurance_laps):
        calls.append([o.label, d is None, base_params.mass, sim_params.mass, round(sim_params.cl_a, 9),
                      cond.cl_a, front_kin, rear_kin, tire, endurance_laps])
        t = plan[o.label]
        k = len([c for c in calls if c[0] == o.label])
        v = t(k) if callable(t) else t
        return dict(times={"skidpad": v, "accel": None}, notes=[f"{o.label}-note", "shared"])
    mp.setattr(T, "evaluate_option", fake_eval)
    S = D.DifferentialSpec
    base_o = T.Option("base", S("base", tbr=1.0, mass_kg=3.0, cost_usd=300.0))
    a = T.Option("a", S("a", tbr=3.0, mass_kg=3.0, cost_usd=900.0, extra_fab_hours=4.0))
    b = T.Option("b", S("b", tbr=3.0, mass_kg=3.0, cost_usd=500.0))
    c = T.Option("c", S("c", tbr=3.0, mass_kg=3.0))
    dd = T.Option("d", S("d", tbr=3.0, mass_kg=3.0, cost_usd=2000.0))
    e = T.Option("e", S("e", tbr=3.0, mass_kg=3.0, cost_usd=900.0))
    f = T.Option("f", S("f", tbr=3.0, mass_kg=3.0, cost_usd=400.0))
    unc = T.Uncertainty(n_draws=4, seed=3)
    for name, opts, pl, kw in (
            ("win", [a, b], {"base": 5.0, "a": 4.6, "b": lambda k: 4.6 + (0.02 if k % 2 else -0.02)}, {}),
            ("win_cheaper", [a, b, c, dd], {"base": 5.0, "a": 4.6, "b": 4.61, "c": 4.6, "d": 4.6}, {}),
            ("win_cheaper2", [a, b, c, dd, e, f], {"base": 5.0, "a": 4.6, "b": 4.61, "c": 4.6, "d": 4.6, "e": 4.6,
                                                   "f": 4.6}, {}),
            ("small", [a], {"base": 5.0, "a": 4.9995}, {"practical_floor_points": 0.0}),
            ("pair_floor", [a, b], {"base": 5.0, "a": 4.6, "b": 4.5}, {"practical_floor_points": "pair"}),
            ("negative", [a, b], {"base": 4.6, "a": 5.0, "b": 4.9}, {}),
            ("negative_mixed", [a, b], {"base": 4.6, "a": 5.0, "b": lambda k: 4.6 + (0.1 if k % 2 else -0.1)}, {}),
            ("noise", [a], {"base": 5.0, "a": lambda k: 5.0 + (0.1 if k % 2 else -0.1)}, {}),
            ("floor_exact", [a], {"base": 5.0, "a": 4.6}, {"practical_floor_points": None}),
            ("nan", [a, b], {"base": 5.0, "a": float("nan"), "b": 4.6}, {}),
            ("zero_draws", [a, b], {"base": 5.0, "a": 4.6, "b": 4.5}, {"unc": T.Uncertainty(n_draws=0)}),
            ("one_draw", [a, b], {"base": 5.0, "a": 4.6, "b": 4.5}, {"unc": T.Uncertainty(n_draws=1)}),
            ("args", [b], {"base": 5.0, "b": 4.5}, dict(base_params=VehicleParams(mass=270.0), sim_params=sim,
                                                        cond=D.ExitCondition(cl_a=2.2), front_kin="FK", rear_kin="RK",
                                                        tire="TY", endurance_laps=3)),
            ("defaults", [b], {"base": 5.0, "b": 4.5}, dict(base_params=VehicleParams(mass=270.0)))):
        plan = pl
        if kw.get("practical_floor_points") == "pair":
            probe = T.compare(opts, base_o, unc=unc)
            kw = {"practical_floor_points": abs(probe.pairwise[("a", "b")]["median"])}
            calls.clear()
        if kw.get("practical_floor_points", 0) is None:
            probe = T.compare(opts, base_o, unc=unc)
            kw = {"practical_floor_points": abs(probe.results[0].delta_points_median)}
        kw.setdefault("unc", unc)
        v = T.compare(opts, base_o, **kw)
        yield f"compare.{name}", [_v(v), calls[:]]
        calls.clear()
    plan = {"base": 5.0, "a": 4.6}
    v = T.compare([a], base_o)
    yield "compare.default_unc", [len(calls), v.results[0].delta_points_median]
    calls.clear()


def test_trade_with_stand_ins(monkeypatch):
    check_lazy("trade.fakes", _fake_cases(monkeypatch))
