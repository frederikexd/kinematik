"""Characterization suite for suspension/kinematik_stochastic.py (mutation gate); see tests/_golden.py. The
metric vector (and its failure modes, injected through a stand-in solver), tolerance specs, clusters and fields
(every preset, pull axis, stated accuracy, shifts, sampling in both distributions, moments and bounds), the
sensitivity matrix, sweeps in both modes against every verdict, the robust nudge (centred, biased, clamped,
verified), alignment prescriptions (restored, partial, unshimmable, bad inputs) and both Markdown reports."""
import numpy as np
import pytest

from suspension import kinematik_stochastic as ks
from suspension.kinematics import Hardpoints
from tests._golden import plain
from tests._golden_lazy import check_lazy


def _try(fn, *a, **k):
    try:
        return plain(fn(*a, **k))
    except (ValueError, TypeError, KeyError) as e:
        return {"raised": type(e).__name__, "message": str(e)}


def _a(x, n=9):
    return plain(np.round(np.asarray(x, float), n))


def _res(r):
    return {"ok": r.ok, "mode": r.mode, "n": r.n, "seed": r.seed, "yield": _a(r.yield_frac), "verdict": r.verdict,
            "ffm": _a(r.fail_frac_per_metric), "sff": _a(r.solver_fail_frac), "bias": _a(r.bias, 7),
            "p05": _a(r.p05, 7), "p95": _a(r.p95, 7), "attr": _a(r.attribution, 6), "bands": _a(r.bands),
            "va": None if r.verify_agreement is None else _a(r.verify_agreement),
            "vw": None if r.verify_worst_err is None else _a(r.verify_worst_err, 6), "warn": r.warnings,
            "worst": [r.worst_metric()[0], _a(r.worst_metric()[1])],
            "dom": [[r.dominant_coord(m)[0], _a(r.dominant_coord(m)[1], 6)] for m in ks.METRICS]}


def _nudge(z):
    return {"ok": z.ok, "reason": z.reason, "shifts": {k: _a(v, 6) for k, v in sorted(z.shifts.items())},
            "pred": _a(z.predicted_yield), "base": _a(z.baseline_yield),
            "ver": None if z.verified_yield is None else _a(z.verified_yield), "clamped": z.clamped}


def _rx(r):
    return {"ok": r.ok, "verdict": r.verdict, "moves": _a(r.moves_mm, 6), "adj": [a.name() for a in r.adjusters],
            "before": _a(r.delta_before, 6), "after": _a(r.delta_after, 6), "bands": _a(r.bands),
            "unreach": r.unreachable, "warn": r.warnings, "lines": r.lines()}


def _ab(hp, d):
    return {k: np.asarray(getattr(hp, k), float) + np.asarray(v, float) for k, v in d.items()}


def _fld(f):
    lo, hi = f.bounds_vec()
    return {"coords": f.coords(), "prov": f.provenance, "cal": f.calibrated, "mean": _a(f.mean_vec()),
            "var": _a(f.var_vec()), "lo": _a(lo), "hi": _a(hi), "n_cl": len(f.clusters),
            "specs": {p: [_a(s.lo), _a(s.hi), s.dist] for p, s in sorted(f.specs.items())}}


class _State:
    def __init__(self, travel, converged=True, nan=False):
        self.travel, self.converged = travel, converged
        self.camber = np.nan if nan else -1.0 - travel / 100
        self.toe, self.scrub_radius, self.caster = travel / 200, 9.0, 5.0
        self.contact_patch = np.array([0.0, 600.0, 0.0])
        self.instant_center = np.array([-1500.0, 40.0 + travel])


def _fake_kin(kind):
    class K:
        def __init__(self, hp):
            if kind == "raise":
                raise RuntimeError("no")

        def sweep(self, travel_min, travel_max, n):
            if kind == "empty":
                return []
            tr = [-10.0, 4e-7, 20.0, 0.3] if kind == "skew" else np.linspace(travel_min, travel_max, n)
            out = [_State(float(t), converged=not (kind == "unconv" and i == 1), nan=kind == "nan")
                   for i, t in enumerate(tr)]
            if kind == "noattr":
                for st in out:
                    del st.converged
            return out
    return K


@pytest.fixture(scope="module")
def hp():
    return Hardpoints.default()


def _cases(hp, monkeypatch):
    yield "consts", [list(ks.METRICS), ks._METRIC_LABELS, list(ks.PERTURBABLE_POINTS), list(ks._AXES)]
    yield "metrics", [_a(ks.metrics_of(hp)[0]), ks.metrics_of(hp)[1], _a(ks.metrics_of(hp, 10.0, 3)[0]),
                      _a(ks.metrics_of(hp, n_travel=4)[0]), _a(ks.metrics_of(hp, 40.0, 7)[0])]
    real = ks.SuspensionKinematics
    for kind in ("raise", "empty", "unconv", "nan", "ok", "noattr", "skew"):
        monkeypatch.setattr(ks, "SuspensionKinematics", _fake_kin(kind))
        yield f"metrics.fake.{kind}", [plain(ks.metrics_of(hp)), plain(ks.metrics_of(hp, n_travel=4))]
    monkeypatch.setattr(ks, "SuspensionKinematics", real)
    p = ks._perturbed(hp, {"upper_outer": [1.0, -2.0, 3.0], "no_such_point": [1, 1, 1]})
    yield "perturbed", [_a(p.upper_outer), _a(hp.upper_outer), _a(p.lower_outer), hasattr(p, "no_such_point")]

    class NoCopy:
        def __init__(self, **kw):
            self.__dict__.update(kw)
    nc = NoCopy(upper_outer=np.array([1.0, 2.0, 3.0]), name="x")
    monkeypatch.setattr(ks, "Hardpoints", NoCopy)
    q = ks._perturbed(nc, {"upper_outer": np.array([1.0, 1.0, 1.0])})
    monkeypatch.setattr(ks, "Hardpoints", Hardpoints)
    yield "perturbed.nocopy", [_a(q.upper_outer), _a(nc.upper_outer), q.name, q is nc]
    S = ks.ToleranceSpec
    yield "spec", [[_a(s.lo), _a(s.hi), s.dist, _a(s.mean), _a(s.var)] for s in (
        S([-0.3, -0.3, -0.3], [1.5, 0.3, 0.3]), S([-1, 0, 0], [1, 0, 2], "normal"), S.symmetric(-2.0),
        S.symmetric(0.5, "normal"), S([0, 0, 0], [0, 0, 0]))]
    yield "spec.err", [_try(S, [0, 0, 0], [-1, 0, 0]), _try(S, [0, 0, 0], [1, 1, 1], "tri"),
                       _try(S, [0, 0, 0], [0, 0, 0], "normal")]
    yield "phi", [[_a(ks._phi(x)), _a(ks._Phi(x))] for x in (-3.0, -1.0, 0.0, 0.5, 2.0)]
    C = ks.ToleranceCluster
    yield "cluster.err", [_try(C, ["upper_outer", "wheel_center"], S.symmetric(1)), _try(C, [], S.symmetric(1)),
                          plain(C(["upper_outer"], S.symmetric(1)).points), C(("lower_outer",), S.symmetric(1)).label]
    F = ks.ToleranceField
    yield "field.err", [_try(F, {"wheel_center": S.symmetric(1)}), _try(F, {}, clusters=["x"])]
    presets = {}
    for shop in ("hand_weld", "jig_weld", "cnc"):
        for kw in ({}, {"weld_pull_mm": 1.0}, {"weld_pull_mm": -0.5, "pull_axis": "z"},
                   {"weld_pull_mm": 0.7, "pull_axis": "y", "tab_accuracy_mm": 2.0}, {"tab_accuracy_mm": 0.3},
                   {"tab_accuracy_mm": 0.0}, {"tab_accuracy_mm": -1.0}, {"tab_accuracy_mm": 0.05}):
            k = f"{shop}.{'.'.join(f'{a}={b}' for a, b in kw.items())}"
            f = presets[k] = F.preset(shop, **kw)
            yield f"preset.{k}", _fld(f)
    yield "preset.default", _fld(F.preset())
    yield "preset.err", [_try(F.preset, "laser"), _try(F.preset, "hand_weld", 1.0, "w")]
    cl = F({"upper_outer": S([-0.2, -0.2, -0.2], [0.6, 0.2, 0.2], "normal")},
           provenance="CMM", calibrated=True,
           clusters=[C(("upper_front_inner", "upper_rear_inner"), S([-1, -0.5, 0], [1, 0.5, 0.5]), "jig"),
                     C(("upper_outer",), S([0, 0, 0], [0.4, 0.4, 0], "normal"))])
    yield "field.cluster", _fld(cl)
    yield "field.empty", _fld(F())
    yield "field.sample", [_a(cl.sample(4, seed=3), 7), _a(presets["hand_weld."].sample(3), 7),
                           _a(F().sample(2)), _a(F({"lower_outer": S([0, 0, 0], [0, 0, 0], "normal")}).sample(2)),
                           _a(F({"lower_outer": S([0, -1, 0], [0, 1, 0], "normal")}).sample(3, 9), 7)]
    sh = cl.shifted({"upper_outer": [1, 0, 0], "tie_rod_inner": [0, 0.5, 0]})
    yield "field.shifted", [_fld(sh), _fld(cl.shifted({}, provenance="first article")),
                            _fld(presets["hand_weld."].shifted({"lower_outer": np.array([0.0, 0.0, -1.0])}))]
    yield "yspec", [_a(ks.YieldSpec().bands()), _a(ks.YieldSpec(camber_bump_deg=-0.5, caster_deg=1).bands())]
    yield "thresholds", plain(ks.StochasticThresholds().__dict__)
    hw = presets["hand_weld.weld_pull_mm=1.0"]
    sens = ks.sensitivity(hp, hw)
    yield "sens", [sens.coords, _a(sens.J, 7), _a(sens.nominal), sens.step_mm, sens.coord_labels()]
    small = F({"tie_rod_outer": S([-0.5, 0, 0], [1.0, 0.2, 0.3])})
    s2 = ks.sensitivity(hp, small, step_mm=0.5)
    yield "sens.small", [s2.coords, _a(s2.J, 7), s2.step_mm]
    monkeypatch.setattr(ks, "SuspensionKinematics", _fake_kin("raise"))
    yield "sens.fail", _try(ks.sensitivity, hp, small)
    monkeypatch.setattr(ks, "SuspensionKinematics", real)
    yield "sweep.err", [_try(ks.stochastic_sweep, hp, F()), _try(ks.stochastic_sweep, hp, small, mode="mc", sens=s2)]
    Y, T = ks.YieldSpec, ks.StochasticThresholds
    for name, kw in (("lin", dict(n=300, n_verify=6)), ("lin_tight", dict(n=200, n_verify=4, yspec=Y(0.02, 0.01))),
                     ("lin_marg", dict(n=200, n_verify=0, thresholds=T(robust_yield=1.01, marginal_yield=0.5))),
                     ("lin_frag", dict(n=100, n_verify=3, thresholds=T(robust_yield=1.01, marginal_yield=1.01))),
                     ("lin_seed", dict(n=150, seed=5, n_verify=2, thresholds=T(verify_agreement=1.01))),
                     ("lin_target", dict(n=100, n_verify=0, target=sens.nominal + 0.1)),
                     ("full", dict(n=8, mode="full")), ("full_tight", dict(n=5, mode="full", yspec=Y(0.01, 0.01))),
                     ("lin_zero", dict(n=0, n_verify=0)), ("full_zero", dict(n=0, mode="full")),
                     ("lin_exact", dict(n=40, n_verify=0, thresholds=T(robust_yield=0.0))),
                     ("lin_nv_big", dict(n=3, n_verify=10))):
        yield f"sweep.{name}", _res(ks.stochastic_sweep(hp, hw, sens=sens, **kw))
    yield "sweep.cluster", _res(ks.stochastic_sweep(hp, cl, n=100, n_verify=2))
    yield "sweep.own_sens", _res(ks.stochastic_sweep(hp, small, n=50, n_verify=1))
    calls = {"n": 0}

    class Flaky(real):
        def __init__(self, h):
            calls["n"] += 1
            if calls["n"] % 3 == 0:
                raise RuntimeError("flaky")
            super().__init__(h)
    monkeypatch.setattr(ks, "SuspensionKinematics", Flaky)
    yield "sweep.flaky_full", _res(ks.stochastic_sweep(hp, hw, n=9, mode="full", sens=sens))
    calls["n"] = 0
    yield "sweep.flaky_lin", _res(ks.stochastic_sweep(hp, hw, n=60, n_verify=6, sens=sens,
                                                      thresholds=T(solver_fail_frac=0.0)))
    calls["n"] = 0
    yield "sweep.flaky_sf", _res(ks.stochastic_sweep(hp, hw, n=6, mode="full", sens=sens,
                                                     thresholds=T(solver_fail_frac=0.5)))
    monkeypatch.setattr(ks, "SuspensionKinematics", _fake_kin("raise"))
    yield "sweep.allfail", _res(ks.stochastic_sweep(hp, hw, n=4, mode="full", sens=sens))
    yield "sweep.allfail_v", _res(ks.stochastic_sweep(hp, hw, n=20, n_verify=3, sens=sens))
    monkeypatch.setattr(ks, "SuspensionKinematics", real)
    base = ks.stochastic_sweep(hp, hw, n=300, n_verify=0, sens=sens)
    yield "nudge.small", _nudge(ks.robust_nudge(hp, hw, base))
    hwb = hw.shifted({p: [2.0, -1.0, 1.5] for p in ks.PERTURBABLE_POINTS[:6]})
    big = ks.stochastic_sweep(hp, hwb, n=300, n_verify=0, yspec=Y(0.1, 0.05, 4.0, 2.0, 0.1))
    yield "sweep.big", _res(big)
    yield "nudge.biased", _nudge(ks.robust_nudge(hp, hwb, big))
    yield "nudge.clamp", _nudge(ks.robust_nudge(hp, hwb, big, freedom_mm=0.05))
    yield "nudge.dict", _nudge(ks.robust_nudge(hp, hwb, big, freedom_mm={
        p: (0.0 if p == "upper_outer" else 10.0) for p in hwb.specs}))
    yield "nudge.dict_missing", _try(ks.robust_nudge, hp, hwb, big, freedom_mm={"upper_outer": 1.0})
    yield "nudge.verify", _nudge(ks.robust_nudge(hp, hwb, big, n_verify_full=4, seed=2))
    yield "md.big", ks.render_stochastic_md(big, ks.robust_nudge(hp, hwb, big, freedom_mm=0.05), title="RL")
    flat = presets["cnc."]
    rflat = ks.stochastic_sweep(hp, flat, n=50, n_verify=0)
    yield "nudge.centred", _nudge(ks.robust_nudge(hp, flat, rflat))
    tiny_b = ks.stochastic_sweep(hp, hw, n=50, n_verify=0, sens=sens, yspec=Y(0.02, 0.01, 0.5, 0.3, 0.02))
    yield "nudge.tight", _nudge(ks.robust_nudge(hp, hw, tiny_b, n_verify_full=2))
    A = ks.Adjuster
    yield "adjuster", [[_a(a.direction()), a.name()] for a in (
        A("tie_rod_inner"), A("upper_outer", "x", label="camber shims"), A("lower_outer", [0, 3, 4]),
        A("tie_rod_outer", axis="y", step=0.0))] + [_try(A("upper_outer", [0, 0, 0]).direction)]
    adj = [A("tie_rod_inner", "z", -3, 3, 0.25, "toe link"), A("upper_outer", "y", -2, 2, 0.5, "camber shims"),
           A("lower_outer", [1, 0, 0], -1, 1, 0.0)]
    for name, built, ad, ys in (
            ("restore", {"upper_outer": [0, 0.8, 0], "tie_rod_inner": [0, 0, 0.4]}, adj, None),
            ("none", {}, adj, None),
            ("partial", {"upper_outer": [0, 4.0, 0], "tie_rod_inner": [0, 0, 3]}, adj[:1],
             Y(0.01, 0.01, 0.5, 0.2, 0.01)),
            ("unshim", {"lower_front_inner": [0, 0, 6.0]}, [A("tie_rod_inner", "z", -1, 1, 0.5)],
             Y(0.05, 0.5, 0.5, 50, 5)),
            ("clamped", {"upper_outer": [0, 3.0, 0]}, [A("upper_outer", "y", -0.5, 0.5, 0.0)], None)):
        yield f"rx.{name}", _try(lambda: _rx(ks.alignment_prescription(hp, _ab(hp, built), ad, ys)))
    yield "rx.err", [_try(ks.alignment_prescription, hp, {"nope": [0, 0, 0]}, adj),
                     _try(ks.alignment_prescription, hp, {}, []),
                     _try(ks.alignment_prescription, hp, _ab(hp, {"upper_outer": [0, 25.01, 0]}), adj),
                     _try(lambda: _rx(ks.alignment_prescription(hp, _ab(hp, {"upper_outer": [0, 25.0, 0]}), adj[:1])))]
    monkeypatch.setattr(ks, "SuspensionKinematics", _fake_kin("raise"))
    yield "rx.nominal_fail", _try(ks.alignment_prescription, hp, {}, adj)
    monkeypatch.setattr(ks, "SuspensionKinematics", real)
    calls["n"] = 0

    class Second(real):
        def __init__(self, h):
            calls["n"] += 1
            if calls["n"] >= 2:
                raise RuntimeError("as-built")
            super().__init__(h)
    monkeypatch.setattr(ks, "SuspensionKinematics", Second)
    yield "rx.asbuilt_fail", _try(ks.alignment_prescription, hp, _ab(hp, {"upper_outer": [0, 1.0, 0]}), adj)
    for stop in (3, 4, 9):
        calls["n"] = 0

        class Later(real):
            def __init__(self, h, stop=stop):
                calls["n"] += 1
                if calls["n"] == stop or (stop == 9 and calls["n"] >= stop):
                    raise RuntimeError("late")
                super().__init__(h)
        monkeypatch.setattr(ks, "SuspensionKinematics", Later)
        yield f"rx.late{stop}", _try(lambda: _rx(ks.alignment_prescription(hp, _ab(hp, {"upper_outer": [0, 1.0, 0]}), adj)))
    monkeypatch.setattr(ks, "SuspensionKinematics", real)
    rx = ks.alignment_prescription(hp, _ab(hp, {"upper_outer": [0, 0.8, 0]}), adj)
    for name, built, ad, ys in (
            ("partial", {"upper_outer": [0, 3.0, 0]}, [A("upper_outer", "y", -0.5, 0.5, 0.0)],
             Y(0.01, 0.01, 0.5, 0.2, 0.01)),
            ("hi_clamp", {"upper_outer": [0, -3.0, 0]}, [A("upper_outer", "y", -0.5, 0.5, 0.0)], None),
            ("same_point", {"upper_outer": [0, 0.8, 0.4]}, [A("upper_outer", "y", -2, 2, 0.0),
                                                           A("upper_outer", "z", -2, 2, 0.0)], None),
            ("zero_band", {"upper_outer": [0, 0.8, 0], "tie_rod_inner": [0, 0, 0.4]}, adj,
             Y(0.0, 0.01, 0.5, 0.2, 0.01)),
            ("default_step", {"upper_outer": [0, 0.8, 0]}, [A("upper_outer", "y", -2, 2)], None)):
        res_rx = ks.alignment_prescription(hp, _ab(hp, built), ad, ys)
        yield f"rx2.{name}", [_rx(res_rx), ks.render_prescription_md(res_rx)]
    for name, built, ad, ys in (
            ("partial", {"upper_outer": [0, 4.0, 0], "tie_rod_inner": [0, 0, 3]}, adj[:1],
             Y(0.01, 0.01, 0.5, 0.2, 0.01)),
            ("unshim", {"lower_front_inner": [0, 0, 6.0]}, [A("tie_rod_inner", "z", -1, 1, 0.5)],
             Y(0.05, 0.5, 0.5, 50, 5))):
        yield f"md.rx.{name}", ks.render_prescription_md(ks.alignment_prescription(hp, _ab(hp, built), ad, ys))
    calls["n"] = 0

    class LateMd(real):
        def __init__(self, h):
            calls["n"] += 1
            if calls["n"] >= 9:
                raise RuntimeError("late")
            super().__init__(h)
    monkeypatch.setattr(ks, "SuspensionKinematics", LateMd)
    yield "md.rx.warn", ks.render_prescription_md(ks.alignment_prescription(hp, _ab(hp, {"upper_outer": [0, 1.0, 0]}),
                                                                            adj))
    monkeypatch.setattr(ks, "SuspensionKinematics", real)
    yield "md.rx", [ks.render_prescription_md(rx), ks.render_prescription_md(
        ks.alignment_prescription(hp, {}, adj), title="FL")]
    nud = ks.robust_nudge(hp, hw, base)
    yield "md.sweep", [ks.render_stochastic_md(base), ks.render_stochastic_md(base, nud, title="FR"),
                       ks.render_stochastic_md(ks.stochastic_sweep(hp, hw, n=50, n_verify=3, sens=sens,
                                                                   yspec=Y(0.02, 0.01)),
                                               ks.robust_nudge(hp, flat, rflat)),
                       ks.render_stochastic_md(ks.stochastic_sweep(hp, cl, n=20, mode="full"))]


def _more(hp, monkeypatch):
    S, F, C, Y, T = ks.ToleranceSpec, ks.ToleranceField, ks.ToleranceCluster, ks.YieldSpec, ks.StochasticThresholds
    two = F({"upper_outer": S.symmetric(0.5)}, clusters=[C(("upper_outer",), S([0, 0, 0], [0.4, 0.4, 0], "normal")),
                                                         C(("upper_outer",), S([0, 0, 0], [0.2, 0, 0.6], "normal"))])
    yield "field.two_clusters", [_a(two.mean_vec()), _a(two.var_vec())]
    off = F({"upper_outer": S.symmetric(0.5)},
            clusters=[C(("upper_outer",), S([-0.2, -0.2, 0], [0.4, 0.4, 0.2], "normal"))])
    yield "field.cluster_offset", _a(off.sample(4, seed=1), 7)
    hw = F.preset("hand_weld", weld_pull_mm=1.0)
    sens = ks.sensitivity(hp, hw)
    real_m = ks.metrics_of
    cnt = {"n": 0}

    def flaky_m(h, *a, **k):
        cnt["n"] += 1
        return (np.full(len(ks.METRICS), np.nan), False) if cnt["n"] == 3 else real_m(h, *a, **k)
    monkeypatch.setattr(ks, "metrics_of", flaky_m)
    yield "sens.half_fail", _try(ks.sensitivity, hp, hw)
    monkeypatch.setattr(ks, "metrics_of", real_m)
    yield "sweep.agree_edge", _res(ks.stochastic_sweep(hp, hw, n=150, seed=5, n_verify=2, sens=sens,
                                                       thresholds=T(verify_agreement=1.0)))
    r0 = ks.stochastic_sweep(hp, hw, n=40, n_verify=0, sens=sens, yspec=Y(0.05, 0.02))
    yield "sweep.robust_edge", _res(ks.stochastic_sweep(hp, hw, n=40, n_verify=0, sens=sens, yspec=Y(0.05, 0.02),
                                                        thresholds=T(robust_yield=r0.yield_frac)))
    yield "sweep.marginal_edge", _res(ks.stochastic_sweep(
        hp, hw, n=40, n_verify=0, sens=sens, yspec=Y(0.05, 0.02),
        thresholds=T(robust_yield=1.01, marginal_yield=r0.yield_frac)))
    calls = {"n": 0}
    real = ks.SuspensionKinematics

    class Flaky(real):
        def __init__(self, h):
            calls["n"] += 1
            if calls["n"] % 3 == 0:
                raise RuntimeError("flaky")
            super().__init__(h)
    monkeypatch.setattr(ks, "SuspensionKinematics", Flaky)
    flaky_lin = ks.stochastic_sweep(hp, hw, n=60, n_verify=6, sens=sens, thresholds=T(solver_fail_frac=0.0))
    calls["n"] = 0
    yield "sweep.sf_edge", _res(ks.stochastic_sweep(hp, hw, n=6, mode="full", sens=sens,
                                                    thresholds=T(solver_fail_frac=2 / 6)))
    monkeypatch.setattr(ks, "SuspensionKinematics", real)
    hwb = hw.shifted({p: [5.0, -3.0, 4.0] for p in ks.PERTURBABLE_POINTS[:6]})
    big = ks.stochastic_sweep(hp, hwb, n=300, n_verify=0, yspec=Y(0.1, 0.05, 4.0, 2.0, 0.1))
    nv = ks.robust_nudge(hp, hwb, big, n_verify_full=2, seed=1)
    yield "nudge.far", [_nudge(ks.robust_nudge(hp, hwb, big)), ks.render_stochastic_md(flaky_lin, nv, title="W")]
    seen = []

    def refuse(*a, **k):
        seen.append([k.get("n"), k.get("seed"), k.get("mode"), _a(k.get("target"))])
        raise ValueError("no")
    monkeypatch.setattr(ks, "stochastic_sweep", refuse)
    z = ks.robust_nudge(hp, hwb, big, n_verify_full=1, seed=4)
    monkeypatch.setattr(ks, "stochastic_sweep", real_sweep)
    yield "nudge.refused", [_nudge(z), seen]
    zb = ks.stochastic_sweep(hp, hwb, n=50, n_verify=0, sens=big.sens, yspec=Y(0.0, 0.05, 4.0, 2.0, 0.1))
    yield "nudge.zero_band", _nudge(ks.robust_nudge(hp, hwb, zb))
    A = ks.Adjuster
    cnt2 = {"n": 0}

    class Probe(real):
        def __init__(self, h):
            cnt2["n"] += 1
            if cnt2["n"] == 3:
                raise RuntimeError("probe")
            super().__init__(h)
    monkeypatch.setattr(ks, "SuspensionKinematics", Probe)
    rx = ks.alignment_prescription(hp, {"upper_outer": np.asarray(hp.upper_outer, float) + [2.0, 3.0, 2.0]},
                                   [A("tie_rod_inner", "z", -3, 3, 0.25, "toe link")],
                                   Y(0.001, 0.001, 0.01, 0.01, 0.001))
    monkeypatch.setattr(ks, "SuspensionKinematics", real)
    yield "md.rx.blind", [rx.unreachable, ks.render_prescription_md(rx)]


real_sweep = ks.stochastic_sweep


def test_more_stochastic_steps(hp, monkeypatch):
    check_lazy("kinematik_stochastic_more", _more(hp, monkeypatch), rel=1e-6, abs_=1e-8)


def test_every_stochastic_step(hp, monkeypatch):
    check_lazy("kinematik_stochastic", _cases(hp, monkeypatch), rel=1e-6, abs_=1e-8)
