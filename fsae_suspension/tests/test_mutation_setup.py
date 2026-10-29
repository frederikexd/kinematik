"""suspension/setup.py: the objective is the vehicle model's own limit and balance; the sensitivity is a central
difference over the span actually used (clipped at a knob's bounds); the optimiser is coordinate descent over each
knob's grid with a penalty outside the balance window. A stand-in objective with a known optimum makes each search
step checkable by hand."""
import math
from dataclasses import replace
from types import SimpleNamespace as NS

import numpy as np
import pytest

from suspension import setup as su
from suspension.dynamics import VehicleDynamics, VehicleParams
from suspension.tiremodel import default_tire


def test_the_knob_table():
    assert list(su.PARAM_KNOBS) == ["weight_dist_front", "cg_height", "spring_rate_front", "spring_rate_rear",
                                    "arb_rate_front", "arb_rate_rear", "static_camber_front", "static_camber_rear"]
    assert su.PARAM_KNOBS["weight_dist_front"] == dict(label="Front weight bias", unit="frac", step=0.02, lo=0.42,
                                                       hi=0.58)
    assert su.PARAM_KNOBS["cg_height"] == dict(label="CG height", unit="mm", step=10.0, lo=240.0, hi=360.0)
    for ax in ("front", "rear"):
        A = ax.capitalize()
        assert su.PARAM_KNOBS[f"spring_rate_{ax}"] == dict(label=f"{A} spring rate", unit="N/mm", step=5.0, lo=20.0,
                                                          hi=120.0)
        assert su.PARAM_KNOBS[f"arb_rate_{ax}"] == dict(label=f"{A} anti-roll bar", unit="N·m/°", step=25.0, lo=0.0,
                                                       hi=400.0)
        assert su.PARAM_KNOBS[f"static_camber_{ax}"] == dict(label=f"{A} static camber", unit="°", step=0.5, lo=-4.0,
                                                            hi=0.0)


def test_prepare_seeds_the_camber_and_switches_to_spring_rates():
    p = VehicleParams(static_camber_front=-1.0, static_camber_rear=-1.2)
    f, r = NS(static=NS(camber=-2.3)), NS(static=NS(camber=-0.7))
    q = su._prepare(p, f, r)
    assert (q.static_camber_front, q.static_camber_rear, q.use_param_camber, q.use_spring_rates) == (-2.3, -0.7, True,
                                                                                                     True)
    assert p.use_param_camber is False and p.static_camber_front == -1.0                # the caller's is untouched
    q = su._prepare(p, None, r)
    assert (q.static_camber_front, q.static_camber_rear, q.use_spring_rates) == (-1.0, -0.7, True)
    q = su._prepare(p, f, None)
    assert (q.static_camber_front, q.static_camber_rear, q.use_spring_rates) == (-2.3, -1.2, True)
    q = su._prepare(p, None, None)
    assert (q.use_param_camber, q.use_spring_rates) == (True, False) and q is not p


def test_the_objective_is_the_vehicle_models_limit_and_balance():
    t = default_tire()
    for p in (VehicleParams(), VehicleParams(weight_dist_front=0.52, cg_height=280.0)):
        v = VehicleDynamics(p, tire=t)
        g = v.max_lateral_g()
        b, uf, ur = v.balance_index(min(1.2, g))
        assert su.evaluate(p, tire=t) == pytest.approx(dict(max_g=g, balance=b, util_front=uf, util_rear=ur))
    weak = VehicleParams(mu_peak=0.9)
    v = VehicleDynamics(weak)
    assert v.max_lateral_g() < 1.2
    assert su.evaluate(weak)["balance"] == pytest.approx(v.balance_index(v.max_lateral_g())[0])
    built = su._build(weak, None, None, t)
    assert built.p is weak and built.tire is t and built.front_kin is None


def _fake(p, front_kin=None, rear_kin=None, tire=None):
    """A separable stand-in: grip peaks at cg 250 mm, camber -2.6 / -1.9 deg, wd 0.5; balance moves with the bars."""
    g = (1.5 - 1e-5 * (p.cg_height - 250.0) ** 2 - 0.02 * (p.static_camber_front + 2.6) ** 2
         - 0.03 * (p.static_camber_rear + 1.9) ** 2 - 2.0 * (p.weight_dist_front - 0.5) ** 2
         + 0.001 * p.spring_rate_front)
    b = 0.002 * (p.arb_rate_front - p.arb_rate_rear) + 0.1 * (p.weight_dist_front - 0.47)
    return dict(max_g=g, balance=b, util_front=0.0, util_rear=0.0)


def test_the_sensitivity_is_a_central_difference_over_the_span_used(monkeypatch):
    monkeypatch.setattr(su, "evaluate", _fake)
    p = VehicleParams(cg_height=355.0, static_camber_front=-1.0, arb_rate_front=0.0)
    r = su.sensitivity(p)
    q = su._prepare(p, None, None)
    assert r["base"] == _fake(q)
    rows = {x["knob"]: x for x in r["rankings"]}
    assert set(rows) == set(su.PARAM_KNOBS)
    for name, spec in su.PARAM_KNOBS.items():
        cur = getattr(q, name)
        hi, lo = min(cur + spec["step"], spec["hi"]), max(cur - spec["step"], spec["lo"])
        e_hi, e_lo = _fake(replace(q, **{name: hi})), _fake(replace(q, **{name: lo}))
        row = rows[name]
        assert row["d_maxg_per_step"] == pytest.approx((e_hi["max_g"] - e_lo["max_g"]) / (hi - lo) * spec["step"])
        assert row["d_balance_per_step"] == pytest.approx((e_hi["balance"] - e_lo["balance"]) / (hi - lo)
                                                          * spec["step"])
        assert (row["label"], row["unit"], row["step"], row["current"]) == (spec["label"], spec["unit"], spec["step"],
                                                                            cur)
    mags = [abs(x["d_maxg_per_step"]) for x in r["rankings"]]
    assert mags == sorted(mags, reverse=True) and mags[0] > mags[-1]
    assert rows["cg_height"]["d_maxg_per_step"] == pytest.approx(-1e-5 * ((360 - 250) ** 2 - (345 - 250) ** 2) / 15 * 10)
    one = su.sensitivity(p, knobs=["cg_height"])
    assert [x["knob"] for x in one["rankings"]] == ["cg_height"]


def test_a_knob_pinned_at_both_bounds_is_skipped(monkeypatch):
    monkeypatch.setattr(su, "evaluate", _fake)
    monkeypatch.setitem(su.PARAM_KNOBS, "cg_height", dict(label="CG height", unit="mm", step=10.0, lo=300.0,
                                                          hi=300.0 + 5e-10))
    r = su.sensitivity(VehicleParams(cg_height=300.0), knobs=["cg_height", "arb_rate_front"])
    assert [x["knob"] for x in r["rankings"]] == ["arb_rate_front"]
    monkeypatch.setitem(su.PARAM_KNOBS, "cg_height", dict(label="CG height", unit="mm", step=10.0, lo=300.0,
                                                          hi=300.0 + 2e-9))
    r = su.sensitivity(VehicleParams(cg_height=300.0), knobs=["cg_height"])
    assert [x["knob"] for x in r["rankings"]] == ["cg_height"]


def _descend(p, knobs, target, tol, n_grid, passes):
    def score(q):
        e = _fake(q)
        off = abs(e["balance"] - target)
        return e["max_g"] - (0.5 * (off - tol) if off > tol else 0.0)
    cur = p
    for _ in range(passes):
        for k in knobs:
            spec = su.PARAM_KNOBS[k]
            best_v, best_s = getattr(cur, k), score(cur)
            for v in np.linspace(spec["lo"], spec["hi"], n_grid):
                s = score(replace(cur, **{k: float(v)}))
                if s > best_s:
                    best_v, best_s = float(v), s
            cur = replace(cur, **{k: best_v})
    return cur, score(cur)


def test_the_optimiser_walks_each_knobs_grid(monkeypatch):
    monkeypatch.setattr(su, "evaluate", _fake)
    p = su._prepare(VehicleParams(cg_height=330.0, static_camber_front=-0.5, arb_rate_front=300.0), None, None)
    r = su.optimise(p)
    want, s = _descend(p, list(su.PARAM_KNOBS), 0.04, 0.06, 5, 2)
    assert r["best_params"] == {k: getattr(want, k) for k in su.PARAM_KNOBS
                                if abs(getattr(want, k) - getattr(p, k)) > 1e-9}
    assert r["best_eval"] == _fake(want) and r["start_eval"] == _fake(p)
    assert r["delta_maxg"] == pytest.approx(_fake(want)["max_g"] - _fake(p)["max_g"]) and r["delta_maxg"] > 0
    h = r["history"]
    assert len(h) == 1 + 2 * 8 and h[0]["score"] == pytest.approx(_fake(p)["max_g"] - 0.5 * (abs(_fake(p)["balance"]
                                                                                              - 0.04) - 0.06))
    assert h[-1]["score"] == pytest.approx(s) and h[1]["knob"] == "weight_dist_front"
    assert h[2] == dict(knob="cg_height", set_to=240.0, score=h[2]["score"], **_fake(replace(p, weight_dist_front=0.5,
                                                                                           cg_height=240.0)))
    tight = su.optimise(p, knobs=["arb_rate_front", "static_camber_front"], target_balance=0.1, balance_tol=0.01,
                        n_grid=9, passes=1)
    want, s = _descend(p, ["arb_rate_front", "static_camber_front"], 0.1, 0.01, 9, 1)
    assert tight["history"][-1]["score"] == pytest.approx(s) and len(tight["history"]) == 3
    assert tight["best_params"] == {k: getattr(want, k) for k in ("arb_rate_front", "static_camber_front")
                                    if abs(getattr(want, k) - getattr(p, k)) > 1e-9}
    flat = su.optimise(replace(p, static_camber_front=-2.5), knobs=["static_camber_front"], n_grid=9)
    assert flat["best_params"] == {}                         # already at a grid point no other beats
    edge = su.optimise(replace(p, static_camber_front=-2.5 + 5e-10), knobs=["static_camber_front"], n_grid=9)
    assert edge["best_params"] == {}


def test_the_penalty_only_bites_outside_the_window(monkeypatch):
    def ev(p, *a, **k):
        return dict(max_g=1.0 + 0.001 * p.arb_rate_front, balance=0.001 * p.arb_rate_front, util_front=0, util_rear=0)
    monkeypatch.setattr(su, "evaluate", ev)
    r = su.optimise(VehicleParams(arb_rate_front=0.0), knobs=["arb_rate_front"], target_balance=0.1, balance_tol=0.1,
                    n_grid=5, passes=1)
    # grid 0, 100, 200, 300, 400: balance 0.0-0.4; inside the window up to 0.2, then the slope 0.001 g per unit
    # against a penalty of 0.5 per unit of balance (0.0005 g per unit of bar): grip still wins, 400 is best
    assert r["best_params"] == {"arb_rate_front": 400.0}
    assert r["history"][-1]["score"] == pytest.approx(1.4 - 0.5 * (0.3 - 0.1))
    monkeypatch.setattr(su, "evaluate", lambda p, *a, **k: dict(max_g=1.0 + 0.001 * p.arb_rate_front,
                                                                balance=0.01 * p.arb_rate_front, util_front=0,
                                                                util_rear=0))
    r = su.optimise(VehicleParams(arb_rate_front=0.0), knobs=["arb_rate_front"], target_balance=0.1, balance_tol=0.1,
                    n_grid=5, passes=1)
    # balance 0-4: the penalty (0.005 g per unit of bar) beats the grip (0.001): stay at 0 (score 1.0, in window)
    assert r["best_params"] == {} and r["history"][-1]["score"] == pytest.approx(1.0)
    at_edge = su.optimise(VehicleParams(arb_rate_front=20.0), knobs=["arb_rate_front"], target_balance=0.1,
                          balance_tol=0.1, n_grid=2, passes=1)
    assert at_edge["history"][0]["score"] == pytest.approx(1.02)          # balance 0.2: on the window edge, no penalty
    assert not math.isnan(at_edge["delta_maxg"])


def test_the_corners_and_tire_reach_every_evaluation(monkeypatch):
    f, r, t = NS(static=NS(camber=-2.1)), NS(static=NS(camber=-1.3)), object()
    seen = []

    def ev(p, front_kin=None, rear_kin=None, tire=None):
        seen.append((front_kin, rear_kin, tire, p.static_camber_front, p.static_camber_rear))
        return _fake(p)
    monkeypatch.setattr(su, "evaluate", ev)
    su.sensitivity(VehicleParams(), f, r, t, knobs=["cg_height"])
    su.optimise(VehicleParams(), f, r, t, knobs=["cg_height"], n_grid=2, passes=1)
    assert len(seen) == 3 + 3 and all(s[:3] == (f, r, t) for s in seen)
    assert all(s[3:] == (-2.1, -1.3) for s in seen)                         # the solved camber seeded both ways
    h = su.optimise(VehicleParams(), knobs=["cg_height"], n_grid=2, passes=1)["history"][0]
    assert set(h) == {"score", "max_g", "balance", "util_front", "util_rear"}
    built = []
    monkeypatch.setattr(su, "VehicleDynamics", lambda p, front_kin=None, rear_kin=None, tire=None:
                        built.append((front_kin, rear_kin, tire)) or "veh")
    assert su._build("p", f, r, t) == "veh" and built == [(f, r, t)]


def test_evaluate_builds_on_the_corners_and_tire(monkeypatch):
    f, r, t, calls = object(), object(), object(), []
    monkeypatch.setattr(su, "_build", lambda p, a, b, c: calls.append((a, b, c)) or NS(
        max_lateral_g=lambda: 1.0, balance_index=lambda g: (0.1, 0.9, 0.8)))
    assert su.evaluate(VehicleParams(), f, r, t) == dict(max_g=1.0, balance=0.1, util_front=0.9, util_rear=0.8)
    assert calls == [(f, r, t)]
