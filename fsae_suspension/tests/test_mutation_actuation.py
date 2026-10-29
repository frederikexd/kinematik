"""Characterization suite for suspension/actuation.py (mutation gate); see tests/_golden.py.

The optimiser cases replace scipy's least_squares and the kinematic metrics with cheap deterministic stand-ins, so the
synthesis bookkeeping (bounds, starts, scoring, node seeding, ride rates) is pinned without the cost of the real solve;
the real metrics are pinned separately on the default hardpoints."""
import copy
import types

import numpy as np
import pytest

from suspension import actuation as ac
from suspension import genesis_repro as gr
from suspension.kinematics import Hardpoints, SuspensionKinematics
from tests._golden import check

HP = Hardpoints.default()


def test_targets_and_geometry_helpers():
    res = {"t": [ac.ActuationTargets(), ac.ActuationTargets.derived(),
                 ac.ActuationTargets.derived(50.0, (-20.0, 30.0), 0.7, 0.1, 0.2, 300.0)],
           "ceil": [ac.ActuationTargets().mr_ceiling, ac.ActuationTargets(travel_mm=(-10.0, 30.0)).mr_ceiling]}
    res["ang"] = [ac._angle_deg((1, 0, 0), v) for v in ((1, 0, 0), (0, 1, 0), (-1, 0, 0), (1, 1, 0), (1, 1e-9, 0))]
    segs = [((0, 0, 0), (10, 0, 0), (5, 5, -5), (5, 5, 5)), ((0, 0, 0), (10, 0, 0), (0, 3, 0), (10, 3, 0)),
            ((0, 0, 0), (10, 0, 0), (12, 1, 0), (15, 1, 0)), ((0, 0, 0), (0, 0, 0), (1, 1, 1), (2, 2, 2)),
            ((0, 0, 0), (10, 0, 0), (3, 4, 0), (3, 4, 0)), ((0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0)),
            ((0, 0, 0), (10, 0, 0), (-5, 2, 0), (-1, 9, 0))]
    res["seg"] = [ac._seg_dist(*s) for s in segs]
    A = np.array([[5, 5, -5], [0, 3, 0], [20, 0, 0]], float)
    B = np.array([[5, 5, 5], [10, 3, 0], [30, 0, 0]], float)
    res["many"] = [ac._seg_dist_many((0, 0, 0), (10, 0, 0), A, B, np.array([1.0, 0.5, 2.0])),
                   ac._seg_dist_many((0, 0, 0), (10, 0, 0), A[2:], B[2:], np.zeros(1))]
    res["toward"] = [ac._toward(np.zeros(3), (3.0, 4.0, 0.0), 10.0), ac._toward(np.ones(3), (1.0, 1.0, 1.0), 7.0)]
    R = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    res["carried"] = ac._carried([((1, 0, 0), (2, 0, 0), 3), ((1, 0, 0), (2, 0, 0), 4, "link"),
                                  ((0, 1, 0), (0, 2, 0), 5, "wheel")], R, np.array([1.0, 2.0, 3.0]))
    h = ac._set(HP, np.arange(1.0, 10.0), ["pushrod_outer", "rocker_axis", "spring_inner"])
    res["set"] = [h.pushrod_outer, h.rocker_axis, h.spring_inner, HP.pushrod_outer]
    check("actuation_helpers", res)


def _residual_cases():
    M = ac.ActuationMetrics
    good = M(ok=True, mr=[0.8, 0.85], mr_static=0.86, spread=0.12, out_of_plane_deg=7.0,
             transmission_deg=(35.0, 150.0), stroke_used_mm=48.0, clearance_mm=3.0, node_gap_mm=40.0,
             arm_offset_mm=8.0, spring_length_mm=(250.0, 230.0, 200.0))
    clean = M(ok=True, mr_static=0.85, spread=0.0, out_of_plane_deg=0.0, transmission_deg=(90.0, 90.0),
              stroke_used_mm=10.0, clearance_mm=100.0, node_gap_mm=0.0, arm_offset_mm=0.0)
    T = ac.ActuationTargets
    return {"bad": (M(ok=False), T(), 1.0), "good": (good, T(), 1.0), "good_s": (good, T(), 0.9),
            "damper": (good, T(damper_extended_mm=240.0), 1.0), "damper_s": (good, T(damper_extended_mm=240.0), 0.95),
            "damper_nolen": (clean, T(damper_extended_mm=240.0), 1.0), "clean": (clean, T(), 1.0),
            "low": (M(ok=True, mr_static=0.7, spread=0.05, out_of_plane_deg=1.0, transmission_deg=(60.0, 120.0),
                      stroke_used_mm=60.0, clearance_mm=10.0, node_gap_mm=10.0, arm_offset_mm=1.0,
                      spring_length_mm=(200.0, 190.0, 150.0)),
                    T(mr_static=0.8, mr_tol=0.02, max_spread=0.2, max_out_of_plane_deg=2.0,
                      transmission_deg=(50.0, 110.0), stroke_mm=40.0, usable_stroke=0.9, min_clearance_mm=8.0,
                      node_reach_mm=5.0, max_arm_offset_mm=0.5, damper_extended_mm=180.0), 0.8)}


def test_residual():
    check("actuation_residual", {k: ac._residual(m, t, shrink=s) if s != 1.0 else ac._residual(m, t)
                                 for k, (m, t, s) in _residual_cases().items()})


def test_metrics_on_default_hardpoints():
    kin = SuspensionKinematics(HP)
    s0 = kin.solve_at_travel(0.0)
    po = np.asarray(s0.pushrod_outer, float)
    rp = np.asarray(HP.rocker_pushrod, float)
    rs, si = np.asarray(HP.rocker_spring, float), np.asarray(HP.spring_inner, float)
    mid = po + 0.5 * (rp - po)
    cap = (mid + [0.0, 0.0, 30.0], mid + [1.0, 0.0, 30.0], 10.0)
    smid = rs + 0.5 * (si - rs)
    scap = (smid + [0.0, 25.0, 0.0], smid + [0.0, 26.0, 0.0], 4.0)
    link = (po, po + np.array([0.0, -300.0, 0.0]), 5.0, "link")
    nodes = np.array([np.asarray(HP.rocker_pivot) + [3.0, 4.0, 0.0], np.asarray(HP.spring_inner) + [0.0, 0.0, 9.0],
                      [5000.0, 0.0, 0.0]])
    res = {"default": ac.actuation_metrics(HP),
           "all": ac.actuation_metrics(HP, (-20.0, 15.0), 5, obstacles=[cap, scap], nodes=nodes,
                                       wheel_obstacles=[cap + ("wheel",), link], pushrod_skip_mm=400.0,
                                       member_radii=(1.0, 2.0, 3.0, 4.0), spring_skip_mm=500.0),
           "skips": ac.actuation_metrics(HP, (-10.0, 10.0), 3, wheel_obstacles=[link], pushrod_skip_mm=5.0,
                                         spring_skip_mm=5.0),
           "frame": ac.actuation_metrics(HP, (-10.0, 10.0), 4, obstacles=[scap], nodes=[]),
           "typeerr": ac.actuation_metrics(HP, (None, 10.0)).reason}
    h = copy.deepcopy(HP)
    h.spring_inner = None
    res["incomplete"] = ac.actuation_metrics(h)
    check("actuation_metrics", res)


class _FakeKin(SuspensionKinematics):
    mode = ""
    calls = 0
    armed = False

    def motion_ratio_curve(self, a, b, n):
        trv, mr = super().motion_ratio_curve(a, b, n)
        type(self).calls, type(self).armed = 0, True
        if self.mode == "nan":
            mr = np.array(mr, float); mr[1] = np.nan
        return trv, mr

    def spring_length_at(self, st, seed=0.0):
        L, th, ok = super().spring_length_at(st, seed=seed)
        type(self).calls += self.armed
        if self.armed and self.mode.startswith("fail") and type(self).calls == int(self.mode[4:]):
            return L, th, False
        return L, th, ok


@pytest.mark.parametrize("mode", ["nan", "fail1", "fail5", "fail12", "fail23"])
def test_metrics_failure_paths(monkeypatch, mode):
    monkeypatch.setattr(ac, "SuspensionKinematics", _FakeKin)
    monkeypatch.setattr(_FakeKin, "mode", mode)
    monkeypatch.setattr(_FakeKin, "calls", 0)
    monkeypatch.setattr(_FakeKin, "armed", False)
    m = ac.actuation_metrics(HP, (-5.0, 5.0), 3)
    check(f"actuation_fail_{mode}", {"m": m})


def _fake_metrics(hp, travel, **kw):
    """Cheap, smooth stand-in: every field is a fixed function of the hardpoints and keyword options."""
    v = np.concatenate([np.asarray(getattr(hp, p), float) for p in ac.ACTUATION_POINTS])
    s = float(np.sum(np.sin(v / 37.0)))
    obs = len(kw.get("obstacles", ())) + 2 * len(kw.get("wheel_obstacles", ()))
    extra = kw.get("pushrod_skip_mm", 0.0) + kw.get("spring_skip_mm", 0.0) + sum(kw.get("member_radii", ()))
    nodes = kw.get("nodes")
    gap = 0.0 if nodes is None else float(np.min(np.linalg.norm(np.asarray(nodes) - v[3:6], axis=1)))
    if v[0] > 1e5:
        return ac.ActuationMetrics(ok=False, reason="off")
    mr0 = 0.8 + 0.01 * s
    return ac.ActuationMetrics(ok=True, travel=list(travel), mr=[mr0 - 0.01, mr0, mr0 + 0.02], mr_static=mr0,
                               spread=0.05 + 0.001 * abs(s), out_of_plane_deg=3.0 + 0.1 * s,
                               transmission_deg=(45.0 + s, 130.0 - s), stroke_used_mm=40.0 + s,
                               clearance_mm=10.0 - obs - 0.01 * extra, node_gap_mm=gap * 0.01,
                               arm_offset_mm=abs(s) * 0.1, spring_length_mm=(250.0, 240.0, 225.0 + s))


def _fake_ls(log, fail_first=0):
    state = {"n": 0}

    def least_squares(fun, x0, bounds, **kw):
        state["n"] += 1
        if state["n"] <= fail_first:
            raise RuntimeError("diverged")
        r = fun(np.asarray(x0, float))
        lo, hi = bounds
        x = np.clip(np.asarray(x0, float) + 0.25 * (hi - lo) * np.tanh(r[:x0.size % 7 + 1].sum()), lo, hi)
        log.append({"x0": x0, "lo": lo, "hi": hi, "r": r, "kw": kw})
        return types.SimpleNamespace(x=x)
    return least_squares


def test_synthesis_bookkeeping(monkeypatch):
    monkeypatch.setattr(ac, "actuation_metrics", _fake_metrics)
    res = {}
    runs = {
        "default": ({}, 0),
        "kw": (dict(targets=ac.ActuationTargets(mr_static=0.7, damper_extended_mm=260.0), box_mm=30.0,
                    pushrod_box_mm=10.0, points=("pushrod_outer", "rocker_axis", "spring_inner"),
                    obstacles=[((0, 0, 0), (1, 0, 0), 2.0)], n_starts=3, seed=5, ride_hz=3.0,
                    sprung_corner_kg=60.0, wheel_obstacles=[((0, 0, 0), (0, 1, 0), 1.0)], pushrod_skip_mm=4.0,
                    axis_box=0.3, member_radii=(1.0, 0.0, 0.0, 2.0), spring_skip_mm=6.0), 0),
        "one_start": (dict(n_starts=1, ride_hz=2.5), 0),
        "zero_starts": (dict(n_starts=0, sprung_corner_kg=50.0), 0),
        "fail_some": (dict(n_starts=3, ride_hz=3.0, sprung_corner_kg=55.0), 2),
        "fail_all": (dict(n_starts=2), 99),
        "nodes": (dict(nodes=np.array([[100.0, 200.0, 300.0], [400.0, 300.0, 200.0], [150.0, 250.0, 350.0],
                                       [0.0, 0.0, 0.0]]), standoff_mm=20.0, n_node_candidates=3, n_starts=2,
                       ride_hz=3.2, sprung_corner_kg=58.0), 0),
        "nodes_fail": (dict(nodes=np.array([[100.0, 200.0, 300.0], [400.0, 300.0, 200.0]]), n_starts=0), 99),
        "nodes_some": (dict(nodes=np.array([[10.0, 20.0, 30.0], [40.0, 30.0, 20.0], [15.0, 25.0, 35.0]]),
                            n_node_candidates=2, n_starts=3), 1),
    }
    for k, (kw, fail) in runs.items():
        log = []
        monkeypatch.setattr(ac, "least_squares", _fake_ls(log, fail))
        r = ac.synthesize_actuation(HP, **kw)
        res[k] = {"r": r, "log": log}
    for bad in (dict(wheel_obstacles=[((0, 0, 0), (0, 1, 0), 1.0)]), dict(pushrod_skip_mm=1.0),
                dict(spring_skip_mm=1.0), dict(member_radii=(0.0, 0.0, 1.0, 0.0))):
        with pytest.raises(ValueError, match="^node-seeded synthesis does not take wheel_obstacles or pushrod_skip"):
            ac.synthesize_actuation(HP, nodes=np.zeros((2, 3)), **bad)
    check("actuation_synth", res)


def test_synthesis_stops_on_a_perfect_start(monkeypatch):
    calls = []

    def perfect(hp, travel, **kw):
        calls.append(1)
        return ac.ActuationMetrics(ok=True, mr=[0.85], mr_static=0.85, spread=0.0, out_of_plane_deg=0.0,
                                   transmission_deg=(90.0, 90.0), stroke_used_mm=1.0, clearance_mm=100.0)

    monkeypatch.setattr(ac, "actuation_metrics", perfect)
    monkeypatch.setattr(ac, "least_squares", lambda fun, x0, bounds, **kw: types.SimpleNamespace(x=x0))
    t = ac.ActuationTargets(mr_static=0.85)
    r = ac.synthesize_actuation(HP, t, n_starts=5)
    assert r.ok and r.residual == [0.0] * 11 and len(calls) == 3
    calls.clear()
    nodes = np.array([[0.0, 0.0, 0.0], [50.0, 0.0, 0.0], [0.0, 60.0, 0.0]])
    r = ac.synthesize_actuation(HP, t, n_starts=5, nodes=nodes, n_node_candidates=3)
    assert r.ok and len(calls) == 6 + 2 + 1


def test_bearing_loads():
    cases = gr.vehicle_load_cases()
    h = copy.deepcopy(HP)
    h.rocker_pushrod = np.asarray(h.rocker_pushrod, float) + np.array([40.0, 0.0, 0.0])
    plain_case = types.SimpleNamespace(Fz=1500.0)
    b = ac.RockerBearing(label="x", C0_N=5000.0, spacing_mm=20.0, X0=0.5, Y0=0.4, required_s0=3.0)
    res = {"default": ac.rocker_bearing_loads(HP, cases), "offset": ac.rocker_bearing_loads(h, cases, b),
           "plain": [{k: v for k, v in r.items() if k != "case"} for r in ac.rocker_bearing_loads(HP, [plain_case])],
           "zero": ac.rocker_bearing_loads(HP, [types.SimpleNamespace(name="0")]),
           "spacing": ac.required_rating_vs_spacing(HP, cases, [10, 40.0], b),
           "spacing_d": ac.required_rating_vs_spacing(HP, cases[:2], [25.0]),
           "select": ac.select_rocker_bearing(h, cases, [ac.RockerBearing(), b,
                                                         ac.RockerBearing(label="big", C0_N=1e6)])}
    h2 = copy.deepcopy(HP)
    h2.spring_inner = None
    with pytest.raises(ValueError, match="^actuation points incomplete"):
        ac.rocker_bearing_loads(h2, cases)
    check("actuation_bearing", res)


def _leaning():
    """Default corner with the spring end of the rocker moved along the rocker axis, so the forces leave its plane."""
    h = copy.deepcopy(HP)
    k = np.asarray(SuspensionKinematics(HP)._rocker_axis, float)
    h.rocker_spring = np.asarray(h.rocker_spring, float) + 12.0 * k
    return h


def test_characterized_metrics_and_bearings():
    res = {}
    rs, si = np.asarray(HP.rocker_spring, float), np.asarray(HP.spring_inner, float)
    u = (si - rs) / np.linalg.norm(si - rs)
    near_rs = (rs - 6.0 * u + [0.0, 0.0, 9.0], rs - 6.0 * u + [1.0, 0.0, 9.0], 3.0)
    near_si = (si + 2.0 * u + [0.0, 7.0, 0.0], si + 2.0 * u + [0.0, 8.0, 0.0], 2.0)
    po = np.asarray(SuspensionKinematics(HP).solve_at_travel(0.0).pushrod_outer, float)
    w = (po - np.asarray(HP.rocker_pushrod, float)) / np.linalg.norm(po - np.asarray(HP.rocker_pushrod, float))
    beyond = [(si + 4.0 * u, si + 4.0 * u + [0.0, 0.0, 1.0], 1.0), (po + 4.0 * w, po + 4.0 * w + [1.0, 0.0, 0.0], 1.0)]
    for k, kw in {"short_skip": dict(spring_skip_mm=4.0), "long_skip": dict(spring_skip_mm=1000.0),
                  "half_mm": dict(spring_skip_mm=0.5, pushrod_skip_mm=0.5), "none": {}}.items():
        m = ac.actuation_metrics(HP, (-8.0, 8.0), 3, obstacles=[near_rs, near_si], **kw)
        res[k + "_end"] = ac.actuation_metrics(HP, (-1.0, 1.0), 3, obstacles=beyond, **kw).member_clearance_mm
        res[k] = [m.clearance_mm, m.member_clearance_mm]
    res["lean"] = ac.actuation_metrics(_leaning(), (-8.0, 8.0), 3)
    res["bad_obstacle"] = ac.actuation_metrics(HP, (-5.0, 5.0), 3, obstacles=[(1.0, 2.0)])
    cases = [types.SimpleNamespace(name="wc", Fz=1800.0, Fy=900.0, Fx_wc=-400.0, F_wc_extra=(50.0, -80.0, 30.0),
                                   M_wc_extra=(2000.0, -1500.0, 900.0)),
             types.SimpleNamespace(Fz=1200.0, Fx=300.0, mz_Nmm=5000.0)]
    b = ac.RockerBearing(label="y", C0_N=8000.0, spacing_mm=18.0, X0=0.5, Y0=40.0, required_s0=2.0)
    rows = ac.rocker_bearing_loads(_leaning(), cases, b)
    rows[1]["case"] = rows[1]["case"].startswith("namespace(")
    res["bearing"] = rows
    res["spacing"] = ac.required_rating_vs_spacing(_leaning(), cases, [12.0, 30.0], b)
    check("actuation_lean", res)


def _fake_ls_moving(log):
    def least_squares(fun, x0, bounds, **kw):
        lo, hi = bounds
        x = np.asarray(x0, float) + 0.1 * (hi - lo)
        r = fun(x)
        log.append({"r": r})
        return types.SimpleNamespace(x=np.clip(x, lo, hi))
    return least_squares


def _fake_metrics_nodes(hp, travel, **kw):
    m = _fake_metrics(hp, travel, **kw)
    nodes = kw.get("nodes")
    m.node_gap_mm = 5.0 if nodes is None else 0.05 * float(np.min(np.linalg.norm(
        np.asarray(nodes) - np.asarray(hp.rocker_pivot, float), axis=1)))
    return m


def test_characterized_node_synthesis(monkeypatch):
    monkeypatch.setattr(ac, "actuation_metrics", _fake_metrics_nodes)
    t = ac.ActuationTargets(min_clearance_mm=15.0, node_reach_mm=0.5)
    nodes = np.array([[100.0, 200.0, 300.0], [400.0, 300.0, 200.0], [150.0, 250.0, 350.0], [10.0, 20.0, 30.0]])
    obs = [((0, 0, 0), (1, 0, 0), 2.0), ((5, 5, 5), (6, 5, 5), 1.0)]
    res = {}
    for k, kw in {"one": dict(n_starts=1), "three": dict(n_starts=3, ride_hz=3.0),
                  "hz_only": dict(n_starts=2, ride_hz=3.0)}.items():
        log = []
        monkeypatch.setattr(ac, "least_squares", _fake_ls_moving(log))
        r = ac.synthesize_actuation(HP, t, nodes=nodes, obstacles=obs, n_node_candidates=3, **kw)
        res[k] = {"r": r, "log": log}
    check("actuation_nodes", res)
