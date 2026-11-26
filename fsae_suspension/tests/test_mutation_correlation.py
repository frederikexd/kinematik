"""Characterization suite for suspension/correlation.py (mutation gate); see tests/_golden.py. The skidpad
conversions both ways (and their refusals), skidpad correlation from a g, a time, both or neither, a model that
raises, custom tolerances, acceleration at and either side of its band, and speed traces: matching, fast, slow,
offset, km/h, constant speed, partly overlapping, disjoint, too short, mismatched, unsorted, NaN-holding,
unparseable, and taken from a lap result in either field spelling."""
import math
from types import SimpleNamespace

import numpy as np

from suspension import correlation as C
from tests._golden import plain
from tests._golden_lazy import check_lazy


class Veh:
    def __init__(self, g):
        self.g = g

    def max_lateral_g(self):
        if self.g is None:
            raise RuntimeError("no tyre")
        return self.g


def _rep(r):
    d = r.as_dict()
    return plain(d) | {"channel_verdicts": [c.verdict for c in r.channels],
                       "trace_arrays": None if r.trace is None or r.trace.distance is None else
                       [plain(r.trace.distance), plain(r.trace.measured), plain(r.trace.predicted)]}


def _cases():
    yield "convert", [C.skidpad_time_from_lateral_g(g, r) for g, r in ((1.5, 9.125), (2.0, 8.0), (0.0, 9.0),
                                                                        (-1.0, 9.0), (math.nan, 9.0), (1.0, 0.0),
                                                                        (1.0, math.inf), (math.inf, 9.0))] + [
        C.skidpad_time_from_lateral_g(1.4)] + [
        C.lateral_g_from_skidpad_time(t, r) for t, r in ((5.0, 9.125), (4.8, 8.0), (0.0, 9.0), (-2.0, 9.0),
                                                          (math.nan, 9.0), (5.0, -1.0), (5.0, math.nan))] + [
        C.lateral_g_from_skidpad_time(5.1)]
    yield "pct", [C._pct_err(p, m) for p, m in ((1.1, 1.0), (1.0, 0.0), (1.0, 1e-10), (math.nan, 1.0), (1.0, math.inf),
                                                 (-2.0, 1.0), (1.0, -1e-9))]
    yield "channel", [C._channel("c", m, p, u, t).as_dict() for m, p, u, t in (
        (1.0, 1.05, "g", 5.0), (1.0, 1.0501, "g", 5.0), (1.0, 0.95, "", 5.0), (0.0, 1.0, "s", 5.0),
        (math.nan, 1.0, "s", 5.0), (1.0, math.nan, "s", 5.0), (2.0, 2.0, "s", 0.0))]
    for name, veh, kw in (("g", Veh(1.6), {"measured_g": 1.55}), ("time", Veh(1.6), {"measured_time_s": 5.0}),
                          ("both", Veh(1.6), {"measured_g": 1.5, "measured_time_s": 4.9}),
                          ("neither", Veh(1.6), {}), ("raises", Veh(None), {"measured_g": 1.5}),
                          ("radius", Veh(1.4), {"measured_g": 1.4, "radius_m": 8.0}),
                          ("tol", Veh(1.6), {"measured_g": 1.5, "tol": {"skidpad_g_pct": 10.0}}),
                          ("badtime", Veh(1.6), {"measured_time_s": -1.0}), ("zero", Veh(0.0), {"measured_g": 1.5})):
        yield f"skidpad.{name}", _rep(C.correlate_skidpad(veh, **kw))
    for m, p, tol in ((4.0, 4.16, None), (4.0, 4.1601, None), (4.0, 3.84, None), (4.0, 4.4, {"accel_time_pct": 12.0}),
                      (0.0, 4.0, None), (4.0, math.nan, None)):
        yield f"accel.{m}.{p}.{tol}", _rep(C.correlate_acceleration(m, p, tol=tol))
    d = np.linspace(0.0, 100.0, 21)
    v = 10.0 + 5.0 * np.sin(d / 15.0)
    traces = {
        "match": (d, v, d, v), "fast": (d, v, d, v * 1.1), "slow": (d, v, d, v * 0.9),
        "offset": (d, v, d + 5.0, v), "noise": (d, v, d, v + 0.2 * np.cos(d)),
        "constant": (d, np.full_like(d, 12.0), d, np.full_like(d, 12.3)),
        "flatish": (d, 12.0 + 0.1 * np.sin(d), d, np.full_like(d, 12.0)),
        "partial": (d, v, d[5:15], v[5:15]), "disjoint": (d, v, d + 500.0, v), "one_overlap": (d, v, d + 100.0, v),
        "short": (d[:1], v[:1], d, v), "mismatch": (d, v[:-1], d, v), "pred_short": (d, v, d[:1], v[:1]),
        "unsorted": (d, v, d[::-1], v[::-1]), "nan": (d, np.where(d == 50.0, np.nan, v), d, v),
        "pred_nan": (d, v, d, np.where(d == 50.0, np.inf, v)), "zero_speed": (d, np.zeros_like(d), d, np.zeros_like(d)),
        "lists": (list(d), list(v), list(d), list(v * 1.02)),
    }
    for name, (md, ms, pd_, ps) in traces.items():
        yield f"trace.{name}", _rep(C.correlate_speed_trace(md, ms, pd_, ps))
    yield "trace.kmh", _rep(C.correlate_speed_trace(d, v * 3.6, d, v, measured_speed_kmh=True))
    yield "trace.tol", _rep(C.correlate_speed_trace(d, v, d, v * 1.1, tol={"trace_rmse_frac": 0.5,
                                                                             "trace_bias_frac": 0.5}))
    yield "trace.bad", _rep(C.correlate_speed_trace(d, v, d, object()))
    yield "trace.none", _rep(C.correlate_speed_trace(d, v))
    for name, lap in (("ds", SimpleNamespace(distance=d, speed=v * 1.01)), ("sv", SimpleNamespace(s=d, v=v * 0.99)),
                      ("mixed", SimpleNamespace(distance=d, v=v)), ("nodist", SimpleNamespace(speed=v)),
                      ("nospeed", SimpleNamespace(distance=d)), ("short", SimpleNamespace(distance=[1.0], speed=[2.0])),
                      ("unequal", SimpleNamespace(distance=d, speed=v[:-2])),
                      ("junk", SimpleNamespace(distance=object(), speed=v)), ("none", None)):
        yield f"lap.{name}", [plain(C._extract_trace(lap)), _rep(C.correlate_speed_trace(d, v, lap_result=lap))]
    yield "lap.explicit_wins", _rep(C.correlate_speed_trace(d, v, d, v, lap_result=SimpleNamespace(distance=d,
                                                                                                   speed=v * 2)))
    ch = C._channel("x", 1.0, 1.0, "g", 5.0)
    tr = C.correlate_speed_trace(d, v, d, v).trace
    yield "finish", [_rep(C._finish([], None, C.DEFAULT_TOL)), _rep(C._finish([ch], tr, C.DEFAULT_TOL, context="")),
                     _rep(C._finish([C._channel("y", 1.0, 2.0, "", 5.0)], tr, C.DEFAULT_TOL, context="c"))]
    yield "units", [C._u(""), C._u(None), C._u("g")]
    yield "edges", [C.skidpad_time_from_lateral_g(0.5, 9.0), C.skidpad_time_from_lateral_g(1.5, 0.5),
                    C.lateral_g_from_skidpad_time(0.5, 9.0), C.lateral_g_from_skidpad_time(5.0, 0.5),
                    C._pct_err(math.inf, 1.0), C._pct_err(1.0, math.inf), C._channel("i", 1.0, math.inf, "", 5.0).as_dict(),
                    C._channel("j", math.inf, 1.0, "", 5.0).as_dict()]
    yield "skidpad.time_radius", _rep(C.correlate_skidpad(Veh(1.6), measured_time_s=4.5, radius_m=8.0))
    d2, v2 = np.array([0.0, 10.0]), np.array([5.0, 6.0])
    yield "trace.two", [_rep(C.correlate_speed_trace(d2, v2, d2, v2 * 1.01)),
                        _rep(C.correlate_speed_trace(d, v, d2, v2)),
                        _rep(C.correlate_speed_trace(d, v, np.array([90.0, 100.0, 110.0]), np.array([9.0, 9.5, 9.8]))),
                        plain(C._extract_trace(SimpleNamespace(distance=d2, speed=v2))),
                        plain(C._extract_trace(SimpleNamespace(distance=None, speed=v))),
                        plain(C._extract_trace(SimpleNamespace(distance=d2, speed=np.array([1.0, 2.0, 3.0]))))]
    yield "trace.half_given", _rep(C.correlate_speed_trace(d, v, d, None, lap_result=SimpleNamespace(distance=d,
                                                                                                     speed=v * 1.03)))
    slow = 0.5 + 0.05 * np.sin(d / 15.0)
    yield "trace.slow_car", [_rep(C.correlate_speed_trace(d, slow, d, slow * 1.2)),
                             _rep(C.correlate_speed_trace(d, slow * 1e-10, d, slow * 1e-10))]
    base = C.correlate_speed_trace(d, v, d, v * 1.04 + 0.1)
    t = base.trace
    yield "trace.exact_tol", [_rep(C.correlate_speed_trace(d, v, d, v * 1.04 + 0.1,
                                                           tol={"trace_rmse_frac": t.rmse_frac,
                                                                "trace_bias_frac": t.bias_frac}))]
    yield "trace.zero_bias", [_rep(C.correlate_speed_trace(d, v, d, v, tol={"trace_bias_frac": -1.0})),
                              _rep(C.correlate_speed_trace(d, v, d, v + 0.5, tol={"trace_bias_frac": 0.0}))]
    yield "consts", [C.G, C.DEFAULT_TOL]


def test_every_correlation():
    check_lazy("correlation", _cases())
