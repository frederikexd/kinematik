"""Characterization suite for suspension/rules_fsae.py (mutation gate); see tests/_golden.py. Every rule and limit,
declared values below, at and above each limit (and missing, NaN and the kWh-to-MJ seam), and logs that trip the
dwell test, the moving-average test, both, neither, regen below the speed floor, slow sampling, too few samples and
NaN gaps."""
import math

import numpy as np

from suspension import rules_fsae as R
from tests._golden import plain
from tests._golden_lazy import check_lazy


def _declared():
    p_lim, v_lim = R.limit("EV.3.3.1"), R.limit("EV.3.3.2")
    mv, me, mm = R.limit("EV.5.1.1a"), R.limit("EV.5.1.1b"), R.limit("EV.5.1.1c")
    base = dict(power_kw=p_lim, pack_v_max=v_lim, module_v=mv, module_kg=mm, module_kwh=me / 3.6, pack_kwh=6.5)
    yield "at_limit", base
    yield "below", {k: v * 0.9 for k, v in base.items()}
    yield "above", {k: v * 1.1 for k, v in base.items()}
    yield "just_above", {k: v + 1e-6 for k, v in base.items()}
    yield "empty", {}
    yield "nan", {k: math.nan for k in base}
    yield "small_pack", dict(pack_kwh=0.5)
    yield "one_module", dict(pack_kwh=me / 3.6)
    yield "two_modules", dict(pack_kwh=3.0)
    yield "inf", dict(module_kwh=math.inf, pack_kwh=math.inf, power_kw=math.inf)
    yield "negative", {k: -1.0 for k in base}


def _log(kind, n=400, fs=200.0):
    t = np.arange(n) / fs
    v = np.full(n, 520.0)
    i = np.full(n, 80.0)
    s = np.full(n, 15.0)
    if kind == "dwell":
        i[100:150] = 200.0                     # 250 ms over the power limit
    elif kind == "blip":
        i[100:110] = 200.0                     # 50 ms: too short for the dwell test
    elif kind == "average":
        i[100:300:2] = 260.0                   # spiky, but the moving average crosses
    elif kind == "voltage":
        v[50:120] = 700.0
    elif kind == "both":
        i[100:150] = 200.0
        v[200:260] = 700.0
        i[300:390:2] = 260.0
    elif kind == "regen":
        i[200:260] = -40.0
        s[200:260] = 0.8
    elif kind == "regen_edges_gap":
        i[200:260] = -40.0
        s[200:260] = 0.8
        v[50:60] = np.nan
    elif kind == "regen_edges":
        i[100:130] = -1.0                      # half a kilowatt of regen at walking pace
        s[100:130] = 0.9
        i[160:190] = 0.0                       # coasting slowly is not regen
        s[160:190] = 0.5
        i[220:250] = -40.0                     # stopped dead
        s[220:250] = 0.0
        i[280:310] = -40.0                     # between 5 km/h over 4.6 and over 3.6
        s[280:310] = 1.2
    elif kind == "exact":
        v[:] = 500.0
        i[100:200] = 160.0                     # exactly 80 kW, held
    elif kind == "dupes":
        t[1::2] = t[0::2]                      # every timestamp twice
    elif kind == "regen_fast":
        i[200:260] = -40.0
    elif kind == "gaps":
        v[10:20] = np.nan
        t[30] = np.nan
        i[100:150] = 200.0
    return t, v, i, s


def _cases():
    yield "ruleset", [plain(R.RULESET), R.RULESET.label(), R.banner()]
    yield "rules", plain(R.RULES)
    yield "by_section", plain(R.rules_by_section())
    yield "limits", {r.rid: R.limit(r.rid) for r in R.RULES} | {"nope": R.limit("nope")}
    yield "coverage", list(R.coverage())
    for name, params in _declared():
        fs = R.check_declared(params)
        yield f"declared.{name}", [plain(fs), [plain(f.rule) for f in fs]]
    for rid in ("EV.3.3.1", "EV.5.1.1c", "T.9.1.1"):
        for val in (None, 0.0, 10.0, 1e6, math.nan):
            for at_most in (True, False):
                yield f"cmp.{rid}.{val}.{at_most}", plain(R._cmp(rid, val, at_most=at_most))
    for kind in ("clean", "dwell", "blip", "average", "voltage", "both", "regen", "regen_fast", "gaps", "regen_edges",
                 "exact", "dupes", "regen_edges_gap"):
        t, v, i, s = _log(kind)
        for speed in (True, False):
            r = R.check_measured(t, v, i, s if speed else None)
            yield f"log.{kind}.{speed}", [plain(r), r.n_violations, r.endurance_penalty_s]
    t, v, i, s = _log("dwell", n=60, fs=8.0)
    r = R.check_measured(t, v, i, s[:30])
    yield "log.slow", [plain(r), r.n_violations, r.endurance_penalty_s]
    for fs in (20.0, 25.0, 2.0):
        t, v, i, s = _log("dwell", n=40, fs=fs)
        r = R.check_measured(t, v, i, s)
        yield f"log.fs{fs}", [plain(r), r.n_violations, r.endurance_penalty_s]
    t, v, i, s = _log("regen")
    for n in (8, 9):
        tt, vv, ii, ss = t[200:200 + n], v[200:200 + n], i[200:200 + n], s[200:200 + n]
        r = R.check_measured(tt, vv, ii, ss)
        yield f"log.n{n}", [plain(r), r.n_violations, r.endurance_penalty_s]
        r = R.check_measured(t, v, i, s[195:195 + n])
        yield f"log.speed{n}", [plain(r), r.n_violations, r.endurance_penalty_s]
    ii = np.where(np.arange(20) % 2 == 0, 200.0, 0.0)
    r = R.check_measured(np.zeros(20), np.full(20, 520.0), ii)
    yield "log.flat_time_spiky", [plain(r), r.n_violations, r.endurance_penalty_s]
    r = R.check_measured(t[:5], v[:5], i[:5])
    yield "log.short", [plain(r), r.n_violations, r.endurance_penalty_s]
    r = R.check_measured(np.zeros(20), np.full(20, 500.0), np.full(20, 10.0))
    yield "log.flat_time", [plain(r), r.n_violations, r.endurance_penalty_s]
    m = np.array([0, 1, 1, 1, 0, 0, 1, 1, 0, 1], bool)
    tt = np.arange(10) * 0.05
    yield "runs", [plain(R._runs_over(np.array([1, 0, 1, 1, 0], bool), tt[:5], 0.0)), plain(R._runs_over(m, tt, 0.0)), plain(R._runs_over(m, tt, 0.1)),
                   plain(R._runs_over(m[:4], tt[:4], 0.0)), plain(R._runs_over(np.ones(5, bool), tt[:5], 0.0)),
                   plain(R._runs_over(np.zeros(0, bool), tt[:0], 0.0))]
    y = np.array([1.0, 2.0, 6.0, 3.0, 0.0, 9.0])
    yield "mavg", [plain(R._moving_average(y, w)) for w in (1, 2, 3, 6, 10)]


def test_every_rule_limit_and_log():
    check_lazy("rules_fsae", _cases())
