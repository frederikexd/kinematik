"""Characterization suite for suspension/ledger_v2.py (mutation gate); see tests/_golden.py. Tyre sizes and compound
groups, team ids, the data checks (every implausible field, a redline cap below the floor, a size only v2 reads),
physics rows, the calibration fit with groups above and below MIN_GROUP, quantiles, team records, predictions with
and without a record, a sealed record and its verification (tampered too), and the best-lap summary. Hashes of the
code are left out: under mutation the code is the mutant."""
from dataclasses import replace
from datetime import UTC, datetime

import pytest

from suspension import ledger as L
from suspension import ledger_v2 as V
from tests._golden import plain
from tests._golden_lazy import check_lazy

BASE = L.CarSpec(event="T", car="1", team="Team", year=2026, source_url="https://x/tid/123/", wheelbase_mm=1530,
                 track_f_mm=1220, track_r_mm=1200, weight_f_kg=120, weight_r_kg=130, tyres="Hoosier 16.0x7.5-10 R20",
                 n_motors=4, power_per_motor_kw=35, max_rpm=20000, ratio_primary=12.4)
SPECS = [BASE,
         replace(BASE, car="2", team="B", source_url="u", tyres="Hoosier 16x7.5-10 LC0", weight_f_kg=140),
         replace(BASE, car="3", team="C", source_url="u", tyres="Continental 175 / 518 R13", n_motors=1,
                 power_per_motor_kw=80, max_rpm=3000, ratio_primary=12.4),
         replace(BASE, car="4", team="D", source_url="u/tid/9", tyres="R20 LC0 mix", wheelbase_mm=1400, track_f_mm=2000,
                 weight_r_kg=10, power_per_motor_kw=200, max_rpm=1000, ratio_primary=40),
         replace(BASE, car="5", team="E", source_url="", tyres="", ratio_secondary=1.2)]


def _sk(best, left=None, right=None, pen=0.0):
    return {"best_s": best, "left_s": left, "right_s": right, "penalty_s": pen}


def _strip(rec):
    return {k: v for k, v in rec.items() if k not in ("code_sha256", "record_sha256")}


def _cases():
    yield "tyre", [V.tyre_radius_m(t) for t in ("175/518 R13", "175 / 518R13", "200/700 R13", "205/399 R10",
                                                 "Hoosier 16.0x7.5-10", "", None, "nonsense", "185/650 13")]
    yield "compound", [V.compound(t) for t in ("Hoosier R20", "lc0", "R20 LC0", "", None, "R25B")]
    yield "team", [V.team_id(s) for s in SPECS]
    yield "sanitize", [plain(V.sanitize(s)) for s in SPECS]
    yield "sanitize.edges", [plain(V.sanitize(replace(BASE, **{f: v})))
                             for f, (lo, hi) in V.PLAUSIBLE.items() for v in (lo, hi, lo - 1.0, hi + 1.0)]
    phys = [V.physics(s) for s in SPECS]
    yield "physics", plain(phys)
    results = {"1": {"skidpad": _sk(4.9, 4.95, 4.85), "accel": _sk(3.7)},
               "2": {"skidpad": _sk(5.1, 5.0, 5.2), "accel": _sk(3.9, pen=2.0)},
               "3": {"skidpad": _sk(None), "accel": _sk(4.4)},
               "4": {"skidpad": _sk(5.3, 5.2, 6.4)},
               "5": {"skidpad": _sk(5.0, 5.05, None), "accel": _sk(3.8)}}
    for name, ph, res in (("all", phys, results), ("none", phys, {}), ("r20x4", [dict(phys[0], car=c) for c in "abcd"],
                          {c: {"skidpad": _sk(4.8 + 0.1 * i)} for i, c in enumerate("abcd")}),
                          ("lc0x4", [dict(phys[1], car=c) for c in "abcdef"],
                           {c: {"skidpad": _sk(5.0 + 0.1 * i), "accel": _sk(3.6 + 0.05 * i)} for i, c in enumerate("abcdef")})):
        for q in (0.5, 0.25):
            yield f"fit.{name}.{q}", plain(V.fit(ph, res, "train", accel_quantile=q))
    yield "fit.default", plain(V.fit(phys, results, "t"))
    yield "quantile", [V._quantile(xs, q) for xs in ([1.0], [3.0, 1.0], [1.0, 2.0, 4.0, 8.0]) for q in (0.0, 0.25, 0.5, 1.0)]
    calib = V.fit(phys, results, "train")
    yield "calibrated", [plain(V.calibrated(p, calib)) for p in phys]
    yield "calibrated.missing_group", plain(V.calibrated(dict(phys[0], compound="weird"), calib))
    grouped = dict(calib, skidpad={phys[0]["compound"]: 0.5})
    yield "calibrated.group", plain(V.calibrated(phys[0], grouped))
    yield "tyre.bounds", [V.tyre_radius_m(t) for t in ("205/400 R13", "205/401 R13", "205/650 R13", "205/651 R13")]
    slow = V.sanitize(replace(BASE, max_rpm=100, ratio_primary=3.0, ratio_secondary=4.0))
    yield "sanitize.slow", plain(slow)
    yield "physics.radius", plain(V.physics(replace(BASE, track_f_mm=1213.37, track_r_mm=1213.37)))
    from datetime import timedelta
    live = V.seal(SPECS[:1], "e", "blind", calib, None)
    t = datetime.strptime(live["sealed_utc"], "%Y-%m-%dT%H:%M:%SZ").replace(tzinfo=UTC)
    yield "seal.now_is_utc", abs(datetime.now(UTC) - t) < timedelta(minutes=5)
    rec_ev = V.seal(SPECS[:2], "e", "blind", calib, {"123": {"skidpad": 0.9, "accel": 1.2}},
                    now=datetime(2026, 1, 1, tzinfo=UTC), record_events=("accel",))
    yield "seal.events", [plain(_strip(rec_ev))["predictions"], V.verify(rec_ev)]
    no_ev = {k: v for k, v in rec_ev.items() if k != "team_record_events"}
    yield "verify.no_events_key", V.verify(no_ev)
    yield "verify.bare", V.verify({k: v for k, v in rec_ev.items() if k not in ("specs", "predictions")})
    record = V.team_record(phys, results, calib)
    yield "record", plain(record)
    yield "predict", [plain(V.predict(p, calib)) for p in phys] + [plain(V.predict(p, calib, record)) for p in phys] + [
        plain(V.predict(phys[0], calib, record, record_events=("skidpad", "accel"))),
        plain(V.predict(phys[0], calib, {"123": {"accel": 1.1}})), plain(V.predict(phys[0], calib, {"123": None}))]
    rec = V.seal(SPECS[:3], "FSG 2026", "blind", calib, record, note="n", now=datetime(2026, 8, 1, 12, 0, tzinfo=UTC))
    yield "seal", plain(_strip(rec))
    yield "seal.hashes", [sorted(rec["code_sha256"]), len(rec["record_sha256"])]
    yield "verify", V.verify(rec)
    bad = dict(rec, note="changed")
    yield "verify.tampered", V.verify(bad)
    moved = dict(rec, predictions=[dict(p, skidpad_s=p["skidpad_s"] + 0.01) for p in rec["predictions"]])
    yield "verify.moved", V.verify(moved)
    yield "seal.hindcast", plain(_strip(V.seal([], "e", "hindcast", calib, None)))["predictions"]
    with pytest.raises(ValueError) as e:
        V.seal([], "e", "other", calib, None)
    yield "seal.badkind", str(e.value)
    yield "best_lap", [plain(V.best_lap_summary(rec, results)), V.best_lap_summary(rec, {}),
                       plain(V.best_lap_summary(rec, {"1": {"skidpad": _sk(4.9, 4.7, 5.5)}, "2": {"skidpad": _sk(5.1, 5.0, 5.2, pen=1.0)},
                                                      "3": {"skidpad": _sk(4.0, 4.1, 4.0)}}))]
    yield "consts", [V.PROTOCOL_VERSION, V.PROTOCOL_TEXT, V._CODE_FILES, V.SKIDPAD_INNER_RADIUS_M, V.SKIDPAD_CLEARANCE_M,
                     V.MIN_REDLINE_SPEED_MS, V.MIN_GROUP, V.PRIOR_SHRINK, V.ACCEL_QUANTILE, V.CHALLENGER_ACCEL_QUANTILE,
                     V.TEAM_RECORD_EVENTS]


def test_every_ledger_v2_step():
    check_lazy("ledger_v2", _cases())
