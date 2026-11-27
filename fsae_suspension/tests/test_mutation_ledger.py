"""Characterization suite for suspension/ledger.py (mutation gate); see tests/_golden.py. Tyre sizes in every
spelling, the ratio rule, specs from complete to bare (every published-or-assumed branch, the 2012 power cap, AWD
and RWD), predictions, the CSV loaders (NA, blanks, commas), a sealed record and its verification (tampered too),
ranks with ties, Spearman with no spread, every exclusion, the scoring summary (odd and even counts) and the
Markdown scorecard. Hashes of the code are left out: under mutation the code is the mutant."""
import math
from dataclasses import replace
from datetime import UTC, datetime

import pytest

from suspension import ledger as L
from tests._golden import plain
from tests._golden_lazy import check_lazy

FULL = L.CarSpec(event="T", car="1", team="A", year=2026, source_url="u", wheelbase_mm=1530, track_f_mm=1220,
                 track_r_mm=1200, weight_f_kg=120, weight_r_kg=130, tyres="Hoosier 16.0x7.5-10 R20", n_motors=4,
                 power_per_motor_kw=35, max_rpm=20000, ratio_primary=12.4)
SPECS = [FULL, replace(FULL, car="2", team="B", year=2012, n_motors=2, power_per_motor_kw=50, tyres="185/40 R13",
                       ratio_primary=3.0, ratio_secondary=4.0),
         replace(FULL, car="3", team="C", weight_f_kg=None, wheelbase_mm=None, track_f_mm=None, track_r_mm=None,
                 tyres="", n_motors=None, power_per_motor_kw=None, max_rpm=None, ratio_primary=None),
         replace(FULL, car="4", team="D", weight_r_kg=0, n_motors=1, power_per_motor_kw=200, tyres="7.5x18 x",
                 ratio_primary=4.0)]


def _strip(rec):
    return {k: v for k, v in rec.items() if k not in ("code_sha256", "record_sha256")}


def _sk(best, left=None, right=None, pen=0.0):
    return {"best_s": best, "left_s": left, "right_s": right, "penalty_s": pen}


def _cases(tmp):
    yield "tyre", [L.tyre_radius_m(t) for t in ("16.0x7.5-10", "18x6 R10", "20.0x7.0-13", "7.5x16", "185/40 R13",
                                                 "205/45ZR13", "185 / 40 R 13", "16,0x7,5-10", "10x7", "25x9", "",
                                                 None, "R20")]
    yield "ratio", [L.overall_ratio(p, s) for p, s in ((None, 2.0), (0.0, 2.0), (-1.0, 2.0), (12.4, None), (4.9, 3.0),
                                                       (5.0, 3.0), (4.0, None), (4.0, 0.0), (4.0, -1.0), (2.5, 2.0))]
    for s in SPECS:
        veh, pt, inputs = L.build(s)
        yield f"build.{s.car}", [inputs, veh.p.mass, veh.p.weight_dist_front, pt.power_kw, pt.drive,
                                 pt.motor_map is None]
    yield "tyre.edges", [L.tyre_radius_m(t) for t in ("16.0X7.5-10", "15x6", "15.5x6", "24x8", "14.9x6", "24.1x8")]
    yield "ratio.edges", [L.overall_ratio(0.5, None), L.overall_ratio(4.0, 0.5), L.overall_ratio(0.5, 3.0)]
    for name, kw in (("half", dict(weight_f_kg=0.5, weight_r_kg=0.5)), ("geo", dict(wheelbase_mm=1600, track_f_mm=1250,
                                                                                    track_r_mm=1180)),
                     ("motors_only", dict(power_per_motor_kw=None)), ("ratio_only", dict(max_rpm=None)),
                     ("none_w", dict(weight_f_kg=None, weight_r_kg=None)), ("power_only", dict(n_motors=None))):
        veh, pt, inputs = L.build(replace(FULL, **kw))
        yield f"build.edge.{name}", [inputs, veh.p.mass, veh.p.weight_dist_front, veh.p.cg_height, veh.p.wheelbase,
                                     veh.p.track_front, veh.p.track_rear, pt.power_kw, pt.drive, pt.motor_map is None]
    import os
    fp = tmp / "fp"
    fp.mkdir()
    for f in L._CODE_FILES:
        (fp / f).write_text(f)
    yield "fingerprint.root", [L.code_fingerprint(fp), L.code_fingerprint(str(fp)) == L.code_fingerprint(fp),
                               L.code_fingerprint() != L.code_fingerprint(fp), os.path.isdir(fp)]
    preds = [L.predict(s) for s in SPECS]
    yield "predict", preds
    yield "num", [L._num(v) for v in (None, "", " ", "NA", "na", "1,5", " 2.25 ")]
    sp = tmp / "specs.csv"
    sp.write_text("event,car,team,year,source_url,wheelbase_mm,track_f_mm,track_r_mm,weight_f_kg,weight_r_kg,tyres,"
                  "n_motors,power_per_motor_kw,max_rpm,ratio_primary,ratio_secondary\n"
                  " FSG , 7 , T , 2026 , u ,1530,NA,,120,130, Hoosier 16x7.5-10 ,4,35,20000,12.4,\n"
                  "FSG,8,U,2025,v,1600,1200,1180,1,1,,NA,,,,\n", encoding="utf-8")
    yield "load_specs", plain(L.load_specs_csv(sp))
    rp = tmp / "results.csv"
    rp.write_text("car,event,best_s,left_s,right_s,penalty_s,source_url\n 1 , skidpad ,4.9,4.95,4.85,, s \n"
                  "1,accel,3.7,,,0,\n2,skidpad,NA,,,2,\n", encoding="utf-8")
    yield "load_results", L.load_results_csv(rp)
    rp2 = tmp / "results2.csv"
    rp2.write_text("car,event,best_s\n3,accel,4.1\n", encoding="utf-8")
    yield "load_results.min", L.load_results_csv(rp2)
    rec = L.seal(SPECS, "FSG 2026", "blind", note="n", now=datetime(2026, 8, 1, 9, 30, tzinfo=UTC))
    yield "seal", plain(_strip(rec))
    yield "seal.hashes", [sorted(rec["code_sha256"]), len(rec["record_sha256"]), sorted(L.code_fingerprint())]
    bare = {k: v for k, v in rec.items() if k not in ("specs", "predictions")}
    yield "verify.bare", L.verify(bare)
    from datetime import datetime as _dt, timedelta
    live = L.seal(SPECS[:1], "e", "blind")
    t = _dt.strptime(live["sealed_utc"], "%Y-%m-%dT%H:%M:%SZ").replace(tzinfo=UTC)
    yield "seal.now_is_utc", abs(_dt.now(UTC) - t) < timedelta(minutes=5)
    yield "verify", [L.verify(rec), L.verify(dict(rec, note="x")),
                     L.verify(dict(rec, predictions=[dict(p, accel_s=p["accel_s"] + 1) for p in rec["predictions"]])),
                     L.verify(dict(rec, code_sha256={}))]
    with pytest.raises(ValueError) as e:
        L.seal([], "e", "x")
    yield "seal.badkind", str(e.value)
    yield "canon", L._canon({"b": 1, "a": [2, "é"]}).decode()
    yield "ranks", [L._ranks(x) for x in ([3, 1, 2], [1, 1, 2], [2, 2, 2], [5, 1, 5, 3, 1], [])]
    yield "spearman.ties", [L.spearman([1, 1, 1], [1, 2, 3]), L.spearman([1, 2, 3], [5, 5, 5]),
                            L.spearman([1, 1, 2, 3], [4, 3, 2, 2]), L.spearman([3, 1, 2, 2, 5], [1, 2, 2, 4, 3])]
    yield "spearman", [L.spearman(a, b) for a, b in (([1, 2, 3], [1, 2, 3]), ([1, 2, 3], [3, 2, 1]), ([1, 2], [1, 2]),
                                                     ([1, 2, 3], [1, 2]), ([1, 1, 1], [1, 2, 3]),
                                                     ([1, 2, 3, 4], [2, 1, 4, 3]), ([1, 2, 3], [5, 5, 5]))]
    yield "excluded", [L._excluded(ev, r) for ev, r in (
        ("skidpad", _sk(None)), ("accel", _sk(3.0, pen=1.0)), ("skidpad", _sk(5.0, 5.0, 5.5)),
        ("skidpad", _sk(5.0, 5.0, 5.51)), ("skidpad", _sk(5.0, 5.0, None)), ("accel", _sk(3.0, 3.0, 9.0)),
        ("skidpad", {"best_s": 5.0}), ("skidpad", _sk(5.0, 5.0, 5.0)))]
    results = {"1": {"skidpad": _sk(4.9, 4.95, 4.85), "accel": _sk(3.7)},
               "2": {"skidpad": _sk(5.2, 5.1, 6.0), "accel": _sk(None)},
               "3": {"skidpad": _sk(5.5, 5.5, 5.4), "accel": _sk(4.2, pen=2.0)},
               "4": {"skidpad": _sk(5.1, 5.1, 5.1), "accel": _sk(4.0)}}
    for name, res in (("all", results), ("none", {}), ("odd", {c: results[c] for c in "134"}),
                      ("one", {"1": results["1"]})):
        sc = L.score(dict(rec, record_sha256="R"), res)
        yield f"score.{name}", plain(sc)
        yield f"card.{name}", L.scorecard_markdown(sc)
    hind = L.seal(SPECS[:1], "old", "hindcast", now=datetime(2020, 1, 1, tzinfo=UTC))
    sc = L.score(dict(hind, record_sha256="H"), {"1": {"skidpad": _sk(4.0), "accel": _sk(0.0)}})
    yield "card.hindcast", [plain(sc), L.scorecard_markdown(sc)]
    yield "consts", [L.PROTOCOL_VERSION, L.ACCEL_DISTANCE_M, L.LOADED_RADIUS_FACTOR, L.POWER_CAP_KW_BY_YEAR,
                     L.POWER_CAP_KW_DEFAULT, L.TRACTION_TORQUE_MARGIN_G, L.EXCLUDE_LR_SPLIT, L.EXCLUDE_PENALTY,
                     L.PROTOCOL_TEXT, L._CODE_FILES, L._NUM, math.isnan(L.spearman([], []))]


def test_every_ledger_step(tmp_path):
    check_lazy("ledger", _cases(tmp_path))
