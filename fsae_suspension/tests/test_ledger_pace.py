"""Autocross relative pace (suspension/ledger_pace.py), test B of benchmarks/ledger/protocol-v2/REGISTRATION.md.

These pin the registered rules: the scale is leave-one-out (a car's own time never enters its prediction), the
baseline is the median of the other cars, penalized or missing runs are excluded, and the registered results in
the repo are what today's code produces."""
import json
import statistics
from pathlib import Path

import pytest

from suspension import ledger as L
from suspension import ledger_pace as P

REPO = Path(__file__).resolve().parents[2]
SPEC = L.CarSpec(event="T", car="1", team="Team", year=2026, source_url="u", wheelbase_mm=1530, track_f_mm=1220,
                 track_r_mm=1200, weight_f_kg=120, weight_r_kg=130, tyres="Hoosier 16.0x7.5-10 R20", n_motors=4,
                 power_per_motor_kw=35, max_rpm=20000, ratio_primary=12.4)


def _res(t, pen=0.0):
    return {"autocross": {"best_s": t, "left_s": None, "right_s": None, "penalty_s": pen}}


def test_lap_is_positive_and_heavier_car_is_slower():
    from dataclasses import replace
    light = P.autocross_lap_s(SPEC)
    heavy = P.autocross_lap_s(replace(SPEC, weight_f_kg=170, weight_r_kg=180))
    assert 10 < light < heavy


def test_leave_one_out_scale_and_baseline():
    lap = {"a": 40.0, "b": 50.0, "c": 60.0, "d": 45.0, "x": 40.0, "y": 40.0}
    res = {"a": _res(80.0), "b": _res(100.0), "c": _res(150.0), "x": _res(81.0, pen=2.0), "y": _res(None)}
    sc = P.score_pace(lap, res)
    rows = {r["car"]: r for r in sc["rows"]}
    assert rows["d"]["excluded"] == "no official result"
    assert rows["x"]["excluded"] == "best time includes a penalty"
    assert rows["y"]["excluded"] == "no valid official time"
    # car c: scale from a and b only (2.0 and 2.0), never from c's own 2.5
    assert rows["c"]["predicted_s"] == pytest.approx(120.0)
    assert rows["c"]["error_pct"] == pytest.approx(-20.0)
    assert rows["c"]["baseline_error_pct"] == pytest.approx(100 * (90.0 - 150.0) / 150.0)
    m = sc["model"]
    assert m["n"] == 3 and m["spearman"] == pytest.approx(1.0)
    assert m["median_abs_pct"] == statistics.median(abs(rows[c]["error_pct"]) for c in "abc")
    assert sc["baseline"]["n"] == 3


def test_too_few_cars_and_no_spread():
    assert P.score_pace({"a": 40.0, "b": 50.0}, {"a": _res(80.0), "b": _res(90.0)})["model"] == {}
    flat = P.score_pace({"a": 40.0, "b": 40.0, "c": 40.0}, {c: _res(80.0) for c in "abc"})
    assert flat["model"]["spearman"] is None and flat["model"]["mae_pct"] == pytest.approx(0.0)


def test_repo_registered_results_reproduce():
    d = REPO / "benchmarks" / "ledger"
    reg = json.loads((d / "protocol-v2" / "registered_results.json").read_text(encoding="utf-8"))
    for y, saved in reg["test_b"].items():
        specs = L.load_specs_csv(d / f"fsg{y}" / "specs.csv")
        sc = P.score_pace({s.car: P.autocross_lap_s(s) for s in specs},
                          L.load_results_csv(d / f"fsg{y}" / "autocross.csv"))
        assert sc["model"]["n"] == saved["model"]["n"]
        assert sc["model"]["mae_pct"] == pytest.approx(saved["model"]["mae_pct"], abs=1e-9)
    assert reg["challenger_adopted"] is False
