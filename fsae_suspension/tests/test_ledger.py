"""The prediction ledger (suspension/ledger.py): public spec -> KinematiK prediction -> sealed record -> scorecard.

These pin what makes a ledger entry trustworthy: the protocol fills unknowns the same way for every car, a sealed
record detects any change to it, today's code reproduces it, the scoring exclusions are the declared ones, and the
FSG 2026 and DUT12 records in the repo still verify."""
import json
import math
from datetime import datetime, UTC
from pathlib import Path

import pytest

from suspension import ledger as L

REPO = Path(__file__).resolve().parents[2]
SPEC = L.CarSpec(event="T", car="1", team="Team", year=2026, source_url="u", wheelbase_mm=1530, track_f_mm=1220,
                 track_r_mm=1200, weight_f_kg=120, weight_r_kg=130, tyres="Hoosier 16.0x7.5-10 R20", n_motors=4,
                 power_per_motor_kw=35, max_rpm=20000, ratio_primary=12.4)


@pytest.mark.parametrize("text, inches", [("Hoosier 16.0x7.5-10 R20", 16), ("18x6 R10 Hoosier LC0", 18),
                                          ("7.5x16 R10, Hoosier", 16), ("Goodyear Eagle G19 20.0x7.0-13", 20),
                                          ("16,0 x 7,5 - 10", 16)])
def test_tyre_radius_from_inch_sizes(text, inches):
    assert L.tyre_radius_m(text) == pytest.approx(inches * 25.4 / 2000)


def test_tyre_radius_metric_and_unknown():
    assert L.tyre_radius_m("Pirelli 185/40 R13 S") == pytest.approx((13 * 25.4 + 2 * 185 * 0.40) / 2000)
    assert L.tyre_radius_m("") is None and L.tyre_radius_m("slicks") is None and L.tyre_radius_m("6x7") is None


def test_overall_ratio_rule():
    assert L.overall_ratio(12.4, None) == 12.4
    assert L.overall_ratio(16, 3) == 16                       # primary is already the whole reduction
    assert L.overall_ratio(2.72, 4.35) == pytest.approx(11.832)
    assert L.overall_ratio(2.72, None) == 2.72
    assert L.overall_ratio(None, 4) is None and L.overall_ratio(0, 4) is None


def test_build_labels_every_input_and_applies_the_rules():
    _veh, pt, inp = L.build(SPEC)
    assert inp["mass"] == {"value": 250, "unit": "kg", "basis": "published"}
    assert inp["weight_dist_front"]["value"] == pytest.approx(0.48)
    assert inp["power"] == {"value": 80.0, "unit": "kW", "basis": "rule"}        # 4 x 35 capped by FS rules
    assert inp["cg_height"]["basis"] == inp["cla"]["basis"] == inp["motor_torque"]["basis"] == "assumed"
    assert pt.drive == "awd" and pt.motor_map is not None and math.isinf(pt.max_tractive_n)
    assert {v["basis"] for v in inp.values()} <= {"published", "rule", "assumed"}


def test_build_with_little_published_falls_back_to_defaults():
    bare = L.CarSpec(event="T", car="2", team="T", year=2012, source_url="u", n_motors=2, power_per_motor_kw=30)
    _veh, pt, inp = L.build(bare)
    assert inp["mass"]["basis"] == "assumed" and inp["wheelbase"]["basis"] == "assumed"
    assert inp["power"]["value"] == 60 and pt.drive == "rwd" and pt.motor_map is None   # no ratio/rpm: flat power
    assert inp["wheel_radius"] == {"value": 0.20, "unit": "m", "basis": "assumed"}
    nothing = L.CarSpec(event="T", car="3", team="T", year=2012, source_url="u")
    assert L.build(nothing)[2]["power"]["value"] == 85.0                         # 2012 rule cap


def test_predict_gives_plausible_fs_times():
    p = L.predict(SPEC)
    assert 4.0 < p["skidpad_s"] < 6.0 and 3.0 < p["accel_s"] < 4.5 and p["warnings"] == []
    heavier = L.predict(L.CarSpec(**{**SPEC.__dict__, "weight_f_kg": 160, "weight_r_kg": 170}))
    assert heavier["accel_s"] > p["accel_s"]


def test_seal_is_tamper_evident_and_reproducible():
    rec = L.seal([SPEC], "T", "blind", "n", now=datetime(2027, 1, 1, tzinfo=UTC))
    assert rec["sealed_utc"] == "2027-01-01T00:00:00Z" and rec["protocol_version"] == L.PROTOCOL_VERSION
    assert L.verify(rec) == {"intact": True, "same_code": True, "reproduced": True}
    bad = json.loads(json.dumps(rec))
    bad["predictions"][0]["skidpad_s"] -= 0.1
    chk = L.verify(bad)
    assert not chk["intact"] and not chk["reproduced"]
    with pytest.raises(ValueError):
        L.seal([SPEC], "T", "maybe")


def test_score_exclusions_and_summary():
    rec = {"event": "T", "kind": "hindcast", "record_sha256": "x",
           "predictions": [{"car": c, "team": c, "skidpad_s": s, "accel_s": a}
                           for c, s, a in (("a", 5.0, 3.5), ("b", 5.2, 3.6), ("c", 5.4, 3.7), ("d", 5.0, 3.5),
                                           ("e", 5.0, 3.5))]}
    res = {"a": {"skidpad": {"best_s": 5.0, "left_s": 5.0, "right_s": 5.0, "penalty_s": 0.0},
                 "accel": {"best_s": 3.5, "penalty_s": 0.0}},
           "b": {"skidpad": {"best_s": 5.5, "left_s": 5.4, "right_s": 5.6, "penalty_s": 0.0},
                 "accel": {"best_s": 3.8, "penalty_s": 0.0}},
           "c": {"skidpad": {"best_s": 6.0, "left_s": 6.0, "right_s": 6.0, "penalty_s": 0.0},
                 "accel": {"best_s": 3.9, "penalty_s": 0.0}},
           "d": {"skidpad": {"best_s": 7.0, "left_s": 5.0, "right_s": 9.0, "penalty_s": 0.0}},
           "e": {"skidpad": {"best_s": 5.6, "left_s": 5.6, "right_s": 5.6, "penalty_s": 0.2},
                 "accel": {"best_s": None, "penalty_s": 0.0}}}
    sc = L.score(rec, res)
    why = {(r["car"], r["event"]): r["excluded"] for r in sc["rows"]}
    assert "differ by more than" in why[("d", "skidpad")] and "penalty" in why[("e", "skidpad")]
    assert why[("e", "accel")] == "no valid official time" and why[("a", "skidpad")] == ""
    s = sc["summary"]["skidpad"]
    assert s["n"] == 3 and s["excluded"] == 2 and s["within_5pct"] == 1
    assert s["bias_pct"] < 0 and s["spearman"] == pytest.approx(1.0)
    assert sc["summary"]["accel"]["n"] == 3 and sc["summary"]["accel"]["median_abs_pct"] == pytest.approx(
        100 * 0.2 / 3.9, rel=1e-6)
    md = L.scorecard_markdown(sc)
    assert "hindcast" in md and "| d |" in md and "Spearman" in md


def test_spearman_edge_cases():
    assert math.isnan(L.spearman([1, 2], [1, 2])) and math.isnan(L.spearman([1, 1, 1], [1, 2, 3]))
    assert L.spearman([1, 2, 3, 4], [4, 3, 2, 1]) == pytest.approx(-1.0)
    assert L.spearman([1, 2, 2, 3], [1, 2, 3, 4]) == pytest.approx(0.9486833, rel=1e-6)   # ties share a rank


def test_csv_loaders_round_trip(tmp_path):
    specs = L.load_specs_csv(REPO / "benchmarks/ledger/fsg2026/specs.csv")
    assert len(specs) == 25 and specs[0].car == "31" and specs[0].n_motors == 4 and specs[5].ratio_primary is None
    res = L.load_results_csv(REPO / "benchmarks/ledger/fsg2026/results.csv")
    assert res["31"]["skidpad"]["best_s"] == 4.678 and res["92"]["skidpad"]["best_s"] is None
    assert all(s.source_url.startswith("https://www.formulastudent.de/") for s in specs)


@pytest.mark.parametrize("case", ["fsg2026", "fsg2012-dut12"])
def test_repo_records_are_intact(case):
    rec = json.loads((REPO / "benchmarks/ledger" / case / "SEALED.json").read_text(encoding="utf-8"))
    chk = L.verify(rec)
    assert chk["intact"], "a sealed record in the repo was edited after sealing"
    assert rec["kind"] == "hindcast"
    if not chk["same_code"]:
        pytest.skip("the code has moved on from the sealed fingerprint; the stored SCORECARD.md is the record")
    assert chk["reproduced"]


def test_score_leaves_out_an_event_with_no_results():
    rec = {"event": "T", "kind": "blind", "record_sha256": "x",
           "predictions": [{"car": "a", "team": "a", "skidpad_s": 5.0, "accel_s": 3.5}]}
    sc = L.score(rec, {"a": {"skidpad": {"best_s": 5.1, "penalty_s": 0.0}}, "zz": {"accel": {"best_s": 3.0}}})
    assert set(sc["summary"]) == {"skidpad"} and "before the competition ran" in L.scorecard_markdown(sc)
