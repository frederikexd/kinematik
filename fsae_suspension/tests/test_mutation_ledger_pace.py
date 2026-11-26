"""Characterization suite for suspension/ledger_pace.py (mutation gate); see tests/_golden.py. The autocross lap of
specs that differ in mass, power and grip, and the leave-one-out pace score with every exclusion, exactly three
cars, too few, ties (no rank spread), errors either side of 5 %, and an exact 5 % miss."""
from dataclasses import replace

from suspension import ledger as L
from suspension import ledger_pace as P
from tests._golden import plain
from tests._golden_lazy import check_lazy

SPEC = L.CarSpec(event="T", car="1", team="Team", year=2026, source_url="u", wheelbase_mm=1530, track_f_mm=1220,
                 track_r_mm=1200, weight_f_kg=120, weight_r_kg=130, tyres="Hoosier 16.0x7.5-10 R20", n_motors=4,
                 power_per_motor_kw=35, max_rpm=20000, ratio_primary=12.4)


def _res(t, pen=0.0):
    return {"autocross": {"best_s": t, "left_s": None, "right_s": None, "penalty_s": pen}}


def _cases():
    yield "lap.base", P.autocross_lap_s(SPEC)
    yield "lap.heavy", P.autocross_lap_s(replace(SPEC, weight_f_kg=170, weight_r_kg=180))
    yield "lap.weak", P.autocross_lap_s(replace(SPEC, n_motors=1, power_per_motor_kw=60))
    lap = {"a": 40.0, "b": 50.0, "c": 60.0, "d": 45.0, "x": 40.0, "y": 40.0, "z": 41.0, "w": 42.0}
    res = {"a": _res(80.0), "b": _res(100.0), "c": _res(150.0), "x": _res(81.0, pen=2.0), "y": _res(None),
           "z": {}, "w": {"skidpad": {"best_s": 5.0}}}
    yield "score.mixed", plain(P.score_pace(lap, res))
    yield "score.three", plain(P.score_pace({"a": 40.0, "b": 50.0, "c": 60.0},
                                            {"a": _res(80.0), "b": _res(100.0), "c": _res(121.0)}))
    yield "score.two", plain(P.score_pace({"a": 40.0, "b": 50.0}, {"a": _res(80.0), "b": _res(90.0)}))
    yield "score.flat", plain(P.score_pace({"a": 40.0, "b": 40.0, "c": 40.0}, {c: _res(80.0) for c in "abc"}))
    yield "score.reversed", plain(P.score_pace({"a": 40.0, "b": 50.0, "c": 60.0, "d": 70.0},
                                               {"a": _res(140.0), "b": _res(120.0), "c": _res(100.0), "d": _res(80.0)}))
    # car b predicted at exactly 105 % of its time: on the 5 % line
    yield "score.edge", plain(P.score_pace({"a": 50.0, "b": 52.5, "c": 50.0, "d": 50.0},
                                           {"a": _res(100.0), "b": _res(100.0), "c": _res(100.0), "d": _res(100.0)}))
    yield "score.empty", plain(P.score_pace({}, {}))
    yield "score.precise", plain(P.score_pace({"a": 40.123456, "b": 50.987654, "c": 61.555555},
                                              {"a": _res(80.4321), "b": _res(101.2468), "c": _res(119.8765)}))
    yield "summary", plain(P._summary([1.0, -2.0, 5.0, -5.0, 5.5, 0.0]))


def test_every_pace_score():
    check_lazy("ledger_pace", _cases())
