"""Prediction ledger protocol v2 (suspension/ledger_v2.py): v1 physics plus five fixed rules, fitted on one
competition and applied to another.

These pin the rules as declared: implausible or self-contradicting published data is set aside, metric tyre sizes
are read, the skidpad radius follows the car's track, the factors are medians over clean runs, the team record is
shrunk halfway and applied only to the declared events, sealed records detect changes and reproduce, and the frozen
calibration and cross-year records in the repo still verify."""
import json
import math
from dataclasses import replace
from datetime import datetime, UTC
from pathlib import Path

import pytest

from suspension import ledger as L
from suspension import ledger_v2 as V

REPO = Path(__file__).resolve().parents[2]
SPEC = L.CarSpec(event="T", car="1", team="Team", year=2026, source_url="https://x/tid/77/", wheelbase_mm=1530,
                 track_f_mm=1220, track_r_mm=1200, weight_f_kg=120, weight_r_kg=130,
                 tyres="Hoosier 16.0x7.5-10 R20", n_motors=4, power_per_motor_kw=35, max_rpm=20000,
                 ratio_primary=12.4)


def test_metric_diameter_sizes_and_fallback():
    assert V.tyre_radius_m("Continental 175 / 518 R13") == pytest.approx(0.259)
    assert V.tyre_radius_m("175/518R13") == pytest.approx(0.259)
    assert V.tyre_radius_m("Hoosier 16.0x7.5-10") == pytest.approx(L.tyre_radius_m("Hoosier 16.0x7.5-10"))
    assert V.tyre_radius_m("Pirelli 185/040 R13") == L.tyre_radius_m("Pirelli 185/040 R13")   # 40 is no diameter
    assert V.tyre_radius_m(None) is None


@pytest.mark.parametrize("text, group", [("Hoosier R20", "R20"), ("hoosier lc0", "LC0"), ("Continental C19", "other"),
                                         ("R20 or LC0", "other"), ("", "other"), (None, "other")])
def test_compound_groups(text, group):
    assert V.compound(text) == group


def test_team_id_from_url_else_name():
    assert V.team_id(SPEC) == "77"
    assert V.team_id(replace(SPEC, source_url="")) == "Team"


def test_sanitize_keeps_good_data():
    s, rej = V.sanitize(SPEC)
    assert s == SPEC and rej == {}


def test_sanitize_rejects_out_of_range_values():
    s, rej = V.sanitize(replace(SPEC, wheelbase_mm=155, track_f_mm=124))
    assert rej == {"wheelbase_mm": 155, "track_f_mm": 124}
    assert s.wheelbase_mm is None and s.track_f_mm is None and s.track_r_mm == 1200


def test_sanitize_rejects_redline_cap_below_threshold():
    s, rej = V.sanitize(replace(SPEC, max_rpm=11600, ratio_primary=12.83))
    assert rej["redline_speed_ms"] < V.MIN_REDLINE_SPEED_MS
    assert s.max_rpm is None and s.ratio_primary is None and s.ratio_secondary is None


def test_sanitize_redline_check_without_tyre_size_uses_default_radius():
    _s, rej = V.sanitize(replace(SPEC, tyres="slicks", max_rpm=8000))
    assert rej["redline_speed_ms"] == pytest.approx(8000 * 2 * math.pi / 60 * 0.20 * L.LOADED_RADIUS_FACTOR / 12.4,
                                                    abs=0.01)


def test_sanitize_hands_metric_size_on_in_inch_form():
    s, _ = V.sanitize(replace(SPEC, tyres="175 / 518 R13"))
    assert L.tyre_radius_m(s.tyres) == pytest.approx(0.259, abs=1e-4)


def test_physics_radius_follows_track():
    p = V.physics(SPEC)
    assert p["inputs"]["skidpad_radius"]["value"] == pytest.approx(7.625 + 0.25 + 1.21 / 2)
    wide = V.physics(replace(SPEC, track_f_mm=1400, track_r_mm=1400))
    assert wide["inputs"]["skidpad_radius"]["value"] > p["inputs"]["skidpad_radius"]["value"]
    assert p["team_id"] == "77" and p["compound"] == "R20" and p["rejected"] == {}
    assert p["skidpad_s"] > 0 and p["accel_s"] > 0


def _row(car, team_id, compound, sk, ac):
    return {"car": car, "team": f"T{car}", "team_id": team_id, "compound": compound, "skidpad_s": sk,
            "accel_s": ac, "inputs": {}, "rejected": {}, "warnings": []}


def _res(sk=None, ac=None, left=None, right=None, pen=0.0):
    out = {}
    if sk is not None:
        out["skidpad"] = {"best_s": sk, "left_s": left, "right_s": right, "penalty_s": pen}
    if ac is not None:
        out["accel"] = {"best_s": ac, "penalty_s": 0.0}
    return out


def test_fit_medians_groups_and_pooling():
    phys = [_row(str(i), str(i), "R20", 4.0, 3.0) for i in range(4)] + [_row("9", "9", "LC0", 4.0, 3.0)]
    res = {str(i): _res(sk=4.0 * (1.1 + 0.01 * i), ac=3.0 * 1.05) for i in range(4)}
    res["9"] = _res(sk=4.0 * 1.5, ac=None)
    res["8"] = _res(sk=1.0)                                      # result for a car with no prediction: ignored
    c = V.fit(phys, res, "train")
    assert c["skidpad"]["R20"] == pytest.approx(1.115)
    assert c["skidpad_pooled"] == pytest.approx(1.12)
    assert c["skidpad"]["LC0"] == c["skidpad"]["other"] == c["skidpad_pooled"]   # fewer than MIN_GROUP cars
    assert c["accel"] == pytest.approx(1.05)
    assert c["n"] == {"skidpad": 5, "accel": 4, "skidpad_R20": 4, "skidpad_LC0": 1, "skidpad_other": 0}
    assert c["source"] == "train" and c["accel_quantile"] == V.ACCEL_QUANTILE


def test_quantile_and_challenger_accel_factor():
    assert V._quantile([3.0, 1.0, 2.0], 0.5) == 2.0
    assert V._quantile([1.0, 2.0, 3.0, 4.0], 0.25) == pytest.approx(1.75)       # numpy's default (linear)
    assert V._quantile([5.0], 0.25) == 5.0 and V._quantile([1.0, 3.0], 1.0) == 3.0
    phys = [_row(str(i), str(i), "R20", 4.0, 1.0) for i in range(5)]
    res = {str(i): _res(ac=1.0 + 0.1 * i) for i in range(5)}
    assert V.fit(phys, res, "x")["accel"] == pytest.approx(1.2)
    ch = V.fit(phys, res, "x", accel_quantile=V.CHALLENGER_ACCEL_QUANTILE)
    assert ch["accel"] == pytest.approx(1.1) and ch["accel_quantile"] == 0.25


def test_fit_excludes_penalized_runs_and_handles_no_data():
    phys = [_row("1", "1", "R20", 4.0, 3.0)]
    c = V.fit(phys, {"1": _res(sk=5.0, pen=0.2)}, "x")
    assert c["n"]["skidpad"] == 0 and c["skidpad_pooled"] == 1.0 and c["accel"] == 1.0
    assert V.fit([_row("1", "1", "R20", 0.0, 3.0)], {"1": _res(sk=5.0)}, "x")["n"]["skidpad"] == 0


def test_team_record_and_predict_shrinkage():
    calib = {"skidpad": {"R20": 1.1, "LC0": 1.1, "other": 1.1}, "skidpad_pooled": 1.1, "accel": 1.0}
    p = _row("1", "A", "R20", 4.0, 3.0)
    rec = V.team_record([p], {"1": _res(sk=4.4 * 1.04, ac=3.3)}, calib)
    assert rec["A"]["skidpad"] == pytest.approx(1.04) and rec["A"]["accel"] == pytest.approx(1.1)
    out = V.predict(p, calib, rec)
    assert out["skidpad_s"] == pytest.approx(4.4 * 1.02, abs=1e-4)          # shrunk halfway to 1
    assert out["accel_s"] == pytest.approx(3.0)                              # accel record not used by default
    assert set(out["team_record"]) == {"skidpad"}
    both = V.predict(p, calib, rec, ("skidpad", "accel"))
    assert both["accel_s"] == pytest.approx(3.15, abs=1e-4)
    assert V.predict(p, calib, None)["skidpad_s"] == pytest.approx(4.4)
    assert V.predict(_row("2", "B", "R20", 4.0, 3.0), calib, rec)["team_record"] == {}


def test_calibrated_unknown_group_uses_pooled():
    calib = {"skidpad": {"R20": 2.0}, "skidpad_pooled": 1.5, "accel": 1.0}
    assert V.calibrated(_row("1", "1", "other", 4.0, 3.0), calib)["skidpad_s"] == pytest.approx(6.0)


CALIB = {"skidpad": {"R20": 1.12, "LC0": 1.1, "other": 1.15}, "skidpad_pooled": 1.12, "accel": 1.05,
         "n": {}, "source": "test"}


def test_seal_verify_roundtrip_and_tamper():
    rec = V.seal([SPEC], "E", "blind", CALIB, {"77": {"skidpad": 1.02}}, now=datetime(2027, 1, 1, tzinfo=UTC))
    assert rec["sealed_utc"] == "2027-01-01T00:00:00Z" and rec["team_record_events"] == ["skidpad"]
    assert V.verify(rec) == {"intact": True, "same_code": True, "reproduced": True}
    bad = json.loads(json.dumps(rec))
    bad["predictions"][0]["skidpad_s"] += 0.1
    v = V.verify(bad)
    assert not v["intact"] and not v["reproduced"]
    with pytest.raises(ValueError):
        V.seal([SPEC], "E", "guess", CALIB, None)


def test_verify_old_record_without_events_field():
    rec = V.seal([SPEC], "E", "hindcast", CALIB, None)
    rec.pop("team_record_events")
    assert V.verify(rec)["reproduced"]


def test_best_lap_summary():
    rec = {"predictions": [{"car": "1", "skidpad_s": 5.0}, {"car": "2", "skidpad_s": 5.0},
                           {"car": "3", "skidpad_s": 5.0}, {"car": "4", "skidpad_s": 5.0}]}
    res = {"1": _res(sk=5.1, left=5.0, right=5.2), "2": _res(sk=5.0, left=None, right=5.0),
           "3": _res(sk=6.0, left=5.0, right=7.0)}                 # 3: L/R split > 10 % -> excluded
    s = V.best_lap_summary(rec, res)
    assert s == {"n": 1, "mae_pct": 0.0, "bias_pct": 0.0, "within_5pct": 1}
    assert V.best_lap_summary(rec, {}) == {}


def test_repo_frozen_calibration_and_cross_year_records():
    d = REPO / "benchmarks" / "ledger" / "protocol-v2"
    import hashlib
    for f, q in (("CALIBRATION.json", V.ACCEL_QUANTILE), ("CALIBRATION_challenger.json", V.CHALLENGER_ACCEL_QUANTILE)):
        cal = json.loads((d / f).read_text(encoding="utf-8"))
        body = {k: v for k, v in cal.items() if k != "sha256"}
        assert hashlib.sha256(L._canon(body)).hexdigest() == cal["sha256"]
        assert cal["protocol_version"] == V.PROTOCOL_VERSION and cal["calibration"]["accel_quantile"] == q
    for f in ("fsg2026_from2025.SEALED.json", "fsg2025_from2026.SEALED.json"):
        rec = json.loads((d / f).read_text(encoding="utf-8"))
        v = V.verify(rec)
        assert v["intact"]
        if v["same_code"]:
            assert v["reproduced"]
