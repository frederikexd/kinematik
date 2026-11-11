"""A team's own car, blind (suspension/team_test.py): plan -> sealed predictions -> measured -> scorecard.

These pin what makes the test blind and fair: at least two setups, unknown inputs refused, private files enter
only as hashes, the record detects any change and reproduces, A vs B rows judge direction with ties left out,
and nothing is refitted."""
import json
from datetime import datetime, UTC
from pathlib import Path

import pytest

from suspension import team_test as T
from suspension.tiremodel import _DEFAULT_FSAE_COEFFS, _DEFAULT_FSAE_FNOMIN

REPO = Path(__file__).resolve().parents[2]
BASE = {"mass": 270, "cg_height": 290, "wheelbase": 1530, "track_front": 1220, "track_rear": 1200,
        "weight_dist_front": 0.47, "power_kw": 80, "drive": "awd", "wheel_radius_m": 0.2, "overall_ratio": 12.4,
        "redline_rpm": 20000}
PLAN = {"team": "T", "car": "C", "test": "day", "base": BASE, "setups": {"A": {}, "B": {"mass": 290}}}
NOW = datetime(2027, 1, 1, tzinfo=UTC)


def test_resolve_and_unknown_inputs():
    assert T.resolve(BASE, {"mass": 1})["mass"] == 1
    with pytest.raises(ValueError, match="massss"):
        T.resolve(BASE, {"massss": 1})


def test_heavier_car_is_slower_and_flat_power_without_motor_map():
    p = T.seal(PLAN, now=NOW)["predictions"]
    assert p["B"]["skidpad_s"] > p["A"]["skidpad_s"] and p["B"]["accel_s"] > p["A"]["accel_s"]
    assert "lap_s" not in p["A"]
    flat = T.predict_setup({k: v for k, v in BASE.items() if k != "redline_rpm"})
    assert flat["accel_s"] > 0
    given = T.predict_setup({**BASE, "motor_torque_nm": 60.0})
    assert given["accel_s"] > 0


def test_needs_two_setups():
    with pytest.raises(ValueError):
        T.seal({**PLAN, "setups": {"A": {}}})
    with pytest.raises(ValueError):
        T.seal({"base": BASE})


def test_private_files_enter_as_hashes_and_course_adds_lap(tmp_path):
    tire = {"coeffs": dict(_DEFAULT_FSAE_COEFFS), "FNOMIN": _DEFAULT_FSAE_FNOMIN}
    (tmp_path / "tire.json").write_text(json.dumps(tire), encoding="utf-8")
    rows = ["x_m,y_m"] + [f"{x},0" for x in range(0, 40)] + [f"40,{y}" for y in range(1, 20)] + \
           [f"{x},20" for x in range(39, -1, -1)] + [f"0,{y}" for y in range(19, 0, -1)]
    (tmp_path / "course.csv").write_text("\n".join(rows) + "\n", encoding="utf-8")
    plan = {**PLAN, "base": {**BASE, "tire_json": "tire.json"}, "course_csv": "course.csv",
            "setups": {"A": {}, "grippier": {"tire_mu_scale": 1.1}}}
    rec = T.seal(plan, root=tmp_path, now=NOW)
    assert set(rec["input_files_sha256"]) == {"tire.json", "course.csv"}
    assert "PCY1" not in json.dumps(rec["input_files_sha256"])
    p = rec["predictions"]
    assert p["grippier"]["lap_s"] < p["A"]["lap_s"] and p["grippier"]["max_lat_g"] > p["A"]["max_lat_g"]
    assert T.verify(rec, root=tmp_path) == {"intact": True, "same_code": True, "same_inputs": True,
                                            "reproduced": True}
    (tmp_path / "tire.json").write_text(json.dumps({**tire, "FNOMIN": 5000}), encoding="utf-8")
    v = T.verify(rec, root=tmp_path)
    assert v["intact"] and not v["same_inputs"] and not v["reproduced"]
    (tmp_path / "tire.json").unlink()
    assert not T.verify(rec, root=tmp_path)["same_inputs"]


def test_tamper_detected_and_score_refuses():
    rec = T.seal(PLAN, now=NOW)
    bad = json.loads(json.dumps(rec))
    bad["predictions"]["A"]["skidpad_s"] -= 0.1
    assert not T.verify(bad)["intact"]
    with pytest.raises(ValueError):
        T.score(bad, {})


def test_score_absolute_ab_and_ties(tmp_path):
    plan = {**PLAN, "setups": {"A": {}, "B": {"mass": 290}, "C": {"mass": 271}}, "noise": {"accel_s": 0.05}}
    rec = T.seal(plan, now=NOW)
    p = rec["predictions"]
    csv = tmp_path / "m.csv"
    csv.write_text("setup,channel,value\n"
                   f"A,skidpad_s,{p['A']['skidpad_s'] * 1.1}\nA,skidpad_s,{p['A']['skidpad_s'] * 1.1}\n"
                   f"B,skidpad_s,{p['B']['skidpad_s'] * 1.1}\n"
                   f"C,skidpad_s,{p['A']['skidpad_s'] * 1.1 - 0.05}\n"
                   f"A,accel_s,3.50\nB,accel_s,3.52\n", encoding="utf-8")
    m = T.load_measured_csv(csv)
    assert m["A"]["skidpad_s"] == pytest.approx(p["A"]["skidpad_s"] * 1.1)
    sc = T.score(rec, m)
    s = sc["summary"]
    assert s["absolute_n"] == 5 and s["absolute_within_5pct"] == 2      # the two accel values
    rows = {(r["pair"], r["channel"]): r for r in sc["ab"]}
    assert rows[("B - A", "skidpad_s")]["sign_right"] is True
    assert rows[("C - A", "skidpad_s")]["sign_right"] is False          # measured faster, predicted slower
    assert rows[("B - A", "accel_s")]["tie"] and rows[("B - A", "accel_s")]["sign_right"] is None
    assert s["ab_ties"] == 1 and s["ab_decided"] == 3
    md = T.scorecard_markdown(sc)
    assert "WRONG" in md and "tie (within noise)" in md and "right" in md
    empty = T.score(rec, {})
    assert empty["summary"]["absolute_mae_pct"] is None and "n/a" in T.scorecard_markdown(empty)


def test_zero_measured_change_with_no_noise_is_decided_without_size():
    rec = T.seal(PLAN, now=NOW)
    sc = T.score(rec, {"A": {"accel_s": 3.5}, "B": {"accel_s": 3.5}})
    r = sc["ab"][0]
    assert r["tie"] and r["size_error_pct"] is None


def test_cli_end_to_end_on_the_repo_example(tmp_path):
    import importlib.util
    spec = importlib.util.spec_from_file_location("tt_cli", REPO / "fsae_suspension" / "scripts" / "team_test.py")
    cli = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cli)
    ex = REPO / "benchmarks" / "team-test" / "example"
    rec_path = tmp_path / "SEALED.json"
    assert cli.main(["seal", "--plan", str(ex / "plan.json"), "--out", str(rec_path)]) == 0
    assert cli.main(["verify", "--record", str(rec_path), "--root", str(ex)]) == 0
    assert cli.main(["score", "--record", str(rec_path), "--measured", str(ex / "measured.csv"),
                     "--out", str(tmp_path / "sc")]) == 0
    assert (tmp_path / "sc" / "SCORECARD.md").exists()
    rec = json.loads(rec_path.read_text(encoding="utf-8"))
    rec["team"] = "someone else"
    rec_path.write_text(json.dumps(rec), encoding="utf-8")
    assert cli.main(["verify", "--record", str(rec_path), "--root", str(ex)]) == 1
    assert cli.main(["score", "--record", str(rec_path), "--measured", str(ex / "measured.csv"),
                     "--out", str(tmp_path / "sc2")]) == 1


# --------------------------------------------------------------------------- #
#  Suspension channels: camber and toe from the team's hardpoints
# --------------------------------------------------------------------------- #
def _hp():
    from suspension.kinematics import Hardpoints
    return Hardpoints.default().as_dict()


def test_kinematics_inline_file_and_overrides(tmp_path):
    from suspension.kinematics import Hardpoints, SuspensionKinematics
    (tmp_path / "rear.json").write_text(json.dumps(_hp()), encoding="utf-8")
    plan = {**PLAN, "kinematics": {"front": {"hardpoints": _hp(), "travel_mm": [-20, 0, 20]},
                                   "rear": {"hardpoints_json": "rear.json"}},
            "setups": {"A": {}, "shim": {"kinematics": {"front": {"static_camber": -2.0}}}}}
    rec = T.seal(plan, root=tmp_path, now=NOW)
    assert "rear.json" in rec["input_files_sha256"]
    a, b = rec["predictions"]["A"], rec["predictions"]["shim"]
    st = SuspensionKinematics(Hardpoints.from_dict(_hp())).solve_at_travel(20.0)
    assert a["camber_front_+20mm"] == pytest.approx(st.camber, abs=1e-3)
    assert a["toe_front_+20mm"] == pytest.approx(st.toe, abs=1e-3)
    assert "camber_rear_-25mm" in a and "camber_rear_+25mm" in a           # default travel set
    assert b["camber_front_+0mm"] == pytest.approx(-2.0) and a["camber_front_+0mm"] == pytest.approx(-1.5)
    assert b["skidpad_s"] == a["skidpad_s"]                                 # a kinematics-only setup
    assert T.verify(rec, root=tmp_path)["reproduced"]


def test_kinematics_that_does_not_solve_is_refused():
    plan = {**PLAN, "kinematics": {"front": {"hardpoints": _hp(), "travel_mm": [1000]}}}
    with pytest.raises(ValueError, match="does not solve"):
        T.seal(plan, now=NOW)


def test_angles_scored_in_degrees_with_tolerance_and_prefix_noise():
    plan = {**PLAN, "kinematics": {"front": {"hardpoints": _hp(), "travel_mm": [0, 20]}},
            "setups": {"A": {}, "shim": {"kinematics": {"front": {"static_camber": -2.0}}}},
            "noise": {"camber": 0.05}, "tolerance": {"camber": 0.3}}
    rec = T.seal(plan, now=NOW)
    p = rec["predictions"]
    meas = {"A": {"camber_front_+20mm": p["A"]["camber_front_+20mm"] + 0.1, "toe_front_+20mm": 0.5,
                  "camber_front_+0mm": -1.5},
            "shim": {"camber_front_+20mm": p["shim"]["camber_front_+20mm"] - 0.5, "camber_front_+0mm": -1.52}}
    sc = T.score(rec, meas)
    ang = {(r["setup"], r["channel"]): r for r in sc["angles"]}
    assert ang[("A", "camber_front_+20mm")]["error_deg"] == pytest.approx(-0.1, abs=1e-3)
    assert ang[("A", "camber_front_+20mm")]["within"] is True
    assert ang[("shim", "camber_front_+20mm")]["within"] is False
    assert ang[("A", "toe_front_+20mm")]["within"] is None                   # no toe tolerance given
    s = sc["summary"]
    assert s["angles_n"] == 5 and s["angles_with_tolerance"] == 4 and s["angles_within_tolerance"] == 2
    assert s["absolute_n"] == 0
    ab = {(r["pair"], r["channel"]): r for r in sc["ab"]}
    assert ab[("shim - A", "camber_front_+0mm")]["tie"] is True              # measured 0.02 deg < 0.05 noise
    md = T.scorecard_markdown(sc)
    assert "Within tolerance" in md and "NO" in md and "no tolerance set" in md
    assert "largest error" in md


def test_tolerance_from_measuring_error(monkeypatch):
    corners = {"front": (_hp(), [-20.0, 0.0, 20.0])}
    zero = T.kinematics_uncertainty(corners, 0.0, n=5)
    assert all(v == 0.0 for v in zero.values())
    small = T.kinematics_uncertainty(corners, 0.1, n=40)
    big = T.kinematics_uncertainty(corners, 1.0, n=40)
    assert big["camber_front_+20mm"] > small["camber_front_+20mm"] > 0.0
    assert big["camber_front_+0mm"] == 0.0               # static camber is an input, not a prediction
    plan = {"kinematics": {"front": {"hardpoints": _hp(), "travel_mm": [-20, 0, 20]}}}
    tol = T.suggested_tolerance(plan, 0.0, 0.1, n=5)
    assert tol == {"camber": 0.1, "toe": 0.1}
    real = T.predict_kinematics
    calls = {"n": 0}

    def flaky(c):
        calls["n"] += 1
        if calls["n"] > 1:
            raise ValueError("does not solve")
        return real(c)
    monkeypatch.setattr(T, "predict_kinematics", flaky)
    assert all(v is None for v in T.kinematics_uncertainty(corners, 0.5, n=3).values())


def test_cli_tolerance(capsys):
    import importlib.util
    spec = importlib.util.spec_from_file_location("tt_cli2", REPO / "fsae_suspension" / "scripts" / "team_test.py")
    cli = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cli)
    ex = REPO / "benchmarks" / "team-test" / "example"
    assert cli.main(["tolerance", "--plan", str(ex / "plan.json"), "--sigma-mm", "0.2", "--gauge-deg", "0.1",
                     "--draws", "10"]) == 0
    assert '"camber"' in capsys.readouterr().out
