"""ICE heat to coolant against a measured SI-engine heat balance (scripts/engine_heat_benchmark.py)."""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))


def test_the_stored_run_passed_and_the_bias_is_on_the_safe_side():
    r = json.loads((ROOT / "benchmarks" / "engine_heat" / "results.json").read_text())
    assert r["passed"] and r["bias"]["coolant_vs_full_load_mean"] > 0      # more heat than measured: radiator sized up


def test_live_run_matches_the_stored_one():
    import engine_heat_benchmark as eb
    assert eb.run() == json.loads((ROOT / "benchmarks" / "engine_heat" / "results.json").read_text())


def test_phase_2_first_run_missed_and_the_new_default_passes():
    import engine_heat_benchmark as eb
    first = json.loads((ROOT / "benchmarks" / "engine_heat" / "results_phase2_first_run.json").read_text())
    assert not first["passed"] and all(0.30 < g["rel"] < 0.40 for g in first["graded"])     # 0.28, kept on file
    r = eb.phase_2()
    assert r["passed"] and all(abs(g["rel"]) < 0.10 for g in r["graded"])
    assert r == json.loads((ROOT / "benchmarks" / "engine_heat" / "results_phase2.json").read_text())
