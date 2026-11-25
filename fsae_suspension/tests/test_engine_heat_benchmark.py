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
