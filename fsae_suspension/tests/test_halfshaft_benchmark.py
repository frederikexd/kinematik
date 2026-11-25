"""The halfshaft fatigue screen against Shigley Eq. 7-8 and problem 6-56 (scripts/halfshaft_benchmark.py)."""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))


def test_the_stored_run_passed():
    assert json.loads((ROOT / "benchmarks" / "halfshaft" / "results.json").read_text())["passed"]


def test_live_run_passes():
    import halfshaft_benchmark as hb
    assert hb.run()["passed"]
