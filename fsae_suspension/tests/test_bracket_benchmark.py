"""Bracket bearing, net section and tear-out against Shigley and AISC 360 (scripts/bracket_benchmark.py)."""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))


def test_the_stored_run_passed():
    assert json.loads((ROOT / "benchmarks" / "bracket" / "results.json").read_text())["passed"]


def test_live_run_passes():
    import bracket_benchmark as bb
    assert bb.run()["passed"]
