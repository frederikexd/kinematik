"""Brake hydraulics to torque against a published disk-brake example (scripts/brake_hydraulics_benchmark.py)."""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))


def test_the_stored_run_passed():
    assert json.loads((ROOT / "benchmarks" / "brake_hydraulics" / "results.json").read_text())["passed"]


def test_live_run_passes():
    import brake_hydraulics_benchmark as bh
    assert bh.run()["passed"]
