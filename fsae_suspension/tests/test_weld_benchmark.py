"""The bracket weld check against Shigley (scripts/weld_benchmark.py)."""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))


def test_the_stored_run_passed():
    assert json.loads((ROOT / "benchmarks" / "weld" / "results.json").read_text())["passed"]


def test_live_run_passes():
    import weld_benchmark as wb
    assert wb.run()["passed"]
