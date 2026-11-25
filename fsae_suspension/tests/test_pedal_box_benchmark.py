"""pedal box benchmark (scripts/pedal_box_benchmark.py): the stored run and a live rerun."""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
STORED = json.loads((ROOT / "benchmarks" / "pedal_box" / "results.json").read_text())


def test_live_rerun_reproduces_the_stored_verdict():
    import pedal_box_benchmark as b
    assert b.run()["passed"] == STORED["passed"]


def test_the_stored_run_passed():
    assert STORED["passed"]
