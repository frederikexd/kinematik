"""Steady lateral load transfer against an OpenSeesPy solve of the same model (scripts/load_transfer_benchmark.py)."""
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))


def test_the_stored_run_passed():
    assert json.loads((ROOT / "benchmarks" / "load_transfer" / "results.json").read_text())["passed"]


def test_live_run_passes():
    pytest.importorskip("openseespy")
    import load_transfer_benchmark as lb
    assert lb.run()["passed"]
