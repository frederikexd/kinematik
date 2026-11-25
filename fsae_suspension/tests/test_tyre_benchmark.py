"""Magic Formula lateral force against MFeval 4.3.1 (scripts/tyre_benchmark.py)."""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))


def test_the_stored_run_passed():
    assert json.loads((ROOT / "benchmarks" / "tyre" / "results.json").read_text())["passed"]


def test_live_run_matches_mfeval():
    import tyre_benchmark as tb
    r = tb.run()
    assert r["passed"]
    mf52 = r["MagicFormula52_Parameters"]
    assert mf52["scope"] == "all" and all(g["worst_dFy_N"] < 1e-3 for g in mf52["groups"])
