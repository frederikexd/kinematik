"""torque distribution benchmark (scripts/torque_distribution_benchmark.py): the stored run and a live rerun."""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
STORED = json.loads((ROOT / "benchmarks" / "torque_distribution" / "results.json").read_text())


def test_live_rerun_reproduces_the_stored_verdict():
    import torque_distribution_benchmark as b
    assert b.run()["passed"] == STORED["passed"]


def test_every_steady_limit_agrees_with_the_global_search():
    assert all(r["passes"] for r in STORED["steady"])


def test_the_one_recorded_miss_is_an_infeasible_instant():
    """The registered run fails on one instant that no split makes feasible: the reported overshoot is 0.5 % above
    the global minimum. Every feasible instant passes."""
    misses = [r for r in STORED["instants"] if not r["passes"]]
    assert len(misses) == 1 and misses[0]["global_umax"] > 1.0
    assert all(r["passes"] for r in STORED["instants"] if r["global_umax"] <= 1.0)
