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


def test_the_stored_run_passed():
    assert STORED["passed"]


def test_the_default_flags_an_infeasible_overshoot_as_a_bound_and_the_exact_search_finds_the_minimum():
    sys.path.insert(0, str(ROOT))
    from suspension import torque_distribution as td
    quick = td.state(16.0, 1 / 15, 0.0, -8.0, "tv")
    exact = td.state(16.0, 1 / 15, 0.0, -8.0, "tv", exact_overshoot=True)
    assert not quick["ok"] and quick["overshoot_upper_bound"] is True
    assert not exact["ok"] and exact["overshoot_upper_bound"] is False
    assert exact["umax"] <= quick["umax"] and abs(exact["umax"] - 2.5079) < 0.002
