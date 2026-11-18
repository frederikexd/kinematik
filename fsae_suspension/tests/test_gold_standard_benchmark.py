"""Bracket FoS and MorphMesh against commercial FE tools' published verification results, limits registered first."""
import importlib.util
from pathlib import Path

_spec = importlib.util.spec_from_file_location(
    "gold_standard_benchmark", Path(__file__).resolve().parents[1] / "scripts" / "gold_standard_benchmark.py")
gs = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(gs)


def test_matches_published_results_of_at_least_three_commercial_tools_each():
    res = gs.main()
    assert res["all_pass"], [r for r in res["rows"] if not r["passes"]]
    assert all(len(t) >= 3 for t in res["tools_per_feature"].values()), res["tools_per_feature"]
