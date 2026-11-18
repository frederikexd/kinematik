"""Bracket FoS root bending against three 3D FEA tools: the recorded run, and a live re-run where they are installed."""
import importlib.util
import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location("bracket_fea_benchmark", ROOT / "scripts" / "bracket_fea_benchmark.py")
bb = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(bb)


def _check(res):
    lim = json.loads((ROOT / "benchmarks" / "bracket_fea" / "limits.json").read_text())["limits"]
    assert len(res["tools"]) == 3
    for tool in res["tools"].values():
        judged = [r for r in tool["rows"] if r["judged"]]
        assert len(judged) >= 5
        assert all(r["average_rel"] <= lim["width_average_rel"] for r in judged)
        assert all(r["peak_rel"] <= lim["peak_across_width_rel"] for r in judged)
        for r in judged:                                   # KinematiK's own number, recomputed now
            assert r["kinematik_MPa"] == pytest.approx(bb.kinematik_stress(bb.L - r["x_from_root_mm"]))


def test_recorded_run_meets_the_registered_limits():
    _check(json.loads((ROOT / "benchmarks" / "bracket_fea" / "results.json").read_text()))


def test_the_fixed_root_is_reported_not_claimed():
    for tool in json.loads((ROOT / "benchmarks" / "bracket_fea" / "results.json").read_text())["tools"].values():
        root = tool["rows"][0]
        assert root["x_from_root_mm"] < 1.0 and not root["judged"]
        assert root["fea_peak_MPa"] > root["kinematik_MPa"]  # every tool puts the fixed root above the hand value


@pytest.mark.filterwarnings("ignore")             # third-party deprecation noise (SfePy on pyparsing)
def test_live_rerun():
    for mod in ("skfem", "sfepy", "openseespy"):
        pytest.importorskip(mod)
    _check(bb.main())
