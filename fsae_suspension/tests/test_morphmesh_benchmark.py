"""MorphMesh's plane-stress kernel against beam theory, re-run live against limits registered before the first run."""
import importlib.util
from pathlib import Path

import pytest

_spec = importlib.util.spec_from_file_location(
    "morphmesh_benchmark", Path(__file__).resolve().parents[1] / "scripts" / "morphmesh_benchmark.py")
mb = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(mb)


def test_plate_kernel_matches_beam_theory_and_converges():
    res = mb.main()
    assert all(res["passes"].values()), res["passes"]


def test_recorded_other_tools_agree_with_morphmesh():
    import json
    rec = json.loads((mb.OUT / "results.json").read_text())["other_tools"]
    assert len(rec) == 4 and all(v["passes"] for k, v in rec.items() if k != "morphmesh")


@pytest.mark.filterwarnings("ignore")             # third-party deprecation noise (SfePy on pyparsing)
def test_live_other_tools():
    for mod in ("skfem", "sfepy", "openseespy"):
        pytest.importorskip(mod)
    res = mb.compare_tools()
    assert all(v["passes"] for k, v in res.items() if k != "morphmesh")
