"""MorphMesh's plane-stress kernel against beam theory, re-run live against limits registered before the first run."""
import importlib.util
from pathlib import Path

_spec = importlib.util.spec_from_file_location(
    "morphmesh_benchmark", Path(__file__).resolve().parents[1] / "scripts" / "morphmesh_benchmark.py")
mb = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(mb)


def test_plate_kernel_matches_beam_theory_and_converges():
    res = mb.main()
    assert all(res["passes"].values()), res["passes"]
