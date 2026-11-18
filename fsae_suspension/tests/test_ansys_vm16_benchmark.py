"""MorphMesh and Bracket FoS against the published Ansys Mechanical APDL results of VM16, limits registered first."""
import importlib.util
from pathlib import Path

_spec = importlib.util.spec_from_file_location(
    "ansys_vm16_benchmark", Path(__file__).resolve().parents[1] / "scripts" / "ansys_vm16_benchmark.py")
vm = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(vm)


def test_matches_ansys_vm16_published_results():
    res = vm.main()
    assert len(res["rows"]) == 5 and res["all_pass"], res["rows"]
