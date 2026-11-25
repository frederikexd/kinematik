"""The chassis frame solver against Ansys Mechanical APDL VM18 and VM217 (scripts/frame_fe_benchmark.py)."""
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))


def test_the_stored_run_passed():
    res = json.loads((ROOT / "benchmarks" / "frame_fe" / "results.json").read_text())
    assert res["passed"]


def test_vm18_live():
    import frame_fe_benchmark as fb
    r = fb.vm18()
    assert r["deflection_in"] == pytest.approx(-2.648, rel=0.005)
    assert r["bending_stress_psi"] == pytest.approx(6366.0, rel=0.005)
    assert r["torsion_shear_psi"] == pytest.approx(3183.0, rel=0.005)


def test_vm217_live_matches_hoff_closer_than_apdl():
    import frame_fe_benchmark as fb
    r = fb.vm217()
    assert r["max_rotation_rad"] == pytest.approx(1.946e-3, rel=0.001)
    assert r["max_bending_moment_lb_in"] == pytest.approx(2.815e7, rel=0.001)
