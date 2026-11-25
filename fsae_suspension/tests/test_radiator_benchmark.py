"""Radiator effectiveness-NTU against an exact solution and a published case (scripts/radiator_benchmark.py)."""
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT))


def test_the_stored_run_passed():
    assert json.loads((ROOT / "benchmarks" / "radiator" / "results.json").read_text())["passed"]


def test_live_run_passes():
    import radiator_benchmark as rb
    assert rb.run()["passed"]


def test_heat_balance_and_the_air_limit():
    from suspension.cooling import radiator_heat_w, size_loop, LoopSpec
    r = radiator_heat_w(ua_w_per_k=500.0, air_mass_flow_kg_s=0.3, coolant_mass_flow_kg_s=0.2,
                        coolant_cp_j_kgk=3500.0, coolant_in_c=70.0, air_in_c=30.0)
    assert r["q_w"] == pytest.approx(0.3 * 1007.0 * (r["air_out_c"] - 30.0))
    assert r["q_w"] == pytest.approx(0.2 * 3500.0 * (70.0 - r["coolant_out_c"]))
    assert r["q_w"] < 0.3 * 1007.0 * 40.0                 # never more than the air can carry
    assert size_loop(LoopSpec()).radiator is None          # no air flow given: the old screen only
    assert size_loop(LoopSpec(air_mass_flow_kg_s=0.02)).ok is False
