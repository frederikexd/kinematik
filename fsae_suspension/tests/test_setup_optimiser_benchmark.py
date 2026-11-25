"""The setup optimiser against a global search, and the wheel-lift cap it exposed (scripts/setup_optimiser_benchmark.py)."""
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def test_the_stored_run_passed():
    r = json.loads((ROOT / "benchmarks" / "setup_optimiser" / "results.json").read_text())
    assert r["passed"] and r["gap_g"] <= 0.005


@pytest.mark.parametrize("g", [1.0, 1.6, 2.2, 3.0])
def test_a_lifted_axle_carries_its_weight_and_no_more(g):
    """Past inner-wheel lift the outer wheel carries the whole axle; the rest of the roll moment goes to the other
    axle. Before the fix the outer wheel kept gaining, so the axle carried more than its weight."""
    from suspension.dynamics import VehicleDynamics, VehicleParams
    p = VehicleParams(cg_height=330.0, weight_dist_front=0.58, roll_stiffness_front=900.0, roll_stiffness_rear=100.0)
    loads, info = VehicleDynamics(p).lateral_load_transfer(g)
    W = p.mass * p.g
    Wf, Wr = W * p.weight_dist_front, W * (1 - p.weight_dist_front)
    assert loads.fl + loads.fr == pytest.approx(Wf) and loads.rl + loads.rr == pytest.approx(Wr)
    assert min(loads.fl, loads.fr, loads.rl, loads.rr) >= 0.0
    if not (info["wheel_lift_front"] and info["wheel_lift_rear"]):
        # the total transfer still closes the moment balance
        tot = info["ltd_front"] * p.track_front + info["ltd_rear"] * p.track_rear
        assert tot == pytest.approx(p.mass * g * p.g * p.cg_height, rel=1e-9)
