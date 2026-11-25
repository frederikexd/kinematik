"""Vortex lattice absolute downforce against Zerihan's wind-tunnel wing in ground effect. The registered run fails
(the lattice has no endplates); these tests pin that the stored result says so and stays what it was."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
R = json.loads((ROOT / "benchmarks" / "aero_ground_effect" / "results.json").read_text())


def test_the_stored_run_is_reported_as_it_came_out():
    assert R["passed"] is False
    assert -0.25 < R["graded"]["freestream_CL"]["rel"] < -0.15
    assert -0.28 < R["graded"]["h_c_0.671_CL"]["rel"] < -0.18


def test_downforce_still_rises_towards_the_ground():
    cl = [p["downforce_CL"] for p in R["points"]]
    assert all(b > a for a, b in zip(cl, cl[1:]))
