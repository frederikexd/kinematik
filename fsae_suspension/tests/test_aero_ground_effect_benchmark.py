"""Vortex lattice absolute downforce against Zerihan's wind-tunnel wing in ground effect (scripts/
aero_ground_effect_benchmark.py). Both registered phases fail; these tests pin what they found."""
import json
import math
import os
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
B = ROOT / "benchmarks" / "aero_ground_effect"
P1 = json.loads((B / "results_phase1.json").read_text())
P2 = json.loads((B / "results.json").read_text())


def test_both_phases_are_reported_as_they_came_out():
    assert P1["passed"] is False and P2["passed"] is False
    assert -0.25 < P1["graded"]["freestream_CL"]["rel"] < -0.15        # wing alone
    assert -0.20 < P2["graded"]["freestream_CL"]["rel"] < -0.12        # with endplates
    assert P2["graded"]["h_c_0.671_CL"]["rel"] > P1["graded"]["h_c_0.671_CL"]["rel"]


def test_downforce_still_rises_towards_the_ground():
    cl = [p["downforce_CL"] for p in P2["points"]]
    assert all(b > a for a, b in zip(cl, cl[1:]))


def test_endplates_add_lift_and_converge_with_their_panel_count():
    import trimesh
    from suspension.aero.backends import Attitude, CaseSpec
    from suspension.aero.vortex_lattice import VortexLatticeModel
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "plate.stl")
        trimesh.creation.box(extents=[0.2, 0.4, 0.002]).export(path)

        def cl(n_height):
            ep = dict(length_m=0.2, height_m=0.1, x_start_m=0.0, z_bottom_m=-0.05, n_height=n_height, n_length=8)
            spec = CaseSpec(attitude=Attitude(pitch_deg=4.0, ride_height_mm=50000.0), geometry_path=path,
                            reference_area_m2=0.08, extra={"endplates": ep} if n_height else {})
            return abs(VortexLatticeModel(20, 8).solve(spec).c_lift)
        bare, coarse, fine = cl(0), cl(6), cl(12)
        assert coarse > 1.05 * bare and fine > 1.05 * bare
        assert math.isclose(coarse, fine, rel_tol=0.02)
