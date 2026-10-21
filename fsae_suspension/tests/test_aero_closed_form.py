# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The aero solvers against closed form (scripts/aero_closed_form.py, PHYSICS.md §10, §11).

The bounds hold today's measured values with a small margin. The lattice's lift
sits about 4% under an independent flat-plate lattice and about 9% under
Helmbold's lifting-surface estimate; that gap is an open item in VALIDATION.md,
so the bounds pin it rather than hide it, and tightening them is the fix."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import aero_closed_form as acf                                            # noqa: E402


def test_sphere_surface_pressure_matches_potential_flow():
    s = acf.sphere_cp_error(subdivisions=3)
    assert s["cp_min"] == pytest.approx(-1.25, abs=0.01)
    assert s["max_dcp"] < 0.035
    assert s["mean_dcp"] < 0.012


def test_wing_lift_against_lifting_line_and_an_independent_lattice():
    w = acf.wing_check(ns=40, nc=10)
    assert abs(w["cl"] / w["lifting_line"] - 1) < 0.15        # the README's bound
    assert abs(w["cl"] / w["independent"] - 1) < 0.06
    assert 0.85 < w["e"] < 1.0


def test_wing_lift_is_grid_converged():
    a = acf.wing_check(ns=30, nc=8)["cl"]
    b = acf.wing_check(ns=40, nc=10)["cl"]
    assert abs(a / b - 1) < 0.01


def test_ground_effect_raises_wing_lift():
    free = acf.wing_check()["cl"]
    near = acf.wing_check(ride_height_mm=30.0)["cl"]
    assert near > 2.0 * free
