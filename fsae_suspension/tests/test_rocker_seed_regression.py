# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Found by the random-geometry benchmark: 12 of 512 random corners failed the rocker solve exactly where a warm-started
sweep crossed the static rocker angle. The seed there was ~1e-17 rad and MINPACK's finite-difference step is relative
to the variable, so the Jacobian was noise. Project Chrono solved all 12; KinematiK now does too."""
import json
from pathlib import Path

import numpy as np
import pytest

from suspension.genesis_repro import hp_from_dict
from suspension.kinematics import SuspensionKinematics

CORNER = Path(__file__).resolve().parents[1] / "benchmarks" / "random_geometry" / "rocker_seed_regression.json"


@pytest.fixture(scope="module")
def kin():
    return SuspensionKinematics(hp_from_dict(json.loads(CORNER.read_text())))


def test_a_warm_started_sweep_solves_the_rocker_through_static(kin):
    seed, failed = 0.0, []
    for s in kin.sweep(-40, 40, 321):
        _, seed, ok = kin.spring_length_at(s, seed=seed)
        if not ok:
            failed.append(float(s.wheel_center[2]))
    assert not failed


@pytest.mark.parametrize("tiny", [1e-17, -1e-17, 8e-18, 1e-30])
def test_a_tiny_seed_solves_as_a_zero_seed_does(kin, tiny):
    st = kin.solve_at_travel(0.3)
    a = kin.spring_length_at(st, seed=0.0)
    b = kin.spring_length_at(st, seed=tiny)
    assert b[2] and a[2] and b[0] == pytest.approx(a[0], abs=1e-9) and b[1] == pytest.approx(a[1], abs=1e-12)
