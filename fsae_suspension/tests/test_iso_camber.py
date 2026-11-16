# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Two definitions of camber are in use. KinematiK prints the front-view projection of the spin axis, as Lotus does
(tests/test_lotus_benchmark.py, camber in steer); MathWorks' K&C rig measures the inclination of the wheel plane
(ISO 8855, SAE J670), which camber_inclination gives (benchmarks/knc_simscape). They differ once the wheel is steered."""
import json
import math
from pathlib import Path

import numpy as np
import pytest

from suspension.genesis_repro import hp_from_dict
from suspension.kinematics import SuspensionKinematics, camber_inclination

ROOT = Path(__file__).resolve().parents[1]


def _winner(name):
    return json.loads((ROOT / "manifests" / f"{name}.recorded.genesis.json").read_text())["recorded"]["winner_hardpoints"]


def test_inclination_is_the_spin_axis_angle_to_the_ground():
    for c, t in ((-2.5, 0.0), (-2.5, 12.0), (1.0, -20.0), (-4.0, 25.0)):
        s = np.array([math.tan(math.radians(t)), 1.0, -math.tan(math.radians(c))])     # projection c, toe t
        s /= np.linalg.norm(s)
        assert camber_inclination(c, t) == pytest.approx(-math.degrees(math.asin(s[2])), abs=1e-12)


def test_the_two_agree_at_zero_toe_and_differ_in_steer():
    w = _winner("front_v11_final")
    s0 = SuspensionKinematics(hp_from_dict(w)).solve_at_travel(0.0)
    assert camber_inclination(s0.camber, s0.toe) == pytest.approx(s0.camber, abs=1e-12)
    s = SuspensionKinematics(hp_from_dict(w), pickup_deltas={"tie_rod_inner": np.array([0.0, 12.0, 0.0])}) \
        .solve_at_travel(0.0)
    assert abs(s.toe) > 10.0 and abs(camber_inclination(s.camber, s.toe) - s.camber) > 0.03
