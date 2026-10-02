# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Instantaneous tire utilisation against the traction ellipse."""
import math

import pytest

from suspension.tire_utilisation import TIRES, limit_envelope, tire_state

M, G = 300.0, 9.81


def test_lateral_equilibrium_and_near_limit_at_design_case():
    s = tire_state(0.0, 1.5)
    assert s["feasible"]
    assert sum(s[t]["Fy"] for t in TIRES) == pytest.approx(M * G * 1.5, rel=1e-6)
    assert s["FL"]["Fy"] + s["FR"]["Fy"] == pytest.approx(0.48 * M * G * 1.5, rel=1e-6)   # yaw balance
    assert 0.95 < s["vehicle_u"] <= 1.0                     # 1.5 g sits at the car's lateral limit
    assert s["FL"]["u"] == max(s[t]["u"] for t in TIRES)    # inside front saturates first (53% front LLTD)


def test_rolling_straight_uses_nothing():
    s = tire_state(0.0, 0.0)
    assert all(s[t]["u"] == pytest.approx(0.0, abs=1e-9) for t in TIRES)


def test_spare_grip_definition():
    s = tire_state(0.5, 0.8)
    for t in TIRES:
        d = s[t]; F = math.hypot(d["Fx"], d["Fy"])
        assert d["spare_n"] == pytest.approx(F * (1 / d["u"] - 1))
        assert d["margin"] == pytest.approx(1 - d["u"])


def test_sixty_percent_bias_locks_the_rears_first():
    s = tire_state(1.2, 0.0, brake_front=0.60)
    assert s["RL"]["u"] > s["FL"]["u"]
    def max_decel(b):
        lo, hi = 0.0, 2.5
        for _ in range(40):
            mid = 0.5 * (lo + hi); st = tire_state(mid, 0.0, brake_front=b)
            if st["feasible"] and st["vehicle_u"] <= 1.0: lo = mid
            else: hi = mid
        return lo
    assert max_decel(0.73) > max_decel(0.60) + 0.2        # the balanced bias gains about 0.3 g


def test_envelope_has_one_tire_at_the_edge_everywhere():
    env = limit_envelope(n_dir=8)
    for r in env:
        assert max(r[f"u_{t}"] for t in TIRES) == pytest.approx(1.0, abs=0.01)
    assert env[4]["a_g"] > env[0]["a_g"]                    # cornering limit exceeds the braking limit at 60% bias
