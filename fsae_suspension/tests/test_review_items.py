# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The independent solver agrees with KinematiK's, and the brake result's grip dependence holds in closed form
(paper section 3.3 and 3.4)."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import independent_kinematics as ik                                       # noqa: E402
import review_checks as rc                                                # noqa: E402


@pytest.mark.parametrize("name", ["front_v11_final", "rear_v11_final"])
def test_the_independent_solver_reproduces_camber_toe_and_roll_centre(name):
    r = ik.compare(name)
    assert r["camber_max_diff_deg"] < 1e-9 and r["toe_max_diff_deg"] < 1e-9
    assert abs(r["rc_independent_mm"] - r["rc_kinematik_mm"]) < 1.0


def test_brake_limit_at_the_balanced_bias_is_mu_and_the_gain_is_grip_dependent():
    b = rc.brake_grip()
    for mu, r in b["road"].items():
        assert abs(r["dbal"] - float(mu)) < 1e-9                         # all four tires saturate together: a/g = mu
        assert r["dbal"] >= max(r["d60"], r["d73"]) - 1e-12
    assert abs(b["const155"]["gain73_pct"] - 24.24) < 0.01
    assert abs(b["mu_balanced_at_60"] - 0.6986) < 1e-3
    assert b["road"]["1.09"]["gain73_pct"] < 0.0 < b["road"]["1.09"]["gain_bal_pct"]   # at low grip 73% is too far forward


def test_the_roll_gradient_is_the_softest_that_holds_at_the_tire_belt_friction():
    from suspension import target_derivation as td
    v, t = td.Vehicle(), td.Tire()
    g = td.roll_gradient_for_friction(v, t, 46.68, "rear", 3.0, 1.82)
    assert 0.78 <= g < 0.80                                             # the declared 0.78 sits just inside it
    assert td.f_min(v.__class__(**{**v.__dict__, "roll_gradient_deg_per_g": g - 1e-3}), 46.68,
                    1.06 * 1.82 / 1.55, 1.06 * 1.82 / 1.55, "rear") <= 3.0
    assert td.roll_gradient_for_friction(v, t, 68.94, "front", 2.6, 1.82) > g   # the rear binds first
    sp = td.springs_only_roll_gradient(v, 52.81, 41.63)
    assert sp["gradient_deg_per_g"] > 1.02                              # springs alone exceed the travel limit: bars needed
