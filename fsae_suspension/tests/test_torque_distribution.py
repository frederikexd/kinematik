# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Rear torque distribution, torque vectoring and tire exploitation."""
import math

import pytest

from suspension.torque_distribution import Car, corner_path, state, steady_limit

G = 9.81


def test_force_and_yaw_balance_hold():
    c = Car(); V, k, dk, at = 9.0, 1 / 9.125, 0.004, 2.0
    r = state(V, k, dk, at, "tv", c)
    assert r["ok"]
    assert r["Fy_f"] + r["Fy_r"] == pytest.approx(c.m * V * V * k, rel=1e-6)
    lhs = c.a * r["Fy_f"] - c.b * r["Fy_r"] + r["mz_x"]
    assert lhs == pytest.approx(c.iz * (V * V * dk + at * k), abs=1e-6)
    assert r["mz_x"] == pytest.approx((r["fx"]["RR"] - r["fx"]["RL"]) * c.t / 2.0)


def test_open_split_is_equal_and_lsd_respects_its_bias_ratio():
    o = state(8.0, 1 / 15.0, 0.0, 3.0, "open")
    assert o["fx"]["RL"] == pytest.approx(o["fx"]["RR"])
    l = state(8.0, 1 / 15.0, 0.0, 3.0, "lsd3")
    assert l["fx"]["RL"] >= l["fx"]["RR"] - 1e-9                     # torque goes to the slower inside wheel
    assert l["fx"]["RL"] <= 3.0 * l["fx"]["RR"] + 1e-6


def test_tv_respects_the_motor_limit():
    c = Car(tv_wheel_max_n=600.0); r = state(10.0, 1 / 15.0, 0.0, 2.0, "tv", c)
    assert max(abs(r["fx"]["RL"]), abs(r["fx"]["RR"])) <= 600.0 + 1e-6


def test_tv_never_worse_and_spool_worst_at_the_hairpin():
    for R in (9.125, 30.0):
        assert steady_limit(R, "tv")["ay_g"] >= steady_limit(R, "open")["ay_g"] - 0.003
    assert steady_limit(4.5, "spool")["ay_g"] < 0.6 * steady_limit(4.5, "open")["ay_g"]


def test_path_is_continuous_in_curvature():
    s, k, dk = corner_path(9.125, 90.0)
    assert k.max() == pytest.approx(1 / 9.125) and k[0] == 0.0 and k[-1] == 0.0
    assert all(abs(k[i + 1] - k[i]) < 0.05 for i in range(len(k) - 1))
