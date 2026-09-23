# ============================================================================
#  KinematiK — tests for car-level tire inputs
#  Copyright (c) 2026 Frederik Thio. Open source.
# ============================================================================
"""Skidpad friction, camber sweep and tire rate recover known inputs, and the
measured tire feeds the target chain."""
import math

import numpy as np
import pytest

from suspension import target_derivation as td
from suspension import tire_field as tf


def test_skidpad_lap_time_gives_the_friction_it_was_driven_at():
    R, mu = 9.125, 1.40
    v = math.sqrt(mu * 9.81 * R)
    t = 2 * math.pi * R / v
    r = tf.mu_from_skidpad(R, lap_times_s=[t + 0.2, t, t + 0.1])
    assert r["mu"] == pytest.approx(mu, rel=1e-9) and r["n_runs"] == 3


def test_downforce_is_removed():
    a = tf.mu_from_skidpad(9.125, ay_g=[1.5])["mu"]
    b = tf.mu_from_skidpad(9.125, ay_g=[1.5], downforce_ClA_m2=2.0)["mu"]
    assert b < a


def test_camber_sweep_finds_the_minimum():
    c = np.array([-3.5, -2.5, -1.5, -0.5])
    t = 5.0 + 0.02 * (c + 1.8) ** 2
    r = tf.camber_optimum_from_sweep(c, t)
    assert r["static_camber_opt_deg"] == pytest.approx(-1.8, abs=1e-9)


def test_camber_sweep_refuses_to_extrapolate():
    c = np.array([-3.0, -2.5, -2.0])
    with pytest.raises(ValueError):
        tf.camber_optimum_from_sweep(c, 5.0 + 0.02 * (c + 1.0) ** 2)


def test_tire_rate_and_wheel_hop():
    d = np.array([2.0, 4.0, 6.0, 8.0])
    r = tf.tire_rate_from_load_test(120.0 * d + 30.0, d, unsprung_corner_kg=11.9,
                                    wheel_rate_N_per_mm=14.8)
    assert r["tire_rate_N_per_mm"] == pytest.approx(120.0)
    assert r["wheel_hop_hz"] == pytest.approx(math.sqrt(134.8e3 / 11.9) / (2 * math.pi))


def test_measured_tire_feeds_the_target_chain():
    t = tf.update_tire(tf.FieldTire(peak_mu=1.40))
    assert t.peak_mu == 1.40
    assert td.targets(td.Vehicle(), t)["anti_squat_floor_pct"] < td.targets()["anti_squat_floor_pct"]


def test_camber_optimum_is_carried_through_gain_and_roll():
    V = td.Vehicle()
    phi, z = td.roll(V, 1.4)
    t = tf.update_tire(tf.FieldTire(static_camber_opt_deg=-2.4, camber_gain_deg_per_mm=-0.037,
                                    skidpad_ay_g=1.4))
    assert t.camber_opt_deg == pytest.approx(-2.4 - 0.037 * z + phi)


def test_threshold_reproduces_the_paper():
    assert tf.mu_threshold_for("anti_squat_floor_pct", 34.28) == pytest.approx(1.671, abs=0.002)
