# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Supplement S6.34: the tire-dependent results on an R25B-fitted tire."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import r25b_check as rc                                                  # noqa: E402


def test_fit_matches_source_a_peak_mu_and_slip():
    belt = rc.make(rc.COEFFS)
    for fz, mu, slip in rc.SOURCE_A:
        s, m = rc.peak(belt, fz)
        assert round(abs(m - mu), 2) <= 0.05 and round(abs(s - slip), 1) <= 0.5      # as quoted in S6.34
    assert round(belt.mu_peak(1100.0), 2) == 2.40


def test_synthetic_column_reproduces_the_paper():
    hp, trail, ratio = rc.final_front()
    t = rc.tires()["synthetic"]
    pk, a, lim = rc.axle_torque(t, trail)
    assert (round(pk, 1), round(lim, 1), round(pk / ratio, 1)) == (40.5, 27.4, 8.1)
    assert round(rc.trail_limit(t, trail, 10.0 * ratio), 1) == 22.4


def test_car_tire_level_holds_every_wall_narrowly_and_the_belt_fit_does_not():
    hp, trail, ratio = rc.final_front()
    T = rc.tires()
    car, belt = T["r25b_car_tire_belt"], T["r25b_belt"]
    assert round(car.mu_peak(1112.0), 2) == 1.82
    assert car.mu_peak(1100.0) <= rc.MU_THR_REAR < belt.mu_peak(1100.0)
    hand_car = rc.axle_torque(car, trail)[0] / ratio
    hand_belt = rc.axle_torque(belt, trail)[0] / ratio
    assert 9.8 < hand_car < 10.0 < 13.0 <= round(hand_belt, 1)
    for g in (1.5, 1.0):                                                 # Ackermann hardly matters for axle force
        assert rc.ackermann(car, hp, g)["final_front_loss_pct"] < 0.1
