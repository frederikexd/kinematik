"""suspension/damper.py: the bilinear-digressive force law on both sides of the knee in bump and rebound, the dyno
fit recovering slopes from points generated with known ones, and the damping ratio against c MR^2 / (2 sqrt(k m))
with the tyre in series."""
import math

import numpy as np
import pytest

from suspension.damper import DamperCurve, damping_ratio, default_damper


def test_the_representative_defaults_are_flagged():
    d = default_damper()
    assert (d.c_bump_low, d.c_bump_high, d.c_reb_low, d.c_reb_high, d.v_knee, d.is_calibrated, d.source) == \
        (6000.0, 2000.0, 9000.0, 3000.0, 0.05, False, "representative")
    assert d.status() == ("UNCALIBRATED — representative FSAE magnitudes (load your damper dyno points to make these "
                          "quantitative; the force law is real)")
    assert DamperCurve(is_calibrated=True).status() == "calibrated to dyno data"


def test_the_force_law_on_each_side_of_the_knee():
    d = DamperCurve(c_bump_low=5000.0, c_bump_high=1500.0, c_reb_low=8000.0, c_reb_high=2500.0, v_knee=0.04)
    assert d.force(0.03) == pytest.approx(150.0) and type(d.force(0.03)) is float
    assert d.force(0.10) == pytest.approx(5000 * 0.04 + 1500 * 0.06)
    assert d.force(-0.03) == pytest.approx(-240.0)
    assert d.force(-0.10) == pytest.approx(-(8000 * 0.04 + 2500 * 0.06))
    assert d.force(0.0) == 0.0
    v = np.array([-0.1, -0.02, 0.0, 0.02, 0.1])
    assert d.force(v) == pytest.approx([-470.0, -160.0, 0.0, 100.0, 290.0])
    tiny = DamperCurve(c_bump_low=5000.0, c_bump_high=1500.0, v_knee=0.0)      # the knee is floored at 0.1 mm/s
    assert tiny.force(0.01) == pytest.approx(5000 * 1e-4 + 1500 * (0.01 - 1e-4))
    v, f = d.curve_points()
    assert v.shape == (101,) and v[0] == -0.5 and v[-1] == 0.5 and f == pytest.approx(d.force(v))
    v, f = d.curve_points(v_max=0.2, n=5)
    assert v == pytest.approx([-0.2, -0.1, 0.0, 0.1, 0.2])


def test_the_dyno_fit_recovers_the_slopes_it_was_given():
    true = DamperCurve(c_bump_low=5500.0, c_bump_high=1800.0, c_reb_low=8500.0, c_reb_high=2600.0, v_knee=0.06)
    v = np.array([-0.3, -0.2, -0.05, -0.02, 0.02, 0.05, 0.2, 0.3])
    fit = DamperCurve.from_dyno_points(v, true.force(v), v_knee=0.06)
    assert (fit.c_bump_low, fit.c_bump_high, fit.c_reb_low, fit.c_reb_high) == pytest.approx(
        (5500.0, 1800.0, 8500.0, 2600.0))
    assert (fit.v_knee, fit.is_calibrated, fit.source) == (0.06, True, "dyno")
    at_knee = DamperCurve.from_dyno_points([0.03, 0.05, 0.06, 0.1, 0.2, -0.06, -0.03, -0.1, -0.2],
                                           true.force(np.array([0.03, 0.05, 0.06, 0.1, 0.2, -0.06, -0.03, -0.1,
                                                                -0.2])), v_knee=0.06)
    assert at_knee.c_bump_low == pytest.approx(5500.0) and at_knee.c_reb_low == pytest.approx(8500.0)
    default_knee = DamperCurve.from_dyno_points(v, DamperCurve(v_knee=0.05).force(v))
    assert default_knee.v_knee == 0.05 and default_knee.c_bump_high == pytest.approx(2000.0)


def test_a_region_with_too_few_points_keeps_the_default_slope():
    one_each = DamperCurve.from_dyno_points([0.02, 0.2, -0.02, -0.2, 0.03], [100, 500, -180, -700, 150])
    assert (one_each.c_bump_high, one_each.c_reb_low, one_each.c_reb_high) == (2000.0, 9000.0, 3000.0)
    assert one_each.c_bump_low == pytest.approx((0.02 * 100 + 0.03 * 150) / (0.02 ** 2 + 0.03 ** 2))
    assert DamperCurve.from_dyno_points([0.1, 0.2, 0.3], [1, 2, 3]) == DamperCurve()
    assert DamperCurve.from_dyno_points([0.1, 0.2, 0.3, 0.4], [1, 2, 3]) == DamperCurve()
    neg = DamperCurve.from_dyno_points([0.01, 0.02, 0.2, 0.3, -0.01, -0.02, -0.2, -0.3],
                                       [-10, -20, 0, 0, 10, 20, 0, 0])
    assert (neg.c_bump_low, neg.c_reb_low) == (0.0, 0.0)            # a negative slope is clipped to zero
    assert neg.c_bump_high == pytest.approx(0.0) and neg.c_reb_high == pytest.approx(0.0)
    tiny = DamperCurve.from_dyno_points([1e-7, 2e-7, 0.2, 0.3], [1.0, 2.0, 300.0, 400.0], v_knee=0.0)
    assert tiny.v_knee == 1e-4 and tiny.c_bump_low == 0.0           # no usable spread: slope 0, not a division by 0


def test_the_damping_ratio_uses_the_ride_rate_and_the_motion_ratio():
    d = DamperCurve(c_bump_low=5000.0, c_reb_low=8000.0)
    z = damping_ratio(d, 60.0, 35.0, motion_ratio=0.9)
    assert z == pytest.approx(5000 * 0.81 / (2 * math.sqrt(35e3 * 60.0)))
    zr = damping_ratio(d, 60.0, 35.0, motion_ratio=0.9, region="rebound", tire_rate_N_per_mm=130.0)
    k_ride = 35.0 * 130.0 / 165.0
    assert zr == pytest.approx(8000 * 0.81 / (2 * math.sqrt(k_ride * 1e3 * 60.0)))
    assert damping_ratio(d, 60.0, 35.0, tire_rate_N_per_mm=0.0) == damping_ratio(d, 60.0, 35.0)
    assert damping_ratio(d, 60.0, 35.0) == pytest.approx(5000 / (2 * math.sqrt(35e3 * 60.0)))
    assert damping_ratio(d, 0.0, 0.0) == pytest.approx(5000 / (2 * math.sqrt(1e-6 * 1e3 * 1e-3)))


def _fit(points, vk=0.05):
    v, f = zip(*points)
    return DamperCurve.from_dyno_points(list(v), list(f), v_knee=vk)


def test_the_region_edges_of_the_dyno_fit():
    d = DamperCurve()
    # a zero-velocity reading with an offset force belongs to neither side; the knee point is low-speed
    c = _fit([(0.0, 30.0), (0.02, 40.0), (0.05, 90.0), (0.2, 500.0), (-0.0, 30.0), (-0.02, -50.0), (-0.05, -80.0),
              (-0.3, -700.0)])
    low = (0.02 * 40.0 + 0.05 * 90.0) / (0.02 ** 2 + 0.05 ** 2)
    reb = (0.02 * 50.0 + 0.05 * 80.0) / (0.02 ** 2 + 0.05 ** 2)
    assert c.c_bump_low == pytest.approx(low, rel=1e-12) and c.c_reb_low == pytest.approx(reb, rel=1e-12)
    # one point past each knee is too few: the knee point itself must not count toward the high-speed fit
    assert (c.c_bump_high, c.c_reb_high) == (d.c_bump_high, d.c_reb_high)


def test_any_positive_tyre_rate_is_put_in_series():
    c = DamperCurve()
    soft = damping_ratio(c, 60.0, 30.0, 1.0, tire_rate_N_per_mm=0.5)
    k = 30.0 * 0.5 / 30.5
    assert soft == pytest.approx(c.c_bump_low / (2 * math.sqrt(k * 1000 * 60.0)), rel=1e-12)
