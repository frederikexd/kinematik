"""suspension/steering_feel.py with a tire whose lateral force law and peak slip are fixed here, so every metric
(the aligning torque curve, its peak, the torque left at the limit, the warning margin, the ratio window and the trail
at which it closes) is checked against arithmetic on that law."""
import math

import numpy as np
import pytest

from suspension import dynamic_ackermann as da
from suspension import steering_feel as sf


class Tire:
    def __init__(self, scale=1.0):
        self.scale = scale

    def fy(self, alpha_rad, fz):
        return -self.scale * 1.5 * fz * np.sin(1.4 * np.arctan(10.0 * np.asarray(alpha_rad)))


@pytest.fixture(autouse=True)
def _peak(monkeypatch):
    def peak(tire, fz):
        assert isinstance(tire, Tire)
        return 8.0 if fz > 600 else 6.0
    monkeypatch.setattr(da, "peak_slip_deg", peak)


def _M(a_deg, fz, t_mech, t0, a0=8.0):
    fy = abs(Tire().fy(np.radians(a_deg), fz))
    return fy * (t_mech + t0 * np.cos(0.5 * np.pi * a_deg / a0)) / 1000.0


def test_brush_and_mechanical_trail():
    assert sf.brush_trail_mm(1200.0) == pytest.approx(1200.0 / (0.083 * 180.0) / 6.0)
    assert sf.brush_trail_mm(1200.0, inflation_kpa=70.0, patch_width_mm=150.0) == pytest.approx(
        1200.0 / (0.070 * 150.0) / 6.0)
    assert sf.mechanical_trail_mm(5.0) == pytest.approx(228.0 * math.tan(math.radians(5.0)))
    assert sf.mechanical_trail_mm(5.0, loaded_radius_mm=203.0, caster_offset_mm=7.0) == pytest.approx(
        203.0 * math.tan(math.radians(5.0)) - 7.0)


def test_the_aligning_curve():
    c = sf.aligning_curve(Tire(), 1200.0, 20.0)
    a = np.linspace(0.0, 14.0, 281)
    t0 = sf.brush_trail_mm(1200.0)
    assert c["alpha_deg"] == pytest.approx(a) and c["alpha_peak_deg"] == 8.0 and c["t0_mm"] == pytest.approx(t0)
    assert c["Fy_N"] == pytest.approx(np.abs(Tire().fy(np.radians(a), 1200.0)))
    assert c["t_pneu_mm"] == pytest.approx(t0 * np.cos(0.5 * np.pi * a / 8.0))
    assert c["M_Nm"] == pytest.approx(_M(a, 1200.0, 20.0, t0))
    c = sf.aligning_curve(Tire(), 1200.0, 20.0, t0_mm=30.0, alpha_max_deg=10.0, n=11)
    assert c["alpha_deg"] == pytest.approx(np.arange(11.0)) and c["t0_mm"] == 30.0
    assert c["M_Nm"] == pytest.approx(_M(np.arange(11.0), 1200.0, 20.0, 30.0))


def test_the_limit_feel_metrics():
    r = sf.limit_feel(Tire(), 1200.0, 15.0, t0_mm=35.0)
    a = np.linspace(0.0, 14.0, 281)
    M = _M(a, 1200.0, 15.0, 35.0)
    k = int(np.argmax(M))
    lim = float(np.interp(8.0, a, M))
    assert r == pytest.approx({"alpha_peak_deg": 8.0, "t0_mm": 35.0, "torque_max_Nm": M[k],
                               "alpha_torque_max_deg": a[k], "torque_at_limit_Nm": lim,
                               "warning_margin_deg": 8.0 - a[k], "lightening": 1.0 - lim / M[k]})
    assert r["torque_at_limit_Nm"] == pytest.approx(abs(Tire().fy(np.radians(8.0), 1200.0)) * 15.0 / 1000.0)
    assert sf.limit_feel(Tire(), 1200.0, 15.0)["t0_mm"] == pytest.approx(sf.brush_trail_mm(1200.0))
    assert math.isnan(sf.limit_feel(Tire(scale=0.0), 1200.0, 15.0)["lightening"])


def test_the_worst_axle_torque_sums_both_wheels_at_the_same_slip():
    r = sf.worst_axle_torque(Tire(), 1200.0, 400.0, 18.0, t0_outer_mm=40.0)
    a = np.linspace(0.0, 14.0, 281)
    tot = _M(a, 1200.0, 18.0, 40.0) + _M(a, 400.0, 18.0, 40.0 * 400.0 / 1200.0, a0=6.0)
    k = int(np.argmax(tot))
    assert r == pytest.approx({"torque_Nm": tot[k], "alpha_deg": a[k]})
    t0 = sf.brush_trail_mm(1200.0)
    tot = _M(a, 1200.0, 18.0, t0) + _M(a, 400.0, 18.0, t0 / 3.0, a0=6.0)
    assert sf.worst_axle_torque(Tire(), 1200.0, 400.0, 18.0)["torque_Nm"] == pytest.approx(tot.max())


def test_the_ratio_window_and_the_trail_that_closes_it():
    w = sf.worst_axle_torque(Tire(), 1200.0, 400.0, 18.0, t0_outer_mm=40.0)
    r = sf.ratio_window(Tire(), 1200.0, 400.0, 18.0, t0_outer_mm=40.0)
    assert r == pytest.approx({"ratio_min": w["torque_Nm"] / 10.0, "ratio_max": 6.0,
                               "feasible": w["torque_Nm"] / 10.0 <= 6.0, "worst_torque_Nm": w["torque_Nm"],
                               "worst_alpha_deg": w["alpha_deg"]})
    lo = w["torque_Nm"] / 10.0
    assert sf.ratio_window(Tire(), 1200.0, 400.0, 18.0, lock_ratio_max=lo, t0_outer_mm=40.0)["feasible"] is True
    assert sf.ratio_window(Tire(), 1200.0, 400.0, 18.0, lock_ratio_max=lo * (1 - 1e-12),
                           t0_outer_mm=40.0)["feasible"] is False
    r = sf.ratio_window(Tire(), 1200.0, 400.0, 18.0, effort_limit_Nm=4.0, lock_ratio_max=9.0, t0_outer_mm=40.0)
    assert (r["ratio_min"], r["ratio_max"]) == pytest.approx((w["torque_Nm"] / 4.0, 9.0))
    worst = lambda t0: sf.worst_axle_torque(Tire(), 1200.0, 400.0, 18.0, t0)["torque_Nm"]
    effort = 0.5 * (worst(0.0) + worst(80.0)) / 9.0                # the window closes inside (0, 80) mm
    t = sf.trail_limit_mm(Tire(), 1200.0, 400.0, 18.0, effort_limit_Nm=effort, lock_ratio_max=9.0)
    ok = lambda t0: sf.ratio_window(Tire(), 1200.0, 400.0, 18.0, effort, 9.0, t0)["feasible"]
    assert 0.0 < t < 80.0 and ok(t - 1e-9) and not ok(t + 1e-9)
    assert sf.trail_limit_mm(Tire(), 1200.0, 400.0, 18.0, effort_limit_Nm=1e6) == math.inf
    t2 = sf.trail_limit_mm(Tire(), 1200.0, 400.0, 18.0, effort_limit_Nm=effort, lock_ratio_max=9.0, hi=t * 1.5)
    assert t2 == pytest.approx(t, abs=1e-9)
    e10 = 0.5 * (worst(0.0) + worst(80.0)) / 6.0 / 10.0          # scale so the defaults (10 N*m, 6) close it too
    tire = Tire(scale=1.0 / e10)
    t3 = sf.trail_limit_mm(tire, 1200.0, 400.0, 18.0)
    ok10 = lambda t0: sf.ratio_window(tire, 1200.0, 400.0, 18.0, 10.0, 6.0, t0)["feasible"]
    assert 0.0 < t3 < 80.0 and ok10(t3 - 1e-9) and not ok10(t3 + 1e-9)


def test_the_lightening_needs_a_positive_peak_torque():
    assert math.isnan(sf.limit_feel(Tire(), 1200.0, -500.0, t0_mm=35.0)["lightening"])
    r = sf.limit_feel(Tire(), 10.0, 15.0, t0_mm=35.0)
    a = np.linspace(0.0, 14.0, 281)
    M = _M(a, 10.0, 15.0, 35.0, a0=6.0)
    assert 0 < M.max() < 1 and r["lightening"] == pytest.approx(1 - np.interp(6.0, a, M) / M.max(), rel=1e-9)
