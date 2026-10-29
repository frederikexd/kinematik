"""suspension/brake_bias.py against the hydraulics written out by hand: pressure = pedal force x ratio x bar share /
master-cylinder area, clamp = pressure x piston area, axle torque = calipers x 2 faces x mu x clamp x radius, with
every part given a distinct value so each term shows; the fade law moves the bias, and the travel check prices the
compliance per circuit."""
import math

import numpy as np
import pytest

from suspension import brake_bias as bb
from suspension.brake_thermal import BrakeThermalParams

FADE_F = BrakeThermalParams(enable_fade=True, T_fade_onset_c=400.0, fade_per_C=0.001, fade_floor=0.5)
FADE_R = BrakeThermalParams(enable_fade=True, T_fade_onset_c=300.0, fade_per_C=0.002, fade_floor=0.6)
SYS = bb.BrakeHydraulics(pedal_ratio=4.5, bar_front=0.58, mc_bore_front_mm=15.0, mc_bore_rear_mm=19.0,
                         piston_front_mm=27.0, piston_rear_mm=24.0, pistons_per_side_front=2, pistons_per_side_rear=1,
                         r_eff_front_mm=88.0, r_eff_rear_mm=71.0, mu_pad_front=0.48, mu_pad_rear=0.41,
                         fade_front=FADE_F, fade_rear=FADE_R, calipers_per_axle=3)


def _torques(F, tf, tr, s=SYS):
    a = lambda d: math.pi * d * d / 4
    muf = s.mu_pad_front * s.fade_front.mu_fade(tf)
    mur = s.mu_pad_rear * s.fade_rear.mu_fade(tr)
    Tf = s.calipers_per_axle * 2 * muf * (F * s.pedal_ratio * s.bar_front / a(s.mc_bore_front_mm)) \
        * (s.pistons_per_side_front * a(s.piston_front_mm)) * s.r_eff_front_mm / 1000
    Tr = s.calipers_per_axle * 2 * mur * (F * s.pedal_ratio * (1 - s.bar_front) / a(s.mc_bore_rear_mm)) \
        * (s.pistons_per_side_rear * a(s.piston_rear_mm)) * s.r_eff_rear_mm / 1000
    return Tf, Tr


def test_the_axle_torques_and_the_bias():
    for tf, tr in ((30.0, 30.0), (500.0, 250.0), (450.0, 420.0), (900.0, 900.0)):
        want = _torques(700.0, tf, tr)
        assert SYS.axle_torques_Nm(700.0, tf, tr) == pytest.approx(want, rel=1e-12)
        assert SYS.bias(tf, tr) == pytest.approx(want[0] / sum(want), rel=1e-12)
    assert SYS.axle_torques_Nm(700.0) == pytest.approx(_torques(700.0, 30.0, 30.0))
    assert SYS.bias() == pytest.approx(SYS.bias(30.0, 30.0)) and SYS.bias(t_rear_c=500.0) > SYS.bias()
    assert SYS.bias(t_front_c=500.0) < SYS.bias()
    assert bb._area(10.0) == pytest.approx(25 * math.pi)


def test_the_defaults():
    d = bb.BrakeHydraulics()
    assert (d.pedal_ratio, d.bar_front, d.mc_bore_front_mm, d.mc_bore_rear_mm, d.piston_front_mm, d.piston_rear_mm,
            d.pistons_per_side_front, d.pistons_per_side_rear, d.r_eff_front_mm, d.r_eff_rear_mm, d.mu_pad_front,
            d.mu_pad_rear, d.calipers_per_axle) == (4.0, 0.55, 15.875, 17.78, 25.4, 25.4, 2, 1, 85.0, 75.0, 0.45, 0.45,
                                                    2)
    assert d.fade_front == BrakeThermalParams() and d.fade_rear == BrakeThermalParams()
    assert d.fade_front is not d.fade_rear
    assert d.bias(900.0, 30.0) == d.bias()                        # fade off by default


def test_the_bar_that_gives_a_target_bias():
    for target, tf, tr in ((0.62, 30.0, 30.0), (0.7, 600.0, 350.0)):
        bar = SYS.bar_for_bias(target, tf, tr)
        hit = bb.BrakeHydraulics(**{**SYS.__dict__, "bar_front": bar})
        assert hit.bias(tf, tr) == pytest.approx(target, abs=1e-12)
        assert 0.05 < bar < 0.95
    assert SYS.bar_for_bias(0.62) == pytest.approx(SYS.bar_for_bias(0.62, 30.0, 30.0), abs=1e-14)
    assert SYS.bar_for_bias(0.9999) == pytest.approx(0.95, abs=1e-12)           # out of reach: the end stop
    assert SYS.bar_for_bias(0.0001) == pytest.approx(0.05, abs=1e-12)


def test_the_drift_along_a_stint():
    tf, tr = [30.0, 450.0, 650.0, 500.0], [30.0, 280.0, 420.0, 350.0]
    r = bb.bias_drift(SYS, tf, tr)
    want = [SYS.bias(a, c) for a, c in zip(tf, tr)]
    assert r["bias"] == pytest.approx(want) and r["min"] == pytest.approx(min(want))
    assert r["max"] == pytest.approx(max(want)) and r["cold"] == pytest.approx(want[0]) and r["min"] != r["max"]
    one = bb.bias_drift(SYS, 500.0, 200.0)
    assert one["bias"] == pytest.approx([SYS.bias(500.0, 200.0)]) and one["cold"] == one["min"] == one["max"]


def test_anti_dive_scales_with_the_front_share():
    assert bb.anti_dive_window(30.0, 0.6, 20.0, 40.0) == pytest.approx((0.4, 0.8))
    assert bb.anti_dive_over_bias(30.0, 0.6, [0.6, 0.5, 0.7]) == pytest.approx([30.0, 25.0, 35.0])


def test_the_pedal_travel_compliance_costs():
    r = bb.pedal_travel_mm(SYS, 60.0, caliper_compliance_mm3_per_bar=5.0, line_compliance_mm3_per_bar=3.0)
    v = (3 * 5.0 + 3.0) * 60.0
    assert r == pytest.approx({"front": 4.5 * v / (math.pi * 15.0 ** 2 / 4), "rear": 4.5 * v / (math.pi * 19.0 ** 2 / 4)})
    d = bb.pedal_travel_mm(SYS, 60.0)
    v = (3 * 4.0 + 2.0) * 60.0
    assert d == pytest.approx({"front": 4.5 * v / (math.pi * 15.0 ** 2 / 4), "rear": 4.5 * v / (math.pi * 19.0 ** 2 / 4)})
    assert list(d) == ["front", "rear"] and np.isfinite(list(d.values())).all()


def test_the_default_rear_temperature_and_the_cold_sample_use_the_rear_trace():
    low = bb.BrakeHydraulics(fade_front=BrakeThermalParams(enable_fade=True, T_fade_onset_c=20.0, fade_per_C=0.01),
                             fade_rear=BrakeThermalParams(enable_fade=True, T_fade_onset_c=20.0, fade_per_C=0.02))
    assert low.bias() == pytest.approx(low.bias(30.0, 30.0)) and low.bias() != pytest.approx(low.bias(30.0, 31.0))
    assert low.axle_torques_Nm(500.0) == pytest.approx(low.axle_torques_Nm(500.0, 30.0, 30.0))
    assert low.bar_for_bias(0.6) == pytest.approx(low.bar_for_bias(0.6, 30.0, 30.0), abs=1e-12)
    assert low.bar_for_bias(0.6, 30.0) != pytest.approx(low.bar_for_bias(0.6, 30.0, 31.0), abs=1e-9)
    r = bb.bias_drift(SYS, [350.0, 500.0], [320.0, 200.0])
    assert r["cold"] == pytest.approx(SYS.bias(350.0, 320.0)) != pytest.approx(SYS.bias(350.0, 30.0))
