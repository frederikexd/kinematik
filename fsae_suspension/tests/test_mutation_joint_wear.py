"""suspension/joint_wear.py against Archard's law worked by hand, the deadband line it implies, and the least-squares
wear rate recovered from clearances generated with a known rate."""
import math

import numpy as np
import pytest

from suspension import joint_wear as jw
from suspension.kinematics import Hardpoints

HP = Hardpoints.default()


def _spec(**kw):
    duty = [jw.JointDuty(load_n=-1500.0, swing_deg=-4.0, cycles_per_km=600.0),
            jw.JointDuty(load_n=800.0, swing_deg=10.0, cycles_per_km=50.0)]
    return jw.JointWearSpec(member="TR", duty=duty, **kw)


def test_the_documented_defaults():
    s = jw.JointWearSpec(member="UF")
    assert (s.ball_radius_mm, s.bearing_width_mm, s.clearance_new_mm, s.n_joints, s.wear_rate_mm3_per_Nm,
            s.provenance, s.duty) == (7.9, 8.0, 0.025, 2, 5e-7, "declared", [])
    assert s.growth_per_km_mm() == 0.0


def test_growth_is_archard_wear_on_both_sides_at_every_joint():
    s = _spec(ball_radius_mm=6.0, bearing_width_mm=7.0, n_joints=3, wear_rate_mm3_per_Nm=2e-7)
    area = 12.0 * 7.0
    g = 0.0
    for load, swing, n in ((1500.0, 4.0, 600.0), (800.0, 10.0, 50.0)):
        h = 2e-7 * (load / area) * (6.0 * math.radians(swing) / 1000.0)
        g += 2 * h * n
    assert s.growth_per_km_mm() == pytest.approx(3 * g, rel=1e-12)
    assert s.clearance_at_km([0, 500, 2000]) == pytest.approx(0.025 + 3 * g * np.array([0, 500, 2000]))
    assert s.clearance_at_km(250).dtype == np.float64


def test_the_deadband_over_life_is_the_lash_deadband_of_the_worn_clearance():
    from suspension import elastokinematics as ek
    s = _spec(wear_rate_mm3_per_Nm=1e-5)
    life = jw.deadband_over_life(HP, [s], [0.0, 3000.0])
    assert list(life["km"]) == [0.0, 3000.0] and set(life) == set(ek.CHANNELS) | {"km"}
    for i, km in enumerate((0.0, 3000.0)):
        want = ek.lash_deadband(HP, {"TR": float(s.clearance_at_km(km))})
        for ch in ek.CHANNELS:
            assert life[ch][i] == pytest.approx(want[ch], rel=1e-12)
    one = jw.deadband_over_life(HP, [s], 3000.0)
    assert one["toe"].shape == (1,) and one["toe"][0] == life["toe"][1]


def test_distance_to_budget_is_where_the_deadband_line_crosses_it():
    s = _spec(wear_rate_mm3_per_Nm=1e-5)
    d0 = jw.deadband_over_life(HP, [s], [0.0])["toe"][0]
    d1 = jw.deadband_over_life(HP, [s], [1000.0])["toe"][0]
    budget = d0 + 3.7 * (d1 - d0)
    assert jw.km_to_budget(HP, [s], "toe", budget) == pytest.approx(3700.0, rel=1e-9)
    assert jw.km_to_budget(HP, [s], "toe", d0) == 0.0
    assert jw.km_to_budget(HP, [s], "toe", d0 * 0.5) == 0.0
    assert jw.km_to_budget(HP, [s], "toe", budget, km_max=3699.0) == math.inf
    assert jw.km_to_budget(HP, [s], "toe", budget, km_max=3700.5) == pytest.approx(3700.0, rel=1e-9)
    far = d0 + 2e5 * (d1 - d0) / 1000.0                    # 200 000 km: past the default 100 000 km horizon
    assert jw.km_to_budget(HP, [s], "toe", far) == math.inf
    near = d0 + 9e4 * (d1 - d0) / 1000.0
    assert jw.km_to_budget(HP, [s], "toe", near) == pytest.approx(9e4, rel=1e-9)
    still = jw.JointWearSpec(member="TR")                  # no duty: never wears
    assert jw.km_to_budget(HP, [still], "toe", 10.0) == math.inf


def test_the_fitted_wear_rate_recovers_the_rate_that_made_the_data():
    s = _spec(wear_rate_mm3_per_Nm=3.3e-7)
    km = np.array([0.0, 400.0, 1200.0, 2500.0])
    noise = np.array([0.0, 2e-5, -1e-5, 1.5e-5])
    measured = s.clearance_at_km(km) + noise
    g1 = jw.JointWearSpec(**{**s.__dict__, "wear_rate_mm3_per_Nm": 1.0}).growth_per_km_mm()
    want = np.sum(km * (measured - 0.025)) / (g1 * np.sum(km * km))
    assert jw.fit_wear_rate(s, km, measured) == pytest.approx(want, rel=1e-12)
    assert jw.fit_wear_rate(s, km, s.clearance_at_km(km)) == pytest.approx(3.3e-7, rel=1e-12)
    assert s.wear_rate_mm3_per_Nm == 3.3e-7                # the spec itself is not changed
    for bad in ((jw.JointWearSpec(member="TR"), km), (s, [0.0, 0.0])):
        with pytest.raises(ValueError) as e:
            jw.fit_wear_rate(bad[0], bad[1], [0.03, 0.03, 0.03, 0.03][:len(bad[1])])
        assert str(e.value) == "fit_wear_rate needs a declared duty and km > 0."


def test_the_default_horizon_and_a_light_duty_fit():
    s = _spec(wear_rate_mm3_per_Nm=1e-5)
    d0 = jw.deadband_over_life(HP, [s], [0.0])["toe"][0]
    d1 = jw.deadband_over_life(HP, [s], [1000.0])["toe"][0]
    just_past = d0 + 100000.5 * (d1 - d0) / 1000.0
    assert jw.km_to_budget(HP, [s], "toe", just_past) == math.inf
    light = jw.JointWearSpec(member="TR", duty=[jw.JointDuty(load_n=20.0, swing_deg=0.5, cycles_per_km=3.0)],
                             wear_rate_mm3_per_Nm=2e-7)
    unit = jw.JointWearSpec(**{**light.__dict__, "wear_rate_mm3_per_Nm": 1.0}).growth_per_km_mm()
    assert 0.0 < unit < 1.0                                     # a light duty: the unit-rate growth is below 1 mm/km
    km = np.array([500.0, 1000.0])
    assert jw.fit_wear_rate(light, km, light.clearance_at_km(km)) == pytest.approx(2e-7, rel=1e-9)
