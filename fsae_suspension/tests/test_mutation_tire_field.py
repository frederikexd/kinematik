"""suspension/tire_field.py: friction from skidpad runs (v = 2 pi R / t, a = v^2 / R, downforce removed), the camber
optimum as the vertex of a parabola through lap times, the tire rate as a load-deflection slope with the wheel-hop
frequency, and the substitution of measured inputs into the target chain."""
import math
from dataclasses import replace

import numpy as np
import pytest

from suspension import target_derivation as td
from suspension import tire_field as tf


def test_mu_from_lap_times_and_from_logged_acceleration():
    r = tf.mu_from_skidpad(8.5, lap_times_s=[5.10, 4.95, 5.02])
    v = 2 * math.pi * 8.5 / 4.95
    ay = v * v / 8.5 / 9.81
    assert r == pytest.approx({"mu": ay, "ay_g": ay, "speed_m_s": v,
                               "mu_spread": ay - (2 * math.pi * 8.5 / 5.10) ** 2 / 8.5 / 9.81, "n_runs": 3})
    d = tf.mu_from_skidpad(8.5, lap_times_s=[4.95], mass_kg=280.0, downforce_ClA_m2=3.0, rho=1.2)
    down = 0.5 * 1.2 * 3.0 * v * v
    assert d["mu"] == pytest.approx(280 * ay * 9.81 / (280 * 9.81 + down)) and d["mu_spread"] == 0.0
    a = tf.mu_from_skidpad(9.0, ay_g=[1.3, 1.42], downforce_ClA_m2=2.0)
    v = math.sqrt(1.42 * 9.81 * 9.0)
    assert a == pytest.approx({"mu": 300 * 1.42 * 9.81 / (300 * 9.81 + 0.5 * 1.225 * 2.0 * v * v), "ay_g": 1.42,
                               "speed_m_s": v, "mu_spread": a["mu_spread"], "n_runs": 2})
    v1 = math.sqrt(1.3 * 9.81 * 9.0)
    assert a["mu_spread"] == pytest.approx(a["mu"] - 300 * 1.3 * 9.81 / (300 * 9.81 + 0.5 * 1.225 * 2.0 * v1 * v1))
    assert tf.mu_from_skidpad(9.0, ay_g=[1.4])["mu"] == pytest.approx(1.4)
    for kw, msg in ((dict(lap_times_s=[5.0, 0.0]), "lap times must be > 0 s"), (dict(), "give lap_times_s or ay_g"),
                    (dict(lap_times_s=[], ay_g=[]), "give lap_times_s or ay_g")):
        with pytest.raises(ValueError) as e:
            tf.mu_from_skidpad(8.5, **kw)
        assert str(e.value) == msg
    assert tf.G0 == 9.81


def test_the_camber_optimum_from_a_lap_time_sweep():
    c = [-3.0, -2.0, -1.0, 0.0]
    t = [5.2 + 0.03 * (x + 1.6) ** 2 for x in c]
    r = tf.camber_optimum_from_sweep(c, t)
    assert r == pytest.approx({"static_camber_opt_deg": -1.6, "lap_time_at_opt_s": 5.2, "fit_rms_s": 0.0}, abs=1e-9)
    noisy = tf.camber_optimum_from_sweep(c, [x + e for x, e in zip(t, (0.002, -0.001, 0.001, -0.002))])
    a, b, k = np.polyfit(c, [x + e for x, e in zip(t, (0.002, -0.001, 0.001, -0.002))], 2)
    assert noisy["static_camber_opt_deg"] == pytest.approx(-b / (2 * a)) and noisy["fit_rms_s"] > 0
    for cc, tt, msg in (([-2.0, -1.0], [5.0, 5.1], "need at least three camber settings"),
                        ([-3.0, -2.0, -1.0], [5.0, 5.2, 5.0], "lap time has no minimum in this sweep (not convex)"),
                        ([-3.0, -2.0, -1.0], [5.3, 5.1, 5.0], None)):
        with pytest.raises(ValueError) as e:
            tf.camber_optimum_from_sweep(cc, tt)
        if msg:
            assert str(e.value) == msg
        else:
            a, b, _ = np.polyfit(cc, tt, 2)
            assert str(e.value) == (f"fitted optimum {-b / (2 * a):.2f} deg lies outside the tested range -3.00 to "
                                    f"-1.00; widen the sweep")
    edge = tf.camber_optimum_from_sweep([-3.0, -2.0, -1.0], [5.0 + 0.1 * (x + 1.0) ** 2 for x in (-3.0, -2.0, -1.0)])
    assert edge["static_camber_opt_deg"] == pytest.approx(-1.0)


def test_the_tire_rate_from_a_load_test():
    r = tf.tire_rate_from_load_test([0.0, 500.0, 1000.0], [0.0, 4.0, 8.0])
    assert r == pytest.approx({"tire_rate_N_per_mm": 125.0, "fit_rms_N": 0.0}, abs=1e-9)
    h = tf.tire_rate_from_load_test([0.0, 500.0, 1000.0], [0.0, 4.0, 8.0], unsprung_corner_kg=11.0,
                                    wheel_rate_N_per_mm=30.0)
    assert h["wheel_hop_hz"] == pytest.approx(math.sqrt(155e3 / 11.0) / (2 * math.pi))
    h0 = tf.tire_rate_from_load_test([0.0, 500.0, 1000.0], [0.0, 4.0, 8.0], unsprung_corner_kg=11.0)
    assert h0["wheel_hop_hz"] == pytest.approx(math.sqrt(125e3 / 11.0) / (2 * math.pi))
    n = tf.tire_rate_from_load_test([0.0, 510.0, 990.0], [0.0, 4.0, 8.0])
    k, c = np.polyfit([0.0, 4.0, 8.0], [0.0, 510.0, 990.0], 1)
    assert n["fit_rms_N"] == pytest.approx(math.sqrt(np.mean((np.array([0, 510, 990]) - (k * np.array([0, 4, 8]) + c))
                                                              ** 2)))
    with pytest.raises(ValueError) as e:
        tf.tire_rate_from_load_test([100.0], [1.0])
    assert str(e.value) == "need at least two load points"


def test_measured_inputs_replace_the_synthetic_ones():
    f = tf.FieldTire()
    assert (f.peak_mu, f.static_camber_opt_deg, f.camber_gain_deg_per_mm, f.skidpad_ay_g) == (None,) * 4
    assert tf.update_tire(f) == td.Tire()
    assert tf.update_tire(tf.FieldTire(peak_mu=1.42)) == replace(td.Tire(), peak_mu=1.42)
    t = tf.update_tire(tf.FieldTire(static_camber_opt_deg=-2.2, camber_gain_deg_per_mm=0.02, skidpad_ay_g=1.35))
    phi, z = td.roll(td.Vehicle(), 1.35)
    assert t.camber_opt_deg == pytest.approx(-2.2 + 0.02 * z + phi) and t.peak_mu == 1.55
    soft = replace(td.Vehicle(), roll_gradient_deg_per_g=1.1)
    t2 = tf.update_tire(tf.FieldTire(static_camber_opt_deg=-2.2, camber_gain_deg_per_mm=0.02, skidpad_ay_g=1.35),
                        base=replace(td.Tire(), peak_mu=1.6), veh=soft)
    phi, z = td.roll(soft, 1.35)
    assert t2.camber_opt_deg == pytest.approx(-2.2 + 0.02 * z + phi) and t2.peak_mu == 1.6
    for kw in (dict(static_camber_opt_deg=-2.0, skidpad_ay_g=1.3), dict(static_camber_opt_deg=-2.0,
                                                                         camber_gain_deg_per_mm=0.02)):
        with pytest.raises(ValueError) as e:
            tf.update_tire(tf.FieldTire(**kw))
        assert str(e.value) == "the camber optimum needs the tested corner's gain and the skidpad lateral acceleration"


def test_comparing_the_targets():
    new = replace(td.Tire(), peak_mu=1.7)
    corners = [td.Corner("F", -2.5, 0.02, 25.0, "front"), td.Corner("R", -2.6, 0.018, 20.0, "rear")]
    r = tf.compare_targets(new, corners)
    a, b = td.targets(td.Vehicle(), td.Tire()), td.targets(td.Vehicle(), new)
    assert r["targets"] == {k: {"synthetic": a[k], "measured": b[k]} for k in a}
    for c, floor in ((corners[0], b["anti_dive_floor_pct"]), (corners[1], b["anti_squat_floor_pct"])):
        assert r["corners"][c.name] == pytest.approx({
            "anti_margin_pct": c.anti_pct - floor,
            "camber_error_deg": td.loaded_camber_error(td.Vehicle(), new, c.static_camber_deg, c.camber_gain_deg_per_mm),
            "static_to_restore_deg": td.static_camber_needed(td.Vehicle(), new, c.camber_gain_deg_per_mm)})
    other = tf.compare_targets(new, corners[:1], base=replace(td.Tire(), peak_mu=1.4), veh=replace(td.Vehicle(),
                                                                                                    mass_kg=280.0))
    assert other["targets"]["anti_dive_floor_pct"]["synthetic"] == td.targets(
        replace(td.Vehicle(), mass_kg=280.0), replace(td.Tire(), peak_mu=1.4))["anti_dive_floor_pct"]


def test_the_friction_at_which_an_anti_stops_clearing_its_floor():
    m = tf.mu_threshold_for("anti_dive_floor_pct", 25.0)
    f = lambda mu: td.targets(td.Vehicle(), replace(td.Tire(), peak_mu=mu))["anti_dive_floor_pct"]
    assert 0.5 < m < 3.0 and f(m) < 25.0 <= f(m + 1e-9)
    m2 = tf.mu_threshold_for("anti_squat_floor_pct", 15.0, veh=replace(td.Vehicle(), mass_kg=280.0),
                             base=replace(td.Tire(), camber_opt_deg=-2.0))
    f2 = lambda mu: td.targets(replace(td.Vehicle(), mass_kg=280.0),
                               replace(td.Tire(), camber_opt_deg=-2.0, peak_mu=mu))["anti_squat_floor_pct"]
    assert f2(m2) < 15.0 <= f2(m2 + 1e-9)
    assert tf.mu_threshold_for("anti_dive_floor_pct", 1e6) == pytest.approx(3.0, abs=1e-9)


def test_the_camber_fit_rms_on_scattered_times():
    c = [-3.0, -2.5, -2.0, -1.5, -1.0]
    t = [5.31, 5.22, 5.205, 5.21, 5.27]
    a, b, k = np.polyfit(c, t, 2)
    r = np.array(t) - np.polyval([a, b, k], c)
    assert tf.camber_optimum_from_sweep(c, t)["fit_rms_s"] == pytest.approx(math.sqrt(np.mean(r ** 2)), rel=1e-9)


def test_sub_second_laps_on_a_tiny_radius_and_a_two_point_rate():
    r = tf.mu_from_skidpad(0.1, lap_times_s=[0.9])
    v = 2 * math.pi * 0.1 / 0.9
    assert r["speed_m_s"] == pytest.approx(v)
    k = tf.tire_rate_from_load_test([1000.0, 1500.0], [5.0, 7.0])
    assert k["tire_rate_N_per_mm"] == pytest.approx(250.0)
