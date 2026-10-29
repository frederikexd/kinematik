"""suspension/halfshaft.py against hand calculations: tripod plunge friction mu T / r, the CV secondary couple
T tan(beta/2) about the normal to the two shafts, the per-shaft torques from the axle loads, the tube shear stress, and
the Goodman diameter, whose utilisation is recomputed here from Shigley's Marin factors."""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from suspension import halfshaft as hs


def test_the_spin_axis_leans_with_camber_and_toe():
    a = hs._spin_axis(-2.0, 0.5)
    v = np.array([math.tan(math.radians(0.5)), 1.0, math.tan(math.radians(2.0))])
    assert a == pytest.approx(v / np.linalg.norm(v)) and hs._spin_axis(0.0, 0.0) == pytest.approx([0, 1, 0])


def test_the_spec_defaults_and_refusals():
    s = hs.HalfshaftSpec((10, 200, 220))
    assert (s.tripod_pitch_radius_mm, s.mu_plunge, s.provenance, s.inboard_joint_mm) == \
        (20.0, 0.03, "declared", (10.0, 200.0, 220.0))
    assert all(type(c) is float for c in s.inboard_joint_mm)
    hs.HalfshaftSpec((0, 0, 0), tripod_pitch_radius_mm=1e-9, mu_plunge=0.0)
    for kw in (dict(tripod_pitch_radius_mm=0.0), dict(mu_plunge=-1e-9)):
        with pytest.raises(ValueError) as e:
            hs.HalfshaftSpec((0, 0, 0), **kw)
        assert str(e.value) == "HalfshaftSpec: pitch radius > 0 and mu >= 0 needed."


def test_plunge_friction_and_the_cv_couple():
    spec = hs.HalfshaftSpec((20.0, 180.0, 200.0), tripod_pitch_radius_mm=18.0, mu_plunge=0.04)
    st = SimpleNamespace(wheel_center=(0.0, 600.0, 228.0), camber=-1.5, toe=0.3)
    r = hs.halfshaft_loads(spec, st, -350.0)
    u = np.array([0.0, 600.0, 228.0]) - [20.0, 180.0, 200.0]
    u /= np.linalg.norm(u)
    s = hs._spin_axis(-1.5, 0.3)
    beta = math.acos(abs(u @ s))
    T = 350e3
    assert r["shaft_angle_deg"] == pytest.approx(math.degrees(beta))
    assert r["plunge_force_N"] == pytest.approx(0.04 * T / 18.0)
    assert r["couple_Nmm"] == pytest.approx(T * math.tan(beta / 2))
    n = np.cross(s, u)
    n /= np.linalg.norm(n)
    for case, sgn in zip(r["cases"], (1.0, -1.0)):
        assert case["F_wc_extra"] == pytest.approx(tuple(sgn * 0.04 * T / 18.0 * u))
        assert case["M_wc_extra"] == pytest.approx(tuple(T * math.tan(beta / 2) * n))
    straight = SimpleNamespace(wheel_center=(0.0, 600.0, 200.0), camber=0.0, toe=0.0)
    r = hs.halfshaft_loads(hs.HalfshaftSpec((0.0, 180.0, 200.0)), straight, 100.0)
    assert r["shaft_angle_deg"] == pytest.approx(0.0, abs=1e-6) and r["cases"][0]["M_wc_extra"] == (0.0, 0.0, 0.0)
    with pytest.raises(ValueError) as e:
        hs.halfshaft_loads(hs.HalfshaftSpec((0.0, 600.0, 200.0)), straight, 100.0)
    assert str(e.value) == "halfshaft: inboard joint coincides with the wheel centre."


def test_shaft_torques_from_the_axle_loads():
    r = hs.shaft_torques_nm()
    g = 9.81
    drive = 300.0 * g * 1.06 / 2 * 0.228
    brake = 0.40 * 300.0 * g * 1.5 / 2 * 0.228
    assert r == pytest.approx({"drive_Nm": drive, "brake_Nm": brake, "peak_Nm": max(drive, brake),
                               "reversing_range_Nm": drive + brake})
    r = hs.shaft_torques_nm(mass_kg=250.0, wheel_radius_mm=203.0, traction_g=0.5, drive_share=0.5, braking_g=2.0,
                            rear_brake_share=0.3)
    drive, brake = 0.5 * 250 * g * 0.5 / 2 * 0.203, 0.3 * 250 * g * 2.0 / 2 * 0.203
    assert r == pytest.approx({"drive_Nm": drive, "brake_Nm": brake, "peak_Nm": brake,
                               "reversing_range_Nm": drive + brake})


def test_shaft_shear_is_tc_over_j():
    assert hs.shaft_shear_mpa(-500.0, 25.0, 15.0) == pytest.approx(
        500e3 * 12.5 / (math.pi * (25.0 ** 4 - 15.0 ** 4) / 32.0))
    assert hs.shaft_shear_mpa(500.0, 25.0) == pytest.approx(16 * 500e3 / (math.pi * 25.0 ** 3))


def _goodman(d, t_max, t_min, sut, fos, surface, kc, vm, ke, kf, ratio):
    a, b = {"ground": (1.58, -0.085), "machined": (4.51, -0.265), "hot_rolled": (57.7, -0.718)}[surface]
    ka = a * sut ** b
    kb = 1.24 * d ** -0.107 if d <= 51 else 1.51 * d ** -0.157
    se = ka * kb * kc * ke * ratio * min(sut, 1400.0)
    k = math.sqrt(3) if vm else 1.0
    sa = k * 16 * abs(0.5 * (t_max - t_min)) * 1e3 / (math.pi * d ** 3)
    sm = k * 16 * abs(0.5 * (t_max + t_min)) * 1e3 / (math.pi * d ** 3)
    return fos * (kf * sa / se + sm / sut), ka, kb, se


@pytest.mark.parametrize("kw", [
    dict(), dict(surface="ground", sut_mpa=1600.0), dict(surface="hot_rolled", von_mises=False, kc=0.59),
    dict(fos=2.0, ke=0.897, kf=1.4, se_ratio=0.45), dict(sut_mpa=700.0, t_max=2500.0, t_min=-2500.0)])
def test_the_goodman_diameter_is_where_the_utilisation_reaches_one(kw):
    t_max, t_min = kw.pop("t_max", 600.0), kw.pop("t_min", -300.0)
    p = dict(sut_mpa=1000.0, fos=1.5, surface="machined", kc=1.0, von_mises=True, ke=1.0, kf=1.0, se_ratio=0.504)
    p.update(kw)
    r = hs.goodman_min_diameter(t_max, t_min, **kw)
    d = r["d_min_mm"]
    args = (t_max, t_min, p["sut_mpa"], p["fos"], p["surface"], p["kc"], p["von_mises"], p["ke"], p["kf"],
            p["se_ratio"])
    u, ka, kb, se = _goodman(d, *args)
    assert u <= 1.0 and _goodman(d * (1 - 1e-9), *args)[0] > 1.0 - 1e-6
    assert u == pytest.approx(1.0, abs=1e-9)
    assert r == pytest.approx({"d_min_mm": d, "T_mean_nm": 0.5 * (t_max + t_min), "T_alt_nm": 0.5 * (t_max - t_min),
                               "ka": ka, "kb": kb, "kc": p["kc"], "ke": p["ke"], "kf": p["kf"], "Se_mpa": se})


def test_the_search_stays_inside_its_bracket_and_switches_the_size_factor_above_51_mm():
    r = hs.goodman_min_diameter(20000.0, -20000.0)
    assert 51.0 < r["d_min_mm"] <= 60.0 and r["kb"] == pytest.approx(1.51 * r["d_min_mm"] ** -0.157)
    assert hs.goodman_min_diameter(1.0, 0.0)["d_min_mm"] == pytest.approx(10.0, abs=1e-9)
    assert hs.goodman_min_diameter(1.0, 0.0, d_lo=5.0, d_hi=8.0)["d_min_mm"] == pytest.approx(5.0, abs=1e-9)
    assert hs.goodman_min_diameter(1e6, 0.0, d_hi=70.0)["d_min_mm"] == pytest.approx(70.0)


def test_the_size_factor_switches_above_51_mm():
    at = lambda d: hs.goodman_min_diameter(1.0, 0.0, d_lo=d, d_hi=d)["kb"]
    assert at(51.0) == pytest.approx(1.24 * 51.0 ** -0.107, rel=1e-12)
    assert at(51.5) == pytest.approx(1.51 * 51.5 ** -0.157, rel=1e-12)
