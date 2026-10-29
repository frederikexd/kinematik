"""suspension/tiremodel.py against the MF5.2 pure-lateral equations written out here from the standard (every
coefficient and scaling factor set to a distinct non-trivial value, so each term is visible), the peak searches against
dense sweeps, the coefficient file round trip, the friction ellipse in closed form and first-order relaxation."""
import json
import math

import numpy as np
import pytest

from suspension import tiremodel as tm

C = {"PCY1": 1.4, "PDY1": 1.6, "PDY2": -0.2, "PDY3": 1.1, "PEY1": -0.5, "PEY2": -0.15, "PEY3": 0.12, "PEY4": 1.7,
     "PKY1": -26.0, "PKY2": 1.7, "PKY3": 0.5, "PHY1": 0.004, "PHY2": -0.002, "PHY3": 0.03, "PVY1": 0.01,
     "PVY2": -0.02, "PVY3": 0.11, "PVY4": 0.05}
S = {"LFZO": 1.1, "LCY": 0.95, "LMUY": 1.05, "LEY": 0.9, "LKY": 1.08, "LHY": 0.8, "LVY": 1.2, "LGAY": 0.85}


def _mf52(a, fz, g, c=C, s=S, fnom=1100.0):
    fz = max(fz, 1e-6)
    dfz = (fz - fnom) / fnom
    g = g * s["LGAY"]
    Cy = c["PCY1"] * s["LCY"]
    Dy = (c["PDY1"] + c["PDY2"] * dfz) * (1 - c["PDY3"] * g * g) * s["LMUY"] * fz
    Shy = (c["PHY1"] + c["PHY2"] * dfz) * s["LHY"] + c["PHY3"] * g
    ax = a + Shy
    Ey = min((c["PEY1"] + c["PEY2"] * dfz) * (1 - (c["PEY3"] + c["PEY4"] * g) * np.sign(ax)) * s["LEY"], 1.0)
    Ky = c["PKY1"] * fnom * math.sin(2 * math.atan(fz / (c["PKY2"] * fnom * s["LFZO"]))) * (1 - c["PKY3"] * abs(g)) \
        * s["LKY"]
    By = Ky / (Cy * Dy + 1e-9)
    Svy = fz * ((c["PVY1"] + c["PVY2"] * dfz) * s["LVY"] + (c["PVY3"] + c["PVY4"] * dfz) * g)
    return Dy * math.sin(Cy * math.atan(By * ax - Ey * (By * ax - math.atan(By * ax)))) + Svy


def test_the_lateral_force_is_mf52_term_by_term():
    t = tm.PacejkaLateral(C, FNOMIN=1100.0, scaling=S)
    for a in (-0.2, -0.05, -0.004, 0.0, 0.01, 0.08, 0.25):
        for fz in (300.0, 1100.0, 1900.0):
            for g in (0.0, 0.03, -0.05):
                assert t.fy(a, fz, g) == pytest.approx(_mf52(a, fz, g), rel=1e-12, abs=1e-9)
    al = np.array([-0.1, 0.0, 0.1])
    assert t.fy(al, 800.0) == pytest.approx([_mf52(x, 800.0, 0.0) for x in al])
    assert t.fy(0.1, 0.0) == pytest.approx(_mf52(0.1, 1e-6, 0.0), abs=1e-6)


def test_defaults_scalings_and_the_required_coefficients():
    t = tm.PacejkaLateral({k: C[k] for k in ("PCY1", "PDY1", "PDY2", "PEY1", "PKY1", "PKY2")})
    assert t.FNOMIN == 6306.0 and t.scaling == {k: 1.0 for k in S}
    ones = {k: 1.0 for k in S}
    sparse = {k: (C[k] if k in ("PCY1", "PDY1", "PDY2", "PEY1", "PKY1", "PKY2") else 0.0) for k in C}
    assert t.fy(0.07, 2000.0) == pytest.approx(_mf52(0.07, 2000.0, 0.0, sparse, ones, 6306.0))
    assert tm.PacejkaLateral(C, scaling={"LMUY": 0.5}).scaling == {**ones, "LMUY": 0.5}
    for k in ("PCY1", "PDY1", "PDY2", "PEY1", "PKY1", "PKY2"):
        with pytest.raises(ValueError) as e:
            tm.PacejkaLateral({x: v for x, v in C.items() if x != k})
        assert str(e.value) == f"Missing required lateral coefficients: ['{k}']"
    assert tm._DEFAULT_SCALING == ones


def test_the_peak_searches():
    t = tm.default_tire()
    for fz, g in ((1100.0, 0.0), (500.0, 0.04)):
        a = np.radians(np.linspace(-15, 15, 121))
        assert t.mu_peak(fz, g) == pytest.approx(np.abs(t.fy(a, fz, g)).max() / fz)
        assert t.peak_force(fz, g) == pytest.approx(t.mu_peak(fz, g) * fz)
        a = np.radians(np.linspace(0, 30, 301))
        assert t.alpha_peak(fz, g) == pytest.approx(np.degrees(a[np.argmax(np.abs(t.fy(a, fz, g)))]))
    assert t.peak_force(-5.0) == 0.0
    slick = tm.PacejkaLateral({**tm._DEFAULT_FSAE_COEFFS, "PKY1": -4.0}, FNOMIN=1100.0)     # peaks past 15 deg
    a15 = np.radians(np.linspace(-15, 15, 121))
    assert int(np.argmax(np.abs(slick.fy(a15, 1100.0)))) in (0, 120)
    a30 = np.radians(np.linspace(-30, 30, 241))
    assert slick.mu_peak(1100.0) == pytest.approx(np.abs(slick.fy(a30, 1100.0)).max() / 1100.0)
    assert slick.mu_peak(1100.0) > np.abs(slick.fy(a15, 1100.0)).max() / 1100.0
    cam = np.linspace(-6.0, 0.5, 40)
    mus = [t.mu_peak(1100.0, math.radians(abs(c))) for c in cam]
    assert t.optimal_camber(1100.0) == pytest.approx((cam[int(np.argmax(mus))], max(mus)))
    cam = np.linspace(-4.0, 0.0, 9)
    mus = [t.mu_peak(900.0, math.radians(abs(c))) for c in cam]
    assert t.optimal_camber(900.0, -4.0, 0.0, 9) == pytest.approx((cam[int(np.argmax(mus))], max(mus)))


def test_the_coefficient_file_round_trip(tmp_path):
    p = tmp_path / "tire.json"
    tm.coeffs_to_json(C, 1250.0, str(p))
    assert json.loads(p.read_text()) == {"coeffs": C, "FNOMIN": 1250.0}
    t = tm.load_from_json(str(p))
    assert t.coeffs == C and t.FNOMIN == 1250.0 and t.scaling == {k: 1.0 for k in S}
    p.write_text(json.dumps({"coeffs": C, "scaling": {"LMUY": 0.9}}))
    t = tm.load_from_json(str(p))
    assert t.FNOMIN == 6306.0 and t.scaling["LMUY"] == 0.9


def test_the_default_tire_and_its_description():
    t = tm.default_tire()
    assert t.coeffs == tm._DEFAULT_FSAE_COEFFS and t.coeffs is not tm._DEFAULT_FSAE_COEFFS and t.FNOMIN == 1100.0
    assert tm._DEFAULT_FSAE_COEFFS["PKY1"] == -28.0 and tm._DEFAULT_FSAE_FNOMIN == 1100.0
    d = tm.describe(t)
    assert d == {"FNOMIN_N": 1100.0, "mu_at_nominal": round(t.mu_peak(1100.0), 3),
                 "mu_light_load": round(t.mu_peak(440.0), 3), "mu_heavy_load": round(t.mu_peak(1980.0), 3),
                 "alpha_peak_deg": float(f"{t.alpha_peak(1100.0):.1f}"),
                 "optimal_camber_deg": round(t.optimal_camber(1100.0)[0], 2)}
    assert d["mu_light_load"] > d["mu_at_nominal"] > d["mu_heavy_load"]


def test_the_friction_ellipse():
    lat = tm.default_tire()
    c = tm.CombinedSlipTire(lat, mu_x_ratio=1.1, ell_kx=2.5, ell_ky=1.8)
    fz = 1000.0
    fy_max, fx_max = lat.peak_force(fz), 1.1 * lat.mu_peak(fz) * fz
    assert c.mu_x_peak(fz) == pytest.approx(1.1 * lat.mu_peak(fz))
    for fx in (0.0, 300.0, -900.0, 5e4):
        u = min(abs(fx) / fx_max, 1.0)
        assert c.available_fy(fx, fz) == pytest.approx(fy_max * max(1 - u ** 2.5, 0) ** (1 / 1.8))
    for fy in (0.0, 400.0, -1200.0, 5e4):
        u = min(abs(fy) / fy_max, 1.0)
        assert c.available_fx(fy, fz) == pytest.approx(fx_max * max(1 - u ** 1.8, 0) ** (1 / 2.5))
    assert c.available_fy(10.0, 0.0) == 0.0                     # no load: no longitudinal capacity, no lateral
    class Dead:
        def mu_peak(self, fz, g=0.0):
            return 0.0

        def peak_force(self, fz, g=0.0):
            return 0.0
    dead = tm.CombinedSlipTire(Dead())
    assert dead.available_fy(1.0, 1000.0) == 0.0 and dead.available_fx(1.0, 1000.0) == 0.0
    d = tm.CombinedSlipTire(lat)
    assert (d.mu_x_ratio, d.ell_kx, d.ell_ky, d.is_calibrated) == (1.05, 2.0, 2.0, False)
    fx, fy = c.friction_circle(fz, n=9)
    th = np.linspace(0, 2 * np.pi, 9)
    assert fx == pytest.approx(fx_max * np.sign(np.cos(th)) * np.abs(np.cos(th)) ** (2 / 2.5))
    assert fy == pytest.approx(fy_max * np.sign(np.sin(th)) * np.abs(np.sin(th)) ** (2 / 1.8))
    assert len(d.friction_circle(fz)[0]) == 72
    assert d.status().startswith("UNCALIBRATED — symmetric friction ellipse")
    assert tm.CombinedSlipTire(lat, is_calibrated=True).status() == "calibrated to drive/brake data"
    assert tm.default_combined_tire().lateral.coeffs == tm._DEFAULT_FSAE_COEFFS
    assert tm.default_combined_tire(lat).lateral is lat


def test_relaxation():
    assert tm.relaxation_length(1100.0) == pytest.approx(0.45)
    assert tm.relaxation_length(2200.0) == pytest.approx(0.45 * 2 ** 0.4)
    assert tm.relaxation_length(500.0, FNOMIN=1000.0, sigma_nominal_m=0.5) == pytest.approx(0.5 * 0.5 ** 0.4)
    assert tm.relaxation_length(0.0) == pytest.approx(0.45 * (1e-6 / 1100.0) ** 0.4)
    assert tm.apply_relaxation_lag(1.0, 0.45, 0.45) == pytest.approx(1 - math.exp(-1))
    assert tm.apply_relaxation_lag(1.0, 0.2, 0.45, alpha_prev=0.5) == pytest.approx(1 + (0.5 - 1) * math.exp(-0.2 / 0.45))
    assert tm.apply_relaxation_lag(1.0, -1.0, 0.45, alpha_prev=0.3) == pytest.approx(0.3)
    assert tm.apply_relaxation_lag(1.0, 0.1, 0.0, alpha_prev=0.3) == pytest.approx(1.0)


def test_describe_on_a_tire_whose_peaks_do_not_round_alike():
    t = tm.PacejkaLateral(C, FNOMIN=1100.0, scaling={**S, "LMUY": S["LMUY"] * 1.01})
    d = tm.describe(t)
    assert d == {"FNOMIN_N": 1100.0, "mu_at_nominal": round(t.mu_peak(1100.0), 3),
                 "mu_light_load": round(t.mu_peak(440.0), 3), "mu_heavy_load": round(t.mu_peak(1980.0), 3),
                 "alpha_peak_deg": float(f"{t.alpha_peak(1100.0):.1f}"),
                 "optimal_camber_deg": round(t.optimal_camber(1100.0)[0], 2)}
    raw = (t.mu_peak(1100.0), t.mu_peak(440.0), t.mu_peak(1980.0))
    assert all(round(x, 3) != round(x, 4) for x in raw[:1]) and any(round(x, 3) != round(x, 4) for x in raw)


def test_the_file_layout_and_a_tiny_nominal_load(tmp_path):
    p = tmp_path / "c.json"
    tm.coeffs_to_json({"PCY1": 1.3}, 900.0, str(p))
    assert p.read_text() == json.dumps({"coeffs": {"PCY1": 1.3}, "FNOMIN": 900.0}, indent=2)
    assert tm.relaxation_length(400.0, FNOMIN=0.5) == pytest.approx(0.45 * 800.0 ** 0.4)
    assert tm.default_combined_tire().is_calibrated is False


def test_the_shape_factor_is_capped_and_a_zero_shifted_slip():
    big = {**C, "PEY1": 1.6, "PEY2": 0.0, "PEY3": 0.0, "PEY4": 0.0}
    t = tm.PacejkaLateral(big, FNOMIN=1100.0, scaling=S)
    for a in (-0.1, 0.05, 0.2):
        assert t.fy(a, 1100.0) == pytest.approx(_mf52(a, 1100.0, 0.0, big), rel=1e-12)
    u = tm.PacejkaLateral(C, FNOMIN=1100.0, scaling=S)
    shy = C["PHY1"] * S["LHY"]
    assert u.fy(-shy, 1100.0) == pytest.approx(_mf52(-shy, 1100.0, 0.0), abs=1e-9)    # shifted slip exactly zero


def test_the_wide_sweep_from_either_edge():
    a30 = np.radians(np.linspace(-30, 30, 241))
    a15 = np.radians(np.linspace(-15, 15, 121))
    for pv, edge in ((0.05, 0), (-0.05, 120)):
        s = tm.PacejkaLateral({**tm._DEFAULT_FSAE_COEFFS, "PKY1": -4.0, "PVY1": pv}, FNOMIN=1100.0)
        assert int(np.argmax(np.abs(s.fy(a15, 1100.0)))) == edge
        assert s.mu_peak(1100.0) == float(np.max(np.abs(s.fy(a30, 1100.0))) / 1100.0)


def test_the_ellipse_with_camber_and_light_loads():
    lat = tm.default_tire()
    c = tm.CombinedSlipTire(lat, mu_x_ratio=1.1, ell_kx=2.5, ell_ky=1.8)
    g, fz = 0.05, 900.0
    fy_max, fx_max = lat.peak_force(fz, g), 1.1 * lat.mu_peak(fz, g) * fz
    assert c.mu_x_peak(fz, g) == pytest.approx(1.1 * lat.mu_peak(fz, g))
    assert c.available_fy(300.0, fz, g) == pytest.approx(fy_max * (1 - (300 / fx_max) ** 2.5) ** (1 / 1.8))
    assert c.available_fx(400.0, fz, g) == pytest.approx(fx_max * (1 - (400 / fy_max) ** 1.8) ** (1 / 2.5))
    fx, fy = c.friction_circle(fz, gamma=g, n=5)
    th = np.linspace(0, 2 * np.pi, 5)
    assert fx == pytest.approx(fx_max * np.sign(np.cos(th)) * np.abs(np.cos(th)) ** 0.8)
    assert fy == pytest.approx(fy_max * np.sign(np.sin(th)) * np.abs(np.sin(th)) ** (2 / 1.8))
    light = 0.5
    assert c.available_fy(0.0, light) == pytest.approx(lat.peak_force(light))
    assert c.available_fx(0.0, light) == pytest.approx(1.1 * lat.mu_peak(light) * light)
    assert c.friction_circle(light, n=3)[0][0] == pytest.approx(1.1 * lat.mu_peak(light) * light)

    class Weak:
        def mu_peak(self, fz, g=0.0):
            return 0.5 / max(fz, 1e-6)

        def peak_force(self, fz, g=0.0):
            return 0.5
    w = tm.CombinedSlipTire(Weak(), mu_x_ratio=1.0)
    assert w.available_fx(0.25, 1000.0) == pytest.approx(0.5 * (1 - 0.5 ** 2) ** 0.5)
    assert w.available_fy(0.25, 1000.0) == pytest.approx(0.5 * (1 - 0.5 ** 2) ** 0.5)


def test_the_status_wording():
    assert tm.CombinedSlipTire(tm.default_tire()).status().endswith(
        "to be quantitative; coupling shape is still physically valid)")


def test_the_wide_sweep_finds_a_peak_inside_it_with_camber():
    a30 = np.radians(np.linspace(-30, 30, 241))
    s = tm.PacejkaLateral({**tm._DEFAULT_FSAE_COEFFS, "PKY1": -10.0, "PVY1": 0.05}, FNOMIN=1100.0)
    g = 0.03
    f30 = np.abs(s.fy(a30, 1100.0, g))
    assert 15.0 < abs(np.degrees(a30[np.argmax(f30)])) < 29.0
    assert s.mu_peak(1100.0, g) == float(np.max(f30) / 1100.0)
    assert s.mu_peak(0.5, g) == float(np.max(np.abs(s.fy(a30, 0.5, g))) / 0.5) or s.mu_peak(0.5, g) == float(
        np.max(np.abs(s.fy(np.radians(np.linspace(-15, 15, 121)), 0.5, g))) / 0.5)
