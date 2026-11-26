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


# ---- coefficient and nominal-load guards: exact boundary and exact words ----
_REQ = {"PCY1": 1.3, "PDY1": 1.0, "PDY2": -0.1, "PEY1": -0.5, "PKY1": -20.0, "PKY2": 1.5}


def _says(exc_info, text):
    assert str(exc_info.value) == text


def test_non_finite_coefficients_are_named_in_order():
    with pytest.raises(ValueError) as e:
        tm.PacejkaLateral({**_REQ, "PKY1": float("nan"), "PCY1": float("inf"), "PDY2": 7})
    _says(e, "Tire coefficients must be finite numbers; not finite: ['PCY1', 'PKY1']")
    tm.PacejkaLateral({**_REQ, "PDY2": 0, "FLAG": True, "NOTE": "text"})      # ints, bools and text pass


def test_nominal_load_must_be_a_finite_number_above_zero():
    for bad in (0.0, -1.0, float("nan"), float("inf"), "900"):
        with pytest.raises(ValueError) as e:
            tm.PacejkaLateral(dict(_REQ), FNOMIN=bad)
        _says(e, f"FNOMIN (nominal load, N) must be a finite number above zero, got {bad!r}")
    assert tm.PacejkaLateral(dict(_REQ), FNOMIN=1e-6).FNOMIN == 1e-6
    assert tm.PacejkaLateral(dict(_REQ), FNOMIN=900).FNOMIN == 900


_TIR = """[MODEL]
FITTYP                   = 61                $Magic Formula version number
[VERTICAL]
FNOMIN                   = 1100.5            $Nominal wheel load
 pcy1 =   1.53   $ lower-case names and padding are read
PDY1                     = -2.4e0
PDY2 = +0.31
PEY1=-0.75
PKY1                     = -5.2E+1
PKY2 = 1.9
PKY5 = 0.004
NOMPRES                  = 82737
LMUY                     = 0.66
LKY = 1.10
$ a comment = 3 is not read
NOT_A_NUMBER = 'abc'
"""


def test_a_tir_file_is_read_with_its_version_defaults_and_scaling(tmp_path):
    p = tmp_path / "front.tir"
    p.write_text(_TIR)
    t = tm.load_tir(str(p))
    assert (t.FNOMIN, t.mf_version, t.pressure_pa) == (1100.5, "6.1", None)
    co = t.coeffs
    assert (co["PCY1"], co["PDY1"], co["PDY2"], co["PEY1"], co["PKY1"], co["PKY2"], co["PKY5"]) == \
        (1.53, -2.4, 0.31, -0.75, -52.0, 1.9, 0.004)
    assert co["NOMPRES"] == 82737.0 and co["FITTYP"] == 61.0
    assert co["PKY4"] == 2.0 and co["PCY1"] == 1.53 and co["PHY3"] == 0.0 and co["PPY5"] == 0.0
    assert set(co) == set(tm._LAT_KEYS) | set(tm._SCALE_KEYS) | {"FITTYP"}
    assert (co["LMUY"], co["LKY"], co["LFZO"], co["LKYC"]) == (0.66, 1.10, 1.0, 1.0)
    assert t.scaling["LMUY"] == 0.66 and t.scaling["LKY"] == 1.10 and t.scaling["LCY"] == 1.0
    assert tm.load_tir(str(p), pressure_pa=90000.0).pressure_pa == 90000.0
    assert tm._LAT_KEYS[-1] == "NOMPRES" and len(tm._LAT_KEYS) == 29
    assert tm._SCALE_KEYS == ("LFZO", "LCY", "LMUY", "LEY", "LKY", "LHY", "LVY", "LGAY", "LKYC")


def test_tir_versions_and_refusals(tmp_path):
    body = _TIR.replace("FITTYP                   = 61", "FITTYP = {}")
    for fit, ver in ((6, "5.2"), (21, "5.2"), (61, "6.1"), (62, "6.1")):
        p = tmp_path / f"v{fit}.tir"
        p.write_text(body.format(fit))
        t = tm.load_tir(str(p))
        assert t.mf_version == ver and t.coeffs["FITTYP"] == float(fit)
    nofit = tmp_path / "nofit.tir"
    nofit.write_text(body.replace("FITTYP = {}\n", ""))
    assert tm.load_tir(str(nofit)).mf_version == "5.2" and tm.load_tir(str(nofit)).coeffs["FITTYP"] == 6.0
    bad = tmp_path / "bad.tir"
    bad.write_text(body.format(52))
    with pytest.raises(ValueError) as e:
        tm.load_tir(str(bad))
    assert str(e.value) == f"{bad}: FITTYP 52 is not MF5.2 (6, 21) or MF6.1/6.2 (61, 62)"
    none = tmp_path / "none.tir"
    none.write_text(_TIR.replace("FNOMIN", "XNOMIN"))
    with pytest.raises(ValueError) as e:
        tm.load_tir(str(none))
    assert str(e.value) == f"{none}: no FNOMIN, not an MF-Tyre .tir file"
    junk = tmp_path / "junk.tir"
    junk.write_bytes(_TIR.encode() + b"\xff\xfe binary tail\n")
    assert tm.load_tir(str(junk)).FNOMIN == 1100.5


def test_every_mfeval_reference_point_to_a_millionth_of_a_newton():
    """MF5.2 and MF6.1/6.2 lateral force against every point MFeval 4.3.1 wrote for its own .tir files, graded or not
    (benchmarks/tyre): the formulas agree to rounding, so any change to them shows."""
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
    import tyre_benchmark as tb
    r = tb.run()
    assert {k for k in r if k != "passed"} >= {"MagicFormula52_Parameters", "MagicFormula61_Parameters",
                                               "MagicFormula62_Parameters", "FSAE_Defaults"}
    for name, v in r.items():
        if name != "passed":
            assert v["groups"] and max(g["worst_dFy_N"] for g in v["groups"]) < 1e-5, name


def _fy61_by_hand(co, sc, fnomin, p, alpha, fz, g):
    """Pacejka (2012) eqs. 4.E19-E30, pure lateral, no turn slip, written out term by term."""
    def c(k, d=0.0):
        return co.get(k, d)

    def lam(k):
        return sc.get(k, 1.0)

    fz0 = fnomin * lam("LFZO")
    dfz = (fz - fz0) / fz0
    dpi = (p - c("NOMPRES")) / c("NOMPRES") if p is not None else 0.0
    lmu_p = 10 * lam("LMUY") / (1 + 9 * lam("LMUY"))
    kya = (c("PKY1") * fz0 * (1 + c("PPY1") * dpi) * (1 - c("PKY3") * abs(g))
           * math.sin(c("PKY4", 2.0) * math.atan(fz / ((c("PKY2") + c("PKY5") * g * g) * (1 + c("PPY2") * dpi) * fz0)))
           * lam("LKY"))
    svyg = fz * (c("PVY3") + c("PVY4") * dfz) * g * lam("LKYC") * lmu_p
    kyg0 = fz * (c("PKY6") + c("PKY7") * dfz) * (1 + c("PPY5") * dpi) * lam("LKYC")
    shy = (c("PHY1") + c("PHY2") * dfz) * lam("LHY") + (kyg0 * g - svyg) / (kya + 1e-6 * (1 if kya >= 0 else -1))
    svy = fz * (c("PVY1") + c("PVY2") * dfz) * lam("LVY") * lmu_p + svyg
    ay = alpha + shy
    dy = ((c("PDY1") + c("PDY2") * dfz) * (1 + c("PPY3") * dpi + c("PPY4") * dpi * dpi) * (1 - c("PDY3") * g * g)
          * lam("LMUY")) * fz
    cy = c("PCY1") * lam("LCY")
    sgn = 1.0 if ay > 0 else (-1.0 if ay < 0 else 0.0)
    ey = min((c("PEY1") + c("PEY2") * dfz) * (1 + c("PEY5") * g * g - (c("PEY3") + c("PEY4") * g) * sgn) * lam("LEY"),
             1.0)
    by = kya / (cy * dy + 1e-6 * (1 if cy * dy >= 0 else -1))
    return dy * math.sin(cy * math.atan(by * ay - ey * (by * ay - math.atan(by * ay)))) + svy


def test_mf61_every_term_including_pressure_and_scaling():
    co = dict(FITTYP=61, PCY1=1.45, PDY1=-2.6, PDY2=0.31, PDY3=4.1, PEY1=-0.4, PEY2=0.21, PEY3=0.12, PEY4=-0.6,
              PEY5=1.3, PKY1=-48.0, PKY2=1.7, PKY3=0.55, PKY4=2.1, PKY5=0.3, PKY6=-0.9, PKY7=0.25, PHY1=0.002,
              PHY2=-0.001, PVY1=0.03, PVY2=-0.02, PVY3=-0.4, PVY4=0.25, PPY1=-0.5, PPY2=0.9, PPY3=-0.12,
              PPY4=0.05, PPY5=-0.3, NOMPRES=83000.0)
    sc = dict(LFZO=1.08, LCY=0.97, LMUY=0.71, LEY=1.05, LKY=0.92, LHY=1.1, LVY=0.9, LGAY=1.0, LKYC=1.15)
    for p in (None, 70000.0, 97000.0):
        t = tm.PacejkaLateral(coeffs=dict(co), FNOMIN=1100.0, scaling=dict(sc), pressure_pa=p)
        assert t.mf_version == "6.1"
        for alpha, fz, g in ((0.08, 900.0, -0.03), (-0.12, 1500.0, 0.05), (0.02, 600.0, 0.0), (0.2, 2100.0, -0.07),
                             (-0.01, 1100.0, 0.02)):
            assert float(t.fy(alpha, fz, g)) == pytest.approx(_fy61_by_hand(co, sc, 1100.0, p, alpha, fz, g),
                                                             rel=1e-9, abs=1e-9)
    # a pressure with no NOMPRES to compare it with changes nothing
    t0 = tm.PacejkaLateral(coeffs={**co, "NOMPRES": 0.0}, FNOMIN=1100.0, pressure_pa=90000.0)
    assert float(t0.fy(0.08, 900.0, -0.03)) == pytest.approx(
        _fy61_by_hand({**co, "NOMPRES": 1.0}, {}, 1100.0, None, 0.08, 900.0, -0.03), rel=1e-12)
    # the load floor: a zero or negative load is read as a micro-newton, not a division by zero
    t1 = tm.PacejkaLateral(coeffs=dict(co), FNOMIN=1100.0)
    assert np.isfinite(t1.fy(0.1, 0.0)) and float(t1.fy(0.1, -50.0)) == float(t1.fy(0.1, 1e-6))
    # vectorised over loads
    fz = np.array([700.0, 1400.0])
    assert np.allclose(t1.fy(0.07, fz, 0.01), [_fy61_by_hand(co, {}, 1100.0, None, 0.07, f, 0.01) for f in fz],
                       rtol=1e-12)


def test_mf61_defaults_for_missing_terms_a_tiny_load_and_a_capped_curvature():
    co = dict(FITTYP=61, PCY1=1.45, PDY1=-2.6, PDY2=0.31, PDY3=4.1, PEY1=-0.4, PEY2=0.21, PEY3=0.12, PEY4=-0.6,
              PKY1=-48.0, PKY2=1.7, PKY3=0.55, PKY5=0.3, PKY6=-0.9, PVY3=-0.4, PPY1=-0.5, PPY2=0.9, PPY3=-0.12)
    t = tm.PacejkaLateral(coeffs=dict(co), FNOMIN=1100.0, pressure_pa=90000.0)     # no PKY4, no NOMPRES
    for alpha, fz, g in ((0.08, 900.0, -0.03), (-0.12, 1500.0, 0.05)):
        assert float(t.fy(alpha, fz, g)) == pytest.approx(
            _fy61_by_hand({**co, "PKY4": 2.0}, {}, 1100.0, None, alpha, fz, g), rel=1e-12)
    # the camber is zero unless given, in the force and in the peak search alike
    assert float(t._fy61(0.08, 900.0)) == float(t._fy61(0.08, 900.0, 0.0))
    d = tm.default_tire()
    assert d.alpha_peak(1100.0) == d.alpha_peak(1100.0, 0.0) != d.alpha_peak(1100.0, 1.0)
    # a load under a newton is used as given (only zero and below are floored)
    assert float(t.fy(0.1, 0.5, 0.02)) == pytest.approx(_fy61_by_hand({**co, "PKY4": 2.0}, {}, 1100.0, None,
                                                                      0.1, 0.5, 0.02), rel=1e-9)
    # any NOMPRES above zero is a reference pressure, however small
    tp = tm.PacejkaLateral(coeffs={**co, "PKY4": 2.0, "NOMPRES": 0.5}, FNOMIN=1100.0, pressure_pa=1.0)
    assert float(tp.fy(0.08, 900.0, -0.03)) == pytest.approx(
        _fy61_by_hand({**co, "PKY4": 2.0, "NOMPRES": 0.5}, {}, 1100.0, 1.0, 0.08, 900.0, -0.03), rel=1e-12)
    # a curvature factor above one is capped at one
    tc = tm.PacejkaLateral(coeffs={**co, "PKY4": 2.0, "PEY1": 1.6, "PEY3": 0.0, "PEY4": 0.0}, FNOMIN=1100.0)
    assert float(tc.fy(0.15, 1100.0, 0.0)) == pytest.approx(
        _fy61_by_hand({**co, "PKY4": 2.0, "PEY1": 1.0, "PEY2": 0.0, "PEY3": 0.0, "PEY4": 0.0}, {}, 1100.0, None,
                      0.15, 1100.0, 0.0), rel=1e-12)
    # a coefficient set with no FITTYP stays MF5.2
    assert tm.PacejkaLateral(coeffs={k: v for k, v in co.items() if k != "FITTYP"}, FNOMIN=1100.0).mf_version == "5.2"
