"""suspension/brake_thermal.py against closed forms and independent re-derivations: the rotating-disc and cross-flow
Nusselt correlations each on their own length scale, the annulus mass and area, the single-stop energy split, the
equilibrium as the root of the convection + radiation balance, the two-node and through-thickness models against
steppers written here (the 1-D one vectorised over the nodes), the fade-test and lap power series built by hand, the
duct-gain inversion, the wear proxy, the optimiser's lightest-feasible pick and Pareto front, and the lap energy."""
import math

import numpy as np
import pytest

from suspension import brake_thermal as bt

SIG = 5.670374419e-8


def test_the_material_library():
    want = {"grey_cast_iron": ("Grey cast iron", 7150.0, 460.0, 52.0, 700.0, 0.55),
            "steel": ("Stainless / alloy steel", 7800.0, 480.0, 25.0, 750.0, 0.35),
            "carbon_carbon": ("Carbon-carbon", 1800.0, 1400.0, 40.0, 1200.0, 0.80),
            "aluminum_mmc": ("Aluminium MMC (SiC)", 2800.0, 800.0, 150.0, 500.0, 0.30)}
    for k, v in want.items():
        m = bt.ROTOR_MATERIALS[k]
        assert (m.name, m.rho_kg_m3, m.cp_j_kgK, m.k_w_mK, m.T_max_c, m.emissivity) == v and m.note
    assert list(bt.ROTOR_MATERIALS) == list(want)
    assert bt.material("steel") is bt.ROTOR_MATERIALS["steel"]
    assert bt.material("unobtainium") is bt.ROTOR_MATERIALS["grey_cast_iron"]
    assert bt._MECH == {"grey_cast_iron": dict(E_gpa=110.0, alpha=1.1e-5, nu=0.26, sigma_lim_mpa=250.0),
                        "steel": dict(E_gpa=200.0, alpha=1.2e-5, nu=0.29, sigma_lim_mpa=500.0),
                        "carbon_carbon": dict(E_gpa=40.0, alpha=2.0e-6, nu=0.15, sigma_lim_mpa=120.0),
                        "aluminum_mmc": dict(E_gpa=120.0, alpha=1.6e-5, nu=0.30, sigma_lim_mpa=300.0)}


def _h(d_mm, v, duct=1.0, R_mm=228.0):
    k, nu, pr = 0.026, 1.6e-5, 0.71
    D = max(d_mm, 1.0) / 1000
    r = D / 2
    w = abs(v) / max(R_mm / 1000, 1e-3)
    h_rot = 0.015 * max(w * r * r / nu, 1.0) ** 0.8 * k / max(r, 1e-3)
    h_x = 0.037 * max(abs(v) * D / nu, 1.0) ** 0.8 * pr ** (1 / 3) * k / max(D, 1e-3)
    return max(math.hypot(h_rot, h_x) * max(duct, 0.1), 12.0)


def test_the_rotating_disc_convection():
    assert (bt._AIR_K, bt._AIR_NU, bt._AIR_PR) == (0.026, 1.6e-5, 0.71)
    for d, v, duct, R in ((220.0, 20.0, 1.0, 228.0), (300.0, -12.0, 2.5, 250.0), (180.0, 30.0, 1.0, 0.5)):
        assert bt.h_rotating_disc(d, v, duct_gain=duct, wheel_radius_mm=R) == pytest.approx(_h(d, v, duct, R),
                                                                                           rel=1e-12)
    assert bt.h_rotating_disc(220.0, 20.0) == pytest.approx(_h(220.0, 20.0))
    assert bt.h_rotating_disc(220.0, 0.0) == 12.0                                     # still air: the floor
    assert bt.h_rotating_disc(220.0, 20.0, duct_gain=0.0) == pytest.approx(max(_h(220.0, 20.0) * 0.1, 12.0))
    assert bt.h_rotating_disc(0.5, 300.0) == pytest.approx(_h(0.5, 300.0))            # a 1 mm floor on the diameter
    assert bt.h_rotating_disc(1.5, 300.0) == pytest.approx(_h(1.5, 300.0))
    assert bt.h_rotating_disc(220.0, 1e-4) == 12.0
    rot_only = 0.015 * (20 / 0.228 * 0.11 ** 2 / 1.6e-5) ** 0.8 * 0.026 / 0.11            # radius-based
    x_only = 0.037 * (20 * 0.22 / 1.6e-5) ** 0.8 * 0.71 ** (1 / 3) * 0.026 / 0.22       # diameter-based
    assert bt.h_rotating_disc(220.0, 20.0) == pytest.approx(math.hypot(rot_only, x_only))


def test_the_parameters_and_the_fade_law():
    p = bt.BrakeThermalParams()
    assert (p.rho_kg_m3, p.cp_rotor, p.heat_to_rotor, p.h_air0, p.h_air1, p.area_factor, p.emissivity,
            p.enable_radiation, p.enable_fade, p.T_fade_onset_c, p.fade_per_C, p.fade_floor, p.T_ambient_c,
            p.ambient_is_local, p.calibrated, p.fitted_to, p._SIGMA) == (
        7150.0, 460.0, 0.90, 30.0, 4.0, 2.4, 0.55, True, False, 450.0, 0.0012, 0.45, 30.0, False, False, "", SIG)
    assert bt.default_brake_thermal_params() == p
    assert p.h_air(15.0) == pytest.approx(90.0) and p.h_air(-15.0) == pytest.approx(90.0)
    assert p.mu_fade(900.0) == 1.0
    f = bt.BrakeThermalParams(enable_fade=True)
    assert f.mu_fade(300.0) == 1.0 and f.mu_fade(450.0) == 1.0
    assert f.mu_fade(600.0) == pytest.approx(1 - 0.0012 * 150) and f.mu_fade(1000.0) == 0.45
    assert bt.BrakeThermalParams(enable_fade=True, fade_floor=0.1).mu_fade(1000.0) == pytest.approx(1 - 0.0012 * 550)


def test_the_convection_both_ways():
    cal = bt.BrakeThermalParams(calibrated=True, h_air0=25.0, h_air1=5.0)
    assert bt.convection_h(cal, 220.0, 10.0) == pytest.approx(75.0)
    assert bt.convection_h(cal, 220.0, 10.0, duct_gain=2.0) == pytest.approx(150.0)
    assert bt.convection_h(cal, 220.0, 10.0, duct_gain=0.01) == pytest.approx(7.5)
    unc = bt.BrakeThermalParams()
    assert bt.convection_h(unc, 250.0, 18.0, 1.7) == pytest.approx(_h(250.0, 18.0, 1.7))
    assert bt.convection_h(unc, 250.0, 18.0) == pytest.approx(_h(250.0, 18.0))


def _ring(d, hat=40.0):
    ro = max(d, 1.0) / 2000
    ri = max(ro - hat / 1000, 0.02)
    return math.pi * (ro * ro - ri * ri)


def test_the_rotor_mass_and_area():
    assert bt.rotor_mass_kg(220.0, 5.0) == pytest.approx(_ring(220.0) * 0.005 * 7150.0)
    assert bt.rotor_mass_kg(260.0, 6.0, vented_fraction=0.3, hat_clearance_mm=55.0, rho_kg_m3=2800.0) == pytest.approx(
        _ring(260.0, 55.0) * 0.006 * 0.7 * 2800.0)
    assert bt.rotor_mass_kg(220.0, 5.0, vented_fraction=0.95) == pytest.approx(_ring(220.0) * 0.005 * 0.2 * 7150.0)
    assert bt.rotor_mass_kg(220.0, 5.0, vented_fraction=-0.5) == pytest.approx(bt.rotor_mass_kg(220.0, 5.0))
    assert bt.rotor_mass_kg(220.0, 5.0, hat_clearance_mm=200.0) == pytest.approx(_ring(220.0, 200.0) * 0.005 * 7150)
    assert bt.rotor_mass_kg(0.5, 5.0) == pytest.approx(math.pi * (0.0005 ** 2 - 0.02 ** 2) * 0.005 * 7150)
    assert bt.convective_area_m2(220.0) == pytest.approx(_ring(220.0) * 2.4)
    assert bt.convective_area_m2(240.0, hat_clearance_mm=50.0, area_factor=3.1) == pytest.approx(_ring(240.0, 50.0)
                                                                                                * 3.1)
    assert bt.convective_area_m2(220.0, area_factor=0.0) == pytest.approx(_ring(220.0) * 0.1)
    assert bt.convective_area_m2(220.0, hat_clearance_mm=300.0) == pytest.approx(_ring(220.0, 300.0) * 2.4)
    assert bt.convective_area_m2(0.2) == pytest.approx(_ring(0.2) * 2.4)


def test_a_single_hard_stop():
    m = bt.BrakeThermalModel(bt.BrakeThermalParams(heat_to_rotor=0.85, cp_rotor=500.0, rho_kg_m3=7000.0,
                                                   T_ambient_c=25.0))
    r = m.single_stop(mass_kg=300.0, v0_ms=25.0, front_bias=0.65, diameter_mm=230.0, thickness_mm=6.0,
                      vented_fraction=0.2, rotating_mass_factor=1.08)
    q = 0.5 * 300 * 625 * 1.08
    mass = _ring(230.0) * 0.006 * 0.8 * 7000.0
    assert (r.q_total_j, r.q_front_j, r.q_per_rotor_j, r.rotor_mass_kg) == pytest.approx(
        (q, q * 0.65 * 0.85, q * 0.65 * 0.85 / 2, mass))
    assert r.delta_T_c == pytest.approx(q * 0.65 * 0.85 / 2 / (mass * 500)) and r.T_peak_c == pytest.approx(
        25.0 + r.delta_T_c) and r.synthesized is True
    assert m.single_stop(mass_kg=300.0, v0_ms=25.0, front_bias=0.65, diameter_mm=230.0, thickness_mm=6.0,
                         T_start_c=200.0, rotating_mass_factor=0.9).q_total_j == pytest.approx(0.5 * 300 * 625)
    d = bt.BrakeThermalModel().single_stop(mass_kg=300.0, v0_ms=25.0, front_bias=0.65, diameter_mm=230.0,
                                           thickness_mm=6.0, T_start_c=200.0)
    assert d.q_total_j == pytest.approx(0.5 * 300 * 625 * 1.05) and d.T_peak_c == pytest.approx(200 + d.delta_T_c)
    assert bt.BrakeThermalModel(bt.BrakeThermalParams(calibrated=True)).single_stop(
        mass_kg=1.0, v0_ms=1.0, front_bias=0.5, diameter_mm=220.0, thickness_mm=5.0).synthesized is False
    tiny = bt.BrakeThermalModel().single_stop(mass_kg=1.0, v0_ms=1.0, front_bias=0.5, diameter_mm=10.0,
                                             thickness_mm=5.0)
    assert tiny.rotor_mass_kg < 0 and tiny.delta_T_c == pytest.approx(tiny.q_per_rotor_j / 1e-6)


def _balance(p, P, h, A, T):
    Ta = p.T_ambient_c
    rad = p.emissivity * SIG * A * ((T + 273.15) ** 4 - (Ta + 273.15) ** 4) if p.enable_radiation else 0.0
    return h * A * (T - Ta) + rad - P


def test_the_equilibrium_is_the_root_of_the_heat_balance():
    for p in (bt.BrakeThermalParams(), bt.BrakeThermalParams(calibrated=True, area_factor=3.0, T_ambient_c=20.0),
              bt.BrakeThermalParams(emissivity=0.9)):
        m = bt.BrakeThermalModel(p)
        for P in (150.0, 2500.0, 2e5):
            r = m.equilibrium(p_brake_avg_w=P, v_work_ms=16.0, diameter_mm=230.0, hat_clearance_mm=45.0,
                              duct_gain=1.4)
            A = _ring(230.0, 45.0) * p.area_factor
            h = bt.convection_h(p, 230.0, 16.0, 1.4)
            assert (r.h_air, r.area_m2, r.p_brake_avg_w) == pytest.approx((h, A, P))
            assert _balance(p, P, h, A, r.T_equilibrium_c) == pytest.approx(0.0, abs=1e-6 * P)
            assert r.faded is False and r.mu_fade_mult == 1.0 and r.synthesized is (not p.calibrated)
    off = bt.BrakeThermalParams(enable_radiation=False)
    r = bt.BrakeThermalModel(off).equilibrium(p_brake_avg_w=900.0, v_work_ms=15.0, diameter_mm=220.0)
    assert r.T_equilibrium_c == pytest.approx(30.0 + 900.0 / (bt.convection_h(off, 220.0, 15.0) * _ring(220.0) * 2.4))
    assert bt.BrakeThermalModel().equilibrium(p_brake_avg_w=0.0, v_work_ms=15.0,
                                              diameter_mm=220.0).T_equilibrium_c == pytest.approx(30.0)
    dull = bt.BrakeThermalParams(emissivity=0.0)
    r = bt.BrakeThermalModel(dull).equilibrium(p_brake_avg_w=900.0, v_work_ms=15.0, diameter_mm=220.0)
    assert r.T_equilibrium_c == pytest.approx(30.0 + 900.0 / (bt.convection_h(dull, 220.0, 15.0) * _ring(220.0) * 2.4))
    fade = bt.BrakeThermalParams(enable_fade=True, T_fade_onset_c=200.0)
    r = bt.BrakeThermalModel(fade).equilibrium(p_brake_avg_w=3000.0, v_work_ms=12.0, diameter_mm=200.0)
    assert r.T_equilibrium_c > 200 and r.faded is True and r.mu_fade_mult == pytest.approx(fade.mu_fade(
        r.T_equilibrium_c))
    cool = bt.BrakeThermalModel(fade).equilibrium(p_brake_avg_w=50.0, v_work_ms=12.0, diameter_mm=200.0)
    assert cool.faded is False
    r = bt.BrakeThermalModel().equilibrium(p_brake_avg_w=900.0, v_work_ms=15.0, diameter_mm=220.0)
    assert r.area_m2 == pytest.approx(_ring(220.0) * 2.4) and r.h_air == pytest.approx(_h(220.0, 15.0))


def test_the_mass_temperature_sweep():
    m = bt.BrakeThermalModel()
    rows = m.mass_temp_sweep(mass_kg=300.0, v0_ms=25.0, front_bias=0.65, thickness_mm=5.0, dia_min_mm=200.0,
                             dia_max_mm=260.0, n=4, vented_fraction=0.1, T_start_c=80.0)
    assert [r["diameter_mm"] for r in rows] == pytest.approx([200.0, 220.0, 240.0, 260.0])
    for r in rows:
        s = m.single_stop(mass_kg=300.0, v0_ms=25.0, front_bias=0.65, diameter_mm=r["diameter_mm"], thickness_mm=5.0,
                          vented_fraction=0.1, T_start_c=80.0)
        assert r == dict(diameter_mm=r["diameter_mm"], rotor_mass_kg=s.rotor_mass_kg, T_peak_c=s.T_peak_c,
                         delta_T_c=s.delta_T_c)
    d = m.mass_temp_sweep(mass_kg=300.0, v0_ms=25.0, front_bias=0.65, thickness_mm=5.0)
    assert len(d) == 25 and d[0]["diameter_mm"] == 180.0 and d[-1]["diameter_mm"] == 300.0
    assert d[0]["T_peak_c"] == pytest.approx(m.single_stop(mass_kg=300.0, v0_ms=25.0, front_bias=0.65,
                                                           diameter_mm=180.0, thickness_mm=5.0).T_peak_c)
    one = m.mass_temp_sweep(mass_kg=300.0, v0_ms=25.0, front_bias=0.65, thickness_mm=5.0, n=1)
    assert [r["diameter_mm"] for r in one] == [180.0, 300.0]


def test_the_provenance():
    assert bt.BrakeThermalModel(bt.BrakeThermalParams(fitted_to="dyno 3")).provenance() == {
        "model": "lumped single-mass rotor thermal (0-D energy balance)", "calibrated": False, "fitted_to": "dyno 3",
        "note": "Equations are exact energy balances; parameters (cp, density, convection h, heat-split) are "
                "representative unless calibrated. Use to triage which rotor geometries deserve a full Ansys run — "
                "not as absolute temperatures."}


def _two_node(p, P, dt, V=None):
    m = bt.material(p.rotor_mat)
    mass = bt.rotor_mass_kg(p.diameter_mm, p.thickness_mm, vented_fraction=p.vented_fraction,
                            hat_clearance_mm=p.hat_clearance_mm, rho_kg_m3=m.rho_kg_m3)
    A = bt.convective_area_m2(p.diameter_mm, hat_clearance_mm=p.hat_clearance_mm, area_factor=p.area_factor)
    Cr, Cp = max(mass * m.cp_j_kgK, 1e-6), max(p.m_pad_kg * p.cp_pad, 1e-6)
    Ta = p.T_ambient_c
    Tr = Tp = Ta
    out = []
    for i, q in enumerate(P):
        q = max(q, 0.0)
        v = V[i] if V is not None and i < len(V) else 0.0
        h = _h(p.diameter_mm, v, p.duct_gain)
        cond = p.k_pad_rotor * (Tr - Tp) if q > 1.0 else 0.0
        rad = p.emissivity * SIG * A * ((Tr + 273.15) ** 4 - (Ta + 273.15) ** 4) if p.enable_radiation else 0.0
        Tr_new = Tr + (q * (1 - p.pad_heat_fraction) - cond - h * A * (Tr - Ta) - rad) / Cr * dt
        Tp_new = Tp + (q * p.pad_heat_fraction + cond - 0.5 * h * p.pad_area_m2 * (Tp - Ta)) / Cp * dt
        Tr, Tp = max(Ta, Tr_new), max(Ta, Tp_new)
        out.append((Tr, Tp))
    return out, mass


def test_the_two_node_model_steps_rotor_and_pad():
    p = bt.TwoNodeParams()
    assert (p.rotor_mat, p.diameter_mm, p.thickness_mm, p.vented_fraction, p.hat_clearance_mm, p.area_factor,
            p.duct_gain, p.m_pad_kg, p.cp_pad, p.pad_area_m2, p.k_pad_rotor, p.pad_heat_fraction, p.T_ambient_c,
            p.ambient_is_local, p.emissivity, p.enable_radiation, p.calibrated, p._SIGMA) == (
        "grey_cast_iron", 220.0, 5.0, 0.0, 40.0, 2.4, 1.0, 0.12, 800.0, 0.0030, 6.0, 0.10, 30.0, False, 0.55, True,
        False, SIG)
    P = [8000.0] * 30 + [0.5] * 10 + [-50.0] * 10 + [0.0] * 20
    V = [20.0 - 0.3 * i for i in range(50)]
    for prm in (p, bt.TwoNodeParams(rotor_mat="steel", diameter_mm=250.0, thickness_mm=6.0, vented_fraction=0.3,
                                    hat_clearance_mm=50.0, area_factor=3.0, duct_gain=2.0, m_pad_kg=0.2, cp_pad=900.0,
                                    pad_area_m2=0.004, k_pad_rotor=9.0, pad_heat_fraction=0.15, T_ambient_c=20.0,
                                    emissivity=0.7, enable_radiation=False, calibrated=True)):
        model = bt.TwoNodeRotorPad(prm)
        tr = model.simulate(P, 0.1, v_series=V)
        want, mass = _two_node(prm, P, 0.1, V)
        assert tr.T_rotor_c == pytest.approx([w[0] for w in want], rel=1e-12)
        assert tr.T_pad_c == pytest.approx([w[1] for w in want], rel=1e-12)
        assert tr.t_s == pytest.approx([0.1 * (i + 1) for i in range(len(P))])
        assert tr.p_in_w == [max(x, 0.0) for x in P] and tr.rotor_mass_kg == pytest.approx(mass)
        assert tr.T_rotor_peak_c == max(tr.T_rotor_c) and tr.T_pad_peak_c == max(tr.T_pad_c)
        assert tr.T_rotor_final_c == tr.T_rotor_c[-1] and tr.material_T_max_c == bt.material(prm.rotor_mat).T_max_c
        assert tr.over_material_limit is (tr.T_rotor_peak_c > tr.material_T_max_c)
        assert tr.synthesized is (not prm.calibrated)
        assert model.A_conv == pytest.approx(bt.convective_area_m2(prm.diameter_mm,
                                                                    hat_clearance_mm=prm.hat_clearance_mm,
                                                                    area_factor=prm.area_factor))
    still = bt.TwoNodeRotorPad().simulate(P[:30], 0.1)
    assert still.T_rotor_c == pytest.approx([w[0] for w in _two_node(p, P[:30], 0.1)[0]], rel=1e-12)
    hot = bt.TwoNodeRotorPad(bt.TwoNodeParams(diameter_mm=150.0, thickness_mm=2.0)).simulate([60000.0] * 100, 0.1)
    assert hot.over_material_limit is True
    empty = bt.TwoNodeRotorPad().simulate([], 0.1)
    assert (empty.T_rotor_peak_c, empty.T_pad_peak_c, empty.T_rotor_final_c, empty.t_s) == (30.0, 30.0, 30.0, [])
    zero = bt.TwoNodeRotorPad(bt.TwoNodeParams(m_pad_kg=0.0, diameter_mm=10.0))
    assert zero.C_pad == 1e-6 and zero.C_rotor == 1e-6


def _one_d(p, n, P, dt, V=None):
    m = bt.material(p.rotor_mat)
    mech = bt._MECH.get(p.rotor_mat, bt._MECH["grey_cast_iron"])
    half = max(p.thickness_mm, 0.5) / 2000
    dx = half / (n - 1)
    a = m.k_w_mK / (m.rho_kg_m3 * m.cp_j_kgK)
    ro = max(p.diameter_mm, 1.0) / 2000
    ri = max(ro - p.hat_clearance_mm / 1000, 0.02)
    face = math.pi * (ro * ro - ri * ri)
    sub = max(math.ceil(dt / (0.5 * dx * dx / a)), 1)
    h_dt = dt / sub
    Ta = p.T_ambient_c
    T = np.full(n, Ta)
    rho_cp = m.rho_kg_m3 * m.cp_j_kgK
    surf, core, sig = [], [], []
    for i, q in enumerate(P):
        q = max(q, 0.0)
        v = V[i] if V is not None and i < len(V) else 0.0
        h = _h(p.diameter_mm, v, p.duct_gain)
        flux = q * (1 - p.pad_heat_fraction) / 2 / max(face, 1e-6)
        for _ in range(sub):
            lap = np.empty(n)
            lap[1:-1] = T[:-2] - 2 * T[1:-1] + T[2:]
            lap[-1] = T[-2] - T[-1]
            lap[0] = T[1] - T[0]
            loss = 0.5 * p.area_factor * h * (T[0] - Ta)
            rad = 0.5 * p.area_factor * p.emissivity * SIG * ((T[0] + 273.15) ** 4 - (Ta + 273.15) ** 4) \
                if p.enable_radiation else 0.0
            new = T + h_dt * a * lap / (dx * dx)
            new[0] += h_dt * (flux - loss - rad) / (dx / 2) / rho_cp
            T = np.maximum(Ta, new)
        surf.append(T[0]); core.append(T[-1])
        sig.append(mech["E_gpa"] * 1e9 * mech["alpha"] * abs(T[0] - T[-1]) / (1 - mech["nu"]) / 1e6)
    return surf, core, sig, mech["sigma_lim_mpa"]


def test_the_through_thickness_model():
    P = [30000.0] * 15 + [-10.0] * 3 + [0.0] * 10
    V = [22.0 - 0.5 * i for i in range(20)]
    for prm, n in ((bt.TwoNodeParams(), 12), (bt.TwoNodeParams(rotor_mat="steel", diameter_mm=250.0,
                                                                thickness_mm=7.0, hat_clearance_mm=50.0,
                                                                area_factor=3.0, duct_gain=1.5,
                                                                pad_heat_fraction=0.2, T_ambient_c=15.0,
                                                                emissivity=0.4, enable_radiation=False,
                                                                calibrated=True), 6)):
        od = bt.OneDRotor(prm, n_nodes=n)
        tr = od.simulate(P, 0.1, v_series=V)
        s, c, sg, lim = _one_d(prm, n, P, 0.1, V)
        assert tr.T_surface_c == pytest.approx(s, rel=1e-10) and tr.T_core_c == pytest.approx(c, rel=1e-10)
        assert tr.sigma_mpa == pytest.approx(sg, rel=1e-8) and tr.sigma_limit_mpa == lim
        assert tr.dT_gradient_c == pytest.approx([a - b for a, b in zip(s, c)], rel=1e-8, abs=1e-9)
        assert tr.t_s == pytest.approx([0.1 * (i + 1) for i in range(len(P))]) and tr.n_nodes == n
        assert (tr.T_surface_peak_c, tr.dT_gradient_peak_c, tr.sigma_peak_mpa) == pytest.approx(
            (max(s), max(tr.dT_gradient_c), max(sg)))
        ratio = max(sg) / lim
        assert tr.crack_risk == ("low" if ratio < 0.5 else "elevated" if ratio < 0.85 else "high")
        assert tr.over_material_limit == bool(max(s) > bt.material(prm.rotor_mat).T_max_c)
        assert tr.synthesized is (not prm.calibrated) and tr.rotor_mass_kg == pytest.approx(bt.rotor_mass_kg(
            prm.diameter_mm, prm.thickness_mm, vented_fraction=prm.vented_fraction,
            hat_clearance_mm=prm.hat_clearance_mm, rho_kg_m3=bt.material(prm.rotor_mat).rho_kg_m3))
        assert tr.material_T_max_c == bt.material(prm.rotor_mat).T_max_c
    od = bt.OneDRotor()
    m = bt.material("grey_cast_iron")
    assert od.n == 12 and od.half_t == pytest.approx(0.0025) and od.dx == pytest.approx(0.0025 / 11)
    assert od.alpha == pytest.approx(m.k_w_mK / (m.rho_kg_m3 * m.cp_j_kgK))
    assert od._max_stable_dt() == pytest.approx(0.5 * od.dx ** 2 / od.alpha)
    assert od.face_area == pytest.approx(_ring(220.0)) and od.A_conv == pytest.approx(_ring(220.0) * 2.4)
    thin = bt.OneDRotor(bt.TwoNodeParams(thickness_mm=0.1), n_nodes=2)
    assert thin.n == 4 and thin.half_t == pytest.approx(0.00025)
    still = bt.OneDRotor(n_nodes=5).simulate(P[:8], 0.05)
    assert still.T_surface_c == pytest.approx(_one_d(bt.TwoNodeParams(), 5, P[:8], 0.05)[0], rel=1e-10)
    empty = bt.OneDRotor().simulate([], 0.1)
    assert (empty.T_surface_peak_c, empty.dT_gradient_peak_c, empty.sigma_peak_mpa, empty.crack_risk) == (
        30.0, 0.0, 0.0, "low")
    odd = bt.OneDRotor(bt.TwoNodeParams(rotor_mat="mystery"), n_nodes=6).simulate([20000.0] * 5, 0.1)
    assert odd.sigma_limit_mpa == 250.0
    hard = bt.OneDRotor(bt.TwoNodeParams(thickness_mm=12.0), n_nodes=6).simulate([3e5] * 10, 0.1)
    assert hard.crack_risk == "high" and hard.over_material_limit is True
    mid = bt.OneDRotor(bt.TwoNodeParams(thickness_mm=12.0), n_nodes=6).simulate([7e4] * 3, 0.1)
    assert mid.crack_risk == "elevated" and mid.over_material_limit is False
    low = bt.OneDRotor(bt.TwoNodeParams(thickness_mm=12.0), n_nodes=6).simulate([5e4] * 3, 0.1)
    assert low.crack_risk == "low"


def test_the_fade_test_series():
    P, V, dt = bt.fade_test_power_series(mass_kg=300.0, v0_ms=20.0, front_bias=0.6, n_stops=2, decel_g=1.0,
                                         gap_s=0.35, dt=0.1, heat_to_rotor=0.8)
    a = 9.81
    steps = int(20.0 / a / 0.1)
    one = [300 * a * 0.6 * max(20.0 - a * s * 0.1, 0.0) * 0.8 / 2 for s in range(steps)] + [0.0] * 3
    vone = [max(20.0 - a * s * 0.1, 0.0) for s in range(steps)] + [12.0] * 3
    assert dt == 0.1 and P == pytest.approx(one * 2) and V == pytest.approx(vone * 2)
    P, V, dt = bt.fade_test_power_series(mass_kg=300.0, v0_ms=20.0, front_bias=0.6, n_stops=0)
    a = 1.2 * 9.81
    steps = int(20.0 / a / 0.1)
    assert len(P) == steps + 80 and P[0] == pytest.approx(300 * a * 0.6 * 20 * 0.9 / 2)
    P, V, dt = bt.fade_test_power_series(mass_kg=300.0, v0_ms=0.3, front_bias=0.6, n_stops=1, decel_g=0.0,
                                         gap_s=0.01, dt=0.1)
    steps = max(int(0.3 / 0.5 / 0.1), 1)
    assert P == pytest.approx([300 * 0.5 * 0.6 * max(0.3 - 0.5 * s * 0.1, 0.0) * 0.9 / 2 for s in range(steps)]
                              + [0.0])
    assert V == pytest.approx([max(0.3 - 0.5 * s * 0.1, 0.0) for s in range(steps)] + [0.18])


def test_the_lap_power_series():
    assert bt.lap_power_series([0.0], [10.0], mass_kg=300.0, front_bias=0.6) == ([0.0], [0.0], 0.1)
    d = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0]
    v = [20.0, 20.0, 14.0, 10.0, 0.1, 6.0]
    P, V, dt = bt.lap_power_series(d, v, mass_kg=300.0, front_bias=0.6, dt=0.2, heat_to_rotor=0.8)
    t = [0.0]
    for i in range(1, 6):
        t.append(t[-1] + 10.0 / max(0.5 * (v[i] + v[i - 1]), 0.3))
    p = [0.0] * 6
    for i in range(5):
        if v[i + 1] < v[i]:
            p[i] = 0.5 * 300 * (v[i] ** 2 - v[i + 1] ** 2) * 0.6 * 0.8 / 2 / max(t[i + 1] - t[i], 1e-4)
    want_p, want_v, j = [], [], 0
    for s in range(int(t[-1] / 0.2)):
        while j < 5 and t[j + 1] < s * 0.2:
            j += 1
        want_p.append(p[j]); want_v.append(v[j])
    assert dt == 0.2 and P == pytest.approx(want_p) and V == pytest.approx(want_v) and max(P) > 0
    g = bt.lap_power_series(d, v, mass_kg=300.0, front_bias=0.6, dt=0.2, long_g=[0.0, 0.1, -1.0])
    assert max(g[0]) == pytest.approx(p[2] / 0.8 * 0.9)        # only sample 2 is a braking sample (and default 0.9)
    short = bt.lap_power_series([0.0, 0.01], [5.0, 4.0], mass_kg=300.0, front_bias=0.5)
    assert short[0] == [pytest.approx(0.5 * 300 * 9 * 0.5 * 0.9 / 2 / max(0.01 / 4.5, 1e-4))]
    same = bt.lap_power_series([0.0, 0.0001], [5.0, 4.0], mass_kg=300.0, front_bias=0.5, dt=1e-6)
    assert same[0][0] == pytest.approx(0.5 * 300 * 9 * 0.5 * 0.9 / 2 / 1e-4)


def test_the_duct_gain_inversion():
    p = bt.BrakeThermalParams()
    r = bt.required_duct_gain(p_brake_avg_w=1500.0, v_work_ms=15.0, diameter_mm=220.0, T_target_c=350.0, params=p,
                              hat_clearance_mm=45.0)
    A = _ring(220.0, 45.0) * 2.4
    q_rad = 0.55 * SIG * A * ((350 + 273.15) ** 4 - (30 + 273.15) ** 4)
    hn = (1500.0 - q_rad) / (A * 320.0)
    hb = bt.convection_h(p, 220.0, 15.0, 1.0)
    assert r == pytest.approx({"required_duct_gain": hn / hb, "h_needed_w_m2K": hn, "h_unducted_w_m2K": hb,
                               "achievable_unducted": hn / hb <= 1.0, "T_target_c": 350.0})
    m = bt.BrakeThermalModel(p).equilibrium(p_brake_avg_w=1500.0, v_work_ms=15.0, diameter_mm=220.0,
                                            hat_clearance_mm=45.0, duct_gain=r["required_duct_gain"])
    if r["required_duct_gain"] >= 0.1 and r["h_needed_w_m2K"] * 1.0 >= 12.0:
        assert m.T_equilibrium_c == pytest.approx(350.0, abs=0.5)
    assert bt.required_duct_gain(p_brake_avg_w=9000.0, v_work_ms=15.0, diameter_mm=220.0,
                                 T_target_c=350.0)["achievable_unducted"] is False
    easy = bt.required_duct_gain(p_brake_avg_w=50.0, v_work_ms=15.0, diameter_mm=220.0, T_target_c=350.0)
    assert easy["achievable_unducted"] is True and easy["h_needed_w_m2K"] == 0.0
    nr = bt.BrakeThermalParams(enable_radiation=False)
    r = bt.required_duct_gain(p_brake_avg_w=800.0, v_work_ms=15.0, diameter_mm=220.0, T_target_c=10.0, params=nr)
    assert r["h_needed_w_m2K"] == pytest.approx(800.0 / (_ring(220.0) * 2.4 * 1.0))


def test_the_pad_wear_index():
    assert bt.pad_wear_index(q_lap_j=2e6, T_pad_peak_c=250.0, front_bias=0.6) == pytest.approx(
        {"wear_index": 6e5 / 5e5, "temp_acceleration": 1.0, "q_per_front_pad_kj": 600.0})
    assert bt.pad_wear_index(q_lap_j=2e6, T_pad_peak_c=500.0, front_bias=0.6)["wear_index"] == pytest.approx(1.2 * 2.0)


def test_one_candidate_is_scored_on_its_fade_test():
    kw = dict(mass_kg=300.0, v0_ms=22.0, front_bias=0.62, n_stops=3, gap_s=4.0, area_factor=2.6, duct_gain=1.3)
    c = bt._evaluate_candidate(230.0, 6.0, 0.2, "steel", T_constraint_c=600.0, **kw)
    tn = bt.TwoNodeParams(rotor_mat="steel", diameter_mm=230.0, thickness_mm=6.0, vented_fraction=0.2,
                          area_factor=2.6, duct_gain=1.3)
    pw, vs, dt = bt.fade_test_power_series(mass_kg=300.0, v0_ms=22.0, front_bias=0.62, n_stops=3, gap_s=4.0)
    tr = bt.TwoNodeRotorPad(tn).simulate(pw, dt, v_series=vs)
    assert (c.diameter_mm, c.thickness_mm, c.vented_fraction, c.material) == (230.0, 6.0, 0.2, "steel")
    assert (c.rotor_mass_kg, c.T_rotor_peak_c, c.T_pad_peak_c, c.T_limit_c) == pytest.approx(
        (tr.rotor_mass_kg, tr.T_rotor_peak_c, tr.T_pad_peak_c, 600.0))
    assert c.margin_c == pytest.approx(600.0 - tr.T_rotor_peak_c) and c.feasible is (c.margin_c >= 0)
    assert (c.T_surface_peak_c, c.sigma_peak_mpa, c.crack_risk) == (0.0, 0.0, "n/a")
    capped = bt._evaluate_candidate(230.0, 6.0, 0.2, "aluminum_mmc", T_constraint_c=900.0, **kw)
    assert capped.T_limit_c == 500.0
    hot = bt._evaluate_candidate(160.0, 3.0, 0.0, "grey_cast_iron", T_constraint_c=150.0, **kw)
    assert hot.feasible is False and hot.margin_c < 0
    sc = bt._evaluate_candidate(230.0, 6.0, 0.2, "steel", T_constraint_c=900.0, screen_cracks=True, **kw)
    od = bt.OneDRotor(tn, n_nodes=6).simulate(pw, dt, v_series=vs)
    assert sc.T_surface_peak_c == pytest.approx(od.T_surface_peak_c) and sc.crack_risk == od.crack_risk
    assert sc.sigma_peak_mpa == pytest.approx(od.sigma_peak_mpa)
    assert sc.feasible is (sc.margin_c >= 0 and od.crack_risk != "high")
    crack = bt._evaluate_candidate(260.0, 12.0, 0.0, "steel", T_constraint_c=2000.0, screen_cracks=True,
                                   **{**kw, "v0_ms": 40.0, "mass_kg": 600.0})
    assert crack.crack_risk == "high" and crack.margin_c >= 0 and crack.feasible is False
    edge = bt._evaluate_candidate(230.0, 6.0, 0.2, "steel", T_constraint_c=c.T_rotor_peak_c, **kw)
    assert edge.margin_c == 0.0 and edge.feasible is True


def test_the_optimiser_picks_the_lightest_survivor_and_the_front():
    kw = dict(mass_kg=300.0, v0_ms=22.0, front_bias=0.62, n_stops=3, gap_s=4.0)
    r = bt.optimise_rotor(T_constraint_c=200.0, dia_range=(180.0, 260.0), dia_steps=3, th_range=(4.0, 8.0),
                          th_steps=2, vent_range=(0.0, 0.4), vent_steps=2, materials=["grey_cast_iron", "steel"],
                          area_factor=2.6, duct_gain=1.2, **kw)
    assert r.n_evaluated == 24 == len(r.candidates) and r.constraint_c == 200.0 and r.synthesized is True
    want = [bt._evaluate_candidate(d, t, v, m, T_constraint_c=200.0, area_factor=2.6, duct_gain=1.2, **kw)
            for m in ("grey_cast_iron", "steel") for d in (180.0, 220.0, 260.0) for t in (4.0, 8.0) for v in (0.0, 0.4)]
    assert r.candidates == want
    assert r.feasible == [c for c in want if c.feasible] and 0 < len(r.feasible) < 24
    assert r.best == min(r.feasible, key=lambda c: c.rotor_mass_kg)
    front = [c for c in r.feasible if not any(o.rotor_mass_kg <= c.rotor_mass_kg and o.T_rotor_peak_c <=
                                              c.T_rotor_peak_c and (o.rotor_mass_kg, o.T_rotor_peak_c) !=
                                              (c.rotor_mass_kg, c.T_rotor_peak_c) for o in r.feasible)]
    assert r.pareto == sorted(front, key=lambda c: c.rotor_mass_kg) and len(r.pareto) >= 1
    none = bt.optimise_rotor(T_constraint_c=31.0, dia_range=(180.0, 220.0), dia_steps=2, th_range=(4.0, 6.0),
                             th_steps=2, vent_steps=1, calibrated=True, **kw)
    assert none.best is None and none.feasible == [] and none.synthesized is False
    assert none.pareto and set(map(id, none.pareto)) <= set(map(id, none.candidates))
    assert [c.vented_fraction for c in none.candidates] == [0.0] * 4
    one = bt.optimise_rotor(T_constraint_c=900.0, dia_steps=0, th_steps=1, vent_steps=1, **kw)
    assert one.n_evaluated == 1 and (one.candidates[0].diameter_mm, one.candidates[0].thickness_mm) == (180.0, 3.0)
    sc = bt.optimise_rotor(T_constraint_c=900.0, dia_steps=1, th_steps=1, vent_steps=1, screen_cracks=True, **kw)
    assert sc.candidates[0].crack_risk != "n/a"
    tie = bt.optimise_rotor(T_constraint_c=900.0, dia_steps=1, th_steps=1, vent_steps=1,
                            materials=["grey_cast_iron", "grey_cast_iron"], **kw)
    assert len(tie.pareto) == 2                                   # identical candidates do not dominate each other


def test_the_lap_brake_energy():
    assert bt.lap_brake_energy([0.0], [5.0], mass_kg=300.0, front_bias=0.6, lap_time_s=60.0) == bt.LapBrakeEnergy(
        0.0, 0.0, 0.0, 0.0, 0.0, 0, 60.0)
    d = [0.0, 10.0, 20.0, 30.0, 30.0, 45.0]
    v = [20.0, 15.0, 15.0, 9.0, 5.0, 12.0]
    r = bt.lap_brake_energy(d, v, mass_kg=300.0, front_bias=0.6, lap_time_s=50.0, heat_to_rotor=0.8)
    dq = [0.5 * 300 * (20 ** 2 - 15 ** 2), 0.5 * 300 * (15 ** 2 - 9 ** 2), 0.5 * 300 * (9 ** 2 - 5 ** 2)]
    peak = max(dq[0] * 0.6 * 0.8 / 2 / (10 / 17.5), dq[1] * 0.6 * 0.8 / 2 / (10 / 12.0))   # the 0 m step has no time
    assert (r.q_lap_j, r.q_front_lap_j, r.p_front_avg_w, r.p_front_peak_w, r.v_brake_mean_ms, r.n_brake_samples,
            r.lap_time_s) == pytest.approx((sum(dq), sum(dq) * 0.48, sum(dq) * 0.48 / 2 / 50, peak,
                                            (17.5 + 12.0 + 7.0) / 3, 3, 50.0))
    g = bt.lap_brake_energy(d, v, mass_kg=300.0, front_bias=0.6, lap_time_s=50.0, long_g=[-1.0, 0.0, 0.0])
    assert g.n_brake_samples == 2 and g.q_lap_j == pytest.approx(dq[0] + dq[2])
    assert g.q_front_lap_j == pytest.approx((dq[0] + dq[2]) * 0.6 * 0.9)
    coast = bt.lap_brake_energy([0.0, 10.0], [10.0, 10.0], mass_kg=300.0, front_bias=0.6, lap_time_s=0.0)
    assert coast.v_brake_mean_ms == 0.0 and coast.lap_time_s == 1e-6 and coast.p_front_avg_w == 0.0
    slow = bt.lap_brake_energy([0.0, 1e-10], [0.0004, 0.0], mass_kg=300.0, front_bias=0.6, lap_time_s=10.0)
    assert slow.p_front_peak_w == 0.0 and slow.n_brake_samples == 1                      # 1e-7 s: no time to count
    fast = bt.lap_brake_energy([0.0, 2e-6], [0.0004, 0.0], mass_kg=300.0, front_bias=0.6, lap_time_s=10.0)
    assert fast.p_front_peak_w == pytest.approx(0.5 * 300 * 0.0004 ** 2 * 0.6 * 0.9 / 2 / (2e-6 / 1e-3))


def test_a_weak_convection_without_radiation_is_linear():
    p = bt.BrakeThermalParams(calibrated=True, h_air0=0.5, h_air1=0.0, enable_radiation=False, area_factor=0.5)
    A = bt.convective_area_m2(200.0, area_factor=0.5)
    r = bt.BrakeThermalModel(p).equilibrium(p_brake_avg_w=40.0, v_work_ms=0.0, diameter_mm=200.0)
    assert 0.5 * A < 1.0 and r.T_equilibrium_c == pytest.approx(p.T_ambient_c + 40.0 / (0.5 * A), rel=1e-12)


def test_both_rotor_models_take_every_geometry_parameter():
    tn = bt.TwoNodeParams(diameter_mm=240.0, thickness_mm=6.0, vented_fraction=0.3, hat_clearance_mm=55.0,
                          area_factor=3.1, rotor_mat="steel")
    rho = bt.material("steel").rho_kg_m3
    for m in (bt.TwoNodeRotorPad(tn), bt.OneDRotor(tn)):
        assert m.A_conv == pytest.approx(bt.convective_area_m2(240.0, hat_clearance_mm=55.0, area_factor=3.1))
        assert m.m_rotor == pytest.approx(bt.rotor_mass_kg(240.0, 6.0, vented_fraction=0.3, hat_clearance_mm=55.0,
                                                           rho_kg_m3=rho))


def test_one_through_thickness_step_inside_the_stability_limit_is_a_single_update():
    tn = bt.TwoNodeParams()
    m = bt.OneDRotor(tn)
    dt = 0.5 * m._max_stable_dt()
    tr = m.simulate([5000.0], dt)
    rho_cp = m.mat.rho_kg_m3 * m.mat.cp_j_kgK
    flux = 5000.0 * (1 - tn.pad_heat_fraction) / 2 / m.face_area
    assert tr.T_surface_c[0] == pytest.approx(tn.T_ambient_c + dt * flux / (m.dx / 2) / rho_cp, rel=1e-12)
    assert tr.T_core_c[0] == tn.T_ambient_c


def test_a_stop_shorter_than_one_step_still_takes_one():
    pw, vs, dt = bt.fade_test_power_series(mass_kg=300.0, v0_ms=1.0, front_bias=0.6, n_stops=1, gap_s=0.4)
    assert len(pw) == 1 + 4 and pw[0] > 0 and pw[1:] == [0.0] * 4


def test_the_lap_series_on_an_exact_time_grid():
    # t = 0, 0.5, 1.0, 1.5 exactly: a sample reached exactly at a grid time is not yet passed
    pw, vs, dt = bt.lap_power_series([0.0, 3.5, 6.0, 7.5], [8.0, 6.0, 4.0, 2.0], mass_kg=300.0, front_bias=0.6, dt=0.5)
    assert vs == [8.0, 8.0, 6.0]
    # a crawl below 0.3 m/s average is floored there, not at a faster speed
    pw, vs, dt = bt.lap_power_series([0.0, 0.4], [1.0, 0.6], mass_kg=300.0, front_bias=0.6, dt=0.1)
    assert len(vs) == 5
    # a zero longitudinal g is not braking
    pw, vs, dt = bt.lap_power_series([0.0, 10.0], [12.0, 8.0], mass_kg=300.0, front_bias=0.6, long_g=[0.0])
    assert pw == [0.0] * len(pw)


def test_the_lap_energy_peak_uses_the_step_length_not_the_position():
    r = bt.lap_brake_energy([100.0, 110.0], [20.0, 15.0], mass_kg=300.0, front_bias=0.6, lap_time_s=50.0)
    dq = 0.5 * 300 * (20 ** 2 - 15 ** 2)
    assert r.p_front_peak_w == pytest.approx(dq * 0.6 * 0.9 / 2 / (10 / 17.5), rel=1e-12)


def test_the_duct_gain_on_the_parameters_area_and_a_weak_calibrated_line():
    p = bt.BrakeThermalParams(area_factor=3.3)
    r = bt.required_duct_gain(p_brake_avg_w=900.0, v_work_ms=15.0, diameter_mm=220.0, T_target_c=350.0, params=p)
    A = bt.convective_area_m2(220.0, area_factor=3.3)
    q_rad = p.emissivity * SIG * A * ((350 + 273.15) ** 4 - (p.T_ambient_c + 273.15) ** 4)
    assert r["h_needed_w_m2K"] == pytest.approx((900.0 - q_rad) / (A * (350.0 - p.T_ambient_c)), rel=1e-12)
    w = bt.BrakeThermalParams(calibrated=True, h_air0=0.4, h_air1=0.0)
    g = bt.required_duct_gain(p_brake_avg_w=900.0, v_work_ms=15.0, diameter_mm=220.0, T_target_c=350.0, params=w)
    assert g["h_unducted_w_m2K"] == pytest.approx(0.4) and g["required_duct_gain"] == pytest.approx(
        g["h_needed_w_m2K"] / 0.4, rel=1e-12)


def test_the_optimiser_defaults():
    kw = dict(mass_kg=300.0, v0_ms=22.0, front_bias=0.62, T_constraint_c=450.0)
    d = bt.optimise_rotor(**kw)
    e = bt.optimise_rotor(materials=["grey_cast_iron"], dia_range=(180.0, 300.0), dia_steps=10, th_range=(3.0, 10.0),
                          th_steps=6, vent_range=(0.0, 0.5), vent_steps=3, n_stops=8, gap_s=6.0, area_factor=2.4,
                          duct_gain=1.0, **kw)
    assert d == e and {c.material for c in d.candidates} == {"grey_cast_iron"}


def test_the_pareto_front_by_dominance(monkeypatch):
    from types import SimpleNamespace
    pts = {180.0: (1.0, 100.0), 190.0: (1.0, 90.0), 200.0: (2.0, 80.0), 210.0: (3.0, 80.0), 220.0: (2.5, 85.0)}
    monkeypatch.setattr(bt, "_evaluate_candidate", lambda dia, th, vent, mat, **k: SimpleNamespace(
        dia=dia, rotor_mass_kg=pts[dia][0], T_rotor_peak_c=pts[dia][1], feasible=True))
    r = bt.optimise_rotor(mass_kg=300.0, v0_ms=22.0, front_bias=0.6, T_constraint_c=1.0, dia_range=(180.0, 220.0),
                          dia_steps=5, th_steps=1, vent_steps=1)
    assert [c.dia for c in r.pareto] == [190.0, 200.0]
