"""suspension/tire_thermal.py: one explicit-Euler step of the three-node energy balance re-derived here with every
parameter set to a distinct non-default value, the mu(T) law on both sides of its optimum, the ideal-gas pressure, the
band weights, the provenance and warning wording, the failure path, and the warm-up run."""
import math

import numpy as np
import pytest

from suspension import tire_thermal as tt
from suspension.tire_cosim import ReferenceTireModel, TireFidelity, WheelState


def test_the_parameter_defaults():
    p = tt.default_thermal_params()
    assert (p.n_bands, p.m_tread_kg, p.m_carcass_kg, p.m_gas_kg, p.cp_tread, p.cp_carcass, p.cv_gas,
            p.k_tread_carcass, p.k_carcass_gas, p.h_air0, p.h_air1, p.area_tread_m2, p.k_track, p.area_carcass_m2,
            p.fric_to_tread, p.roll_resist_coeff, p.eff_radius_m, p.cold_pressure_pa, p.atm_pressure_pa,
            p.enable_mu_feedback, p.T_opt_c, p.mu_gain_per_C, p.mu_gain_per_C_hot, p.mu_floor, p.calibrated,
            p.fitted_to) == (3, 1.2, 3.8, 0.035, 1900.0, 1100.0, 718.0, 18.0, 6.0, 12.0, 5.5, 0.16, 22.0, 0.35,
                             0.72, 0.018, 0.22, 83000.0, 101325.0, False, 85.0, 0.0042, 0.0065, 0.55, False, "")


def test_convection_and_the_grip_law():
    p = tt.ThermalParams()
    assert p.h_air(-10.0) == pytest.approx(12.0 + 55.0) and p.h_air(0.0) == 12.0
    assert p.mu_scale(20.0) == 1.0
    on = tt.ThermalParams(enable_mu_feedback=True)
    assert on.mu_scale(65.0) == pytest.approx(1 - 0.0042 * 20)
    assert on.mu_scale(105.0) == pytest.approx(1 - 0.0065 * 20)
    assert on.mu_scale(85.0) == 1.0 and on.mu_scale(-200.0) == 0.55 and on.mu_scale(300.0) == 0.55


P = tt.ThermalParams(n_bands=2, m_tread_kg=1.5, m_carcass_kg=3.0, m_gas_kg=0.04, cp_tread=1800.0, cp_carcass=1200.0,
                     cv_gas=700.0, k_tread_carcass=15.0, k_carcass_gas=5.0, h_air0=10.0, h_air1=4.0,
                     area_tread_m2=0.18, k_track=20.0, area_carcass_m2=0.3, fric_to_tread=0.7,
                     roll_resist_coeff=0.02, cold_pressure_pa=80000.0, atm_pressure_pa=100000.0)


def _model(params=P):
    m = tt.ThermalTireModel(params=params)
    m.T_tread = np.array([60.0, 70.0])
    m.T_carcass, m.T_gas = 45.0, 35.0
    return m


def test_one_step_of_the_energy_balance():
    m = _model()
    ws = WheelState(alpha=0.05, kappa=0.03, gamma=0.02, Fz=1000.0, v_x=15.0, ambient_temp_c=20.0, track_temp_c=30.0,
                    dt=0.01)
    ref = ReferenceTireModel(lateral=m.lateral)
    f = ref.step(ws)
    Q = (abs(f.Fy * 15.0 * math.tan(0.05)) + abs(f.Fx * 15.0 * 0.03) + 0.02 * 1000.0 * 15.0) * 0.7
    bias = min(abs(math.tan(0.05)) + 0.5 * 0.02, 0.6)
    w = np.array([1 - bias, 1 + bias]) / 2.0
    h = 10.0 + 4.0 * 15.0
    Tt, Tc, Tg = np.array([60.0, 70.0]), 45.0, 35.0
    dT = [(Q * w[i] - 7.5 * (Tt[i] - Tc) - h * 0.09 * (Tt[i] - 20.0) - 10.0 * (Tt[i] - 30.0)) / (0.75 * 1800.0) * 0.01
          for i in range(2)]
    dTc = (15.0 * (65.0 - Tc) - 5.0 * (Tc - Tg) - h * 0.3 * (Tc - 20.0)) / (3.0 * 1200.0) * 0.01
    dTg = 5.0 * (Tc - Tg) / (0.04 * 700.0) * 0.01
    out = m.step(ws)
    assert out.tread_temp_c == pytest.approx(Tt + dT) and m.T_tread == pytest.approx(Tt + dT)
    assert out.carcass_temp_c == pytest.approx(Tc + dTc) and out.gas_temp_c == pytest.approx(Tg + dTg)
    assert (out.Fx, out.Fy) == pytest.approx((f.Fx, f.Fy))
    assert out.inflation_pressure_pa == pytest.approx(180000.0 * (Tg + dTg + 273.15) / 298.15 - 100000.0)
    assert out.synthesized == ["tread_temp_c", "carcass_temp_c", "gas_temp_c", "inflation_pressure_pa",
                               "carcass_deflection_m", "contact_length_m", "contact_width_m", "pressure_distribution"]


def test_off_the_ground_and_out_of_range_inputs():
    m = _model()
    ws = WheelState(alpha=float("nan"), kappa=5.0, Fz=0.5, v_x=-3.0, ambient_temp_c=float("inf"), track_temp_c=200.0,
                    dt=5.0)
    h = 10.0
    Tt, Tc, Tg = np.array([60.0, 70.0]), 45.0, 35.0
    dT = [(-7.5 * (Tt[i] - Tc) - h * 0.09 * (Tt[i] - 25.0)) / (0.75 * 1800.0) * 1.0 for i in range(2)]
    out = m.step(ws)                     # v clamped to 0 (no heat), dt to 1 s, ambient defaulted to 25, Fz < 1: airborne
    assert out.tread_temp_c == pytest.approx(Tt + dT)
    hot = _model()
    hot.T_tread = np.array([340.0, 349.0])
    hot.step(WheelState(alpha=1.4, Fz=5e4, v_x=150.0, dt=1.0, ambient_temp_c=80.0, track_temp_c=120.0))
    assert hot.T_tread.max() <= 350.0 and hot.T_carcass <= 300.0 and hot.T_gas <= 250.0
    cold = _model()
    cold.T_tread = np.array([-39.0, -39.0])
    cold.T_carcass = cold.T_gas = -39.0
    cold.step(WheelState(Fz=1000.0, v_x=0.0, dt=1.0, ambient_temp_c=-40.0, track_temp_c=-40.0))
    assert cold.T_tread.min() >= -40.0


def test_one_band_and_the_band_weights():
    m = tt.ThermalTireModel(params=tt.ThermalParams(n_bands=1))
    assert list(m._band_weights(WheelState(alpha=0.3))) == [1.0]
    z = tt.ThermalTireModel(params=tt.ThermalParams(n_bands=0))
    assert len(z.T_tread) == 1
    m3 = tt.ThermalTireModel()
    w = m3._band_weights(WheelState(alpha=0.1, gamma=-0.04))
    b = abs(math.tan(0.1)) + 0.02
    assert w == pytest.approx(np.linspace(1 - b, 1 + b, 3) / 3.0)
    big = m3._band_weights(WheelState(alpha=1.4, gamma=0.5))
    r = np.clip(np.linspace(0.4, 1.6, 3), 0.05, None)
    assert big == pytest.approx(r / r.sum())
    bad = m3._band_weights(WheelState(alpha=float("nan"), gamma=float("nan")))
    assert bad == pytest.approx([1 / 3] * 3)
    neg = m3._band_weights(WheelState(alpha=-2.0))
    bb = min(abs(math.tan(-1.5)), 0.6)
    assert neg == pytest.approx(np.clip(np.linspace(1 - bb, 1 + bb, 3), 0.05, None) / 3.0 / (
        np.clip(np.linspace(1 - bb, 1 + bb, 3), 0.05, None).sum() / 3.0))


def test_initial_state_reset_and_pressure():
    m = tt.ThermalTireModel(init_temp_c=40.0)
    assert list(m.T_tread) == [40.0] * 3 and (m.T_carcass, m.T_gas, m._mu_scale_last) == (40.0, 40.0, 1.0)
    m.T_gas = 80.0
    assert m.hot_pressure_pa() == pytest.approx((83000 + 101325) * (353.15 / 298.15) - 101325)
    m.reset(WheelState(ambient_temp_c=20.0, track_temp_c=40.0))
    assert list(m.T_tread) == [30.0] * 3 and m.T_gas == 30.0
    m.T_gas = 99.0
    m.reset()
    assert m.T_gas == 40.0 and m.mean_tread_c() == 40.0
    d = tt.ThermalTireModel()
    assert d.T_carcass == 25.0 and d.hot_pressure_pa() == pytest.approx(83000.0)


def test_provenance_synthesised_channels_and_warnings():
    m = tt.ThermalTireModel()
    pv = m.provenance()
    assert (pv.backend, pv.fidelity, pv.is_calibrated, pv.parameter_file, pv.fitted_to) == (
        "lumped-thermal", TireFidelity.THERMAL, False, None, "")
    assert pv.notes.startswith("3-node lumped thermal network (tread/carcass/gas) on a Pacejka force core. UNCALIBRATED:")
    cal = tt.ThermalTireModel(params=tt.ThermalParams(calibrated=True, fitted_to="rig 2026"))
    pc = cal.provenance()
    assert pc.is_calibrated is True and pc.fitted_to == "rig 2026" and pc.notes == (
        "3-node lumped thermal network (tread/carcass/gas) on a Pacejka force core, calibrated to temperature-swept "
        "data.")
    assert cal._synth() == ["carcass_deflection_m", "contact_length_m", "contact_width_m", "pressure_distribution"]
    fb = tt.ThermalTireModel(params=tt.ThermalParams(enable_mu_feedback=True))
    assert fb._synth()[-1] == "mu_thermal" and "mu_thermal" not in tt.ThermalTireModel(
        params=tt.ThermalParams(enable_mu_feedback=True, calibrated=True))._synth()
    msg = ("thermal channels are UNCALIBRATED (representative parameters, no temperature-swept data) — flagged "
           "synthesized; do not read as absolute temperature.")
    assert m.warnings().count(msg) == 1 and msg not in cal.warnings()
    m._warn("x")
    m._warn("x")
    assert m.warnings().count("x") == 1
    w = [msg]
    m._uncal_warn(w)
    assert w == [msg]


def test_the_grip_feedback_scales_the_peak_and_is_restored():
    p = tt.ThermalParams(enable_mu_feedback=True, T_opt_c=85.0)
    m = tt.ThermalTireModel(params=p)
    m.T_tread = np.array([45.0, 45.0, 45.0])
    ws = WheelState(alpha=0.08, Fz=1000.0, v_x=10.0, dt=0.001)
    base = ReferenceTireModel(lateral=tt.default_tire()).step(ws).Fy
    out = m.step(ws)
    assert m._mu_scale_last == pytest.approx(1 - 0.0042 * 40)
    assert abs(out.Fy) < abs(base) and m.lateral.scaling["LMUY"] == 1.0
    off = tt.ThermalTireModel()
    off._apply_mu_scale(0.5)
    assert off.lateral.scaling["LMUY"] == 1.0
    off._restore_mu_scale()


def test_a_failing_step_holds_the_temperatures():
    m = _model()
    out = m.step(WheelState(alpha=None, Fz=800.0))           # float(None) fails inside the step
    assert list(out.tread_temp_c) == [60.0, 70.0] and out.carcass_temp_c == 45.0 and out.gas_temp_c == 35.0
    assert any(w.startswith("thermal step failed (TypeError); held previous temperatures") for w in m.warnings())

    class Broken:
        lateral = tt.default_tire()

        def step(self, ws):
            raise RuntimeError("boom")

        def reset(self, s=None):
            pass

        def warnings(self):
            return []
    b = tt.ThermalTireModel(reference=Broken())
    o = b.step(WheelState(Fz=-5.0))
    assert (o.Fx, o.Fy, o.Fz) == (0.0, 0.0, 0.0) and o.inflation_pressure_pa == pytest.approx(83000.0)
    o2 = b.step(WheelState(Fz=700.0))
    assert o2.Fz == 700.0 and o2.synthesized[0] == "tread_temp_c"


def test_the_warm_up_run():
    r = tt.simulate_warmup(duration_s=2.0, dt=0.01)
    assert len(r.t) == 200 and r.t[1] == pytest.approx(0.01) and r.tread_c.shape == (200, 3)
    assert r.tread_mean_c() == pytest.approx(r.tread_c.mean(axis=1))
    assert r.tread_mean_c()[-1] > r.tread_mean_c()[0] > 28.5 - 1e-9
    assert r.calibrated is False and "UNCALIBRATED" in r.status and r.warnings
    assert np.all(r.mu_scale == 1.0) and np.all(np.abs(r.Fy) > 0)
    assert r.pressure_pa == pytest.approx(184325.0 * (r.gas_c + 273.15) / 298.15 - 101325.0)
    assert r.carcass_c[-1] != r.carcass_c[0]
    m = tt.ThermalTireModel(params=tt.ThermalParams(enable_mu_feedback=True))
    r2 = tt.simulate_warmup(m, alpha_deg=6.0, Fz=900.0, v_x=25.0, gamma_deg=-2.0, kappa=0.05, ambient_c=10.0,
                            track_c=20.0, duration_s=1.0, dt=0.02)
    assert len(r2.t) == 50 and r2.tread_c[0].mean() > 15.0 and r2.mu_scale[0] < 1.0
    one = tt.simulate_warmup(duration_s=0.0)
    assert len(one.t) == 1
    assert tt.psi(6894.757) == pytest.approx(1.0) and tt.psi(0) == 0.0


def _manual_run(m, n, **kw):
    """The warm-up loop written out: reset to the mean of ambient and track, then step a fixed wheel state."""
    c = dict(alpha_deg=3.0, Fz=1100.0, v_x=18.0, gamma_deg=1.0, kappa=0.0, ambient_c=25.0, track_c=32.0, dt=5e-3)
    c.update(kw)
    m.reset(WheelState(ambient_temp_c=c["ambient_c"], track_temp_c=c["track_c"]))
    ws = WheelState(alpha=math.radians(c["alpha_deg"]), Fz=c["Fz"], v_x=c["v_x"], gamma=math.radians(c["gamma_deg"]),
                    kappa=c["kappa"], ambient_temp_c=c["ambient_c"], track_temp_c=c["track_c"], dt=c["dt"])
    outs = [m.step(ws) for _ in range(n)]
    return [o.carcass_temp_c for o in outs], [o.Fy for o in outs], [o.gas_temp_c for o in outs]


def test_the_warm_up_hands_every_input_to_the_step():
    kw = dict(alpha_deg=5.0, Fz=950.0, v_x=22.0, gamma_deg=-2.5, kappa=0.04, ambient_c=12.0, track_c=40.0, dt=0.01)
    r = tt.simulate_warmup(tt.ThermalTireModel(), duration_s=0.3, **kw)
    carc, fy, gas = _manual_run(tt.ThermalTireModel(), 30, **kw)
    assert r.carcass_c == pytest.approx(carc) and r.Fy == pytest.approx(fy) and r.gas_c == pytest.approx(gas)
    d = tt.simulate_warmup()
    assert len(d.t) == 24000 and d.t[1] == pytest.approx(5e-3)
    carc, fy, gas = _manual_run(tt.ThermalTireModel(), 50)
    assert d.carcass_c[:50] == pytest.approx(carc) and d.Fy[:50] == pytest.approx(fy)


def test_the_reference_backend_shares_the_lateral_and_hears_the_reset():
    lat = tt.default_tire()
    assert tt.ThermalTireModel(lateral=lat)._ref.lateral is lat
    seen = []

    class Ref(ReferenceTireModel):
        def reset(self, state=None):
            seen.append(state)
    m = tt.ThermalTireModel(reference=Ref(lateral=lat))
    s = WheelState(ambient_temp_c=10.0, track_temp_c=30.0)
    m.reset(s)
    assert seen == [s]


def test_the_grip_law_just_past_the_optimum():
    p = tt.ThermalParams(enable_mu_feedback=True, T_opt_c=85.0, mu_gain_per_C=0.01, mu_gain_per_C_hot=0.002)
    assert p.mu_scale(85.5) == pytest.approx(1 - 0.002 * 0.5) and p.mu_scale(84.5) == pytest.approx(1 - 0.01 * 0.5)


def test_the_grip_scale_restores_the_declared_lmuy():
    p = tt.ThermalParams(enable_mu_feedback=True, T_opt_c=85.0)
    lat = tt.default_tire()
    lat.scaling = {**lat.scaling, "LMUY": 0.7}
    m = tt.ThermalTireModel(lateral=lat, params=p)
    m.T_tread = np.full(len(m.T_tread), 45.0)
    ws = WheelState(alpha=0.08, Fz=1000.0, v_x=10.0, dt=0.001)
    out = m.step(ws)
    assert lat.scaling["LMUY"] == 0.7
    ref = tt.default_tire()
    ref.scaling = {**ref.scaling, "LMUY": 0.7 * m._mu_scale_last}
    assert out.Fy == pytest.approx(ReferenceTireModel(lateral=ref).step(ws).Fy)
    bare = tt.default_tire()
    bare.scaling = {}
    mb = tt.ThermalTireModel(lateral=bare, params=p)
    mb._apply_mu_scale(0.5)
    assert mb._mu_saved == 1.0 and bare.scaling == {"LMUY": 0.5}
    mb._restore_mu_scale()
    assert bare.scaling == {"LMUY": 1.0}
    mb._restore_mu_scale()
    assert bare.scaling == {"LMUY": 1.0}                                   # a second restore changes nothing


def test_the_band_weights_clamp_the_slip_angle():
    m = tt.ThermalTireModel()
    for a, b in ((2.0, 1.5), (-2.0, -1.5)):
        w = m._band_weights(WheelState(alpha=a, gamma=0.0))
        assert w == pytest.approx(m._band_weights(WheelState(alpha=b, gamma=0.0)))


def test_a_non_finite_slip_reads_as_zero_and_a_light_wheel_still_touches():
    a, b = tt.ThermalTireModel(), tt.ThermalTireModel()
    ws = dict(Fz=900.0, v_x=15.0, dt=0.01, kappa=0.0)
    oa = a.step(WheelState(alpha=float("nan"), **ws))
    ob = b.step(WheelState(alpha=0.0, **ws))
    assert list(oa.tread_temp_c) == pytest.approx(list(ob.tread_temp_c))
    hot = dict(alpha=0.0, Fz=1.5, v_x=0.0, dt=0.05, ambient_temp_c=25.0, track_temp_c=60.0)
    c = tt.ThermalTireModel()
    d = tt.ThermalTireModel(params=tt.ThermalParams(k_track=0.0))
    assert c.step(WheelState(**hot)).tread_temp_c[0] > d.step(WheelState(**hot)).tread_temp_c[0]


def test_massless_nodes_hit_the_temperature_clamps():
    p = tt.ThermalParams(m_tread_kg=0.0, m_carcass_kg=0.0, m_gas_kg=0.0)
    hot = tt.ThermalTireModel(params=p, init_temp_c=60.0)
    o = hot.step(WheelState(alpha=0.3, Fz=1500.0, v_x=30.0, dt=0.01, ambient_temp_c=25.0, track_temp_c=30.0))
    assert set(np.round(o.tread_temp_c, 6)) <= {350.0, -40.0} and o.carcass_temp_c in (300.0, -40.0)
    cold = tt.ThermalTireModel(params=p, init_temp_c=20.0)
    o = cold.step(WheelState(alpha=0.0, Fz=0.0, v_x=0.0, dt=0.01, ambient_temp_c=-30.0, track_temp_c=-30.0))
    assert list(o.tread_temp_c) == [-40.0] * len(o.tread_temp_c) and o.carcass_temp_c == -40.0
    g = tt.ThermalTireModel(params=p, init_temp_c=20.0)
    g.T_carcass = 200.0
    assert g.step(WheelState(alpha=0.0, Fz=0.0, v_x=0.0, dt=0.01)).gas_temp_c == 250.0
    g2 = tt.ThermalTireModel(params=p, init_temp_c=20.0)
    g2.T_carcass = -39.0
    assert g2.step(WheelState(alpha=0.0, Fz=0.0, v_x=0.0, dt=0.01)).gas_temp_c == -40.0


def test_the_uncalibrated_note():
    assert tt.ThermalTireModel().provenance().notes == (
        "3-node lumped thermal network (tread/carcass/gas) on a Pacejka force core. UNCALIBRATED: equations are "
        "textbook energy balance, but masses, heat-transfer coefficients and the mu(T) law are representative defaults, "
        "NOT fitted to temperature-swept tyre data. Treat temperatures as a physically-shaped warm-up curve, good for "
        "relative/setup work, NOT as measured degrees. Every thermal channel is flagged synthesized.")
    assert tt.ThermalTireModel().provenance().parameter_file is None


def test_a_non_finite_longitudinal_force_carries_no_heat():
    class NanFx(ReferenceTireModel):
        def step(self, ws):
            o = super().step(ws)
            o.Fx = float("nan")
            return o
    a = tt.ThermalTireModel(reference=NanFx(lateral=tt.default_tire()))
    b = tt.ThermalTireModel()
    ws = WheelState(alpha=0.05, kappa=0.0, Fz=900.0, v_x=15.0, dt=0.01)
    assert list(a.step(ws).tread_temp_c) == pytest.approx(list(b.step(ws).tread_temp_c))


class _Flat(ReferenceTireModel):
    """Constant forces whatever the wheel state, so only the thermal sanitising decides the heat."""
    seen = None

    def step(self, ws):
        self.seen = ws
        o = super().step(WheelState(alpha=0.05, Fz=900.0, v_x=15.0, dt=0.001))
        o.Fy, o.Fx = 600.0, 250.0
        return o


def _tread(ws, p=None):
    m = tt.ThermalTireModel(reference=_Flat(lateral=tt.default_tire()), params=p)
    o = m.step(ws)
    return [*o.tread_temp_c, o.carcass_temp_c]          # flat: pytest.approx does not take nested sequences


def test_every_input_is_clamped_to_its_physical_range():
    base = dict(alpha=0.1, kappa=0.05, v_x=20.0, Fz=900.0, dt=0.01, ambient_temp_c=25.0, track_temp_c=40.0)
    for k, wild, tame in (("alpha", 2.0, 1.5), ("alpha", -2.0, -1.5), ("kappa", 1.6, 1.0), ("kappa", -1.6, -1.0),
                          ("v_x", 200.0, 150.0), ("v_x", -5.0, 0.0), ("Fz", 2e5, 1e5), ("Fz", -50.0, 0.0),
                          ("ambient_temp_c", 100.0, 80.0), ("ambient_temp_c", -60.0, -40.0),
                          ("track_temp_c", 150.0, 120.0), ("track_temp_c", -60.0, -40.0),
                          ("track_temp_c", float("nan"), 30.0), ("ambient_temp_c", float("nan"), 25.0),
                          ("alpha", float("nan"), 0.0)):
        assert _tread(WheelState(**{**base, k: wild})) == pytest.approx(_tread(WheelState(**{**base, k: tame}))), k
    assert _tread(WheelState(**{**base, "alpha": 1.2})) != pytest.approx(_tread(WheelState(**{**base, "alpha": 1.5})))


def test_non_finite_forces_carry_no_heat_while_sliding():
    class NanFy(_Flat):
        def step(self, ws):
            o = super().step(ws)
            o.Fy = float("nan")
            return o

    class NanFx(_Flat):
        def step(self, ws):
            o = super().step(ws)
            o.Fx = float("nan")
            return o
    ws = WheelState(alpha=0.1, kappa=0.05, v_x=20.0, Fz=900.0, dt=0.01)
    for cls, zero in ((NanFy, "Fy"), (NanFx, "Fx")):
        a = tt.ThermalTireModel(reference=cls(lateral=tt.default_tire()))

        class Z(_Flat):
            def step(self, w, _z=zero):
                o = _Flat.step(self, w)
                setattr(o, _z, 0.0)
                return o
        b = tt.ThermalTireModel(reference=Z(lateral=tt.default_tire()))
        assert list(a.step(ws).tread_temp_c) == pytest.approx(list(b.step(ws).tread_temp_c))


def test_the_carcass_clamps_hot_and_the_failure_path_retries_the_same_state():
    p = tt.ThermalParams(m_carcass_kg=0.0)
    m = tt.ThermalTireModel(params=p, init_temp_c=20.0)
    m.T_tread = np.full(len(m.T_tread), 300.0)
    assert m.step(WheelState(Fz=900.0, v_x=5.0, dt=0.01)).carcass_temp_c == 300.0
    ref = _Flat(lateral=tt.default_tire())
    f = tt.ThermalTireModel(reference=ref)
    ws = WheelState(alpha=0.1, Fz=900.0, v_x=10.0, dt=None)                    # float(None) fails in the thermal step
    o = f.step(ws)
    assert ref.seen is ws and o.Fy == 600.0


def test_the_warm_up_tread_follows_every_input_exactly():
    kw = dict(alpha_deg=5.0, Fz=950.0, v_x=22.0, gamma_deg=-2.5, kappa=0.04, ambient_c=12.0, track_c=40.0, dt=0.01)
    r = tt.simulate_warmup(tt.ThermalTireModel(), duration_s=0.3, **kw)
    m = tt.ThermalTireModel()
    m.reset(WheelState(ambient_temp_c=12.0, track_temp_c=40.0))
    ws = WheelState(alpha=math.radians(5.0), Fz=950.0, v_x=22.0, gamma=math.radians(-2.5), kappa=0.04,
                    ambient_temp_c=12.0, track_temp_c=40.0, dt=0.01)
    tread = np.array([m.step(ws).tread_temp_c for _ in range(30)])
    assert r.tread_c == pytest.approx(tread, rel=1e-13)


class _ForcesOf(_Flat):
    """_Flat with chosen tyre forces, so the heat from friction can be compared between force sets."""

    def __init__(self, fx, fy, **kw):
        super().__init__(**kw)
        self.fx, self.fy = fx, fy

    def step(self, ws):
        o = super().step(ws)
        o.Fy, o.Fx = self.fy, self.fx
        return o


def test_non_finite_forces_heat_like_no_force_and_the_track_counts():
    ws = WheelState(alpha=0.1, kappa=0.08, v_x=20.0, Fz=900.0, dt=0.01, ambient_temp_c=25.0, track_temp_c=40.0)

    def tread(fx, fy, w=ws):
        o = tt.ThermalTireModel(reference=_ForcesOf(fx, fy, lateral=tt.default_tire())).step(w)
        return [*o.tread_temp_c, o.carcass_temp_c]
    assert tread(float("nan"), float("nan")) == pytest.approx(tread(0.0, 0.0), rel=1e-12)
    assert tread(float("inf"), 0.0) == pytest.approx(tread(0.0, 0.0), rel=1e-12)
    assert tread(0.0, float("-inf")) == pytest.approx(tread(0.0, 0.0), rel=1e-12)
    assert tread(400.0, 0.0) != pytest.approx(tread(0.0, 0.0), rel=1e-12)       # a finite Fx does heat the tread
    assert tread(0.0, 400.0) != pytest.approx(tread(0.0, 0.0), rel=1e-12)
    hot = WheelState(**{**vars(ws), "track_temp_c": 90.0}) if hasattr(ws, "__dict__") else None
    assert tread(250.0, 600.0, hot) != pytest.approx(tread(250.0, 600.0), rel=1e-12)   # the track temperature is read
