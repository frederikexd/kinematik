"""suspension/thermic_patch.py: the parabolic grip window (hot side narrowed by the slope ratio, clipped at the floor),
the sliding-power flash flux |F| u |sin alpha| over the area, the three-node radial ladder against a stepper written
here corner by corner, the loss ranking in newtons, the summaries and the verdict sentence."""
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

from suspension import thermic_patch as tp_
from suspension.tire_thermal import ThermalParams

TP = ThermalParams(T_opt_c=85.0, mu_gain_per_C=0.004, mu_gain_per_C_hot=0.008, mu_floor=0.6)


def test_the_parabolic_window():
    T = np.array([50.0, 85.0, 95.0, 120.0, 200.0, 0.0])
    hw_hot = 35.0 * 0.004 / 0.008
    want = [1 - (-35 / 35) ** 2, 1.0, 1 - (10 / hw_hot) ** 2, 1 - (35 / hw_hot) ** 2, 1 - (115 / hw_hot) ** 2,
            1 - (85 / 35) ** 2]
    assert tp_.parabolic_mu_scale(T, TP) == pytest.approx(np.clip(want, 0.6, 1.0))
    assert tp_.parabolic_mu_scale(70.0, TP, half_width_c=20.0, floor=0.1) == pytest.approx(1 - (15 / 20) ** 2)
    assert tp_.parabolic_mu_scale(84.0, TP, half_width_c=1.0, floor=-5.0) == pytest.approx(0.0)
    flat = ThermalParams(mu_gain_per_C=0.0, mu_gain_per_C_hot=0.0, T_opt_c=85.0, mu_floor=0.0)
    assert tp_.parabolic_mu_scale(86.0, flat) == pytest.approx(1 - (1 / 35) ** 2)
    assert tp_.parabolic_mu_scale(86.0, TP, half_width_c=0.0, floor=-1e12) == pytest.approx(1 - (1 / 1e-6) ** 2)


def test_the_flash_flux():
    q = tp_._sliding_power_flux(np.array([200.0, -400.0]), np.array([-1500.0, 900.0]), np.array([0.08, -0.05]),
                                np.array([-20.0, 15.0]), TP, 0.12)
    want = [TP.fric_to_tread * math.hypot(1500, 70) * 20 * abs(math.sin(0.08)) / 0.12,
            TP.fric_to_tread * math.hypot(900, 140) * 15 * abs(math.sin(-0.05)) / 0.12]
    assert q == pytest.approx(want)
    assert tp_._sliding_power_flux(0.0, 100.0, 0.1, 10.0, TP, 0.0) == pytest.approx(
        TP.fric_to_tread * 100 * 10 * math.sin(0.1) / 1e-6)


def _res(n=25, dt=0.01, slide=0.12, u=18.0, Fz=900.0, ok=True, meta=None):
    t = np.arange(n) * dt
    ramp = np.minimum(t / (n * dt / 2), 1.0)[:, None]
    Fy = (np.array([1200.0, -1500.0, 800.0, -1000.0]) * ramp)
    Fx = np.tile([100.0, 150.0, -300.0, -250.0], (n, 1)) * ramp
    alpha = np.array([slide, -slide, 0.6 * slide, -0.6 * slide]) * ramp
    Fzs = np.tile([Fz, Fz * 1.2, Fz * 0.9, Fz * 1.1], (n, 1))
    return NS(ok=ok, t=t, Fz=Fzs, Fy=Fy, Fx=Fx, alpha=alpha, u=np.full(n, u), meta=meta)


def _ladder(res, tp, lp, T0=55.0, amb=30.0, gas=None):
    gas = T0 if gas is None else gas
    t, n = res.t, len(res.t)
    Cs = max(tp.m_tread_kg * lp.surface_mass_frac * tp.cp_tread, 1e-6)
    Cc = max(tp.m_tread_kg * (1 - lp.surface_mass_frac) * tp.cp_tread, 1e-6)
    Ck = max(tp.m_carcass_kg * tp.cp_carcass, 1e-6)
    A = tp.area_tread_m2
    kt = tp.k_track * lp.surface_to_road_frac
    Q = tp_._sliding_power_flux(res.Fx, res.Fy, res.alpha, res.u[:, None], tp, A) * A
    Qr = tp.roll_resist_coeff * np.abs(res.Fz) * tp.eff_radius_m * (np.abs(res.u) / tp.eff_radius_m)[:, None]
    S, C, K = [np.full(4, T0)], [np.full(4, T0)], [np.full(4, T0)]
    for i in range(1, n):
        dtl = t[i] - t[i - 1]
        if not (dtl > 0):
            S.append(S[-1]); C.append(C[-1]); K.append(K[-1]); continue
        m = max(1, math.ceil(dtl / lp.max_substep_s))
        dt = dtl / m
        h = (tp.h_air0 + tp.h_air1 * abs(res.u[i])) * A * lp.surface_mass_frac
        qg, qr = 0.5 * (Q[i] + Q[i - 1]), 0.5 * (Qr[i] + Qr[i - 1])
        load = 0.5 * (res.Fz[i] + res.Fz[i - 1]) > 1.0
        s, c, k = S[-1].copy(), C[-1].copy(), K[-1].copy()
        for _ in range(m):
            road = np.where(load, kt * (s - lp.track_temp_c), 0.0)
            s, c, k = (s + dt * (qg - h * (s - amb) - road - lp.k_surface_core * (s - c)) / Cs,
                       c + dt * (lp.k_surface_core * (s - c) - lp.k_core_carcass * (c - k)) / Cc,
                       k + dt * (lp.k_core_carcass * (c - k) + qr - lp.k_carcass_gas * (k - gas)) / Ck)
        S.append(np.clip(s, -40, 400)); C.append(np.clip(c, -40, 400)); K.append(np.clip(k, -40, 400))
    return np.array(S), np.array(C), np.array(K), Q / A


def test_the_ladder_matches_a_hand_stepper():
    lp = tp_.LadderParams()
    assert (lp.surface_mass_frac, lp.k_surface_core, lp.k_core_carcass, lp.k_carcass_gas, lp.surface_to_road_frac,
            lp.track_temp_c, lp.max_substep_s) == (0.10, 14.0, 16.0, 6.0, 0.22, 42.0, 2.0e-3)
    assert tp_.default_ladder_params() == lp
    res = _res(meta={"maneuver": "step"})
    r = tp_.run_thermic_patch(res)
    S, C, K, q = _ladder(res, ThermalParams(), lp)
    assert r.ok and r.T_surface == pytest.approx(S, rel=1e-12) and r.T_core == pytest.approx(C, rel=1e-12)
    assert r.T_carcass == pytest.approx(K, rel=1e-12) and r.q_flash == pytest.approx(q)
    ms = tp_.parabolic_mu_scale(C, ThermalParams())
    assert r.mu_scale == pytest.approx(ms) and r.Fy_cold == pytest.approx(np.abs(res.Fy))
    assert r.Fy_hot == pytest.approx(np.abs(res.Fy) * ms)
    assert r.dFy_frac == pytest.approx(np.where(np.abs(res.Fy) > 1.0, 1 - ms, 0.0))
    assert r.meta == {"maneuver": "step", "init_temp_c": 55.0, "ambient_c": 30.0, "gas_c": 55.0, "half_width_c": 35.0}
    assert r.T_opt_c == ThermalParams().T_opt_c and r.calibrated is False and r.warnings == []
    lp2 = tp_.LadderParams(surface_mass_frac=0.2, k_surface_core=10.0, k_core_carcass=12.0, k_carcass_gas=4.0,
                           surface_to_road_frac=0.3, track_temp_c=50.0, max_substep_s=4e-3)
    tp2 = ThermalParams(T_opt_c=80.0, calibrated=True)
    res2 = _res(n=12, dt=0.03, slide=0.2, u=25.0, Fz=0.4)                         # coarse log, patch off the road
    r2 = tp_.run_thermic_patch(res2, tp2, lp2, init_temp_c=70.0, ambient_c=20.0, gas_c=60.0, half_width_c=25.0)
    S, C, K, _ = _ladder(res2, tp2, lp2, 70.0, 20.0, 60.0)
    assert r2.T_surface == pytest.approx(S, rel=1e-12) and r2.T_carcass == pytest.approx(K, rel=1e-12)
    assert r2.mu_scale == pytest.approx(tp_.parabolic_mu_scale(C, tp2, half_width_c=25.0))
    assert r2.calibrated is True and r2.meta["gas_c"] == 60.0 and r2.meta["maneuver"] == ""
    on = tp_.run_thermic_patch(_res(n=12, dt=0.03, Fz=1.5), tp2, lp2)
    off = tp_.run_thermic_patch(_res(n=12, dt=0.03, Fz=0.5), tp2, lp2)
    assert not np.allclose(on.T_surface, off.T_surface)


def test_bad_time_steps_and_bad_inputs():
    res = _res(n=6)
    res.t = np.array([0.0, 0.01, 0.01, 0.005, np.nan, 0.03])
    r = tp_.run_thermic_patch(res)
    assert r.ok and np.all(r.T_surface[2] == r.T_surface[1]) and np.all(r.T_surface[3] == r.T_surface[2])
    assert np.all(r.T_core[4] == r.T_core[3])
    assert tp_.run_thermic_patch(None).warnings == [
        "ThermicPatch needs a solved transient run; got a failed/empty one."]
    assert tp_.run_thermic_patch(_res(ok=False)).ok is False
    assert tp_.run_thermic_patch(NS(t=[0.0, 0.1])).ok is False                      # no ok flag at all
    bad = _res()
    bad.Fy = np.zeros(3)
    w = tp_.run_thermic_patch(bad).warnings[0]
    assert w.startswith("ThermicPatch could not read the transient arrays (") and w.endswith(").")
    assert tp_.run_thermic_patch(_res(n=1)).warnings == ["Transient run too short to integrate heat."]
    blow = tp_.run_thermic_patch(_res(n=4, dt=0.01), ThermalParams(m_tread_kg=0.0, m_carcass_kg=0.0),
                                 init_temp_c=60.0)
    assert np.all(np.isin(np.round(blow.T_surface[1:], 9), [-40.0, 400.0, 60.0])) and blow.ok
    nanf = _res(n=4)
    nanf.Fy = nanf.Fy.copy()
    nanf.Fy[2, 0] = np.nan
    nr = tp_.run_thermic_patch(nanf, init_temp_c=61.0)
    assert nr.T_surface[2, 0] == 61.0 and nr.warnings == [
        "ThermicPatch clamped a non-finite node temperature; check the transient log's time base."]


def test_the_failed_result_and_the_readouts():
    f = tp_.ThermicResult.failed("why")
    assert (f.ok, f.T_opt_c, f.calibrated, f.warnings, f.t.shape, f.T_core.shape) == (False, 85.0, False, ["why"], (1,),
                                                                                   (1, 4))
    assert np.all(f.mu_scale == 1.0) and np.all(f.dFy_frac == 0.0)
    assert f.worst_instant() == {"ok": False} and f.summary() == {"ok": False, "warnings": ["why"]}
    r = tp_.run_thermic_patch(_res(n=30, slide=0.3, u=25.0), init_temp_c=105.0)
    lost = r.Fy_cold * r.dFy_frac
    i, ci = np.unravel_index(int(np.nanargmax(lost)), lost.shape)
    w = r.worst_instant()
    assert w == {"ok": True, "t_s": r.t[i], "corner": ("FL", "FR", "RL", "RR")[ci], "T_surface_c": r.T_surface[i, ci],
                 "T_core_c": r.T_core[i, ci], "mu_scale": r.mu_scale[i, ci], "dFy_frac": r.dFy_frac[i, ci],
                 "Fy_lost_N": pytest.approx(r.Fy_cold[i, ci] - r.Fy_hot[i, ci])}
    assert r.peak_core_c() == np.nanmax(r.T_core) and r.peak_surface_c() == np.nanmax(r.T_surface)
    assert r.summary() == {"ok": True, "peak_surface_c": r.peak_surface_c(), "peak_core_c": r.peak_core_c(),
                           "min_mu_scale": float(np.nanmin(r.mu_scale)), "max_dFy_frac": float(np.nanmax(r.dFy_frac)),
                           "calibrated": False, "worst": w}
    empty = tp_.ThermicResult.failed("x")
    empty.ok = True
    empty.dFy_frac = np.zeros((0, 4))
    assert empty.worst_instant() == {"ok": True}
    empty.T_core = np.zeros((0, 4)); empty.T_surface = np.zeros((0, 4))
    assert math.isnan(empty.peak_core_c()) and math.isnan(empty.peak_surface_c())
    broken = tp_.ThermicResult.failed("x")
    broken.ok, broken.dFy_frac, broken.Fy_cold = True, np.full((1, 4), np.nan), np.full((1, 4), np.nan)
    assert broken.worst_instant() == {"ok": True}
    s = broken.summary()
    assert s["ok"] is True and s["min_mu_scale"] == 1.0 and s["worst"] == {"ok": True}
    broken.T_core = None
    assert broken.summary() == {"ok": True}


def test_the_verdict_sentence():
    assert tp_.verdict_sentence(tp_.ThermicResult.failed("a")) == "ThermicPatch: no valid thermal history (a)."
    cool = tp_.run_thermic_patch(_res(n=10, slide=0.01), init_temp_c=85.0)
    assert tp_.verdict_sentence(cool) == ("No thermal grip window breach: the core tread stayed inside the compound "
                                          f"window (peak core {cool.peak_core_c():.0f}°C) through the event.")
    hot = tp_.run_thermic_patch(_res(n=30, slide=0.3, u=25.0), init_temp_c=105.0)
    w = hot.worst_instant()
    assert tp_.verdict_sentence(hot) == (
        f"At t={w['t_s']*1e3:.0f}ms, flash surface heat reaches {w['T_surface_c']:.0f}°C and the {w['corner']} core "
        f"tread hits {w['T_core_c']:.0f}°C — outside the grip window centred on {hot.T_opt_c:.0f}°C — dropping "
        f"available lateral force by {w['dFy_frac']*100:.0f}% ({w['Fy_lost_N']:.0f} N) (UNCALIBRATED — representative "
        f"parameters).")
    cal = tp_.run_thermic_patch(_res(n=30, slide=0.3, u=25.0), ThermalParams(calibrated=True), init_temp_c=105.0)
    assert tp_.verdict_sentence(cal).endswith(" N).")
    edge = tp_.ThermicResult.failed("x")
    edge.ok, edge.dFy_frac, edge.Fy_cold = True, np.full((1, 4), 1e-3), np.full((1, 4), 100.0)
    edge.Fy_hot = edge.Fy_cold * (1 - 1e-3)
    assert tp_.verdict_sentence(edge).startswith("At t=")
    edge.dFy_frac = np.full((1, 4), 0.9e-3)
    assert tp_.verdict_sentence(edge).startswith("No thermal grip window breach")


def test_a_log_finer_than_the_substep_shapes_and_varying_drive():
    lp = tp_.LadderParams(max_substep_s=0.01)
    res = _res(n=10, dt=0.001)
    res.u = np.linspace(12.0, 20.0, 10)
    res.Fz = res.Fz * np.linspace(0.8, 1.2, 10)[:, None]
    r = tp_.run_thermic_patch(res, lp=lp)
    S, C, K, _ = _ladder(res, ThermalParams(), lp)
    assert r.T_surface == pytest.approx(S, rel=1e-12) and r.T_carcass == pytest.approx(K, rel=1e-12)
    col = _res(n=10, dt=0.001)
    col.u = res.u.reshape(-1, 1); col.Fz = res.Fz; col.t = col.t.reshape(-1, 1)
    assert tp_.run_thermic_patch(col, lp=lp).T_core == pytest.approx(r.T_core)
    two = tp_.run_thermic_patch(_res(n=2))
    assert two.ok and two.T_surface.shape == (2, 4)
    flat = _res(n=3, slide=0.0)
    assert np.all(tp_.run_thermic_patch(flat).q_flash == 0.0)


def test_a_skipped_step_holds_every_node():
    res = _res(n=5)
    res.t = np.array([0.0, 0.01, 0.02, 0.02, 0.03])
    r = tp_.run_thermic_patch(res, gas_c=20.0)
    assert np.all(r.T_carcass[3] == r.T_carcass[2]) and np.any(r.T_carcass[2] != r.T_carcass[1])


def test_massless_nodes_clamp_every_layer():
    p = ThermalParams(m_tread_kg=0.0, m_carcass_kg=0.0)
    one = tp_.LadderParams(max_substep_s=1.0)                                   # a single explicit step
    hot = tp_.run_thermic_patch(_res(n=2, slide=0.3, u=30.0), p, one, init_temp_c=60.0, gas_c=500.0)
    assert hot.T_carcass[1].tolist() == [400.0] * 4
    core = tp_.run_thermic_patch(_res(n=3, slide=0.3, u=30.0), ThermalParams(m_tread_kg=0.0), one, init_temp_c=60.0)
    assert core.T_surface[1].tolist() == [400.0] * 4 and core.T_core[2].tolist() == [400.0] * 4
    cold = tp_.run_thermic_patch(_res(n=3, slide=0.0, u=0.0, Fz=0.0), p, one, init_temp_c=-39.0, ambient_c=-100.0,
                                 gas_c=-1000.0)
    assert cold.T_carcass[1].tolist() == [-40.0] * 4 and cold.T_surface[1].tolist() == [-40.0] * 4
    assert cold.T_core[2].tolist() == [-40.0] * 4


def test_the_rejection_message_without_an_ok_flag():
    assert tp_.run_thermic_patch(NS(t=[0.0, 0.1])).warnings == [
        "ThermicPatch needs a solved transient run; got a failed/empty one."]


def _made(t, Fy, dfrac, calibrated=False):
    n = len(t)
    Fy = np.asarray(Fy, float)
    d = np.asarray(dfrac, float)
    z = np.zeros((n, 4))
    return tp_.ThermicResult(ok=True, t=np.asarray(t, float), T_surface=z + 120.0, T_core=z + 110.0, T_carcass=z,
                             q_flash=z, mu_scale=1 - d, Fy_cold=Fy, Fy_hot=Fy * (1 - d), dFy_frac=d, T_opt_c=85.0,
                             calibrated=calibrated)


def test_the_worst_instant_is_ranked_in_newtons_at_every_corner():
    for ci, name in enumerate(("FL", "FR", "RL", "RR")):
        Fy = np.full((2, 4), 100.0); Fy[1, ci] = 1500.0
        d = np.full((2, 4), 0.4); d[1, ci] = 0.3                          # a bigger fraction elsewhere, fewer newtons
        w = _made([0.0, 0.5], Fy, d).worst_instant()
        assert (w["corner"], w["t_s"], w["Fy_lost_N"]) == (name, 0.5, pytest.approx(450.0))
    r = _made([0.0, 0.5], np.full((2, 4), 1000.0), np.full((2, 4), 0.3) * [[0, 0, 0, 0], [1, 0, 0, 0]])
    assert tp_.verdict_sentence(r) == ("At t=500ms, flash surface heat reaches 120°C and the FL core tread hits 110°C — "
                                       "outside the grip window centred on 85°C — dropping available lateral force "
                                       "by 30% (300 N) (UNCALIBRATED — representative parameters).")
    assert tp_.verdict_sentence(_made([0.0, 0.5], np.full((2, 4), 1000.0), np.full((2, 4), 0.3),
                                      calibrated=True)).endswith("by 30% (300 N).")
    assert _made([0.0, 1.0], np.full((2, 4), 1.0), np.zeros((2, 4)), calibrated=True).summary()["calibrated"] is True


def test_the_failed_result_and_its_sentence():
    f = tp_.ThermicResult.failed("a")
    for arr in (f.T_carcass, f.q_flash, f.Fy_cold, f.Fy_hot, f.T_surface, f.T_core, f.dFy_frac):
        assert arr.shape == (1, 4) and np.all(arr == 0.0)
    assert f.mu_scale.shape == (1, 4)
    f.warnings.append("b")
    assert tp_.verdict_sentence(f) == "ThermicPatch: no valid thermal history (a; b)."
    broken = _made([0.0, 1.0], np.full((2, 4), np.nan), np.full((2, 4), np.nan))
    assert broken.worst_instant() == {"ok": True}
    assert tp_.verdict_sentence(broken).startswith("No thermal grip window breach")
