"""suspension/pack_thermal.py against a forward-Euler network written here: I^2 R per cell from the pack current
split over the parallel cells, 4-neighbour conduction, convection to a fan-shaped h field with the wake offset, the
stability sub-stepping, the warn/limit crossing times, the current trace rebuilt from a lap, and the fan study's
ranking and summary."""
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

from suspension import pack_thermal as pt

PROXY = pt.AirflowParams(mode="proxy")      # the recorded characterization predates the duct default


def test_the_cell_and_the_layout():
    c = pt.CellParams()
    assert (c.mass_kg, c.cp, c.r_internal_ohm, c.r_tempco_per_c, c.nominal_v, c.surface_area_m2, c.contact_area_m2,
            c.pitch_mm, c.temp_limit_c, c.temp_warn_c, c.enable_entropic, c.dudt_v_per_k, c.calibrated,
            c.fitted_to) == (0.070, 1000.0, 0.022, -0.0010, 3.6, 0.0042, 8.0e-5, 22.0, 60.0, 50.0, False, 0.0,
                             False, "")
    assert c.r_at(45.0) == pytest.approx(0.022 * (1 - 0.02)) and c.r_at(25.0) == pytest.approx(0.022)
    assert pt.CellParams(r_tempco_per_c=-0.1).r_at(100.0) == 1e-4
    assert pt.default_cell_params() == pt.CellParams()
    L = pt.PackLayout()
    assert (L.rows, L.cols, L.series, L.parallel, L.ambient_c, L.ambient_is_local, L.ENV_CHANNEL) == (
        6, 14, 84, 6, 30.0, False, "cooling_inlet")
    assert L.n_cells == 84 and L.pack_nominal_v == pytest.approx(84 * 3.6) and L.extent_mm() == (14 * 22.0, 6 * 22.0)
    z = pt.PackLayout(rows=0, cols=-2, series=0, parallel=0)
    assert (z.rows, z.cols, z.series, z.parallel) == (1, 1, 1, 1)
    f = pt.PackLayout(rows=2.7, cols=3.2, series=5.9, parallel=2.5)
    assert (f.rows, f.cols, f.series, f.parallel) == (2, 3, 5, 2)
    p = pt.PackLayout(rows=2, cols=3, cell=pt.CellParams(pitch_mm=10.0)).positions_mm()
    assert p.tolist() == [[5, 5], [15, 5], [25, 5], [5, 15], [15, 15], [25, 15]]


def test_the_airflow_field():
    a = pt.AirflowParams(mode="proxy")
    assert (a.h_floor, a.h_peak, a.ref_cfm, a.enable_wake_warming, a.wake_warm_per_cell_c, a.calibrated) == (
        5.5, 75.0, 120.0, True, 0.15, False)
    pos = np.array([[0.0, 0.0], [30.0, 40.0], [100.0, 0.0]])
    fans = [pt.Fan(0.0, 0.0, cfm=60.0, throw_mm=50.0), pt.Fan(100.0, 0.0)]
    want = np.full(3, 5.5)
    for f in fans:
        d2 = (pos[:, 0] - f.x_mm) ** 2 + (pos[:, 1] - f.y_mm) ** 2
        want += 69.5 * f.cfm / 120.0 * np.exp(-d2 / f.throw_mm ** 2)
    assert a.h_field(pos, fans) == pytest.approx(want, rel=1e-12)
    assert pt.Fan(1, 2).cfm == 120.0 and pt.Fan(1, 2).throw_mm == 90.0 and pt.Fan(1, 2).label == ""
    assert a.h_field(pos, []).tolist() == [5.5] * 3 and a.h_field(pos, None).tolist() == [5.5] * 3
    neg = a.h_field(pos, [pt.Fan(0, 0, cfm=-50.0)])
    assert neg.tolist() == [5.5] * 3
    tiny = a.h_field(pos, [pt.Fan(0, 0, throw_mm=0.0)])
    assert tiny[0] == pytest.approx(75.0) and tiny[1] == pytest.approx(5.5)
    bad = a.h_field(pos, [pt.Fan("x", 0), pt.Fan(100.0, 0.0)])                # a broken fan is skipped
    assert bad == pytest.approx(want - 69.5 * 0.5 * np.exp(-(pos[:, 0] ** 2 + pos[:, 1] ** 2) / 2500.0), rel=1e-12)
    assert pt.AirflowParams(mode="proxy", ref_cfm=0.0).h_field(pos[:1], [pt.Fan(0, 0, cfm=1e-6)])[0] == pytest.approx(75.0)
    sunk = pt.AirflowParams(mode="proxy", h_peak=1.0).h_field(pos, [pt.Fan(0, 0)])
    assert sunk.tolist() == [5.5] * 3                                          # clipped at the floor
    assert pt.AirflowParams(mode="proxy").h_field(np.zeros((4, 1)), []).tolist() == [5.5] * 4


def _lap(d, v, lg):
    return NS(distance=np.array(d, float), speed=np.array(v, float), long_g=np.array(lg, float))


P = NS(g=9.81, mass=300.0, rho=1.2, cd_a=1.3, rolling_g=0.02, V_MIN=0.5)


def _current(lap, p, vpack, eff=0.9, reff=0.55, rmax=0.35, regen=True):
    n = len(lap.speed)
    t, cur = np.zeros(n), np.zeros(n)
    for i in range(1, n):
        ds = lap.distance[i] - lap.distance[i - 1]
        if not (np.isfinite(ds) and ds > 0):
            t[i] = t[i - 1]
            continue
        vi = max(lap.speed[i], p.V_MIN)
        t[i] = t[i - 1] + ds / max(0.5 * (lap.speed[i] + lap.speed[i - 1]), p.V_MIN)
        a = (lap.long_g[i] if np.isfinite(lap.long_g[i]) else 0.0) * p.g
        fd, fr = 0.5 * p.rho * p.cd_a * vi * vi, p.rolling_g * p.mass * p.g
        F = p.mass * a + fd + fr
        if F > 0:
            cur[i] = F * vi / eff / vpack
        elif regen:
            cur[i] = -min(max(-p.mass * a - fd - fr, 0.0), rmax * p.mass * p.g) * vi * reff / vpack
    return t, cur


LAP = _lap([0, 10, 25, 25, 40, 60, 80], [10, 14, 18, 18, 0.2, 9, 12], [0.5, 0.6, 0.3, 0.1, -1.6, np.nan, -0.05])


def test_the_current_trace():
    t, c = pt.pack_current_trace(LAP, P, 300.0, inverter_motor_eff=0.85, regen_eff=0.6, regen_max_g=0.3)
    wt, wc = _current(LAP, P, 300.0, 0.85, 0.6, 0.3)
    assert t == pytest.approx(wt, rel=1e-12) and c == pytest.approx(wc, rel=1e-12, abs=1e-12)
    assert c.min() < 0 and c.max() > 0
    off = pt.pack_current_trace(LAP, P, 300.0, regen_enabled=False)[1]
    assert off == pytest.approx(_current(LAP, P, 300.0, regen=False)[1], rel=1e-12, abs=1e-12) and off.min() == 0.0
    d = pt.pack_current_trace(LAP, NS(), 0.5)
    q = NS(g=9.81, mass=280.0, rho=1.225, cd_a=1.2, rolling_g=0.015, V_MIN=0.5)
    assert d[1] == pytest.approx(_current(LAP, q, 1.0)[1], rel=1e-12, abs=1e-12)          # pack floored at 1 V
    z = pt.pack_current_trace(LAP, P, 300.0, inverter_motor_eff=0.0)[1]
    assert z == pytest.approx(_current(LAP, P, 300.0, eff=1e-3)[1], rel=1e-12, abs=1e-12)
    msgs = []
    assert [x.tolist() for x in pt.pack_current_trace(_lap([0], [5], [0]), P, 300.0, warn=msgs.append)] == [[0.0], [0.0]]
    assert msgs == ["Lap trace too short to build a current history."]
    msgs = []
    out = pt.pack_current_trace(NS(speed=[1, 2], distance=None, long_g=[0, 0]), P, 300.0, warn=msgs.append)
    assert [x.tolist() for x in out] == [[0.0], [0.0]] and msgs == ["Current-trace construction failed; using zero current."]
    big = _lap([0, 1e300], [1e-300, 1e300], [0.0, 1e300])                       # overflow is zeroed, not kept
    assert np.all(np.isfinite(pt.pack_current_trace(big, P, 300.0)[1]))
    assert pt.pack_current_trace(LAP, P, 300.0) is not None


def _oracle(layout, fans, air, k, t, cur, T0=None, laps=1):
    cell = layout.cell
    n, R, C = layout.n_cells, layout.rows, layout.cols
    if laps > 1:
        ts, cs = [t], [cur]
        for _ in range(1, laps):
            ts.append(t + ts[-1][-1] + (t[1] - t[0]))
            cs.append(cur)
        t, cur = np.concatenate(ts), np.concatenate(cs)
    pos = layout.positions_mm()
    h = air.h_field(pos, fans)
    if air.enable_wake_warming and h.max() > air.h_floor + 1e-9:
        off = np.clip((h.max() - h) / h.max(), 0, 1) * max(air.wake_warm_per_cell_c, 0.0) * 6.0
    else:
        off = np.zeros(n)
    Cc, A = cell.mass_kg * cell.cp, cell.surface_area_m2
    neigh = [(r * C + c, r * C + c + 1) for r in range(R) for c in range(C - 1)] + \
            [(r * C + c, r * C + c + C) for r in range(R - 1) for c in range(C)]
    T0 = layout.ambient_c if T0 is None else T0
    T = np.full(n, T0)
    Tair = layout.ambient_c + off
    dts = 0.5 * Cc / (h.max() * A + 4 * k)
    hist, peak = [T.copy()], T.copy()
    tw, tl = np.full(n, np.nan), np.full(n, np.nan)
    for s in range(1, len(t)):
        dtm = max(t[s] - t[s - 1], 0.0)
        Ic = cur[s] / layout.parallel
        if dtm > 0:
            ns = min(max(math.ceil(dtm / dts), 1), 5000)
            dt = dtm / ns
            for _ in range(ns):
                q = Ic * Ic * cell.r_at(float(np.mean(T))) * np.ones(n)
                if cell.enable_entropic and cell.dudt_v_per_k != 0:
                    q = q + Ic * (T + 273.15) * cell.dudt_v_per_k
                qc = np.zeros(n)
                for a, b in neigh:
                    qc[a] += k * (T[b] - T[a])
                    qc[b] -= k * (T[b] - T[a])
                T = T + (q + h * A * (Tair - T) + qc) * dt / Cc
        hist.append(T.copy())
        peak = np.maximum(peak, T)
        tw[(T >= cell.temp_warn_c) & np.isnan(tw)] = t[s]
        tl[(T >= cell.temp_limit_c) & np.isnan(tl)] = t[s]
    return t, cur, np.array(hist), peak, tw, tl


SMALL = pt.PackLayout(rows=2, cols=3, series=10, parallel=2, cell=pt.CellParams(mass_kg=0.02, temp_warn_c=34.0,
                                                                                temp_limit_c=36.0))
FANS = [pt.Fan(10.0, 10.0, cfm=150.0, throw_mm=30.0)]


def test_the_network_against_euler_written_here():
    t = np.array([0.0, 2.0, 2.0, 5.0, 9.0, 14.0])
    cur = np.array([0.0, 80.0, 90.0, 120.0, -40.0, 100.0])
    air = pt.AirflowParams(wake_warm_per_cell_c=0.4)
    m = pt.PackThermalModel(SMALL, FANS, air, k_cell_cell=0.5)
    r = m.simulate(t, cur, init_temp_c=31.0)
    tt, cc, hist, peak, tw, tl = _oracle(SMALL, FANS, air, 0.5, t, cur, 31.0)
    assert r.ok and r.rows == 2 and r.cols == 3 and r.time_s.tolist() == tt.tolist()
    assert r.pack_current_a.tolist() == cc.tolist()
    assert r.temp_history_c == pytest.approx(hist, rel=1e-12) and r.final_temp_c == pytest.approx(hist[-1], rel=1e-12)
    assert r.peak_temp_c == pytest.approx(peak, rel=1e-12)
    assert np.array_equal(np.isnan(r.time_to_warn_s), np.isnan(tw)) and np.nanmax(np.abs(r.time_to_warn_s - tw)) == 0
    assert np.array_equal(np.isnan(r.time_to_limit_s), np.isnan(tl))
    k = int(np.argmax(peak))
    assert (r.hottest_cell_index, r.hottest_cell_rc, r.hottest_peak_c) == (k, (k // 3, k % 3), pytest.approx(peak[k]))
    assert r.breach_count == int((peak >= 36.0).sum()) and r.any_cell_breached_limit is bool((peak >= 36.0).any())
    assert r.synthesized is True and r.provenance.startswith("SYNTHESIZED: per-cell temperatures")
    assert r.temp_grid_c().shape == (2, 3) and r.peak_grid_c()[k // 3, k % 3] == pytest.approx(peak[k])
    assert 0 < np.isnan(tw).sum() < 6 or np.isnan(tw).sum() == 0


def test_repeated_laps_defaults_and_entropic_heat():
    t = np.array([1.0, 3.0, 6.0])
    cur = np.array([0.0, 150.0, 60.0])
    cell = pt.CellParams(mass_kg=0.02, enable_entropic=True, dudt_v_per_k=2e-4)
    lay = pt.PackLayout(rows=2, cols=2, parallel=1, cell=cell)
    m = pt.PackThermalModel(lay, FANS, pt.AirflowParams(enable_wake_warming=False))
    r = m.simulate(t, cur, n_laps=3)
    tt, cc, hist, peak, tw, tl = _oracle(lay, FANS, pt.AirflowParams(enable_wake_warming=False), 0.35, t, cur, None, 3)
    assert r.time_s.tolist() == tt.tolist() == [1, 3, 6, 9, 11, 14, 17, 19, 22]
    assert r.temp_history_c == pytest.approx(hist, rel=1e-12) and r.temp_history_c[0, 0] == 30.0
    assert pt.PackThermalModel().simulate(t, cur).final_temp_c.shape == (84,)
    d = pt.PackThermalModel()
    assert (d.layout, d.fans, d.airflow, d.k_cc, d.warnings) == (pt.PackLayout(), [], pt.AirflowParams(), 0.35, [])
    zero_entropy = pt.PackThermalModel(pt.PackLayout(rows=1, cols=2, cell=pt.CellParams(enable_entropic=True)))
    base = pt.PackThermalModel(pt.PackLayout(rows=1, cols=2))
    assert zero_entropy.simulate(t, cur).final_temp_c == pytest.approx(base.simulate(t, cur).final_temp_c)


def test_calibration_and_the_provenance_lines():
    cal = pt.PackLayout(rows=1, cols=2, cell=pt.CellParams(calibrated=True))
    r = pt.PackThermalModel(cal, airflow=pt.AirflowParams(calibrated=True)).simulate([0, 1], [0, 10])
    assert r.synthesized is False and r.provenance == (
        "CALIBRATED: cell thermal/electrical parameters and the airflow map are fitted to measured data; absolute "
        "temperatures are quotable within the fit's validity.")
    half = pt.PackThermalModel(cal).simulate([0, 1], [0, 10])
    assert half.synthesized is True and half.provenance == (
        "SYNTHESIZED: per-cell temperatures are physically-shaped estimates from an uncalibrated lumped network + "
        "geometric airflow proxy. Trustworthy for RANKING layouts and locating the hot corner; absolute °C needs "
        "calibrated=True on both the cell and the airflow map.")
    other = pt.PackThermalModel(pt.PackLayout(rows=1, cols=2), airflow=pt.AirflowParams(calibrated=True))
    assert other.simulate([0, 1], [0, 10]).synthesized is True


def test_inputs_that_cannot_run_and_the_stability_cap():
    m = pt.PackThermalModel(pt.PackLayout(rows=2, cols=2))
    m.warnings = ["earlier"]
    f = m.simulate([0.0], [1.0])
    assert f.ok is False and f.warnings == ["earlier", "Current history too short to integrate."]
    assert f.final_temp_c.shape == (4,) and np.isnan(f.final_temp_c).all() and f.hottest_cell_rc == (-1, -1)
    assert (f.hottest_cell_index, f.breach_count, f.any_cell_breached_limit, f.synthesized) == (-1, 0, False, True)
    assert f.provenance == "run failed; no thermal field produced" and f.temp_history_c.shape == (1, 4)
    assert f.temp_grid_c().shape == (2, 2) and f.peak_grid_c().shape == (2, 2)
    assert pt.PackThermalResult.failed(0, 0, []).final_temp_c.shape == (1,)
    odd = pt.PackThermalResult.failed(0, 3, [])
    assert odd.temp_grid_c().shape == (1,) and odd.peak_grid_c().shape == (1,)
    assert m.simulate([0.0, 1.0], [5.0]).ok is False
    trimmed = pt.PackThermalModel(pt.PackLayout(rows=1, cols=1)).simulate([0, 1, 2, 3], [0, 5, np.nan])
    assert trimmed.ok and trimmed.time_s.tolist() == [0, 1, 2] and trimmed.pack_current_a.tolist() == [0, 5, 0]
    nant = pt.PackThermalModel(pt.PackLayout(rows=1, cols=1)).simulate([0, np.nan, 3], [0, 5, np.inf])
    assert nant.time_s.tolist() == [0, 0, 3] and nant.pack_current_a.tolist() == [0, 5, 0]
    crash = pt.PackThermalModel(pt.PackLayout(rows=1, cols=2), k_cell_cell=0.35)
    crash.layout.cell = None
    out = crash.simulate([0, 1], [0, 1])
    assert out.ok is False and out.warnings[-1].startswith("pack thermal run crashed: ")
    hot = pt.PackThermalModel(pt.PackLayout(rows=1, cols=1, cell=pt.CellParams(mass_kg=1e-6)))
    r = hot.simulate([0.0, 1000.0], [0.0, 0.0])
    assert hot._stability_violated is True and r.warnings == [
        "UNSTABLE: this step needs more than 5000 sub-steps to satisfy the explicit-Euler stability limit, and has been "
        "capped there. The integrator is now running past its stability bound, so the temperature field will oscillate "
        "and diverge — these are not merely coarse results, they are wrong. Shorten the macro step, coarsen the cell "
        "grid, or use a smaller cell heat capacity."]
    ok = pt.PackThermalModel(pt.PackLayout(rows=1, cols=1))
    ok.simulate([0.0, 1.0], [0.0, 0.0])
    assert ok._stability_violated is False and ok.warnings == []
    m._warn(""), m._warn("x"), m._warn("x")
    assert m.warnings == ["earlier", "x"]


def test_the_wake_offset_and_the_neighbours():
    m = pt.PackThermalModel(pt.PackLayout(rows=2, cols=3), airflow=pt.AirflowParams(wake_warm_per_cell_c=0.5))
    h = np.array([8.0, 20.0, 40.0, 10.0, 30.0, 8.0])
    assert m._wake_air_offset(h) == pytest.approx(np.clip((40.0 - h) / 40.0, 0, 1) * 3.0)
    assert m._wake_air_offset(np.full(6, 8.0)).tolist() == [0.0] * 6
    assert m._wake_air_offset(np.full(6, 8.0 + 1e-10)).tolist() == [0.0] * 6
    assert m._wake_air_offset(np.array([])).tolist() == []
    neg = pt.PackThermalModel(airflow=pt.AirflowParams(wake_warm_per_cell_c=-1.0))
    assert neg._wake_air_offset(h).tolist() == [0.0] * 6
    assert m._wake_air_offset("junk").tolist() == [0.0] * 6
    assert m._neighbours() == [(0, 1), (0, 3), (1, 2), (1, 4), (2, 5), (3, 4), (4, 5)]
    assert pt.PackThermalModel(pt.PackLayout(rows=1, cols=1))._neighbours() == []


def test_the_lap_driver(monkeypatch):
    seen = {}
    orig = pt.pack_current_trace

    def spy(lap, params, pack_nominal_v, **kw):
        seen.update(kw, v=pack_nominal_v)
        return orig(lap, params, pack_nominal_v, **kw)
    monkeypatch.setattr(pt, "pack_current_trace", spy)
    lay = pt.PackLayout(rows=2, cols=2, series=20, parallel=2)
    ev = NS(inverter_motor_eff=0.8, regen_eff=0.5, regen_max_g=0.2, regen_enabled=False)
    r = pt.simulate_pack_thermal(LAP, P, layout=lay, fans=FANS, airflow=pt.AirflowParams(), ev=ev, k_cell_cell=0.4,
                                 init_temp_c=33.0, n_laps=2)
    assert seen["v"] == pytest.approx(72.0) and (seen["inverter_motor_eff"], seen["regen_eff"], seen["regen_max_g"],
                                                seen["regen_enabled"]) == (0.8, 0.5, 0.2, False)
    t, c = _current(LAP, P, 72.0, 0.8, 0.5, 0.2, False)
    want = pt.PackThermalModel(lay, FANS, pt.AirflowParams(), 0.4).simulate(t, c, init_temp_c=33.0, n_laps=2)
    assert r.temp_history_c == pytest.approx(want.temp_history_c, rel=1e-12)
    pt.simulate_pack_thermal(LAP, P)
    assert (seen["inverter_motor_eff"], seen["regen_eff"], seen["regen_max_g"], seen["regen_enabled"]) == (
        0.90, 0.55, 0.35, True)
    pt.simulate_pack_thermal(LAP, P, ev=NS())
    assert (seen["inverter_motor_eff"], seen["regen_eff"], seen["regen_max_g"], seen["regen_enabled"]) == (
        0.90, 0.55, 0.35, True)
    lap3 = _lap(LAP.distance, LAP.speed, LAP.long_g)
    lap3.meta = {"laps": 3}
    n3 = pt.simulate_pack_thermal(lap3, P, layout=pt.PackLayout(rows=1, cols=1))
    assert n3.time_s.size == 3 * 7
    lap0 = _lap(LAP.distance, LAP.speed, LAP.long_g)
    lap0.meta = {"laps": 0}
    assert pt.simulate_pack_thermal(lap0, P, layout=pt.PackLayout(rows=1, cols=1)).time_s.size == 7
    nometa = _lap(LAP.distance, LAP.speed, LAP.long_g)
    nometa.meta = {}
    assert pt.simulate_pack_thermal(nometa, P, layout=pt.PackLayout(rows=1, cols=1)).time_s.size == 7
    short = pt.simulate_pack_thermal(_lap([0], [5], [0]), P)
    assert short.ok is False and short.warnings[0] == "Lap trace too short to build a current history."


def test_the_fan_study(monkeypatch):
    lay = pt.PackLayout(rows=1, cols=2)
    peaks = {(): (40.0, 0), (1.0,): (35.0, 0), (2.0,): (math.nan, 0), (3.0,): (30.0, 2)}
    calls = []

    def fake(lap, params, layout=None, fans=None, airflow=None, ev=None, k_cell_cell=None, n_laps=None):
        calls.append((layout, airflow, ev, k_cell_cell, n_laps))
        pk, br = peaks[tuple(f.x_mm for f in fans)]
        return NS(hottest_peak_c=pk, hottest_cell_rc=(0, 1), breach_count=br, synthesized=False, provenance="P",
                  warnings=["w"] if fans else ["w", "v"])
    monkeypatch.setattr(pt, "simulate_pack_thermal", fake)
    sets = [[], [pt.Fan(1.0, 0.0)], [pt.Fan(2.0, 0.0)], [pt.Fan(3.0, 0.0)]]
    s = pt.optimize_fan_placement("lap", "params", sets, layout=lay, ev="ev", k_cell_cell=0.2, n_laps=4)
    assert [c.hottest_peak_c for c in s.candidates] == [35.0, 40.0, math.inf, 30.0]
    assert s.best is s.candidates[0] and s.synthesized is False and s.provenance == "P" and s.warnings == ["w", "v"]
    assert all(c[0] is lay and c[2:] == ("ev", 0.2, 4) for c in calls)
    assert s.candidates[0].fans == sets[1] and s.candidates[0].fans is not sets[1]
    assert s.summary().split("\n") == [
        "Fan-placement study (lower peak cell temp is better):",
        "  1. peak   35.0°C  hot cell r0c1  breaches=0  fans=[(1,0)mm]",
        "  2. peak   40.0°C  hot cell r0c1  breaches=0  fans=[no fan]",
        "  3. peak    inf°C  hot cell r0c1  breaches=0  fans=[(2,0)mm]",
        "  4. peak   30.0°C  hot cell r0c1  breaches=2  fans=[(3,0)mm]"]
    calls.clear()
    d = pt.optimize_fan_placement("lap", "params", None)
    assert len(d.candidates) == 1 and calls[0][0] == pt.PackLayout() and calls[0][1] == pt.AirflowParams(mode="proxy")
    peaks[()] = (40.0, 0)
    syn = pt.FanPlacementStudy(candidates=d.candidates, synthesized=True, provenance="")
    assert syn.summary().endswith("\n  NOTE: temperatures are SYNTHESIZED (uncalibrated); the RANKING is the "
                                  "deliverable, not absolute °C.")
    empty = pt.FanPlacementStudy([], True, "")
    assert empty.best is None and empty.summary() == "No fan-placement candidates evaluated." and empty.warnings == []
    two = pt.FanPlacementStudy([pt.FanPlacementCandidate([pt.Fan(1.4, 2.6), pt.Fan(10, 20)], 33.333, (1, 2), 0, None)],
                               False, "")
    assert two.summary().split("\n")[1] == "  1. peak   33.3°C  hot cell r1c2  breaches=0  fans=[(1,3)mm, (10,20)mm]"


def test_the_default_fan_grid():
    lay = pt.PackLayout(rows=2, cols=4, cell=pt.CellParams(pitch_mm=20.0))
    sets = pt.fan_grid_candidates(lay, nx=2, ny=1, cfm=90.0, throw_mm=60.0)
    assert sets[0] == [] and len(sets) == 3
    assert [(s[0].x_mm, s[0].y_mm, s[0].cfm, s[0].throw_mm, s[0].label) for s in sets[1:]] == [
        (20.0, 20.0, 90.0, 60.0, "fan@(20,20)"), (60.0, 20.0, 90.0, 60.0, "fan@(60,20)")]
    d = pt.fan_grid_candidates(lay)
    assert len(d) == 7 and (d[1][0].cfm, d[1][0].throw_mm) == (120.0, 90.0)
    assert (d[1][0].x_mm, d[1][0].y_mm) == pytest.approx((80 / 6, 10.0)) and d[-1][0].y_mm == pytest.approx(30.0)
    z = pt.fan_grid_candidates(lay, nx=0, ny=0)
    assert len(z) == 2 and (z[1][0].x_mm, z[1][0].y_mm) == (40.0, 20.0)


def _capped(lay, fans):
    m = pt.PackThermalModel(lay, fans, PROXY, k_cell_cell=500.0)
    h = m.airflow.h_field(lay.positions_mm(), fans)
    dt_stable = 0.5 * lay.cell.mass_kg * lay.cell.cp / (h.max() * lay.cell.surface_area_m2 + 2000.0)
    return m.simulate([0.0, 5001.5 * dt_stable], [0.0, 900.0], init_temp_c=50.0)


def test_characterized_runs():
    from tests._golden import check
    res = {}
    a = PROXY
    pos = np.array([[0.0, 0.0], [10.0, 30.0], [40.0, 10.0], [0.4, 0.0]])
    res["h"] = [a.h_field(pos, [pt.Fan(10.0, 25.0, throw_mm=20.0)]), a.h_field(pos, [pt.Fan(0.0, 0.0, throw_mm=0.5)]),
                a.h_field([["a", "b", "c"], ["d", "e", "f"]], [])]
    res["failed"] = pt.PackThermalResult.failed(2, 3, ["w"])
    d = np.arange(0.0, 14.0) * 0.5
    d[5] = d[4]
    v = np.array([1.0, 4.0, 6.0, 8.0, 9.0, 9.0, 8.5, 8.0, 7.0, 5.0, 4.0, np.nan, 3.0, np.inf])
    lg = np.array([0.0, 0.6, 0.4, 0.2, 0.0, 0.0, -0.03, -0.05, -0.2, -1.0, -0.6, 0.0, np.nan, 0.1])
    lap = NS(distance=d, speed=v, long_g=lg)
    odd = NS(g=9.7, mass=250.0, rho=1.1, cd_a=1.0, rolling_g=0.03, V_MIN=1.3)
    res["trace"] = [pt.pack_current_trace(lap, odd, 300.0), pt.pack_current_trace(lap, NS(), 300.0),
                    pt.pack_current_trace(lap, odd, 300.0, inverter_motor_eff=0.8, regen_eff=0.7, regen_max_g=0.1),
                    pt.pack_current_trace(lap, odd, 300.0, regen_enabled=False)]
    lay = pt.PackLayout(rows=2, cols=3, series=3, parallel=2, ambient_c=25.0)
    hot = pt.PackLayout(rows=2, cols=3, series=3, parallel=1, ambient_c=45.0,
                        cell=pt.CellParams(enable_entropic=True, dudt_v_per_k=0.0))
    ent = pt.PackLayout(rows=2, cols=3, series=3, parallel=1, cell=pt.CellParams(dudt_v_per_k=0.002))
    fans = [pt.Fan(10.0, 10.0, cfm=150.0, throw_mm=25.0)]
    t = np.array([0.0, 0.4, 0.9, 1.5, 3.0, 6.0])
    cur = np.array([0.0, 400.0, np.inf, -np.inf, 300.0, 250.0])

    def short(r):
        return {k: getattr(r, k) for k in ("ok", "time_s", "final_temp_c", "peak_temp_c", "time_to_warn_s",
                                           "time_to_limit_s", "pack_current_a", "hottest_cell_index",
                                           "hottest_cell_rc", "hottest_peak_c", "any_cell_breached_limit",
                                           "breach_count", "synthesized", "warnings")}
    runs = {"lay": pt.PackThermalModel(lay, fans, PROXY).simulate(t, cur),
            "hot2": pt.PackThermalModel(hot, fans, PROXY, k_cell_cell=0.1).simulate(t, cur * 3, n_laps=2),
            "hot3": pt.PackThermalModel(hot, fans, PROXY).simulate(t, cur * 3, init_temp_c=55.0, n_laps=3),
            "ent": pt.PackThermalModel(ent, fans, PROXY).simulate(t, cur),
            "cap": _capped(lay, fans)}
    res["runs"] = {k: short(r) for k, r in runs.items()}
    res["hist"] = runs["lay"].temp_history_c
    ev = NS(inverter_motor_eff=0.85, regen_eff=0.6, regen_max_g=0.2, regen_enabled=True)
    laps = {"bare": lap, "meta2": NS(distance=d, speed=v, long_g=lg, meta={"laps": 2}),
            "meta0": NS(distance=d, speed=v, long_g=lg, meta={"laps": 0}),
            "metax": NS(distance=d, speed=v, long_g=lg, meta={})}
    res["drv"] = {k: short(pt.simulate_pack_thermal(lp, odd, layout=hot, fans=fans, airflow=PROXY)) for k, lp in laps.items()}
    res["drv_ev"] = short(pt.simulate_pack_thermal(lap, odd, layout=hot, fans=fans, airflow=PROXY, ev=ev, init_temp_c=40.0,
                                                   n_laps=2))
    res["drv_dflt"] = short(pt.simulate_pack_thermal(lap, odd, layout=hot))
    st = pt.optimize_fan_placement(lap, odd, [[], fans, [pt.Fan(50.0, 20.0)]], layout=hot)
    res["study"] = [[c.fans, c.hottest_peak_c, c.hottest_cell_rc, c.breach_count, short(c.result)]
                    for c in st.candidates] + [st.synthesized, st.provenance, st.warnings]
    cal = pt.optimize_fan_placement(lap, odd, None, layout=pt.PackLayout(rows=1, cols=2, cell=pt.CellParams(
        calibrated=True)), airflow=pt.AirflowParams(mode="proxy", calibrated=True), ev=ev, k_cell_cell=0.9, n_laps=2)
    res["cal"] = [short(cal.best.result), cal.synthesized, cal.provenance]
    check("pack_thermal_runs", res)


# ------------------------------------------------------------- duct mode: the Zukauskas tube-bank correlation -- #
def _nu(c, m, re, pr, corr):
    return c * re ** m * pr ** 0.36 * corr


@pytest.mark.parametrize("re, rows, ratio, stag, c, m, corr", [
    (50.0, 1, 1.0, False, 0.80, 0.4, 0.6768),
    (50.0, 1, 1.0, True, 0.90, 0.4, 0.6273),
    (100.0, 3, 1.0, False, 0.51, 0.5, 0.8687),          # the 100 boundary belongs to the middle range
    (500.0, 2, 1.0, True, 0.51, 0.5, 0.7689),
    (1000.0, 20, 1.0, False, 0.27, 0.63, 1.0),          # the 1000 boundary belongs to the bank range
    (5000.0, 6, 1.0, False, 0.27, 0.63, 0.9465),
    (5000.0, 25, 1.5, True, 0.35 * 1.5 ** 0.2, 0.6, 1.0),
    (5000.0, 19, 2.0, True, 0.40, 0.6, 0.9986),         # st/sl = 2 takes the constant 0.40
    (5000.0, 10, 2.5, True, 0.40, 0.6, 0.9765),
    (2e5, 4, 1.0, False, 0.021, 0.84, 0.9054),          # the 2e5 boundary belongs to the high range
    (3e5, 5, 1.0, True, 0.022, 0.84, 0.9254),
])
def test_zukauskas_branches(re, rows, ratio, stag, c, m, corr):
    assert pt.zukauskas_nu(re, 0.7, rows, ratio, stag) == pytest.approx(_nu(c, m, re, 0.7, corr), rel=1e-12)


def test_zukauskas_defaults_are_inline_square():
    assert pt.zukauskas_nu(5000.0, 0.7, 4) == pytest.approx(_nu(0.27, 0.63, 5000.0, 0.7, 0.9054), rel=1e-12)


def test_airflow_duct_defaults():
    a = pt.AirflowParams()
    assert (a.mode, a.duct_cfm, a.cell_diameter_mm, a.cell_length_mm, a.staggered) == ("duct", 0.0, 21.0, 70.0, False)
    assert (a.air_k_w_mk, a.air_nu_m2_s, a.air_pr, a.h_scale, a.h_floor) == (0.02662, 1.6046e-5, 0.7067, 1.0, 5.5)


def test_h_scale_multiplies_every_mode():
    pos = _grid(27.0, 27.0, 4, 3)
    fans = [pt.Fan(x_mm=0.0, y_mm=40.0, cfm=60.0)]
    for mode in ("proxy", "duct"):
        base = pt.AirflowParams(mode=mode).h_field(pos, fans)
        assert np.allclose(pt.AirflowParams(mode=mode, h_scale=1.2).h_field(pos, fans), 1.2 * base, rtol=1e-12)


def test_fan_draught_warning_only_with_fans_outside_duct_mode():
    lay = pt.PackLayout(rows=2, cols=3)
    t, cur = np.arange(0.0, 20.0), np.full(20, 50.0)
    fans = [pt.Fan(x_mm=0.0, y_mm=0.0, cfm=60.0)]
    msg = ("Fan-draught airflow map: a geometric estimate, not benchmarked. Against three tube-bank correlations "
           "for air pushed through a pack it was off by up to 95 % and cooled the wrong end; for a ducted pack "
           "use the duct mode.")
    assert msg in pt.PackThermalModel(layout=lay, fans=fans, airflow=PROXY).simulate(t, cur).warnings
    assert msg not in pt.PackThermalModel(layout=lay, airflow=PROXY).simulate(t, cur).warnings
    assert msg not in pt.PackThermalModel(layout=lay, fans=fans).simulate(t, cur).warnings      # default: duct
    assert msg not in pt.PackThermalModel(layout=lay, fans=fans,
                                          airflow=pt.AirflowParams(mode="duct")).simulate(t, cur).warnings


def _grid(sx_mm, sy_mm, nx, ny):
    gx, gy = np.meshgrid((np.arange(nx) + 0.5) * sx_mm, (np.arange(ny) + 0.5) * sy_mm)
    return np.column_stack([gx.ravel(), gy.ravel()])


def _expected_rows(a, sx, sy, nx, ny, cfm, stag=False):
    d, s_l, s_t = a.cell_diameter_mm * 1e-3, sx * 1e-3, sy * 1e-3
    v = cfm * 0.000471947443 / (ny * s_t * a.cell_length_mm * 1e-3)
    vmax = v * s_t / (s_t - d)
    if stag:
        s_d = math.hypot(s_l, s_t / 2.0)
        if 2.0 * (s_d - d) < s_t - d:
            vmax = v * s_t / (2.0 * (s_d - d))
    re = vmax * d / a.air_nu_m2_s
    means = [pt.zukauskas_nu(re, a.air_pr, k, s_t / s_l, stag) * a.air_k_w_mk / d for k in range(1, nx + 1)]
    rows = [means[0]] + [(k + 1) * means[k] - k * means[k - 1] for k in range(1, nx)]
    return np.maximum(np.array(rows), a.h_floor)


def test_duct_rows_on_a_grid():
    a = pt.AirflowParams(mode="duct", duct_cfm=40.0)
    h = a.h_field(_grid(27.0, 27.0, 6, 4), []).reshape(4, 6)
    assert np.allclose(h, _expected_rows(a, 27.0, 27.0, 6, 4, 40.0)[None, :], rtol=1e-12, atol=0)
    assert h[0, 0] < h[0, 2] and h[1, 3] == h[0, 3]                   # first rows cooler; rows across alike


def test_duct_flow_from_the_fans_when_not_given():
    pos = _grid(27.0, 27.0, 5, 3)
    fans = [pt.Fan(x_mm=0.0, y_mm=0.0, cfm=30.0), pt.Fan(x_mm=0.0, y_mm=50.0, cfm=10.0), pt.Fan(0.0, 0.0, cfm=-5.0)]
    a = pt.AirflowParams(mode="duct")
    assert np.allclose(a.duct_h_field(pos, fans), pt.AirflowParams(mode="duct", duct_cfm=40.0).duct_h_field(pos),
                       rtol=1e-12, atol=0)


def test_duct_staggered_both_gap_cases():
    for sx, sy in ((15.0, 40.0), (40.0, 30.0)):                      # diagonal gap tightest / transverse tightest
        a = pt.AirflowParams(mode="duct", duct_cfm=25.0, staggered=True)
        h = a.duct_h_field(_grid(sx, sy, 4, 3)).reshape(3, 4)[0]
        assert np.allclose(h, _expected_rows(a, sx, sy, 4, 3, 25.0, stag=True), rtol=1e-12, atol=0)


def test_duct_single_row_or_column_uses_two_diameters():
    a = pt.AirflowParams(mode="duct", duct_cfm=10.0)
    one_col = a.duct_h_field(_grid(27.0, 30.0, 1, 3))
    assert np.allclose(one_col, _expected_rows(a, 42.0, 30.0, 1, 3, 10.0)[0], rtol=1e-12, atol=0)
    one_row = a.duct_h_field(_grid(30.0, 27.0, 3, 1))
    assert np.allclose(one_row, _expected_rows(a, 30.0, 42.0, 3, 1, 10.0), rtol=1e-12, atol=0)


def test_duct_falls_back_to_the_floor():
    pos = _grid(27.0, 27.0, 3, 2)
    floor = pt.AirflowParams().h_floor
    assert np.all(pt.AirflowParams(mode="duct").duct_h_field(pos) == floor)                     # no flow
    assert np.all(pt.AirflowParams(mode="duct", duct_cfm=10.0, cell_diameter_mm=27.0).duct_h_field(pos) == floor)
    assert np.all(pt.AirflowParams(mode="duct", duct_cfm=10.0, cell_length_mm=0.0).duct_h_field(pos) == floor)
    assert np.all(pt.AirflowParams(mode="duct", duct_cfm=1e-4).duct_h_field(pos) == floor)       # floor clamps
    assert pt.AirflowParams(mode="duct", duct_cfm=10.0).duct_h_field(np.zeros(3)).size == 0
    junk = pt.AirflowParams(mode="duct", duct_cfm=10.0).duct_h_field([["a", "b"], ["c", "d"], ["e", "f"]])
    assert junk.tolist() == [floor] * 3
    bad = pt.AirflowParams(mode="duct", duct_cfm=10.0, air_nu_m2_s=0.0)                         # raises inside
    assert np.all(bad.duct_h_field(pos) == floor)


def test_zukauskas_default_ratio_is_one():
    assert pt.zukauskas_nu(5000.0, 0.7, 4, staggered=True) == pytest.approx(_nu(0.35, 0.6, 5000.0, 0.7, 0.8942),
                                                                            rel=1e-12)


def test_duct_staggered_gap_between_two_and_three_diagonals():
    #  2 (s_d - d) = 8 mm < s_t - d = 10 mm < 3 (s_d - d) = 12 mm: the diagonal gap sets the speed
    sx = math.sqrt(25.0 ** 2 - 15.5 ** 2)
    a = pt.AirflowParams(mode="duct", duct_cfm=25.0, staggered=True)
    h = a.duct_h_field(_grid(sx, 31.0, 4, 3)).reshape(3, 4)[0]
    assert np.allclose(h, _expected_rows(a, sx, 31.0, 4, 3, 25.0, stag=True), rtol=1e-6, atol=0)   # x to 1e-6 mm


def test_duct_staggered_one_or_two_columns_still_use_the_pitch_ratio():
    a = pt.AirflowParams(mode="duct", duct_cfm=20.0, staggered=True)
    one = a.duct_h_field(_grid(27.0, 30.0, 1, 3))
    assert np.allclose(one, _expected_rows(a, 42.0, 30.0, 1, 3, 20.0, stag=True)[0], rtol=1e-12, atol=0)
    two = a.duct_h_field(_grid(50.0, 30.0, 2, 3)).reshape(3, 2)[0]
    assert np.allclose(two, _expected_rows(a, 50.0, 30.0, 2, 3, 20.0, stag=True), rtol=1e-12, atol=0)


def test_duct_small_given_flow_beats_the_fans():
    pos = _grid(27.0, 27.0, 3, 2)
    fans = [pt.Fan(x_mm=0.0, y_mm=0.0, cfm=60.0)]
    a = pt.AirflowParams(mode="duct", duct_cfm=0.5)
    assert np.allclose(a.duct_h_field(pos, fans), pt.AirflowParams(mode="duct", duct_cfm=0.5).duct_h_field(pos),
                       rtol=1e-12, atol=0)
    assert not np.allclose(a.duct_h_field(pos, fans), pt.AirflowParams(mode="duct").duct_h_field(pos, fans))


def test_duct_mode_dispatch_passes_the_fans():
    pos = _grid(27.0, 27.0, 3, 2)
    fans = [pt.Fan(x_mm=0.0, y_mm=0.0, cfm=40.0)]
    a = pt.AirflowParams(mode="duct")
    assert np.array_equal(a.h_field(pos, fans), a.duct_h_field(pos, fans))
    assert a.h_field(pos, fans)[0] > a.h_floor


def test_duct_floor_has_one_value_per_cell():
    pos = _grid(27.0, 27.0, 3, 2)
    for a in (pt.AirflowParams(mode="duct"), pt.AirflowParams(mode="duct", duct_cfm=10.0, air_nu_m2_s=0.0)):
        h = a.duct_h_field(pos)
        assert h.shape == (6,) and h.dtype == float and np.all(h == 5.5)


def test_duct_gap_warning_below_one_and_a_quarter_diameters():
    t, cur = np.arange(0.0, 20.0), np.full(20, 50.0)
    duct = pt.AirflowParams(mode="duct", duct_cfm=30.0)

    def warns(pitch, air=duct):
        lay = pt.PackLayout(rows=2, cols=3, cell=pt.CellParams(pitch_mm=pitch))
        return pt.PackThermalModel(layout=lay, airflow=air).simulate(t, cur).warnings
    msg = ("Cell pitch 26 mm is under 1.25 x the 21 mm can: the tube-bank correlation behind the duct mode was "
           "fitted to wider gaps, so its cooling here is an extrapolation.")
    assert msg in warns(26.0)
    assert not any("Cell pitch" in w for w in warns(26.25))                  # exactly 1.25 x is inside the range
    assert not any("Cell pitch" in w for w in warns(26.0, pt.AirflowParams(mode="proxy")))
    assert not any("Cell pitch" in w for w in warns(26.0, pt.AirflowParams()))      # duct, but no flow


def test_cooling_band_runs_at_one_plus_and_minus_the_spread():
    lay = pt.PackLayout(rows=2, cols=3, parallel=6)
    t, cur = np.arange(0.0, 200.0), np.full(200, 90.0)
    air = pt.AirflowParams(h_scale=0.5)                                 # proxy mode: the fans set the cooling
    fans = [pt.Fan(0.0, 0.0, cfm=60.0)]
    b = pt.cooling_band(lay, t, cur, fans=fans, airflow=air, k_cell_cell=0.2, n_laps=2, spread=0.25)

    def peak(scale):
        from dataclasses import replace
        r = pt.PackThermalModel(layout=lay, fans=fans, airflow=replace(air, h_scale=0.5 * scale),
                                k_cell_cell=0.2).simulate(t, cur, n_laps=2)
        return float(np.max(r.peak_temp_c))
    assert b["peak_c"] == pytest.approx(peak(1.0), rel=1e-12) and b["result"].ok
    assert b["peak_low_c"] == pytest.approx(peak(1.25), rel=1e-12)
    assert b["peak_high_c"] == pytest.approx(peak(0.75), rel=1e-12)
    assert b["peak_low_c"] < b["peak_c"] < b["peak_high_c"]
    assert b["result"].time_s.size == 2 * t.size


def test_cooling_band_defaults_and_a_failed_run():
    lay = pt.PackLayout(rows=2, cols=2)
    t, cur = np.arange(0.0, 100.0), np.full(100, 60.0)
    b = pt.cooling_band(lay, t, cur)
    from dataclasses import replace
    hi = pt.PackThermalModel(layout=lay, airflow=pt.AirflowParams(h_scale=0.8)).simulate(t, cur)
    assert b["peak_high_c"] == pytest.approx(float(np.max(hi.peak_temp_c)), rel=1e-12)
    assert replace(pt.AirflowParams()).h_scale == 1.0
    bad = pt.cooling_band(lay, [0.0], [1.0])
    assert math.isnan(bad["peak_c"]) and math.isnan(bad["peak_low_c"]) and not bad["result"].ok
