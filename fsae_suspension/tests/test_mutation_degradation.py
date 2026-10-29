"""suspension/degradation.py: the first-order heat-soak ramp, differential thermal expansion of the chassis pickups
(alpha x local dT x distance from the datum, local dT decaying from the heat source), the empirical tyre grip window,
how each degraded state is composed from nominal, thermal and compliant solves, the drift split by cause, the proxy
grip loss, and the tolerance map. The solver-backed helpers are checked once against the real solver and stubbed
where only the composition is under test."""
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

from suspension import degradation as dg
from suspension import compliance as comp
from suspension import loadpath as lp
from suspension.kinematics import Hardpoints, SuspensionKinematics

HP = Hardpoints.default()


def test_the_heat_soak_ramp():
    r = dg.ThermalRamp()
    assert (r.T_amb, r.T_soak, r.tau_min, r.run_min) == (20.0, 55.0, 6.0, 25.0)
    c = dg.ThermalRamp(T_amb=15.0, T_soak=70.0, tau_min=4.0, run_min=30.0)
    assert c.temp_at(5.0) == pytest.approx(15 + 55 * (1 - math.exp(-5 / 4)))
    assert c.delta_T_at(5.0) == pytest.approx(55 * (1 - math.exp(-5 / 4)))
    assert c.progress_at(5.0) == pytest.approx(1 - math.exp(-5 / 4))
    assert dg.ThermalRamp(T_amb=30.0, T_soak=30.0).progress_at(9.0) == 0.0
    assert dg.THERMAL_ALPHA == {"Steel 4130": 12.3e-6, "Steel mild": 12.0e-6, "Aluminium 6061": 23.6e-6,
                                "Aluminium 7075": 23.4e-6, "Titanium Ti-6Al-4V": 8.6e-6,
                                "Carbon (axial, representative)": 1.0e-6}


def test_differential_thermal_expansion():
    m = dg.ThermalExpansionModel()
    assert (m.material, m.datum, m.heat_source, m.gradient_len) == ("Steel 4130", None, None, 350.0)
    assert m.heated_points == ("upper_front_inner", "upper_rear_inner", "lower_front_inner", "lower_rear_inner",
                               "tie_rod_inner", "spring_inner", "rocker_pivot")
    assert m.alpha() == 12.3e-6 and dg.ThermalExpansionModel(material="Aluminium 6061").alpha() == 23.6e-6
    assert dg.ThermalExpansionModel(material="cheese").alpha() == 12.3e-6
    lf, lr = np.asarray(HP.lower_front_inner, float), np.asarray(HP.lower_rear_inner, float)
    src = np.array([lf[0], 0.0, lf[2]])
    datum = np.array([(lf[0] + lr[0]) / 2, 0.0, (lf[2] + lr[2]) / 2])
    assert m._source(HP) == pytest.approx(src) and m._datum(HP) == pytest.approx(datum)
    hot = m.expanded(HP, 40.0)
    for name in m.heated_points:
        p = getattr(HP, name, None)
        if p is None:
            continue
        p = np.asarray(p, float)
        dT = 40.0 * math.exp(-np.linalg.norm(p - src) / 350.0)
        assert np.asarray(getattr(hot, name)) == pytest.approx(datum + (p - datum) * (1 + 12.3e-6 * dT), rel=1e-12)
        assert m.point_shift_mm(HP, name, 40.0) == pytest.approx(12.3e-6 * dT * np.linalg.norm(p - datum))
    assert np.asarray(hot.upper_outer) == pytest.approx(np.asarray(HP.upper_outer))       # the upright does not grow
    assert HP.lower_front_inner is not hot.lower_front_inner
    e = dg.ThermalExpansionModel(material="Aluminium 6061", datum=[0.0, 300.0, 100.0], heat_source=[100.0, 200.0, 50.0],
                                 gradient_len=0.0, heated_points=("tie_rod_inner", "nope", "upper_front_inner"))
    assert e._local_dT(HP, [100.0, 200.0, 50.0], 30.0) == pytest.approx(30.0)
    assert e._local_dT(HP, [100.0, 200.0, 50.000001], 30.0) == pytest.approx(30.0 * math.exp(-1.0))
    h2 = e.expanded(HP, 30.0)
    assert np.asarray(h2.lower_front_inner) == pytest.approx(np.asarray(HP.lower_front_inner))
    at_datum = HP.copy()
    at_datum.tie_rod_inner = np.array([0.0, 300.0, 100.0 + 5e-10])
    assert np.asarray(e.expanded(at_datum, 30.0).tie_rod_inner) == pytest.approx(at_datum.tie_rod_inner)
    near = HP.copy()
    near.tie_rod_inner = np.array([0.0, 300.0, 100.0 + 2e-9])
    near2 = dg.ThermalExpansionModel(material="Aluminium 6061", datum=[0.0, 300.0, 100.0],
                                     heat_source=[0.0, 300.0, 100.0], heated_points=("tie_rod_inner",))
    moved = np.asarray(near2.expanded(near, 1e6).tie_rod_inner)
    assert moved[2] - 100.0 == pytest.approx(2e-9 * (1 + 23.6e-6 * 1e6), rel=1e-3)


def test_the_tyre_models_and_the_state_properties():
    g = dg.TyreThermalModel()
    assert (g.T_peak, g.width, g.floor) == (80.0, 45.0, 0.82)
    assert g.grip_mult(80.0) == 1.0 and g.grip_mult(95.0) == pytest.approx(1 - (15 / 45) ** 2)
    assert g.grip_mult(100.0) == 0.82                                       # 1 - (20/45)^2 is under the floor
    assert g.grip_mult(200.0) == 0.82
    r = dg.TyreThermalRamp()
    assert (r.T_amb, r.T_settle, r.tau_min) == (20.0, 95.0, 2.5)
    assert r.temp_at(3.0) == pytest.approx(20 + 75 * (1 - math.exp(-3 / 2.5)))
    s = dg.DegradedState(1.0, 2.0, 0.3, -1.0, -1.1, -1.4, 0.1, 0.12, 0.2, 50.0, 47.5, 0.95)
    assert (s.camber_drift, s.toe_drift, s.ic_drift_mm) == pytest.approx((-0.4, 0.1, -2.5))
    assert (s.camber_gain_nominal, s.camber_gain_thermal, s.bumpsteer_nominal, s.bumpsteer_thermal, s.converged) == (
        0.0, 0.0, 0.0, 0.0, True)
    c = dg.DegradationConfig()
    assert (c.lateral_g, c.axle, c.tube_od_mm, c.tube_wall_mm, c.use_bushings, c.bushing_rate_N_per_mm,
            c.n_steps) == (1.4, "front", 19.05, 0.9, True, 4000.0, 16)


def test_the_corner_metrics_come_from_the_real_solver():
    k = SuspensionKinematics(HP)
    cam, toe, icz = dg._corner_metrics(HP)
    assert (cam, toe, icz) == pytest.approx((k.static.camber, k.static.toe, np.asarray(k.static.instant_center)[1]))
    sts = k.sweep(-25.0, 25.0, 9)
    cg, bs = dg._corner_curves(HP)
    tr = [s.travel for s in sts]
    assert cg == pytest.approx(abs(np.polyfit(tr, [s.camber for s in sts], 1)[0]) * 25.0)
    assert bs == pytest.approx(max(s.toe for s in sts) - min(s.toe for s in sts))


def test_the_compliant_corner_and_the_wheel_load(monkeypatch):
    made = []
    monkeypatch.setattr(comp.JointCompliance, "rubber_bushing", staticmethod(lambda: "rubber"))
    monkeypatch.setattr(comp.CompliantCorner, "with_bushings", classmethod(
        lambda cls, hp, bushing, od_mm, wall_mm: made.append(("b", hp, bushing, od_mm, wall_mm)) or "B"))
    monkeypatch.setattr(comp.CompliantCorner, "uniform_tube", classmethod(
        lambda cls, hp, od_mm, wall_mm: made.append(("u", hp, od_mm, wall_mm)) or "U"))
    cfg = dg.DegradationConfig(tube_od_mm=22.0, tube_wall_mm=1.2, lateral_g=1.2, axle="rear")
    s = dg.DegradationSolver(HP, config=cfg)
    assert s._compliant_corner("hp") == "B" and made[-1] == ("b", "hp", "rubber", 22.0, 1.2)
    s.cfg = dg.DegradationConfig(use_bushings=False, tube_od_mm=22.0, tube_wall_mm=1.2)
    assert s._compliant_corner("hp") == "U" and made[-1] == ("u", "hp", 22.0, 1.2)
    monkeypatch.setattr(comp.JointCompliance, "rubber_bushing", staticmethod(lambda: None))
    s.cfg = cfg
    assert s._compliant_corner("hp") == "U"

    def boom():
        raise RuntimeError
    monkeypatch.setattr(comp.JointCompliance, "rubber_bushing", staticmethod(boom))
    assert s._compliant_corner("hp") == "U"
    w = s._wheel_load(HP)
    Fz = 1500.0 * (1 + 0.4 * 1.2)
    assert (w.Fx, w.Fy, w.Fz) == pytest.approx((0.0, -1.2 * Fz, Fz))
    monkeypatch.setattr(comp, "corner_wheel_load", lambda veh, axle, g, outer: ("load", veh, axle, g, outer))
    s.vehicle = "VEH"
    assert s._wheel_load(HP) == ("load", "VEH", "rear", 1.2, True)


def _stub(monkeypatch, s, solved=True):
    def metrics(hp):
        x = float(np.asarray(hp.tie_rod_inner)[0])
        return -1.0 + 1e-3 * x, 0.1 + 2e-3 * x, 40.0 + x
    monkeypatch.setattr(dg, "_corner_metrics", metrics)
    monkeypatch.setattr(dg, "_corner_curves", lambda hp: (0.5 + float(np.asarray(hp.tie_rod_inner)[1]) * 1e-4, 0.02))

    class Corner:
        def solve(self, load):
            if not solved:
                raise ValueError
            return NS(compliance_camber=-0.15, compliance_toe=0.04, converged=load.Fz > 0)
    monkeypatch.setattr(s, "_compliant_corner", lambda hp: Corner())


def test_a_degraded_state_is_composed_from_its_solves(monkeypatch):
    th = dg.ThermalRamp(T_soak=80.0)
    ex = dg.ThermalExpansionModel(heated_points=("tie_rod_inner",), gradient_len=1e9)
    s = dg.DegradationSolver(HP, thermal=th, expansion=ex, tyre_grip=dg.TyreThermalModel(T_peak=70.0),
                             tyre_temp=dg.TyreThermalRamp(T_settle=110.0))
    _stub(monkeypatch, s)
    st = s._state_at(10.0)
    hot = ex.expanded(HP, th.delta_T_at(10.0))
    m0, mh = dg._corner_metrics(HP), dg._corner_metrics(hot)
    assert (st.t_min, st.delta_T, st.progress) == pytest.approx((10.0, th.delta_T_at(10.0), th.progress_at(10.0)))
    assert (st.camber_nominal, st.toe_nominal, st.ic_z_nominal) == pytest.approx(m0)
    assert (st.camber_thermal, st.toe_thermal) == pytest.approx(mh[:2]) and st.camber_thermal != st.camber_nominal
    assert (st.camber_compliant, st.toe_compliant, st.ic_z_compliant) == pytest.approx((mh[0] - 0.15, mh[1] + 0.04,
                                                                                         mh[2]))
    assert (st.camber_gain_nominal, st.camber_gain_thermal) == pytest.approx((dg._corner_curves(HP)[0],
                                                                               dg._corner_curves(hot)[0]))
    assert st.bumpsteer_nominal == st.bumpsteer_thermal == 0.02 and st.converged is True
    assert st.grip_mult == pytest.approx(dg.TyreThermalModel(T_peak=70.0).grip_mult(
        dg.TyreThermalRamp(T_settle=110.0).temp_at(10.0)))
    s2 = dg.DegradationSolver(HP, thermal=th, expansion=ex)
    _stub(monkeypatch, s2, solved=False)
    f = s2._state_at(10.0)
    assert f.converged is False and (f.camber_compliant, f.toe_compliant) == pytest.approx(mh[:2])
    s3 = dg.DegradationSolver(HP, thermal=th, expansion=ex)
    _stub(monkeypatch, s3)
    monkeypatch.setattr(s3, "_wheel_load", lambda hp: lp.WheelLoad(Fz=0.0))
    assert s3._state_at(1.0).converged is False
    s4 = dg.DegradationSolver(HP, config=dg.DegradationConfig(n_steps=4), thermal=dg.ThermalRamp(run_min=9.0))
    monkeypatch.setattr(s4, "_state_at", lambda t: t)
    c = s4.run()
    assert c.states == pytest.approx([0.0, 3.0, 6.0, 9.0]) and c.solver is s4
    d = dg.DegradationSolver(HP)
    assert (d.thermal, d.expansion, d.tyre_grip, d.tyre_temp, d.cfg, d.vehicle) == (
        dg.ThermalRamp(), dg.ThermalExpansionModel(), dg.TyreThermalModel(), dg.TyreThermalRamp(),
        dg.DegradationConfig(), None)


def _st(cn, ct, cc, tn, tt, tc, cgn=0.0, cgt=0.0, bn=0.0, bt=0.0, grip=0.95, conv=True, t=0.0, dT=0.0):
    return dg.DegradedState(t, dT, 0.5, cn, ct, cc, tn, tt, tc, 50.0, 48.0, grip, cgn, cgt, bn, bt, conv)


def test_the_dominant_mechanism():
    assert dg._dominant_mechanism(_st(-1, -1, -1.6, 0, 0, 0)) == \
        "structural compliance (dominant) — chase bushing/link stiffness"
    assert dg._dominant_mechanism(_st(-1, -1.6, -1.6, 0, 0, 0)) == \
        "thermal expansion (dominant) — chase subframe thermal growth"
    assert dg._dominant_mechanism(_st(-1, -1.1, -1.2, 0, 0, 0)) == "mixed thermal + compliance"
    assert dg._dominant_mechanism(_st(-1, -1.1, -1.6, 0, 0, 0)) == "mixed thermal + compliance"   # exactly 5x
    assert dg._dominant_mechanism(_st(-1, -1, -1, 0, 0.01, 0.01, cgn=0.1, cgt=0.1, bn=0.0, bt=0.0)).startswith("thermal")
    assert dg._dominant_mechanism(_st(-1, -1, -1, 0, 0, 0, cgn=0.1, cgt=0.2)).startswith("thermal")
    assert dg._dominant_mechanism(_st(-1, -1, -1, 0, 0, 0, bn=0.1, bt=0.2)).startswith("thermal")
    assert dg._dominant_mechanism(_st(-1, -1, -1, 0, 0, 1e-8)).startswith("structural")


def test_the_curve_summaries():
    cold = _st(-1.0, -1.0, -1.1, 0.10, 0.10, 0.12, grip=0.9, t=0.0)
    hot = _st(-1.0, -1.04, -1.31, 0.10, 0.115, 0.16, cgn=0.5, cgt=0.53, bn=0.02, bt=0.025, grip=0.97, conv=False,
              t=25.0, dT=33.33)
    c = dg.DegradationCurve([cold, hot], solver=None)
    a = c.as_arrays()
    assert a["t_min"].tolist() == [0.0, 25.0] and a["delta_T"].tolist() == [0.0, 33.33]
    assert a["progress"].tolist() == [0.5, 0.5] and a["camber_drift"] == pytest.approx([-0.1, -0.31])
    assert a["camber_thermal_drift"] == pytest.approx([0.0, -0.04]) and a["toe_drift"] == pytest.approx([0.02, 0.06])
    assert a["toe_thermal_drift"] == pytest.approx([0.0, 0.015]) and a["ic_drift_mm"].tolist() == [-2.0, -2.0]
    assert a["grip_mult"].tolist() == [0.9, 0.97]
    assert c.lap15_summary() == {
        "camber_drift_deg": round(-0.31, 3), "camber_thermal_deg": round(-0.04, 3),
        "camber_compliance_deg": round(-0.27, 3), "toe_drift_deg": round(0.06, 3),
        "toe_thermal_deg": round(0.015, 3), "toe_compliance_deg": round(0.045, 3), "instant_centre_drift_mm": -2.0,
        "camber_gain_thermal_shift_deg": round(0.03, 4), "bumpsteer_thermal_shift_deg": round(0.005, 4),
        "dominant_mechanism": dg._dominant_mechanism(hot), "grip_mult_cold": 0.9, "grip_mult_hot": 0.97,
        "delta_T_final": 33.3, "converged": False}
    assert dg.DegradationCurve([cold, cold], None).lap15_summary()["converged"] is True
    p = c.laptime_delta_proxy()
    al = 0.31 * 0.04 + 0.06 * 0.06
    assert p == {"grip_loss_tyre_pct": round(100 * 0.03, 2), "grip_loss_alignment_pct": round(100 * al, 2),
                 "grip_loss_total_pct": round(100 * (0.03 + al), 2),
                 "note": "PROXY: % loss of peak lateral capability. Tyre term is EMPIRICAL; alignment term uses "
                         "calibratable grip sensitivities. Feed to lapsim.py for a time in seconds."}
    q = c.laptime_delta_proxy(camber_grip_sens_per_deg=0.1, toe_grip_sens_per_deg=0.2)
    assert q["grip_loss_alignment_pct"] == round(100 * (0.031 + 0.012), 2)


def test_the_tolerance_map(monkeypatch):
    def state(self, t):
        x = np.asarray(self.hp.upper_front_inner, float)
        y = np.asarray(self.hp.tie_rod_outer, float)
        z = np.asarray(self.hp.lower_outer, float)
        return NS(camber_compliant=0.01 * x.sum() + 0.002 * z[2], toe_compliant=0.03 * y[1] - 0.005 * y[0])
    monkeypatch.setattr(dg.DegradationSolver, "_state_at", state)
    sol = dg.DegradationSolver(HP, thermal=dg.ThermalRamp(run_min=20.0))
    m = dg.tolerance_sensitivity(HP, sol, jig_tol_mm=0.5)
    rows = {r["point"]: r for r in m.rows}
    assert list(rows) == ["upper_front_inner", "upper_rear_inner", "lower_front_inner", "lower_rear_inner",
                          "upper_outer", "lower_outer", "tie_rod_inner", "tie_rod_outer"]
    dT = sol.thermal.delta_T_at(20.0)
    want = {"upper_front_inner": 0.005, "tie_rod_outer": 0.015, "lower_outer": 0.001}
    for name, r in rows.items():
        sens = want.get(name, 0.0)
        th = sol.expansion.point_shift_mm(HP, name, dT) if name in sol.expansion.heated_points else 0.0
        assert r["alignment_move_deg"] == pytest.approx(round(sens, 4), abs=1e-12)
        assert r["build_sens_deg_per_mm"] == pytest.approx(round(sens / 0.5, 4), abs=1e-12)
        assert r["thermal_shift_mm"] == round(th, 3)
        assert r["combined_score"] == round(sens + 0.5 * th * sens, 4)
    scores = np.array([r["combined_score"] for r in m.rows])
    hi = np.percentile(scores, 75)
    assert [r["recommendation"] for r in m.rows] == ["CNC / tight tol" if s >= hi else "hand-weld OK" for s in scores]
    assert m.critical_points() == sorted(m.rows, key=lambda r: -r["combined_score"])[:2] == m.table()[:2]
    assert m.critical_points(top_n=3)[2] == m.table()[2]
    few = dg.tolerance_sensitivity(HP, sol, points=["upper_front_inner", "nope", "lower_outer"])
    assert [r["point"] for r in few.rows] == ["upper_front_inner", "lower_outer"]
    assert [r["recommendation"] for r in few.rows] == ["CNC / tight tol", "hand-weld OK"]
    assert few.rows[0]["build_sens_deg_per_mm"] == pytest.approx(0.01)                   # default 1 mm jig
    zero = dg.tolerance_sensitivity(HP, sol, jig_tol_mm=0.0, points=["upper_front_inner"])
    assert zero.rows[0]["build_sens_deg_per_mm"] == 0.0
    assert dg.tolerance_sensitivity(HP, sol, points=["nope"]).rows == []
    d = dg.tolerance_sensitivity(HP, points=["upper_front_inner"])
    assert d.rows[0]["alignment_move_deg"] == pytest.approx(0.01)


def test_the_dominance_boundary_and_shifts_with_nonzero_baselines():
    assert dg._dominant_mechanism(_st(-1.0, -1.25, -1.25 - 1.25, 0, 0, 0)) == "mixed thermal + compliance"   # exactly 5x
    assert dg._dominant_mechanism(_st(-1.0, -1.25, -1.25 - 1.375, 0, 0, 0)).startswith("structural")       # 5.5x
    assert dg._dominant_mechanism(_st(-1.0, -1.0 - 1.375, -1.0 - 1.375 - 0.25, 0, 0, 0)).startswith("thermal")
    assert dg._dominant_mechanism(_st(-1.0, -1.0 - 1.25, -1.0 - 1.25 - 0.25, 0, 0, 0)) == "mixed thermal + compliance"
    for kw in (dict(cgn=0.4, cgt=0.4), dict(bn=0.3, bt=0.3)):
        assert dg._dominant_mechanism(_st(-1, -1, -1.1, 0, 0, 0, **kw)).startswith("structural")
    assert dg._dominant_mechanism(_st(-1, -1, -1.1, 0.2, 0.2, 0.2)).startswith("structural")


def test_the_summaries_round_the_last_state():
    a = _st(-1.0, -1.0, -1.1, 0.1, 0.1, 0.12, grip=0.9123456)
    b = _st(-1.0, -1.01, -1.2, 0.1, 0.11, 0.13, grip=0.95)
    hot = _st(-1.0, -1.0412345, -1.3123456, 0.1, 0.1154321, 0.1612345, cgn=0.5, cgt=0.5312345, bn=0.02,
              bt=0.0254321, grip=0.9712345, dT=33.3456)
    hot.ic_z_compliant = 47.123456
    c = dg.DegradationCurve([a, b, hot], None)
    s = c.lap15_summary()
    assert s["camber_drift_deg"] == round(-0.3123456, 3) and s["camber_thermal_deg"] == round(-0.0412345, 3)
    assert s["camber_compliance_deg"] == round(-1.3123456 + 1.0412345, 3) and s["toe_drift_deg"] == round(0.0612345, 3)
    assert s["toe_thermal_deg"] == round(0.0154321, 3) and s["toe_compliance_deg"] == round(0.1612345 - 0.1154321, 3)
    assert s["instant_centre_drift_mm"] == round(47.123456 - 50.0, 2)
    assert s["camber_gain_thermal_shift_deg"] == round(0.0312345, 4) and s["bumpsteer_thermal_shift_deg"] == round(
        0.0054321, 4)
    assert s["grip_mult_cold"] == round(0.9123456, 3) and s["grip_mult_hot"] == round(0.9712345, 3)
    assert s["delta_T_final"] == 33.3
    p = c.laptime_delta_proxy()
    al = 0.3123456 * 0.04 + 0.0612345 * 0.06
    assert p["grip_loss_tyre_pct"] == round(100 * (1 - 0.9712345), 2)
    assert p["grip_loss_alignment_pct"] == round(100 * al, 2) and p["grip_loss_total_pct"] == round(
        100 * (1 - 0.9712345 + al), 2)


def test_the_solver_keeps_its_vehicle_and_hands_the_hot_corner_on(monkeypatch):
    assert dg.DegradationSolver(HP, vehicle="V").vehicle == "V"
    th = dg.ThermalRamp(T_soak=80.0)
    ex = dg.ThermalExpansionModel(heated_points=("tie_rod_inner",), gradient_len=1e9)
    s = dg.DegradationSolver(HP, thermal=th, expansion=ex)
    _stub(monkeypatch, s)
    seen = []
    real_cc = s._compliant_corner
    monkeypatch.setattr(s, "_compliant_corner", lambda hp: seen.append(("cc", hp)) or real_cc(hp))
    monkeypatch.setattr(s, "_wheel_load", lambda hp: seen.append(("wl", hp)) or lp.WheelLoad(Fz=900.0))
    s._state_at(10.0)
    hot = ex.expanded(HP, th.delta_T_at(10.0))
    assert [k for k, _ in seen] == ["cc", "wl"]
    for _, hp in seen:
        assert np.asarray(hp.tie_rod_inner) == pytest.approx(np.asarray(hot.tie_rod_inner))
        assert not np.allclose(np.asarray(hp.tie_rod_inner), np.asarray(HP.tie_rod_inner))
    e = dg.ThermalExpansionModel(heated_points=("nope", "upper_front_inner"))
    assert not np.allclose(np.asarray(e.expanded(HP, 40.0).upper_front_inner), np.asarray(HP.upper_front_inner))
    d = HP.copy()
    m = dg.ThermalExpansionModel(heated_points=("tie_rod_inner", "upper_front_inner"))
    d.tie_rod_inner = m._datum(HP)
    assert not np.allclose(np.asarray(m.expanded(d, 40.0).upper_front_inner), np.asarray(HP.upper_front_inner))


def test_the_tolerance_map_hands_every_setting_on(monkeypatch):
    def state(self, t):
        x = np.asarray(self.hp.upper_front_inner, float)
        q = (x[0] - np.asarray(HP.upper_front_inner, float)[0])
        k = (t * 0.013 + self.cfg.lateral_g * 0.0071 + self.expansion.gradient_len * 1e-6 + self.tyre_grip.T_peak
             * 1e-5 + self.tyre_temp.T_settle * 1e-6 + self.thermal.T_soak * 1e-7 + (self.vehicle or 0) * 1e-3)
        return NS(camber_compliant=k + 0.0123457 * q * q + 0.0031 * q, toe_compliant=0.0)
    monkeypatch.setattr(dg.DegradationSolver, "_state_at", state)
    sol = dg.DegradationSolver(HP, thermal=dg.ThermalRamp(run_min=21.0, T_soak=61.0),
                               expansion=dg.ThermalExpansionModel(gradient_len=400.0),
                               tyre_grip=dg.TyreThermalModel(T_peak=77.0), tyre_temp=dg.TyreThermalRamp(T_settle=91.0),
                               config=dg.DegradationConfig(lateral_g=1.3), vehicle=7)
    r = dg.tolerance_sensitivity(HP, sol, jig_tol_mm=0.7, points=["upper_front_inner"]).rows[0]
    sens = 0.0123457 * 0.49 + 0.0031 * 0.7
    assert r["alignment_move_deg"] == round(sens, 4) and r["build_sens_deg_per_mm"] == round(sens / 0.7, 4)
    assert list(r) == ["point", "build_sens_deg_per_mm", "thermal_shift_mm", "alignment_move_deg", "combined_score",
                       "recommendation"]


def test_the_cnc_cut_is_the_upper_quartile(monkeypatch):
    pts = ["upper_front_inner", "upper_rear_inner", "lower_front_inner", "lower_rear_inner", "tie_rod_inner"]
    gain = dict(zip(pts, (0.01, 0.02, 0.03, 0.04, 0.05)))

    def state(self, t):
        return NS(camber_compliant=sum(g * float(np.asarray(getattr(self.hp, p), float).sum()) for p, g in gain.items()),
                  toe_compliant=0.0)
    monkeypatch.setattr(dg.DegradationSolver, "_state_at", state)
    m = dg.tolerance_sensitivity(HP, dg.DegradationSolver(HP, expansion=dg.ThermalExpansionModel(heated_points=())),
                                 points=pts)
    sc = [r["combined_score"] for r in m.rows]
    assert sc == pytest.approx([0.01, 0.02, 0.03, 0.04, 0.05])
    assert [r["recommendation"] for r in m.rows] == ["hand-weld OK"] * 3 + ["CNC / tight tol"] * 2   # 0.04 is the p75
