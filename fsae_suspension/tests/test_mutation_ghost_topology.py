"""Characterization suite for suspension/ghost_topology.py (mutation gate); see tests/_golden.py."""
import dataclasses
import math
import types

import numpy as np
import pytest

from suspension import ghost_topology as gt
from suspension import loadpath as lp
from suspension.kinematics import Hardpoints
from tests._golden import check

HP = Hardpoints.default()
W = lp.WheelLoad


def _ns(**kw):
    return types.SimpleNamespace(**kw)


def test_sections_tire_and_rc():
    res = {}
    secs = {"d": gt.MemberSection(), "al": gt.MemberSection("Aluminium 6061", 12.0, 1.0, 276.0),
            "odd": gt.MemberSection("Unobtainium", 15.0, 1.2, 300.0), "solid": gt.MemberSection(od_mm=10.0, wall_mm=5.0),
            "zero": gt.MemberSection(od_mm=0.0, wall_mm=0.0)}
    for k, s in secs.items():
        res[k] = [s.area_mm2, s.I_mm4, s.euler_pcr_N(0.0), s.euler_pcr_N(-5.0), s.euler_pcr_N(300.0),
                  [s.margins(T, L) for T in (5000.0, -5000.0, -200.0, 0.0, -1e-300) for L in (250.0, 900.0)]]
    res["uni"] = [gt.uniform_sections(), gt.uniform_sections(16.0, 1.2, "Aluminium 7075", 500.0, 12.0, 1.0),
                  gt.uniform_sections(tie_od_mm=10.0), gt.uniform_sections(tie_wall_mm=2.0)]
    res["tire"] = [gt.TireSensitivity.representative(W(Fy=fy, Fz=fz)) for fy in (-100.0, 0.0, 100.0)
                   for fz in (-50.0, 0.0, 1500.0)]
    res["tire_kw"] = gt.TireSensitivity.representative(W(Fy=-1.0, Fz=2000.0), 30.0, 200.0)
    res["dfy"] = [gt.TireSensitivity(2.0, -3.0).dFy(0.5, 0.25), gt.TireSensitivity().dFy(1.0, 1.0)]
    cp = np.array([0.0, 600.0, 0.0])
    res["rc"] = [gt._rc_height_mm(_ns(contact_patch=cp, instant_center=np.array([y, z])), 1200.0)
                 for y, z in ((2000.0, 150.0), (600.0, 30.0), (600.0 + 1e-10, 30.0), (np.nan, 1.0), (-500.0, 80.0),
                              (np.inf, 2.0))]
    res["consts"] = [gt._MARGIN_MEMBERS, gt.VERDICTS, gt._CORNER_INDEX, sorted(gt._RIGHT_SIDE),
                     gt.GhostThresholds()]
    check("ghost_basic", res)


def test_corner_solves():
    res = {}
    gc = gt.GhostCorner.uniform_tube(HP, wheel_rate_N_per_mm=35.0, Fz_static_N=700.0)
    res["init"] = [gc.wheel_rate, gc.Fz_static, gc.track_mm, gc._travel_clamp, sorted(gc.sections)]
    res["travel"] = [gc._travel_for(fz) for fz in (700.0, 1400.0, 700.0 + 35.0 * 41, 700.0 - 35.0 * 45, 0.0)]
    res["travel_none"] = [gt.GhostCorner(gc.cc)._travel_for(900.0),
                          gt.GhostCorner(gc.cc, wheel_rate_N_per_mm=0.0, Fz_static_N=1.0)._travel_for(900.0),
                          gt.GhostCorner(gc.cc, wheel_rate_N_per_mm=-1.0, Fz_static_N=1.0)._travel_for(900.0),
                          gt.GhostCorner(gc.cc, wheel_rate_N_per_mm=20.0)._travel_for(900.0)]
    res["rigid"] = [gc._rigid_at(v).camber for v in (0.0, 5e-10, 12.0, -20.0)]
    real = gc.cc.rigid_kin.solve_at_travel
    gc.cc.rigid_kin.solve_at_travel = lambda tr: dataclasses.replace(real(tr), converged=False) \
        if hasattr(real(tr), "converged") else _ns(converged=False, camber=99.0)
    res["rigid_nc"] = gc._rigid_at(12.0).camber
    gc.cc.rigid_kin.solve_at_travel = real
    loads = {"lat": W(Fx=200.0, Fy=-3000.0, Fz=1500.0), "small": W(Fy=-10.0, Fz=700.0),
             "brake": W(Fx=-1500.0, Fy=500.0, Fz=1900.0, Mz=1e4), "clamp": W(Fy=-2000.0, Fz=4000.0)}
    for k, ld in loads.items():
        g = gc.solve_instant(ld, t=0.25)
        res["si_" + k] = [g.summary(), g.load_path_shift, g.margins, g.d_ic_mm, g.travel_mm, g.load_open]
    res["si_kw"] = gc.solve_instant(loads["lat"], tire=gt.TireSensitivity(-800.0, 2500.0), fos_limit=3.0,
                                    feedback_tol_N=0.1, max_feedback_iter=1).summary()
    res["si_polish"] = gc.solve_instant(loads["lat"], tire=gt.TireSensitivity(-800.0, 2500.0), feedback_tol_N=1e-9,
                                        max_feedback_iter=3).summary()
    res["si_cap"] = gc.solve_instant(loads["lat"], tire=gt.TireSensitivity(-150.0, 300.0), feedback_tol_N=1e-9,
                                     max_feedback_iter=0).summary()
    res["si_div"] = gc.solve_instant(loads["lat"], tire=gt.TireSensitivity(-9e4, 9e5)).summary()
    res["si_tol"] = gc.solve_instant(loads["lat"], feedback_tol_N=1e6).summary()
    weak = gt.GhostCorner(gc.cc, gt.uniform_sections(od_mm=6.0, wall_mm=0.3), 35.0, 700.0)
    res["si_weak"] = weak.solve_instant(loads["lat"]).summary()
    mid = gt.GhostCorner(gc.cc, gt.uniform_sections(od_mm=10.0, wall_mm=0.6), 35.0, 700.0)
    res["si_mid"] = mid.solve_instant(loads["brake"], fos_limit=50.0).summary()
    part = gt.GhostCorner(gc.cc, {"UF": gt.MemberSection()}, track_mm=1100.0)
    res["si_part"] = [part.solve_instant(loads["small"]).summary(), sorted(part.solve_instant(loads["small"]).margins)]
    none = gt.GhostCorner(gc.cc, {})
    res["si_none"] = none.solve_instant(loads["small"]).summary()
    orig = gc.cc.solve
    gc.cc.solve = lambda load: dataclasses.replace(orig(load), converged=False)
    res["si_nc"] = gc.solve_instant(loads["small"]).warnings
    gc.cc.solve = orig
    soft = gt.GhostCorner.uniform_tube(HP, material="Aluminium 6061", od_mm=12.0, wall_mm=1.0, yield_MPa=276.0,
                                       k_tab=1500.0, wheel_rate_N_per_mm=30.0, Fz_static_N=650.0, track_mm=1150.0)
    res["soft"] = [soft.track_mm, soft.sections["UF"], soft.solve_instant(loads["lat"]).summary()]
    check("ghost_corner", res)


def _fake_instant(t, Fy, Fz, cam=-1.5, toe=0.1, dc=0.0, dt=0.0, rc=0.0, cp=0.0, fos=5.0, conv=True, gain=0.0,
                  warn=()):
    st = _ns(camber=cam, toe=toe)
    return gt.GhostInstant(t=t, load=W(Fy=Fy, Fz=Fz), load_open=W(Fy=Fy, Fz=Fz), travel_mm=0.0, rigid=st, ghost=st,
                           d_camber=dc, d_toe=dt, d_rc_height_mm=rc, d_ic_mm=np.zeros(2), d_cp_lateral_mm=cp,
                           load_path_shift={}, margins={"UF": {"mode": "buckling"}}, min_fos=fos,
                           min_fos_member="UF", loop_gain=gain, feedback_converged=conv, feedback_dFy_N=0.0,
                           compliance=None, warnings=list(warn))


class _FakeCorner:
    def __init__(self, table):
        self.table, self.calls = table, []

    def solve_instant(self, load, t=0.0, tire=None, fos_limit=1.5):
        self.calls.append([load, t, tire, fos_limit])
        return _fake_instant(t, load.Fy, load.Fz, **self.table.get(round(load.Fy), {}))


def test_audit_logic():
    res = {}
    t = np.linspace(0.0, 1.0, 11)
    Fz = np.full(11, 900.0)
    tables = {
        "faithful": {},
        "diverge": {-300: dict(conv=False, gain=1.4), -500: dict(conv=False, gain=2.0)},
        "camber_inv": {-300: dict(dc=1.6), -400: dict(dc=1.5 + 0.049), -500: dict(dc=3.0)},
        "toe_inv": {-300: dict(dt=-0.16), -400: dict(dt=-0.149)},
        "zero_rigid": {-300: dict(cam=1e-7, dc=-1.0, toe=0.0, dt=1.0)},
        "breach": {-300: dict(fos=1.2), -400: dict(fos=0.8), -500: dict(fos=1.5)},
        "breach_warn": {-300: dict(fos=1.2), -400: dict(fos=1.49)},
        "degraded": {-300: dict(dc=0.51), -400: dict(dc=-0.6, dt=0.3)},
        "deg_toe": {-300: dict(dt=0.21)}, "deg_rc": {-300: dict(rc=10.5)}, "deg_rc_nan": {-300: dict(rc=np.nan)},
        "deg_cp": {-300: dict(cp=-2.5)}, "deg_edge": {-300: dict(dc=0.5, dt=0.2, rc=10.0, cp=2.0)},
        "inv_and_deg": {-300: dict(dc=2.0), -400: dict(dt=0.5)},
        "slew_warn": {-300: dict(warn=("faster than the quasi-static model: x",))},
        "all": {-300: dict(conv=False, gain=1.1, dc=2.0, fos=0.5)},
    }
    Fy = np.array([-100.0, -200.0, -300.0, -400.0, -500.0, -450.0, -350.0, -250.0, -150.0, -50.0, 0.0])
    for k, tb in tables.items():
        fc = _FakeCorner(tb)
        a = gt.ghost_audit(fc, t, np.zeros(11), Fy, Fz, corner_label=k, n_samples=11)
        res[k] = [a, a.summary(), gt.render_ghost_md(a), [c[1:] for c in fc.calls]]
    th = gt.GhostThresholds(fos_limit=2.0, camber_inversion=0.01, toe_flip_deg=0.01, degraded_camber=0.1,
                            degraded_toe=0.1, degraded_rc_mm=1.0, degraded_cp_mm=0.5, slew_limit_ms=300.0)
    fc = _FakeCorner(tables["breach"])
    a = gt.ghost_audit(fc, t, np.zeros(11), Fy, Fz, corner_label="th", n_samples=0,
                       tire=gt.TireSensitivity(1.0, 2.0), thresholds=th, cache_quantum_N=120.0)
    res["thresh"] = [a.summary(), [c[1:] for c in fc.calls], a.trace("Fy"), a.trace("t")]
    step = np.where(t < 0.5, -100.0, -2000.0)
    fc = _FakeCorner({})
    a = gt.ghost_audit(fc, np.linspace(0.0, 0.01, 11), np.zeros(11), step, Fz, n_samples=3)
    res["slew"] = [a.summary(), [g.warnings for g in a.instants]]
    a2 = gt.ghost_audit(fc, np.linspace(0.0, 1.0, 11), np.zeros(11), np.full(11, -100.0), Fz, n_samples=4)
    res["flat"] = a2.summary()
    res["short"] = gt.ghost_audit(_FakeCorner({}), [0.0, 0.001], [0.0, 1.0], [-5.0, -3000.0], [800.0, 900.0]).summary()
    for k, args in {"empty": ([], [], [], []), "nan": ([0.0, 1.0], [0.0, np.nan], [0.0, 1.0], [1.0, 1.0]),
                    "inf_y": ([0.0, 1.0], [0.0, 0.0], [0.0, np.inf], [1.0, 1.0]),
                    "nan_z": ([0.0, 1.0], [0.0, 0.0], [0.0, 1.0], [1.0, np.nan])}.items():
        a = gt.ghost_audit(_FakeCorner({}), *args, corner_label=k)
        res["e_" + k] = [a.summary(), gt.render_ghost_md(a)]
    res["pick"] = [gt._pick_sample_indices(np.arange(9.0), np.array([0, 5, 0, 0, 0, 0, 0, 0, 0.0]),
                                           np.array([0, 0, -7, 0, 0, 0, 0, 0, 0.0]),
                                           np.array([5, 5, 5, 9, 5, 1, 5, 5, 5.0]), n) for n in (0, 1, 3, 9)]
    keys = ["t", "d_camber", "d_toe", "camber_ghost", "camber_rigid", "d_rc", "d_cp", "min_fos", "loop_gain",
            "Fy", "Fz"]
    a = gt.ghost_audit(_FakeCorner({-300: dict(dc=0.3, dt=-0.1, rc=2.0, cp=1.0, fos=3.0, gain=0.2)}), t,
                       np.zeros(11), Fy, Fz, n_samples=11)
    res["traces"] = {k: a.trace(k) for k in keys}
    g = _fake_instant(0.0, -5.0, 1.0, rc=np.nan)
    md_a = gt.GhostAudit("x", "RIGID_FAITHFUL", [], [g], [{"check": "c", "severity": "odd", "message": "m"}],
                         gt.GhostThresholds(), 1, 0)
    res["md_nan"] = gt.render_ghost_md(md_a)
    check("ghost_audit", res)


def test_transient_wrapper():
    res = {}
    fc = _FakeCorner({})
    n = 5
    r = _ns(ok=True, t=np.linspace(0, 0.4, n), Fx=np.arange(4 * n, dtype=float).reshape(n, 4) * 10.0,
            Fy=np.arange(4 * n, dtype=float).reshape(n, 4) * -20.0, Fz=600.0 + np.arange(4 * n, dtype=float).reshape(n, 4))
    for c in ("FL", "fr", "RL", "RR"):
        a = gt.ghost_audit_transient(fc, r, corner=c, n_samples=2)
        res[c] = [a.corner_label, [[g.load.Fy, g.load.Fz, g.t] for g in a.instants]]
    res["calls"] = [[c[0].Fx, c[0].Fy, c[0].Fz] for c in fc.calls]
    res["failed"] = gt.ghost_audit_transient(fc, _ns(ok=False, warnings=["a", "b", "c"]), corner="FL").summary()
    res["failed_nowarn"] = gt.ghost_audit_transient(fc, _ns(), corner="RR").summary()
    with pytest.raises(ValueError) as e:
        gt.ghost_audit_transient(fc, r, corner="XX")
    res["bad"] = str(e.value)
    check("ghost_transient", res)


def test_real_audit():
    gc = gt.GhostCorner.uniform_tube(HP, wheel_rate_N_per_mm=35.0, Fz_static_N=700.0)
    t = np.linspace(0.0, 0.8, 41)
    shape = np.sin(np.pi * t / 0.8)
    Fz = 700.0 + 1500.0 * shape
    a = gt.ghost_audit(gc, t, np.zeros_like(t), -3.0 * Fz * shape, Fz, corner_label="FR-demo", n_samples=3)
    check("ghost_real", {"s": a.summary(), "md": gt.render_ghost_md(a)})


def test_characterized_wc_loads_and_track():
    gc = gt.GhostCorner.uniform_tube(HP, wheel_rate_N_per_mm=35.0, Fz_static_N=700.0, track_mm=1430.0)
    wc = W(Fx=150.0, Fy=-2600.0, Fz=1600.0, Fx_wc=-900.0)
    g = gc.solve_instant(wc, t=0.0125)
    g0 = gc.solve_instant(W(Fx=150.0, Fy=-2600.0, Fz=1600.0), t=0.0125)
    res = {"g": g, "g0": g0}
    t = np.array([0.0, 0.0125, 0.0375, 0.0625, 0.0875, 0.1125])
    Fz = np.array([700.0, 1500.0, 2600.0, 2900.0, 2000.0, 900.0])
    a = gt.ghost_audit(gc, t, 0.4 * Fz, -3.2 * Fz, Fz, corner_label="FL-wc", n_samples=4)
    res["audit"] = {"s": a.summary(), "md": gt.render_ghost_md(a)}
    check("ghost_wc", res)


def test_characterized_messages_and_options():
    gc = gt.GhostCorner.uniform_tube(HP, wheel_rate_N_per_mm=35.0, Fz_static_N=700.0, tie_od_mm=12.7)
    bare = gt.GhostCorner(gc.cc, wheel_rate_N_per_mm=0.5, Fz_static_N=700.0)
    res = {"tie": gc.solve_instant(W(Fy=-1800.0, Fz=1400.0)).min_fos, "bare_sections": sorted(bare.sections),
           "bare_travel": bare._travel_for(900.0), "tiny_fy": gc.solve_instant(W(Fy=0.5, Fz=900.0)).load,
           "short": gt.MemberSection(od_mm=19.05, wall_mm=0.9).euler_pcr_N(0.5)}
    t = np.array([0.0, 0.0125, 0.0625, 0.1125, 0.1625])
    table = {-300: dict(conv=False, gain=1.4, dc=1.6, fos=0.8), -400: dict(dt=-0.16, fos=1.0), -500: dict(dc=0.51)}
    a = gt.ghost_audit(_FakeCorner(table), t, np.zeros(5), np.array([-100.0, -300.0, -400.0, -500.0, -200.0]),
                       np.full(5, 900.0), corner_label="X", n_samples=5)
    res["md"] = gt.render_ghost_md(a)
    check("ghost_messages", res)


def test_an_empty_or_non_finite_audit_keeps_its_thresholds():
    th = gt.GhostThresholds(fos_limit=1.7)
    for t, fy in ((np.zeros(0), np.zeros(0)), (np.zeros(2), np.array([np.nan, 0.0]))):
        a = gt.ghost_audit(_FakeCorner({}), t, np.zeros(t.size), fy, np.zeros(t.size), thresholds=th)
        assert a.verdict == "RIGID_FAITHFUL" and a.thresholds is th
