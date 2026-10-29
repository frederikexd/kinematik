"""Characterization suite for suspension/simulforge.py (mutation gate); see tests/_golden.py.

The coupled runs are kept to a few dozen milliseconds of manoeuvre; the linter is driven by hand-built results and a
stand-in ghost corner so every branch is reached without long transient solves."""
import types

import numpy as np
import pytest

from suspension import earshot as es
from suspension import fusebox as fb
from suspension import ghost_topology as gt
from suspension import loadpath as lp
from suspension import simulforge as sf
from suspension import transient as tr
from suspension.proof_engine import EvidenceGrade
from tests._golden import check

SHORT = dict(t_end=0.04, t_step=0.005, ramp=0.005)


def test_params_and_degradation():
    res = {}
    acts = [sf.ActuatorParams(), sf.ActuatorParams(R_ohm=0.0, L_H=1e-3, Kt=0.0), sf.ActuatorParams(gear=50.0, eff=0.5)]
    res["act"] = [[a.tau_ms, a.torque_from_current(3.0), a.current_for_torque(40.0)] for a in acts]
    res["bus"] = [sf.BusParams().r_total_ohm, sf.BusParams(R_int_ohm=-0.5, R_harness_ohm=0.1).r_total_ohm]
    degs = list(sf.degradation_presets().values()) + [
        sf.Degradation("neg", "n", "s", -1.0, -0.1, -2.0, -3.0, -0.5, -0.2),
        sf.Degradation("big", "b", "s", 2.0, 0.05, 30.0, 1.0, 2.0, 1.0), sf.Degradation(authority_scale=0.5)]
    bus, act = sf.BusParams(), sf.ActuatorParams()
    res["deg"] = [[d, d.apply(bus, act)] for d in degs]
    res["untouched"] = [bus, act]
    res["presets"] = sf.degradation_presets()
    res["thresholds"] = sf.ForgeThresholds()
    res["verdicts"] = [v.value for v in sf.ForgeVerdict]
    check("forge_params", res)


def _telem(n=40, dt=0.001, lag=0, scale=1.0, online=None, nb=0, e=1.0, v0=24.0, sag=1.0, cmd=None):
    t = np.arange(n) * dt
    base = np.sin(np.arange(n) * 0.4) * 100.0 + 120.0 if cmd is None else cmd
    M_cmd = np.stack([base, 0.5 * base], axis=1)
    act = np.roll(base, lag) * scale
    M_act = np.stack([act, 0.5 * act], axis=1)
    on = np.ones(n, bool) if online is None else online
    return sf.ForgeTelemetry(t=t, V_bus=v0 - sag * np.abs(np.sin(t * 50.0)), I_bus=8.0 + np.abs(base) * 0.05,
                             i_act=np.stack([base * 0.03, base * 0.02], axis=1), M_cmd=M_cmd, M_act=M_act,
                             online=on, E_Wh=np.linspace(0.0, e, n), n_brownouts=nb)


def test_telemetry_summary_and_lag():
    res = {}
    z = np.zeros(0)
    empty = sf.ForgeTelemetry(z, z, z, np.zeros((0, 2)), np.zeros((0, 2)), np.zeros((0, 2)), np.zeros(0, bool), z)
    one = sf.ForgeTelemetry(np.zeros(1), np.ones(1), np.ones(1), np.ones((1, 2)), np.ones((1, 2)), np.ones((1, 2)),
                            np.zeros(1, bool), np.ones(1))
    tl = {"empty": empty, "one": one, "short": _telem(7), "eight": _telem(8), "base": _telem(),
          "lag3": _telem(lag=3, scale=0.8), "lag60": _telem(n=200, lag=30, dt=0.002), "flat": _telem(cmd=np.ones(40)),
          "zero_cmd": _telem(cmd=np.zeros(40)), "off": _telem(online=np.arange(40) % 3 == 0, nb=2),
          "dt": _telem(dt=0.02, lag=2)}
    for k, t in tl.items():
        res[k] = [t.summary(), t.response_lag_ms()]
    a = _telem()
    a.M_act = np.zeros_like(a.M_act)
    res["act_zero"] = a.summary()
    check("forge_telem", res)


def test_solver_algebraic_and_coupled_runs(monkeypatch):
    res = {}
    s = sf.MechatronicSolver(None)
    y = s.initial_state(12.0)
    y[sf.IPHI] = 0.01
    A0 = s.algebraic(0.0, y, tr.DriverInput(), tr.RoadInput())
    s._M_held = np.array([120.0, -80.0])
    A1 = s.algebraic(0.0, y, tr.DriverInput(), tr.RoadInput())
    s._M_held = np.array([0.0, 50.0])
    A2 = s.algebraic(0.0, y, tr.DriverInput(), tr.RoadInput())
    res["alg"] = [[A["F_act"], A["F_susp"]] for A in (A0, A1, A2)]
    whole = {"alg": [A0, A1, A2]}
    bus, act = sf.BusParams(), sf.ActuatorParams()
    drv, road, t_end, u0, label = tr.step_steer_maneuver(**SHORT)
    runs = {"nominal": (bus, act), "dead": sf.degradation_presets()["dead_actuator"].apply(bus, act),
            "brown": sf.Degradation(r_harness_add_ohm=0.5, v_brownout_add=3.0, i_other_add_A=10.0).apply(bus, act),
            "soft": (sf.BusParams(reboot_s=-1.0, V_oc=24.0), sf.ActuatorParams(kp=40000.0, kd=3000.0, duty_max=0.5))}
    for k, (b, a) in runs.items():
        r = sf.MechatronicSolver(None).run_coupled(t_end, b, a, driver=drv, road=road, u0=u0)
        res[k] = [r.ok, r.elec, r.elec.summary(), r.mech.roll, r.mech.Fz, r.mech.meta, r.meta, r.warnings,
                  r.load_cycle_ptp_N(), r.roll_peak_deg()]
        whole[k] = r
    for k, (b, a_) in {"reboot": (sf.BusParams(V_oc=13.0, reboot_s=0.006, I_other_A=40.0), act),
                       "no_torque": (bus, sf.ActuatorParams(M_max_Nm=0.0)),
                       "no_duty": (bus, sf.ActuatorParams(duty_max=0.0))}.items():
        whole[k] = sf.MechatronicSolver(None).run_coupled(t_end, b, a_, driver=drv, road=road, u0=u0)
    whole["u0_dflt"] = sf.MechatronicSolver(None).run_coupled(0.01, bus, act, driver=drv, road=road)
    r = sf.MechatronicSolver(None).run_coupled(0.004, bus, act)
    res["defaults"] = [r.ok, r.elec.summary(), r.mech.t]
    r = sf.MechatronicSolver(None).run_coupled(0.004, None, act)
    res["fail"] = [r.ok, r.warnings, r.elec.summary(), r.load_cycle_ptp_N(), r.roll_peak_deg()]
    whole["fail"] = r
    real = tr.TransientSolver._rk4_step
    calls = {"n": 0}

    def blow(self, tt, yy, h, d, rd):
        calls["n"] += 1
        out = real(self, tt, yy, h, d, rd)
        return out * np.nan if calls["n"] >= 3 else out

    monkeypatch.setattr(tr.TransientSolver, "_rk4_step", blow)
    r = sf.MechatronicSolver(None).run_coupled(0.01, bus, act, driver=drv, road=road, u0=u0)
    res["blow"] = [r.ok, r.warnings, r.mech.t, r.elec.t, r.elec.V_bus, r.mech.roll]
    check("forge_runs", res)
    whole["blow"] = r
    check("forge_runs_whole", whole)


def test_run_simulforge_dispatch():
    res = {}
    whole = {}
    kinds = {"step_steer": SHORT, "snap_oversteer": dict(t_end=0.03, lift_time=0.01),
             "brake_to_throttle": dict(t_end=0.03, brake_time=0.01, brake_dur=0.01),
             "curb_strike": dict(t_end=0.03, t_hit=0.01)}
    for k, kw in kinds.items():
        r = sf.run_simulforge(None, kind=k, **kw)
        res[k] = [r.ok, r.meta, r.mech.meta, r.elec.summary(), r.degradation.key]
        whole[k] = r
    for k, deg in {"str": "sagging_pack", "unknown_str": "nope", "obj": sf.Degradation(key="z", r_int_scale=2.0),
                   "none": None}.items():
        r = sf.run_simulforge(None, degradation=deg, bus=sf.BusParams(V_oc=24.0),
                              actuator=sf.ActuatorParams(kp=9000.0), **SHORT)
        res["deg_" + k] = [r.degradation, r.bus, r.actuator, r.elec.summary()]
    bad = sf.run_simulforge(None, kind="teleport", degradation="corroded_connector")
    res["bad"] = [bad.ok, bad.warnings, bad.degradation, bad.bus, bad.actuator, bad.elec.summary()]
    boom = sf.run_simulforge(None, kind="step_steer", degradation="dead_actuator", no_such_kw=1)
    res["boom"] = [boom.ok, boom.mech.warnings, boom.degradation.key, boom.bus, boom.actuator, boom.warnings]
    res["params"] = sf.run_simulforge(None, params=tr.TransientParams.from_vehicle(None), **SHORT).elec.summary()
    check("forge_dispatch", res)
    whole["bad"], whole["boom"] = bad, boom
    check("forge_dispatch_whole", whole)


class _Gc:
    """Stand-in ghost corner: the member FoS scales with |Fy| so a harder event reads weaker."""

    def __init__(self, k=1.0):
        self.k = k

    def solve_instant(self, load, t=0.0, tire=None, fos_limit=1.5):
        st = types.SimpleNamespace(camber=-1.5, toe=0.0)
        f_uf = self.k * 4000.0 / max(abs(load.Fy), 1.0)
        mg = {"UF": {"fos": f_uf, "mode": "yield (tension)"}, "TR": {"fos": 2.5 * self.k, "mode": "buckling"},
              "PR": {"fos": float("inf"), "mode": "yield (tension)"}}
        m = min(mg, key=lambda k: mg[k]["fos"])
        return gt.GhostInstant(t=t, load=load, load_open=load, travel_mm=0.0, rigid=st, ghost=st, d_camber=0.0,
                               d_toe=0.0, d_rc_height_mm=0.0, d_ic_mm=np.zeros(2), d_cp_lateral_mm=0.0,
                               load_path_shift={}, margins=mg, min_fos=mg[m]["fos"], min_fos_member=m,
                               loop_gain=0.0, feedback_converged=True, feedback_dFy_N=0.0, compliance=None,
                               warnings=[])


def _result(ok=True, fy=1000.0, fz=1500.0, roll=0.02, telem=None, bus=None, act=None, deg=None, n=12):
    t = np.arange(n) * 0.001
    shape = np.sin(np.linspace(0.0, np.pi, n))
    F = lambda a: np.stack([a * shape * c for c in (1.0, 1.1, 0.9, 1.05)], axis=1)
    mech = types.SimpleNamespace(ok=ok, t=t, Fx=F(100.0), Fy=F(fy), Fz=F(fz) + 700.0,
                                 roll=roll * shape, warnings=[] if ok else ["mech failed"], meta={})
    return sf.SimulForgeResult(mech=mech, elec=telem if telem is not None else _telem(n),
                               degradation=deg or sf.Degradation(), bus=bus or sf.BusParams(),
                               actuator=act or sf.ActuatorParams(), warnings=[] if ok else ["run failed"])


def test_forge_lint_branches():
    res = {}
    whole = {}
    nom = _result()
    ab = es.ABDesign(effect_predicted=0.3, noise_sigma=0.8)
    ab_tight = es.ABDesign(effect_predicted=0.12, noise_sigma=0.3, alpha=0.1, power=0.7)
    cases = {
        "same": (nom, _result(), {}),
        "brownout": (nom, _result(telem=_telem(nb=2, online=np.arange(40) > 5)), {}),
        "authority": (nom, _result(telem=_telem(scale=0.5)), {}),
        "authority_edge": (nom, _result(telem=_telem(scale=0.86)), {}),
        "lag": (nom, _result(telem=_telem(n=120, lag=20)), {}),
        "cycle": (nom, _result(fz=1800.0), {}),
        "roll": (nom, _result(roll=0.03), {}),
        "flat_nom": (_result(fz=0.0, roll=0.0), _result(fz=0.0), {}),
        "failed": (_result(ok=False), _result(), {}),
        "ghost_held": (nom, _result(), {"gc": _Gc()}),
        "ghost_drop": (nom, _result(fy=2000.0), {"gc": _Gc()}),
        "ghost_cross": (_result(fy=2000.0), _result(fy=2900.0), {"gc": _Gc(), "corner": "RL"}),
        "ghost_failed": (nom, _result(ok=False), {"gc": _Gc()}),
        "order": (nom, _result(telem=_telem(scale=1.0, cmd=np.full(40, 900.0))), {"gc": _Gc(0.5)}),
        "base_path": (nom, _result(fy=1100.0), {"gc": _Gc(), "base_path": fb.seed_paths()[0]
                                                  if isinstance(fb.seed_paths(), list) else None,
                                                  "designated_fuse_key": "branch_conn"}),
        "ab": (nom, _result(telem=_telem(e=40.0)), {"ab_design": ab}),
        "ab_tight": (nom, _result(telem=_telem(e=4000.0)), {"ab_design": ab_tight, "laps_per_session_est": 0.5}),
        "ab_starved": (nom, _result(telem=_telem(e=75.0)), {"ab_design": es.ABDesign(1.0, 0.5)}),
        "ab_half": (nom, _result(telem=_telem(e=75.0)), {"ab_design": es.ABDesign(1.0, 0.5),
                                                          "laps_per_session_est": 0.5}),
        "bill": (nom, _result(telem=_telem(e=600.0)), {"laps_per_session_est": 10.0}),
        "thresh": (nom, _result(telem=_telem(scale=0.9, lag=1)),
                   {"thresholds": sf.ForgeThresholds(0.95, 0.5, 1.01, 1.01, 0.0, 3.0), "gc": _Gc()}),
        "abort": (nom, _result(bus=None), {}),
    }
    cases["abort"][1].bus = None
    for k, (a, b, kw) in cases.items():
        lint = sf.forge_lint(a, b, **kw)
        res[k] = [lint.verdict, lint.flags, lint.findings, lint.note, lint.session_before, lint.session_after,
                  lint.summary(), sf.render_forge_md(lint),
                  None if lint.ghost_degraded is None else lint.ghost_degraded.summary(),
                  None if lint.path_audit_degraded is None else
                  [getattr(lint.path_audit_degraded, "leader_key", ""), lint.path_audit_degraded.verdict]]
        whole[k] = {f: getattr(lint, f) for f in vars(lint) if f not in ("ghost_nominal", "ghost_degraded")}
    check("forge_lint", res)
    check("forge_lint_whole", whole)


def test_path_elements():
    res = {}
    r = _result()
    res["elec"] = [sf.electrical_path_elements(r), sf.electrical_path_elements(r, EvidenceGrade.ESTIMATE)]
    z = _result()
    z.elec.i_act = np.zeros((0, 2))
    res["elec_empty"] = sf.electrical_path_elements(z)
    a = gt.ghost_audit_transient(_Gc(), r.mech, corner="FL", n_samples=3)
    paths = fb.seed_paths()
    base = paths[0] if isinstance(paths, list) else next(iter(paths.values()))
    member = types.SimpleNamespace(instants=[types.SimpleNamespace(margins={"UF": {"fos": 0.0001}, "XX": {}}),
                                             types.SimpleNamespace(margins={"UF": {"fos": 3.0},
                                                                            "LR": {"fos": float("nan")}})])
    res["mech"] = [sf._mech_elements_from_ghost(a, None), sf._mech_elements_from_ghost(a, base),
                   sf._mech_elements_from_ghost(member, None)]
    check("forge_paths", res)


class _GcGate(_Gc):
    """Ghost corner whose members read infinitely strong under light lateral load."""

    def solve_instant(self, load, t=0.0, tire=None, fos_limit=1.5):
        g = super().solve_instant(load, t, tire, fos_limit)
        if abs(load.Fy) < 1500.0:
            g.margins = {k: dict(v, fos=float("inf")) for k, v in g.margins.items()}
            g.min_fos = float("inf")
        return g


def test_characterized_edges(monkeypatch):
    """Telemetry, solver wiring and lint branches the main suites never reach."""
    import json
    from tests._golden import plain
    res = {}
    t = _telem(lag=3)
    t.t = t.t + 0.5
    t.online = np.arange(40) % 7 != 0
    two = _telem(n=2)
    two.online = np.array([False, True])
    e = sf._empty_telemetry()
    res["telem"] = [t.summary(), two.summary(), e.summary(),
                    [(f, getattr(e, f).shape, getattr(e, f).dtype.name) for f in vars(e) if f != "n_brownouts"]]
    hollow = _result()
    hollow.mech.Fz = np.zeros((0, 4))
    hollow.mech.roll = np.zeros(0)
    res["hollow"] = [hollow.load_cycle_ptp_N(), hollow.roll_peak_deg()]
    sim = sf.MechatronicSolver("VEH", params=tr.TransientParams())
    res["veh"] = sim.veh
    seen = []

    class Rec(sf.MechatronicSolver):
        def __init__(self, *a, **k):
            seen.append([list(a), sorted(k), None if k.get("params") is None else k["params"].dt])
            super().__init__(None, params=k.get("params"))
    monkeypatch.setattr(sf, "MechatronicSolver", Rec)
    p = tr.TransientParams()
    p.dt = 0.002
    out = sf.run_simulforge("CAR", params=p, degradation="no_such_preset", **SHORT)
    monkeypatch.undo()
    res["wiring"] = [seen, out.degradation, out.meta.get("dt")]
    drv, road, t_end, u0, _ = tr.step_steer_maneuver(**SHORT)
    res["trickle"] = sf.MechatronicSolver(None).run_coupled(t_end, sf.BusParams(),
                                                            sf.ActuatorParams(duty_max=0.02), driver=drv,
                                                            road=road, u0=u0).elec.summary()
    res["weak_k"] = sf.ActuatorParams(Kt=0.01, gear=1.0, eff=0.5).current_for_torque(3.0)
    res["neg_auth"] = sf.Degradation(authority_scale=-0.5).apply(sf.BusParams(), sf.ActuatorParams())[1]
    uf = fb.PathElement(key="UF", label="Upper front wishbone", fos=2.0, grade=EvidenceGrade.MEASURED,
                        severity=fb.Severity.S3_FORBIDDEN, replace_cost_usd=321.0, downtime_days=4.5)
    base = fb.OverloadPath(key="b", label="b", story="s", elements=[uf], designated_fuse_key="UF")
    nom = _result()
    fr = _result(fz=1600.0)
    fr.mech.Fz[:, 1] *= 1.6
    nan_roll = _result()
    nan_roll.mech.roll = np.zeros(0)
    f3a, f3b = _result(ok=False), _result(ok=False)
    f3a.warnings, f3b.warnings = ["wa1", "wa2"], ["wb1", "wb2"]
    cases = {
        "base_match": (nom, _result(fy=1100.0), {"gc": _Gc(), "base_path": base}),
        "cycle_fr": (nom, fr, {}),
        "nan_roll": (nan_roll, _result(fz=1800.0), {}),
        "three_warn": (f3a, f3b, {}),
        "one_brownout": (nom, _result(telem=_telem(nb=1, online=np.arange(40) > 3)), {}),
        "auth_scaled": (_result(telem=_telem(scale=0.8)), _result(telem=_telem(scale=0.75)), {}),
        "lag_ratio": (_result(telem=_telem(n=120, lag=10)), _result(telem=_telem(n=120, lag=20)), {}),
        "inf_nominal": (_result(fy=100.0), _result(fy=3000.0), {"gc": _GcGate()}),
    }
    whole = {}
    for k, (a, b, kw) in cases.items():
        lint = sf.forge_lint(a, b, **kw)
        whole[k] = {f: getattr(lint, f) for f in vars(lint) if f not in ("ghost_nominal", "ghost_degraded")}
        whole[k]["md"] = sf.render_forge_md(lint)
    odd = sf.forge_lint(nom, _result())
    odd.findings.append(sf.ForgeFinding("ODD", "purple", "an unclassified finding", {}))
    res["odd_md"] = sf.render_forge_md(odd)
    res["lint"] = whole
    res["elements"] = sf._mech_elements_from_ghost(gt.ghost_audit_transient(_Gc(), nom.mech, corner="FL",
                                                                             n_samples=3), base)
    check("forge_edges", json.loads(json.dumps(plain(res))))


def test_characterized_edges_more():
    """Second batch: rarer lint branches and coupled runs at the limits of their inputs."""
    import json
    from tests._golden import plain
    nom = _result()
    rr = _result(fz=1600.0)
    rr.mech.Fz[:, 3] *= 1.6
    cases = {
        "cycle_rr": (nom, rr, {}),
        "inf_nominal_held": (_result(fy=100.0), _result(fy=1600.0), {"gc": _GcGate()}),
        "crossed_fr": (_result(fy=2000.0), _result(fy=2900.0), {"gc": _Gc()}),
        "no_telemetry": (nom, _result(ok=False, telem=sf._empty_telemetry()), {}),
        "reserve": (nom, _result(telem=_telem(e=30.0), bus=sf.BusParams(reserve_kwh=1.5)), {}),
        "bill_partial": (nom, _result(telem=_telem(e=30.0)), {}),
    }
    res = {}
    for k, (a, b, kw) in cases.items():
        lint = sf.forge_lint(a, b, **kw)
        res[k] = {f: getattr(lint, f) for f in vars(lint) if f not in ("ghost_nominal", "ghost_degraded")}
        res[k]["md"] = sf.render_forge_md(lint)
    drv, road, t_end, u0, _ = tr.step_steer_maneuver(**SHORT)
    bus, act = sf.BusParams(), sf.ActuatorParams()
    p2 = tr.TransientParams()
    p2.substeps = 2
    runs = {"tiny_torque": (None, bus, sf.ActuatorParams(M_max_Nm=0.5), t_end, u0),
            "dead_bus": (None, sf.BusParams(V_oc=0.0, V_brownout=0.0), act, t_end, u0),
            "trickle": (None, bus, sf.ActuatorParams(duty_max=0.02), t_end, u0),
            "substeps": (p2, bus, act, t_end, u0),
            "zero_time": (None, bus, act, 0.0, u0),
            "bad_u0": (None, bus, act, t_end, "fast")}
    for k, (params, b, a, te, u) in runs.items():
        res["run_" + k] = sf.MechatronicSolver(None, params=params).run_coupled(te, b, a, driver=drv, road=road, u0=u)
    ldrv, lroad, lt_end, lu0, _ = tr.step_steer_maneuver(steer_deg=-4.0, **SHORT)
    for k, a in {"left_tiny_torque": sf.ActuatorParams(M_max_Nm=0.5),
                 "left_trickle": sf.ActuatorParams(duty_max=0.02)}.items():
        res["run_" + k] = sf.MechatronicSolver(None).run_coupled(lt_end, bus, a, driver=ldrv, road=lroad, u0=lu0)
    check("forge_edges_more", json.loads(json.dumps(plain(res))))
