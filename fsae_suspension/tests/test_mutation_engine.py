"""Characterization suite for powertrain/engine.py (mutation gate); see tests/_golden.py."""
import types

import numpy as np
import pytest

from powertrain import engine as en
from suspension.interfaces import IntegrationLedger, SubsystemInterface
from tests._golden import check

def _motors():
    # built per test, not at import, so a mutant in MotorCurve.from_peak is exercised
    return {
        "plain": en.MotorCurve.from_peak(180.0, 80.0, 6500.0),
        "kt": en.MotorCurve.from_peak(180.0, 80.0, 6500.0, kt_nm_per_a=0.5, i_max_a=300.0, voltage_v=350.0,
                                      eta_motor=0.85),
        "manual": en.MotorCurve(rpm=np.array([0.0, 1000.0, 3000.0]), torque_nm=np.array([100.0, 120.0, 60.0]),
                                redline_rpm=2500.0, kt_nm_per_a=2.0, i_max_a=50.0),
    }


def test_motor_curves():
    res = {}
    rpm = np.array([-10.0, 0.0, 500.0, 2500.0, 2600.0, 3000.0, 4000.0, 6500.0, 7000.0])
    for k, m in _motors().items():
        res[k] = {"rpm": m.rpm[::37], "tq": m.torque_nm[::37], "at": m.torque_at(rpm),
                  "scalar": float(m.torque_at(1200.0))}
    check("engine_motor", res)


def _params(**kw):
    base = dict(motor=_motors()["kt"], tire_radius_m=0.22, gear_ratio=4.0, mass_kg=260.0, drivetrain_eff=0.9,
                mu=1.3, driven_weight_frac=0.55, cg_height_m=0.3, wheelbase_m=1.6, cda_m2=1.2, crr=0.02,
                rot_inertia_factor=1.08)
    base.update(kw)
    return en.DrivetrainParams(**base)


def test_launch_and_gear_sweep():
    res = {}
    cases = {"kt": _params(), "plain": _params(motor=_motors()["plain"]), "manual": _params(motor=_motors()["manual"]),
             "zero_gear": _params(gear_ratio=0.0), "grippy": _params(mu=3.0, gear_ratio=2.0),
             "default": en.DrivetrainParams(motor=_motors()["plain"])}
    for k, p in cases.items():
        r = en.simulate_launch(p, distance_m=30.0, dt=0.05, t_max_s=6.0)
        res[k] = {"r": r, "t": r.time_to_distance_s, "v": r.v_final_kmh}
    res["v0"] = en.simulate_launch(_params(), distance_m=20.0, v0_ms=5.0, dt=0.1)
    res["capped"] = en.simulate_launch(_params(), distance_m=500.0, dt=0.1, t_max_s=1.0)
    sw = en.optimize_gear_ratio(_params(), ratios=[2.0, 3.5, 5.0, 9.0], distance_m=25.0, dt=1e-3,
                                keep_traces=True)
    res["sweep"] = {"s": sw, "table": sw.as_table()}
    dnf = en.optimize_gear_ratio(_params(motor=_motors()["manual"]), ratios=[0.5, 0.0], distance_m=400.0, dt=0.5)
    res["dnf"] = {"s": dnf, "table": dnf.as_table()}
    tie = en.optimize_gear_ratio(_params(), ratios=[3.0, 3.0], distance_m=10.0, dt=1e-3)
    res["tie"] = [tie.best_ratio, tie.runs]
    dflt = en.optimize_gear_ratio(_params(), distance_m=8.0, dt=0.1)
    res["dflt"] = [dflt.ratios, dflt.times_s, dflt.best_ratio]
    near = en.optimize_gear_ratio(_params(), ratios=[4.0], distance_m=10.0, dt=0.05)
    res["near"] = [near.times_s, near.best.x_m[-1]]
    check("engine_launch", res)


def test_cooling_network():
    res = {}
    fl, w = en.CoolantProps(), en.CoolantProps.water()
    seg = en.PipeSegment(d_m=0.02, length_m=1.5, k_minor=1.2)
    seg_r = en.PipeSegment(d_m=0.02, length_m=1.5, roughness_m=1e-4)
    res["seg"] = [s.audit(q, f) for s in (seg, seg_r) for f in (fl, w)
                  for q in (0.0, 1e-8, 1e-7, 2e-6, 4e-5, 3e-4, -3e-4)]
    res["area"] = seg.area_m2
    ys = list(en.STANDARD_Y_BRANCHES.items()) + [("custom", en.YBranch("c", 0.03, 0.02, angle_deg=60.0,
                                                                        k0_run=0.2, k0_branch=1.4))]
    res["y"] = {k: [y._k_branch, y.audit(4e-4, 0.3, fl).as_dict(), y.audit(4e-4, -1.0, w).as_dict(),
                    y.audit(4e-4, 2.0, fl).as_dict()] for k, y in ys}
    pump = en.PumpCurve(dp0_pa=4e4, q_max_m3s=6e-4)
    res["pump"] = [pump.dp(q) for q in (0.0, 3e-4, 6e-4, 9e-4)] + [en.PumpCurve().dp(1e-4)]
    rad = en.Radiator(ua_w_per_k=300.0, frontal_area_m2=0.05, air_capture_eff=0.6, fan_floor_ms=3.0,
                      dp_coolant_k=3.0, d_port_m=0.025)
    vv = np.array([0.0, 4.0, 10.0, 25.0])
    res["rad"] = [rad.c_air(vv), rad.q_reject(90.0, 30.0, vv, 300.0), rad.q_reject(90.0, 30.0, vv, 5000.0),
                  en.Radiator().c_air(1.0), en.Radiator().q_reject(80.0, 20.0, 12.0, 1e4)]
    nets = {"default": en.CoolingNetwork(),
            "custom": en.CoolingNetwork(fluid=w, pump=pump, radiator=rad, segments=(seg,),
                                        junctions=(ys[-1][1], en.STANDARD_Y_BRANCHES["29-12"]),
                                        leg_k={"c": (5.0, 0.5), "29mm-12mm": (0.0, 40.0)}),
            "bare": en.CoolingNetwork(segments=(), junctions=())}
    for k, n in nets.items():
        res[k] = {"op": n.operating_point(), "audit": n.audit(), "fixed": n.audit(2e-4, fixed_split=0.25),
                  "dp": [n.system_dp(q) for q in (1e-5, 2e-4)],
                  "split": [n._junction_split(j, q) for j in n.junctions for q in (1e-5, 3e-4)]}
    check("engine_cooling", res)


def test_lap_thermal():
    res = {}
    drive = en.simulate_launch(_params(), distance_m=30.0, dt=0.1)
    net = en.CoolingNetwork()
    th = en.simulate_lap_thermal(net, drive=drive, n_laps=2, dt=0.2)
    res["drive"] = {"r": th, "peak": th.t_peak_c}
    res["drive_q"] = en.simulate_lap_thermal(net, drive=drive, q_gen_w=np.full(drive.t_s.size, 1500.0),
                                             n_laps=1, t_amb_c=25.0, t0_c=60.0, coolant_volume_l=3.0,
                                             wetted_metal_kg=2.0, dt=0.25)
    t = np.linspace(1.0, 11.0, 6)
    res["explicit"] = en.simulate_lap_thermal(net, t_s=t, v_car_ms=[0, 5, 10, 15, 10, 5], n_laps=2, dt=0.5)
    res["explicit_q"] = en.simulate_lap_thermal(net, t_s=t, v_car_ms=[0, 5, 10, 15, 10, 5],
                                                q_gen_w=[9e3, 1e3, 0, 0, 2e3, 5e3], n_laps=1, dt=0.5)
    for kw in ({}, {"t_s": t}, {"v_car_ms": t}):
        with pytest.raises(ValueError, match="^need `drive` or explicit"):
            en.simulate_lap_thermal(net, **kw)
    check("engine_thermal", res)


def test_ledger_helpers():
    res = {}
    led = IntegrationLedger()
    led.set(SubsystemInterface(name="a", mass_kg=40.0))
    led.set(SubsystemInterface(name="b", mass_kg=None))
    led.set(SubsystemInterface(name="c", mass_kg=12.5))
    sources = {"none": None, "ledger": led, "dict": {"x": {"mass_kg": 10.0}, "y": {"mass_kg": "bad"},
                                                      "z": {"mass_kg": None}, "w": {}},
               "nested": {"interfaces": {"x": {"mass_kg": 5.0}}, "mass": 1},
               "subs": {"subsystems": {"x": types.SimpleNamespace(mass_kg=7.0)}},
               "items": {"items": {"x": {"mass_kg": 2.0}}, "interfaces": 5},
               "list": [1, 2], "asdict_list": types.SimpleNamespace(as_dict=lambda: [1]),
               "zero": {"x": {"mass_kg": 0.0}}, "tiny": {"x": {"mass_kg": 1e-7}}, "objs": {"x": object()},
               "neg": {"x": {"mass_kg": -3.0}, "y": {"mass_kg": 2.0}}}
    for k, s in sources.items():
        res[k] = [en.total_mass_from_ledger(s), en.total_mass_from_ledger(s, driver_kg=70.0, fallback_kg=111.0)]
    res["iface"] = [en._iface_get(None, "a"), en._iface_get(None, "a", 3), en._iface_get({"a": None}, "a", 4),
                    en._iface_get({"a": 2}, "a", 4), en._iface_get(types.SimpleNamespace(a=None), "a", 5),
                    en._iface_get(types.SimpleNamespace(a=6), "a", 5), en._iface_get({}, "a")]
    p = _params(mass_kg=150.0)
    res["with"] = [p.with_ledger_mass(led).mass_kg, p.with_ledger_mass(None, driver_kg=0.0).mass_kg,
                   p.with_ledger_mass({"x": {"mass_kg": 100.0}}, driver_kg=60.0).mass_kg]
    drive = en.simulate_launch(_params(), distance_m=15.0, dt=0.1)
    th = en.simulate_lap_thermal(en.CoolingNetwork(), drive=drive, n_laps=1, dt=0.5)
    l2 = IntegrationLedger()
    l2.set(SubsystemInterface(name="powertrain", mass_kg=30.0))
    out = en.publish_to_ledger(l2, drive=drive, thermal=th, updated_by="t")
    res["pub"] = [out is l2, l2.interfaces["powertrain"], l2.interfaces["cooling"]]
    l3 = IntegrationLedger()
    en.publish_to_ledger(l3, drive=drive)
    res["pub_drive"] = [sorted(l3.interfaces), l3.interfaces["powertrain"]]
    l4 = IntegrationLedger()
    en.publish_to_ledger(l4, thermal=th)
    res["pub_th"] = [sorted(l4.interfaces), l4.interfaces["cooling"]]

    class Put:
        def __init__(self):
            self.d = {}

        def get(self, k):
            return self.d.get(k)

        def put(self, it):
            self.d[it.name] = it

    class NoSet:
        def get(self, k):
            return None

    pl = Put()
    en.publish_to_ledger(pl, drive=drive, thermal=th)
    res["put"] = [pl.d["powertrain"], pl.d["cooling"]]
    ns = NoSet()
    res["noset"] = en.publish_to_ledger(ns, drive=drive, thermal=th) is ns
    res["none"] = [en.publish_to_ledger(None, drive=drive), en.publish_to_ledger(5, drive=drive)]
    check("engine_ledger", res)


def test_publish_without_interfaces(monkeypatch):
    import sys
    monkeypatch.setitem(sys.modules, "suspension.interfaces", None)
    monkeypatch.delattr(sys.modules["suspension"], "interfaces")
    led = types.SimpleNamespace(get=lambda k: (_ for _ in ()).throw(AssertionError("touched")))
    assert en.publish_to_ledger(led, drive=object()) is led


def test_characterized_defaults_and_edges():
    res = {}
    long = en.MotorCurve(rpm=np.array([0.0, 1000.0, 3000.0]), torque_nm=np.array([100.0, 120.0, 60.0]),
                         redline_rpm=4000.0)
    res["long"] = long.torque_at(np.array([-5.0, 2000.0, 3500.0, 4500.0]))
    ld = en.simulate_launch(_params())
    res["launch_dflt"] = [ld.time_to_distance_s, ld.v_final_kmh, ld.x_m[-1], ld.t_s.size, ld.t_s[-1]]
    res["half_gear"] = en.simulate_launch(_params(gear_ratio=0.5), distance_m=5.0, dt=0.05)
    sw = en.optimize_gear_ratio(_params())
    res["sweep_dflt"] = [sw.ratios, sw.times_s, sw.best_ratio]
    drive = en.simulate_launch(_params(), distance_m=30.0, dt=0.1)
    res["thermal_dflt"] = en.simulate_lap_thermal(en.CoolingNetwork(), drive=drive)
    res["thermal_water"] = en.simulate_lap_thermal(en.CoolingNetwork(fluid=en.CoolantProps.water()), drive=drive,
                                                   n_laps=1, dt=0.5)
    res["mass_bad_first"] = en.total_mass_from_ledger({"a": {"mass_kg": "bad"}, "b": {"mass_kg": 5.0}})
    y = en.YBranch("c", 0.03, 0.02, angle_deg=60.0, k0_run=3.0, k0_branch=0.2)
    net = en.CoolingNetwork(junctions=(y,), leg_k={"other": (3.0, 3.0)})
    res["split_missing_leg"] = [net._junction_split(y, q) for q in (1e-5, 3e-4)]
    check("engine_edges", res)
