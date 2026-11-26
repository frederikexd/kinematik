"""suspension/pt_integration.py against tests/golden: the assumption checker over a corpus that tries every trigger
phrase of every rule on its own and every numeric branch on both sides, the number extractor on every unit spelling,
and the gear sweep, sprocket, cooling point, spec sheet, motor envelope, myth checks and DFMEA rows for set inputs.
Regenerate the golden file with KINEMATIK_REGOLD=1 only after reading the diff."""
import dataclasses
import enum
import json
import math
import os
from pathlib import Path
from types import SimpleNamespace

import pytest

from suspension import pt_integration as pti
from suspension.laptime import MotorMap

GOLD = Path(__file__).parent / "golden" / "pt_integration.json"

# every trigger phrase, group by group, rule by rule (copied from check_assumption: each is meant to fire alone)
R1 = (["cap", "limit", "restrict", "max", "ceiling", "cannot exceed", "can't exceed", "can not exceed"],
      ["rpm", "redline", "rev", "speed"], ["kw", "kilowatt", "power", "watt"])
R2 = (["continuous", "sustained", "steady"], ["kw", "power"])
R3 = (["base speed", "corner speed", "peak power", "max power", "full power"], ["rpm", "redline"])
R4 = (["top speed", "max speed", "maximum speed", "km/h", "kmh", "kph", "fast", "velocity"],)
R6 = (["80 kw", "80kw", "fsae cap", "fsae limit", "rule", "compliant", "comply", "legal", "within the cap",
       "under the cap"],)
R7 = (["more power", "higher power", "increase power", "bigger motor", "more kw", "higher kw"],
      ["faster", "more speed", "higher speed", "quicker", "better acceleration", "higher top speed"])


def _one_at_a_time(groups, glue=" / "):
    out = []
    for i, g in enumerate(groups):
        for p in g:
            out.append(glue.join(p if j == i else other[0] for j, other in enumerate(groups)))
    return out


CORPUS = (
    _one_at_a_time(R1) + _one_at_a_time(R2) + _one_at_a_time(R3) + _one_at_a_time(R4) + _one_at_a_time(R6)
    + _one_at_a_time(R7)
    + [
        # rule 1: the right redline for the wrong reason, a wrong one, and none
        "the 80 kW cap limits us to 7000 rpm", "the 80 kW cap limits us to 7199 rpm",
        "the 80 kW cap limits us to 7200 rpm", "the 80 kW cap limits us to 6801 rpm", "power cap limits the redline",
        "the 80 kW cap limits us to 5000 rpm",
        # rule 2
        "continuous power is 90 kW", "continuous power is 80.6 kW", "continuous power is 80.4 kW",
        "continuous power is 56 kW", "continuous power is 59.9 kW", "continuous power is 60 kW",
        "continuous power is 52.1 kW", "continuous power is 52 kW", "continuous power is 30 kW",
        "continuous power",
        # rule 3
        "peak power at redline 7000 rpm", "peak power at max rpm", "peak power at maximum rpm",
        "base speed is 3322 rpm", "base speed is 3150 rpm", "base speed is 3500 rpm", "base speed is 2000 rpm",
        "base speed rpm",
        # rule 4 (needs a final drive): speeds either side of 5 km/h, an rpm claim, nothing
        "top speed is 172 km/h", "top speed is 168 km/h", "top speed is 160 km/h", "top speed is 200 km/h",
        "6000 rpm top speed", "top speed",
        # rule 5: torque at an rpm, inside and outside 8 %, below and above base speed
        "torque at 2000 rpm is 230 Nm", "torque at 2000 rpm is 250 Nm", "torque at 2000 rpm is 150 Nm",
        "torque at 5000 rpm is 150 Nm", "torque at 5000 rpm is 230 Nm", "torque at 5000 rpm is 140 N·m",
        "torque is high",
        # boundaries and the phrases only reachable on their own
        "continuous power is 80.5 kW", "top speed is 166.4 km/h", "top speed is 177.4 km/h", "base speed is 3505 rpm",
        "base speed is 3150 rpm", "the motor makes 200 Nm", "base speed equals max rpm", "base speed at maximum rpm",
        "full power at 5000 rpm", "more power is good", "it is quicker now", "torque at 3321 rpm is 230 Nm", "torque at 9000 rpm is 10 Nm", "the 80 kW CAP on RPM",
        # fall-through
        "the car is red", "",
    ])

NUMBERS = ["7000 rpm", "7,000 RPM", "6500 rev", "6500 revs", "6500 revolutions per minute", "6500 rev per min",
           "80 kW", "80 KW", "80 kilowatt", "230 N·m", "230 N⋅m", "230 N*m", "230 Nm", "230 Nm", "230 newton meter",
           "230 newton-metre", "230 Newton metre", "120 km/h", "120 kmh", "120 kph", "120 km per h", "35 %",
           "3.5:1", "3.5 : 1", "1.2.3 rpm", "no numbers", "6000 rpm and 80 kW and 200 Nm and 100 km/h and 50% and 4:1"]


def _plain(x):
    if dataclasses.is_dataclass(x) and not isinstance(x, type):
        return {f.name: _plain(getattr(x, f.name)) for f in dataclasses.fields(x)}
    if isinstance(x, enum.Enum):
        return x.value
    if isinstance(x, dict):
        return {str(k): _plain(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_plain(v) for v in x]
    if hasattr(x, "tolist"):
        return _plain(x.tolist())
    if isinstance(x, float):
        return None if math.isnan(x) else float(f"{x:.10g}")
    return x


def _map():
    return MotorMap.from_peak(230.0, 80.0, 6000.0, final_drive=3.5, wheel_radius_m=0.20)


def _everything():
    ok, over = pti.motor_envelope(230.0, 80.0, 7000.0), pti.motor_envelope(300.0, 120.0, 7000.0)
    out = {"assumptions": [], "numbers": {s: pti._extract_numbers(s) for s in NUMBERS}}
    for text in CORPUS:
        for name, env in (("ok", ok), ("over", over)):
            for fd in (None, 3.5):
                r = pti.check_assumption(text, env, gear_final_drive=fd)
                out["assumptions"].append([text, name, fd, r.verdict, r.matched_rule, r.explanation,
                                           _plain(r.user_values)])
    r = pti.check_assumption("top speed is 120 km/h", ok, gear_final_drive=3.5, wheel_r_m=0.25)
    out["assumptions"].append(["wheel 0.25", r.verdict, r.matched_rule, r.explanation])
    out["envelopes"] = {}
    for key, args, kw in (("ok", (230.0, 80.0, 7000.0), {}), ("over", (300.0, 120.0, 7000.0), {}),
                          ("floors", (0.0, 0.0, 10.0), dict(continuous_frac=0.0, n=16)),
                          ("frac", (230.0, 80.0, 7000.0), dict(continuous_frac=2.0, rule_cap_kw=60.0, n=24))):
        env = pti.motor_envelope(*args, **kw)
        out["envelopes"][key] = {"env": _plain(env), "redline_kw": env.power_at_redline_kw(),
                                 "explanation": env.explanation(), "note": env.peak_vs_continuous_note(),
                                 "myths": _plain(pti.power_rpm_myth_checks(env)),
                                 "myths_fd": _plain(pti.power_rpm_myth_checks(env, gear_final_drive=4.0,
                                                                              wheel_r_m=0.21))}
    out["sweeps"] = {}
    for obj in pti.GearObjective:
        s = pti.GearRatioSolver(_map(), mass_kg=300.0, wheel_r_m=0.20).sweep([0, -1, 2.5, 3.0, 3.5, 4.5, 6.0, 9.0],
                                                                              obj)
        out["sweeps"][obj.value] = {"label": obj.label(), "result": _plain(s), "table": _plain(s.as_table())}
    s = pti.GearRatioSolver(_map(), mass_kg=250.0, wheel_r_m=0.23, drivetrain_eff=0.8, mu=1.1, cda=0.6, crr=0.03,
                            rear_frac=0.6).sweep([3.0, 4.0, 5.0])
    out["sweeps"]["custom"] = {"result": _plain(s), "table": _plain(s.as_table())}
    out["sprockets"] = [_plain(pti.sprocket_design(*a)) for a in
                        ((3.5, 230.0), (3.5, 230.0, "#40 (1/2\")", 11), (1.01, 230.0, "#35 (3/8\")", 5),
                         (4.2, 180.0, "unknown chain", 13))]
    fan = pti.FanCurve.from_points("f", [(0, 300), (1500, 180), (900, 250), (2500, 0)])
    out["cooling"] = [_plain(pti.cooling_operating_point(f, k, **kw)) for f, k, kw in (
        (pti.FanCurve.spal_default(), 1e-4, dict(heat_to_reject_w=3000.0)),
        (fan, 5e-5, dict(heat_to_reject_w=8000.0, air_delta_t_c=15.0, air_density=1.1, air_cp=1000.0)),
        (fan, 1e-2, {}), (fan, 0.0, dict(heat_to_reject_w=100.0)))]
    out["fan"] = [fan.pressure_at(q) for q in (-10.0, 0.0, 450.0, 900.0, 2000.0, 9000.0)]
    out["system_k"] = [pti.system_k_from_point(1000.0, 150.0), pti.system_k_from_point(0.0, 150.0)]
    out["heat"] = [pti.estimate_motor_heat_w(80.0), pti.estimate_motor_heat_w(80.0, 0.5, 2.0),
                   pti.estimate_motor_heat_w(80.0, 1.5, 0.0)]
    out["driveline"] = [pti.driveline_peak_torque_nm(3.5, 230.0), pti.driveline_peak_torque_nm(3.5, 230.0, 0.9)]
    out["spec"] = [pti.powertrain_spec_sheet(architecture="single motor", power_kw=80.0, peak_torque_nm=230.0,
                                             hv_voltage_v=588.0, pack_kwh=6.5, final_drive=3.5),
                   pti.powertrain_spec_sheet(architecture="hub", power_kw=79.6, peak_torque_nm=229.5,
                                             hv_voltage_v=400.4, pack_kwh=7.125, final_drive=4.444,
                                             driven_teeth=49, motor_teeth=11, chain_tension_n=8123.4,
                                             driveline_torque_nm=780.6, motor_mass_kg=21.25, heat_reject_w=4321.5,
                                             cooling_flow_cms=0.1234, is_estimate=False),
                   pti.powertrain_spec_sheet(architecture="x", power_kw=1, peak_torque_nm=1, hv_voltage_v=1,
                                             pack_kwh=1, final_drive=1, driven_teeth=40)]
    sp = pti.sprocket_design(3.5, 230.0)
    cp = pti.cooling_operating_point(pti.FanCurve.spal_default(), 1e-4, heat_to_reject_w=3000.0)
    out["dfmea"] = [_plain(pti.dfmea_rows_from_analysis()),
                    _plain(pti.dfmea_rows_from_analysis(sprocket=sp, cooling=cp, output_torque_nm=780.0,
                                                        mount_load_n=5000.0, owner="FT")),
                    _plain(pti.dfmea_rows_from_analysis(
                        cooling=pti.cooling_operating_point(fan, 1e-2, heat_to_reject_w=20000.0)))]
    out["dfmea_partial"] = [_plain(pti.dfmea_rows_from_analysis(output_torque_nm=780.0)),
                            _plain(pti.dfmea_rows_from_analysis(mount_load_n=5000.0))]
    stall = pti.FanCurve.from_points("s", [(100, 100), (1000, 0)])
    near = pti.cooling_operating_point(stall, 0.0099)
    base = pti.cooling_operating_point(fan, 5e-5)
    out["cooling_edges"] = [_plain(pti.cooling_operating_point(stall, 0.01)), _plain(near),
                            _plain(pti.cooling_operating_point(fan, 5e-5, heat_to_reject_w=base.cooling_capacity_w)),
                            _plain(pti.cooling_operating_point(fan, 5e-5,
                                                               heat_to_reject_w=base.cooling_capacity_w - 0.5)),
                            _plain(pti.cooling_operating_point(fan, 5e-5, heat_to_reject_w=-50.0))]
    toy = SimpleNamespace(_rpm=[1000.0, 2000.0, 6000.0], _t=[150.0, 250.0, 100.0])
    out["wheel_force"] = [pti._motor_wheel_force(toy, v, fd, r, eff) for v, fd, r, eff in (
        (0.0, 4.0, 0.2, 0.9), (0.5, 4.0, 0.2, 0.9), (3.0, 4.0, 0.2, 0.9), (10.0, 4.0, 0.2, 0.9),
        (40.0, 4.0, 0.2, 0.9), (60.0, 4.0, 0.2, 0.9), (5.0, 4.0, 0.01, 0.01))]
    out["redline_speed"] = [pti._speed_at_redline_kmh(toy, fd, 0.2) for fd in (0.5, 3.0, 0.0)]
    out["accel"] = [pti._accel_0_75(_map(), 3.5, 0.2, 0.9, 300.0, 1.4, 1.1, 0.018),
                    pti._accel_0_75(_map(), 3.5, 0.2, 0.9, 300.0, 1.4, 1.1, 0.018, rear_frac=0.7, dist_m=50.0,
                                    cg_height_m=1.2, wheelbase_m=1.55),
                    pti._accel_0_75(_map(), 3.5, 0.2, 0.9, 300.0, 1.4, 1.1, 0.018, cg_height_m=0.0,
                                    wheelbase_m=0.0),
                    pti._accel_0_75(SimpleNamespace(_rpm=[0.0, 100.0], _t=[1.0, 0.0]), 1.0, 0.2, 0.5, 300.0,
                                    1.4, 1.1, 0.018),
                    pti._accel_0_75(_map(), 3.5, 0.2, 0.9, 1.5, 1.4, 0.0, 0.0, dist_m=5.0),
                    pti._accel_0_75(_map(), 3.5, 0.2, 0.9, 300.0, 1.4, 1.1, 0.018, cg_height_m=1.2, wheelbase_m=0.2),
                    pti._accel_0_75(_map(), 3.5, 0.2, 0.9, 300.0, 1.4, 1.1, 0.018, cg_height_m=0.3, wheelbase_m=0.5)]
    toy2 = SimpleNamespace(_rpm=[50.0, 2000.0, 6000.0], _t=[100.0, 250.0, 100.0])
    out["wheel_force_low"] = [pti._motor_wheel_force(toy2, v, 4.0, 0.2, 0.9) for v in (0.0, 0.5, 1.0)]
    out["envelope_capped"] = _plain(pti.motor_envelope(50.0, 80.0, 7000.0))
    out["envelope_just_over"] = _plain(pti.motor_envelope(230.0, 80.5, 7000.0))
    sv = pti.GearRatioSolver(_map(), mass_kg=300.0, wheel_r_m=0.20)
    out["sweep_edges"] = [_plain(sv.sweep([0.0, -2.0])), _plain(sv.sweep([3.456])),
                          _plain(sv.sweep([0.5, 1.2, 2.3, 3.456]).as_table()),
                          _plain(sv.sweep([0.5, 1.2, 2.3, 3.456]))]
    out["accel_cap_binding"] = pti._accel_0_75(_map(), 3.5, 0.2, 0.9, 100.0, 1.4, 1.1, 0.018, cg_height_m=0.4,
                                               wheelbase_m=0.5)
    out["myths_default_wheel"] = _plain(pti.power_rpm_myth_checks(pti.motor_envelope(230.0, 80.0, 7000.0),
                                                                  gear_final_drive=4.0))
    out["sprocket_far"] = _plain(pti.sprocket_design(0.5, 100.0))
    out["sprocket_40"] = _plain(pti.sprocket_design(3.5, 230.0, "#40 / 420 (1/2\")", 12))
    out["fan_ties"] = [pti.FanCurve.from_points("t", [(900, 250), (0, 300), (900, 240), (2000, 0)]).pressure_at(q)
                       for q in (899.0, 900.0, 901.0)]
    weak = SimpleNamespace(_rpm=[0.0, 3000.0, 6000.0], _t=[3.0, 3.0, 0.0])
    out["top_speed"] = [pti._top_speed_kmh(m, fd, 0.2, 0.9, 300.0, cda, crr) for m, fd, cda, crr in (
        (_map(), 3.5, 1.1, 0.018), (_map(), 3.5, 0.0, 0.0), (weak, 1.0, 1.1, 0.018), (weak, 6.0, 1.1, 0.018),
        (weak, 100.0, 1.1, 0.018))]
    return out


def test_every_output_word_for_word():
    got = json.loads(json.dumps(_everything(), ensure_ascii=False))
    if os.environ.get("KINEMATIK_REGOLD"):
        GOLD.write_text(json.dumps(got, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
    want = json.loads(GOLD.read_text(encoding="utf-8"))
    for key in want:
        if isinstance(want[key], list):
            assert len(got[key]) == len(want[key]), key
            for i, (g, w) in enumerate(zip(got[key], want[key])):
                assert g == w, (key, i)
        else:
            assert got[key] == want[key], key


def test_the_corpus_reaches_every_rule():
    rules = {a[4] for a in json.loads(GOLD.read_text(encoding="utf-8"))["assumptions"] if len(a) == 7}
    assert rules >= {"power_caps_rpm__coincidence", "power_caps_rpm", "continuous_exceeds_peak",
                     "continuous_correct", "continuous_unverified", "peak_power_at_redline", "base_speed_correct",
                     "base_speed_wrong", "base_speed_no_number", "top_speed_correct", "top_speed_wrong",
                     "top_speed_no_number", "torque_at_rpm_correct", "torque_at_rpm_wrong", "cap_compliance__over",
                     "cap_compliance__ok", "more_power_more_speed", "no_rule_matched"}


def test_the_solver_refuses_no_motor_map():
    with pytest.raises(ValueError, match=r"^GearRatioSolver needs a MotorMap \(peak torque, power, redline\) to "
                                         r"sweep ratios against\.$"):
        pti.GearRatioSolver(None, mass_kg=300, wheel_r_m=0.2)
