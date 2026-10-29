"""suspension/cooling.py: Q = m_dot cp dT for the loop (rise, required UA off the cold end and the mean, steady
coolant, time to the limit on the loop's own mass, the boiling margin with the cap), the root-sum-square uncertainty
of a heat-rejection rig and its inversion, the heat load from the drive losses, the reference tables, and the
advice text exactly as the user reads it in each branch."""
import math

import pytest

from suspension import cooling as c

LOOP_NOTES = {
    'default': ['50/50 ethylene glycol / water: cp 3400 J/kg·K, ρ 1070 kg/m³. About 19 % less heat capacity than water. The freeze protection is worthless in a summer competition and you pay for it in radiator area.', '12 L/min → 0.2140 kg/s → **5.5 K rise** across the load at 4.00 kW.', 'Radiator UA needed against the MEAN core temperature (52.7 °C) is 225 W/K; the figure below drives off the cold end instead, which is conservative by design — it asks for a bigger radiator, never a smaller one.', 'Radiator needs **267 W/K** to hold 50 °C against 35 °C ambient; you have 120 W/K (0.45× margin).', '⚠️ The radiator is undersized for this load at this ambient. The loop will keep climbing until the coolant is hot enough to shed the heat — which is what `steady coolant` below tells you, and it is above your limit.', "With the radiator ignored entirely, the loop's own thermal mass buys only **34 s** before the limit. That is the number that matters for Acceleration and Skidpad, where the car is done before the radiator has done anything."],
    'weak': ['50/50 ethylene glycol / water: cp 3400 J/kg·K, ρ 1070 kg/m³. About 19 % less heat capacity than water. The freeze protection is worthless in a summer competition and you pay for it in radiator area.', '12 L/min → 0.2140 kg/s → **12.4 K rise** across the load at 9.00 kW.', 'Radiator UA needed against the MEAN core temperature (56.2 °C) is 425 W/K; the figure below drives off the cold end instead, which is conservative by design — it asks for a bigger radiator, never a smaller one.', 'Radiator needs **600 W/K** to hold 50 °C against 35 °C ambient; you have 60 W/K (0.10× margin).', '⚠️ The radiator is undersized for this load at this ambient. The loop will keep climbing until the coolant is hot enough to shed the heat — which is what `steady coolant` below tells you, and it is above your limit.', "With the radiator ignored entirely, the loop's own thermal mass buys only **3 s** before the limit. That is the number that matters for Acceleration and Skidpad, where the car is done before the radiator has done anything.", '⚠️ Only -78 °C between the steady coolant temperature and the boiling point at a 1 bar cap. Local film temperatures at the stator are well above bulk, so this margin is smaller than it looks.'],
    'good': ['Distilled water + inhibitor: cp 4180 J/kg·K, ρ 997 kg/m³. The best coolant by a wide margin and usually banned for road use, not competition — check your rules.', '12 L/min → 0.1994 kg/s → **2.4 K rise** across the load at 2.00 kW.', 'Radiator UA needed against the MEAN core temperature (56.2 °C) is 94 W/K; the figure below drives off the cold end instead, which is conservative by design — it asks for a bigger radiator, never a smaller one.', 'Radiator needs **100 W/K** to hold 55 °C against 35 °C ambient; you have 400 W/K (4.00× margin).'],
}
RIG = {
    'pt100_b': ('**This rig cannot do its job.** ±17.1% on heat rejection against a ±10% target — it cannot distinguish two radiators that differ by less than about 34%, and a summer of data will not settle any argument you build it to settle.', ['ΔT at this load and flow is **5.50 K**.', 'PT100 class B: ±(0.3 + 0.005·|T|) — 0.65 °C at 70 °C. → two independent sensors give ±0.65 × √2 = ±0.919 K on the difference.', 'Turbine: ±3 % of reading.', 'Contributions: flow ±3.0%, ΔT ±16.7%, cp ±2.0% → total ±17.1% (root-sum-square).', '**temperature difference dominates.** Spending money anywhere else first is spending it in the wrong place.', 'The temperature term is the largest and it is also the cheapest to fix: a matched PT100 pair measures the difference directly and cancels most of the common-mode error. Same money as two good absolute sensors, several times the resolution.']),
    'matched': ('This rig resolves heat rejection to ±2.0%, which meets the ±10% you asked for.', ['ΔT at this load and flow is **14.40 K**.', 'Matched PT100 pair on ΔT: Calibrated as a pair, so common-mode error cancels. This is the one to buy. → ±0.050 K on ΔT directly.', 'Coriolis mass flow: ±0.2 % and measures mass directly, removing the density assumption too. Expensive.', 'Contributions: flow ±0.2%, ΔT ±0.3%, cp ±2.0% → total ±2.0% (root-sum-square).', '**coolant cp dominates.** Spending money anywhere else first is spending it in the wrong place.']),
    'flow': ('This rig resolves heat rejection to ±5.4%, which meets the ±20% you asked for.', ['ΔT at this load and flow is **38.39 K**.', 'PT100 1/10 DIN: ±(0.03 + 0.0005·|T|). Not expensive any more. → two independent sensors give ±0.065 × √2 = ±0.092 K on the difference.', 'Paddle-wheel: ±5 % of reading, and worse near the bottom of its range.', 'Contributions: flow ±5.0%, ΔT ±0.2%, cp ±2.0% → total ±5.4% (root-sum-square).', '**flow meter dominates.** Spending money anywhere else first is spending it in the wrong place.']),
}
HEAT_NOTES = ['40.0 kW shaft at 90% motor × 95% inverter → 6.78 kW of loss.', '80% of that loss assumed to reach the coolant (5.43 kW); the rest leaves through the housings. That split is an assumption, and it is the one to measure first on the rig.']


def test_the_tables_and_lookup():
    assert {k: (v.key, v.name, v.cp_j_kgk, v.rho_kg_m3, v.boil_c_at_1bar) for k, v in c.COOLANTS.items()} == {
        "water": ("water", "Distilled water + inhibitor", 4180.0, 997.0, 100.0),
        "eg50": ("eg50", "50/50 ethylene glycol / water", 3400.0, 1070.0, 107.0),
        "eg30": ("eg30", "30/70 ethylene glycol / water", 3750.0, 1040.0, 104.0),
        "pg50": ("pg50", "50/50 propylene glycol / water", 3480.0, 1040.0, 106.0)}
    assert c.coolant(None) is c.COOLANTS["water"] and c.coolant(" EG-50 ") is c.COOLANTS["eg50"]
    assert c.coolant("eg/30") is c.COOLANTS["eg30"] and c.coolant("propylene") is c.COOLANTS["pg50"]
    with pytest.raises(KeyError) as e:
        c.coolant("oil")
    assert e.value.args[0] == "unknown coolant 'oil' — have eg30, eg50, pg50, water"
    assert c.coolant("") is c.COOLANTS["water"]
    assert {k: (v.name, v.abs_c, v.differential) for k, v in c.TEMP_SENSORS.items()} == {
        "type_k": ("Type K thermocouple", 2.2, False), "pt100_b": ("PT100 class B", 0.65, False),
        "pt100_a": ("PT100 class A", 0.29, False), "pt100_110din": ("PT100 1/10 DIN", 0.065, False),
        "matched_pair": ("Matched PT100 pair on ΔT", 0.05, True), "thermopile": ("Differential thermopile", 0.02,
                                                                               True)}
    assert {k: (v.name, v.rel) for k, v in c.FLOW_METERS.items()} == {
        "paddle": ("Paddle-wheel", 0.05), "turbine": ("Turbine", 0.03), "ultrasonic": ("Clamp-on ultrasonic", 0.02),
        "coriolis": ("Coriolis mass flow", 0.002)}
    assert all(v.note for v in list(c.COOLANTS.values()) + list(c.TEMP_SENSORS.values()) + list(
        c.FLOW_METERS.values()))


def test_the_heat_load():
    q, n = c.heat_load_w(40000.0, motor_eff=0.9, inverter_eff=0.95, coolant_fraction=0.8)
    loss = 40000.0 / (0.9 * 0.95) - 40000.0
    assert q == pytest.approx(loss * 0.8) and n == HEAT_NOTES
    assert c.heat_load_w(40000.0)[0] == pytest.approx((40000.0 / (0.94 * 0.97) - 40000.0) * 0.85)
    assert c.heat_load_w(-5.0)[0] == 0.0
    assert c.heat_load_w(1.0, motor_eff=0.0, inverter_eff=0.5)[0] == pytest.approx((1.0 / 1e-6 - 1.0) * 0.85)


def _loop(s):
    cl = c.COOLANTS[s.coolant_key]
    md = s.flow_lpm / 60000.0 * cl.rho_kg_m3
    dT = s.heat_w / (md * cl.cp_j_kgk)
    ua = s.heat_w / (s.coolant_in_c - s.ambient_c)
    steady = s.ambient_c + s.heat_w / s.radiator_ua_w_per_k
    t = s.loop_volume_l / 1000 * cl.rho_kg_m3 * cl.cp_j_kgk * max(s.max_coolant_c - s.coolant_in_c, 0) / s.heat_w
    boil = cl.boil_c_at_1bar + 20 * max(s.cap_pressure_bar - 1, 0) - steady
    return dT, md, ua, s.radiator_ua_w_per_k / ua, steady, t, boil


def test_sizing_the_loop():
    d = c.LoopSpec()
    assert (d.heat_w, d.coolant_key, d.flow_lpm, d.ambient_c, d.ambient_is_local, d.coolant_in_c, d.max_coolant_c,
            d.radiator_ua_w_per_k, d.loop_volume_l, d.cap_pressure_bar, d.ENV_CHANNEL) == (
        4000.0, "eg50", 12.0, 35.0, False, 50.0, 65.0, 120.0, 2.5, 1.1, "cooling_inlet")
    specs = {"default": c.LoopSpec(), "weak": c.LoopSpec(heat_w=9000.0, radiator_ua_w_per_k=60.0, loop_volume_l=0.5,
                                                        cap_pressure_bar=1.0),
             "good": c.LoopSpec(heat_w=2000.0, coolant_key="water", radiator_ua_w_per_k=400.0, loop_volume_l=8.0,
                                coolant_in_c=55.0, max_coolant_c=70.0, cap_pressure_bar=1.5)}
    for k, s in specs.items():
        r = c.size_loop(s)
        assert (r.delta_t_k, r.mass_flow_kg_s, r.required_ua_w_per_k, r.ua_margin, r.steady_coolant_c,
                r.time_to_limit_s, r.boil_margin_c) == pytest.approx(_loop(s))
        assert r.notes == LOOP_NOTES[k] and r.spec is s
    assert c.size_loop().ok is False and c.size_loop(specs["good"]).ok is True
    assert c.size_loop(None).delta_t_k == pytest.approx(_loop(c.LoopSpec())[0])
    hot = c.size_loop(c.LoopSpec(heat_w=2000.0, coolant_key="water", radiator_ua_w_per_k=400.0, loop_volume_l=8.0,
                                 coolant_in_c=55.0, max_coolant_c=38.0))
    assert hot.time_to_limit_s == 0.0 and hot.ok is False                     # steady 40 °C is over a 38 °C limit
    boil = c.size_loop(c.LoopSpec(heat_w=2000.0, coolant_key="water", radiator_ua_w_per_k=140.0, max_coolant_c=200.0,
                                  cap_pressure_bar=2.0))
    assert boil.boil_margin_c == pytest.approx(120.0 - 35.0 - 2000 / 140) and boil.ok is True
    near = c.size_loop(c.LoopSpec(heat_w=2000.0, coolant_key="water", radiator_ua_w_per_k=140.0, ambient_c=80.0,
                                  coolant_in_c=95.0, max_coolant_c=200.0, cap_pressure_bar=1.0))
    assert near.ua_margin > 1.0 and near.boil_margin_c < 15.0 and near.ok is False   # the boiling margin alone
    off = c.size_loop(c.LoopSpec(heat_w=0.0))
    assert off.time_to_limit_s == math.inf and off.ua_margin == math.inf and off.delta_t_k == 0.0
    same = c.size_loop(c.LoopSpec(coolant_in_c=35.0, ambient_c=35.0, flow_lpm=0.0, radiator_ua_w_per_k=0.0))
    assert same.required_ua_w_per_k == pytest.approx(4000.0 / 1e-6) and same.delta_t_k == pytest.approx(4000 / 1e-9)
    assert same.steady_coolant_c == pytest.approx(35.0 + 4000.0 / 1e-9)


def test_the_rig_uncertainty():
    cases = {"pt100_b": dict(heat_w=4000.0, flow_lpm=12.0, coolant_key="eg50", temp_sensor="pt100_b",
                             flow_meter="turbine"),
             "matched": dict(heat_w=4000.0, flow_lpm=4.0, coolant_key="water", temp_sensor="matched_pair",
                             flow_meter="coriolis"),
             "flow": dict(heat_w=8000.0, flow_lpm=3.0, coolant_key="water", temp_sensor="pt100_110din",
                          flow_meter="paddle", target_resolution_rel=0.2)}
    for k, kw in cases.items():
        r = c.rig_uncertainty(**kw)
        cl = c.COOLANTS[kw["coolant_key"]]
        ts, fm = c.TEMP_SENSORS[kw["temp_sensor"]], c.FLOW_METERS[kw["flow_meter"]]
        dT = kw["heat_w"] / (kw["flow_lpm"] / 60000 * cl.rho_kg_m3 * cl.cp_j_kgk)
        udt = (ts.abs_c if ts.differential else ts.abs_c * math.sqrt(2)) / dT
        tot = math.sqrt(fm.rel ** 2 + udt ** 2 + 0.02 ** 2)
        assert (r.delta_t_k, r.heat_w, r.u_flow_rel, r.u_dt_rel, r.u_cp_rel, r.u_total_rel, r.resolvable_w) == \
            pytest.approx((dT, kw["heat_w"], fm.rel, udt, 0.02, tot, tot * kw["heat_w"]))
        assert (r.verdict, r.notes) == RIG[k]
    assert c.rig_uncertainty(**cases["matched"]).dominant == "coolant cp"
    assert c.rig_uncertainty(**cases["flow"]).dominant == "flow meter"
    d = c.rig_uncertainty(heat_w=4000.0, flow_lpm=12.0, coolant_key="eg50", temp_sensor="nope", flow_meter="nope",
                          cp_uncertainty_rel=0.05, target_resolution_rel=0.5)
    assert (d.u_flow_rel, d.u_cp_rel) == (0.03, 0.05) and d.u_dt_rel == pytest.approx(
        0.65 * math.sqrt(2) / c.rig_uncertainty(**cases["pt100_b"]).delta_t_k)
    assert d.verdict.startswith("This rig resolves heat rejection to")
    edge = c.rig_uncertainty(heat_w=4000.0, flow_lpm=4.0, coolant_key="water", temp_sensor="thermopile",
                             flow_meter="coriolis", cp_uncertainty_rel=0.0, target_resolution_rel=0.0)
    assert edge.verdict.startswith("**This rig cannot do its job.**")
    big = c.rig_uncertainty(heat_w=1e6, flow_lpm=1.0, coolant_key="water", temp_sensor="type_k", flow_meter="turbine")
    assert big.delta_t_k > 5 and not any("hard thing to measure" in n for n in big.notes)
    zero = c.rig_uncertainty(heat_w=0.0, flow_lpm=0.0, coolant_key="water", temp_sensor="type_k", flow_meter="turbine")
    assert zero.delta_t_k == 0.0 and zero.u_dt_rel == pytest.approx(2.2 * math.sqrt(2) / 1e-9)


def test_the_required_temperature_difference():
    assert c.required_delta_t(temp_sensor="pt100_b", flow_meter="turbine") == pytest.approx(
        0.65 * math.sqrt(2) / math.sqrt(0.01 - 0.0009 - 0.0004))
    assert c.required_delta_t(temp_sensor="thermopile", flow_meter="coriolis", target_resolution_rel=0.05,
                              cp_uncertainty_rel=0.01) == pytest.approx(0.02 / math.sqrt(0.0025 - 0.000004 - 0.0001))
    assert c.required_delta_t(temp_sensor="?", flow_meter="?") == c.required_delta_t(temp_sensor="pt100_b",
                                                                                    flow_meter="turbine")
    assert c.required_delta_t(temp_sensor="pt100_b", flow_meter="paddle", target_resolution_rel=0.05) is None
    assert c.required_delta_t(temp_sensor="pt100_b", flow_meter="paddle", target_resolution_rel=0.05,
                              cp_uncertainty_rel=0.0) is None                    # exactly used up


def test_large_powers_tiny_flows_and_tiny_loads():
    q, n = c.heat_load_w(1e6, motor_eff=0.9, inverter_eff=0.95, coolant_fraction=0.8)
    loss = 1e6 / (0.9 * 0.95) - 1e6
    assert n[0] == f"{1e6/1000:.1f} kW shaft at 90% motor × 95% inverter → {loss/1000:.2f} kW of loss."
    r = c.rig_uncertainty(heat_w=100.0, flow_lpm=1e-5, coolant_key="water", temp_sensor="type_k", flow_meter="turbine")
    assert r.delta_t_k == pytest.approx(100.0 / (1e-5 / 60000 * 997 * 4180))
    low = c.rig_uncertainty(heat_w=2000.0, flow_lpm=12.0, coolant_key="water", temp_sensor="matched_pair",
                            flow_meter="coriolis")
    assert low.delta_t_k < 5.0 and low.notes[-1] == (
        f"A {low.delta_t_k:.1f} K difference is a hard thing to measure well. Plumb the rig so it can run BELOW the "
        "car's design flow rate — half the flow doubles ΔT and halves this error term outright. That is a valve and a "
        "bypass, decided now, not after the hoses are cut.")
    s = c.LoopSpec(heat_w=0.5, coolant_key="water", ambient_c=35.0, coolant_in_c=35.2)
    r = c.size_loop(s)
    cl = c.COOLANTS["water"]
    md = 12.0 / 60000 * cl.rho_kg_m3
    dT = 0.5 / (md * cl.cp_j_kgk)
    assert r.ua_margin == pytest.approx(120.0 / (0.5 / 0.2))
    assert r.time_to_limit_s == pytest.approx(2.5 / 1000 * cl.rho_kg_m3 * cl.cp_j_kgk * 29.8 / 0.5)
    mean = 35.2 + 0.5 * dT - 35.0
    assert f"is {0.5 / max(mean, 1e-6):.0f} W/K" in r.notes[2]
