# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  suspension/cooling.py — the coolant loop, and whether your rig can see it
# ============================================================================
"""
Loop sizing, and the question a cooling test rig lives or dies on.

TWO HALVES, AND THE SECOND ONE IS THE IMPORTANT ONE.

SIZING is arithmetic. Motor and inverter losses become heat, the heat has to
leave through a radiator, and the coolant carries it there. Flow rate, the
temperature rise per lap, and how long the loop takes to reach its limit all
fall out of `Q = ṁ·cp·ΔT` and a radiator UA. It is worth doing early because
it sets the radiator area before the bodywork is drawn, and it is not hard.

INSTRUMENTATION is where cooling rigs are actually won and lost, and it is
almost never done before the rig is built. The rig exists to measure heat
rejection. Heat rejection is not measured — it is COMPUTED, from a flow rate
and two temperatures, and the uncertainty of those three inputs propagates
straight into the answer:

    (u_Q/Q)² = (u_V̇/V̇)² + (u_ΔT/ΔT)² + (u_cp/cp)²

The temperature term is the one that bites. Two independent ±0.5 °C sensors
give ±0.7 °C on their difference. Across a ΔT of 4 °C that is ±18 % before the
flow meter has said anything — and a rig with ±18 % on its output cannot tell
a good radiator from a bad one, cannot validate a CFD model to any useful
tolerance, and will produce a summer of data that proves nothing.

Two fixes, both cheap, both structural, both nearly always missed:

  RAISE ΔT. The error is a fraction of ΔT, so running the rig at low flow to
  open the temperature difference buys accuracy directly. A rig plumbed only
  for the car's design flow rate cannot do this, and that is a plumbing
  decision made months before anyone computes an error bar.

  MEASURE ΔT DIRECTLY. A matched sensor pair — or a differential thermopile —
  cancels most of the common-mode error, because what you need is the
  difference, not two absolute temperatures. Same money, several times the
  resolution.

So `rig_uncertainty` tells you what your sensor list actually resolves, and
`required_delta_t` inverts it: given the sensors you can afford, what
temperature difference must the rig be designed to produce for the answer to
be worth having? Ask that before ordering hoses, not after.

Pure Python + NumPy. Self-test: python3 -m suspension.cooling
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field as _dcfield
from typing import Dict, List, Optional, Tuple

import numpy as np


__all__ = ["Coolant", "COOLANTS", "coolant", "LoopSpec", "LoopResult",
           "size_loop", "TempSensor", "FlowMeter", "TEMP_SENSORS",
           "FLOW_METERS", "RigResult", "rig_uncertainty", "required_delta_t",
           "heat_load_w"]


# =========================================================================== #
#  Coolants
# =========================================================================== #
@dataclass(frozen=True)
class Coolant:
    key: str
    name: str
    cp_j_kgk: float
    rho_kg_m3: float
    boil_c_at_1bar: float
    note: str = ""


COOLANTS: Dict[str, Coolant] = {
    "water": Coolant("water", "Distilled water + inhibitor", 4180.0, 997.0,
                     100.0,
                     "The best coolant by a wide margin and usually banned "
                     "for road use, not competition — check your rules."),
    "eg50": Coolant("eg50", "50/50 ethylene glycol / water", 3400.0, 1070.0,
                    107.0,
                    "About 19 % less heat capacity than water. The freeze "
                    "protection is worthless in a summer competition and you "
                    "pay for it in radiator area."),
    "eg30": Coolant("eg30", "30/70 ethylene glycol / water", 3750.0, 1040.0,
                    104.0, "A reasonable compromise if you must run glycol."),
    "pg50": Coolant("pg50", "50/50 propylene glycol / water", 3480.0, 1040.0,
                    106.0, "Less toxic, slightly worse pumping viscosity."),
}


def coolant(key: str) -> Coolant:
    k = (key or "water").strip().lower().replace("/", "").replace("-", "")
    if k in COOLANTS:
        return COOLANTS[k]
    for c in COOLANTS.values():
        if k and k in c.name.lower():
            return c
    raise KeyError(f"unknown coolant '{key}' — have "
                   f"{', '.join(sorted(COOLANTS))}")


def heat_load_w(shaft_power_w: float, motor_eff: float = 0.94,
                inverter_eff: float = 0.97,
                coolant_fraction: float = 0.85) -> Tuple[float, List[str]]:
    """Heat into the coolant from a given shaft power.

    Not all loss goes into the coolant: some radiates and convects off the
    housings. `coolant_fraction` is the share the loop has to carry, and it
    is a declared assumption rather than a measurement.
    """
    p = max(float(shaft_power_w), 0.0)
    p_elec = p / max(motor_eff * inverter_eff, 1e-6)
    loss = p_elec - p
    q = loss * float(coolant_fraction)
    notes = [
        f"{p/1000:.1f} kW shaft at {motor_eff:.0%} motor × "
        f"{inverter_eff:.0%} inverter → {loss/1000:.2f} kW of loss.",
        f"{coolant_fraction:.0%} of that loss assumed to reach the coolant "
        f"({q/1000:.2f} kW); the rest leaves through the housings. That split "
        f"is an assumption, and it is the one to measure first on the rig.",
    ]
    return q, notes


# =========================================================================== #
#  Loop sizing
# =========================================================================== #
@dataclass
class LoopSpec:
    # Radiator air INLET temperature, inherited from IntegrationLedger via the
    # 'cooling_inlet' channel. Set ambient_is_local = True to pin it locally.
    ENV_CHANNEL = "cooling_inlet"

    heat_w: float = 4000.0
    coolant_key: str = "eg50"
    flow_lpm: float = 12.0
    ambient_c: float = 35.0
    ambient_is_local: bool = False
    coolant_in_c: float = 50.0
    max_coolant_c: float = 65.0        # motor/inverter inlet limit
    radiator_ua_w_per_k: float = 120.0
    loop_volume_l: float = 2.5
    cap_pressure_bar: float = 1.1


@dataclass
class LoopResult:
    spec: LoopSpec
    delta_t_k: float
    mass_flow_kg_s: float
    required_ua_w_per_k: float
    ua_margin: float
    steady_coolant_c: float
    time_to_limit_s: float
    boil_margin_c: float
    ok: bool
    notes: List[str] = _dcfield(default_factory=list)


def size_loop(spec: Optional[LoopSpec] = None) -> LoopResult:
    """Flow, temperature rise, radiator UA and how long you have."""
    s = spec or LoopSpec()
    c = coolant(s.coolant_key)
    m_dot = float(s.flow_lpm) / 60.0 / 1000.0 * c.rho_kg_m3     # kg/s
    dT = s.heat_w / max(m_dot * c.cp_j_kgk, 1e-9)

    #  Radiator: UA needed to reject the load at the available air-side
    #  temperature difference, using the coolant inlet as the driving temp.
    drive_k = max(s.coolant_in_c - s.ambient_c, 1e-6)
    ua_req = s.heat_w / drive_k
    margin = (s.radiator_ua_w_per_k / ua_req) if ua_req > 0 else float("inf")
    steady = s.ambient_c + s.heat_w / max(s.radiator_ua_w_per_k, 1e-9)

    #  Transient: with no radiator at all, how long from inlet to limit?
    m_loop = s.loop_volume_l / 1000.0 * c.rho_kg_m3
    head = max(s.max_coolant_c - s.coolant_in_c, 0.0)
    t_limit = (m_loop * c.cp_j_kgk * head / max(s.heat_w, 1e-9)
               if s.heat_w > 0 else float("inf"))

    #  Boiling point rises about 20 °C per bar of cap pressure for water-like
    #  mixes; declared, approximate, and enough to say whether you are close.
    boil = c.boil_c_at_1bar + 20.0 * max(s.cap_pressure_bar - 1.0, 0.0)
    boil_margin = boil - steady

    notes = [
        f"{c.name}: cp {c.cp_j_kgk:g} J/kg·K, ρ {c.rho_kg_m3:g} kg/m³. "
        f"{c.note}",
        f"{s.flow_lpm:g} L/min → {m_dot:.4f} kg/s → **{dT:.1f} K rise** "
        f"across the load at {s.heat_w/1000:.2f} kW.",
        f"Radiator needs **{ua_req:.0f} W/K** to hold {s.coolant_in_c:g} °C "
        f"against {s.ambient_c:g} °C ambient; you have "
        f"{s.radiator_ua_w_per_k:g} W/K ({margin:.2f}× margin).",
    ]
    if margin < 1.0:
        notes.append(
            "⚠️ The radiator is undersized for this load at this ambient. "
            "The loop will keep climbing until the coolant is hot enough to "
            "shed the heat — which is what `steady coolant` below tells you, "
            "and it is above your limit.")
    if t_limit < 120:
        notes.append(
            f"With the radiator ignored entirely, the loop's own thermal "
            f"mass buys only **{t_limit:.0f} s** before the limit. That is "
            f"the number that matters for Acceleration and Skidpad, where "
            f"the car is done before the radiator has done anything.")
    if boil_margin < 15.0:
        notes.append(
            f"⚠️ Only {boil_margin:.0f} °C between the steady coolant "
            f"temperature and the boiling point at a "
            f"{s.cap_pressure_bar:g} bar cap. Local film temperatures at the "
            f"stator are well above bulk, so this margin is smaller than it "
            f"looks.")
    ok = bool(margin >= 1.0 and steady <= s.max_coolant_c
              and boil_margin >= 15.0)
    return LoopResult(s, dT, m_dot, ua_req, margin, steady, t_limit,
                      boil_margin, ok, notes)


# =========================================================================== #
#  The instrumentation question
# =========================================================================== #
@dataclass(frozen=True)
class TempSensor:
    key: str
    name: str
    abs_c: float                 # absolute accuracy, °C, at loop temperatures
    differential: bool = False   # measures ΔT directly (matched / thermopile)
    note: str = ""


TEMP_SENSORS: Dict[str, TempSensor] = {
    "type_k": TempSensor("type_k", "Type K thermocouple", 2.2, False,
                         "Cheap, fast, and hopeless for a ΔT measurement."),
    "pt100_b": TempSensor("pt100_b", "PT100 class B", 0.65, False,
                          "±(0.3 + 0.005·|T|) — 0.65 °C at 70 °C."),
    "pt100_a": TempSensor("pt100_a", "PT100 class A", 0.29, False,
                          "±(0.15 + 0.002·|T|)."),
    "pt100_110din": TempSensor("pt100_110din", "PT100 1/10 DIN", 0.065, False,
                               "±(0.03 + 0.0005·|T|). Not expensive any more."),
    "matched_pair": TempSensor("matched_pair", "Matched PT100 pair on ΔT",
                               0.05, True,
                               "Calibrated as a pair, so common-mode error "
                               "cancels. This is the one to buy."),
    "thermopile": TempSensor("thermopile", "Differential thermopile", 0.02,
                             True,
                             "Measures the difference directly; no absolute "
                             "reading at all."),
}


@dataclass(frozen=True)
class FlowMeter:
    key: str
    name: str
    rel: float                   # relative accuracy, fraction of reading
    note: str = ""


FLOW_METERS: Dict[str, FlowMeter] = {
    "paddle": FlowMeter("paddle", "Paddle-wheel", 0.05,
                        "±5 % of reading, and worse near the bottom of its "
                        "range."),
    "turbine": FlowMeter("turbine", "Turbine", 0.03, "±3 % of reading."),
    "ultrasonic": FlowMeter("ultrasonic", "Clamp-on ultrasonic", 0.02,
                            "±2 %, non-invasive, sensitive to pipe wall and "
                            "fluid assumptions."),
    "coriolis": FlowMeter("coriolis", "Coriolis mass flow", 0.002,
                          "±0.2 % and measures mass directly, removing the "
                          "density assumption too. Expensive."),
}


@dataclass
class RigResult:
    delta_t_k: float
    heat_w: float
    u_flow_rel: float
    u_dt_rel: float
    u_cp_rel: float
    u_total_rel: float
    resolvable_w: float
    dominant: str
    verdict: str
    notes: List[str] = _dcfield(default_factory=list)


def rig_uncertainty(*, heat_w: float, flow_lpm: float, coolant_key: str,
                    temp_sensor: str, flow_meter: str,
                    cp_uncertainty_rel: float = 0.02,
                    target_resolution_rel: float = 0.10) -> RigResult:
    """What your sensor list actually resolves.

    Root-sum-square propagation through `Q = ρ·V̇·cp·ΔT`. Two independent
    absolute sensors contribute √2 times their individual accuracy to the
    difference; a matched pair or thermopile contributes its ΔT spec directly,
    which is the entire reason to buy one.
    """
    c = coolant(coolant_key)
    ts = TEMP_SENSORS[temp_sensor] if temp_sensor in TEMP_SENSORS \
        else TEMP_SENSORS["pt100_b"]
    fm = FLOW_METERS[flow_meter] if flow_meter in FLOW_METERS \
        else FLOW_METERS["turbine"]

    m_dot = float(flow_lpm) / 60.0 / 1000.0 * c.rho_kg_m3
    dT = float(heat_w) / max(m_dot * c.cp_j_kgk, 1e-9)
    u_dt_abs = ts.abs_c if ts.differential else ts.abs_c * math.sqrt(2.0)
    u_dt_rel = u_dt_abs / max(dT, 1e-9)
    u_flow_rel = fm.rel
    u_total = math.sqrt(u_flow_rel ** 2 + u_dt_rel ** 2
                        + cp_uncertainty_rel ** 2)

    terms = {"flow meter": u_flow_rel, "temperature difference": u_dt_rel,
             "coolant cp": cp_uncertainty_rel}
    dominant = max(terms, key=lambda k: terms[k])

    if u_total <= target_resolution_rel:
        verdict = (f"This rig resolves heat rejection to ±{u_total:.1%}, "
                   f"which meets the ±{target_resolution_rel:.0%} you asked "
                   f"for.")
    else:
        verdict = (f"**This rig cannot do its job.** ±{u_total:.1%} on heat "
                   f"rejection against a ±{target_resolution_rel:.0%} target "
                   f"— it cannot distinguish two radiators that differ by "
                   f"less than about {u_total*2:.0%}, and a summer of data "
                   f"will not settle any argument you build it to settle.")

    notes = [
        f"ΔT at this load and flow is **{dT:.2f} K**.",
        f"{ts.name}: {ts.note} → "
        + (f"±{u_dt_abs:.3f} K on ΔT directly."
           if ts.differential else
           f"two independent sensors give ±{ts.abs_c:g} × √2 = "
           f"±{u_dt_abs:.3f} K on the difference."),
        f"{fm.name}: {fm.note}",
        f"Contributions: flow ±{u_flow_rel:.1%}, ΔT ±{u_dt_rel:.1%}, "
        f"cp ±{cp_uncertainty_rel:.1%} → total ±{u_total:.1%} "
        f"(root-sum-square).",
        f"**{dominant} dominates.** Spending money anywhere else first is "
        f"spending it in the wrong place.",
    ]
    if not ts.differential and u_dt_rel > u_flow_rel:
        notes.append(
            "The temperature term is the largest and it is also the cheapest "
            "to fix: a matched PT100 pair measures the difference directly "
            "and cancels most of the common-mode error. Same money as two "
            "good absolute sensors, several times the resolution.")
    if dT < 5.0:
        notes.append(
            f"A {dT:.1f} K difference is a hard thing to measure well. "
            "Plumb the rig so it can run BELOW the car's design flow rate — "
            "half the flow doubles ΔT and halves this error term outright. "
            "That is a valve and a bypass, decided now, not after the hoses "
            "are cut.")
    return RigResult(dT, float(heat_w), u_flow_rel, u_dt_rel,
                     cp_uncertainty_rel, u_total,
                     u_total * float(heat_w), dominant, verdict, notes)


def required_delta_t(*, temp_sensor: str, flow_meter: str,
                     target_resolution_rel: float = 0.10,
                     cp_uncertainty_rel: float = 0.02) -> Optional[float]:
    """Invert it: the ΔT the rig must be designed to produce.

    Returns None when the sensors cannot reach the target at ANY temperature
    difference — which happens whenever the flow meter alone already exceeds
    it, and is worth knowing before the rig is welded up.
    """
    ts = TEMP_SENSORS.get(temp_sensor, TEMP_SENSORS["pt100_b"])
    fm = FLOW_METERS.get(flow_meter, FLOW_METERS["turbine"])
    budget_sq = (target_resolution_rel ** 2 - fm.rel ** 2
                 - cp_uncertainty_rel ** 2)
    if budget_sq <= 0:
        return None
    u_dt_abs = ts.abs_c if ts.differential else ts.abs_c * math.sqrt(2.0)
    return u_dt_abs / math.sqrt(budget_sq)


# =========================================================================== #
#  Self-test
# =========================================================================== #
def _selftest() -> int:
    fails = 0

    def chk(name, cond, detail=""):
        nonlocal fails
        print(f"  {'PASS' if cond else 'FAIL'}  {name}"
              + (f"   {detail}" if detail and not cond else ""))
        if not cond:
            fails += 1

    print("· heat load")
    q, _n = heat_load_w(40000.0)
    chk("loss is a sane fraction", 1500 < q < 6000, f"{q}")

    print("· loop")
    r = size_loop(LoopSpec(heat_w=4000.0, flow_lpm=12.0, coolant_key="eg50"))
    chk("delta T is plausible", 3.0 < r.delta_t_k < 8.0, f"{r.delta_t_k}")
    chk("water beats glycol",
        size_loop(LoopSpec(coolant_key="water")).delta_t_k
        < size_loop(LoopSpec(coolant_key="eg50")).delta_t_k)
    weak = size_loop(LoopSpec(heat_w=9000.0, radiator_ua_w_per_k=60.0))
    chk("undersized radiator is caught", not weak.ok)
    chk("undersized radiator is explained",
        any("undersized" in n for n in weak.notes))

    print("· rig")
    bad = rig_uncertainty(heat_w=4000.0, flow_lpm=12.0, coolant_key="eg50",
                          temp_sensor="pt100_b", flow_meter="paddle")
    chk("a plausible rig fails the 10 % target", bad.u_total_rel > 0.10,
        f"{bad.u_total_rel}")
    chk("verdict says so", "cannot do its job" in bad.verdict)
    chk("temperature dominates", bad.dominant == "temperature difference",
        bad.dominant)

    good = rig_uncertainty(heat_w=4000.0, flow_lpm=12.0, coolant_key="eg50",
                           temp_sensor="matched_pair", flow_meter="turbine")
    chk("a matched pair rescues it", good.u_total_rel < bad.u_total_rel / 2,
        f"{good.u_total_rel} vs {bad.u_total_rel}")

    low = rig_uncertainty(heat_w=4000.0, flow_lpm=4.0, coolant_key="eg50",
                          temp_sensor="pt100_b", flow_meter="turbine")
    hi = rig_uncertainty(heat_w=4000.0, flow_lpm=20.0, coolant_key="eg50",
                         temp_sensor="pt100_b", flow_meter="turbine")
    chk("lower flow measures better", low.u_total_rel < hi.u_total_rel,
        f"{low.u_total_rel} vs {hi.u_total_rel}")

    print("· inverse")
    need = required_delta_t(temp_sensor="pt100_b", flow_meter="turbine")
    chk("required delta T is returned", need and need > 5.0, str(need))
    chk("impossible target returns None",
        required_delta_t(temp_sensor="pt100_b", flow_meter="paddle",
                         target_resolution_rel=0.03) is None)

    print(f"\n{'ALL PASS' if not fails else f'{fails} FAILURES'}")
    return 1 if fails else 0


if __name__ == "__main__":
    import sys
    sys.exit(_selftest())
