"""Plausibility checks on powertrain inputs: catch a wrong value where it enters KinematiK.

The garbage-in benchmark (docs/GIGO_BENCHMARK_PLAN.md) found that KinematiK accepted every harmful wrong input
given to it: a mass in pounds, a wheel radius in millimetres, an efficiency of 93. These checks compare each value
with the range any real vehicle of its class falls in, and with the rest of the deck, and say what the likely slip
is. A value outside its range is not necessarily wrong (a record car can be one), so it warns, it does not refuse;
only values with no physical meaning (an efficiency above 1) are refused, by the functions themselves.

Two classes of range:

* ``GENERIC``: anything from an electric two-wheeler to a 40 t truck. Wide, so it catches thousandfold slips,
  percent-for-fraction slips and wrong length units, not a factor of two.
* ``FSAE``: a Formula Student car, from the rules and the cars this repository holds (the MathWorks Simscape car at
  405 kg is the heaviest). Narrow enough to catch pounds for kilograms, ft² for m² and an inverted gear ratio.

Each range carries the reason it is where it is; none was fitted to the benchmark's errors.

    from suspension import plausibility
    plausibility.check({"mass_kg": 608.0}, vehicle_class="fsae")   # warns: pounds typed as kilograms?
"""
from __future__ import annotations

import math
import warnings


class PlausibilityWarning(UserWarning):
    """A powertrain input outside the range of any real vehicle of its class."""


#: name: (low, high, unit, why the range is there, the slip a value outside it usually is)
GENERIC = {
    "mass_kg": (60.0, 60000.0, "kg", "an e-scooter with rider to a 40 t truck with margin",
                "grams or tonnes typed into a kg field"),
    "cda_m2": (0.1, 12.0, "m²", "a streamlined bike to a truck with trailer", "ft² typed into m², or Cd for CdA"),
    "cla_m2": (-2.0, 8.0, "m²", "lift to the downforce of a wing car", "ft² typed into m²"),
    "crr": (0.002, 0.05, "", "a train-like 0.002 to a soft off-road tire", "rolling resistance typed in percent"),
    "drivetrain_eff": (0.4, 1.0, "", "a worn chain drive to a direct drive", "an efficiency typed in percent"),
    "efficiency": (0.05, 1.0, "", "any point of a motor or engine map", "an efficiency typed in percent"),
    "wheel_radius_m": (0.1, 0.7, "m", "a scooter wheel to a truck tire", "mm, cm or inches typed into metres"),
    "mu": (0.2, 3.0, "", "wet ice excluded, to a slick with downforce", "grip typed in percent"),
    "final_drive": (0.8, 30.0, "", "an overdrive to a hub-motor reduction", "the ratio entered as its inverse"),
    "aux_power_w": (5.0, 30000.0, "W", "a dashboard to a bus's air conditioning (0 is allowed: no auxiliaries)",
                    "kW typed into a W field, or W into kW"),
    "peak_power_w": (500.0, 1.5e6, "W", "an e-bike to a hypercar", "kW typed into a W field, or W into kW"),
    "pack_voltage_v": (12.0, 1500.0, "V", "a 12 V system to the highest traction voltages",
                       "a cell voltage typed as the pack voltage"),
    "capacity_ah": (0.5, 2000.0, "Ah", "a tool battery to a bus pack", "mAh typed into Ah"),
    "pack_resistance_ohm": (0.0005, 5.0, "Ω", "a bus pack to a small high-voltage string", "mΩ typed into Ω"),
    "motor_max_rpm": (500.0, 40000.0, "rpm", "a hub motor to the fastest traction motors", "rad/s typed into rpm"),
    "motor_peak_torque_nm": (1.0, 6000.0, "N·m", "a hub motor to a truck motor", "kN·m or a unit slip"),
    "cg_height_m": (0.08, 2.5, "m", "a kart to a truck", "mm typed into metres"),
    "wheelbase_m": (0.5, 8.0, "m", "a kart to a truck", "mm typed into metres"),
    "specific_power_w_kg": (10.0, 2000.0, "W/kg", "a loaded truck to a hypercar", "a power or mass unit slip"),
}

#: A Formula Student car (FS/FSAE rules and the cars in benchmarks/powertrain); missing names fall back to GENERIC.
FSAE = {
    "mass_kg": (150.0, 500.0, "kg", "the lightest car with driver to the 405 kg Simscape car, with margin",
                "pounds typed as kilograms, or the car weighed in tonnes"),
    "cda_m2": (0.4, 2.5, "m²", "a slick car without wings to a full aero package", "ft² typed into m², or Cd for CdA"),
    "cla_m2": (0.0, 6.0, "m²", "no aero to the largest wing packages", "ft² typed into m²"),
    "wheel_radius_m": (0.18, 0.30, "m", "10 in to 13 in wheels with tire", "the diameter typed as the radius"),
    "mu": (0.8, 2.5, "", "a hard compound to a slick with downforce", "grip typed in percent"),
    "final_drive": (1.5, 20.0, "", "a direct-ish drive to a hub-motor planetary", "the ratio entered as its inverse"),
    "peak_power_w": (5000.0, 200000.0, "W", "a small single-cylinder car to the largest four-motor EVs",
                     "kW typed into a W field, or hp into kW"),
    "pack_voltage_v": (60.0, 600.0, "V", "the rules' 600 V ceiling", "a cell voltage typed as the pack voltage"),
    "capacity_ah": (2.0, 300.0, "Ah", "a small sprint pack to the 230 Ah Simscape pack", "mAh typed into Ah"),
    "cg_height_m": (0.15, 0.5, "m", "a low car to a tall one", "mm typed into metres"),
    "wheelbase_m": (1.525, 2.2, "m", "the rules' 1525 mm minimum to a long car", "mm typed into metres"),
}

CLASSES = {None: {}, "generic": {}, "fsae": FSAE}
#: Terms a deck switches off with 0 (no auxiliaries, no rolling resistance or drag in an idealized case): 0 is allowed.
OPTIONAL_TERMS = ("aux_power_w", "crr", "cda_m2", "cla_m2")


def _range(name, vehicle_class):
    if vehicle_class not in CLASSES:
        raise ValueError(f"unknown vehicle class {vehicle_class!r}; one of {sorted(c for c in CLASSES if c)}")
    return CLASSES[vehicle_class].get(name, GENERIC.get(name))


def check(values: dict, vehicle_class: str | None = None, where: str = "", warn: bool = True) -> list[str]:
    """Every value outside its class's range: a message each (also issued as a PlausibilityWarning unless
    warn=False). Values are in the units the tables state (SI: kg, m, m², W, V, Ah, Ω, rpm, N·m; efficiencies,
    Crr and grip dimensionless). Names not in the tables, and values that are None or not finite, are left alone (the functions
    refuse what has no meaning). A 0 in OPTIONAL_TERMS switches that term off and is allowed. A peak power with a mass
    also checks the specific power."""
    vals = dict(values)
    if vals.get("peak_power_w") is not None and vals.get("mass_kg"):
        vals["specific_power_w_kg"] = vals["peak_power_w"] / vals["mass_kg"]
    out = []
    for name, x in vals.items():
        rng = _range(name, vehicle_class)
        if rng is None or x is None:
            continue
        xs = [float(v) for v in (x if isinstance(x, (list, tuple)) else [x])]
        xs = [v for v in xs if math.isfinite(v) and not (name in OPTIONAL_TERMS and v == 0.0)]
        lo, hi, unit, why, slip = rng
        bad = [v for v in xs if not lo <= v <= hi]
        if bad:
            v = max(bad, key=lambda b: max(b / hi, lo / b) if b > 0 else math.inf)
            u = f" {unit}" if unit else ""
            msg = f"{where + ': ' if where else ''}{name} = {v:.4g}{u} is outside {lo:g}–{hi:g}{u} ({why}); is it {slip}?"
            out.append(msg)
    if warn:
        for m in out:
            warnings.warn(m, PlausibilityWarning, stacklevel=2)
    return out
