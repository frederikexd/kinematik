"""Powertrain integration layer — the engine behind making the powertrain
sub-team's Excel/screenshot workflow irrelevant.

WHY THIS EXISTS
---------------
Elbee Racing's powertrain sub-team still works the traditional way: a gear-ratio
spreadsheet ("Alec's sheet"), a DFMEA workbook, a screenshot "spec sheet", and
placeholder CAD for the cooling package. Every other sub-team publishes their
numbers to KinematiK's integration ledger so the cross-team physics checks fire;
powertrain doesn't, which is exactly where the miscommunication comes from.

This module gives the EV Powertrain tab the missing pieces, each one replacing a
specific Excel artifact seen in the team's meeting deck:

  * GearRatioSolver          -> replaces Alec's gear-ratio spreadsheet. Sweeps
                                final-drive ratios against the motor map + the
                                car, and reports accel / top-speed / launch /
                                redline-margin for each, picking the optimum for
                                the chosen objective.
  * sprocket_design()        -> the slide-4 sprocket task: tooth count, pitch
                                diameter, chain tension and the tooth force the
                                FEA has to clear, straight from the ratio + motor
                                torque.
  * FanCurve / cooling_operating_point()
                             -> turns the SPAL fan datasheet (the cooling test
                                rig's fan) into a real operating point against
                                the coolant-loop system resistance, and checks it
                                against the heat the motor + pack actually reject.
  * powertrain_spec_sheet()  -> the live "Design EV Spec Sheet", generated from
                                ledger truth instead of a screenshot.

Everything here is pure / unit-tested and free of Streamlit so the test-suite can
exercise it headless. The UI layer (streamlit_app.py) calls in and writes results
to the IntegrationLedger so the driveline-torque, HV-voltage, thermal and
mount-load checks finally include powertrain.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Sequence, Tuple

import numpy as np


# --------------------------------------------------------------------------- #
#  Gear-ratio / final-drive optimisation  (replaces Alec's spreadsheet)
# --------------------------------------------------------------------------- #
class GearObjective(str, Enum):
    """What the team is optimising the final drive for. The three real choices an
    FSAE-EV powertrain lead defends to the design judges."""

    ACCEL = "accel"            # 75 m acceleration event — launch & low-speed pull
    TOPSPEED = "topspeed"      # reach the highest straight-line speed within redline
    BALANCED = "balanced"      # best autocross/endurance compromise (default)

    def label(self) -> str:
        return {
            "accel": "Acceleration (75 m)",
            "topspeed": "Top speed",
            "balanced": "Balanced (autocross/endurance)",
        }[self.value]


@dataclass
class GearCandidate:
    """One final-drive ratio scored against the motor + car."""

    final_drive: float
    # straight-line metrics
    top_speed_kmh: float            # speed where motor force == resistance (or redline)
    redline_speed_kmh: float        # road speed at motor redline in this ratio
    redline_limited: bool           # True if top speed is capped by redline, not power
    accel_0_75_s: float             # modelled 0->75 m time (lower is better)
    launch_force_n: float           # tractive force at launch (low speed) at the wheel
    # usefulness flags
    grip_limited_launch: bool       # launch force exceeds tyre grip (good headroom)
    score: float = 0.0              # objective score (higher = better)
    notes: List[str] = field(default_factory=list)


@dataclass
class GearSweepResult:
    objective: GearObjective
    candidates: List[GearCandidate]
    best: Optional[GearCandidate]
    warnings: List[str] = field(default_factory=list)

    def as_table(self) -> List[dict]:
        out = []
        for c in self.candidates:
            out.append({
                "Final drive": round(c.final_drive, 2),
                "0–75 m (s)": round(c.accel_0_75_s, 2),
                "Top speed (km/h)": round(c.top_speed_kmh, 1),
                "Redline @ (km/h)": round(c.redline_speed_kmh, 1),
                "Launch force (N)": round(c.launch_force_n, 0),
                "Limited by": "redline" if c.redline_limited else "power/drag",
                "Score": round(c.score, 3),
                "★": "★" if (self.best is not None and c is self.best) else "",
            })
        return out


def _motor_wheel_force(motor_map, v_ms: float, final_drive: float,
                       wheel_r: float, eff: float) -> float:
    """Tractive force at the contact patch at road speed v, for a given ratio,
    using the motor torque/speed map. Mirrors MotorMap.wheel_force but lets us
    override the final drive without mutating the map."""
    v = max(v_ms, 0.05)
    r = max(wheel_r, 0.05)
    rpm = (v / r) * final_drive * 60.0 / (2.0 * math.pi)
    rp = np.asarray(motor_map._rpm, float)
    tq = np.asarray(motor_map._t, float)
    if rpm <= rp[0]:
        t_motor = float(tq[0])
    elif rpm >= rp[-1]:
        t_motor = 0.0
    else:
        t_motor = float(np.interp(rpm, rp, tq))
    t_wheel = t_motor * final_drive * max(eff, 0.05)
    return t_wheel / r


def _redline_rpm(motor_map) -> float:
    return float(np.asarray(motor_map._rpm, float)[-1])


def _speed_at_redline_kmh(motor_map, final_drive: float, wheel_r: float) -> float:
    """Road speed when the motor is at its redline in this ratio."""
    redline = _redline_rpm(motor_map)
    omega = redline * 2.0 * math.pi / 60.0       # rad/s motor
    v = omega * wheel_r / max(final_drive, 1e-6)  # m/s road
    return v * 3.6


def _resistance_n(v_ms: float, mass_kg: float, cda: float, crr: float,
                  rho: float = 1.2, g: float = 9.81) -> float:
    """Aero drag + rolling resistance at speed v."""
    return 0.5 * rho * cda * v_ms * v_ms + crr * mass_kg * g


def _accel_0_75(motor_map, final_drive: float, wheel_r: float, eff: float,
                mass_kg: float, mu: float, cda: float, crr: float,
                rear_frac: float = 0.55, dist_m: float = 75.0) -> float:
    """Forward-Euler 0->dist time. Tractive force = min(motor force, tyre grip).
    Deliberately simple but ratio-sensitive: a too-tall gear starves launch, a
    too-short gear hits redline before the line."""
    dt = 0.005
    v = 0.0
    s = 0.0
    t = 0.0
    g = 9.81
    grip_cap = mu * mass_kg * g * rear_frac   # rear-driven traction ceiling
    while s < dist_m and t < 30.0:
        f_motor = _motor_wheel_force(motor_map, v, final_drive, wheel_r, eff)
        f_drive = min(f_motor, grip_cap)
        f_net = f_drive - _resistance_n(v, mass_kg, cda, crr)
        a = f_net / max(mass_kg, 1.0)
        v = max(0.0, v + a * dt)
        s += v * dt
        t += dt
    return t


def _top_speed_kmh(motor_map, final_drive: float, wheel_r: float, eff: float,
                   mass_kg: float, cda: float, crr: float) -> Tuple[float, bool]:
    """Highest sustainable speed: where motor force first falls to resistance,
    OR redline speed if that comes first. Returns (kmh, redline_limited).

    If the force/drag crossover lands within ~2% of redline speed, the motor is
    effectively the limiter (a taller ratio would reach more speed), so we report
    it as redline-limited."""
    v_redline = _speed_at_redline_kmh(motor_map, final_drive, wheel_r) / 3.6
    v = 1.0
    last_ok = 0.0
    step = 0.25
    while v <= v_redline + step:
        f = _motor_wheel_force(motor_map, v, final_drive, wheel_r, eff)
        r = _resistance_n(v, mass_kg, cda, crr)
        if f <= r:
            # crossover found; if it's right up against redline, the motor (not
            # drag) is what's stopping us going faster -> redline-limited
            near_redline = last_ok >= 0.98 * v_redline
            return last_ok * 3.6, near_redline
        last_ok = v
        v += step
    # never out-resisted within redline -> redline-limited
    return v_redline * 3.6, True


class GearRatioSolver:
    """Sweeps final-drive ratios against the motor map and the car, scoring each
    for the chosen objective. This is the whole of Alec's spreadsheet, made live
    and tied to the real motor curve and vehicle mass."""

    def __init__(self, motor_map, *, mass_kg: float, wheel_r_m: float,
                 drivetrain_eff: float = 0.90, mu: float = 1.4,
                 cda: float = 1.10, crr: float = 0.018,
                 rear_frac: float = 0.55):
        if motor_map is None:
            raise ValueError("GearRatioSolver needs a MotorMap (peak torque, "
                             "power, redline) to sweep ratios against.")
        self.motor_map = motor_map
        self.mass_kg = float(mass_kg)
        self.wheel_r = float(wheel_r_m)
        self.eff = float(drivetrain_eff)
        self.mu = float(mu)
        self.cda = float(cda)
        self.crr = float(crr)
        self.rear_frac = float(rear_frac)

    def sweep(self, ratios: Sequence[float],
              objective: GearObjective = GearObjective.BALANCED
              ) -> GearSweepResult:
        cands: List[GearCandidate] = []
        warns: List[str] = []
        grip_cap = self.mu * self.mass_kg * 9.81 * self.rear_frac
        for fd in ratios:
            fd = float(fd)
            if fd <= 0:
                continue
            top, redlimited = _top_speed_kmh(
                self.motor_map, fd, self.wheel_r, self.eff,
                self.mass_kg, self.cda, self.crr)
            rl_speed = _speed_at_redline_kmh(self.motor_map, fd, self.wheel_r)
            accel = _accel_0_75(
                self.motor_map, fd, self.wheel_r, self.eff,
                self.mass_kg, self.mu, self.cda, self.crr, self.rear_frac)
            launch = _motor_wheel_force(self.motor_map, 1.0, fd,
                                        self.wheel_r, self.eff)
            grip_lim = launch >= grip_cap
            notes = []
            if redlimited:
                notes.append("Top speed capped by motor redline, not power/drag "
                             "— a taller ratio would reach higher speed.")
            if grip_lim:
                notes.append("Launch force exceeds tyre grip — full traction "
                             "headroom (good for accel).")
            else:
                notes.append("Launch force below tyre grip — leaving traction on "
                             "the table at the line.")
            cands.append(GearCandidate(
                final_drive=fd, top_speed_kmh=top, redline_speed_kmh=rl_speed,
                redline_limited=redlimited, accel_0_75_s=accel,
                launch_force_n=launch, grip_limited_launch=grip_lim, notes=notes))

        if not cands:
            return GearSweepResult(objective, [], None,
                                   ["No valid ratios to sweep."])

        # ---- score per objective (normalise each metric 0..1, higher better) -- #
        accels = np.array([c.accel_0_75_s for c in cands])
        tops = np.array([c.top_speed_kmh for c in cands])

        def _norm_lower(a):  # lower is better -> invert
            rng = (a.max() - a.min()) or 1.0
            return 1.0 - (a - a.min()) / rng

        def _norm_higher(a):
            rng = (a.max() - a.min()) or 1.0
            return (a - a.min()) / rng

        s_accel = _norm_lower(accels)
        s_top = _norm_higher(tops)
        for i, c in enumerate(cands):
            if objective == GearObjective.ACCEL:
                c.score = float(s_accel[i])
            elif objective == GearObjective.TOPSPEED:
                c.score = float(s_top[i])
            else:  # balanced: reward accel, but penalise a ratio that's so short
                # it's redline-limited (can't hold speed on the long straight)
                pen = 0.25 if c.redline_limited else 0.0
                c.score = float(0.65 * s_accel[i] + 0.35 * s_top[i] - pen)

        best = max(cands, key=lambda c: c.score)
        return GearSweepResult(objective, cands, best, warns)


# --------------------------------------------------------------------------- #
#  Sprocket + chain design  (the slide-4 sprocket / output-shaft task)
# --------------------------------------------------------------------------- #
# ANSI roller-chain pitches (mm) keyed by the usual FSAE call-outs.
CHAIN_PITCH_MM = {
    "#25 (1/4\")": 6.35,
    "#35 (3/8\")": 9.525,
    "#40 / 420 (1/2\")": 12.70,
    "#50 / 520 (5/8\")": 15.875,
}


@dataclass
class SprocketDesign:
    """Output-shaft sprocket sized to a final-drive ratio + motor torque. Gives
    the tooth force and chain tension the FEA must clear (slide 4)."""

    final_drive: float
    motor_sprocket_teeth: int
    driven_sprocket_teeth: int
    actual_ratio: float
    chain_label: str
    chain_pitch_mm: float
    motor_pitch_dia_mm: float
    driven_pitch_dia_mm: float
    peak_motor_torque_nm: float
    chain_tension_n: float          # tight-side tension at peak torque
    tooth_force_n: float            # force on the engaged tooth (≈ chain tension)
    teeth_in_mesh: int              # approximate, on the driven sprocket
    warnings: List[str] = field(default_factory=list)


def _pitch_diameter_mm(teeth: int, pitch_mm: float) -> float:
    """Standard roller-chain sprocket pitch diameter: D = p / sin(pi/N)."""
    teeth = max(int(teeth), 7)
    return pitch_mm / math.sin(math.pi / teeth)


def sprocket_design(final_drive: float, peak_motor_torque_nm: float,
                    chain_label: str = "#35 (3/8\")",
                    motor_sprocket_teeth: int = 14) -> SprocketDesign:
    """Design the driven (output-shaft) sprocket to hit `final_drive` from a chosen
    motor-pinion tooth count, and report the chain tension / tooth force at peak
    motor torque. That force is the FEA input the slide-4 task is asking for."""
    pitch = CHAIN_PITCH_MM.get(chain_label, 9.525)
    motor_sprocket_teeth = max(int(motor_sprocket_teeth), 9)
    driven = int(round(final_drive * motor_sprocket_teeth))
    driven = max(driven, motor_sprocket_teeth + 1)   # must be a reduction
    actual = driven / motor_sprocket_teeth

    d_motor = _pitch_diameter_mm(motor_sprocket_teeth, pitch)
    d_driven = _pitch_diameter_mm(driven, pitch)

    # Chain tension at peak torque acts at the MOTOR sprocket pitch radius:
    #   T_chain = torque / r_pitch  (the slack side ≈ 0 at peak pull)
    r_motor_m = (d_motor / 1000.0) / 2.0
    tension = peak_motor_torque_nm / max(r_motor_m, 1e-4)
    # Teeth in mesh ≈ half the driven sprocket wrap for a typical centre distance
    teeth_mesh = max(3, int(round(driven * 0.4)))

    warns = []
    if motor_sprocket_teeth < 11:
        warns.append(f"Motor pinion has only {motor_sprocket_teeth} teeth — small "
                     "pinions raise chain tension and wear; ≥13 is kinder.")
    if abs(actual - final_drive) > 0.15:
        warns.append(f"Closest integer-tooth ratio is {actual:.2f} vs the "
                     f"{final_drive:.2f} target — adjust pinion teeth to get closer.")
    if tension > 6000:
        warns.append(f"Chain tension {tension:.0f} N is high for FSAE chain — "
                     "consider a larger pitch (e.g. #40/520) or a lower ratio.")

    return SprocketDesign(
        final_drive=final_drive, motor_sprocket_teeth=motor_sprocket_teeth,
        driven_sprocket_teeth=driven, actual_ratio=actual,
        chain_label=chain_label, chain_pitch_mm=pitch,
        motor_pitch_dia_mm=d_motor, driven_pitch_dia_mm=d_driven,
        peak_motor_torque_nm=peak_motor_torque_nm,
        chain_tension_n=tension, tooth_force_n=tension,
        teeth_in_mesh=teeth_mesh, warnings=warns)


def driveline_peak_torque_nm(final_drive: float, peak_motor_torque_nm: float,
                             drivetrain_eff: float = 0.97) -> float:
    """Wheel/output-shaft peak torque after the reduction — the number the ledger's
    driveline-torque check compares against the CV/driveshaft rating."""
    return peak_motor_torque_nm * final_drive * drivetrain_eff


# --------------------------------------------------------------------------- #
#  Cooling test rig — SPAL fan curve operating point
# --------------------------------------------------------------------------- #
# The VA14-AP11/C-34A brushed axial fan datasheet, digitised from the team's PDF.
# Static pressure rise (Pa) vs airflow (m^3/h). This is the cooling test rig's fan.
SPAL_VA14_AP11_C34A = {
    "name": "SPAL VA14-AP11/C-34A (brushed axial)",
    # (airflow_m3h, static_pressure_pa)
    "curve": [
        (707.0, 0.0), (675.0, 25.0), (638.0, 50.0), (599.0, 74.0),
        (548.0, 100.0), (491.0, 124.0), (416.0, 149.0), (361.0, 174.0),
        (311.0, 200.0), (262.0, 224.0), (194.0, 251.0), (147.0, 276.0),
        (97.0, 298.0), (45.0, 324.0), (0.0, 349.0),
    ],
    "free_flow_m3h": 707.0,
    "max_static_pa": 349.0,
    "nominal_current_a": 7.0,
}


@dataclass
class FanCurve:
    name: str
    flow_m3h: np.ndarray            # ascending airflow
    dp_pa: np.ndarray              # static pressure at each flow

    @staticmethod
    def from_points(name: str, points: Sequence[Tuple[float, float]]) -> "FanCurve":
        pts = sorted(points, key=lambda p: p[0])
        f = np.array([p[0] for p in pts], float)
        d = np.array([p[1] for p in pts], float)
        return FanCurve(name=name, flow_m3h=f, dp_pa=d)

    @staticmethod
    def spal_default() -> "FanCurve":
        return FanCurve.from_points(SPAL_VA14_AP11_C34A["name"],
                                    SPAL_VA14_AP11_C34A["curve"])

    def pressure_at(self, flow_m3h: float) -> float:
        return float(np.interp(flow_m3h, self.flow_m3h, self.dp_pa))


@dataclass
class CoolingOperatingPoint:
    flow_m3h: float                 # operating airflow where fan == system curve
    flow_cms: float                 # same, in m^3/s (the ledger's unit)
    static_pressure_pa: float       # operating static pressure
    system_k: float                 # system resistance coefficient (dp = k*Q^2)
    fan_name: str
    # heat check
    heat_to_reject_w: float         # motor + pack heat the loop must dump
    cooling_capacity_w: float       # heat this airflow can carry at the design ΔT
    margin_w: float                 # capacity - need (negative = under-cooled)
    adequate: bool
    design_delta_t_c: float
    warnings: List[str] = field(default_factory=list)


def cooling_operating_point(fan: FanCurve, system_k: float,
                            *, heat_to_reject_w: float = 0.0,
                            air_delta_t_c: float = 20.0,
                            air_density: float = 1.2,
                            air_cp: float = 1005.0) -> CoolingOperatingPoint:
    """Intersect the fan curve with a quadratic system resistance dp = k·Q²
    (Q in m³/h) to find the operating airflow, then check whether that airflow can
    carry the motor+pack heat at the design air-side ΔT.

    system_k is the loop+radiator restriction. Smaller k (open ducting) -> more
    flow; larger k (tight rig plumbing) -> less. The cooling test rig exists to
    *measure* this k; here it's the knob that the rig will pin down.
    """
    fan = fan
    # sample the fan range and find where fan dp crosses k*Q^2
    qs = np.linspace(fan.flow_m3h.min(), fan.flow_m3h.max(), 600)
    fan_dp = np.interp(qs, fan.flow_m3h, fan.dp_pa)
    sys_dp = system_k * qs * qs
    diff = fan_dp - sys_dp
    # first sign change from + to - is the operating point
    op_q = qs[-1]
    for i in range(1, len(qs)):
        if diff[i - 1] >= 0 and diff[i] < 0:
            # linear interpolate the crossing
            f0, f1 = diff[i - 1], diff[i]
            frac = f0 / (f0 - f1) if (f0 - f1) != 0 else 0.0
            op_q = qs[i - 1] + frac * (qs[i] - qs[i - 1])
            break
    op_dp = float(np.interp(op_q, fan.flow_m3h, fan.dp_pa))
    op_cms = op_q / 3600.0

    # air-side heat capacity at this flow and design ΔT: Q_dot = m_dot*cp*ΔT
    m_dot = air_density * op_cms                      # kg/s
    capacity_w = m_dot * air_cp * air_delta_t_c
    margin = capacity_w - heat_to_reject_w
    adequate = (heat_to_reject_w <= 0) or (margin >= 0)

    warns = []
    if heat_to_reject_w > 0 and not adequate:
        warns.append(
            f"At {op_q:.0f} m³/h the fan can only carry {capacity_w:.0f} W at a "
            f"{air_delta_t_c:.0f} °C air rise, but the motor+pack reject "
            f"{heat_to_reject_w:.0f} W — under-cooled by {-margin:.0f} W. Lower the "
            "loop restriction, add a second fan, or accept a higher coolant temp.")
    if op_q <= fan.flow_m3h.min() + 1e-6:
        warns.append("Operating point is at the fan's stall end — system "
                     "restriction is too high for this fan.")
    return CoolingOperatingPoint(
        flow_m3h=op_q, flow_cms=op_cms, static_pressure_pa=op_dp,
        system_k=system_k, fan_name=fan.name, heat_to_reject_w=heat_to_reject_w,
        cooling_capacity_w=capacity_w, margin_w=margin, adequate=adequate,
        design_delta_t_c=air_delta_t_c, warnings=warns)


def system_k_from_point(flow_m3h: float, dp_pa: float) -> float:
    """Back out the system resistance coefficient from one measured (flow, dp)
    point on the test rig: k = dp / Q². This is what the rig produces."""
    q = max(flow_m3h, 1e-6)
    return dp_pa / (q * q)


# --------------------------------------------------------------------------- #
#  Live spec sheet (replaces the screenshot "Design EV Spec Sheet")
# --------------------------------------------------------------------------- #
def powertrain_spec_sheet(*, architecture: str, power_kw: float,
                          peak_torque_nm: float, hv_voltage_v: float,
                          pack_kwh: float, final_drive: float,
                          driven_teeth: Optional[int] = None,
                          motor_teeth: Optional[int] = None,
                          chain_tension_n: Optional[float] = None,
                          driveline_torque_nm: Optional[float] = None,
                          motor_mass_kg: Optional[float] = None,
                          heat_reject_w: Optional[float] = None,
                          cooling_flow_cms: Optional[float] = None,
                          is_estimate: bool = True) -> List[dict]:
    """Assemble the live design spec sheet as a list of {Parameter, Value, Unit,
    Source} rows — generated from the values the team has actually committed, so it
    never goes stale the way a screenshot does."""
    src = "estimate" if is_estimate else "committed"
    rows = [
        ("Motor architecture", architecture, "", src),
        ("Peak tractive power", f"{power_kw:.0f}", "kW", src),
        ("Peak motor torque", f"{peak_torque_nm:.0f}", "N·m", src),
        ("HV system voltage", f"{hv_voltage_v:.0f}", "V", src),
        ("Pack energy", f"{pack_kwh:.2f}", "kWh", src),
        ("Final drive ratio", f"{final_drive:.2f}", ":1", src),
    ]
    if motor_teeth and driven_teeth:
        rows.append(("Sprocket teeth (motor / driven)",
                     f"{motor_teeth} / {driven_teeth}", "", src))
    if chain_tension_n is not None:
        rows.append(("Chain tension @ peak torque", f"{chain_tension_n:.0f}", "N", src))
    if driveline_torque_nm is not None:
        rows.append(("Output-shaft peak torque", f"{driveline_torque_nm:.0f}", "N·m", src))
    if motor_mass_kg is not None:
        rows.append(("Motor + drivetrain mass", f"{motor_mass_kg:.1f}", "kg", src))
    if heat_reject_w is not None:
        rows.append(("Heat rejected (motor+inverter)", f"{heat_reject_w:.0f}", "W", src))
    if cooling_flow_cms is not None:
        rows.append(("Cooling airflow required", f"{cooling_flow_cms:.3f}", "m³/s", src))
    return [{"Parameter": p, "Value": v, "Unit": u, "Source": s}
            for (p, v, u, s) in rows]


def estimate_motor_heat_w(power_kw: float, inverter_motor_eff: float = 0.90,
                          duty: float = 0.6) -> float:
    """Rough continuous heat the motor+inverter reject: (1-eff) of the average
    electrical power over an endurance duty cycle. Defensible planning number for
    sizing the cooling loop, clearly an estimate until measured on the rig."""
    avg_power_w = power_kw * 1000.0 * max(0.05, min(1.0, duty))
    return avg_power_w * (1.0 - max(0.05, min(0.99, inverter_motor_eff)))


# --------------------------------------------------------------------------- #
#  Motor operating envelope  —  power vs RPM, base speed, peak vs continuous
# --------------------------------------------------------------------------- #
# WHY THIS EXISTS (a real miscommunication it prevents):
#   Chassis lead: "are we limiting rpm to ~7k since motor power can't exceed 80kW?"
#   Powertrain lead: "that's for peak power. hoping to limit peak to 80kW while
#                     also having 80kW continuous but I'm not sure how that works"
#
# Two tangled misconceptions:
#   (1) Power cap ≠ RPM cap. A motor reaches its power ceiling at BASE SPEED, then
#       holds constant power (torque ∝ 1/rpm) all the way to redline. Capping power
#       at 80 kW does NOT cap rpm — the motor keeps spinning to redline at 80 kW.
#   (2) "Peak 80 kW AND 80 kW continuous" is not how it works. FSAE caps TRACTIVE
#       power at 80 kW at the accumulator — that is the ceiling, full stop. "Peak"
#       vs "continuous" is a THERMAL story: peak is what you may draw briefly;
#       continuous is what the motor+inverter can sustain without overheating, and
#       it is ≤ peak, set by cooling — never a second, higher number.
#
# This analyzer makes both facts visible and machine-checkable so the assumption
# never has to be litigated in a chat thread again.

FSAE_TRACTIVE_POWER_CAP_KW = 80.0   # FSAE-EV rule: tractive system power ≤ 80 kW


@dataclass
class MotorEnvelope:
    """The motor's operating envelope, with the power/RPM relationship made
    explicit and the peak-vs-continuous distinction spelled out."""

    peak_torque_nm: float
    peak_power_kw: float
    redline_rpm: float
    base_speed_rpm: float            # where flat torque first hits peak power
    # sampled curves over rpm (for plotting)
    rpm: np.ndarray
    torque_nm: np.ndarray
    power_kw: np.ndarray
    # continuous (thermal) limit
    continuous_power_kw: float       # sustained power the cooling allows (≤ peak)
    continuous_torque_nm: float      # torque at base speed for the continuous limit
    # rule compliance
    rule_cap_kw: float
    over_cap: bool                   # does declared peak exceed the FSAE cap?
    notes: List[str] = field(default_factory=list)

    def power_at_redline_kw(self) -> float:
        return float(self.power_kw[-1])

    def explanation(self) -> str:
        """Plain-language answer to 'does capping power cap rpm?' — no."""
        return (
            f"Peak power ({self.peak_power_kw:.0f} kW) is reached at the **base "
            f"speed of {self.base_speed_rpm:.0f} rpm**, not at redline. From base "
            f"speed up to the {self.redline_rpm:.0f} rpm redline the motor holds "
            f"≈{self.peak_power_kw:.0f} kW while torque falls off as 1/rpm. So "
            f"**capping power at {self.peak_power_kw:.0f} kW does NOT cap rpm** — "
            f"the motor still spins to {self.redline_rpm:.0f} rpm, it just can't "
            f"make more than {self.peak_power_kw:.0f} kW doing it. RPM is limited "
            f"by the motor's redline and your gear ratio, a separate thing from "
            f"the power cap.")

    def peak_vs_continuous_note(self) -> str:
        return (
            f"'Peak' vs 'continuous' is a thermal limit, not two power numbers. "
            f"The FSAE rule caps tractive power at {self.rule_cap_kw:.0f} kW — "
            f"that's the ceiling. **Peak {self.peak_power_kw:.0f} kW** is what you "
            f"may pull in short bursts; **continuous {self.continuous_power_kw:.0f} "
            f"kW** is what the motor+inverter can hold without overheating, and it "
            f"is at or below peak — set by cooling, never above the cap. You cannot "
            f"have 'peak 80 kW and a higher continuous'; continuous ≤ peak ≤ "
            f"{self.rule_cap_kw:.0f} kW always.")


def motor_envelope(peak_torque_nm: float, peak_power_kw: float,
                   redline_rpm: float, *,
                   continuous_frac: float = 0.7,
                   rule_cap_kw: float = FSAE_TRACTIVE_POWER_CAP_KW,
                   n: int = 240) -> MotorEnvelope:
    """Build the motor operating envelope from the three datasheet numbers, find
    the base speed, sample power/torque over rpm, and apply the FSAE power cap +
    a thermal continuous limit.

    continuous_frac: the fraction of peak power the cooling lets you sustain
    indefinitely (0.7 is a typical planning value; the cooling-rig analysis
    refines it). It is always ≤ 1.0 — continuous can never exceed peak.
    """
    peak_torque_nm = max(float(peak_torque_nm), 1.0)
    peak_power_kw = max(float(peak_power_kw), 0.1)
    redline_rpm = max(float(redline_rpm), 1000.0)
    continuous_frac = max(0.1, min(1.0, float(continuous_frac)))

    # honour the FSAE ceiling: the *usable* peak can't exceed the cap
    over_cap = peak_power_kw > rule_cap_kw + 1e-6
    usable_peak_kw = min(peak_power_kw, rule_cap_kw)
    peak_power_w = usable_peak_kw * 1000.0

    # base speed: flat torque region ends where T*omega first equals peak power
    omega_base = peak_power_w / peak_torque_nm            # rad/s
    rpm_base = omega_base * 60.0 / (2.0 * math.pi)
    rpm_base = min(rpm_base, redline_rpm * 0.98)

    rpm = np.linspace(0.0, redline_rpm, n)
    omega = rpm * 2.0 * math.pi / 60.0
    # torque: flat peak below base speed, constant-power hyperbola above
    torque = np.where(rpm <= rpm_base, peak_torque_nm,
                      peak_power_w / np.maximum(omega, 1e-3))
    torque = np.minimum(torque, peak_torque_nm)          # never above peak torque
    power_w = torque * omega
    power_w = np.minimum(power_w, peak_power_w)           # clamp to the cap
    power_kw = power_w / 1000.0

    cont_power_kw = usable_peak_kw * continuous_frac
    cont_torque = (cont_power_kw * 1000.0) / max(omega_base, 1e-3)
    cont_torque = min(cont_torque, peak_torque_nm)

    notes: List[str] = []
    if over_cap:
        notes.append(
            f"Declared peak {peak_power_kw:.0f} kW exceeds the FSAE "
            f"{rule_cap_kw:.0f} kW tractive cap — the car must electronically limit "
            f"power to {rule_cap_kw:.0f} kW. The envelope above is shown clamped.")
    notes.append(
        f"Base speed ≈ {rpm_base:.0f} rpm. Below it the motor is torque-limited "
        f"(flat {peak_torque_nm:.0f} N·m); above it it's power-limited "
        f"({usable_peak_kw:.0f} kW), torque falling to "
        f"{peak_power_w/max(redline_rpm*2*math.pi/60.0,1e-3):.0f} N·m at redline.")

    return MotorEnvelope(
        peak_torque_nm=peak_torque_nm, peak_power_kw=usable_peak_kw,
        redline_rpm=redline_rpm, base_speed_rpm=rpm_base,
        rpm=rpm, torque_nm=torque, power_kw=power_kw,
        continuous_power_kw=cont_power_kw, continuous_torque_nm=cont_torque,
        rule_cap_kw=rule_cap_kw, over_cap=over_cap, notes=notes)


@dataclass
class MythCheck:
    """A specific wrong assumption, whether the current numbers trigger it, and
    the correct statement. Designed to catch the exact confusions that show up in
    team chat before they become a design decision."""

    claim: str
    verdict: str          # "myth" | "depends" | "true"
    correction: str


def power_rpm_myth_checks(env: MotorEnvelope, *,
                          gear_final_drive: Optional[float] = None,
                          wheel_r_m: float = 0.228) -> List[MythCheck]:
    """Return the canonical myth-busters for the power/rpm/continuous confusion,
    answered against THIS motor envelope so the numbers are concrete."""
    checks: List[MythCheck] = []

    # Myth 1: capping power to 80 kW means limiting rpm to ~7k
    checks.append(MythCheck(
        claim="We must limit RPM (e.g. to ~7k) because motor power can't exceed 80 kW.",
        verdict="myth",
        correction=(
            f"No — power and rpm are separate limits. The motor hits "
            f"{env.peak_power_kw:.0f} kW at its base speed (~{env.base_speed_rpm:.0f} "
            f"rpm) and then holds that power all the way to the "
            f"{env.redline_rpm:.0f} rpm redline. Limiting power does not lower the "
            f"redline; the controller just stops torque rising past the point where "
            f"T×ω = {env.peak_power_kw:.0f} kW. RPM is set by redline and gearing.")))

    # Myth 2: peak 80 kW AND 80 kW continuous as two different things
    checks.append(MythCheck(
        claim="We can run peak 80 kW and also have a separate, higher continuous power.",
        verdict="myth",
        correction=(
            f"Continuous power is always ≤ peak. The {env.rule_cap_kw:.0f} kW FSAE "
            f"cap is the ceiling for BOTH. 'Continuous' "
            f"(~{env.continuous_power_kw:.0f} kW here) is what cooling lets you "
            f"sustain; 'peak' ({env.peak_power_kw:.0f} kW) is a short burst up to the "
            f"cap. You raise continuous by cooling better, not by exceeding the cap.")))

    # Contextual: what redline actually buys, given gearing
    if gear_final_drive:
        v_redline = (env.redline_rpm * 2 * math.pi / 60.0) * wheel_r_m / gear_final_drive * 3.6
        checks.append(MythCheck(
            claim="Redline choice is about the power cap.",
            verdict="depends",
            correction=(
                f"Redline ({env.redline_rpm:.0f} rpm) with your "
                f"{gear_final_drive:.2f}:1 final drive sets top speed "
                f"(~{v_redline:.0f} km/h), not the power cap. Pick redline/gearing "
                f"for the speed range you need; the 80 kW cap is enforced separately "
                f"by the controller.")))

    return checks


# --------------------------------------------------------------------------- #
#  Auto-generated DFMEA rows  (answers slide 9: "is DFMEA worth the time?")
# --------------------------------------------------------------------------- #
# The team is openly questioning whether DFMEAs are too time-consuming to be
# worth it. The answer is to stop hand-writing them: the powertrain tab already
# computes the evidence (chain tension, cooling margin, mount load, output
# torque), so it can pre-fill DFMEA rows with real numbers and the analysis that
# already justifies them. A member then edits real rows instead of a blank sheet.
def dfmea_rows_from_analysis(*, sprocket: Optional[SprocketDesign] = None,
                             cooling: Optional[CoolingOperatingPoint] = None,
                             output_torque_nm: Optional[float] = None,
                             mount_load_n: Optional[float] = None,
                             owner: str = "") -> List[dict]:
    """Build ready-to-edit DFMEA records (matching dfmea.DFMEARow.to_record keys)
    seeded from this tab's analysis, with the computed numbers written into the
    cause / detection / evidence fields so the row is defensible on creation."""
    rows: List[dict] = []

    def _row(**kw):
        base = dict(
            Subsystem="Drivetrain", **{"Item / Component": ""},
            **{"Function / Requirement": ""}, **{"Failure Mode": ""},
            **{"Effect of Failure": ""}, Severity=5,
            **{"Potential Cause / Mechanism": ""}, Occurrence=4,
            **{"Prevention Controls": ""}, **{"Detection Controls": ""},
            Detection=4, **{"Recommended Action": ""}, Owner=owner,
            **{"Due Date": ""}, Status="Open", **{"Evidence / Notes": ""})
        base.update(kw)
        base["RPN"] = int(base["Severity"]) * int(base["Occurrence"]) * int(base["Detection"])
        return base

    if sprocket is not None:
        rows.append(_row(
            Subsystem="Sprocket / Output Shaft",
            **{"Item / Component":
               f"Output-shaft sprocket ({sprocket.driven_sprocket_teeth}T, "
               f"{sprocket.chain_label})"},
            **{"Function / Requirement":
               f"Transmit {sprocket.peak_motor_torque_nm:.0f} N·m motor torque to "
               f"the wheels at {sprocket.actual_ratio:.2f}:1 without tooth or "
               "chain failure"},
            **{"Failure Mode": "Sprocket tooth shears / chain skips under peak torque"},
            **{"Effect of Failure": "Loss of drive, possible DNF, debris in driveline"},
            Severity=8, Occurrence=4, Detection=4,
            **{"Potential Cause / Mechanism":
               f"Tooth force {sprocket.tooth_force_n:.0f} N at peak torque exceeds "
               "tooth bending/shear capacity; under-spec material or thickness"},
            **{"Prevention Controls":
               "Size sprocket to the computed tooth force with margin; verify by FEA"},
            **{"Detection Controls":
               "FEA at the computed tooth force; chain-tension check; visual "
               "inspection of teeth after each session"},
            **{"Recommended Action":
               f"Run sprocket FEA at the {sprocket.tooth_force_n:.0f} N tooth force "
               "with ≥1.5 FoS; confirm chain rating exceeds "
               f"{sprocket.chain_tension_n:.0f} N tension"},
            **{"Evidence / Notes":
               f"Auto-seeded from gear panel: {sprocket.actual_ratio:.2f}:1, "
               f"tension {sprocket.chain_tension_n:.0f} N, link FEA report here"}))

    if cooling is not None:
        _sev = 8 if not cooling.adequate else 6
        rows.append(_row(
            Subsystem="Cooling",
            **{"Item / Component": f"Radiator + {cooling.fan_name}"},
            **{"Function / Requirement":
               f"Reject {cooling.heat_to_reject_w:.0f} W at the design air ΔT "
               f"({cooling.design_delta_t_c:.0f} °C) on the endurance duty cycle"},
            **{"Failure Mode": "Cooling package can't carry motor+pack heat"},
            **{"Effect of Failure":
               "Thermal derate or over-temp shutdown mid-endurance; DNF"},
            Severity=_sev, Occurrence=(6 if not cooling.adequate else 4), Detection=4,
            **{"Potential Cause / Mechanism":
               f"Operating airflow {cooling.flow_m3h:.0f} m³/h gives "
               f"{cooling.cooling_capacity_w:.0f} W capacity vs "
               f"{cooling.heat_to_reject_w:.0f} W needed "
               f"(margin {cooling.margin_w:+.0f} W); loop too restrictive"},
            **{"Prevention Controls":
               "Size loop restriction so the fan operating point clears the heat "
               "load; validate on the cooling test rig"},
            **{"Detection Controls":
               "Cooling test rig: measure flow + ΔT at representative heat load; "
               "log coolant temps in endurance"},
            **{"Recommended Action":
               ("Reduce loop restriction or add a second fan — currently "
                f"under-cooled by {-cooling.margin_w:.0f} W"
                if not cooling.adequate else
                "Validate the operating point on the test rig; confirm margin holds "
                "with a hot ambient")},
            **{"Evidence / Notes":
               f"Auto-seeded from cooling panel: op point {cooling.flow_m3h:.0f} m³/h "
               f"@ {cooling.static_pressure_pa:.0f} Pa; tie to test-rig log"}))

    if output_torque_nm is not None and mount_load_n is not None:
        rows.append(_row(
            Subsystem="Motor Mounting",
            **{"Item / Component": "Motor / diff mount to chassis"},
            **{"Function / Requirement":
               f"React {output_torque_nm:.0f} N·m drive torque "
               f"({mount_load_n:.0f} N peak) into the chassis without yielding"},
            **{"Failure Mode": "Mount yields / cracks under peak drive + bump load"},
            **{"Effect of Failure":
               "Motor/diff shifts, chain misalignment, possible loss of drive"},
            Severity=8, Occurrence=3, Detection=4,
            **{"Potential Cause / Mechanism":
               f"Peak mount reaction {mount_load_n:.0f} N exceeds bracket capacity; "
               "weld/fastener under-spec; fatigue from vibration"},
            **{"Prevention Controls":
               "FEA the mount at the peak reaction with a fatigue check; declare the "
               "load to chassis so they design their pickup for it"},
            **{"Detection Controls":
               "FEA report; torque-check fasteners; inspect welds after events"},
            **{"Recommended Action":
               f"FEA motor & diff mount at {mount_load_n:.0f} N with ≥1.5 FoS; "
               "confirm chassis pickup is designed for it (ledger mount-load check)"},
            **{"Evidence / Notes":
               "Auto-seeded from publish panel; link mount FEA + chassis confirmation"}))

    return rows
