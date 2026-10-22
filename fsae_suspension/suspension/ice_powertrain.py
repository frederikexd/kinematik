# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Combustion (ICE) powertrain: the same questions the EV layer answers, for an engine and a gearbox.

WHAT EACH EV FEATURE BECOMES
----------------------------
    EV                                   ICE (this module)
    motor torque curve                   engine torque curve (dyno, GT-POWER or Ricardo WAVE export), capped by the
                                         intake restrictor's choked air flow
    single reduction                     primary reduction x gear ratios x final drive, shift points, shift time
    launch / 75 m                        clutch launch at a held rpm, slip to lock-up, timed shifts with the drive cut
    final-drive sweep                    final-drive sweep with the whole gear set
    lap time                             the QSS lap solver (laptime.py) on the best-gear tractive envelope
    pack energy, can it finish           fuel per lap from the engine work, tank volume, endurance fuel, CO2
    regen and brake balance              engine braking on the overrun and how it moves the brake balance
    motor/inverter heat to coolant       engine heat to coolant as a declared share of the fuel energy
    electrical feasibility gate          restrictor, rule diameter, tank, fuel pump, injector duty, cooling, fuel
                                         limit, top speed at redline
    architecture comparison              engine-option comparison: 75 m, lap, fuel, CO2, mass
    CO2 for the efficiency event         litres of fuel x the rule's CO2 factor (and kWh x factor for an EV)

WHAT IS DECLARED, NOT COMPUTED
------------------------------
* The torque curve. KinematiK does not model the engine's breathing; the curve comes from a dyno or an engine
  code. The default is a REPRESENTATIVE restricted 600 cc inline-four shape, flagged as such everywhere.
* Brake thermal efficiency (BTE) is one number for the whole map, so fuel use is a planning estimate: part-load
  pumping losses are not resolved. Replace it with a BSFC map from the dyno when you have one.
* The share of fuel energy that reaches the coolant, the engine's overrun torque and its rotating inertia.
* Fuel properties (density, lower heating value, stoichiometric air-fuel ratio) are typical published values.

RULE NUMBERS USED (check them against the current year's rules before relying on them)
-------------------------------------------------------------------------------------
* Restrictor 20.0 mm for gasoline and 19.0 mm for E85 (Formula SAE rule IC.2.4.3 in the 2021 rules).
* CO2 factors 2.31 kg/L gasoline, 1.65 kg/L E85, 0.65 kg/kWh electric, and the 26 L/100 km combustion fuel
  limit, from the Formula SAE efficiency rules and the 2024 efficiency proposal. The efficiency SCORE needs the
  field's best values, so it is not computed here.

Units: SI unless the name says otherwise (mm, kW, g/s, L). Never raises from the public entry points that the UI
calls; the physics functions raise ValueError on inputs that have no meaning (negative diameter, empty curve).
"""
from __future__ import annotations

import copy
import io
import math
from dataclasses import dataclass, field, replace

import numpy as np

from .throttle_dynamics import compressible_mass_flow

G = 9.81
P_ISA = 101325.0        # Pa
T_ISA = 298.15          # K (25 C, the usual dyno reference)


# =========================================================================== #
#  Fuel and the rules that depend on it
# =========================================================================== #
@dataclass(frozen=True)
class Fuel:
    key: str
    name: str
    density_kg_l: float          # at 15 C
    lhv_mj_kg: float             # lower heating value
    afr_stoich: float            # stoichiometric air-fuel ratio, mass
    co2_kg_per_l: float          # efficiency-event CO2 factor
    restrictor_mm: float         # rule maximum
    source: str = ""


FUELS: dict[str, Fuel] = {
    "gasoline": Fuel("gasoline", "Gasoline (100 octane)", 0.745, 43.0, 14.7, 2.31, 20.0,
                     "density, LHV, AFR: typical published values; CO2 factor and restrictor: FSAE rules"),
    "e85": Fuel("e85", "E85", 0.781, 29.2, 9.8, 1.65, 19.0,
                "density, LHV, AFR: typical published values for E85; CO2 factor and restrictor: FSAE rules"),
}
CO2_KG_PER_KWH_EV = 0.65
FUEL_LIMIT_L_PER_100KM = 26.0          # combustion limit in the 2024 efficiency proposal; check the current rules
ENDURANCE_KM = 22.0


def fuel(key: str) -> Fuel:
    """The fuel record for a key: density kg/L, LHV MJ/kg, AFR by mass, CO2 kg/L, restrictor mm."""
    if key not in FUELS:
        raise ValueError(f"unknown fuel {key!r}; one of {sorted(FUELS)}")
    return FUELS[key]


def co2_ev_kg(energy_kwh: float) -> float:
    """CO2 the efficiency event charges an EV for its net pack energy."""
    return max(float(energy_kwh), 0.0) * CO2_KG_PER_KWH_EV


# =========================================================================== #
#  Restrictor: choked air flow and the power it can feed
# =========================================================================== #
def choked_mass_flow_kg_s(d_mm: float, p0_pa: float = P_ISA, t0_k: float = T_ISA, cd: float = 1.0) -> float:
    """Mass flow through a choked circular throat, delegated to the repository's compressible orifice flow
    (throttle_dynamics.compressible_mass_flow, the owner; physics_owners.py records this module as a delegate):
        m_dot = Cd A p0 sqrt(gamma / (R T0)) (2 / (gamma + 1))^((gamma + 1) / (2 (gamma - 1)))
    A 20 mm throat at 101.325 kPa and 298 K passes 0.0745 kg/s at Cd = 1."""
    if not (d_mm > 0 and p0_pa > 0 and t0_k > 0 and 0 < cd <= 1.0):
        raise ValueError("restrictor diameter, stagnation state and Cd must be positive (Cd <= 1)")
    area = math.pi * (d_mm / 1000.0) ** 2 / 4.0
    return compressible_mass_flow(area, p0_pa, 0.0, T_up_K=t0_k, Cd=cd)


def restrictor_power_ceiling_kw(fuel_key: str = "gasoline", d_mm: float | None = None, bte: float = 0.30,
                                cd: float = 0.95, p0_pa: float = P_ISA, t0_k: float = T_ISA) -> float:
    """Brake power the restricted air can feed: burned fuel is air / AFR_stoich (richer mixtures add fuel that
    does not burn), so P = m_dot_air / AFR_stoich x LHV x BTE. An upper bound at any rpm: below the choking rpm
    the engine breathes less than this."""
    f = fuel(fuel_key)
    d = f.restrictor_mm if d_mm is None else float(d_mm)
    if not 0 < bte < 1:
        raise ValueError("brake thermal efficiency must be between 0 and 1")
    m_air = choked_mass_flow_kg_s(d, p0_pa, t0_k, cd)
    return m_air / f.afr_stoich * f.lhv_mj_kg * 1e3 * bte


# =========================================================================== #
#  Engine
# =========================================================================== #
@dataclass
class EngineMap:
    """Crank torque against rpm. Beyond redline the engine makes nothing (the limiter)."""
    rpm: list[float]
    torque_nm: list[float]
    redline_rpm: float
    idle_rpm: float = 1300.0
    inertia_kgm2: float = 0.025        # crank, clutch and gearbox input reflected to the crank (declared estimate)
    overrun_torque_nm: float = 8.0     # drag torque on the overrun, positive magnitude (declared estimate)
    displacement_cc: float = 599.0
    name: str = ""
    source: str = "user"               # "user" | "dyno" | "engine code" | "representative"
    power_cap_kw: float | None = None  # a power ceiling applied at every rpm (set by restricted())

    def __post_init__(self):
        r = np.asarray(self.rpm, float)
        t = np.asarray(self.torque_nm, float)
        if r.size < 2 or r.size != t.size:
            raise ValueError("the torque curve needs at least two (rpm, torque) points of equal length")
        if np.any(np.diff(r) <= 0):
            raise ValueError("rpm must increase strictly along the curve")
        if self.redline_rpm <= r[0]:
            raise ValueError("redline must lie above the first rpm point")
        self._r, self._t = r, t

    def torque_at(self, rpm: float) -> float:
        """Crank torque, Nm, at rpm; 0 Nm beyond redline, capped by power_cap_kw when set."""
        if rpm > self.redline_rpm:
            return 0.0
        t = float(np.interp(rpm, self._r, self._t))      # flat below the first and beyond the last point
        if self.power_cap_kw is not None:
            t = min(t, self.power_cap_kw * 1e3 / max(rpm * 2 * math.pi / 60, 1e-6))
        return t

    def power_kw_at(self, rpm: float) -> float:
        """Crank power, kW, at rpm: torque (Nm) times crank speed."""
        return self.torque_at(rpm) * rpm * 2.0 * math.pi / 60.0 / 1e3

    def peak(self) -> dict:
        """Peak power (kW) and torque (Nm) and the rpm of each."""
        grid = np.linspace(self._r[0], min(self.redline_rpm, self._r[-1]), 400)
        p = np.array([self.power_kw_at(x) for x in grid])
        t = np.array([self.torque_at(x) for x in grid])
        return {"power_kw": float(p.max()), "power_rpm": float(grid[p.argmax()]),
                "torque_nm": float(t.max()), "torque_rpm": float(grid[t.argmax()])}

    def restricted(self, fuel_key: str = "gasoline", bte: float = 0.30, cd: float = 0.95,
                   d_mm: float | None = None) -> EngineMap:
        """This curve capped at the restrictor's power ceiling, T <= P_ceiling / omega at every rpm (not only at
        the curve's points, where linear interpolation along the hyperbola would overshoot). Power in kW, torque in Nm."""
        return replace(self, power_cap_kw=restrictor_power_ceiling_kw(fuel_key, d_mm, bte=bte, cd=cd),
                       source=self.source + " (restrictor-capped)")

    def restrictor_check(self, fuel_key: str = "gasoline", bte: float = 0.30, cd: float = 0.95,
                         d_mm: float | None = None) -> dict:
        """Peak power against the ceiling of the declared restrictor (the rule maximum when d_mm is None). Power in kW, from torque in Nm."""
        ceiling = restrictor_power_ceiling_kw(fuel_key, d_mm, bte=bte, cd=cd)
        pk = self.peak()
        return {"ceiling_kw": ceiling, "peak_kw": pk["power_kw"], "peak_rpm": pk["power_rpm"],
                "exceeds": pk["power_kw"] > ceiling * 1.0001, "bte": bte}

    @classmethod
    def representative_600_i4(cls) -> EngineMap:
        """A restricted 600 cc inline-four SHAPE (peak about 61 kW at 11,000 rpm, 58 Nm at 9,000 rpm). A
        placeholder until the team declares its dyno or engine-code curve; never a measurement."""
        return cls(rpm=[3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000, 11000, 12000, 13000, 13500],
                   torque_nm=[38, 42, 46, 50, 54, 57, 58, 57, 53, 47, 41, 38],
                   redline_rpm=13500.0, idle_rpm=1300.0, displacement_cc=599.0,
                   name="600 cc inline-four, restricted (representative)", source="representative")

    @classmethod
    def from_csv(cls, text: str, redline_rpm: float | None = None, **kw) -> EngineMap:
        """Read a dyno or engine-code export: a header row, then rpm and either torque (torque_nm, Nm, lbft)
        or power (power_kw, kW, hp). Units are taken from the header; a bare 'torque' or 'power' is Nm or kW."""
        rows = [r for r in io.StringIO(text.strip()).read().splitlines() if r.strip()]
        head = [h.strip().lower() for h in rows[0].replace(";", ",").replace("\t", ",").split(",")]
        data = np.array([[float(x) for x in r.replace(";", ",").replace("\t", ",").split(",")[:len(head)]]
                         for r in rows[1:]])
        i_rpm = next(i for i, h in enumerate(head) if "rpm" in h)
        rpm = data[:, i_rpm]
        tq = None
        for i, h in enumerate(head):
            if "lb" in h and "ft" in h:
                tq = data[:, i] * 1.3558179
            elif "torque" in h or h in ("nm", "tq"):
                tq = data[:, i]
            elif "hp" in h and tq is None:
                tq = data[:, i] * 745.69987 / np.maximum(rpm * 2 * math.pi / 60, 1e-6)
            elif ("power" in h or "kw" in h) and tq is None:
                tq = data[:, i] * 1e3 / np.maximum(rpm * 2 * math.pi / 60, 1e-6)
        if tq is None:
            raise ValueError("no torque or power column found (headers: rpm plus torque_nm, lbft, power_kw or hp)")
        order = np.argsort(rpm)
        return cls(rpm=list(rpm[order]), torque_nm=list(tq[order]),
                   redline_rpm=float(redline_rpm or rpm.max()), source=kw.pop("source", "dyno"), **kw)


# =========================================================================== #
#  Gearbox
# =========================================================================== #
@dataclass
class Gearbox:
    """Overall ratio of gear g = primary x ratios[g] x final_drive (engine turns per wheel turn)."""
    primary: float = 2.111
    ratios: tuple = (2.750, 2.000, 1.667, 1.444, 1.304, 1.208)
    final_drive: float = 3.0
    shift_time_s: float = 0.08          # drive cut per upshift (declared; electric or pneumatic shifter)
    efficiency: float = 0.92            # gearbox and chain, crank to wheel (declared)
    tire_radius_m: float = 0.225
    source: str = "representative: CBR600RR (2007+) primary and gear ratios; check your engine's service manual"

    def __post_init__(self):
        if not self.ratios or any(r <= 0 for r in self.ratios) or self.primary <= 0 or self.final_drive <= 0:
            raise ValueError("every ratio must be positive")
        if any(a <= b for a, b in zip(self.ratios, self.ratios[1:])):
            raise ValueError("gear ratios must fall from first gear to top gear")

    @property
    def n(self) -> int:
        """Number of gears (dimensionless)."""
        return len(self.ratios)

    def overall(self, gear: int) -> float:
        """Overall ratio of a gear, engine turns per wheel turn (dimensionless)."""
        return self.primary * self.ratios[gear] * self.final_drive

    def rpm_at(self, v_ms: float, gear: int) -> float:
        """Engine rpm at road speed v (m/s) in a gear."""
        return max(v_ms, 0.0) / self.tire_radius_m * self.overall(gear) * 60.0 / (2.0 * math.pi)

    def speed_at(self, rpm: float, gear: int) -> float:
        """Road speed, m/s, at an engine rpm in a gear."""
        return rpm * 2.0 * math.pi / 60.0 / self.overall(gear) * self.tire_radius_m

    def wheel_force(self, v_ms: float, gear: int, eng: EngineMap) -> float:
        """Drive force at the contact patch in this gear; below the first mapped rpm the clutch is slipping and
        the engine is held at that rpm, so the force is that rpm's."""
        rpm = max(self.rpm_at(v_ms, gear), float(eng._r[0]))
        return eng.torque_at(rpm) * self.overall(gear) * self.efficiency / self.tire_radius_m

    def best_gear(self, v_ms: float, eng: EngineMap) -> int:
        """Index of the gear with the most drive force (N) at road speed v (m/s)."""
        f = [self.wheel_force(v_ms, g, eng) for g in range(self.n)]
        return int(np.argmax(f))

    def envelope_force(self, v_ms: float, eng: EngineMap) -> float:
        """Best-gear drive force at the contact patch, N, at road speed v (m/s)."""
        return max(self.wheel_force(v_ms, g, eng) for g in range(self.n))

    def top_speed_at_redline(self, eng: EngineMap) -> float:
        """Road speed, m/s, at redline in top gear."""
        return self.speed_at(eng.redline_rpm, self.n - 1)

    def shift_speeds(self, eng: EngineMap) -> list[float]:
        """Upshift speed out of each gear but the last: where the next gear's force overtakes this one's, found
        by bisection, or this gear's redline speed if the next gear never overtakes before it. Speeds in m/s."""
        out = []
        for g in range(self.n - 1):
            v_red = self.speed_at(eng.redline_rpm, g)
            lo = max(self.speed_at(float(eng._r[0]), g), 0.1)

            def _diff(v):
                return self.wheel_force(v, g, eng) - self.wheel_force(v, g + 1, eng)
            if _diff(v_red * 0.999) > 0:            # this gear still pulls harder at redline: run to the limiter
                out.append(v_red)
                continue
            a, b = lo, v_red * 0.999
            if _diff(a) <= 0:
                out.append(a)
                continue
            for _ in range(80):
                m = 0.5 * (a + b)
                a, b = (m, b) if _diff(m) > 0 else (a, m)
            out.append(0.5 * (a + b))
        return out


# =========================================================================== #
#  Vehicle for the launch and the energy integral
# =========================================================================== #
@dataclass
class ICEVehicle:
    mass_kg: float = 300.0              # car with driver
    driven_weight_frac: float = 0.55    # static share on the driven (rear) axle
    cg_height_m: float = 0.28
    wheelbase_m: float = 1.55
    mu: float = 1.4                     # peak longitudinal friction
    cda_m2: float = 1.10
    cla_m2: float = 2.5
    rear_aero_frac: float = 0.5
    crr: float = 0.015
    rho: float = 1.225
    wheel_inertia_kgm2: float = 0.6     # all four wheels with tires and brake discs (declared estimate)

    def m_eff(self, gear_overall: float | None, eng: EngineMap | None, eff: float = 1.0, r: float = 0.225) -> float:
        """Translational mass plus rotating inertia reflected to the wheel; the engine counts only when it is
        coupled (clutch locked, gear engaged). Mass in kg, inertias in kg·m², radius in m."""
        j = self.wheel_inertia_kgm2
        if gear_overall is not None and eng is not None:
            j += eng.inertia_kgm2 * gear_overall ** 2 * eff
        return self.mass_kg + j / r ** 2

    def resistance_n(self, v: float) -> float:
        return 0.5 * self.rho * self.cda_m2 * v * v + self.crr * self.mass_kg * G


# =========================================================================== #
#  Launch and the 75 m
# =========================================================================== #
@dataclass
class LaunchResult:
    t_s: np.ndarray
    x_m: np.ndarray
    v_ms: np.ndarray
    a_ms2: np.ndarray
    rpm: np.ndarray
    gear: np.ndarray                    # 1-based, 0 during a shift
    wheel_force_n: np.ndarray
    traction_limit_n: np.ndarray
    slipping: np.ndarray
    shifts: list = field(default_factory=list)          # (t, v, from_gear, to_gear), 1-based
    fuel_g: float = 0.0
    clutch_heat_kj: float = 0.0
    distance_m: float = 75.0
    finished: bool = True
    warnings: list = field(default_factory=list)

    @property
    def time_s(self) -> float:
        return float(self.t_s[-1]) if self.finished else float("inf")

    @property
    def trap_speed_kmh(self) -> float:
        """Speed at the end of the run, km/h."""
        return float(self.v_ms[-1] * 3.6)


def simulate_launch(eng: EngineMap, gb: Gearbox, veh: ICEVehicle | None = None, *, fuel_key: str = "gasoline",
                    bte: float = 0.30, launch_rpm: float = 9000.0, clutch_capacity_nm: float = float("inf"),
                    distance_m: float = 75.0, dt: float = 1e-3, t_max_s: float = 20.0) -> LaunchResult:
    """Standing start: the engine is held at launch_rpm while the clutch slips (transmitting the lesser of the
    engine's torque there and the clutch capacity), locks when the wheel speed brings the gearbox input up to
    launch_rpm, and upshifts at the gearbox's shift speeds with the drive cut for shift_time_s. Each step:
        N_rear   = m g w_r + m a_prev h / L + 1/2 rho ClA v^2 x rear share
        F        = min(F_drive, mu N_rear)
        a        = (F - 1/2 rho CdA v^2 - Crr m g) / m_eff        (engine inertia counted only when coupled)
    Semi-implicit Euler, as engine.simulate_launch for the EV."""
    veh = veh or ICEVehicle()
    f = fuel(fuel_key)
    r = gb.tire_radius_m
    shift_v = gb.shift_speeds(eng)
    n_cap = int(t_max_s / dt) + 2
    T, X, V, A, RPM, GR, FW, FT, SL = (np.zeros(n_cap) for _ in range(9))
    v = x = t = a_prev = 0.0
    gear, slipping, shift_left, next_gear = 0, True, 0.0, 0
    fuel_kg, clutch_j, shifts, i = 0.0, 0.0, [], 0
    while x < distance_m and i < n_cap - 1:
        overall = gb.overall(gear)
        rpm_in = gb.rpm_at(v, gear)
        if slipping and rpm_in >= launch_rpm:
            slipping = False
        if shift_left > 0:
            f_drive, rpm_e, coupled, t_eng = 0.0, gb.rpm_at(v, next_gear), False, 0.0
            shift_left -= dt
            if shift_left <= 0:
                gear = next_gear
        else:
            if not slipping and gear < gb.n - 1 and (v >= shift_v[gear] or rpm_in >= eng.redline_rpm):
                shifts.append((t, v, gear + 1, gear + 2))
                if gb.shift_time_s > 0:
                    next_gear, shift_left = gear + 1, gb.shift_time_s
                    f_drive, rpm_e, coupled, t_eng = 0.0, rpm_in, False, 0.0
                else:                                        # instant shift: drive on in the next gear this step
                    gear += 1
                    overall = gb.overall(gear)
                    rpm_e = gb.rpm_at(v, gear)
                    t_eng = eng.torque_at(rpm_e)
                    f_drive, coupled = t_eng * overall * gb.efficiency / r, True
            elif slipping:
                rpm_e = launch_rpm
                t_eng = min(eng.torque_at(launch_rpm), clutch_capacity_nm)
                f_drive, coupled = t_eng * overall * gb.efficiency / r, False
            else:
                rpm_e = rpm_in
                t_eng = eng.torque_at(rpm_e)
                f_drive, coupled = t_eng * overall * gb.efficiency / r, True
        down_r = 0.5 * veh.rho * veh.cla_m2 * v * v * veh.rear_aero_frac
        n_r = veh.mass_kg * G * veh.driven_weight_frac + veh.mass_kg * a_prev * veh.cg_height_m / veh.wheelbase_m
        f_trac = veh.mu * max(n_r + down_r, 0.0)
        f_w = min(f_drive, f_trac)
        m_eff = veh.m_eff(overall if coupled else None, eng, gb.efficiency, r)
        a = (f_w - veh.resistance_n(v)) / m_eff
        if f_drive > 0 and gb.overall(gear) > 0:
            t_used = f_w * r / (overall * gb.efficiency)          # the driver feeds only what the tires take
            p_eng = t_used * rpm_e * 2 * math.pi / 60
            fuel_kg += p_eng / (bte * f.lhv_mj_kg * 1e6) * dt
            if slipping:
                w_in = rpm_in * 2 * math.pi / 60
                clutch_j += t_used * (launch_rpm * 2 * math.pi / 60 - w_in) * dt
        T[i], X[i], V[i], A[i], RPM[i] = t, x, v, a, rpm_e
        GR[i], FW[i], FT[i], SL[i] = (0 if shift_left > 0 else gear + 1), f_w, f_trac, slipping
        v = max(v + a * dt, 0.0)
        x += v * dt
        t += dt
        a_prev = a
        i += 1
    s = slice(0, max(i, 1))
    res = LaunchResult(T[s].copy(), X[s].copy(), V[s].copy(), A[s].copy(), RPM[s].copy(), GR[s].astype(int),
                       FW[s].copy(), FT[s].copy(), SL[s].astype(bool), shifts, fuel_kg * 1e3, clutch_j / 1e3,
                       distance_m, finished=x >= distance_m)
    if not res.finished:
        res.warnings.append(f"did not reach {distance_m:.0f} m in {t_max_s:.0f} s; check the curve and the ratios")
    if eng.source.startswith("representative"):
        res.warnings.append("the torque curve is a representative shape, not your engine's dyno curve")
    return res


@dataclass
class FinalDriveSweep:
    ratios: np.ndarray
    times_s: np.ndarray
    trap_kmh: np.ndarray
    top_speed_kmh: np.ndarray
    best_ratio: float
    best: LaunchResult

    def as_table(self) -> list:
        """Rows of final drive, 75 m time (s), trap speed and top speed (km/h)."""
        return [{"final_drive": float(r), "t_75m_s": float(t), "trap_kmh": float(v), "top_speed_kmh": float(s)}
                for r, t, v, s in zip(self.ratios, self.times_s, self.trap_kmh, self.top_speed_kmh)]


def sweep_final_drive(eng: EngineMap, gb: Gearbox, veh: ICEVehicle | None = None, ratios=None,
                      **launch_kw) -> FinalDriveSweep:
    """The 75 m time for each final drive with the whole gear set, and the top speed at redline in top gear."""
    ratios = np.asarray(ratios if ratios is not None else np.linspace(1.8, 4.2, 25), float)
    times, trap, top, best, best_t, best_r = [], [], [], None, float("inf"), float(ratios[0])
    for fd in ratios:
        g2 = replace(gb, final_drive=float(fd))
        res = simulate_launch(eng, g2, veh, **launch_kw)
        times.append(res.time_s)
        trap.append(res.trap_speed_kmh)
        top.append(g2.top_speed_at_redline(eng) * 3.6)
        if res.time_s < best_t:
            best_t, best, best_r = res.time_s, res, float(fd)
    return FinalDriveSweep(ratios, np.asarray(times), np.asarray(trap), np.asarray(top), best_r, best)


# =========================================================================== #
#  Lap time: the QSS solver on the best-gear envelope
# =========================================================================== #
def lap_powertrain(eng: EngineMap, gb: Gearbox, veh: ICEVehicle | None = None, **pt_kw):
    """A laptime.Powertrain whose drive force is the best-gear envelope of this engine and gearbox. The QSS solver
    accelerates the bare mass (a = F / m), so the force handed to it is F m / m_eff: the same acceleration the
    car gets once the engine's and the wheels' inertia in that gear are counted. In first gear the engine's
    inertia alone adds about a third to the mass, which is why this matters for an ICE and barely for an EV."""
    from . import laptime as lt
    veh = veh or ICEVehicle()

    @dataclass
    class ICELapPowertrain(lt.Powertrain):
        engine: EngineMap | None = None
        gearbox: Gearbox | None = None

        def tractive_force(self, v_ms: float) -> float:
            """Drive force handed to the QSS solver, N, at road speed v (m/s)."""
            v = max(v_ms, 0.05)
            g = self.gearbox.best_gear(v, self.engine)
            m_eff = veh.m_eff(self.gearbox.overall(g), self.engine, self.gearbox.efficiency,
                              self.gearbox.tire_radius_m)
            return self.gearbox.envelope_force(v, self.engine) * veh.mass_kg / m_eff

    pk = eng.peak()["power_kw"]
    base = dict(power_kw=pk, drivetrain_eff=gb.efficiency, cda=veh.cda_m2, cla=veh.cla_m2, rho=veh.rho,
                crr=veh.crr, drive="rwd")
    base.update(pt_kw)
    return ICELapPowertrain(engine=eng, gearbox=gb, **base)


@dataclass
class ICELapResult:
    lap: object                          # laptime.LapResult
    shifts: int
    shift_penalty_s: float
    lap_time_s: float                    # QSS lap plus the first-order shift penalty
    gear_trace: list
    warnings: list = field(default_factory=list)


def simulate_lap(vdyn, track, eng: EngineMap, gb: Gearbox, veh: ICEVehicle | None = None) -> ICELapResult:
    """QSS lap (laptime.simulate_lap, the solver the EV lap uses) on the best-gear envelope, plus a first-order
    shift penalty. QSS cannot cut the drive, so each upshift is charged separately: while the drive is cut for
    t_s the car loses a t_s of speed, and that deficit costs about L a t_s / v^2 over the L metres left before
    the next braking point. Times in s, distances in m, speeds in m/s."""
    from . import laptime as lt
    veh = veh or ICEVehicle()
    lap = lt.simulate_lap(vdyn, track, lap_powertrain(eng, gb, veh))
    if not lap.ok or len(lap.v) < 2:
        return ICELapResult(lap, 0, 0.0, lap.lap_time_s, [], [lap.warning or "lap failed"])
    s, v = np.asarray(lap.s), np.asarray(lap.v)
    gears = [gb.best_gear(x, eng) for x in v]
    dv2 = np.diff(v * v)
    accel = np.append(dv2 / (2 * np.maximum(np.diff(s), 1e-6)), 0.0)
    n_shift, pen = 0, 0.0
    for k in range(1, len(v)):
        if gears[k] > gears[k - 1] and accel[k] > 0:
            n_shift += 1
            j = k
            while j < len(v) - 1 and accel[j] > 0:
                j += 1
            L = s[j] - s[k]
            pen += min(gb.shift_time_s, L * accel[k] * gb.shift_time_s / max(v[k] ** 2, 1.0))
    w = []
    if eng.source.startswith("representative"):
        w.append("the torque curve is a representative shape, not your engine's dyno curve")
    if float(np.max(v)) > gb.top_speed_at_redline(eng) * 0.999:
        w.append("the car reaches the limiter in top gear on this track; a longer final drive may help")
    return ICELapResult(lap, n_shift, pen, lap.lap_time_s + pen, [g + 1 for g in gears], w)


# =========================================================================== #
#  Fuel, endurance, CO2 and the heat that reaches the coolant
# =========================================================================== #
@dataclass
class FuelResult:
    fuel_kg: float
    fuel_l: float
    co2_kg: float
    engine_work_mj: float
    lap_time_s: float
    lap_m: float
    peak_fuel_g_s: float
    mean_coolant_heat_w: float
    peak_coolant_heat_w: float
    l_per_100km: float
    warnings: list = field(default_factory=list)


def fuel_for_lap(lap, eng: EngineMap, gb: Gearbox, veh: ICEVehicle | None = None, *, fuel_key: str = "gasoline",
                 bte: float = 0.30, overrun_fuel_g_s: float = 0.05, coolant_frac: float = 0.28) -> FuelResult:
    """Fuel for one lap from its speed trace. Where the car accelerates or holds speed, the engine supplies
        P_engine = (m_eff a + 1/2 rho CdA v^2 + Crr m g) v / eta_gearbox
    and burns P_engine / (BTE x LHV); off the throttle it burns overrun_fuel_g_s. The coolant takes coolant_frac of
    the fuel's chemical power (typically a quarter to a third for a spark-ignition engine at load)."""
    veh = veh or ICEVehicle()
    f = fuel(fuel_key)
    s, v = np.asarray(lap.s, float), np.asarray(lap.v, float)
    if s.size < 2:
        raise ValueError("the lap has no speed trace")
    ds = np.diff(s)
    vm = 0.5 * (v[1:] + v[:-1])
    dt = ds / np.maximum(vm, 0.05)
    a = np.diff(v * v) / (2 * np.maximum(ds, 1e-6))
    m_eff = np.array([veh.m_eff(gb.overall(gb.best_gear(x, eng)), eng, gb.efficiency, gb.tire_radius_m) for x in vm])
    p_wheel = (m_eff * a + 0.5 * veh.rho * veh.cda_m2 * vm ** 2 + veh.crr * veh.mass_kg * G) * vm
    p_wheel = np.minimum(p_wheel, np.array([gb.envelope_force(x, eng) for x in vm]) * vm)   # never above the curve
    on = p_wheel > 0
    p_eng = np.where(on, p_wheel / gb.efficiency, 0.0)
    fuel_rate = np.where(on, p_eng / (bte * f.lhv_mj_kg * 1e6), overrun_fuel_g_s / 1e3)      # kg/s
    fuel_kg = float(np.sum(fuel_rate * dt))
    chem_w = fuel_rate * f.lhv_mj_kg * 1e6
    t_lap = float(np.sum(dt))
    fuel_l = fuel_kg / f.density_kg_l
    lap_m = float(s[-1] - s[0])
    w = []
    if eng.source.startswith("representative"):
        w.append("fuel rests on a declared BTE and a representative curve; replace with a BSFC map from the dyno")
    return FuelResult(fuel_kg, fuel_l, fuel_l * f.co2_kg_per_l, float(np.sum(p_eng * dt)) / 1e6, t_lap, lap_m,
                      float(fuel_rate.max() * 1e3), float(np.sum(chem_w * dt) / t_lap * coolant_frac),
                      float(chem_w.max() * coolant_frac), fuel_l / max(lap_m, 1e-6) * 1e5, w)


@dataclass
class EnduranceFuel:
    laps: float
    fuel_l: float
    co2_kg: float
    l_per_100km: float
    tank_l: float | None
    tank_margin_l: float | None
    within_fuel_limit: bool


def endurance_fuel(per_lap: FuelResult, distance_km: float = ENDURANCE_KM, tank_l: float | None = None,
                   fuel_key: str = "gasoline", reserve_frac: float = 0.10) -> EnduranceFuel:
    """Endurance distance on the lap's fuel, with a reserve (fuel slosh and unusable tank volume). Litres, kg CO2, L/100 km."""
    laps = distance_km * 1e3 / max(per_lap.lap_m, 1e-6)
    fl = per_lap.fuel_l * laps
    margin = None if tank_l is None else tank_l - fl * (1 + reserve_frac)
    l100 = fl / distance_km * 100
    return EnduranceFuel(laps, fl, fl * fuel(fuel_key).co2_kg_per_l, l100, tank_l, margin,
                         l100 <= FUEL_LIMIT_L_PER_100KM)


# =========================================================================== #
#  Engine braking and the brake balance (the ICE side of regen)
# =========================================================================== #
def engine_braking_force_n(v_ms: float, eng: EngineMap, gb: Gearbox, gear: int | None = None) -> float:
    """Rear drag force from the overrun torque in the gear the car is in (best gear at this speed by default)."""
    g = gb.best_gear(v_ms, eng) if gear is None else gear
    return eng.overrun_torque_nm * gb.overall(g) / gb.tire_radius_m


def brake_balance_with_engine_braking(decel_g: float, bias_front: float, v_ms: float, eng: EngineMap,
                                      gb: Gearbox, veh: ICEVehicle | None = None, gear: int | None = None) -> dict:
    """The mechanical brakes supply what engine braking does not, split by the declared bias; the engine adds
    its drag to the rear. Returns the effective front share of the total braking force. Forces in N, deceleration in g."""
    veh = veh or ICEVehicle()
    f_total = decel_g * G * veh.mass_kg
    f_eb = min(engine_braking_force_n(v_ms, eng, gb, gear), f_total)
    f_mech = f_total - f_eb
    front = bias_front * f_mech
    return {"total_n": f_total, "engine_braking_n": f_eb, "front_n": front, "rear_n": f_total - front,
            "effective_front_share": front / max(f_total, 1e-9), "declared_bias": bias_front}


# =========================================================================== #
#  Cooling: the engine's heat into the existing loop sizing
# =========================================================================== #
def size_cooling(fuel_res: FuelResult, *, flow_lpm: float = 25.0, ambient_c: float = 35.0,
                 coolant_in_c: float = 85.0, max_coolant_c: float = 105.0, radiator_ua_w_per_k: float = 700.0,
                 loop_volume_l: float = 2.5, cap_pressure_bar: float = 1.1, coolant_key: str = "eg50",
                 use_peak: bool = False):
    """cooling.size_loop with the engine's lap-mean (or peak) coolant heat and engine-side temperatures. Heat in W, temperatures in °C, flow in L/min, UA in W/K."""
    from .cooling import LoopSpec, size_loop
    heat = fuel_res.peak_coolant_heat_w if use_peak else fuel_res.mean_coolant_heat_w
    return size_loop(LoopSpec(heat_w=heat, coolant_key=coolant_key, flow_lpm=flow_lpm, ambient_c=ambient_c,
                              coolant_in_c=coolant_in_c, max_coolant_c=max_coolant_c,
                              radiator_ua_w_per_k=radiator_ua_w_per_k, loop_volume_l=loop_volume_l,
                              cap_pressure_bar=cap_pressure_bar))


# =========================================================================== #
#  Feasibility gate (the ICE side of ev_electrical_check)
# =========================================================================== #
@dataclass
class Finding:
    check: str
    status: str          # PASS | FAIL | WARN | MISSING
    detail: str


def feasibility_gate(eng: EngineMap, gb: Gearbox, per_lap: FuelResult, *, fuel_key: str = "gasoline",
                     bte: float = 0.30, restrictor_mm: float | None = None, tank_l: float | None = None,
                     fuel_pump_lph: float | None = None, injector_cc_min: float | None = None,
                     n_injectors: int = 4, lap_top_speed_ms: float | None = None, cooling=None) -> list[Finding]:
    """Every check a combustion car can fail before it turns a wheel. An undeclared input is MISSING, never a
    pass."""
    f = fuel(fuel_key)
    out: list[Finding] = []
    if restrictor_mm is None:
        out.append(Finding("restrictor diameter", "MISSING", f"declare it; the rule maximum for {f.name} is "
                                                             f"{f.restrictor_mm:.1f} mm"))
    elif restrictor_mm > f.restrictor_mm + 1e-9:
        out.append(Finding("restrictor diameter", "FAIL", f"{restrictor_mm:.2f} mm exceeds the {f.restrictor_mm:.1f} mm "
                                                          f"maximum for {f.name}"))
    else:
        out.append(Finding("restrictor diameter", "PASS", f"{restrictor_mm:.2f} mm within {f.restrictor_mm:.1f} mm"))
    rc = eng.restrictor_check(fuel_key, bte, d_mm=restrictor_mm)
    out.append(Finding("curve against the restrictor", "WARN" if rc["exceeds"] else "PASS",
                       f"peak {rc['peak_kw']:.1f} kW at {rc['peak_rpm']:.0f} rpm against a {rc['ceiling_kw']:.1f} kW "
                       f"ceiling at BTE {bte:.2f}" + ("; the curve is unrestricted or the BTE is low" if rc["exceeds"]
                                                       else "")))
    end = endurance_fuel(per_lap, tank_l=tank_l, fuel_key=fuel_key)
    if tank_l is None:
        out.append(Finding("fuel tank", "MISSING", f"endurance needs {end.fuel_l:.2f} L plus 10% reserve; declare "
                                                   "the usable tank volume"))
    else:
        out.append(Finding("fuel tank", "PASS" if end.tank_margin_l >= 0 else "FAIL",
                           f"{end.fuel_l:.2f} L for {ENDURANCE_KM:.0f} km plus 10% reserve against {tank_l:.2f} L "
                           f"(margin {end.tank_margin_l:+.2f} L)"))
    out.append(Finding("fuel limit", "PASS" if end.within_fuel_limit else "WARN",
                       f"{end.l_per_100km:.1f} L/100 km against {FUEL_LIMIT_L_PER_100KM:.0f} L/100 km (2024 "
                       "efficiency proposal; check the current rules); CO2 "
                       f"{end.co2_kg:.2f} kg"))
    peak_lph = per_lap.peak_fuel_g_s * 3.6 / f.density_kg_l
    if fuel_pump_lph is None:
        out.append(Finding("fuel pump", "MISSING", f"peak demand {peak_lph:.1f} L/h; declare the pump's flow at "
                                                   "regulated pressure"))
    else:
        out.append(Finding("fuel pump", "PASS" if fuel_pump_lph >= 1.2 * peak_lph else "FAIL",
                           f"{fuel_pump_lph:.0f} L/h against {peak_lph:.1f} L/h peak x 1.2"))
    if injector_cc_min is None:
        out.append(Finding("injector duty", "MISSING", "declare the injector flow (cc/min at rail pressure)"))
    else:
        cap_g_s = injector_cc_min / 60.0 * f.density_kg_l * n_injectors
        duty = per_lap.peak_fuel_g_s / max(cap_g_s, 1e-9)
        out.append(Finding("injector duty", "PASS" if duty <= 0.85 else "FAIL",
                           f"{duty:.0%} at peak demand (limit 85%)"))
    vmax = gb.top_speed_at_redline(eng)
    if lap_top_speed_ms is not None:
        out.append(Finding("top gear", "PASS" if vmax >= lap_top_speed_ms else "WARN",
                           f"redline in top gear at {vmax * 3.6:.0f} km/h; the lap reaches {lap_top_speed_ms * 3.6:.0f} "
                           "km/h"))
    if cooling is None:
        out.append(Finding("cooling", "MISSING", "size the loop with size_cooling()"))
    else:
        out.append(Finding("cooling", "PASS" if cooling.ok else "FAIL",
                           f"{cooling.spec.heat_w / 1e3:.1f} kW to coolant; radiator UA margin {cooling.ua_margin:.2f}, "
                           f"steady coolant {cooling.steady_coolant_c:.0f} C"))
    if eng.source.startswith("representative"):
        out.append(Finding("torque curve", "WARN", "representative shape; declare the dyno or engine-code curve"))
    return out


# =========================================================================== #
#  Engine-option comparison (the ICE side of the EV architecture comparison)
# =========================================================================== #
@dataclass
class EngineOption:
    name: str
    engine: EngineMap
    gearbox: Gearbox
    mass_delta_kg: float = 0.0          # against the base vehicle
    fuel_key: str = "gasoline"
    bte: float = 0.30


def compare_engines(options: list[EngineOption], vdyn, track, veh: ICEVehicle | None = None) -> list[dict]:
    """75 m, lap and endurance fuel for each option on the same car and track; mass deltas reach both the launch
    and the lap. Never raises: a failed option carries its error."""
    veh = veh or ICEVehicle()
    rows = []
    for o in options:
        try:
            v2 = replace(veh, mass_kg=veh.mass_kg + o.mass_delta_kg)
            vd2 = copy.copy(vdyn)
            vd2.p = replace(vdyn.p, mass=vdyn.p.mass + o.mass_delta_kg)
            launch = simulate_launch(o.engine, o.gearbox, v2, fuel_key=o.fuel_key, bte=o.bte)
            lap = simulate_lap(vd2, track, o.engine, o.gearbox, v2)
            fr = fuel_for_lap(lap.lap, o.engine, o.gearbox, v2, fuel_key=o.fuel_key, bte=o.bte)
            end = endurance_fuel(fr, fuel_key=o.fuel_key)
            rows.append({"option": o.name, "t_75m_s": launch.time_s, "lap_s": lap.lap_time_s,
                         "endurance_fuel_l": end.fuel_l, "co2_kg": end.co2_kg, "mass_kg": v2.mass_kg,
                         "peak_kw": o.engine.peak()["power_kw"], "error": ""})
        except Exception as e:                       # noqa: BLE001 - surfaced per option
            rows.append({"option": o.name, "error": str(e)})
    return rows


def publish_to_ledger(ledger, eng: EngineMap, fuel_res: FuelResult | None = None,
                      updated_by: str = "suspension.ice_powertrain"):
    """Peak torque and power to the powertrain interface, coolant heat to cooling; never raises into the UI. Torque in Nm, power in kW, heat in W."""
    try:
        from . import interfaces as ifm
        if ledger is None or not hasattr(ledger, "get"):
            return ledger
        pk = eng.peak()
        it = ledger.get("powertrain") or ifm.SubsystemInterface(name="powertrain")
        it.peak_torque_nm, it.peak_power_kw = pk["torque_nm"], pk["power_kw"]
        it.updated_by, it.is_estimate = updated_by, True
        setter = getattr(ledger, "set", None) or getattr(ledger, "put", None)
        if setter:
            setter(it)
        if fuel_res is not None:
            c = ledger.get("cooling") or ifm.SubsystemInterface(name="cooling")
            c.heat_reject_w, c.updated_by, c.is_estimate = fuel_res.mean_coolant_heat_w, updated_by, True
            if setter:
                setter(c)
    except Exception:                                # noqa: BLE001 - the ledger is optional here
        pass
    return ledger
