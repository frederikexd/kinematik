"""
ev_electrical_check.py — Electrical feasibility gate for KinematiK
====================================================================
The electrics lead identified a real problem: the lap-time simulator can produce
target times that are *physically impossible* for the electrical system to support.
A 47-second autocross might look tight but achievable on paper — but if it demands
sustained current the fuse will blow in, the pack voltage collapses under, or the
pack simply runs dry before crossing the finish line, the lap doesn't happen.

This module bridges the gap.  Given:
  - A simulated lap result (speed profile vs. distance / time)
  - The electrical system parameters loaded from the electrics lead's Excel workbook

…it computes, at each point along the lap:
  1. Required mechanical power at the wheels          (P_wheel, kW)
  2. Required electrical power from the pack          (P_elec = P_wheel / η_drivetrain)
  3. Required pack current                            (I_pack = P_elec / V_pack, A)
  4. Required current per cell in parallel strings    (I_cell = I_pack / n_parallel, A)

Then it checks three hard limits from the Excel spec:
  ├─ Fuse limit:   peak I_pack  ≤  Fuse Max (A)
  ├─ Cell limit:   peak I_cell  ≤  rated cell current (Capacity / nominal discharge C)
  └─ Energy limit: total energy drawn  ≤  usable pack energy (kWh)

And computes the *maximum physically achievable lap time* given the fuse/current
cap — i.e. the lap time that the electrical system can actually sustain, not the
one the mechanical solver found.

Usage
-----
>>> from suspension.ev_electrical_check import ElecParams, check_lap_electrical
>>> params = ElecParams.from_excel("path/to/FSAE_EV_Power_Draw.xlsx")
>>> result = check_lap_electrical(lap_result, params, drivetrain_eff=0.90)
>>> if result.fuse_blown:
...     st.error(result.fuse_message)

All computation is pure Python / NumPy — no Streamlit import here so the module
is usable in unit tests and in the Streamlit UI alike.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from collections.abc import Sequence
import math

import numpy as np
from . import plausibility
from ._num import all_finite as _all_finite, finite as _finite, finite_scalar as _finite_scalar, vec as _vec

# ─────────────────────────────────────────────────────────────────────────────
# Data classes
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class ElecParams:
    """Electrical system parameters parsed from the electrics lead's Excel sheet."""

    # ── Pack topology ─────────────────────────────────────────────────────────
    pack_voltage_v: float = 504.0          # nominal pack voltage (V)
    pack_capacity_ah: float = 15.0         # pack capacity (Ah)
    n_parallel: int = 3                    # parallel cell strings
    n_series: int = 140                    # series cells per string
    cell_capacity_ah: float = 5.0          # individual cell capacity (Ah)
    cell_r_ohm: float = 0.0128            # cell internal resistance (Ω)

    # ── Limits ────────────────────────────────────────────────────────────────
    fuse_max_a: float = 50.0              # pack-level fuse rating (A)
    nominal_cell_current_a: float = 16.667  # rated current per cell (A) from Excel

    # ── Motor / inverter ─────────────────────────────────────────────────────
    motor_peak_power_kw: float = 150.0    # motor nameplate peak power (kW)
    motor_peak_torque_nm: float = 120.0   # motor nameplate peak torque (N·m)
    motor_efficiency: float = 0.9545      # motor efficiency (fraction)
    motor_max_dc_v: float = 925.0         # motor max DC bus voltage (V)
    wheel_diameter_in: float = 18.0       # wheel diameter (inches)

    # ── Derived / cached ─────────────────────────────────────────────────────
    # computed on post_init
    pack_energy_kwh: float = field(init=False)
    usable_energy_kwh: float = field(init=False)   # 92% usable by default
    max_power_from_fuse_kw: float = field(init=False)

    def __post_init__(self):
        # Pack energy: V × Ah  → Wh → kWh
        self.pack_energy_kwh = (self.pack_voltage_v * self.pack_capacity_ah) / 1000.0
        self.usable_energy_kwh = self.pack_energy_kwh * 0.92   # standard 92% usable
        # The fuse caps instantaneous pack current; convert to a power ceiling
        self.max_power_from_fuse_kw = (self.fuse_max_a * self.pack_voltage_v) / 1000.0

    # ── Factory ──────────────────────────────────────────────────────────────
    @classmethod
    def from_excel(cls, path: str) -> ElecParams:
        """
        Parse an ElecParams from the electrics lead's Excel workbook.

        Reads two sheets:
          'Battery Pack Calcs'  — pack topology, fuse, cell specs
          'ElecPropulsion'      — motor peak power, torque, efficiency, voltage, wheel size

        Falls back to sensible FSAE-EV defaults on any missing key.
        """
        try:
            import openpyxl  # type: ignore
        except ImportError:
            return cls()  # can't parse, return defaults

        try:
            wb = openpyxl.load_workbook(path, read_only=True, data_only=True)  # pragma: no mutate
        except Exception:
            return cls()

        # --- Pack sheet ---
        # Resolved by alias and then by content, NOT by an exact sheet name.
        # The old `if "Battery Pack Calcs" in wb.sheetnames` guard meant this
        # method silently returned its own dataclass defaults for any workbook
        # using the other spelling — and because those defaults happen to be
        # 504 V / 15 Ah / 140S3P, the output looked correct while ignoring the
        # file entirely, surviving every edit the team made to it.
        from .ev_excel_roundtrip import resolve_pack_sheet, resolve_ep_sheet

        pack: dict = {}
        ws_pack = resolve_pack_sheet(wb)
        if ws_pack is not None:
            for row in ws_pack.iter_rows(values_only=True):
                if row and row[0] and len(row) > 1 and row[1] is not None:
                    pack[str(row[0]).strip()] = row[1]

        # --- ElecPropulsion sheet (first 12 rows of labelled params) ---
        ep: dict = {}
        ws_ep = resolve_ep_sheet(wb)
        if ws_ep is not None:
            for i, row in enumerate(ws_ep.iter_rows(values_only=True)):
                if i >= 12:
                    break
                if row and row[0] and len(row) > 1 and row[1] is not None:
                    ep[str(row[0]).strip()] = row[1]

        def g(d, *keys, default=None):
            """Grab first matching key from dict, ignoring trailing spaces."""
            for k in keys:
                for dk in d:
                    if dk.strip().lower() == k.strip().lower():
                        v = d[dk]
                        try:
                            return float(v)
                        except (TypeError, ValueError):
                            return default
            return default

        n_par  = int(g(pack, "Parrallel Battery Count", "Parallel Battery Count") or 3)  # pragma: no mutate (labels compare case-blind)
        n_ser  = int(g(pack, "Series Battery Count") or 140)  # pragma: no mutate (labels compare case-blind)
        cell_c = g(pack, "Capacity Battery Cell (Ah)") or 5.0  # pragma: no mutate (labels compare case-blind)
        cell_r = g(pack, "Internal Resistance Battery Cell (Ohms)") or 0.0128  # pragma: no mutate (labels compare case-blind)
        v_pack = g(pack, "Battery Pack Nominal Voltage (V)") or (n_ser * 3.6)  # pragma: no mutate (labels compare case-blind)
        cap_ah = g(pack, "Pack Capacity (Ah)") or (cell_c * n_par)  # pragma: no mutate
        fuse   = g(pack, "Fuse Max (A)") or 50.0  # pragma: no mutate (labels compare case-blind)
        # The sheet's "Current Across a Single Cell" is the fuse current shared
        # across the parallel strings (the class default, 16.667 A, is 50 A / 3).
        # Fall back to that, not to pack Ah / strings: that is a 1C figure, a
        # different quantity, and flagged cell overcurrent at under a third of
        # the sheet's own limit whenever the row was blank.
        i_cell = g(pack, "Current Across a Single Cell (A)") or (fuse / max(n_par, 1))  # pragma: no mutate (labels compare case-blind)

        mot_pk_kw  = g(ep, "Motor Peak Power (kW)") or 150.0  # pragma: no mutate (labels compare case-blind)
        mot_pk_nm  = g(ep, "Motor Peak Torque (Nm)") or 120.0  # pragma: no mutate (labels compare case-blind)
        mot_eff    = g(ep, "Motor Efficiency") or 0.9545  # pragma: no mutate (labels compare stripped and case-blind)
        mot_v      = g(ep, "Motor Max DC Voltage (V)") or 925.0  # pragma: no mutate (labels compare case-blind)
        whl_in     = g(ep, "Wheel Diameter (in)") or 18.0  # pragma: no mutate (labels compare case-blind)

        return cls(
            pack_voltage_v=v_pack,
            pack_capacity_ah=cap_ah,
            n_parallel=n_par,
            n_series=n_ser,
            cell_capacity_ah=cell_c,
            cell_r_ohm=cell_r,
            fuse_max_a=fuse,
            nominal_cell_current_a=i_cell,
            motor_peak_power_kw=mot_pk_kw,
            motor_peak_torque_nm=mot_pk_nm,
            motor_efficiency=mot_eff,
            motor_max_dc_v=mot_v,
            wheel_diameter_in=whl_in,
        )

    @classmethod
    def from_excel_bytes(cls, file_bytes: bytes) -> ElecParams:
        """Parse from an in-memory file (e.g. st.file_uploader)."""
        import io
        try:
            import openpyxl
        except ImportError:
            return cls()
        try:
            wb = openpyxl.load_workbook(io.BytesIO(file_bytes), read_only=True, data_only=True)  # pragma: no mutate
        except Exception:
            return cls()
        # write to a temp file and re-use from_excel (simplest)
        import tempfile, os
        tmp = tempfile.NamedTemporaryFile(suffix=".xlsx", delete=False)  # pragma: no mutate
        tmp.write(file_bytes)
        tmp.close()
        try:
            result = cls.from_excel(tmp.name)
        finally:
            os.unlink(tmp.name)
        return result


@dataclass
class ElecCheckResult:
    """
    Full electrical feasibility report for one simulated lap.

    Flags
    -----
    fuse_blown       True if peak pack current exceeds the fuse rating
    cell_overcurrent True if any individual cell exceeds its rated current
    energy_empty     True if the lap drains the pack below the usable floor
    ok               True only when *all three* checks pass

    Numbers
    -------
    peak_current_a       Maximum pack current seen anywhere on lap (A)
    peak_current_speed   Speed at which that peak occurred (m/s)
    avg_current_a        Mean pack current over the traction phase (A)
    energy_used_kwh      Total electrical energy drawn from the pack this lap (kWh)
    usable_energy_kwh    Pack's usable energy ceiling for comparison (kWh)
    max_safe_speed_ms    Highest speed the electrical system can sustain continuously
    max_safe_power_kw    Fuse-limited power ceiling (kW)
    current_profile_a    np.ndarray — pack current at each lap sample (A)
    power_profile_kw     np.ndarray — electrical power at each lap sample (kW)
    speed_profile_ms     np.ndarray — speed at each sample (m/s)

    Messages
    --------
    fuse_message         Human-readable fuse verdict
    cell_message         Human-readable per-cell verdict
    energy_message       Human-readable energy verdict
    summary              One-line overall verdict
    warnings             List of additional engineering notes
    """

    # ── Flags ─────────────────────────────────────────────────────────────────
    ok: bool = True
    fuse_blown: bool = False
    cell_overcurrent: bool = False
    energy_empty: bool = False

    # ── Current / power numbers ────────────────────────────────────────────────
    peak_current_a: float = 0.0
    peak_current_speed_ms: float = 0.0
    avg_current_a: float = 0.0
    energy_used_kwh: float = 0.0
    usable_energy_kwh: float = 0.0
    max_safe_speed_ms: float = 0.0
    max_safe_power_kw: float = 0.0
    fuse_max_a: float = 50.0

    # ── Per-cell numbers ──────────────────────────────────────────────────────
    peak_cell_current_a: float = 0.0
    cell_current_limit_a: float = 0.0

    # ── Profiles (arrays) ─────────────────────────────────────────────────────
    current_profile_a: np.ndarray = field(default_factory=lambda: np.array([]))
    power_profile_kw: np.ndarray  = field(default_factory=lambda: np.array([]))
    speed_profile_ms: np.ndarray  = field(default_factory=lambda: np.array([]))

    # ── Messages ──────────────────────────────────────────────────────────────
    fuse_message: str = ""
    cell_message: str = ""
    energy_message: str = ""
    summary: str = ""
    warnings: list = field(default_factory=list)


# ─────────────────────────────────────────────────────────────────────────────
# Core computation
# ─────────────────────────────────────────────────────────────────────────────

def traction_power_w(v, a, *, vehicle_mass_kg: float = 230.0, drag_cda: float = 1.10,
                     crr: float = 0.018, rho_air: float = 1.225,
                     drivetrain_eff: float = 0.90, regen_eff: float = 0.0,
                     regen_cap_w: float = math.inf, rot_mass_factor: float = 1.0, aux_power_w: float = 0.0,
                     motor_eff_map=None, cla_m2: float = 0.0, tire: TireLosses | None = None, a_lat=None,
                     toe_rad=None):
    """Tractive force, wheel power and pack power along a speed profile.

        F       = k m a + ½ ρ CdA v² + Crr m g          (k: rotating-inertia factor)
        P_wheel = F v
        P_pack  = max(P_wheel, 0) / η  +  regen_eff · max(min(P_wheel, 0), −regen_cap_w)  +  P_aux

    When the deceleration is larger than the road load alone would give, the car
    is coasting or on the brakes. With regen_eff = 0 (the default) the motor
    draws nothing there; with regen_eff > 0 the braking power, up to regen_cap_w
    at the wheels, flows back into the pack at that efficiency (wheel to pack),
    and P_pack goes negative. aux_power_w is drawn all the time, moving or not
    (low-voltage system, pumps, fans). Returns (F_N, P_wheel_W, P_pack_W) as arrays.

    motor_eff_map, a pair (shaft power in W, motor-and-inverter efficiency), makes
    the drive efficiency depend on load, as a motor's does: driving, the pack then
    supplies P_shaft / η_motor(P_shaft) with P_shaft = P_wheel / η, the
    efficiency interpolated linearly in shaft power and held flat beyond the
    table (the way GT-SUITE and FASTSim read a motor map). η then covers the
    gearing alone; regen keeps its own regen_eff. A MotorEfficiencyMap instead
    reads the efficiency off the motor's speed-and-torque table, as suppliers
    give it.

    The tires carry the weight plus the downforce, Fz = m g + ½ ρ ClA v²
    (cla_m2, 0 by default), and rolling resistance is Crr Fz, or, with tire
    (a TireLosses, e.g. from a .tir file), the tire's own load- and
    speed-dependent rolling resistance. With a_lat (lateral acceleration, m/s²,
    one value per point) and tire, cornering costs the slip-angle drag
    F_y² / C_α (F_y = m a_lat, C_α the tires' cornering stiffness at Fz), which
    a straight-line road load leaves out. With toe_rad (each tire's toe angle as
    the car runs, rad) and tire, the tires also scrub against each other all
    the time: C_α,i tan δ_i sin δ_i, the alignment's share of the road load.
    """
    if not 0.0 < drivetrain_eff <= 1.0 or not 0.0 <= regen_eff <= 1.0:
        raise ValueError("drive and regeneration efficiencies are fractions, at most 1: was one typed in percent?")
    peak = None if motor_eff_map is None or isinstance(motor_eff_map, MotorEfficiencyMap) else max(motor_eff_map[0])
    plausibility.check({"mass_kg": vehicle_mass_kg, "cda_m2": drag_cda, "cla_m2": cla_m2,
                        "crr": None if tire is not None else crr, "drivetrain_eff": drivetrain_eff,
                        "aux_power_w": aux_power_w, "peak_power_w": peak}, where="traction_power_w")
    v = _vec(v)
    a = _vec(a)
    fz = vehicle_mass_kg * 9.81 + 0.5 * rho_air * cla_m2 * v ** 2
    rolling = crr * fz if tire is None else tire.rolling_force_n(fz, v)
    F = rot_mass_factor * vehicle_mass_kg * a + 0.5 * rho_air * drag_cda * v ** 2 + rolling
    if a_lat is not None:
        if tire is None:
            raise ValueError("cornering drag needs the tires' cornering stiffness: pass tire=TireLosses(...)")
        F = F + tire.cornering_drag_n(vehicle_mass_kg * _vec(a_lat), fz)
    if toe_rad is not None:
        if tire is None:
            raise ValueError("toe scrub needs the tires' cornering stiffness: pass tire=TireLosses(...)")
        F = F + tire.toe_drag_n(toe_rad, fz)
    P_wheel = F * v
    P_pack = np.maximum(P_wheel, 0.0) / max(drivetrain_eff, 0.50)
    if isinstance(motor_eff_map, MotorEfficiencyMap):
        P_pack = P_pack / motor_eff_map.efficiency_at(P_pack, v)
    elif motor_eff_map is not None:
        P_pack = P_pack / motor_efficiency(P_pack, motor_eff_map)
    if regen_eff:
        P_pack = P_pack + regen_eff * np.maximum(np.minimum(P_wheel, 0.0), -regen_cap_w)
    return F, P_wheel, P_pack + aux_power_w


def _bilinear(xg, yg, z, x, y):
    """z (rows along xg, columns along yg) at the points (x, y), bilinear, held flat beyond the grid."""
    x = np.clip(x, xg[0], xg[-1])
    y = np.clip(y, yg[0], yg[-1])
    # either searchsorted side, and the floor of 0 (x is already clipped), give the same value: the interpolation is
    # continuous across a grid line
    i = np.clip(np.searchsorted(xg, x, side="right") - 1, 0, xg.size - 2)  # pragma: no mutate
    j = np.clip(np.searchsorted(yg, y, side="right") - 1, 0, yg.size - 2)  # pragma: no mutate
    tx = (x - xg[i]) / (xg[i + 1] - xg[i])
    ty = (y - yg[j]) / (yg[j + 1] - yg[j])
    return (z[i, j] * (1 - tx) * (1 - ty) + z[i + 1, j] * tx * (1 - ty)
            + z[i, j + 1] * (1 - tx) * ty + z[i + 1, j + 1] * tx * ty)


@dataclass(frozen=True)
class MotorEfficiencyMap:
    """A motor's efficiency over speed and torque, the way suppliers tabulate it: efficiency[i][j] at rpm[i] and
    torque_nm[j], fractions in (0, 1]. n_motors identical motors share the drive equally, each through gear_ratio
    (motor turns per wheel turn) to wheels of wheel_radius_m."""
    rpm: tuple
    torque_nm: tuple
    efficiency: tuple
    n_motors: int = 1
    gear_ratio: float = 1.0
    wheel_radius_m: float = 0.225

    def __post_init__(self):
        n, t, e = _vec(self.rpm), _vec(self.torque_nm), _vec(self.efficiency)
        if n.ndim != 1 or t.ndim != 1 or n.size < 2 or t.size < 2 or e.shape != (n.size, t.size):
            raise ValueError("the motor map needs at least two speeds and two torques, and one efficiency per pair "
                             "(rows: speed, columns: torque)")
        if np.any(np.diff(n) <= 0) or np.any(np.diff(t) <= 0):
            raise ValueError("motor map speeds and torques must rise strictly")
        if np.any(e <= 0) or np.any(e > 1):
            raise ValueError("motor map efficiencies must lie in (0, 1]")
        if self.n_motors < 1 or self.gear_ratio <= 0 or self.wheel_radius_m <= 0:
            raise ValueError("motor count, gear ratio and wheel radius must be positive")

    def efficiency_at(self, p_shaft_w, v_ms):
        """Efficiency of each motor when they together deliver p_shaft_w (W) at road speed v_ms (m/s)."""
        w = np.abs(_vec(v_ms)) / self.wheel_radius_m * self.gear_ratio            # motor speed, rad/s
        torque = np.abs(_vec(p_shaft_w)) / (self.n_motors * np.maximum(w, 1e-9))
        return _bilinear(_vec(self.rpm), _vec(self.torque_nm), _vec(self.efficiency), w * 30.0 / math.pi, torque)


@dataclass(frozen=True)
class TireLosses:
    """Rolling resistance and cornering stiffness of the car's tires, in the terms of a Pacejka MF 6.1 .tir file
    (TireLosses.from_tir), at nominal pressure and no camber. Per tire, at load f_z and speed v:
        rolling force   F_r  = f_z (QSY1 + QSY3 |v / V0| + QSY4 (v / V0)^4) (f_z / Fz0)^(QSY7 - 1)
        cornering stiff C_α  = |PKY1| Fz0 sin(PKY4 atan(f_z / (PKY2 Fz0)))          (N/rad)
    n_tires share the load equally."""
    qsy1: float = 0.01
    qsy3: float = 0.0
    qsy4: float = 0.0
    qsy7: float = 1.0
    fz_nom_n: float = 1000.0
    v_ref_ms: float = 16.7
    pky1: float = -15.0
    pky2: float = 1.7
    pky4: float = 2.0
    n_tires: int = 4

    def __post_init__(self):
        if self.fz_nom_n <= 0 or self.v_ref_ms <= 0 or self.n_tires < 1 or self.pky2 <= 0:
            raise ValueError("nominal load, reference speed, PKY2 and the tire count must be positive")

    def rolling_force_n(self, fz_total_n, v_ms):
        """Rolling resistance of all the tires together (N) under a total load fz_total_n (N) at speed v_ms (m/s)."""
        fz = np.maximum(_vec(fz_total_n), 0.0) / self.n_tires
        u = _vec(v_ms) / self.v_ref_ms
        return self.n_tires * fz * (self.qsy1 + self.qsy3 * np.abs(u) + self.qsy4 * u ** 4) * (
            np.maximum(fz, 1e-9) / self.fz_nom_n) ** (self.qsy7 - 1.0)            # no load, no rolling force

    def cornering_stiffness_n_rad(self, fz_total_n):
        """Cornering stiffness of all the tires together (N/rad) under a total load fz_total_n (N)."""
        fz = np.maximum(_vec(fz_total_n), 0.0) / self.n_tires
        return self.n_tires * abs(self.pky1) * self.fz_nom_n * np.sin(
            self.pky4 * np.arctan(fz / (self.pky2 * self.fz_nom_n)))

    def cornering_drag_n(self, f_lat_n, fz_total_n):
        """Slip-angle drag (N) while the tires make the lateral force f_lat_n (N): F_y tan α ≈ F_y² / C_α, with the
        tires sharing the force in proportion to their stiffness."""
        c = np.maximum(self.cornering_stiffness_n_rad(fz_total_n), 1e-9)
        return _vec(f_lat_n) ** 2 / c

    def toe_drag_n(self, toe_rad, fz_total_n):
        """Scrub drag (N) of tires running at toe angles toe_rad (rad, one per tire, sign immaterial): each at slip
        angle δ makes C_α δ sideways, which costs C_α tan δ sin δ along the road; C_α per tire at an equal share of
        fz_total_n (N)."""
        toe = _vec(toe_rad)
        if toe.ndim != 1 or toe.size != self.n_tires:
            raise ValueError(f"give one toe angle per tire ({self.n_tires})")
        c = self.cornering_stiffness_n_rad(fz_total_n) / self.n_tires
        return c * float(np.sum(np.tan(toe) * np.sin(toe)))

    @classmethod
    def from_tir(cls, text: str, n_tires: int = 4) -> TireLosses:
        """From the text of a .tir file (keys in capitals, as the format writes them). Coefficients the file does not
        give keep the class defaults; LONGVL is the reference speed V0 (m/s) and FNOMIN the nominal load Fz0 (N)."""
        import re
        keys = {"QSY1": "qsy1", "QSY3": "qsy3", "QSY4": "qsy4", "QSY7": "qsy7", "FNOMIN": "fz_nom_n",
                "LONGVL": "v_ref_ms", "PKY1": "pky1", "PKY2": "pky2", "PKY4": "pky4"}
        kw = {}
        for key, name in keys.items():
            m = re.search(rf"(?m)^\s*{key}\s*=\s*([-+0-9.eE]+)", text)
            if m:
                kw[name] = _finite_scalar(f".tir {key}", m.group(1))
        if isinstance(n_tires, bool) or not isinstance(n_tires, int) or n_tires < 1:
            raise ValueError(f"n_tires must be a whole number of tires, got {n_tires!r}")
        return cls(n_tires=n_tires, **kw)


def motor_efficiency(p_shaft_w, eff_map):
    """Efficiency (dimensionless) at shaft power p_shaft_w (W) from a (power W, efficiency) table: linear between
    the points, flat beyond them. The table needs at least two points, power rising strictly, efficiency in (0, 1]."""
    pw, eta = (np.asarray(x, dtype=float) for x in eff_map)  # pragma: no mutate (np.interp takes ints too)
    if pw.ndim != 1 or pw.size < 2 or pw.size != eta.size:
        raise ValueError("the motor map needs at least two (power, efficiency) points of equal length")
    if np.any(np.diff(pw) <= 0):
        raise ValueError("motor map power must rise strictly")
    if np.any(eta <= 0) or np.any(eta > 1):
        raise ValueError("motor map efficiencies must lie in (0, 1]")
    return np.interp(np.abs(_vec(p_shaft_w)), pw, eta)


def pack_along_lap(p_terminal_w, t_s, *, ocv_soc, soc0: float, capacity_ah: float, r_pack_ohm: float,
                   r_soc_temp=None, heat_capacity_j_k: float | None = None, temp0_k: float = 298.15,
                   temps_out: list | None = None):
    """Terminal voltage (V), current (A) and state of charge along a lap for a pack whose open-circuit voltage falls
    as it discharges: ocv_soc is a (state of charge, V) table, read linearly. Each step draws p_terminal_w (W) at
    the terminals through the pack's resistance (as battery_internal_power_w, clipped at the most the pack can give)
    from the voltage at the charge left; the charge then drops by I dt / capacity. Returns (v_terminal_V, i_A, soc).

    The resistance is r_pack_ohm, or, with r_soc_temp = (states of charge, temperatures K, resistances Ω with one row
    per state of charge), read bilinearly at the charge and temperature of the moment. With heat_capacity_j_k
    (J/K) the pack starts at temp0_k and heats by I² R dt / C each step (no cooling: a lap is short against a
    pack's thermal time constant); temps_out, if a list, receives the temperature at every point (K)."""
    p, t = _vec(p_terminal_w), _vec(t_s)
    if p.shape != t.shape or p.size < 1:
        raise ValueError("one power per time point")
    if capacity_ah <= 0:
        raise ValueError("capacity must be positive")
    soc_pts, v_pts = (np.asarray(x, dtype=float) for x in ocv_soc)  # pragma: no mutate (np.interp takes ints)
    if soc_pts.size < 2 or soc_pts.size != v_pts.size or np.any(np.diff(soc_pts) <= 0):
        raise ValueError("the OCV table needs at least two (state of charge, V) points, charge rising strictly")
    if r_soc_temp is not None:
        rs, rt, rr = (np.asarray(x, dtype=float) for x in r_soc_temp)  # pragma: no mutate (bilinear takes ints)
        if rs.size < 2 or rt.size < 2 or rr.shape != (rs.size, rt.size):
            raise ValueError("the resistance table needs at least two charges and two temperatures, one value each")
    if heat_capacity_j_k is not None and heat_capacity_j_k <= 0:
        raise ValueError("heat capacity must be positive")
    dt = np.diff(t, prepend=t[0])
    soc = np.empty_like(p)
    i = np.empty_like(p)
    v = np.empty_like(p)
    q, temp, r = float(soc0), float(temp0_k), r_pack_ohm
    for k in range(p.size):
        if k:
            q -= i[k - 1] * dt[k] / (3600.0 * capacity_ah)
            if heat_capacity_j_k is not None:
                temp += i[k - 1] ** 2 * r * dt[k] / heat_capacity_j_k
        if r_soc_temp is not None:
            r = float(_bilinear(rs, rt, rr, q, temp))
        ocv = float(np.interp(q, soc_pts, v_pts))
        i[k] = battery_internal_power_w(p[k], ocv, r)[1]
        v[k] = ocv - i[k] * r
        soc[k] = q
        if temps_out is not None:
            temps_out.append(temp)
    return v, i, soc


def pack_temperature_along(i_a, t_s, *, r_pack_ohm: float, heat_capacity_j_k: float, temp0_k: float,
                           cooling_w_k: float = 0.0, ambient_k: float | None = None):
    """Pack temperature (K) along a log of pack current i_a (A) at times t_s (s): a lumped pack of heat capacity
    heat_capacity_j_k (J/K) heated by I² R and cooled by cooling_w_k (W/K) towards ambient_k (K):
        C dT/dt = I² R − hA (T − T_ambient)
    stepped forward from temp0_k (the current of each step heats over the step that follows it)."""
    i, t = _vec(i_a), _vec(t_s)
    if i.shape != t.shape or i.size < 1:
        raise ValueError("one current per time point")
    if heat_capacity_j_k <= 0 or cooling_w_k < 0:
        raise ValueError("heat capacity must be positive and cooling not negative")
    if cooling_w_k and ambient_k is None:
        raise ValueError("cooling needs an ambient temperature")
    dt = np.diff(t)
    temps = np.empty_like(i)
    temps[0] = temp0_k
    for k in range(1, i.size):
        loss = cooling_w_k * (temps[k - 1] - ambient_k) if cooling_w_k else 0.0
        temps[k] = temps[k - 1] + (i[k - 1] ** 2 * r_pack_ohm - loss) * dt[k - 1] / heat_capacity_j_k
    return temps


def battery_internal_power_w(p_terminal_w, ocv_v: float, r_pack_ohm: float):
    """Power the cells give up (positive) or take in (negative) for a terminal power, with the pack's I²R loss.

        V_oc I − I² R = P_terminal   →   I = (V_oc − √(V_oc² − 4 R P_terminal)) / (2 R)
        P_internal = V_oc I = P_terminal + I² R

    The loss is counted both ways: discharging costs more than the terminal power, charging stores less. A demand
    above the pack's maximum transferable power V_oc² / 4R is clipped to it (I = V_oc / 2R). Returns
    (P_internal_W, I_A).
    """
    p = _vec(p_terminal_w)
    if r_pack_ohm <= 0:
        return p.copy(), p / ocv_v
    disc = np.maximum(ocv_v * ocv_v - 4.0 * r_pack_ohm * p, 0.0)
    i = (ocv_v - np.sqrt(disc)) / (2.0 * r_pack_ohm)
    return ocv_v * i, i


def fuse_limited_speed(max_pack_power_w: float, *, vehicle_mass_kg: float = 230.0,
                       drag_cda: float = 1.10, crr: float = 0.018,
                       rho_air: float = 1.225, drivetrain_eff: float = 0.90) -> float:
    """Highest steady speed whose road load the fuse-limited pack power can carry.

    Solves ½ρCdA v³ + Crr·m·g·v = P_pack,max · η on a 0.1-100 m/s grid (100 m/s
    when even that is within the cap).
    """
    v_test = np.linspace(0.1, 100.0, 5000)
    P_test = (0.5 * rho_air * drag_cda * v_test ** 3
              + crr * vehicle_mass_kg * 9.81 * v_test)
    idx = np.searchsorted(P_test, max_pack_power_w * max(drivetrain_eff, 0.50))
    return float(v_test[min(idx, len(v_test) - 1)])


def check_lap_electrical(
    speed_ms: Sequence[float],
    distance_m: Sequence[float],
    params: ElecParams,
    *,
    drivetrain_eff: float = 0.90,
    vehicle_mass_kg: float = 230.0,
    drag_cda: float = 1.10,
    crr: float = 0.018,
    rho_air: float = 1.225,
) -> ElecCheckResult:
    """
    Check whether the simulated lap speed profile is electrically achievable.

    Parameters
    ----------
    speed_ms      : Speed at each sample point (m/s).  Can be a lap result's `.v`.
    distance_m    : Distance at each sample point (m). Can be a lap result's `.s`.
    params        : ElecParams parsed from the electrics lead's Excel workbook.
    drivetrain_eff: Combined inverter + motor + gearbox efficiency (fraction).
    vehicle_mass_kg: Total vehicle mass including driver (kg).
    drag_cda      : Aerodynamic drag area (m²).  Defaults to FSAE-typical value.
    crr           : Rolling resistance coefficient.
    rho_air       : Air density (kg/m³).

    Returns
    -------
    ElecCheckResult with all flags, numbers, profiles, and human-readable messages.
    """
    v = _vec(speed_ms)
    s = _vec(distance_m)

    if len(v) < 2:
        r = ElecCheckResult()
        r.ok = False
        r.summary = "Not enough speed data to run the electrical check."
        return r

    # ── Steps 1-3: tractive force, wheel power, pack power ────────────────────
    # a = v dv/ds. Under braking the deceleration outweighs the road load and
    # the motor draws nothing; the old clamp kept drag and rolling power on the
    # pack through every braking zone.
    a = v * np.gradient(v, s)
    eff = max(drivetrain_eff, 0.50)
    _F, _P_wheel_w, P_elec_w = traction_power_w(
        v, a, vehicle_mass_kg=vehicle_mass_kg, drag_cda=drag_cda, crr=crr,
        rho_air=rho_air, drivetrain_eff=eff)
    P_elec_kw = P_elec_w / 1000.0                     # kW drawn from pack

    # ── Step 4: pack current ──────────────────────────────────────────────────
    V_pack = params.pack_voltage_v
    I_pack = P_elec_w / max(V_pack, 1.0)              # A drawn from pack

    # ── Step 5: per-cell current ──────────────────────────────────────────────
    I_cell = I_pack / max(params.n_parallel, 1)       # A per cell

    # ── Step 6: energy integral (trapezoidal rule) ────────────────────────────
    # ΔE = P × Δt;  Δt = Δs / v_avg
    dt = np.zeros(len(v))
    for i in range(1, len(v)):
        v_avg = (v[i - 1] + v[i]) / 2.0
        ds = abs(s[i] - s[i - 1])
        dt[i] = ds / max(v_avg, 0.5)

    energy_per_step_wh = P_elec_w * dt / 3600.0      # Wh at each step
    total_energy_kwh = float(np.sum(energy_per_step_wh)) / 1000.0

    # ── Step 7: limits ────────────────────────────────────────────────────────
    peak_I     = float(np.max(I_pack))
    peak_speed = float(v[int(np.argmax(I_pack))])
    _traction_mask = I_pack > 0.1  # pragma: no mutate
    avg_I      = float(np.mean(I_pack[_traction_mask])) if _traction_mask.any() else 0.0

    peak_I_cell  = float(np.max(I_cell))
    fuse_limit   = params.fuse_max_a
    cell_limit   = params.nominal_cell_current_a

    fuse_blown       = peak_I > fuse_limit  # pragma: no mutate
    cell_overcurrent = peak_I_cell > cell_limit  # pragma: no mutate
    energy_empty     = total_energy_kwh > params.usable_energy_kwh

    # ── Step 8: maximum safe continuous speed (fuse-limited) ──────────────────
    # At constant speed: P_wheel = F_drag × v = ½ρCdA v³ + crr·mg·v
    # P_elec = P_wheel / eff  ≤  fuse_limit × V_pack
    max_safe_P_kw = params.max_power_from_fuse_kw
    max_safe_P_w  = max_safe_P_kw * 1000.0

    # Solve  ½ρCdA v³ + crr·mg·v  =  max_safe_P_w × eff   for v  (numerically)
    max_safe_speed_ms = fuse_limited_speed(
        max_safe_P_w, vehicle_mass_kg=vehicle_mass_kg, drag_cda=drag_cda, crr=crr,
        rho_air=rho_air, drivetrain_eff=eff)

    # ── Step 9: assemble result ───────────────────────────────────────────────
    ok = not (fuse_blown or cell_overcurrent or energy_empty)

    # Fuse message
    if fuse_blown:
        margin = peak_I - fuse_limit
        fuse_msg = (
            f"🚨 FUSE WILL BLOW — peak pack current {peak_I:.1f} A exceeds "
            f"the {fuse_limit:.0f} A fuse by {margin:.1f} A "
            f"(at {peak_speed * 3.6:.1f} km/h). "
            f"The lap asks the pack for {float(np.max(P_elec_kw)):.1f} kW at its peak; "
            f"the fuse actually caps you at {max_safe_P_kw:.1f} kW "
            f"(~{max_safe_speed_ms * 3.6:.1f} km/h sustained). "
            f"Slow your roll — or upsize the fuse with the electrics lead."
        )
    else:
        headroom = fuse_limit - peak_I
        fuse_msg = (
            f"✅ Fuse OK — peak {peak_I:.1f} A vs {fuse_limit:.0f} A limit "
            f"({headroom:.1f} A headroom)."
        )

    # Cell current message
    if cell_overcurrent:
        cell_msg = (
            f"⚠️ Cell overcurrent — each cell sees {peak_I_cell:.1f} A peak "
            f"(limit {cell_limit:.1f} A across {params.n_parallel} parallel strings). "
            f"This degrades pack life and risks thermal runaway."
        )
    else:
        cell_msg = (
            f"✅ Cell current OK — {peak_I_cell:.1f} A peak per cell "
            f"(rated {cell_limit:.1f} A)."
        )

    # Energy message
    if energy_empty:
        deficit = total_energy_kwh - params.usable_energy_kwh
        energy_msg = (
            f"🚨 PACK RUNS DRY — this lap draws {total_energy_kwh:.3f} kWh, "
            f"but only {params.usable_energy_kwh:.3f} kWh is usable "
            f"({deficit:.3f} kWh short). The car won't finish."
        )
    else:
        pct_used = (total_energy_kwh / params.usable_energy_kwh * 100.0
                    if params.usable_energy_kwh > 0 else 0.0)
        energy_msg = (
            f"✅ Energy OK — lap uses {total_energy_kwh:.3f} kWh "
            f"({pct_used:.1f}% of {params.usable_energy_kwh:.3f} kWh usable pack)."
        )

    # Summary
    issues = []
    if fuse_blown:
        issues.append("fuse over-current")
    if cell_overcurrent:
        issues.append("cell over-current")
    if energy_empty:
        issues.append("energy deficit")
    if ok:
        summary = (
            f"✅ Electrically feasible — lap is within fuse, cell, and energy limits. "
            f"Peak draw {peak_I:.1f} A / {max_safe_P_kw:.1f} kW fuse ceiling."
        )
    else:
        summary = (
            f"❌ Lap NOT electrically feasible — {', '.join(issues)}. "
            f"Fuse ceiling: {fuse_limit:.0f} A ({max_safe_P_kw:.1f} kW at {V_pack:.0f} V). "
            f"Estimated max sustainable speed: {max_safe_speed_ms * 3.6:.1f} km/h."
        )

    warnings_list = []
    if max_safe_speed_ms < float(np.max(v)):  # pragma: no mutate
        max_v_kmh = float(np.max(v)) * 3.6
        warnings_list.append(
            f"Top speed in the sim ({max_v_kmh:.1f} km/h) exceeds the fuse-limited "
            f"sustained ceiling ({max_safe_speed_ms * 3.6:.1f} km/h). "
            f"Brief bursts may be OK (capacitive support), but sustained speed is capped."
        )
    if params.pack_capacity_ah < 10.0:
        warnings_list.append(
            "Pack capacity is below 10 Ah — double-check the Excel sheet values."
        )

    return ElecCheckResult(
        ok=ok,
        fuse_blown=fuse_blown,
        cell_overcurrent=cell_overcurrent,
        energy_empty=energy_empty,
        peak_current_a=peak_I,
        peak_current_speed_ms=peak_speed,
        avg_current_a=avg_I,
        energy_used_kwh=total_energy_kwh,
        usable_energy_kwh=params.usable_energy_kwh,
        max_safe_speed_ms=max_safe_speed_ms,
        max_safe_power_kw=max_safe_P_kw,
        fuse_max_a=fuse_limit,
        peak_cell_current_a=peak_I_cell,
        cell_current_limit_a=cell_limit,
        current_profile_a=I_pack,
        power_profile_kw=P_elec_kw,
        speed_profile_ms=v,
        fuse_message=fuse_msg,
        cell_message=cell_msg,
        energy_message=energy_msg,
        summary=summary,
        warnings=warnings_list,
    )


def check_lap_from_speed_csv(
    speed_mph: Sequence[float],
    time_s: Sequence[float],
    params: ElecParams,
    *,
    drivetrain_eff: float = 0.90,
    vehicle_mass_kg: float = 230.0,
    drag_cda: float = 1.10,
    crr: float = 0.018,
) -> ElecCheckResult:
    """
    Run the electrical check from the SpeedVsTime CSV data in the Excel workbook
    (columns: time_s, speed_mph).

    Converts speed to m/s, integrates to get distance, then delegates to
    `check_lap_electrical`.
    """
    t = _finite("speed trace time (s)", time_s)
    v_mph = _finite("speed trace speed (mph)", speed_mph)
    if t.size != v_mph.size:
        raise ValueError(f"the speed trace needs one speed per time, got {v_mph.size} and {t.size}")
    if np.any(np.diff(t) <= 0):
        raise ValueError("speed trace time must rise from one sample to the next")
    vehicle_mass_kg = _finite_scalar("vehicle mass (kg)", vehicle_mass_kg, positive=True)
    v_ms = v_mph * 0.44704                          # mph → m/s

    # Integrate speed over time to get distance
    s = np.zeros(len(t))
    for i in range(1, len(t)):  # pragma: no mutate (a constant offset in s changes nothing downstream)
        dt = t[i] - t[i - 1]
        s[i] = s[i - 1] + 0.5 * (v_ms[i - 1] + v_ms[i]) * dt

    if np.max(np.abs(v_mph)) > 1000.0:
        raise ValueError("a speed above 1000 mph in the trace: check the column and its unit")
    for label, val in (("drivetrain efficiency", drivetrain_eff), ("drag CdA (m²)", drag_cda), ("rolling resistance", crr)):
        _finite_scalar(label, val)
    res = check_lap_electrical(
        v_ms, s, params,
        drivetrain_eff=drivetrain_eff,
        vehicle_mass_kg=vehicle_mass_kg,
        drag_cda=drag_cda,
        crr=crr,
    )
    if not _all_finite(res):
        raise ValueError("the electrical check gave a result that is not a finite number: one of the pack, motor "
                         "or vehicle parameters is nan, infinite or far out of range")
    return res


def load_speed_vs_time_from_bytes(file_bytes: bytes) -> tuple[np.ndarray, np.ndarray]:
    """
    Load the SpeedVsTime sheet from in-memory Excel bytes.

    Returns (time_s, speed_mph) arrays.

    Raises
    ------
    ValueError / ImportError  propagated from load_speed_vs_time on bad data.
    """
    import tempfile, os
    tmp = tempfile.NamedTemporaryFile(suffix=".xlsx", delete=False)  # pragma: no mutate
    tmp.write(file_bytes)
    tmp.close()
    try:
        result = load_speed_vs_time(tmp.name)
    finally:
        os.unlink(tmp.name)
    return result


def load_speed_vs_time(path: str) -> tuple[np.ndarray, np.ndarray]:
    """
    Load the SpeedVsTime sheet from the electrics lead's Excel workbook.

    Returns
    -------
    (time_s, speed_mph)  — both as float arrays, header row stripped.

    Raises
    ------
    ValueError  if the sheet is missing or contains fewer than 2 numeric rows.
    """
    try:
        import openpyxl
    except ImportError:
        raise ImportError("openpyxl is required to read Excel files. Install it with: pip install openpyxl")
    try:
        wb = openpyxl.load_workbook(path, read_only=True, data_only=True)  # pragma: no mutate
    except Exception as exc:
        raise ValueError(f"Could not open Excel workbook: {exc}") from exc

    # By alias and then by content, as the pack and propulsion sheets are: an
    # exact-name lookup is what took the Lap time tab down when a sheet was
    # renamed (see ev_excel_roundtrip.resolve_sheet).
    from .ev_excel_roundtrip import resolve_svt_sheet
    ws = resolve_svt_sheet(wb)
    if ws is None:
        available = ", ".join(wb.sheetnames) or "(none)"  # pragma: no mutate
        raise ValueError(
            f"No speed-vs-time sheet found in the workbook. "
            f"Available sheets: {available}. "
            f"Please add a sheet named 'SpeedVsTime' with columns: time_s, speed_mph."
        )

    times, speeds = [], []
    for row in ws.iter_rows(values_only=True):
        if len(row) < 2:
            continue  # pragma: no mutate (read-only rows are all padded to one width)
        t, v = row[0], row[1]
        if isinstance(t, (int, float)) and isinstance(v, (int, float)):
            times.append(float(t))
            speeds.append(float(v))

    if len(times) < 2:
        raise ValueError(
            f"SpeedVsTime sheet has only {len(times)} numeric data row(s) — need at least 2. "
            f"Make sure column A = time (s) and column B = speed (mph), with a header row on row 1."
        )

    return np.array(times), np.array(speeds)
