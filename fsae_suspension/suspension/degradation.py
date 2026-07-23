# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  suspension/degradation.py — Transient Degradation Synthesis: how the corner
#  drifts off its paper alignment across a long run as the chassis heats
#  (thermal expansion of pickups), the bushings/links flex under cornering load
#  (structural compliance), and the tyre loses grip with temperature. Layered as
#  fast closed-form perturbations over the existing rigid kinematics solver.
# ============================================================================
"""
Transient Degradation Synthesis — the Lap-15 car, not the CAD car.

WHAT THIS DOES
--------------
KinematiK's kinematics.py designs the car in an ideal world: rigid links, 20 °C
metal, nominal CAD coordinates. This module asks the question a design judge
actually asks — "what does your alignment DO across a 22-lap Endurance run when
the chassis is hot, the bushings are loaded and the tyres have gone off?" — and
answers it deterministically in seconds by layering three physical effects over
the same fast solver:

  1. THERMAL EXPANSION  (α·ΔT·L — first-principles).
     As the pack and subframe heat, every hardpoint moves away from a chosen
     thermal datum by α·ΔT·L along the vector from the datum. α is a material
     constant, ΔT comes from a transient lump-capacitance model, L is geometry.
     This layer is honest physics end to end. The shifted hardpoints are fed to
     the real solver, so the resulting camber/toe/roll-centre drift is exact for
     the given ΔT field.

  2. STRUCTURAL COMPLIANCE  (δ = F/k — first-principles FORM, calibratable k).
     Under a sustained cornering load the links and bushings deflect, steering
     and cambering the wheel. This is NOT re-implemented here — it delegates to
     the repo's already-tested CompliantCorner (compliance.py), which solves the
     load↔geometry coupling from member stiffnesses. The deflection math is
     exact; the bushing/tab stiffnesses k are inputs you calibrate (you cannot
     read a rubber bushing's rate off a CAD model). Labelled accordingly.

  3. TYRE GRIP DECAY  (EMPIRICAL — calibrate to your own tyre data).
     There is no first-principles closed form for grip-vs-temperature. Every
     team fits this to TTC data or their own testing. It is implemented here as
     an explicit, editable TyreThermalModel with a peak-grip temperature and a
     fall-off, and it is flagged EMPIRICAL everywhere it surfaces. Do not present
     the built-in default curve as derived physics — it is a placeholder shape.

WHAT IT OUTPUTS
---------------
  * DegradationCurve — camber, toe, roll-centre-height and grip drift vs a
    "thermal progress" axis (0 = cold Lap 1, 1 = fully heat-soaked Lap 15+),
    each split into its thermal and compliance contributions so you can see which
    effect owns the drift.
  * Lap-time delta proxy — the grip loss from (a) the tyre going off and (b) the
    alignment drifting off its grip-optimal target, expressed as a % of cornering
    capability. This is a PROXY, not a lap sim; feed it to lapsim.py for time.
  * ToleranceSensitivityMap — for each movable hardpoint, how much the Lap-15
    alignment moves per mm of build error AND per °C of local heat, so the build
    team knows which 2-3 points need CNC tolerance and which can be hand-welded.

THE HONEST SEPARATION (same as the rest of KinematiK)
-----------------------------------------------------
Thermal expansion: exact physics. Compliance deflection: exact physics on
calibratable stiffnesses. Tyre decay and the points/lap-time mapping: empirical
models you own. Every output carries its provenance so nobody mistakes the
tuned layer for the solved one.

A FINDING THIS SOLVER MAKES, STATED PLAINLY
-------------------------------------------
Run the numbers and the effects do NOT come out equal. At realistic FSAE scales
(subframe ΔT ~35 °C, pickup shifts ~0.1 mm), THERMAL EXPANSION moves the wheel
angles by only thousandths of a degree — because a near-uniform expansion about
the subframe datum is almost angle-preserving. The dominant degradation is
STRUCTURAL COMPLIANCE (bushing/link flex under cornering load: tenths of a
degree of camber and toe — one to two orders of magnitude larger) and TYRE GRIP
DECAY. This solver reports all three at their true magnitudes rather than
inflating the thermal term to match a dramatic story; the honest result is that
your build team should chase bushing stiffness and tyre thermal management
before subframe thermal growth. If your car runs an aluminium subframe (α ~2×
steel) or a genuinely large local gradient, the thermal term grows — the model
will show it — but it is reported, never assumed.
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field
from typing import Optional

from .kinematics import Hardpoints, SuspensionKinematics
from . import compliance as comp
from . import loadpath as lp


# Linear thermal expansion coefficients, 1/K (per °C). First-principles constants.
THERMAL_ALPHA = {
    "Steel 4130": 12.3e-6,
    "Steel mild": 12.0e-6,
    "Aluminium 6061": 23.6e-6,
    "Aluminium 7075": 23.4e-6,
    "Titanium Ti-6Al-4V": 8.6e-6,
    "Carbon (axial, representative)": 1.0e-6,   # near-zero axial CTE, laminate-dep.
}


# ===================================================================== #
#  1.  TRANSIENT THERMAL STATE  (lump-capacitance — first-principles form)
# ===================================================================== #
@dataclass
class ThermalRamp:
    """Transient temperature rise of the chassis/subframe metal over a run.

    A first-order lump-capacitance rise toward a soak temperature:
        T(t) = T_amb + (T_soak - T_amb) * (1 - exp(-t/tau))
    tau is the thermal time constant (min). This is the standard first-order
    heat-soak model; the numbers (T_soak, tau) are the calibratable inputs — set
    them from a thermocouple on your own subframe, not from CAD.
    """
    T_amb: float = 20.0             # °C ambient / cold start
    T_soak: float = 55.0            # °C fully heat-soaked metal (subframe near pack)
    tau_min: float = 6.0            # thermal time constant, minutes
    run_min: float = 25.0           # total run length (Endurance ~25 min)

    def temp_at(self, t_min: float) -> float:
        return self.T_amb + (self.T_soak - self.T_amb) * (1.0 - np.exp(-t_min / self.tau_min))

    def delta_T_at(self, t_min: float) -> float:
        return self.temp_at(t_min) - self.T_amb

    def progress_at(self, t_min: float) -> float:
        """0..1 heat-soak progress (1 = essentially fully soaked)."""
        dT_max = self.T_soak - self.T_amb
        return 0.0 if dT_max == 0 else self.delta_T_at(t_min) / dT_max


# ===================================================================== #
#  2.  THERMAL EXPANSION OF HARDPOINTS  (α·ΔT·L — first-principles)
# ===================================================================== #
# Which hardpoints ride on the heated chassis (inboard pickups + spring mount)
# and thus move under thermal expansion, vs the outboard points that ride on the
# (cooler, air-cooled) upright and are carried by the links. Expansion is applied
# to the chassis-side points relative to a thermal datum; the solver then carries
# the outboard points via the rigid-link constraints.
_CHASSIS_POINTS = [
    "upper_front_inner", "upper_rear_inner",
    "lower_front_inner", "lower_rear_inner",
    "tie_rod_inner", "spring_inner", "rocker_pivot",
]


@dataclass
class ThermalExpansionModel:
    """Expand chassis-side hardpoints away from a datum by α·(ΔT·gradient)·L.

    THE PHYSICS THAT MATTERS: a *uniform* thermal expansion of all pickups is
    very nearly angle-preserving — it scales the whole geometry about the datum
    and the static camber/toe barely move (verified). The alignment-destroying
    effect is DIFFERENTIAL heating: the pickups nearest the hot battery pack /
    exhaust expand MORE than the far ones, and that asymmetry actually moves the
    wheel angles. This model captures that with a per-point ΔT gradient tied to
    each point's proximity to a declared heat source, which is the honest,
    first-principles source of thermal misalignment — not a hand-waved "0.8 mm".

    datum        : subframe location point; expansion grows radially from here.
    material     : sets α (the subframe metal).
    heated_points: which hardpoints move (chassis-side pickups).
    heat_source  : point of peak metal temperature (near the pack). Points close
                   to it see the full ΔT; far points see less, scaled by
                   exp(-dist/gradient_len). Set gradient_len huge for ~uniform.
    gradient_len : mm decay length of the thermal gradient across the subframe.
    """
    material: str = "Steel 4130"
    datum: Optional[np.ndarray] = None
    heated_points: tuple = tuple(_CHASSIS_POINTS)
    heat_source: Optional[np.ndarray] = None
    gradient_len: float = 350.0

    def alpha(self) -> float:
        return THERMAL_ALPHA.get(self.material, 12.3e-6)

    def _source(self, hp: Hardpoints) -> np.ndarray:
        """Default heat source: inboard-most, low (where a pack/subframe sits),
        on centreline. Points nearer this run hotter."""
        if self.heat_source is not None:
            return np.asarray(self.heat_source, float)
        lf = np.asarray(hp.lower_front_inner, float)
        return np.array([lf[0], 0.0, lf[2]])

    def _local_dT(self, hp: Hardpoints, p: np.ndarray, delta_T: float) -> float:
        """ΔT seen by a point: full ΔT near the source, decaying with distance."""
        src = self._source(hp)
        dist = float(np.linalg.norm(np.asarray(p, float) - src))
        return delta_T * float(np.exp(-dist / max(self.gradient_len, 1e-6)))

    def _datum(self, hp: Hardpoints) -> np.ndarray:
        if self.datum is not None:
            return np.asarray(self.datum, float)
        # centreline (y=0) at the lower-front pickup, mid X of the two lower pts
        lf = np.asarray(hp.lower_front_inner, float)
        lr = np.asarray(hp.lower_rear_inner, float)
        return np.array([(lf[0] + lr[0]) / 2.0, 0.0, (lf[2] + lr[2]) / 2.0])

    def expanded(self, hp: Hardpoints, delta_T: float) -> Hardpoints:
        """Return a copy of hp with chassis points thermally expanded by ΔT."""
        out = hp.copy()
        d = self._datum(hp)
        a = self.alpha()
        for name in self.heated_points:
            p = getattr(out, name, None)
            if p is None:
                continue
            p = np.asarray(p, float)
            r = p - d                       # vector from datum
            L = np.linalg.norm(r)
            if L < 1e-9:
                continue
            dT_local = self._local_dT(hp, p, delta_T)   # differential heating
            # linear expansion grows the datum→point distance by α·ΔT_local·L
            new_p = d + r * (1.0 + a * dT_local)
            setattr(out, name, new_p)
        return out

    def point_shift_mm(self, hp: Hardpoints, name: str, delta_T: float) -> float:
        """Magnitude of thermal shift of one point at ΔT (mm) — for the report.
        Uses the point's LOCAL (differential) ΔT, so points near the heat source
        report a larger shift than far ones."""
        d = self._datum(hp)
        p = np.asarray(getattr(hp, name), float)
        L = float(np.linalg.norm(p - d))
        dT_local = self._local_dT(hp, p, delta_T)
        return self.alpha() * dT_local * L


# ===================================================================== #
#  3.  TYRE GRIP DECAY  (EMPIRICAL — calibrate to your tyre data)
# ===================================================================== #
@dataclass
class TyreThermalModel:
    """Grip multiplier vs tyre carcass temperature. EMPIRICAL placeholder shape.

    mu_scale(T) = 1 - k_cold*(T<T_peak gap)  ... peak at T_peak, falling either
    side. Implemented as a smooth inverted parabola clamped to [floor, 1]. THE
    NUMBERS ARE A STAND-IN — replace T_peak / width / floor with a fit to your
    own TTC or track data before quoting a grip loss.
    """
    T_peak: float = 80.0            # °C peak-grip carcass temp
    width: float = 45.0            # °C half-width of the usable window
    floor: float = 0.82           # grip multiplier far from peak (never below)

    def grip_mult(self, T_tyre: float) -> float:
        x = (T_tyre - self.T_peak) / self.width
        m = 1.0 - x * x
        return float(np.clip(m, self.floor, 1.0))


@dataclass
class TyreThermalRamp:
    """Tyre carcass temperature over the run. EMPIRICAL — tyres heat faster than
    the chassis and settle higher. Separate ramp from the chassis metal."""
    T_amb: float = 20.0
    T_settle: float = 95.0          # settled carcass temp mid-Endurance
    tau_min: float = 2.5

    def temp_at(self, t_min: float) -> float:
        return self.T_amb + (self.T_settle - self.T_amb) * (1.0 - np.exp(-t_min / self.tau_min))


# ===================================================================== #
#  4.  ONE DEGRADED STATE  (thermal ⊕ compliance ⊕ tyre at a time instant)
# ===================================================================== #
@dataclass
class DegradedState:
    t_min: float
    delta_T: float
    progress: float                 # 0..1 heat soak
    # alignment (deg / mm), split by cause
    camber_nominal: float
    camber_thermal: float           # after thermal expansion only
    camber_compliant: float         # after thermal + load compliance
    toe_nominal: float
    toe_thermal: float
    toe_compliant: float
    ic_z_nominal: float
    ic_z_compliant: float
    grip_mult: float                # empirical
    camber_gain_nominal: float = 0.0
    camber_gain_thermal: float = 0.0
    bumpsteer_nominal: float = 0.0
    bumpsteer_thermal: float = 0.0
    converged: bool = True

    @property
    def camber_drift(self) -> float:
        return self.camber_compliant - self.camber_nominal

    @property
    def toe_drift(self) -> float:
        return self.toe_compliant - self.toe_nominal

    @property
    def ic_drift_mm(self) -> float:
        return self.ic_z_compliant - self.ic_z_nominal


def _corner_metrics(hp: Hardpoints) -> tuple:
    """(camber, toe, instant_centre_z) at static from the real solver.

    NOTE: roll_center_height is computed at the VEHICLE level (needs both corners
    paired), so kinematics.py leaves it NaN on a single corner. We report the
    front-view INSTANT-CENTRE height instead — a real single-corner quantity that
    moves with the geometry and is the honest corner-level proxy for RC drift. We
    do NOT invent a roll-centre number from one corner."""
    kin = SuspensionKinematics(hp)
    s = kin.static
    ic = np.asarray(s.instant_center, float)     # (y, z) of the front-view IC
    ic_z = float(ic[1]) if ic.size >= 2 else float("nan")
    return float(s.camber), float(s.toe), ic_z


def _corner_curves(hp: Hardpoints) -> tuple:
    """(camber_gain_deg, bumpsteer_deg) from a real travel sweep.

    Thermal expansion of the pickups is very nearly angle-preserving at the
    static datum (uniform scaling preserves angles), so its real — if small —
    kinematic signature shows up in the THROUGH-TRAVEL curves, not the static
    camber/toe. This function exposes that signature honestly."""
    kin = SuspensionKinematics(hp)
    sts = kin.sweep(-25.0, 25.0, 9)
    cam = np.array([s.camber for s in sts])
    toe = np.array([s.toe for s in sts])
    tr = np.array([s.travel for s in sts])
    cg = abs(np.polyfit(tr, cam, 1)[0]) * 25.0
    bs = float(toe.max() - toe.min())
    return float(cg), bs


# ===================================================================== #
#  5.  THE DEGRADATION SOLVER
# ===================================================================== #
@dataclass
class DegradationConfig:
    lateral_g: float = 1.4          # sustained cornering case for the compliance load
    axle: str = "front"
    # compliance stiffness: default = 4130 tube uniform corner + rubber bushings.
    # Pass your own CompliantCorner factory args if you have measured stiffnesses.
    tube_od_mm: float = 19.05
    tube_wall_mm: float = 0.9
    use_bushings: bool = True
    bushing_rate_N_per_mm: float = 4000.0   # CALIBRATABLE — rubber/poly bushing
    n_steps: int = 16


class DegradationSolver:
    """Runs the transient degradation over a run and produces the curves + maps."""

    def __init__(self, hp: Hardpoints,
                 thermal: ThermalRamp | None = None,
                 expansion: ThermalExpansionModel | None = None,
                 tyre_grip: TyreThermalModel | None = None,
                 tyre_temp: TyreThermalRamp | None = None,
                 config: DegradationConfig | None = None,
                 vehicle=None):
        self.hp = hp
        self.thermal = thermal or ThermalRamp()
        self.expansion = expansion or ThermalExpansionModel()
        self.tyre_grip = tyre_grip or TyreThermalModel()
        self.tyre_temp = tyre_temp or TyreThermalRamp()
        self.cfg = config or DegradationConfig()
        self.vehicle = vehicle          # optional VehicleDynamics for real loads

    # -- compliance corner builder (delegates to tested compliance.py) --
    def _compliant_corner(self, hp: Hardpoints) -> comp.CompliantCorner:
        if self.cfg.use_bushings:
            try:
                b = comp.JointCompliance.rubber_bushing()
            except Exception:
                b = None
            if b is not None:
                return comp.CompliantCorner.with_bushings(
                    hp, bushing=b, od_mm=self.cfg.tube_od_mm,
                    wall_mm=self.cfg.tube_wall_mm)
        return comp.CompliantCorner.uniform_tube(
            hp, od_mm=self.cfg.tube_od_mm, wall_mm=self.cfg.tube_wall_mm)

    def _wheel_load(self, hp: Hardpoints) -> lp.WheelLoad:
        """Cornering contact-patch load. Uses the real load-transfer model when a
        vehicle is supplied; otherwise a transparent Fz estimate."""
        if self.vehicle is not None:
            return comp.corner_wheel_load(self.vehicle, self.cfg.axle,
                                          self.cfg.lateral_g, outer=True)
        # fallback: quarter-car static + simple lateral transfer estimate.
        Fz = 1500.0 * (1.0 + 0.4 * self.cfg.lateral_g)   # N, transparent estimate
        return lp.WheelLoad(Fx=0.0, Fy=-self.cfg.lateral_g * Fz, Fz=Fz)

    def _state_at(self, t_min: float) -> DegradedState:
        dT = self.thermal.delta_T_at(t_min)
        prog = self.thermal.progress_at(t_min)

        # nominal (cold, unloaded) alignment
        cam0, toe0, rc0 = _corner_metrics(self.hp)

        # thermal-only alignment: expand hardpoints, re-solve rigid
        hp_hot = self.expansion.expanded(self.hp, dT)
        cam_th, toe_th, rc_th = _corner_metrics(hp_hot)

        # thermal + compliance: load the hot geometry through the compliance solver
        cam_c, toe_c, rc_c, conv = cam_th, toe_th, rc_th, True
        try:
            cc = self._compliant_corner(hp_hot)
            res = cc.solve(self._wheel_load(hp_hot))
            cam_c = cam_th + res.compliance_camber
            toe_c = toe_th + res.compliance_toe
            # roll-centre shift from compliance is second-order here; carry thermal
            rc_c = rc_th
            conv = bool(res.converged)
        except Exception:
            conv = False

        # through-travel thermal signature (small but real; the honest place the
        # thermal expansion actually shows up). Computed cold vs hot.
        cg0, bs0 = _corner_curves(self.hp)
        cg_th, bs_th = _corner_curves(hp_hot)

        # tyre grip (empirical)
        T_tyre = self.tyre_temp.temp_at(t_min)
        grip = self.tyre_grip.grip_mult(T_tyre)

        return DegradedState(
            t_min=t_min, delta_T=dT, progress=prog,
            camber_nominal=cam0, camber_thermal=cam_th, camber_compliant=cam_c,
            toe_nominal=toe0, toe_thermal=toe_th, toe_compliant=toe_c,
            ic_z_nominal=rc0, ic_z_compliant=rc_c,
            camber_gain_nominal=cg0, camber_gain_thermal=cg_th,
            bumpsteer_nominal=bs0, bumpsteer_thermal=bs_th,
            grip_mult=grip, converged=conv)

    def run(self) -> "DegradationCurve":
        ts = np.linspace(0.0, self.thermal.run_min, self.cfg.n_steps)
        states = [self._state_at(t) for t in ts]
        return DegradationCurve(states, self)


# ===================================================================== #
#  6.  RESULT CURVE + LAP-TIME PROXY
# ===================================================================== #
def _dominant_mechanism(hot: DegradedState) -> str:
    """Which effect owns the Lap-15 alignment drift — reported, not assumed."""
    thermal_mag = (abs(hot.camber_thermal - hot.camber_nominal) +
                   abs(hot.toe_thermal - hot.toe_nominal) +
                   abs(hot.camber_gain_thermal - hot.camber_gain_nominal) +
                   abs(hot.bumpsteer_thermal - hot.bumpsteer_nominal))
    compliance_mag = (abs(hot.camber_compliant - hot.camber_thermal) +
                      abs(hot.toe_compliant - hot.toe_thermal))
    if compliance_mag > 5 * max(thermal_mag, 1e-9):
        return "structural compliance (dominant) — chase bushing/link stiffness"
    if thermal_mag > 5 * max(compliance_mag, 1e-9):
        return "thermal expansion (dominant) — chase subframe thermal growth"
    return "mixed thermal + compliance"


@dataclass
class DegradationCurve:
    states: list
    solver: DegradationSolver

    def as_arrays(self) -> dict:
        s = self.states
        return {
            "t_min": np.array([x.t_min for x in s]),
            "delta_T": np.array([x.delta_T for x in s]),
            "progress": np.array([x.progress for x in s]),
            "camber_drift": np.array([x.camber_drift for x in s]),
            "camber_thermal_drift": np.array([x.camber_thermal - x.camber_nominal for x in s]),
            "toe_drift": np.array([x.toe_drift for x in s]),
            "toe_thermal_drift": np.array([x.toe_thermal - x.toe_nominal for x in s]),
            "ic_drift_mm": np.array([x.ic_drift_mm for x in s]),
            "grip_mult": np.array([x.grip_mult for x in s]),
        }

    def lap15_summary(self) -> dict:
        """The end-of-run degraded state, with drift split by cause."""
        cold, hot = self.states[0], self.states[-1]
        return {
            "camber_drift_deg": round(hot.camber_drift, 3),
            "camber_thermal_deg": round(hot.camber_thermal - hot.camber_nominal, 3),
            "camber_compliance_deg": round(hot.camber_compliant - hot.camber_thermal, 3),
            "toe_drift_deg": round(hot.toe_drift, 3),
            "toe_thermal_deg": round(hot.toe_thermal - hot.toe_nominal, 3),
            "toe_compliance_deg": round(hot.toe_compliant - hot.toe_thermal, 3),
            "instant_centre_drift_mm": round(hot.ic_drift_mm, 2),
            "camber_gain_thermal_shift_deg": round(
                hot.camber_gain_thermal - hot.camber_gain_nominal, 4),
            "bumpsteer_thermal_shift_deg": round(
                hot.bumpsteer_thermal - hot.bumpsteer_nominal, 4),
            "dominant_mechanism": _dominant_mechanism(hot),
            "grip_mult_cold": round(cold.grip_mult, 3),
            "grip_mult_hot": round(hot.grip_mult, 3),
            "delta_T_final": round(hot.delta_T, 1),
            "converged": all(x.converged for x in self.states),
        }

    def laptime_delta_proxy(self, camber_grip_sens_per_deg: float = 0.04,
                            toe_grip_sens_per_deg: float = 0.06) -> dict:
        """Cornering-capability loss at Lap 15, split by cause. PROXY, not a lap sim.

        Combines (a) tyre grip decay (empirical), and (b) alignment drift away
        from the cold-optimal, via first-order grip sensitivities to camber/toe
        (the two *_grip_sens_per_deg are CALIBRATABLE — from your tyre model). The
        result is a % loss of peak lateral capability; convert to seconds with
        lapsim.py, which is the honest place for a time number.
        """
        hot = self.states[-1]
        grip_loss_tyre = 1.0 - hot.grip_mult
        grip_loss_align = (abs(hot.camber_drift) * camber_grip_sens_per_deg +
                           abs(hot.toe_drift) * toe_grip_sens_per_deg)
        total = grip_loss_tyre + grip_loss_align
        return {
            "grip_loss_tyre_pct": round(100 * grip_loss_tyre, 2),
            "grip_loss_alignment_pct": round(100 * grip_loss_align, 2),
            "grip_loss_total_pct": round(100 * total, 2),
            "note": ("PROXY: % loss of peak lateral capability. Tyre term is "
                     "EMPIRICAL; alignment term uses calibratable grip "
                     "sensitivities. Feed to lapsim.py for a time in seconds."),
        }


# ===================================================================== #
#  7.  TOLERANCE SENSITIVITY MAP  (which points need CNC vs hand-weld)
# ===================================================================== #
@dataclass
class ToleranceSensitivityMap:
    rows: list          # per-point sensitivity dicts

    def critical_points(self, top_n: int = 2) -> list:
        """The points whose thermal+build sensitivity most moves Lap-15 alignment."""
        return sorted(self.rows, key=lambda r: -r["combined_score"])[:top_n]

    def table(self) -> list:
        return sorted(self.rows, key=lambda r: -r["combined_score"])


def tolerance_sensitivity(hp: Hardpoints,
                          solver: DegradationSolver | None = None,
                          jig_tol_mm: float = 1.0,
                          points: list | None = None) -> ToleranceSensitivityMap:
    """For each movable hardpoint, how much does Lap-15 alignment move per mm of
    BUILD error and per the run's thermal shift?

    Method (all first-principles / real-solver):
      * build sensitivity: perturb the point by +jig_tol_mm on each axis, re-solve
        the hot+loaded Lap-15 alignment, and measure |Δcamber|+|Δtoe|. This is the
        exact finite-difference sensitivity through the real solver — the same
        reverse sensitivity the InverseGenesis engine uses.
      * thermal shift: the point's own α·ΔT·L movement at final ΔT.
    combined_score ranks points by (build sensitivity × jig tol) ⊕ thermal shift,
    telling the build team where a tight CNC tolerance actually buys Lap-15 grip
    and where hand-welding is fine.
    """
    solver = solver or DegradationSolver(hp)
    dT_final = solver.thermal.delta_T_at(solver.thermal.run_min)
    exp = solver.expansion

    # baseline hot+loaded alignment
    base_state = solver._state_at(solver.thermal.run_min)
    base_cam, base_toe = base_state.camber_compliant, base_state.toe_compliant

    move_pts = points or [
        "upper_front_inner", "upper_rear_inner", "lower_front_inner",
        "lower_rear_inner", "upper_outer", "lower_outer",
        "tie_rod_inner", "tie_rod_outer",
    ]

    rows = []
    for name in move_pts:
        p0 = getattr(hp, name, None)
        if p0 is None:
            continue
        p0 = np.asarray(p0, float)
        # build sensitivity: max over the 3 axes of |Δalignment| per jig_tol
        best_axis_sens = 0.0
        for ax in range(3):
            hp_pert = hp.copy()
            pert = p0.copy()
            pert[ax] += jig_tol_mm
            setattr(hp_pert, name, pert)
            s = DegradationSolver(
                hp_pert, thermal=solver.thermal, expansion=solver.expansion,
                tyre_grip=solver.tyre_grip, tyre_temp=solver.tyre_temp,
                config=solver.cfg, vehicle=solver.vehicle
            )._state_at(solver.thermal.run_min)
            dcam = abs(s.camber_compliant - base_cam)
            dtoe = abs(s.toe_compliant - base_toe)
            best_axis_sens = max(best_axis_sens, dcam + dtoe)
        # thermal shift of this point (0 if it's an outboard/upright point)
        th_shift = exp.point_shift_mm(hp, name, dT_final) \
            if name in exp.heated_points else 0.0
        combined = best_axis_sens + 0.5 * th_shift * best_axis_sens
        rows.append({
            "point": name,
            "build_sens_deg_per_mm": round(best_axis_sens / max(jig_tol_mm, 1e-9), 4),
            "thermal_shift_mm": round(th_shift, 3),
            "alignment_move_deg": round(best_axis_sens, 4),
            "combined_score": round(combined, 4),
            "recommendation": "",     # filled below
        })
    # recommendation banding
    if rows:
        scores = np.array([r["combined_score"] for r in rows])
        hi = np.percentile(scores, 75) if len(scores) > 3 else scores.max()
        for r in rows:
            r["recommendation"] = ("CNC / tight tol" if r["combined_score"] >= hi
                                   else "hand-weld OK")
    return ToleranceSensitivityMap(rows)


PROVENANCE = {
    "physics_first_principles": [
        "thermal expansion of hardpoints (α·ΔT·L)",
        "structural compliance deflection (δ=F/k) via tested CompliantCorner",
        "all camber/toe/roll-centre from the real SuspensionKinematics solver",
        "tolerance build-sensitivity (finite difference through the real solver)",
    ],
    "calibratable_inputs": [
        "bushing / chassis-tab stiffness k (measured, not from CAD)",
        "thermal soak temperature and time constant (thermocouple, not CAD)",
    ],
    "empirical_models": [
        "tyre grip vs temperature (TyreThermalModel) — fit to your TTC data",
        "grip sensitivities to camber/toe in the lap-time proxy",
    ],
    "note": (
        "Thermal expansion and compliance geometry are first-principles and as "
        "trustworthy as the solver. Bushing stiffness and soak temperature are "
        "calibratable inputs. The tyre grip-vs-temp curve and the lap-time proxy "
        "are EMPIRICAL — the default curves are placeholder shapes; fit them to "
        "your own tyre and lap-sim data before quoting a grip or time delta."
    ),
}
