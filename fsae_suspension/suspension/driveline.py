# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  suspension/driveline.py — the differential the lap sim never had.
#
#  Written to close a specific hole: LapSimParams.drive_grip_frac is a single
#  hand-typed scalar standing in for the entire rear-axle traction limit. That
#  makes every differential in the world identical to the sim. This module
#  DERIVES that scalar from the actual corner-exit load case and the actual
#  torque-biasing behaviour of a named diff, so an open diff, a Torsen/helical
#  ATB and a spool stop scoring the same lap time.
# ============================================================================
"""
Differential & rear-axle traction model.

WHY THIS EXISTS
---------------
KinematiK can already reconcile a subsystem's mass and CG into the vehicle model
(``worthwhile.vehicle_from_ledger``) and turn a lap time into points. What it
could NOT do is say anything about a part whose value is *not* mass — a
differential is bought for how it splits torque between two rear wheels that are
carrying very different vertical loads at corner exit. The existing sim reduces
that whole question to ``drive_grip_frac``, a constant the user types in. Two
diffs with the same mass therefore produce byte-identical lap times, and any
"is this worth $2,600" question is answered entirely by the number the user
guessed. That is not an evaluation, it is an echo.

THE MODEL
---------
Corner exit, steady lateral acceleration, RWD. Per rear wheel:

    Fz_i            vertical load from the real lateral-load-transfer solver in
                    dynamics.py (roll stiffness split + geometric/roll-centre
                    component), plus the rear share of aero downforce.
    F_cap_i         the tyre's peak force at that load — taken from whichever
                    grip model is attached (Pacejka MF5.2 when a tyre is loaded,
                    the linear placeholder otherwise). Isotropic friction is
                    ASSUMED, i.e. the friction ellipse is treated as a circle.
    lambda          lateral utilisation = a_y / a_y_max at this condition.
    Fx_cap_i        = F_cap_i * sqrt(1 - lambda**2)      (friction circle)
    T_cap_i         = Fx_cap_i * r_wheel

The diff then decides how much of that the axle can actually use:

    open      T_total = 2 * T_cap_inside
    LSD       T_out   = min(T_cap_out, TBR * T_cap_in + preload)
              T_total = T_cap_in + T_out
    spool     T_total = T_cap_in + T_cap_out

The LSD form degenerates correctly at both ends: TBR = 1, preload = 0 gives the
open diff exactly; TBR -> infinity gives the spool exactly. Torque bias ratio is
the manufacturer's own published figure for the part.

MAPPING BACK INTO THE LAP SIM
-----------------------------
lapsim computes its traction limit as
``F_grip = drive_grip_frac * mu * (m*g + Fz_aero)``. So the physically correct
value of that knob is simply

    drive_grip_frac = F_x_available_at_rear_axle / (mu * (m*g + Fz_aero))

evaluated at the same condition, with the same mu the sim would use. That is
what ``drive_grip_frac_for()`` returns. No change to lapsim is required — the
knob stops being a guess and starts being a derived quantity.

WHAT IS PHYSICS AND WHAT IS NOT — READ THIS BEFORE QUOTING A NUMBER
-------------------------------------------------------------------
PHYSICS (as good as the rest of KinematiK):
  * lateral load transfer, per-wheel Fz            — dynamics.py, tested
  * load-sensitive tyre capacity at each Fz        — attached tyre model
  * the torque-split algebra above                 — exact given TBR/preload
  * mass and CG roll-up into the vehicle           — exact arithmetic

ESTIMATE / ASSUMPTION (flagged everywhere it is used):
  * friction CIRCLE, not a measured combined-slip ellipse
  * lateral force shared between the rear tyres in proportion to capacity
  * a single corner-exit condition compressed into one scalar for the sim
  * the LOCK PENALTY (below) — a locked axle resists yaw on power. This is real
    and it is the reason a spool is not simply the best diff, but the magnitude
    here is a coefficient, not a solved yaw balance. It is exposed as a swept
    parameter, never as a silent constant, and the trade module reports whether
    the conclusion survives the whole sweep.

A quoted TBR is itself a nominal figure: real bias varies with preload state,
oil, wear and axle torque. Treat single-point outputs accordingly — that is why
``trade.py`` refuses to hand back a point estimate.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import Optional

import numpy as np

from .dynamics import VehicleParams, VehicleDynamics
from .interfaces import SubsystemInterface


# --------------------------------------------------------------------------- #
#  1.  The part
# --------------------------------------------------------------------------- #
OPEN, HELICAL_ATB, PLATE_LSD, SPOOL = "open", "helical_atb", "plate_lsd", "spool"


@dataclass
class DifferentialSpec:
    """A differential as a purchasable, mountable, torque-biasing object.

    ``tbr`` is the manufacturer's torque bias ratio (T_high / T_low). 1.0 is an
    open diff. A spool is represented by ``kind=SPOOL``, not by a huge TBR, so
    the scrub/yaw penalty can be applied honestly.

    ``preload_nm`` is static breakaway torque at the diff — for the TRE units
    this is the thing the shims/springs and the Mk2.5 external adjuster change.

    Fabrication fields exist because a cheaper listing is often not a cheaper
    part: "no sprocket adapter" means somebody on the team designs, machines and
    validates a sprocket carrier, and that labour and risk belong in the trade.
    """
    name: str
    kind: str = HELICAL_ATB
    tbr: float = 1.0                        # torque bias ratio; 1.0 == open
    preload_nm: float = 0.0                 # static breakaway torque, N·m
    mass_kg: Optional[float] = None
    cost_usd: Optional[float] = None
    adjustable_preload: bool = False
    # packaging bounding box (mm) and CG in vehicle frame — feeds the ledger
    env_x_mm: Optional[float] = None
    env_y_mm: Optional[float] = None
    env_z_mm: Optional[float] = None
    cg_x_mm: Optional[float] = None
    cg_y_mm: Optional[float] = None
    cg_z_mm: Optional[float] = None
    mount_points: Optional[int] = None
    peak_torque_nm: Optional[float] = None  # rated input torque capacity
    # what the listing does NOT include
    sprocket_adapter_included: bool = True
    extra_fab_hours: float = 0.0            # team labour to make it usable
    extra_parts_usd: float = 0.0            # carriers, mounts, chain, sprockets
    lead_time_weeks: Optional[float] = None
    is_estimate: bool = True
    source: str = ""                        # where these numbers came from

    # ---- derived -------------------------------------------------------- #
    def effective_tbr(self) -> float:
        if self.kind == OPEN:
            return 1.0
        if self.kind == SPOOL:
            return float("inf")
        return max(1.0, float(self.tbr))

    def total_acquisition_usd(self) -> Optional[float]:
        """Purchase price plus the parts you must buy to actually run it.
        Labour is reported separately — see ``extra_fab_hours`` — because team
        hours and dollars are different budgets with different scarcity."""
        if self.cost_usd is None:
            return None
        return float(self.cost_usd) + float(self.extra_parts_usd)

    def to_interface(self, name: str = "powertrain-diff",
                     owner: str = "", rationale: str = "") -> SubsystemInterface:
        """Expose the part to the existing IntegrationLedger so the mass, CG,
        envelope and mount checks in interfaces.py run on it for free."""
        return SubsystemInterface(
            name=name, mass_kg=self.mass_kg,
            cg_x_mm=self.cg_x_mm, cg_y_mm=self.cg_y_mm, cg_z_mm=self.cg_z_mm,
            env_x_mm=self.env_x_mm, env_y_mm=self.env_y_mm, env_z_mm=self.env_z_mm,
            mount_points=self.mount_points, mounts_on="chassis",
            peak_torque_nm=self.peak_torque_nm,
            is_estimate=self.is_estimate,
            rationale=rationale or f"{self.kind} diff, TBR {self.tbr:g}, "
                                   f"preload {self.preload_nm:g} N·m. {self.source}",
            owner=owner)


# --------------------------------------------------------------------------- #
#  2.  Corner-exit traction with a differential in the loop
# --------------------------------------------------------------------------- #
@dataclass
class ExitCondition:
    """The load case the axle is evaluated at.

    The corner is specified as a FRACTION OF THE CAR'S OWN LATERAL LIMIT, not an
    absolute g. An absolute value silently means different things on a grippy car
    and a slippery one, and at high values it pins the friction circle at zero
    longitudinal capacity — at which point every differential scores identically
    by construction and the comparison is meaningless. Set ``lateral_g`` only if
    you specifically want a fixed absolute condition.
    """
    lateral_g_frac: Optional[float] = 0.70   # share of max_lateral_g being used
    lateral_g: Optional[float] = None        # absolute override, g
    speed_ms: float = 14.0          # m/s at that point
    r_wheel_m: float = 0.225        # loaded rolling radius (18" OD tyre)
    rear_aero_frac: float = 0.50    # share of total downforce on the rear axle
    rho: float = 1.225
    cl_a: float = 2.5               # must match LapSimParams.cl_a to stay honest


@dataclass
class TractionResult:
    fz_inside_n: float
    fz_outside_n: float
    t_cap_inside_nm: float
    t_cap_outside_nm: float
    t_inside_nm: float
    t_outside_nm: float
    t_total_nm: float
    fx_axle_n: float
    lock_ratio: float               # (T_out-T_in)/(T_out+T_in): 0 open, ->1 spool
    lateral_util: float             # lambda
    mu_car: float
    drive_grip_frac: float
    notes: list = field(default_factory=list)


def _mu_car(veh: VehicleDynamics, fz_total_n: float) -> float:
    """The same whole-car effective mu the lap sim uses, so the returned
    drive_grip_frac plugs into lapsim without a unit or definition mismatch."""
    p = veh.p
    try:
        loads, _ = veh.lateral_load_transfer(0.0)
        # scale the static corner loads up to the requested total (aero included)
        scale = fz_total_n / max(p.mass * p.g, 1.0)
        f, r = veh.axle_grip(type(loads)(loads.fl * scale, loads.fr * scale,
                                         loads.rl * scale, loads.rr * scale))
        mu = (f + r) / max(fz_total_n, 1.0)
        if not np.isfinite(mu) or mu <= 0:
            raise ValueError
        return float(min(mu, 3.0))
    except Exception:
        return float(p.mu_peak - p.tire_load_sens * (fz_total_n / 4.0))


def axle_traction(veh: VehicleDynamics, spec: DifferentialSpec,
                  cond: ExitCondition | None = None) -> TractionResult:
    """Solve the rear-axle tractive force this differential can actually deliver
    at the given corner-exit condition, and express it as the lap sim's
    ``drive_grip_frac``."""
    cond = cond or ExitCondition()
    p = veh.p
    notes: list[str] = []

    # --- resolve the corner: a fraction of THIS car's limit unless overridden
    try:
        ay_max = float(veh.max_lateral_g())
        if not np.isfinite(ay_max) or ay_max <= 0:
            raise ValueError
    except Exception:
        ay_max = 1.5
        notes.append("max_lateral_g() was unusable; assumed 1.5 g for the "
                     "friction-circle split.")
    if cond.lateral_g is not None:
        lateral_g = float(cond.lateral_g)
    else:
        lateral_g = float(cond.lateral_g_frac or 0.70) * ay_max

    # --- vertical loads: real lateral load transfer + rear share of downforce
    loads, _info = veh.lateral_load_transfer(lateral_g)
    fz_aero_total = 0.5 * cond.rho * cond.cl_a * cond.speed_ms ** 2
    fz_aero_rear_per_wheel = 0.5 * cond.rear_aero_frac * fz_aero_total
    fz_in = max(loads.rl + fz_aero_rear_per_wheel, 0.0)     # inside rear
    fz_out = max(loads.rr + fz_aero_rear_per_wheel, 0.0)    # outside rear
    if fz_in <= 1.0:
        notes.append("Inside rear wheel is at (or off) zero load — an open diff "
                     "delivers essentially no drive here. This is the condition "
                     "the LSD is bought for; it is also where the model is most "
                     "sensitive, so check the swept band, not the point value.")

    # --- per-wheel force capacity from the attached grip model
    f_cap_in = veh._corner_force(fz_in, "rear")
    f_cap_out = veh._corner_force(fz_out, "rear")

    # --- how much of the friction circle is already spent on cornering
    lam = min(max(lateral_g / ay_max, 0.0), 1.0)
    long_frac = math.sqrt(max(0.0, 1.0 - lam ** 2))
    if lam >= 0.97:
        notes.append("Lateral demand is essentially at the tyre limit — almost "
                     "no longitudinal capacity remains, so every differential "
                     "scores alike here by construction. Lower lateral_g_frac "
                     "for a comparison that means anything.")

    t_cap_in = f_cap_in * long_frac * cond.r_wheel_m
    t_cap_out = f_cap_out * long_frac * cond.r_wheel_m

    # --- the differential's decision
    if spec.kind == SPOOL:
        t_in, t_out = t_cap_in, t_cap_out
    elif spec.kind == OPEN or spec.effective_tbr() <= 1.0 + 1e-9:
        t_in = t_cap_in
        t_out = min(t_cap_out, t_cap_in + spec.preload_nm)
    else:
        t_in = t_cap_in
        t_out = min(t_cap_out, spec.effective_tbr() * t_cap_in + spec.preload_nm)
    t_total = t_in + t_out
    denom = (t_in + t_out)
    lock_ratio = 0.0 if denom <= 0 else (t_out - t_in) / denom

    fx_axle = t_total / max(cond.r_wheel_m, 1e-6)
    fz_total = p.mass * p.g + fz_aero_total
    mu = _mu_car(veh, fz_total)
    dgf = fx_axle / max(mu * fz_total, 1e-6)

    return TractionResult(
        fz_inside_n=fz_in, fz_outside_n=fz_out,
        t_cap_inside_nm=t_cap_in, t_cap_outside_nm=t_cap_out,
        t_inside_nm=t_in, t_outside_nm=t_out, t_total_nm=t_total,
        fx_axle_n=fx_axle, lock_ratio=lock_ratio, lateral_util=lam,
        mu_car=mu, drive_grip_frac=float(min(max(dgf, 0.0), 1.0)), notes=notes)


def drive_grip_frac_for(veh: VehicleDynamics, spec: DifferentialSpec,
                        cond: ExitCondition | None = None) -> float:
    """Just the scalar, for dropping straight into ``LapSimParams``."""
    return axle_traction(veh, spec, cond).drive_grip_frac


# --------------------------------------------------------------------------- #
#  3.  The other side of the ledger: locking costs you cornering
# --------------------------------------------------------------------------- #
#  A locked (or heavily biased) axle makes unequal tractive forces at the rear
#  contact patches. That difference is a yaw moment OPPOSING the turn, plus tyre
#  scrub in tight radii. It is why nobody wins autocross on a spool despite the
#  spool having the best traction number above.
#
#  READ THIS: k below is the LAP-AVERAGED fractional loss of usable lateral grip
#  at full lock. It is lap-averaged because the penalty only acts while power is
#  being applied mid-corner, which is a minority of any lap; the instantaneous
#  per-corner value is several times larger. There is no yaw-balance solver
#  behind it. It is the single least defensible number in this module, which is
#  why its band DELIBERATELY STARTS AT ZERO — "the penalty is negligible" is
#  inside the space the trade study explores, and if the conclusion flips inside
#  that band, trade.py will say so instead of picking a side.
LOCK_PENALTY_K_DEFAULT = 0.015      # lap-averaged fractional a_y loss at lock=1
LOCK_PENALTY_K_BAND = (0.0, 0.05)   # includes "no penalty at all", on purpose


def lock_lateral_derate(lock_ratio: float, k: float = LOCK_PENALTY_K_DEFAULT) -> float:
    """Multiplier on usable lateral grip due to diff lock. ESTIMATE — see the
    module docstring. Returns 1.0 for an open diff by construction."""
    lr = min(max(float(lock_ratio), 0.0), 1.0)
    return float(max(0.5, 1.0 - k * lr))


def apply_lock_derate(p: VehicleParams, lock_ratio: float,
                      k: float = LOCK_PENALTY_K_DEFAULT) -> VehicleParams:
    """Return a copy of the vehicle params with the lateral grip derated for
    diff lock. Applied to mu_peak so it flows through the whole grip stack."""
    return replace(p, mu_peak=p.mu_peak * lock_lateral_derate(lock_ratio, k))


# --------------------------------------------------------------------------- #
#  4.  Reference parts
# --------------------------------------------------------------------------- #
#  Every number here is either from the vendor listing or explicitly marked as a
#  team-supplied estimate. Nothing is invented to make a comparison come out.
CATALOG: dict[str, DifferentialSpec] = {
    "open": DifferentialSpec(
        name="Open differential (baseline)", kind=OPEN, tbr=1.0, preload_nm=0.0,
        mass_kg=None, cost_usd=None, is_estimate=True,
        source="Baseline concept — supply your own mass/cost."),
    "spool": DifferentialSpec(
        name="Spool / solid axle", kind=SPOOL, tbr=float("inf"), preload_nm=0.0,
        mass_kg=None, cost_usd=None, is_estimate=True,
        source="Baseline concept — supply your own mass/cost."),
    # --- the part this module was written to evaluate -------------------- #
    "tre_mk2_center": DifferentialSpec(
        name="TRE Mk2 Quaife ATB — Center Drive, no sprocket adapter",
        kind=HELICAL_ATB,
        tbr=2.5,                  # ESTIMATE: helical ATB class figure, NOT a
                                  # published TRE number. Override with the
                                  # value from the TRE ATB information PDF.
        preload_nm=25.0,          # ESTIMATE: shim/spring dependent, adjustable
        mass_kg=4.04,             # 8.9 lb "as little as", incl. bearings and
                                  # tripod housings — vendor listing
        cost_usd=2600.0,          # Center Drive / no Mk2.5 adjuster — listing
        adjustable_preload=False,
        sprocket_adapter_included=False,
        extra_fab_hours=25.0,     # ESTIMATE: design+make+check a sprocket carrier
        extra_parts_usd=250.0,    # ESTIMATE: carrier stock, sprocket, chain
        is_estimate=True,
        source=("taylor-race.com Mk2 listing: 8.9 lb incl. bearings and tripod "
                "housings; Center Drive + no adjuster variant $2,600. TBR and "
                "preload are CLASS ESTIMATES pending the TRE ATB PDF.")),
    "tre_mk2_center_adj": DifferentialSpec(
        name="TRE Mk2 Quaife ATB — Center Drive + Mk2.5 external preload adjuster",
        kind=HELICAL_ATB, tbr=2.5, preload_nm=25.0, mass_kg=4.2,
        cost_usd=3025.0, adjustable_preload=True,
        sprocket_adapter_included=False,
        extra_fab_hours=25.0, extra_parts_usd=250.0, is_estimate=True,
        source=("Same listing, Center Drive + Mk2.5 ADJUSTABLE preload variant "
                "$3,025. Mass is an ESTIMATE (+~0.15 kg for the adjuster).")),
}


def catalog(key: str, **overrides) -> DifferentialSpec:
    """Fetch a reference spec, overriding any field with your own measured or
    vendor-confirmed number. Overriding is the intended workflow."""
    base = CATALOG[key]
    return replace(base, **overrides) if overrides else base


PROVENANCE = {
    "physics_grounded": [
        "per-wheel rear vertical load (dynamics.lateral_load_transfer)",
        "load-sensitive tyre capacity (attached Pacejka or linear fallback)",
        "torque-split algebra (exact for a given TBR and preload)",
        "drive_grip_frac mapping (identical definition to lapsim's own use)",
    ],
    "estimate_flagged": [
        "friction CIRCLE in place of a measured combined-slip ellipse",
        "lateral force shared between rear tyres in proportion to capacity",
        "one corner-exit condition compressed into a single sim scalar",
        "lock/yaw penalty coefficient — swept, never asserted",
        "any TBR or preload not confirmed against the vendor's own data",
    ],
    "hard_rule": (
        "A differential's value is a difference between two traction limits, "
        "and that difference is smaller than the uncertainty in mu, TBR and the "
        "lock penalty. Single-point outputs from this module are inputs to a "
        "swept comparison, not answers. See trade.py."
    ),
}
