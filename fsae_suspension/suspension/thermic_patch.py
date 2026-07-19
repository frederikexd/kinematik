# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  suspension/thermic_patch.py — 🔥 ThermicPatch: a lightweight 3-node radial
#  thermodynamic ladder that marches ALONG an existing transient run and scales
#  Pacejka grip by tread-core temperature per instant.
# ============================================================================
"""
ThermicPatch — flash-heat grip degradation along the transient you already solve.

WHY THIS MODULE EXISTS, AND WHY IT IS NOT A DUPLICATE OF tire_thermal.py
-----------------------------------------------------------------------
`tire_thermal.py` already ships a full, honest lumped thermal *network* for the
co-simulation boundary (three masses, ideal-gas pressure rise, provenance flags).
That module is the right tool when you are stepping a tyre model forward yourself
inside a co-sim and want a physically complete channel. It is deliberately heavy:
it owns state, wants a `WheelState` per step, and is built to be *the* tyre.

ThermicPatch answers a different, narrower question that Ghost Topology raises and
nothing else in the repo answers: **given a transient run that is already solved
(`TransientResult`: Fz, Fy, Fx, alpha, u per millisecond), how does the contact
patch heat up over those few hundred milliseconds, and where does the core tread
temperature push grip in or out of the compound's thermal window mid-event?**

So this module is a *post-processor*, not a solver-in-the-loop. It reads the
already-solved force/slip history and integrates a 3-node radial ladder over it:

    Surface  (thin, fast) — takes the frictional flash heat, loses it to air +
                            the road, conducts down to Core.
    Core     (the grip-determining band) — its temperature drives mu(T).
    Carcass  (slow) — buffers heat and exchanges with the inflation-gas volume,
                      here folded into a fixed sink temperature (the gas node is
                      a slow reservoir over a ~0.5 s manoeuvre, so we do not carry
                      its ideal-gas pressure state — tire_thermal.py does that job).

WHAT IS PHYSICAL AND WHAT IS A PLACEHOLDER (read before trusting a number)
-------------------------------------------------------------------------
The EQUATIONS are a textbook explicit finite-difference energy balance and are
safe. The PARAMETERS (layer masses, split of tread mass into surface/core, the
inter-layer conductances, the mu(T) window) are REPRESENTATIVE, not measured —
they are reused from `tire_thermal.ThermalParams` precisely so ThermicPatch and
the co-sim channel cannot silently disagree about the same tyre. Every output
therefore carries `calibrated=False` unless the ThermalParams it was built from
is itself calibrated. Like the rest of the repo: ship the code, not a fabricated
temperature dressed up as measurement.

WHAT IT REPORTS
---------------
`run_thermic_patch(res, ...)` returns a `ThermicResult` with, per corner and per
instant: surface / core / carcass temperature, the frictional flash-heat flux,
the grip multiplier `mu_scale` from the core temperature, and the resulting loss
in available lateral force vs the same instant run cold-optimal. Plus a headline
`worst_instant()` that names the millisecond, the corner, and the grip drop — the
exact sentence Ghost Topology wants to print next to a compliance-induced camber
spike.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from .tire_thermal import ThermalParams, default_thermal_params


# --------------------------------------------------------------------------- #
#  Layer split — how the tread mass of the lumped model is divided into the
#  fast surface skin and the grip-determining core band for the ladder.
# --------------------------------------------------------------------------- #
@dataclass
class LadderParams:
    """
    The extra parameters the 3-node ladder needs on top of `ThermalParams`.

    These split the single `m_tread_kg` of the lumped model into a thin, fast
    SURFACE skin and a thicker CORE band, and set the conductances between the
    three radial nodes. Representative, uncalibrated — same status as everything
    in ThermalParams.
    """
    # Fraction of the tread mass that is the thin, fast-responding surface skin.
    # This is the top ~1-2 mm that actually sees the sliding friction — it must
    # be THIN (low heat capacity) or the flash spike is smeared out and never
    # leads the core, which is the physical signature the whole module reports.
    surface_mass_frac: float = 0.10
    # radial conductances (W/K) between the ladder nodes
    k_surface_core: float = 14.0        # surface skin <-> grip core
    k_core_carcass: float = 16.0        # grip core   <-> carcass
    # the carcass node exchanges with the (slow) inflation-gas reservoir, which
    # over a sub-second manoeuvre is effectively a fixed sink at its temperature
    k_carcass_gas: float = 6.0          # carcass <-> gas reservoir
    # Contact-patch conduction to the road. CRITICAL modelling point: the road is
    # only a heat sink over the CONTACT PATCH, not the whole tread band that the
    # air convects off. Applying ThermalParams.k_track over the full tread area
    # (the co-sim channel's convention) over-damps the flash spike — the very
    # thing this module exists to catch. So we scale the road conductance by the
    # patch's fraction of the tread, and the road itself is a WARMED track, not
    # ambient air: a sliding patch drags heat into the asphalt, which heats.
    surface_to_road_frac: float = 0.22  # contact-patch fraction of the tread band
    track_temp_c: float = 42.0          # warmed track surface the patch conducts to
    # explicit-FD safety: cap the internal sub-step so the ladder stays stable
    # even if the transient log is coarse. The ladder sub-steps to <= this.
    max_substep_s: float = 2.0e-3


def default_ladder_params() -> LadderParams:
    return LadderParams()


# --------------------------------------------------------------------------- #
#  Result container
# --------------------------------------------------------------------------- #
@dataclass
class ThermicResult:
    """
    Per-corner, per-instant thermal + grip history produced by ThermicPatch.

    All 2-D arrays are (n_samples, 4) in the FL, FR, RL, RR corner order that the
    transient solver uses. Temperatures are °C, fluxes W/m², grip multipliers
    dimensionless, force losses in newtons and as a fraction.
    """
    ok: bool
    t: np.ndarray                       # (n,)   time, s
    T_surface: np.ndarray               # (n,4)  surface skin temp, °C
    T_core: np.ndarray                  # (n,4)  grip core temp, °C
    T_carcass: np.ndarray               # (n,4)  carcass temp, °C
    q_flash: np.ndarray                 # (n,4)  frictional flash-heat flux, W/m²
    mu_scale: np.ndarray                # (n,4)  Pacejka-D multiplier from T_core
    Fy_cold: np.ndarray                 # (n,4)  |Fy| the instant would give at mu=1
    Fy_hot: np.ndarray                  # (n,4)  |Fy| after thermal scaling
    dFy_frac: np.ndarray                # (n,4)  fractional lateral-force loss
    T_opt_c: float                      # window centre used, °C
    calibrated: bool                    # True only if the ThermalParams was
    warnings: list[str] = field(default_factory=list)
    meta: dict = field(default_factory=dict)

    @staticmethod
    def failed(msg: str) -> "ThermicResult":
        z = np.zeros(1)
        z4 = np.zeros((1, 4))
        return ThermicResult(
            ok=False, t=z.copy(), T_surface=z4.copy(), T_core=z4.copy(),
            T_carcass=z4.copy(), q_flash=z4.copy(), mu_scale=np.ones((1, 4)),
            Fy_cold=z4.copy(), Fy_hot=z4.copy(), dFy_frac=z4.copy(),
            T_opt_c=85.0, calibrated=False, warnings=[msg])

    # ---- headline readouts --------------------------------------------- #
    def worst_instant(self) -> dict:
        """
        The single instant/corner with the largest thermal grip loss. This is the
        sentence Ghost Topology prints: when, which corner, how hot, how much grip.
        """
        out = {"ok": bool(self.ok)}
        if not self.ok or self.dFy_frac.size == 0:
            return out
        try:
            # Rank by NEWTONS of lateral force lost, not the bare fraction: a 45%
            # drop on a corner carrying 2 N is noise, a 12% drop on 1500 N is the
            # event. Newtons lost = Fy_cold * dFy_frac (== Fy_cold - Fy_hot).
            lost_N = self.Fy_cold * self.dFy_frac
            k = int(np.nanargmax(lost_N))
            i, ci = np.unravel_index(k, lost_N.shape)
            corner = ("FL", "FR", "RL", "RR")[ci]
            out.update(
                t_s=float(self.t[i]),
                corner=corner,
                T_surface_c=float(self.T_surface[i, ci]),
                T_core_c=float(self.T_core[i, ci]),
                mu_scale=float(self.mu_scale[i, ci]),
                dFy_frac=float(self.dFy_frac[i, ci]),
                Fy_lost_N=float(self.Fy_cold[i, ci] - self.Fy_hot[i, ci]),
            )
        except Exception:
            pass
        return out

    def peak_core_c(self) -> float:
        return float(np.nanmax(self.T_core)) if self.T_core.size else float("nan")

    def peak_surface_c(self) -> float:
        return float(np.nanmax(self.T_surface)) if self.T_surface.size else float("nan")

    def summary(self) -> dict:
        s = {"ok": bool(self.ok)}
        if not self.ok:
            s["warnings"] = list(self.warnings)
            return s
        try:
            s.update(
                peak_surface_c=self.peak_surface_c(),
                peak_core_c=self.peak_core_c(),
                min_mu_scale=float(np.nanmin(self.mu_scale)),
                max_dFy_frac=float(np.nanmax(self.dFy_frac)),
                calibrated=bool(self.calibrated),
            )
            s["worst"] = self.worst_instant()
        except Exception:
            pass
        return s


# --------------------------------------------------------------------------- #
#  The window law — a parabolic D-scaling centred on the compound's optimum.
#
#  The brief asks specifically for a parabola centred on the thermal window
#  (e.g. 75–90 °C) rather than the piecewise-linear law tire_thermal already
#  uses. We honour that here, while STILL bounding it by the same mu_floor and
#  reading the same window centre / calibration flag off ThermalParams, so the
#  two modules describe one tyre.
# --------------------------------------------------------------------------- #
def parabolic_mu_scale(T_core_c, tp: ThermalParams,
                       half_width_c: float = 35.0,
                       floor: Optional[float] = None) -> np.ndarray:
    """
    Grip multiplier D(T)/D_opt as a parabola centred on `tp.T_opt_c`, equal to
    1.0 at the optimum and falling to `floor` at +/- `half_width_c`. Vectorised.

    Asymmetric by construction on the hot side: overheating past the window is
    penalised harder than being equally cold, matching tire_thermal's steeper
    hot-side slope — the parabola's effective half-width is narrower above T_opt.
    """
    T = np.asarray(T_core_c, dtype=float)
    floor = tp.mu_floor if floor is None else float(floor)
    dT = T - tp.T_opt_c
    # hot side falls off faster: shrink the effective half-width above optimum in
    # the same ratio tire_thermal uses between its hot/cold linear slopes.
    hot_ratio = max(tp.mu_gain_per_C, 1e-6) / max(tp.mu_gain_per_C_hot, 1e-6)
    hw = np.where(dT >= 0.0, half_width_c * hot_ratio, half_width_c)
    frac = 1.0 - (dT / np.maximum(hw, 1e-6)) ** 2
    return np.clip(frac, floor, 1.0)


# --------------------------------------------------------------------------- #
#  The ladder integrator
# --------------------------------------------------------------------------- #
def _sliding_power_flux(Fx, Fy, alpha, u, tp: ThermalParams,
                        area_m2: float) -> np.ndarray:
    """
    Flash-heat generation flux q_gen (W/m²) at the contact patch.

    Closed form per the brief: the sliding-power product of shear force and
    sliding speed. The sliding velocity of the patch is approximated as the
    forward speed times sin(slip angle) for the lateral component; the fraction
    of that power deposited in the tread is tp.fric_to_tread. Divided by the
    patch/tread area to give a flux the ladder can drive a temperature with.
    """
    u = np.abs(np.asarray(u, dtype=float))
    Fy = np.abs(np.asarray(Fy, dtype=float))
    Fx = np.abs(np.asarray(Fx, dtype=float))
    # lateral sliding speed at the patch from the (lagged) slip angle
    v_slide = u * np.abs(np.sin(np.asarray(alpha, dtype=float)))
    # shear force doing the sliding work: lateral dominates in a cornering
    # transient; include a modest longitudinal contribution so combined events
    # (brake->throttle) still heat. Kept as |F|·v, not vectorial, on purpose —
    # this is a flash-heat magnitude, not a signed power.
    shear = np.hypot(Fy, 0.35 * Fx)
    p_slide = shear * v_slide                       # W per tyre
    q = tp.fric_to_tread * p_slide / max(area_m2, 1e-6)
    return np.maximum(q, 0.0)


def run_thermic_patch(res, tp: Optional[ThermalParams] = None,
                      lp: Optional[LadderParams] = None,
                      init_temp_c: float = 55.0,
                      ambient_c: float = 30.0,
                      gas_c: Optional[float] = None,
                      half_width_c: float = 35.0) -> ThermicResult:
    """
    March the 3-node radial ladder along a solved `TransientResult`.

    Parameters
    ----------
    res : TransientResult
        A solved transient run (from suspension.transient). Must carry t, Fz, Fy,
        Fx, alpha, u. A failed/empty result yields a failed ThermicResult.
    tp : ThermalParams, optional
        Thermal parameters (masses, conduction, convection, window). Defaults to
        the same representative FSAE set tire_thermal uses.
    lp : LadderParams, optional
        The ladder-specific split/conductances. Defaults to representative.
    init_temp_c : float
        Uniform starting temperature of all three nodes (a warmed tyre entering
        the event). °C.
    ambient_c : float
        Air temperature the surface convects to. °C.
    gas_c : float, optional
        Inflation-gas reservoir temperature the carcass exchanges with. Defaults
        to init_temp_c (a soaked tyre) if not given. °C.
    half_width_c : float
        Half-width of the parabolic grip window (cold side). °C.

    Never raises: on any fault returns `ThermicResult.failed(reason)`.
    """
    tp = tp or default_thermal_params()
    lp = lp or default_ladder_params()
    if gas_c is None:
        gas_c = init_temp_c

    # ---- guard the input ------------------------------------------------ #
    if res is None or not getattr(res, "ok", False):
        return ThermicResult.failed(
            "ThermicPatch needs a solved transient run; got a failed/empty one.")
    try:
        t = np.asarray(res.t, dtype=float).reshape(-1)
        Fz = np.asarray(res.Fz, dtype=float).reshape(len(t), 4)
        Fy = np.asarray(res.Fy, dtype=float).reshape(len(t), 4)
        Fx = np.asarray(res.Fx, dtype=float).reshape(len(t), 4)
        alpha = np.asarray(res.alpha, dtype=float).reshape(len(t), 4)
        u = np.asarray(res.u, dtype=float).reshape(-1)
    except Exception as e:
        return ThermicResult.failed(
            f"ThermicPatch could not read the transient arrays ({e}).")
    n = len(t)
    if n < 2:
        return ThermicResult.failed("Transient run too short to integrate heat.")

    # ---- node heat capacities (J/K) ------------------------------------ #
    m_surf = tp.m_tread_kg * lp.surface_mass_frac
    m_core = tp.m_tread_kg * (1.0 - lp.surface_mass_frac)
    C_surf = max(m_surf * tp.cp_tread, 1e-6)
    C_core = max(m_core * tp.cp_tread, 1e-6)
    C_carc = max(tp.m_carcass_kg * tp.cp_carcass, 1e-6)

    area = tp.area_tread_m2
    k_track = tp.k_track * lp.surface_to_road_frac

    # ---- flash-heat flux per instant (W/m²) then power into surface (W) - #
    q_flux = _sliding_power_flux(Fx, Fy, alpha, u[:, None], tp, area)   # (n,4)
    Q_surface_gen = q_flux * area                                       # W, (n,4)

    # rolling-hysteresis trickle into the carcass (small, keeps a soaked tyre
    # from cooling to ambient between slides). |omega|~u/r_eff.
    omega = np.abs(u) / max(tp.eff_radius_m, 1e-3)
    Q_roll = (tp.roll_resist_coeff * np.abs(Fz)
              * tp.eff_radius_m * omega[:, None])                       # W, (n,4)

    # ---- integrate the ladder ------------------------------------------ #
    Ts = np.empty((n, 4)); Tc = np.empty((n, 4)); Tk = np.empty((n, 4))
    Ts[0] = Tc[0] = Tk[0] = init_temp_c
    warnings: list[str] = []
    clamped = False

    for i in range(1, n):
        dt_log = float(t[i] - t[i - 1])
        if not np.isfinite(dt_log) or dt_log <= 0.0:
            Ts[i] = Ts[i - 1]; Tc[i] = Tc[i - 1]; Tk[i] = Tk[i - 1]
            continue
        # sub-step the explicit FD for stability on coarse logs
        n_sub = max(1, int(np.ceil(dt_log / lp.max_substep_s)))
        dt = dt_log / n_sub
        # Convection off the thin surface skin, W/K. The skin is only the top
        # ~1-2 mm; its exposed convective area is a fraction of the full tread
        # band's `area` (the rest of the band's air exchange belongs to the
        # deeper mass, which here is folded into the slower core/carcass path).
        # Scaling by the surface mass fraction keeps the skin from over-cooling
        # and lets a hard slide push it above the core, as a flash layer must.
        h = tp.h_air(u[i]) * area * lp.surface_mass_frac
        # linear interpolation of the drive terms across the log interval
        Qg = 0.5 * (Q_surface_gen[i] + Q_surface_gen[i - 1])           # (4,)
        Qr = 0.5 * (Q_roll[i] + Q_roll[i - 1])                         # (4,)
        loaded = 0.5 * (Fz[i] + Fz[i - 1]) > 1.0                       # patch down?
        s, c, k = Ts[i - 1].copy(), Tc[i - 1].copy(), Tk[i - 1].copy()
        for _ in range(n_sub):
            # surface: + friction gen, - air conv, - road conduction (if loaded),
            #          - conduction to core. Road conduction is to the WARMED
            #          track, not ambient air, and only over the contact patch.
            q_road = np.where(loaded, k_track * (s - lp.track_temp_c), 0.0)
            dS = (Qg
                  - h * (s - ambient_c)
                  - q_road
                  - lp.k_surface_core * (s - c)) / C_surf
            # core: + from surface, - to carcass  (no direct air path: buried)
            dC = (lp.k_surface_core * (s - c)
                  - lp.k_core_carcass * (c - k)) / C_core
            # carcass: + from core, + rolling trickle, - to gas reservoir
            dK = (lp.k_core_carcass * (c - k)
                  + Qr
                  - lp.k_carcass_gas * (k - gas_c)) / C_carc
            s = s + dt * dS
            c = c + dt * dC
            k = k + dt * dK
        # physical clamp (an explicit scheme on a pathological log can overshoot)
        for arr in (s, c, k):
            bad = ~np.isfinite(arr)
            if np.any(bad):
                clamped = True
                arr[bad] = init_temp_c
        s = np.clip(s, -40.0, 400.0)
        c = np.clip(c, -40.0, 400.0)
        k = np.clip(k, -40.0, 400.0)
        Ts[i], Tc[i], Tk[i] = s, c, k
    if clamped:
        warnings.append("ThermicPatch clamped a non-finite node temperature; "
                        "check the transient log's time base.")

    # ---- grip scaling from the CORE temperature ------------------------ #
    mu_scale = parabolic_mu_scale(Tc, tp, half_width_c=half_width_c)    # (n,4)
    Fy_cold = np.abs(Fy)
    Fy_hot = Fy_cold * mu_scale
    with np.errstate(divide="ignore", invalid="ignore"):
        dFy_frac = np.where(Fy_cold > 1.0, 1.0 - mu_scale, 0.0)

    return ThermicResult(
        ok=True, t=t, T_surface=Ts, T_core=Tc, T_carcass=Tk,
        q_flash=q_flux, mu_scale=mu_scale, Fy_cold=Fy_cold, Fy_hot=Fy_hot,
        dFy_frac=dFy_frac, T_opt_c=tp.T_opt_c, calibrated=bool(tp.calibrated),
        warnings=warnings,
        meta={"maneuver": (res.meta or {}).get("maneuver", ""),
              "init_temp_c": init_temp_c, "ambient_c": ambient_c,
              "gas_c": gas_c, "half_width_c": half_width_c})


# --------------------------------------------------------------------------- #
#  A convenience narrator — the exact Ghost-Topology-style verdict sentence.
# --------------------------------------------------------------------------- #
def verdict_sentence(tr: ThermicResult) -> str:
    """One human sentence naming the worst thermal grip loss in the run."""
    if not tr.ok:
        return "ThermicPatch: no valid thermal history (" + \
               "; ".join(tr.warnings) + ")."
    w = tr.worst_instant()
    if not w.get("ok") or w.get("dFy_frac", 0.0) < 1e-3:
        return ("No thermal grip window breach: the core tread stayed inside the "
                f"compound window (peak core {tr.peak_core_c():.0f}°C) through the "
                "event.")
    cal = "" if tr.calibrated else " (UNCALIBRATED — representative parameters)"
    return (f"At t={w['t_s']*1e3:.0f}ms, flash surface heat reaches "
            f"{w['T_surface_c']:.0f}°C and the {w['corner']} core tread hits "
            f"{w['T_core_c']:.0f}°C — outside the grip window centred on "
            f"{tr.T_opt_c:.0f}°C — dropping available lateral force by "
            f"{w['dFy_frac']*100:.0f}% ({w['Fy_lost_N']:.0f} N){cal}.")
