# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  Module: simulforge — the unified mechatronic co-solver
# ============================================================================
"""
SimulForge — the mechanical car and its electrical nervous system, solved as
ONE system of equations, with real-world electrical degradation wired straight
into the kinematic loop.

WHY THIS MODULE EXISTS
----------------------
Every simulation seat on a formula team draws the same silent boundary: the
vehicle-dynamics model assumes its actuators are IDEAL (infinite bandwidth,
perfect voltage, zero sag), and the electrical model assumes its loads are
STATIC (a worst-case current on a spreadsheet row). The wire between the pack
and the active anti-roll bar exists in neither model. Reality lives on that
wire, in three specific ways no siloed tool can see:

  1. THE ACTUATOR IS A CIRCUIT, NOT A TORQUE SOURCE. An active ARB servo is a
     motor: its torque is limited by the voltage it can push against its own
     back-EMF, its response is lagged by its winding inductance, and BOTH
     depend on the bus voltage AT THAT MILLISECOND — which depends on what
     every other load is drawing, which depends on what the car is doing.
     Feed it the pristine 24 V of the schematic and the simulated car gets
     roll authority the real car never had.

  2. DEGRADATION IS A KINEMATIC INPUT. A corroded connector adds milliohms; a
     tired pack adds internal resistance; both turn transient current draw
     into voltage sag, sag into lost actuator authority, lost authority into
     MORE roll, more roll into higher outer-wheel load cycles — a purely
     ELECTRICAL defect expressing itself as a purely MECHANICAL overload.
     The failure crosses the disciplinary boundary; the tools don't.

  3. THE STRUCTURE AND THE SESSION PAY THE BILL. The extra load cycles land
     on the same wishbone members the Ghost Topology audit margins, re-price
     the same first-failure pecking order Fusebox seals a charter over, and
     the energy the actuator burns comes out of the same pack budget Earshot
     converts into laps a test session can afford. One electrical defect,
     three ledgers — and today no tool posts the entry to any of them.

WHAT IT DOES
------------
  * LIVE ACTUATOR STATE-SPACE COUPLING. The transient vehicle DAE
    (transient.py: 24 mechanical states + the algebraic tyre/load block) is
    EXTENDED with the actuator electrical states — winding current per axle
    servo — and the bus algebraic constraint (voltage balance under the total
    draw). The combined system is semi-explicit index-1, exactly like the
    mechanical DAE alone: the bus voltage is an explicit function of the
    current states, so no implicit solve is added. Integration is a staggered
    (Gauss–Seidel) co-step at the mechanical dt: the electrical block is
    advanced with the EXACT exponential update of its linear RL dynamics
    (winding τ = L/R is comparable to the 1 ms mechanical step, so exact —
    not Euler — matters), the delivered roll moments are held zero-order over
    the mechanical RK4 step, and the resulting roll rate feeds back as
    back-EMF the next co-step. Loads migrate → currents change → the bus
    sags → the moments weaken → loads migrate. The loop is closed every
    millisecond, both directions.
  * THE CONTROLLER, HONESTLY LIMITED. A PD active-roll controller commands a
    restoring moment; the drive applies feed-forward voltage CLAMPED to the
    duty ceiling of the bus voltage it actually has. Authority is therefore
    an OUTPUT of the electrical state, never an input. A brownout latch
    models the ugly truth of digital controllers on a sagging bus: below the
    brownout threshold the controller is OFFLINE for a reboot time, the
    command is zero, and the car is passive whether the designer likes it or
    not.
  * THE MULTI-DISCIPLINARY FAILURE LINTER. Run the SAME manoeuvre twice —
    pristine electrics vs a declared degradation (sagging pack, corroded
    connector, browned-out controller, dead actuator) — and lint the pair
    across every ledger the defect touches:
      - RESPONSE: delivered-vs-commanded moment authority and lag, brownout
        dwell, peak sag — the electrical facts;
      - STRUCTURE: the load-cycle amplification per corner, and (when a
        GhostCorner is supplied) the transient FoS re-audited under the
        degraded load history — the ghost_topology stack as the structural
        judge, fifth consumer, zero new physics;
      - PECKING ORDER: a Fusebox overload path re-priced with the degraded
        loads, PLUS the electrical chain (fuse, connector, drive) racing the
        mechanical members for first failure in the same probability
        arithmetic — the fuse box and the wishbone finally in one ordering;
      - SESSION: the actuator's energy bill converted through Earshot's own
        laps_from_pack into laps LOST, and the A/B design re-judged — an
        electrical defect that quietly turns a RESOLVABLE test day
        UNDERPOWERED is named before the trailer loads.

THE HONESTY CONTRACT
--------------------
  * The staggered co-step is a co-simulation, and says so: exchange is at the
    mechanical dt with zero-order hold on the moments. The winding update is
    exact for the held inputs; the coupling error is O(dt) in the exchange,
    which at 1 ms against a >30 ms roll mode is the same time-scale-
    separation argument transient.py already stands on. `meta` records both.
  * Defaults are FSAE-representative stand-ins and say so; every parameter
    is a knob a team sets from a datasheet or a measurement.
  * Nothing in the public API raises. A failed run returns a flagged result;
    a lint over a failed run returns a lint that says what it couldn't judge.
  * The linter never manufactures certainty: findings carry the numbers they
    were computed from, and couplings that weren't supplied (no GhostCorner,
    no session design) are reported ABSENT, not silently skipped.

No streamlit / pandas / plotly imports. Unit-testable headless.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, asdict
from enum import Enum
from typing import Callable, Optional

import numpy as np

from . import transient as tr
from .transient import (TransientSolver, TransientParams, TransientResult,
                        DriverInput, RoadInput, N_STATES, IU, IPHI, IPHID,
                        FL, FR, RL, RR)
from . import ghost_topology as gt
from . import fusebox as fb
from . import earshot as es
from .proof_engine import EvidenceGrade

__all__ = [
    "ActuatorParams", "BusParams", "Degradation", "degradation_presets",
    "ForgeTelemetry", "SimulForgeResult", "MechatronicSolver",
    "run_simulforge", "ForgeThresholds", "ForgeFinding", "ForgeLint",
    "forge_lint", "electrical_path_elements", "render_forge_md",
]


# --------------------------------------------------------------------------- #
#  Parameters — the actuator, the bus, and the declared degradation
# --------------------------------------------------------------------------- #
@dataclass
class ActuatorParams:
    """
    One active anti-roll actuator: a DC servo + reduction driving a torsion
    element, folded to the roll axis. Everything is a datasheet number.

      R_ohm, L_H     : winding resistance / inductance (the L/R lag IS the
                       electrical response time — do not zero it).
      Kt, Ke         : torque and back-EMF constants (N·m/A, V·s/rad at the
                       MOTOR shaft; SI Kt == Ke for an ideal PMDC).
      gear           : motor rad per roll rad, actuator lever folded in.
      eff            : gearbox+drive efficiency (torque path).
      duty_max       : drive voltage ceiling as a fraction of the live bus.
      M_max_Nm       : mechanical torque ceiling at the roll axis (hard stop /
                       structural rating of the bar), applied to the COMMAND.
      kp, kd         : PD active-roll gains (N·m per rad, N·m per rad/s).
      i_quiescent_A  : controller + drive idle draw.
      fuse_rating_A  : the actuator branch fuse (Fusebox's electrical racer).
      conn_rating_A  : the branch connector's continuous rating.
    Defaults are representative of a ~200 W FSAE active-ARB servo and say so.
    """
    R_ohm: float = 0.55
    L_H: float = 0.9e-3
    Kt: float = 0.055
    Ke: float = 0.055
    gear: float = 120.0
    eff: float = 0.72
    duty_max: float = 0.95
    M_max_Nm: float = 900.0
    kp: float = 12_000.0
    kd: float = 900.0
    i_quiescent_A: float = 0.35
    fuse_rating_A: float = 25.0
    conn_rating_A: float = 20.0

    @property
    def tau_ms(self) -> float:
        return 1000.0 * self.L_H / max(self.R_ohm, 1e-9)

    def torque_from_current(self, i_A: float) -> float:
        """Delivered roll-axis moment for a winding current (signed)."""
        return self.Kt * self.gear * self.eff * i_A

    def current_for_torque(self, M_Nm: float) -> float:
        k = self.Kt * self.gear * self.eff
        return M_Nm / k if k > 0 else 0.0


@dataclass
class BusParams:
    """
    The LV bus feeding the actuators: an open-circuit source behind a series
    resistance chain. The chain is where degradation lives.

      V_oc            : open-circuit (rested) bus voltage.
      R_int_ohm       : source internal resistance (pack + BMS + main run).
      R_harness_ohm   : branch harness + connector resistance (NOMINAL).
      I_other_A       : everything else on the bus (pumps, fans, logger).
      V_brownout      : controller resets below this.
      reboot_s        : time offline after a brownout reset.
      pack_kwh, usable_frac, kwh_per_lap, reserve_kwh :
                        the Earshot session-budget channel — same currency
                        the endurance energy budget runs on.
    """
    V_oc: float = 25.2
    R_int_ohm: float = 0.045
    R_harness_ohm: float = 0.020
    I_other_A: float = 6.0
    V_brownout: float = 17.5
    reboot_s: float = 0.35
    pack_kwh: float = 6.5
    usable_frac: float = 0.85
    kwh_per_lap: float = 0.28
    reserve_kwh: float = 0.4

    @property
    def r_total_ohm(self) -> float:
        return max(self.R_int_ohm + self.R_harness_ohm, 0.0)


@dataclass
class Degradation:
    """
    One declared real-world electrical defect, applied to the pristine
    bus/actuator pair. Multipliers of 1 and additions of 0 are the pristine
    car; every field is one physical story.

      r_int_scale       : pack ageing / cold pack (× on R_int).
      r_harness_add_ohm : corrosion / a crimped-not-soldered joint (+ ohms).
      v_oc_drop         : starting the session down on charge (− volts).
      i_other_add_A     : an undeclared load that appeared on the bus.
      authority_scale   : mechanical de-rate of the actuator (0 = dead servo,
                          the unplugged-connector case).
      v_brownout_add    : a marginal regulator raising the effective reset
                          threshold (+ volts).
    """
    key: str = "nominal"
    label: str = "Pristine electrics"
    story: str = "The schematic's car: rested pack, clean connectors."
    r_int_scale: float = 1.0
    r_harness_add_ohm: float = 0.0
    v_oc_drop: float = 0.0
    i_other_add_A: float = 0.0
    authority_scale: float = 1.0
    v_brownout_add: float = 0.0

    def apply(self, bus: BusParams, act: ActuatorParams
              ) -> tuple[BusParams, ActuatorParams]:
        """Return DEGRADED COPIES; never mutates the inputs."""
        b = BusParams(**asdict(bus))
        a = ActuatorParams(**asdict(act))
        b.R_int_ohm *= max(self.r_int_scale, 0.0)
        b.R_harness_ohm += max(self.r_harness_add_ohm, 0.0)
        b.V_oc = max(b.V_oc - max(self.v_oc_drop, 0.0), 0.0)
        b.I_other_A += max(self.i_other_add_A, 0.0)
        b.V_brownout += max(self.v_brownout_add, 0.0)
        s = float(np.clip(self.authority_scale, 0.0, 1.0))
        a.M_max_Nm *= s
        a.duty_max *= s if s > 0 else 0.0
        return b, a


def degradation_presets() -> dict[str, Degradation]:
    """The named defects the linter ships with. Each is one field-report."""
    return {
        "nominal": Degradation(),
        "sagging_pack": Degradation(
            key="sagging_pack", label="Sagging pack",
            story="An aged / cold pack: internal resistance up 3×, starting "
                  "1.5 V down. Every transient amp now costs real volts.",
            r_int_scale=3.0, v_oc_drop=1.5),
        "corroded_connector": Degradation(
            key="corroded_connector", label="Corroded connector",
            story="One branch connector gone green: +120 mΩ in series. "
                  "Invisible at idle, a voltage cliff under actuator load.",
            r_harness_add_ohm=0.120),
        "brownout_margin": Degradation(
            key="brownout_margin", label="Brownout-marginal controller",
            story="A marginal regulator: the controller resets 2.5 V earlier "
                  "than spec, and rides the sag into reboot loops.",
            v_brownout_add=2.5, r_int_scale=1.6),
        "dead_actuator": Degradation(
            key="dead_actuator", label="Dead actuator",
            story="The connector vibrated off on the trailer: full passive "
                  "car, quiescent draw still on the bus.",
            authority_scale=0.0),
    }


# --------------------------------------------------------------------------- #
#  Result — the mechanical trace plus the electrical one, one object
# --------------------------------------------------------------------------- #
@dataclass
class ForgeTelemetry:
    """Electrical/actuator history, sampled at the mechanical log rate."""
    t: np.ndarray
    V_bus: np.ndarray            # live bus voltage, V
    I_bus: np.ndarray            # total bus draw, A
    i_act: np.ndarray            # (n, 2) winding current per axle [F, R], A
    M_cmd: np.ndarray            # (n, 2) commanded roll moment, N·m
    M_act: np.ndarray            # (n, 2) delivered roll moment, N·m
    online: np.ndarray           # controller-online mask (bool)
    E_Wh: np.ndarray             # cumulative bus energy, Wh
    n_brownouts: int = 0

    def summary(self) -> dict:
        ok = self.t.size > 0
        cmd = np.abs(self.M_cmd).sum(axis=1) if ok else np.zeros(1)
        act = np.abs(self.M_act).sum(axis=1) if ok else np.zeros(1)
        peak_cmd = float(np.max(cmd)) if ok else 0.0
        authority = float(np.max(act) / peak_cmd) if peak_cmd > 1e-9 else \
            (1.0 if ok else 0.0)
        return {
            "v_min": float(np.min(self.V_bus)) if ok else float("nan"),
            "i_peak": float(np.max(self.I_bus)) if ok else float("nan"),
            "sag_peak_V": (float(self.V_bus[0] - np.min(self.V_bus))
                           if ok else float("nan")),
            "authority": authority,
            "offline_ms": (float(np.sum(~self.online))
                           * (float(self.t[1] - self.t[0]) * 1000.0
                              if self.t.size > 1 else 0.0)),
            "n_brownouts": self.n_brownouts,
            "energy_Wh": float(self.E_Wh[-1]) if ok else 0.0,
            "response_lag_ms": self.response_lag_ms(),
        }

    def response_lag_ms(self) -> float:
        """
        Delivered-vs-commanded lag: the shift (ms) maximising the cross-
        correlation of |ΣM_act| against |ΣM_cmd| over a ±50 ms window. 0 for
        an ideal actuator; grows with winding τ and with sag-clamped drive.
        """
        if self.t.size < 8:
            return 0.0
        dt = float(self.t[1] - self.t[0])
        a = np.abs(self.M_cmd).sum(axis=1)
        b = np.abs(self.M_act).sum(axis=1)
        a = a - a.mean(); b = b - b.mean()
        if np.allclose(a, 0) or np.allclose(b, 0):
            return 0.0
        w = max(int(round(0.050 / dt)), 1)
        best, best_lag = -np.inf, 0
        for lag in range(0, w + 1):
            v = float(np.dot(a[:a.size - lag], b[lag:]))
            if v > best:
                best, best_lag = v, lag
        return best_lag * dt * 1000.0


@dataclass
class SimulForgeResult:
    """One co-solved run: the mechanical result and its electrical shadow."""
    mech: TransientResult
    elec: ForgeTelemetry
    degradation: Degradation
    bus: BusParams
    actuator: ActuatorParams
    warnings: list = field(default_factory=list)
    meta: dict = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        return bool(getattr(self.mech, "ok", False))

    def load_cycle_ptp_N(self) -> np.ndarray:
        """Per-corner contact-load cycle amplitude (peak-to-peak Fz), N."""
        if not self.ok or self.mech.Fz.size == 0:
            return np.zeros(4)
        return np.ptp(self.mech.Fz, axis=0)

    def roll_peak_deg(self) -> float:
        if not self.ok or self.mech.roll.size == 0:
            return float("nan")
        return float(np.max(np.abs(self.mech.roll))) * 180.0 / math.pi


# --------------------------------------------------------------------------- #
#  The co-solver
# --------------------------------------------------------------------------- #
class MechatronicSolver(TransientSolver):
    """
    TransientSolver with the actuator moments injected into the algebraic
    block, and a co-stepping `run_coupled` that advances the electrical
    states between mechanical steps.

    The injection is exact w.r.t. the parent model: the delivered per-axle
    roll moment is converted to the equivalent left/right wheel-force couple
    at the half-track, added to F_susp — the same channel the passive ARB
    uses — so the sprung roll equation, the unsprung reaction and the tyre
    loads all see it through the parent's own force balance. The moment is
    held zero-order across the RK4 sub-steps of one mechanical dt (the
    co-simulation exchange), which is the stated coupling approximation.
    """

    def __init__(self, *args, **kw):
        super().__init__(*args, **kw)
        # held delivered roll moments [front, rear], N·m, + in +phi direction
        self._M_held = np.zeros(2)

    # ---- the injected algebraic block ----------------------------------- #
    def algebraic(self, t, y, driver, road) -> dict:  # noqa: D401
        A = super().algebraic(t, y, driver, road)
        Mf, Mr = float(self._M_held[0]), float(self._M_held[1])
        if Mf != 0.0 or Mr != 0.0:
            y_i = A["y_i"]
            p = self.p
            # per-wheel force c·sign(y_i) adds −t_axle·c to the roll moment
            # (see derivatives' M_roll_susp = Σ(−y_i·F)); c = −M/t injects +M.
            cf = -Mf / max(p.track_front, 1e-3)
            cr = -Mr / max(p.track_rear, 1e-3)
            F_act = np.array([cf, cf, cr, cr]) * np.sign(y_i)
            A["F_susp"] = A["F_susp"] + F_act
            A["F_act"] = F_act
        else:
            A["F_act"] = np.zeros(4)
        return A

    # ---- the coupled run ------------------------------------------------- #
    def run_coupled(self, t_end: float,
                    bus: BusParams, act: ActuatorParams,
                    driver: DriverInput | None = None,
                    road: RoadInput | None = None,
                    u0: float = 10.0) -> SimulForgeResult:
        """
        Staggered co-integration, exchange at the mechanical dt:

          per step k:
            1. read roll state (φ, φ̇) from the mechanical state;
            2. controller: M_cmd = clip(−(kp·φ + kd·φ̇), ±M_max) per axle,
               zero while the brownout latch holds;
            3. drive: V_app = clip(R·i_ref + Ke·ω, ±duty_max·V_bus) — the
               feed-forward voltage the sagging bus can actually supply;
            4. winding: EXACT exponential update of L di/dt = V_app − R·i −
               Ke·ω over dt (inputs held);
            5. bus (algebraic): I = Σ|i| + quiescent + others,
               V_bus = V_oc − I·R_total; brownout latch below V_brownout;
            6. delivered M = Kt·gear·eff·i, held; mechanical RK4 step.

        Never raises; a blow-up returns a flagged result like the parent.
        """
        self.warnings = []
        self._peak_cache.clear()
        self._last_ax = 0.0
        self._last_ay = 0.0
        driver = driver or DriverInput()
        road = road or RoadInput()
        p = self.p
        try:
            y = self.initial_state(u0)
            dt = max(float(p.dt), 1e-5)
            nsub = max(int(p.substeps), 1)
            hsub = dt / nsub
            n = max(int(round(float(t_end) / dt)), 1)

            # --- electrical state ---
            i_w = np.zeros(2)                    # winding currents [F, R]
            V_bus = bus.V_oc - (2 * act.i_quiescent_A + bus.I_other_A) \
                * bus.r_total_ohm
            offline_until = -1.0
            n_brownouts = 0
            E_Wh = 0.0
            tau = act.L_H / max(act.R_ohm, 1e-9)
            decay = math.exp(-dt / max(tau, 1e-9))

            # --- logs ---
            m = n + 1
            L_t = np.zeros(m); L_V = np.zeros(m); L_I = np.zeros(m)
            L_i = np.zeros((m, 2)); L_cmd = np.zeros((m, 2))
            L_act = np.zeros((m, 2)); L_on = np.ones(m, bool)
            L_E = np.zeros(m)

            def elec_step(tt: float) -> tuple[np.ndarray, float, float, bool]:
                nonlocal i_w, V_bus, offline_until, n_brownouts, E_Wh
                phi = float(y[IPHI]); phid = float(y[IPHID])
                online = tt >= offline_until
                # 2. controller command (roll-restoring, per axle: split 50/50)
                if online and act.M_max_Nm > 0:
                    M_tot = -(act.kp * phi + act.kd * phid)
                    M_cmd = np.clip(np.array([0.5, 0.5]) * M_tot,
                                    -act.M_max_Nm, act.M_max_Nm)
                else:
                    M_cmd = np.zeros(2)
                # 3.–4. drive + exact winding update, per axle
                omega = act.gear * phid          # motor speed, rad/s
                v_ceiling = act.duty_max * max(V_bus, 0.0)
                drive_live = online and v_ceiling > 0.0
                for k2 in range(2):
                    if not drive_live:
                        # H-bridge tristated (offline / dead branch): the
                        # winding is an OPEN circuit — no back-EMF current.
                        i_w[k2] = 0.0
                        continue
                    i_ref = act.current_for_torque(float(M_cmd[k2]))
                    v_ff = act.R_ohm * i_ref + act.Ke * omega
                    v_app = float(np.clip(v_ff, -v_ceiling, v_ceiling))
                    i_inf = (v_app - act.Ke * omega) / max(act.R_ohm, 1e-9)
                    i_w[k2] = i_inf + (i_w[k2] - i_inf) * decay
                # 5. bus algebraic + brownout latch
                I_bus = float(np.sum(np.abs(i_w))
                              + 2 * act.i_quiescent_A + bus.I_other_A)
                V_bus = max(bus.V_oc - I_bus * bus.r_total_ohm, 0.0)
                if online and V_bus < bus.V_brownout:
                    offline_until = tt + max(bus.reboot_s, 0.0)
                    n_brownouts += 1
                    online = False
                    i_w *= 0.0                   # drive tristates on reset
                    M_cmd = np.zeros(2)
                E_Wh += V_bus * I_bus * dt / 3600.0
                # 6. delivered moments, held for the mechanical step
                M_del = np.array([act.torque_from_current(i_w[0]),
                                  act.torque_from_current(i_w[1])])
                self._M_held = M_del
                return M_cmd, I_bus, V_bus, online

            # mechanical log arrays (parent shapes)
            T = np.zeros(m)
            U = np.zeros(m); V = np.zeros(m); R = np.zeros(m)
            BETA = np.zeros(m); AX = np.zeros(m); AY = np.zeros(m)
            GX = np.zeros(m); GY = np.zeros(m); PSI = np.zeros(m)
            HEAVE = np.zeros(m); PITCH = np.zeros(m); ROLL = np.zeros(m)
            FZ = np.zeros((m, 4)); FY = np.zeros((m, 4))
            FX = np.zeros((m, 4)); AL = np.zeros((m, 4)); SV = np.zeros((m, 4))
            ST = np.zeros(m); THr = np.zeros(m); BR = np.zeros(m)

            def log(idx, tt, yy, M_cmd, I_bus, V_now, online):
                A = self.algebraic(tt, yy, driver, road)
                T[idx] = tt
                U[idx] = yy[IU]; V[idx] = yy[1]; R[idx] = yy[2]
                BETA[idx] = math.atan2(yy[1], max(abs(yy[IU]), p.u_min))
                AX[idx] = self._last_ax; AY[idx] = self._last_ay
                GX[idx] = yy[3]; GY[idx] = yy[4]; PSI[idx] = yy[5]
                HEAVE[idx] = yy[6]; PITCH[idx] = yy[10]; ROLL[idx] = yy[IPHI]
                FZ[idx] = A["Fz"]; FY[idx] = A["Fy"]; FX[idx] = A["Fx"]
                AL[idx] = yy[20:24]; SV[idx] = A["delta_vel"]
                ST[idx] = A["steer"]; THr[idx] = A["throttle"]
                BR[idx] = A["brake"]
                L_t[idx] = tt; L_V[idx] = V_now; L_I[idx] = I_bus
                L_i[idx] = i_w; L_cmd[idx] = M_cmd
                L_act[idx] = self._M_held; L_on[idx] = online
                L_E[idx] = E_Wh

            # prime and log t = 0
            self._M_held = np.zeros(2)
            self.derivatives(0.0, y, driver, road)
            log(0, 0.0, y, np.zeros(2), 2 * act.i_quiescent_A + bus.I_other_A,
                V_bus, True)

            blew_up = False
            for k in range(1, n + 1):
                tt = (k - 1) * dt
                M_cmd, I_bus, V_now, online = elec_step(tt)
                for _ in range(nsub):
                    y = self._rk4_step(tt, y, hsub, driver, road)
                    tt += hsub
                    if not np.all(np.isfinite(y)):
                        blew_up = True
                        break
                    y = self._clamp(y)
                if blew_up:
                    self._warn("Coupled integration produced a non-finite "
                               "state and was stopped early; trace truncated.")
                    sl = slice(0, k)
                    T, U, V, R, BETA = T[sl], U[sl], V[sl], R[sl], BETA[sl]
                    AX, AY, GX, GY, PSI = AX[sl], AY[sl], GX[sl], GY[sl], PSI[sl]
                    HEAVE, PITCH, ROLL = HEAVE[sl], PITCH[sl], ROLL[sl]
                    FZ, FY, FX, AL, SV = FZ[sl], FY[sl], FX[sl], AL[sl], SV[sl]
                    ST, THr, BR = ST[sl], THr[sl], BR[sl]
                    L_t, L_V, L_I = L_t[sl], L_V[sl], L_I[sl]
                    L_i, L_cmd, L_act = L_i[sl], L_cmd[sl], L_act[sl]
                    L_on, L_E = L_on[sl], L_E[sl]
                    break
                self.derivatives(k * dt, y, driver, road)
                log(k, k * dt, y, M_cmd, I_bus, V_now, online)

            mech = TransientResult(
                ok=not blew_up, t=T, u=U, v=V, r=R, beta=BETA, ax=AX, ay=AY,
                X=GX, Y=GY, psi=PSI, heave=HEAVE, pitch=PITCH, roll=ROLL,
                Fz=FZ, Fy=FY, Fx=FX, alpha=AL, susp_vel=SV,
                steer=ST, throttle=THr, brake=BR,
                warnings=list(self.warnings),
                meta=dict(dt=dt, substeps=nsub, n_steps=len(T),
                          coupling="staggered Gauss-Seidel, ZOH moments, "
                                   "exact RL winding update",
                          winding_tau_ms=tau * 1000.0,
                          izz=p.izz_eff(), m_sprung=p.m_sprung))
            elec = ForgeTelemetry(t=L_t, V_bus=L_V, I_bus=L_I, i_act=L_i,
                                  M_cmd=L_cmd, M_act=L_act, online=L_on,
                                  E_Wh=L_E, n_brownouts=n_brownouts)
            return SimulForgeResult(mech=mech, elec=elec,
                                    degradation=Degradation(), bus=bus,
                                    actuator=act,
                                    warnings=list(self.warnings),
                                    meta=dict(mech.meta))
        except Exception as e:                                  # noqa: BLE001
            mech = TransientResult.failed(
                self.warnings + [f"Coupled run failed entirely "
                                 f"({type(e).__name__}: {e})."])
            elec = ForgeTelemetry(t=np.zeros(0), V_bus=np.zeros(0),
                                  I_bus=np.zeros(0), i_act=np.zeros((0, 2)),
                                  M_cmd=np.zeros((0, 2)),
                                  M_act=np.zeros((0, 2)),
                                  online=np.zeros(0, bool), E_Wh=np.zeros(0))
            return SimulForgeResult(mech=mech, elec=elec,
                                    degradation=Degradation(), bus=bus,
                                    actuator=act, warnings=mech.warnings)


def run_simulforge(veh, kind: str = "step_steer",
                   degradation: Degradation | str | None = None,
                   bus: BusParams | None = None,
                   actuator: ActuatorParams | None = None,
                   params: TransientParams | None = None,
                   **maneuver_kw) -> SimulForgeResult:
    """
    Build and co-solve one named manoeuvre under one declared degradation.
    `kind` is transient.py's set {step_steer, snap_oversteer,
    brake_to_throttle, curb_strike}. Never raises.
    """
    bus = bus or BusParams()
    actuator = actuator or ActuatorParams()
    if isinstance(degradation, str):
        degradation = degradation_presets().get(degradation, Degradation())
    degradation = degradation or Degradation()
    d_bus, d_act = degradation.apply(bus, actuator)
    builders = {
        "step_steer": tr.step_steer_maneuver,
        "snap_oversteer": tr.snap_oversteer_maneuver,
        "brake_to_throttle": tr.brake_to_throttle_maneuver,
        "curb_strike": tr.curb_strike_maneuver,
    }
    try:
        builder = builders.get(kind)
        if builder is None:
            res = SimulForgeResult(
                mech=TransientResult.failed(
                    [f"Unknown manoeuvre '{kind}'. Options: "
                     f"{sorted(builders)}."]),
                elec=ForgeTelemetry(np.zeros(0), np.zeros(0), np.zeros(0),
                                    np.zeros((0, 2)), np.zeros((0, 2)),
                                    np.zeros((0, 2)), np.zeros(0, bool),
                                    np.zeros(0)),
                degradation=degradation, bus=d_bus, actuator=d_act)
            res.warnings = list(res.mech.warnings)
            return res
        drv, road, t_end, u0, label = builder(**maneuver_kw)
        sim = MechatronicSolver(veh, params=params)
        out = sim.run_coupled(t_end, d_bus, d_act, driver=drv, road=road,
                              u0=u0)
        out.degradation = degradation
        out.meta["maneuver"] = label
        out.mech.meta["maneuver"] = label
        return out
    except Exception as e:                                      # noqa: BLE001
        return SimulForgeResult(
            mech=TransientResult.failed([f"SimulForge '{kind}' failed "
                                         f"({type(e).__name__}: {e})."]),
            elec=ForgeTelemetry(np.zeros(0), np.zeros(0), np.zeros(0),
                                np.zeros((0, 2)), np.zeros((0, 2)),
                                np.zeros((0, 2)), np.zeros(0, bool),
                                np.zeros(0)),
            degradation=degradation, bus=d_bus, actuator=d_act)


# --------------------------------------------------------------------------- #
#  The multi-disciplinary failure linter
# --------------------------------------------------------------------------- #
class ForgeVerdict(str, Enum):
    """Ordered worst-first; the governing verdict is the first that fires."""
    STRUCTURAL_REGRESSION = "STRUCTURAL_REGRESSION"
    ORDER_INVERTED = "ORDER_INVERTED"
    BROWNOUT = "BROWNOUT"
    AUTHORITY_LOST = "AUTHORITY_LOST"
    SESSION_STARVED = "SESSION_STARVED"
    RESPONSE_LAGGED = "RESPONSE_LAGGED"
    COUPLED_FAITHFUL = "COUPLED_FAITHFUL"


@dataclass
class ForgeThresholds:
    """The lint gates. Every one is a judgement knob and is printed."""
    authority_min: float = 0.85      # delivered/commanded floor
    lag_ms_max: float = 12.0         # response lag ceiling
    load_cycle_amp_max: float = 1.10 # degraded/nominal Fz-cycle ceiling
    roll_amp_max: float = 1.15       # degraded/nominal roll-peak ceiling
    fos_drop_max: float = 0.10       # allowed worst-FoS drop (absolute)
    fos_limit: float = 1.5           # the standing structural rule


@dataclass
class ForgeFinding:
    """One attributed lint finding."""
    code: str
    severity: str                    # "red" | "amber" | "info"
    text: str
    data: dict = field(default_factory=dict)


@dataclass
class ForgeLint:
    """The full multi-disciplinary lint of one nominal/degraded pair."""
    verdict: str
    flags: list
    findings: list                   # list[ForgeFinding]
    nominal: SimulForgeResult
    degraded: SimulForgeResult
    thresholds: ForgeThresholds
    ghost_nominal: Optional[gt.GhostAudit] = None
    ghost_degraded: Optional[gt.GhostAudit] = None
    path_audit_nominal: Optional[fb.PathAudit] = None
    path_audit_degraded: Optional[fb.PathAudit] = None
    session_before: Optional[dict] = None
    session_after: Optional[dict] = None
    note: str = ""

    def summary(self) -> dict:
        return {
            "verdict": self.verdict, "flags": list(self.flags),
            "degradation": self.degraded.degradation.label,
            "findings": [asdict(f) for f in self.findings],
            "elec_nominal": self.nominal.elec.summary(),
            "elec_degraded": self.degraded.elec.summary(),
            "note": self.note,
        }


def electrical_path_elements(res: SimulForgeResult,
                             grade: EvidenceGrade = EvidenceGrade.MODELLED
                             ) -> list[fb.PathElement]:
    """
    The actuator branch's electrical chain as Fusebox racers: capacity /
    peak-demand in the SAME FoS currency the mechanical members use, so the
    branch fuse, the connector and the wishbone race in one ordering.
    """
    act = res.actuator
    i_pk = float(np.max(np.abs(res.elec.i_act))) if res.elec.i_act.size \
        else 0.0
    i_pk = max(i_pk, 1e-6)
    return [
        fb.PathElement(key="branch_fuse", label="Actuator branch fuse",
                       fos=act.fuse_rating_A / i_pk, grade=grade,
                       severity=fb.Severity.S1_FUSE_GRADE,
                       replace_cost_usd=2.0, downtime_days=0.02,
                       note=f"rating {act.fuse_rating_A:.0f} A vs peak draw "
                            f"{i_pk:.1f} A in this event"),
        fb.PathElement(key="branch_conn", label="Actuator branch connector",
                       fos=act.conn_rating_A / i_pk, grade=grade,
                       severity=fb.Severity.S2_STRUCTURAL,
                       replace_cost_usd=18.0, downtime_days=0.2,
                       note=f"continuous rating {act.conn_rating_A:.0f} A vs "
                            f"peak {i_pk:.1f} A"),
    ]


def _mech_elements_from_ghost(audit: gt.GhostAudit,
                              base: Optional[fb.OverloadPath]
                              ) -> list[fb.PathElement]:
    """
    Per-member worst transient FoS from a ghost audit, as PathElements.
    Costs/grades inherited from a base path where member keys match.
    """
    worst: dict[str, float] = {}
    for g in audit.instants:
        for mkey, mg in g.margins.items():
            f = float(mg.get("fos", float("inf")))
            if math.isfinite(f):
                worst[mkey] = min(worst.get(mkey, float("inf")), f)
    out = []
    for mkey, f in sorted(worst.items()):
        proto = base.element(mkey) if base is not None else None
        out.append(fb.PathElement(
            key=mkey, label=(proto.label if proto else f"Member {mkey}"),
            fos=max(f, 1e-3),
            grade=(proto.grade if proto else EvidenceGrade.MODELLED),
            severity=(proto.severity if proto else fb.Severity.S2_STRUCTURAL),
            replace_cost_usd=(proto.replace_cost_usd if proto else 60.0),
            downtime_days=(proto.downtime_days if proto else 1.0),
            note="worst transient FoS from the ghost audit of this event"))
    return out


def forge_lint(nominal: SimulForgeResult, degraded: SimulForgeResult,
               gc: Optional[gt.GhostCorner] = None,
               corner: str = "FR",
               base_path: Optional[fb.OverloadPath] = None,
               designated_fuse_key: str = "branch_fuse",
               ab_design: Optional[es.ABDesign] = None,
               laps_per_session_est: float = 20.0,
               thresholds: Optional[ForgeThresholds] = None) -> ForgeLint:
    """
    Lint the pristine/degraded pair across every ledger the defect touches.
    Optional couplings — GhostCorner (structure), a base OverloadPath
    (pecking order), an ABDesign (session) — deepen the lint when supplied
    and are reported ABSENT when not. Never raises.
    """
    th = thresholds or ForgeThresholds()
    findings: list[ForgeFinding] = []
    flags: list[str] = []
    lint = ForgeLint(verdict=ForgeVerdict.COUPLED_FAITHFUL.value, flags=flags,
                     findings=findings, nominal=nominal, degraded=degraded,
                     thresholds=th)
    try:
        if not nominal.ok or not degraded.ok:
            lint.note = ("one or both co-solved runs are flagged failed — "
                         "the lint judged only what survived: "
                         + "; ".join((nominal.warnings + degraded.warnings)[:2]))
        # ---- 1. RESPONSE — the electrical facts -------------------------- #
        en, ed = nominal.elec.summary(), degraded.elec.summary()
        if ed["n_brownouts"] > 0:
            flags.append(ForgeVerdict.BROWNOUT.value)
            findings.append(ForgeFinding(
                "BROWNOUT", "red",
                f"The controller browned out {ed['n_brownouts']}× "
                f"(bus min {ed['v_min']:.1f} V vs threshold "
                f"{degraded.bus.V_brownout:.1f} V), spending "
                f"{ed['offline_ms']:.0f} ms of the event passive mid-"
                f"manoeuvre — the worst possible moment to lose the bar.",
                {"v_min": ed["v_min"], "offline_ms": ed["offline_ms"]}))
        if ed["authority"] < th.authority_min * max(en["authority"], 1e-9):
            flags.append(ForgeVerdict.AUTHORITY_LOST.value)
            findings.append(ForgeFinding(
                "AUTHORITY_LOST", "red",
                f"Delivered roll authority fell to {ed['authority']:.0%} of "
                f"command (pristine: {en['authority']:.0%}) — the sagging "
                f"drive voltage cannot push the commanded current against "
                f"back-EMF. Sag peak {ed['sag_peak_V']:.1f} V at "
                f"{ed['i_peak']:.1f} A.",
                {"authority": ed["authority"], "sag_V": ed["sag_peak_V"]}))
        lag_n, lag_d = en["response_lag_ms"], ed["response_lag_ms"]
        if lag_d > max(th.lag_ms_max, 1.5 * max(lag_n, 1e-9)):
            flags.append(ForgeVerdict.RESPONSE_LAGGED.value)
            findings.append(ForgeFinding(
                "RESPONSE_LAGGED", "amber",
                f"Actuator response lag grew from {lag_n:.1f} ms to "
                f"{lag_d:.1f} ms — the bar now arrives after the load "
                f"transient it was meant to blunt.",
                {"lag_nominal_ms": lag_n, "lag_degraded_ms": lag_d}))
        # ---- 2. STRUCTURE — load cycles, and the ghost re-audit ---------- #
        cyc_n = nominal.load_cycle_ptp_N()
        cyc_d = degraded.load_cycle_ptp_N()
        with np.errstate(divide="ignore", invalid="ignore"):
            amp = np.where(cyc_n > 1.0, cyc_d / np.maximum(cyc_n, 1.0), 1.0)
        worst_i = int(np.argmax(amp))
        corner_names = ["FL", "FR", "RL", "RR"]
        roll_ratio = (degraded.roll_peak_deg()
                      / max(nominal.roll_peak_deg(), 1e-9)) \
            if math.isfinite(nominal.roll_peak_deg()) else 1.0
        if float(np.max(amp)) > th.load_cycle_amp_max \
                or roll_ratio > th.roll_amp_max:
            findings.append(ForgeFinding(
                "LOAD_CYCLE_AMPLIFIED", "amber",
                f"The electrical defect amplified the {corner_names[worst_i]} "
                f"contact-load cycle {float(amp[worst_i]):.2f}× "
                f"({cyc_n[worst_i]:.0f} → {cyc_d[worst_i]:.0f} N ptp) and the "
                f"roll peak {roll_ratio:.2f}× — a wiring defect expressed as "
                f"a structural duty increase.",
                {"amp_per_corner": [float(a) for a in amp],
                 "roll_ratio": roll_ratio}))
        if gc is not None and nominal.ok and degraded.ok:
            lint.ghost_nominal = gt.ghost_audit_transient(
                gc, nominal.mech, corner=corner)
            lint.ghost_degraded = gt.ghost_audit_transient(
                gc, degraded.mech, corner=corner)
            sn = lint.ghost_nominal.summary()
            sd = lint.ghost_degraded.summary()
            fos_drop = float(sn["worst_fos"] - sd["worst_fos"]) \
                if (math.isfinite(sn["worst_fos"])
                    and math.isfinite(sd["worst_fos"])) else 0.0
            crossed = (sd["worst_fos"] < th.fos_limit <= sn["worst_fos"]) \
                if (math.isfinite(sn["worst_fos"])
                    and math.isfinite(sd["worst_fos"])) else False
            if crossed or fos_drop > th.fos_drop_max \
                    or (lint.ghost_degraded.verdict
                        != lint.ghost_nominal.verdict):
                flags.append(ForgeVerdict.STRUCTURAL_REGRESSION.value)
                findings.append(ForgeFinding(
                    "STRUCTURAL_REGRESSION", "red",
                    f"Ghost re-audit under the degraded loads: worst "
                    f"transient FoS {sn['worst_fos']:.2f} → "
                    f"{sd['worst_fos']:.2f} ({sd['worst_fos_member']}), "
                    f"verdict {lint.ghost_nominal.verdict} → "
                    f"{lint.ghost_degraded.verdict}"
                    + (f" — the {th.fos_limit:.1f} rule is now breached "
                       f"by an ELECTRICAL defect." if crossed else "."),
                    {"fos_nominal": sn["worst_fos"],
                     "fos_degraded": sd["worst_fos"],
                     "member": sd["worst_fos_member"]}))
            else:
                findings.append(ForgeFinding(
                    "GHOST_HELD", "info",
                    f"Ghost re-audit under the degraded loads holds: worst "
                    f"FoS {sn['worst_fos']:.2f} → {sd['worst_fos']:.2f}, "
                    f"verdict unchanged ({lint.ghost_degraded.verdict}).",
                    {}))
        elif gc is None:
            findings.append(ForgeFinding(
                "GHOST_ABSENT", "info",
                "No GhostCorner supplied — the structural re-audit was NOT "
                "run. Load-cycle amplification above is the only structural "
                "signal in this lint.", {}))
        # ---- 3. PECKING ORDER — the cross-disciplinary Fusebox race ------ #
        if lint.ghost_nominal is not None and lint.ghost_degraded is not None:
            def _path(audit, res):
                els = _mech_elements_from_ghost(audit, base_path) \
                    + electrical_path_elements(res)
                return fb.OverloadPath(
                    key="forge_event", label="Co-solved event chain",
                    story="Mechanical members (worst transient FoS from the "
                          "ghost audit) racing the actuator branch's "
                          "electrical chain for first failure.",
                    elements=els,
                    designated_fuse_key=designated_fuse_key)
            pn, pd = _path(lint.ghost_nominal, nominal), \
                _path(lint.ghost_degraded, degraded)
            lint.path_audit_nominal = fb.audit_path(pn)
            lint.path_audit_degraded = fb.audit_path(pd)
            an, ad = lint.path_audit_nominal, lint.path_audit_degraded
            first_n = getattr(an, "leader_key", "") or ""
            first_d = getattr(ad, "leader_key", "") or ""
            if (first_n != first_d) or (ad.verdict != an.verdict):
                flags.append(ForgeVerdict.ORDER_INVERTED.value)
                findings.append(ForgeFinding(
                    "ORDER_INVERTED", "red",
                    f"The first-failure pecking order MOVED under the "
                    f"defect: most-likely-first '{first_n}' → '{first_d}', "
                    f"path verdict {an.verdict.value} → {ad.verdict.value}. "
                    f"An electrical defect re-chose the car's victim.",
                    {"first_nominal": first_n, "first_degraded": first_d}))
            else:
                findings.append(ForgeFinding(
                    "ORDER_HELD", "info",
                    f"Pecking order held under the defect: '{first_d}' "
                    f"remains most-likely-first, verdict "
                    f"{ad.verdict.value}.", {}))
        # ---- 4. SESSION — the Earshot energy bill ------------------------ #
        e_n = en["energy_Wh"]; e_d = ed["energy_Wh"]
        dur = float(degraded.elec.t[-1]) if degraded.elec.t.size else 0.0
        # events-per-lap scaling: treat the manoeuvre as representative of the
        # transient content of a lap; the extra Wh scales by an estimate the
        # caller controls. Stated, not hidden.
        extra_kwh_lap = max(e_d - e_n, 0.0) / 1000.0 * laps_per_session_est \
            / max(laps_per_session_est, 1.0) * 8.0  # ~8 such transients/lap
        b = degraded.bus
        laps_before = es.laps_from_pack(b.pack_kwh, b.usable_frac,
                                        b.kwh_per_lap, b.reserve_kwh)
        laps_after = es.laps_from_pack(b.pack_kwh, b.usable_frac,
                                       b.kwh_per_lap + extra_kwh_lap,
                                       b.reserve_kwh)
        lint.session_before = {"laps_per_config": laps_before,
                               "kwh_per_lap": b.kwh_per_lap}
        lint.session_after = {"laps_per_config": laps_after,
                              "kwh_per_lap": b.kwh_per_lap + extra_kwh_lap,
                              "event_Wh_nominal": e_n,
                              "event_Wh_degraded": e_d,
                              "event_s": dur}
        if ab_design is not None:
            d_before = es.ABDesign(ab_design.effect_predicted,
                                   ab_design.noise_sigma, ab_design.alpha,
                                   ab_design.power,
                                   laps_available_per_config=laps_before)
            d_after = es.ABDesign(ab_design.effect_predicted,
                                  ab_design.noise_sigma, ab_design.alpha,
                                  ab_design.power,
                                  laps_available_per_config=laps_after)
            lint.session_before.update(d_before.as_dict())
            lint.session_after.update(d_after.as_dict())
            if d_after.verdict != d_before.verdict \
                    and d_after.verdict != es.ABVerdict.RESOLVABLE:
                flags.append(ForgeVerdict.SESSION_STARVED.value)
                findings.append(ForgeFinding(
                    "SESSION_STARVED", "amber",
                    f"The defect's energy bill ({e_d - e_n:+.1f} Wh per "
                    f"event, ≈{extra_kwh_lap*1000:.0f} Wh/lap extra) shrinks "
                    f"the pack budget {laps_before} → {laps_after} laps per "
                    f"config — the A/B session goes "
                    f"{d_before.verdict.value} → {d_after.verdict.value} "
                    f"(miss probability now "
                    f"{d_after.miss_probability:.0%}).",
                    {"laps_before": laps_before, "laps_after": laps_after}))
            else:
                findings.append(ForgeFinding(
                    "SESSION_HELD", "info",
                    f"Session budget holds: {laps_before} → {laps_after} "
                    f"laps per config, A/B verdict "
                    f"{d_after.verdict.value}.", {}))
        elif laps_after < laps_before:
            findings.append(ForgeFinding(
                "SESSION_BILL", "info",
                f"The defect's energy bill costs {laps_before - laps_after} "
                f"lap(s) per config off the pack budget "
                f"({laps_before} → {laps_after}). Supply an ABDesign to "
                f"judge whether that starves a planned test.", {}))
        # ---- governing verdict ------------------------------------------- #
        for v in ForgeVerdict:
            if v.value in flags:
                lint.verdict = v.value
                break
        return lint
    except Exception as e:                                      # noqa: BLE001
        lint.note = (lint.note + f" Lint aborted partway "
                     f"({type(e).__name__}: {e}); findings above are "
                     f"complete up to the abort.").strip()
        lint.verdict = lint.verdict or ForgeVerdict.COUPLED_FAITHFUL.value
        return lint


# --------------------------------------------------------------------------- #
#  Markdown report
# --------------------------------------------------------------------------- #
def render_forge_md(lint: ForgeLint) -> str:
    L: list[str] = []
    d = lint.degraded.degradation
    L.append("# SimulForge lint — unified mechatronic co-solve")
    L.append("")
    L.append(f"**Verdict: `{lint.verdict}`**"
             + (f"  (flags: {', '.join(lint.flags)})" if lint.flags else ""))
    L.append("")
    L.append(f"Defect under test: **{d.label}** — {d.story}")
    if lint.note:
        L.append("")
        L.append(f"_{lint.note}_")
    en, ed = lint.nominal.elec.summary(), lint.degraded.elec.summary()
    L.append("")
    L.append("## The electrical facts")
    L.append("")
    L.append(f"| | pristine | degraded |")
    L.append(f"|---|---|---|")
    L.append(f"| bus minimum (V) | {en['v_min']:.1f} | {ed['v_min']:.1f} |")
    L.append(f"| peak draw (A) | {en['i_peak']:.1f} | {ed['i_peak']:.1f} |")
    L.append(f"| roll authority | {en['authority']:.0%} | "
             f"{ed['authority']:.0%} |")
    L.append(f"| response lag (ms) | {en['response_lag_ms']:.1f} | "
             f"{ed['response_lag_ms']:.1f} |")
    L.append(f"| brownouts | {en['n_brownouts']} | {ed['n_brownouts']} |")
    L.append(f"| event energy (Wh) | {en['energy_Wh']:.1f} | "
             f"{ed['energy_Wh']:.1f} |")
    L.append("")
    L.append("## Findings")
    L.append("")
    icon = {"red": "🔴", "amber": "🟠", "info": "ℹ️"}
    for f in lint.findings:
        L.append(f"* {icon.get(f.severity, '•')} **{f.code}** — {f.text}")
    if lint.ghost_degraded is not None:
        s = lint.ghost_degraded.summary()
        L.append("")
        L.append(f"Structural judge: ghost audit under degraded loads gives "
                 f"worst transient FoS **{s['worst_fos']:.2f}** "
                 f"({s['worst_fos_member']}), verdict "
                 f"`{lint.ghost_degraded.verdict}`.")
    if lint.session_after is not None:
        L.append("")
        L.append(f"Session ledger: {lint.session_before['laps_per_config']} → "
                 f"{lint.session_after['laps_per_config']} laps per config "
                 f"after the defect's energy bill.")
    L.append("")
    L.append("_Coupling: staggered co-step at the mechanical dt, zero-order-"
             "held moments, exact RL winding update. Defaults are FSAE-"
             "representative stand-ins — set every knob from your "
             "datasheets._")
    return "\n".join(L)


# --------------------------------------------------------------------------- #
#  Self-test
# --------------------------------------------------------------------------- #
def _self_test() -> None:
    presets = degradation_presets()
    nom = run_simulforge(None, kind="step_steer", degradation="nominal")
    assert nom.ok, nom.warnings
    assert nom.elec.t.size == nom.mech.t.size
    assert float(nom.elec.E_Wh[-1]) > 0.0
    # the active bar must actually reduce roll vs its own dead-actuator case
    dead = run_simulforge(None, kind="step_steer",
                          degradation=presets["dead_actuator"])
    assert dead.ok
    assert nom.roll_peak_deg() < dead.roll_peak_deg() * 0.995, \
        (nom.roll_peak_deg(), dead.roll_peak_deg())
    # a corroded connector must sag harder than pristine
    cor = run_simulforge(None, kind="step_steer",
                         degradation=presets["corroded_connector"])
    assert cor.elec.summary()["v_min"] <= nom.elec.summary()["v_min"] + 1e-9
    lint = forge_lint(nom, cor)
    assert lint.verdict in {v.value for v in ForgeVerdict}
    md = render_forge_md(lint)
    assert "SimulForge lint" in md
    # never-raise contract
    bad = run_simulforge(None, kind="no_such_maneuver")
    assert not bad.ok
    empty_lint = forge_lint(bad, bad)
    assert isinstance(empty_lint.verdict, str)
    print("simulforge self-test ok:",
          f"nominal roll {nom.roll_peak_deg():.2f}° vs dead "
          f"{dead.roll_peak_deg():.2f}°;",
          f"corroded v_min {cor.elec.summary()['v_min']:.1f} V;",
          f"lint verdict {lint.verdict}")


if __name__ == "__main__":
    _self_test()
