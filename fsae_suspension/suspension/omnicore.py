# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  suspension/omnicore.py — 🪐 OmniCore, the vehicle-synthesis referee
# ============================================================================
"""
OmniCore — the system-level referee over the three synthesis engines.

Subsystems are always at war. The geometry InverseGenesis wants may demand an
actuator SimulForge shows browning the bus out mid-event; the control gains
that keep the car composed burn endurance energy; the bracket MorphMesh grows
for those loads may be one the mission's own shop class vetoes. Until now
every one of those arguments was settled in a different tab, by a different
person, on a different day — and the trade was never a number.

OmniCore closes the loop, in three honest moves:

  1. MISSION GRAMMAR — a typed mission profile ("lightweight 4WD electric
     off-road car, $15,000 budget, ±2 mm weld-pull shop") is parsed by a
     deterministic keyword grammar — NOT a language model. Every token the
     grammar consumed, every default it assumed, and every word it ignored is
     printed as a receipt. What the parser didn't understand, it says so,
     instead of hallucinating an interpretation.

  2. THE REFEREE — one shared-subsolve sweep across a declared configuration
     lattice (shop class × actuator size × structural volume budget). Each
     configuration is scored on five axes in real currencies:
       · event composure  ∫|roll| dt over the mission manoeuvre (deg·s) —
         a lap-time PROXY, named as such (mapping deg·s to seconds is the
         lap-sim's job, not this tab's);
       · endurance range  laps supported by the pack with the actuator's
         measured event energy folded in (Earshot's currency);
       · structural mass  the grown bracket × the declared tab count (kg);
       · decision cost    USD of exactly what this sweep varies — actuators,
         brackets, shop capex — on top of a declared base-vehicle cost;
       · build yield      InverseGenesis' fraction-of-built-cars-in-band.
     A configuration any engine vetoes is INFEASIBLE with the vetoing engine
     NAMED; the mission's own budget vetoes the same way. The survivors form
     a screening Pareto front; every dominated configuration carries a
     receipt naming the configuration that beats it on every axis at once.

  3. THE SELF-HEALING TWIN — feed measured telemetry summaries back in
     (bus sag, response lag, event energy, roll peak, camber drift). Each
     channel is judged against the nominal model's declared bands; the
     deviation PATTERN is cosine-matched against the signature every named
     Degradation preset predicts (the Saboteur's trick, pointed at the
     running car) — so the verdict is not "something drifted" but "this
     matches a corroded branch connector, magnitude 0.8× predicted". The
     heal plan is then arithmetic, not vibes: a gain de-rate computed from
     what the sagged bus can actually deliver, and a 3-D-printable shim
     thickness from the measured camber drift and the declared bolt span.

What this is NOT: it is a SCREENING referee at declared coarse fidelity —
n-starts, yield samples, mesh cells and iteration caps are all printed on the
result — and the front it draws is the shortlist for the engines' own tabs
(and eventually ANSYS/ADAMS), never a substitute for them. The twin's
diagnosis is a named suspect for the audit to start with, not a certified
root cause; a pattern matching nothing in the catalog says so honestly.

Pure Python + NumPy, headless, flagged-not-raised. Self-test:
    python3 -m suspension.omnicore
UI in ui/omnicore.py.
"""

from __future__ import annotations

import json
import math
import re
import time
from dataclasses import dataclass, field as _dcfield, asdict
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np

from .kinematics import Hardpoints
from . import inverse_genesis as ig
from . import simulforge as sf
from . import ghost_topology as gt
from . import morphmesh as mmx
from . import transient as tr
from .compliance import CompliantCorner
from .kinematik_stochastic import ToleranceField


# --------------------------------------------------------------------------- #
#  1 · The mission grammar — a receipt, not a language model
# --------------------------------------------------------------------------- #
_MANEUVER_WORDS = {
    "curb_strike": ("bumpy", "bump", "curb", "kerb", "off-road", "offroad",
                    "rough", "high-frequency", "washboard", "gravel"),
    "snap_oversteer": ("snap", "oversteer", "drift", "aggressive",
                       "autocross"),
    "brake_to_throttle": ("braking", "trail-brake", "chicane", "transition"),
    "step_steer": ("smooth", "track", "circuit", "slalom", "step"),
}

_PRIORITY_WORDS = {
    "mass": ("lightweight", "light", "mass", "weight"),
    "range": ("range", "endurance", "efficient", "efficiency", "battery"),
    "cost": ("cheap", "budget-driven", "affordable", "low-cost"),
    "composure": ("fast", "handling", "composed", "responsive", "lap"),
    "yield": ("resilient", "buildable", "yield"),
}

_DRIVE_WORDS = {"4wd": ("4wd", "awd", "four-wheel"),
                "2wd": ("2wd", "rwd", "fwd", "two-wheel")}

# tokens the grammar recognises but that change nothing this sweep varies —
# consumed so they don't land in "ignored", and disclosed as such.
_ACK_WORDS = ("electric", "ev", "vehicle", "car", "optimized", "optimised",
              "synthesize", "synthesise", "terrain", "shop", "floor",
              "manufacturing")

_BUDGET_RE = re.compile(r"[\$€£]?\s*(\d{1,3}(?:[,.]\d{3})+|\d+(?:\.\d+)?)\s*"
                        r"(k\b|thousand\b)?", re.IGNORECASE)
_TOL_RE = re.compile(r"(?:±|\+/?-|\\pm)?\s*(\d+(?:\.\d+)?)\s*mm", re.IGNORECASE)


@dataclass
class MissionSpec:
    """The typed mission profile + the parse receipt."""
    text: str
    maneuver: str = "step_steer"
    drive: str = "unspecified"
    budget_usd: Optional[float] = None
    weld_pull_mm: float = 0.0
    shop_accuracy_mm: Optional[float] = None
    shop: str = "hand_weld"                 # accuracy → class, worst-fit
    priorities: Dict[str, float] = _dcfield(default_factory=dict)
    consumed: List[str] = _dcfield(default_factory=list)   # token → meaning
    assumptions: List[str] = _dcfield(default_factory=list)
    ignored: List[str] = _dcfield(default_factory=list)

    def summary(self) -> dict:
        return {k: (round(v, 3) if isinstance(v, float) else v)
                for k, v in asdict(self).items() if k != "text"}


def _shop_from_accuracy(u_mm: float) -> str:
    """The worst (cheapest) shop class whose declared positional accuracy
    still covers the stated error — the honest reading of '±2 mm shop'."""
    if u_mm <= 0.05:
        return "cnc"
    if u_mm <= 0.5:
        return "jig_weld"
    return "hand_weld"


def parse_mission(text: str) -> MissionSpec:
    """Deterministic grammar over the mission prompt. Same text in, same
    spec out, forever — and everything it did is on the receipt."""
    spec = MissionSpec(text=text or "")
    raw = (text or "").strip()
    low = raw.lower()

    # --- budget: the largest money-looking number ------------------------- #
    best = None
    for m in _BUDGET_RE.finditer(raw):
        tokn = m.group(0).strip()
        has_cur = tokn[:1] in "$€£"
        v = float(m.group(1).replace(",", "").replace(".", "", 
                  m.group(1).count(".") if "," in m.group(1) else 0))
        try:
            v = float(m.group(1).replace(",", ""))
        except ValueError:
            continue
        if m.group(2):
            v *= 1000.0
        if not has_cur and not m.group(2) and v < 1000:
            continue                      # "±2 mm" and friends are not money
        if best is None or v > best[0]:
            best = (v, tokn)
    if best:
        spec.budget_usd = best[0]
        spec.consumed.append(f"'{best[1]}' → budget ${best[0]:,.0f}")

    # --- shop accuracy: '±2 mm' -------------------------------------------- #
    tol = _TOL_RE.search(raw)
    if tol:
        u = float(tol.group(1))
        spec.shop_accuracy_mm = u
        spec.shop = _shop_from_accuracy(u)
        spec.consumed.append(f"'{tol.group(0).strip()}' → shop accuracy "
                             f"±{u:g} mm → class '{spec.shop}'")
        if "pull" in low or "weld" in low:
            spec.weld_pull_mm = u
            spec.consumed.append(f"'weld…pull' near the tolerance → "
                                 f"systematic weld pull {u:g} mm on the tabs")
    else:
        spec.assumptions.append(f"no shop tolerance stated — assuming class "
                                f"'{spec.shop}' (the club-garage default)")

    # --- word classes ------------------------------------------------------ #
    words = re.findall(r"[a-zA-Z0-9][a-zA-Z0-9\-]+", low)
    used: set = set()
    for kind, keys in _MANEUVER_WORDS.items():
        hit = [w for w in words if any(w.startswith(k) for k in keys)]
        if hit and spec.maneuver == "step_steer" and kind != "step_steer":
            spec.maneuver = kind
            spec.consumed.append(f"'{hit[0]}' → mission manoeuvre "
                                 f"'{kind}'")
        used.update(hit if hit else [])
    for drv, keys in _DRIVE_WORDS.items():
        hit = [w for w in words if w in keys]
        if hit:
            spec.drive = drv
            spec.consumed.append(
                f"'{hit[0]}' → drive '{drv}' (disclosed, not varied: this "
                "sweep's transient model carries one driveline)")
            used.update(hit)
    for pri, keys in _PRIORITY_WORDS.items():
        hit = [w for w in words if any(w.startswith(k) for k in keys)]
        if hit:
            spec.priorities[pri] = spec.priorities.get(pri, 1.0) + 1.0
            spec.consumed.append(f"'{hit[0]}' → priority weight on "
                                 f"'{pri}'")
            used.update(hit)
    ack = [w for w in words if any(w.startswith(a) for a in _ACK_WORDS)]
    used.update(ack)

    if spec.maneuver == "step_steer" and not any(
            "manoeuvre" in c for c in spec.consumed):
        spec.assumptions.append("no terrain words recognised — assuming the "
                                "step-steer manoeuvre")
    if not spec.priorities:
        spec.assumptions.append("no priority words recognised — the knee "
                                "pick will weight every axis equally")
    if spec.budget_usd is None:
        spec.assumptions.append("no budget recognised — the budget referee "
                                "stands down (no configuration is "
                                "budget-vetoed)")

    stop = {"a", "an", "the", "for", "and", "with", "by", "to", "of", "on",
            "in", "error", "mm", "constrained", "profile", "mission",
            "budget", "weld", "pull", "weld-pull"}
    spec.ignored = sorted({w for w in words
                           if w not in used and w not in stop
                           and len(w) > 2
                           and any(ch.isalpha() for ch in w)})[:20]
    return spec


# --------------------------------------------------------------------------- #
#  2 · The knobs — every constant a declared, editable number
# --------------------------------------------------------------------------- #
@dataclass
class OmniKnobs:
    """Everything the referee's arithmetic stands on. All of it printable,
    none of it hidden. Representative 2026 numbers, not quotes."""
    # lattice
    actuator_scales: Dict[str, float] = _dcfield(
        default_factory=lambda: {"compact": 0.6, "standard": 1.0,
                                 "authority": 1.5})
    volfracs: Tuple[float, ...] = (0.32, 0.45)
    compare_shops: bool = True            # mission shop + the next-better class
    # screening fidelity — printed on the result
    genesis_starts: int = 2
    genesis_yield_n: int = 400
    genesis_max_iter: int = 8
    morph_h_mm: float = 2.5
    morph_max_iter: int = 10
    morph_betas: Tuple[float, ...] = (1.0, 4.0)
    morph_rounds: int = 2
    audit_samples: int = 10
    fan_cases: int = 3
    member: str = "LF"
    corner: str = "FR"
    # geometry intent (terrain-shaped targets)
    band_camber_deg: float = 0.25
    band_toe_deg: float = 0.12
    camber_gain_deg_per_30mm: float = 0.6
    volume_box_mm: float = 10.0
    # mass model
    n_tabs: int = 8                        # grown tabs fitted per car
    act_mass_base_kg: float = 0.9
    act_mass_per_Nm: float = 0.0018        # ~2.5 kg at 900 N·m roll authority
    # cost model (USD)
    act_cost_base: float = 180.0
    act_cost_per_Nm: float = 0.55
    tab_cost: Dict[str, float] = _dcfield(
        default_factory=lambda: {"hand_weld": 18.0, "jig_weld": 26.0,
                                 "cnc": 45.0})
    shop_capex: Dict[str, float] = _dcfield(
        default_factory=lambda: {"hand_weld": 0.0, "jig_weld": 220.0,
                                 "cnc": 600.0})
    material_usd_per_kg: float = 9.0
    base_vehicle_usd: float = 11_000.0     # everything this sweep does NOT vary
    # range model
    events_per_lap: float = 14.0

    def fidelity_note(self) -> str:
        return (f"screening fidelity: genesis {self.genesis_starts} starts / "
                f"{self.genesis_yield_n} yield samples / "
                f"{self.genesis_max_iter} iter; morph {self.morph_h_mm:g} mm "
                f"cells / {self.morph_max_iter} iter × {len(self.morph_betas)}"
                f" β / {self.morph_rounds} rounds; audit "
                f"{self.audit_samples} instants, {self.fan_cases} fan cases")


_SHOP_LADDER = ["hand_weld", "jig_weld", "cnc"]


def _shops_for(mission: MissionSpec, knobs: OmniKnobs) -> List[str]:
    """The mission's implied class, plus the next-better one — so the front
    can price what a jig (or a machinist) is actually worth."""
    i = _SHOP_LADDER.index(mission.shop)
    out = [mission.shop]
    if knobs.compare_shops and i + 1 < len(_SHOP_LADDER):
        out.append(_SHOP_LADDER[i + 1])
    return out


# --------------------------------------------------------------------------- #
#  3 · Objectives, configurations, and the front
# --------------------------------------------------------------------------- #
#  axis key → (label, unit, sense)  sense +1 = maximise, −1 = minimise
#  axis key → (label, unit, sense, display format)
AXES: Dict[str, Tuple[str, str, int, str]] = {
    "composure": ("event composure ∫|roll|dt", "deg·s", -1, "{:.3f}"),
    "laps":      ("endurance range", "laps", +1, "{:.2f}"),
    "mass":      ("grown structure mass", "kg", -1, "{:.2f}"),
    "cost":      ("decision-scope cost", "USD", -1, "{:,.0f}"),
    "yield":     ("build yield", "fraction", +1, "{:.3f}"),
}


def fmt_axis(key: str, v: float) -> str:
    return AXES[key][3].format(v) if np.isfinite(v) else "—"


@dataclass
class ConfigPoint:
    """One whole-vehicle configuration and its scorecard."""
    cid: int
    shop: str
    actuator: str
    volfrac: float
    objectives: Dict[str, float] = _dcfield(default_factory=dict)
    feasible: bool = True
    vetoes: List[str] = _dcfield(default_factory=list)      # engine-named
    flags: List[str] = _dcfield(default_factory=list)
    detail: Dict[str, object] = _dcfield(default_factory=dict)

    @property
    def label(self) -> str:
        return f"#{self.cid} {self.shop}·{self.actuator}·vf{self.volfrac:g}"


def pareto_mask(points: List[Dict[str, float]],
                axes: Dict[str, Tuple[str, str, int, str]] = AXES
                ) -> List[bool]:
    """True where no other point is at-least-as-good on every axis and
    strictly better on one. Pure arithmetic, checkable by hand."""
    keys = list(axes)
    sgn = np.array([axes[k][2] for k in keys], float)
    M = np.array([[p[k] for k in keys] for p in points], float) * sgn
    n = len(points)
    keep = [True] * n
    for i in range(n):
        for j in range(n):
            if i == j:
                continue
            if np.all(M[j] >= M[i] - 1e-12) and np.any(M[j] > M[i] + 1e-12):
                keep[i] = False
                break
    return keep


def knee_pick(points: List[ConfigPoint], mission: MissionSpec
              ) -> Optional[int]:
    """The referee's pick: min priority-weighted normalised distance to the
    utopia point over the FEASIBLE set. One reading of the front — the front
    itself is the answer."""
    feas = [p for p in points if p.feasible]
    if not feas:
        return None
    keys = list(AXES)
    w = np.array([1.0 + mission.priorities.get(
        {"composure": "composure", "laps": "range", "mass": "mass",
         "cost": "cost", "yield": "yield"}[k], 0.0) for k in keys])
    sgn = np.array([AXES[k][2] for k in keys], float)
    M = np.array([[p.objectives[k] for k in keys] for p in feas]) * sgn
    lo, hi = M.min(axis=0), M.max(axis=0)
    span = np.where(hi - lo > 1e-12, hi - lo, 1.0)
    N = (hi - M) / span                      # 0 = best on that axis
    score = (N * w).sum(axis=1)
    return feas[int(np.argmin(score))].cid


def dominance_receipts(points: List[ConfigPoint]) -> List[str]:
    """For every dominated feasible configuration, name one that strictly
    beats it — the receipt that settles the meeting."""
    feas = [p for p in points if p.feasible]
    keys = list(AXES)
    sgn = np.array([AXES[k][2] for k in keys], float)
    M = {p.cid: np.array([p.objectives[k] for k in keys]) * sgn for p in feas}
    out = []
    for p in feas:
        for q in feas:
            if q.cid == p.cid:
                continue
            if np.all(M[q.cid] >= M[p.cid] - 1e-12) and \
                    np.any(M[q.cid] > M[p.cid] + 1e-12):
                out.append(f"{p.label} is beaten on every axis at once by "
                           f"{q.label}.")
                break
    return out


# --------------------------------------------------------------------------- #
#  4 · The shared-subsolve orchestration
# --------------------------------------------------------------------------- #
@dataclass
class OmniResult:
    mission: MissionSpec
    knobs: OmniKnobs
    configs: List[ConfigPoint]
    pareto_ids: List[int]
    knee_id: Optional[int]
    receipts: List[str]
    ledger: Dict[str, int]                  # engine call counts
    elapsed_s: float
    warnings: List[str] = _dcfield(default_factory=list)

    @property
    def ok(self) -> bool:
        return any(c.feasible for c in self.configs)

    def summary(self) -> dict:
        return {
            "mission": self.mission.summary(),
            "fidelity": self.knobs.fidelity_note(),
            "configs": [{
                "label": c.label, "feasible": c.feasible,
                "objectives": {k: round(v, 4)
                               for k, v in c.objectives.items()},
                "vetoes": c.vetoes, "flags": c.flags}
                for c in self.configs],
            "pareto": [c.label for c in self.configs
                       if c.cid in self.pareto_ids],
            "knee": next((c.label for c in self.configs
                          if c.cid == self.knee_id), None),
            "receipts": self.receipts,
            "ledger": self.ledger,
            "elapsed_s": round(self.elapsed_s, 2),
            "warnings": self.warnings,
        }

    def to_json(self) -> str:
        return json.dumps(self.summary(), indent=2)


def _terrain_targets(hp: Hardpoints, mission: MissionSpec, knobs: OmniKnobs
                     ) -> ig.GenesisTargets:
    """Terrain-shaped intent: linear camber gain toward the terrain's number,
    dead bump steer — expressed relative to the nominal so the ask stays
    inside the solver's reach at screening fidelity."""
    st = np.array([-30.0, 0.0, 30.0])
    vals, ok = ig.curves_of(hp, st)
    if not ok:
        raise ValueError("nominal geometry does not solve at ±30 mm")
    g = knobs.camber_gain_deg_per_30mm
    if mission.maneuver == "curb_strike":
        g *= 0.6           # bumpy terrain: gentler gain, keep the patch flat
    cam = vals["camber_deg"]
    cam0 = float(cam[1])
    s_nom = float(cam[2] - cam[0]) / 2.0          # nominal gain per 30 mm
    s_tgt = 0.5 * (s_nom + math.copysign(g, s_nom if s_nom else -1.0))
    cam_t = cam0 + s_tgt * np.array([-1.0, 0.0, +1.0])
    toe_t = 0.5 * vals["toe_deg"]                 # halve the bump steer
    return ig.GenesisTargets([
        ig.TargetCurve("camber_deg", st, cam_t,
                       np.full(3, knobs.band_camber_deg)),
        ig.TargetCurve("toe_deg", st, toe_t,
                       np.full(3, knobs.band_toe_deg)),
    ])


def _actuator_for(scale: float) -> sf.ActuatorParams:
    a = sf.ActuatorParams()
    a.M_max_Nm *= scale
    a.kp *= scale
    a.kd *= scale
    a.fuse_rating_A *= max(scale, 0.4)
    a.conn_rating_A *= max(scale, 0.4)
    return a


def _composure_deg_s(res: sf.SimulForgeResult) -> float:
    if not res.ok or res.mech.roll.size < 2:
        return float("nan")
    t = res.mech.t
    return float(np.trapezoid(np.abs(res.mech.roll), t) * 180.0 / math.pi)


def _laps_supported(res: sf.SimulForgeResult, knobs: OmniKnobs) -> float:
    b = res.bus
    e_lap = b.kwh_per_lap + res.elec.summary()["energy_Wh"] \
        * knobs.events_per_lap / 1000.0
    avail = max(b.pack_kwh * b.usable_frac - b.reserve_kwh, 0.0)
    return avail / max(e_lap, 1e-9)


def _decision_cost(shop: str, act_scale: float, tab_mass_g: float,
                   knobs: OmniKnobs) -> float:
    act = 2.0 * (knobs.act_cost_base
                 + knobs.act_cost_per_Nm * 900.0 * act_scale)
    tabs = knobs.n_tabs * (knobs.tab_cost[shop]
                           + knobs.material_usd_per_kg * tab_mass_g / 1000.0)
    return act + tabs + knobs.shop_capex[shop]


def _structure_mass_kg(tab_mass_g: float, act_scale: float,
                       knobs: OmniKnobs) -> float:
    act = 2.0 * (knobs.act_mass_base_kg
                 + knobs.act_mass_per_Nm * 900.0 * act_scale)
    return knobs.n_tabs * tab_mass_g / 1000.0 + act


def _ghost_corner(hp: Hardpoints, corner: str) -> gt.GhostCorner:
    cc = CompliantCorner.uniform_tube(hp)
    params = tr.TransientParams.from_vehicle(None)
    k_wheel = (params.k_wheel_front if corner in ("FL", "FR")
               else params.k_wheel_rear) / 1000.0
    Fz = float(tr.TransientSolver().static_corner_loads()[
        {"FL": 0, "FR": 1, "RL": 2, "RR": 3}[corner]])
    track = (params.track_front if corner in ("FL", "FR")
             else params.track_rear) * 1000.0
    return gt.GhostCorner(cc, gt.uniform_sections(),
                          wheel_rate_N_per_mm=k_wheel, Fz_static_N=Fz,
                          track_mm=track)


def run_omnicore(hp: Optional[Hardpoints], mission: MissionSpec,
                 knobs: Optional[OmniKnobs] = None,
                 progress: Optional[Callable[[str], None]] = None
                 ) -> OmniResult:
    """The whole referee. Shares sub-solves across the lattice on purpose:
    one genesis per shop class, one co-solve + audit per actuator size, one
    growth per (shop × actuator × volume budget). Never raises — a broken
    engine vetoes its configurations by name instead."""
    t_start = time.time()
    hp = hp or Hardpoints.default()
    knobs = knobs or OmniKnobs()
    say = progress or (lambda s: None)
    warnings: List[str] = []
    ledger = {"genesis": 0, "simulforge": 0, "ghost_audits": 0, "morph": 0,
              "fe_solves": 0}

    shops = _shops_for(mission, knobs)

    # ---- InverseGenesis: one solve per shop class ------------------------- #
    gen: Dict[str, Optional[ig.GenesisResult]] = {}
    try:
        targets = _terrain_targets(hp, mission, knobs)
        box = knobs.volume_box_mm
        vol = ig.LegalVolume({
            "lower_outer": (hp.lower_outer - box, hp.lower_outer + box),
            "upper_outer": (hp.upper_outer - box, hp.upper_outer + box),
            "tie_rod_outer": (hp.tie_rod_outer - box,
                              hp.tie_rod_outer + box)})
    except Exception as e:                                    # noqa: BLE001
        targets, vol = None, None
        warnings.append(f"target construction failed ({e}) — every "
                        "configuration is vetoed by InverseGenesis.")
    for shop in shops:
        if targets is None:
            gen[shop] = None
            continue
        say(f"InverseGenesis — shop '{shop}'…")
        try:
            fld = ToleranceField.preset(
                shop, weld_pull_mm=(mission.weld_pull_mm
                                    if shop == mission.shop else 0.0))
            gen[shop] = ig.inverse_genesis(
                hp, targets, vol, fld=fld,
                n_starts=knobs.genesis_starts,
                n_yield=knobs.genesis_yield_n,
                max_iter=knobs.genesis_max_iter)
            ledger["genesis"] += 1
        except Exception as e:                                # noqa: BLE001
            gen[shop] = None
            warnings.append(f"InverseGenesis raised for shop '{shop}' "
                            f"({type(e).__name__}: {e}).")

    # ---- SimulForge + Ghost audit: one per actuator size ------------------ #
    forge: Dict[str, Optional[sf.SimulForgeResult]] = {}
    audits: Dict[str, Optional[gt.GhostAudit]] = {}
    for name, scale in knobs.actuator_scales.items():
        say(f"SimulForge — actuator '{name}' through "
            f"'{mission.maneuver}'…")
        try:
            r = sf.run_simulforge(None, kind=mission.maneuver,
                                  actuator=_actuator_for(scale))
            forge[name] = r
            ledger["simulforge"] += 1
        except Exception as e:                                # noqa: BLE001
            forge[name] = None
            warnings.append(f"SimulForge raised for actuator '{name}' "
                            f"({type(e).__name__}: {e}).")
            audits[name] = None
            continue
        try:
            gc = _ghost_corner(hp, knobs.corner)
            audits[name] = gt.ghost_audit_transient(
                gc, r.mech, corner=knobs.corner,
                n_samples=knobs.audit_samples) if r.ok else None
            if audits[name] is not None:
                ledger["ghost_audits"] += 1
        except Exception as e:                                # noqa: BLE001
            audits[name] = None
            warnings.append(f"Ghost audit raised for actuator '{name}' "
                            f"({type(e).__name__}: {e}).")

    # ---- MorphMesh: one growth per (shop × actuator × volfrac) ------------ #
    configs: List[ConfigPoint] = []
    cid = 0
    for shop in shops:
        g = gen.get(shop)
        limits = mmx.FabricationLimits.from_shop(shop)
        for aname in knobs.actuator_scales:
            r = forge.get(aname)
            audit = audits.get(aname)
            for vf in knobs.volfracs:
                cid += 1
                cp = ConfigPoint(cid=cid, shop=shop, actuator=aname,
                                 volfrac=float(vf))
                # -- genesis verdicts (shared per shop) -------------------- #
                if g is None or not g.ok or g.winner is None:
                    reason = (g.reason if g is not None and not g.ok
                              else "engine unavailable")
                    cp.feasible = False
                    cp.vetoes.append(f"InverseGenesis: {reason}")
                    y = float("nan")
                else:
                    y = float(g.winner.yield_frac)
                    if g.winner.verdict == "KNIFE_EDGE":
                        cp.flags.append("InverseGenesis: KNIFE_EDGE winner — "
                                        "the geometry sits on a sensitivity "
                                        "edge at this shop's scatter.")
                # -- forge verdicts (shared per actuator) ------------------ #
                if r is None or not r.ok:
                    cp.feasible = False
                    cp.vetoes.append(
                        "SimulForge: the co-solve failed"
                        + (" — " + "; ".join(r.warnings[:1]) if r else "."))
                    comp = laps = float("nan")
                else:
                    s = r.elec.summary()
                    if s["n_brownouts"] > 0:
                        cp.feasible = False
                        cp.vetoes.append(
                            f"SimulForge: {s['n_brownouts']} bus brownout(s) "
                            "mid-event — the controller resets while the car "
                            "is still in the manoeuvre.")
                    comp = _composure_deg_s(r)
                    laps = _laps_supported(r, knobs)
                    if s["authority"] < 0.7:
                        cp.flags.append(
                            f"SimulForge: delivered authority "
                            f"{s['authority']:.2f} of command — the bus, not "
                            "the gains, is the limit.")
                # -- morph (per full triple) ------------------------------- #
                m = None
                if audit is not None:
                    try:
                        dom = mmx.PlateDomain.chassis_tab(
                            h_mm=knobs.morph_h_mm)
                        m = mmx.morph_from_audit(
                            audit, hp, member=knobs.member, dom=dom,
                            limits=limits, n_cases=knobs.fan_cases,
                            volfrac=float(vf),
                            max_iter=knobs.morph_max_iter,
                            betas=knobs.morph_betas,
                            max_rounds=knobs.morph_rounds)
                        ledger["morph"] += 1
                        ledger["fe_solves"] += m.n_solves
                    except Exception as e:                    # noqa: BLE001
                        warnings.append(f"MorphMesh raised for {cp.label} "
                                        f"({type(e).__name__}: {e}).")
                if m is None or not m.ok:
                    cp.feasible = False
                    v = m.verdict if m is not None else "no transient audit"
                    cp.vetoes.append(f"MorphMesh: {v} — no buildable shape "
                                     "inside this volume budget at this "
                                     "shop's limits.")
                    tab_g = float("nan")
                else:
                    tab_g = float(m.mass_g)
                    if m.verdict == "COARSENED" and m.coarsen_premium:
                        cp.flags.append(
                            "MorphMesh: shop veto forced coarsening — "
                            f"{m.coarsen_premium['d_compliance_frac']*100:.0f}"
                            "% compliance premium paid for buildability.")
                    if np.isfinite(m.fos) and m.fos < 1.5:
                        cp.flags.append(
                            f"MorphMesh: screen FoS {m.fos:.2f} < 1.5 at "
                            "peak fan loads"
                            + (f" — {m.suggested_thickness_mm:.1f} mm plate "
                               "makes the rule."
                               if m.suggested_thickness_mm else "."))
                # -- assemble the scorecard -------------------------------- #
                mass = (_structure_mass_kg(tab_g,
                                           knobs.actuator_scales[aname],
                                           knobs)
                        if np.isfinite(tab_g) else float("nan"))
                cost = (_decision_cost(shop, knobs.actuator_scales[aname],
                                       tab_g, knobs)
                        if np.isfinite(tab_g) else float("nan"))
                cp.objectives = {"composure": comp, "laps": laps,
                                 "mass": mass, "cost": cost, "yield": y}
                # -- the mission's own budget referee ---------------------- #
                if (mission.budget_usd is not None and np.isfinite(cost)
                        and knobs.base_vehicle_usd + cost
                        > mission.budget_usd):
                    over = knobs.base_vehicle_usd + cost - mission.budget_usd
                    cp.feasible = False
                    cp.vetoes.append(
                        f"Mission budget: ${knobs.base_vehicle_usd:,.0f} "
                        f"base + ${cost:,.0f} decision scope overshoots "
                        f"${mission.budget_usd:,.0f} by ${over:,.0f}.")
                if any(not np.isfinite(v) for v in cp.objectives.values()) \
                        and cp.feasible:
                    cp.feasible = False
                    cp.vetoes.append("scorecard incomplete — an engine "
                                     "returned no number for an axis.")
                cp.detail = {"genesis_verdict":
                             (g.winner.verdict if g and g.ok and g.winner
                              else None),
                             "morph_verdict": (m.verdict if m else None),
                             "forge": (r.elec.summary() if r and r.ok
                                       else None)}
                configs.append(cp)

    # ---- the front and the receipts --------------------------------------- #
    feas = [c for c in configs if c.feasible]
    if feas:
        mask = pareto_mask([c.objectives for c in feas])
        pareto_ids = [c.cid for c, k in zip(feas, mask) if k]
    else:
        pareto_ids = []
        warnings.append("no feasible configuration survived the referees — "
                        "the vetoes below name which engine (or the budget) "
                        "killed each one.")
    knee = knee_pick(configs, mission)
    receipts = dominance_receipts(configs)

    return OmniResult(mission=mission, knobs=knobs, configs=configs,
                      pareto_ids=pareto_ids, knee_id=knee,
                      receipts=receipts, ledger=ledger,
                      elapsed_s=time.time() - t_start, warnings=warnings)


# --------------------------------------------------------------------------- #
#  5 · The self-healing twin — drift, a named suspect, and arithmetic to heal
# --------------------------------------------------------------------------- #
#  channel key → (label, unit, default band)  band = "still nominal" half-width
TWIN_CHANNELS: Dict[str, Tuple[str, str, float]] = {
    "v_min":           ("bus minimum voltage", "V", 0.5),
    "sag_peak_V":      ("peak bus sag", "V", 0.5),
    "i_peak":          ("peak bus draw", "A", 4.0),
    "response_lag_ms": ("actuator response lag", "ms", 4.0),
    "energy_Wh":       ("event energy", "Wh", 0.03),
    "authority":       ("delivered/commanded authority", "–", 0.05),
    "roll_peak_deg":   ("peak body roll", "deg", 0.25),
}


def _channel_vec(res: sf.SimulForgeResult) -> Dict[str, float]:
    s = res.elec.summary()
    out = {k: float(s[k]) for k in TWIN_CHANNELS if k in s}
    out["roll_peak_deg"] = res.roll_peak_deg()
    return out


@dataclass
class TwinBaseline:
    maneuver: str
    channels: Dict[str, float]
    bands: Dict[str, float]

    def summary(self) -> dict:
        return {"maneuver": self.maneuver,
                "channels": {k: round(v, 4)
                             for k, v in self.channels.items()},
                "bands": self.bands}


@dataclass
class DefectSignature:
    key: str
    label: str
    story: str
    dz: Dict[str, float]        # band-normalised predicted deviation


@dataclass
class TwinDiagnosis:
    baseline: TwinBaseline
    measured: Dict[str, float]
    z: Dict[str, float]                       # band-normalised deviation
    channel_verdicts: Dict[str, str]          # NOMINAL / WATCH / DEGRADED
    suspect: Optional[str]
    suspect_label: Optional[str]
    cosine: float
    magnitude: float                          # ×(predicted signature)
    note: str

    @property
    def drifting(self) -> bool:
        return any(v != "NOMINAL" for v in self.channel_verdicts.values())

    def summary(self) -> dict:
        return {"z": {k: round(v, 2) for k, v in self.z.items()},
                "verdicts": self.channel_verdicts,
                "suspect": self.suspect, "suspect_label": self.suspect_label,
                "cosine": round(self.cosine, 3),
                "magnitude": round(self.magnitude, 2),
                "note": self.note}


def twin_baseline(maneuver: str,
                  actuator: Optional[sf.ActuatorParams] = None,
                  bus: Optional[sf.BusParams] = None,
                  bands: Optional[Dict[str, float]] = None
                  ) -> Tuple[TwinBaseline, sf.SimulForgeResult]:
    """One nominal co-solve → the declared 'still healthy' envelope."""
    r = sf.run_simulforge(None, kind=maneuver, actuator=actuator, bus=bus)
    b = bands or {k: TWIN_CHANNELS[k][2] for k in TWIN_CHANNELS}
    return TwinBaseline(maneuver=maneuver, channels=_channel_vec(r),
                        bands=b), r


def defect_signatures(baseline: TwinBaseline,
                      actuator: Optional[sf.ActuatorParams] = None,
                      bus: Optional[sf.BusParams] = None,
                      presets: Optional[Dict[str, sf.Degradation]] = None,
                      progress: Optional[Callable[[str], None]] = None
                      ) -> List[DefectSignature]:
    """Run the SAME manoeuvre once under every named Degradation preset and
    record the band-normalised deviation pattern each one predicts. The
    Saboteur's signature idea, pointed at the running car."""
    say = progress or (lambda s: None)
    presets = presets or sf.degradation_presets()
    sigs: List[DefectSignature] = []
    for key, d in presets.items():
        if key == "nominal":
            continue
        say(f"signature — '{key}'…")
        r = sf.run_simulforge(None, kind=baseline.maneuver, degradation=d,
                              actuator=actuator, bus=bus)
        ch = _channel_vec(r)
        dz = {k: (ch[k] - baseline.channels[k]) / baseline.bands[k]
              for k in baseline.channels if k in ch}
        sigs.append(DefectSignature(key=key, label=d.label, story=d.story,
                                    dz=dz))
    return sigs


_MATCH_COS = 0.75      # below this, the honest verdict is "matches nothing"


def diagnose(baseline: TwinBaseline, measured: Dict[str, float],
             signatures: List[DefectSignature]) -> TwinDiagnosis:
    """Band-normalise the measured deviation and cosine-match it against the
    predicted signatures. Deterministic, checkable by hand."""
    keys = [k for k in baseline.channels if k in measured]
    z = {k: (float(measured[k]) - baseline.channels[k]) / baseline.bands[k]
         for k in keys}
    verdicts = {k: ("NOMINAL" if abs(v) <= 1.0 else
                    "WATCH" if abs(v) <= 2.0 else "DEGRADED")
                for k, v in z.items()}
    zv = np.array([z[k] for k in keys])
    best_key = best_label = None
    best_cos, mag = 0.0, 0.0
    if np.linalg.norm(zv) > 1e-9:
        for s in signatures:
            sv = np.array([s.dz.get(k, 0.0) for k in keys])
            ns = np.linalg.norm(sv)
            if ns < 1e-9:
                continue
            c = float(np.dot(zv, sv) / (np.linalg.norm(zv) * ns))
            if c > best_cos:
                best_cos, best_key, best_label = c, s.key, s.label
                mag = float(np.linalg.norm(zv) / ns)
    if not any(abs(v) > 1.0 for v in z.values()):
        note = ("every channel inside its declared band — the physical car "
                "still matches the nominal model.")
        best_key = best_label = None
    elif best_key is not None and best_cos >= _MATCH_COS:
        note = (f"the deviation pattern matches '{best_label}' at cosine "
                f"{best_cos:.2f}, magnitude {mag:.2f}× the predicted "
                "signature — a named suspect for the audit, not a certified "
                "root cause.")
    else:
        note = ("drift confirmed, but the pattern matches nothing in the "
                "degradation catalog (best cosine "
                f"{best_cos:.2f} < {_MATCH_COS}) — said honestly instead of "
                "naming a false suspect. Widen the catalog or measure the "
                "branch directly.")
        best_key = best_label = None
    return TwinDiagnosis(baseline=baseline, measured=dict(measured), z=z,
                         channel_verdicts=verdicts, suspect=best_key,
                         suspect_label=best_label, cosine=best_cos,
                         magnitude=mag, note=note)


@dataclass
class HealPlan:
    gain_scale: Optional[float]
    kp_new: Optional[float]
    kd_new: Optional[float]
    gain_note: str
    shim_mm: Optional[float]
    shim_note: str
    cautions: List[str]

    def summary(self) -> dict:
        return {"gain_scale": (round(self.gain_scale, 3)
                               if self.gain_scale else None),
                "kp_new": (round(self.kp_new, 0) if self.kp_new else None),
                "kd_new": (round(self.kd_new, 0) if self.kd_new else None),
                "gain_note": self.gain_note,
                "shim_mm": (round(self.shim_mm, 2)
                            if self.shim_mm is not None else None),
                "shim_note": self.shim_note, "cautions": self.cautions}


def heal_plan(diag: TwinDiagnosis, forge_nominal: sf.SimulForgeResult,
              camber_drift_deg: float = 0.0,
              bolt_span_mm: float = 80.0) -> HealPlan:
    """Arithmetic, not vibes. Two independent corrections:

    · GAIN DE-RATE — steady-state deliverable moment under the MEASURED sagged
      bus: I = duty_max · V_min / R (winding inductance ignored at steady
      state, said so), M = Kt·gear·eff·I, capped at the (possibly de-rated)
      M_max. Gains scale so the peak nominal command fits inside that — the
      controller stops asking for torque the electrics cannot deliver, which
      restores PREDICTABILITY, not performance.
    · SHIM — the measured camber drift θ over the declared mount bolt span s
      is closed by a wedge of thickness t = s·tan|θ| — the same arithmetic
      the Stochastic tab's Alignment Prescription runs, priced here as a
      3-D-printable first correction. Verify with a measured re-alignment.
    """
    cautions: List[str] = []
    act, bus = forge_nominal.actuator, forge_nominal.bus
    if diag.suspect is not None:
        d = sf.degradation_presets().get(diag.suspect)
        if d is not None:
            bus, act = d.apply(bus, act)
            if d.authority_scale < 0.999:
                cautions.append(
                    "the matched defect de-rates the actuator itself "
                    f"(authority ×{d.authority_scale:g}) — no gain retune "
                    "restores lost hardware authority; inspect the branch "
                    "(fuse, connector) first. Fusebox's racer.")
    v_min = float(diag.measured.get("v_min",
                                    diag.baseline.channels.get("v_min",
                                                               bus.V_oc)))
    gain_scale = kp_new = kd_new = None
    gain_note = "no gain retune computed — no electrical drift measured."
    if forge_nominal.ok and forge_nominal.elec.M_cmd.size:
        m_peak_cmd = float(np.max(np.abs(forge_nominal.elec.M_cmd)))
        i_max = act.duty_max * max(v_min, 0.0) / max(act.R_ohm, 1e-9)
        m_deliv = min(act.torque_from_current(i_max), act.M_max_Nm)
        if m_peak_cmd > 1e-6:
            gain_scale = float(np.clip(m_deliv / m_peak_cmd, 0.05, 1.0))
            kp_new = forge_nominal.actuator.kp * gain_scale
            kd_new = forge_nominal.actuator.kd * gain_scale
            gain_note = (
                f"deliverable moment at the measured V_min "
                f"({v_min:.1f} V): {m_deliv:.0f} N·m vs peak nominal "
                f"command {m_peak_cmd:.0f} N·m → gain scale "
                f"{gain_scale:.2f}. Steady-state algebra (winding L "
                "ignored); re-run SimulForge with the new gains to verify.")
            if gain_scale >= 0.999:
                gain_note = ("the degraded bus still delivers the peak "
                             "nominal command — no de-rate needed on this "
                             "evidence.")
    shim_mm = None
    shim_note = "no camber drift declared — no shim computed."
    if abs(camber_drift_deg) > 1e-6:
        shim_mm = bolt_span_mm * math.tan(math.radians(abs(camber_drift_deg)))
        side = "outboard-top" if camber_drift_deg < 0 else "outboard-bottom"
        shim_note = (
            f"measured camber drift {camber_drift_deg:+.2f}° over a "
            f"{bolt_span_mm:g} mm bolt span → wedge t = s·tan|θ| = "
            f"{shim_mm:.2f} mm at the {side} mount face. Print in steps of "
            "0.05 mm; verify with a re-alignment (the Stochastic tab's "
            "Alignment Prescription is the verified version of this "
            "arithmetic).")
    cautions.append("a screening heal, not a certified one — the twin "
                    "corrects toward the declared nominal, and the nominal's "
                    "own evidence grade lives in the Proof Engine ledger.")
    return HealPlan(gain_scale=gain_scale, kp_new=kp_new, kd_new=kd_new,
                    gain_note=gain_note, shim_mm=shim_mm,
                    shim_note=shim_note, cautions=cautions)


# --------------------------------------------------------------------------- #
#  6 · Markdown reports
# --------------------------------------------------------------------------- #
def render_omni_md(res: OmniResult) -> str:
    L: List[str] = []
    L.append("# OmniCore — vehicle-synthesis referee")
    L.append("")
    L.append("## Mission receipt")
    L.append("")
    L.append(f"> {res.mission.text.strip() or '(empty prompt)'}")
    L.append("")
    for c in res.mission.consumed:
        L.append(f"- understood: {c}")
    for a in res.mission.assumptions:
        L.append(f"- assumed: {a}")
    if res.mission.ignored:
        L.append(f"- ignored (the grammar is a grammar, not a language "
                 f"model): {', '.join(res.mission.ignored)}")
    L.append("")
    L.append(f"_{res.knobs.fidelity_note()}_")
    L.append("")
    L.append("## Scorecard")
    L.append("")
    hdr = "| config | " + " | ".join(
        f"{AXES[k][0]} ({AXES[k][1]})" for k in AXES) + " | verdict |"
    L.append(hdr)
    L.append("|" + "---|" * (len(AXES) + 2))
    for c in res.configs:
        cells = [fmt_axis(k, c.objectives.get(k, float("nan")))
                 for k in AXES]
        tag = ("⭐ knee" if c.cid == res.knee_id else
               "front" if c.cid in res.pareto_ids else
               "dominated" if c.feasible else "INFEASIBLE")
        L.append(f"| {c.label} | " + " | ".join(cells) + f" | {tag} |")
    L.append("")
    for c in res.configs:
        for v in c.vetoes:
            L.append(f"- 🔴 {c.label} vetoed — {v}")
        for f in c.flags:
            L.append(f"- 🟡 {c.label} — {f}")
    if res.receipts:
        L.append("")
        L.append("## Dominance receipts")
        L.append("")
        for r in res.receipts:
            L.append(f"- {r}")
    L.append("")
    L.append(f"_Engine ledger: {res.ledger['genesis']} genesis solves, "
             f"{res.ledger['simulforge']} co-solves, "
             f"{res.ledger['ghost_audits']} ghost audits, "
             f"{res.ledger['morph']} growths "
             f"({res.ledger['fe_solves']} FE solves) in "
             f"{res.elapsed_s:.1f} s. A screening front — promote the knee "
             "to the engines' own tabs, then to ANSYS/ADAMS._")
    for w in res.warnings:
        L.append(f"- ⚠️ {w}")
    return "\n".join(L)


def render_twin_md(diag: TwinDiagnosis, plan: Optional[HealPlan] = None
                   ) -> str:
    L: List[str] = []
    L.append("# OmniCore twin — drift audit")
    L.append("")
    L.append("| channel | nominal | measured | z (bands) | verdict |")
    L.append("|---|---|---|---|---|")
    for k in diag.z:
        lab, unit, _ = TWIN_CHANNELS[k]
        L.append(f"| {lab} ({unit}) | {diag.baseline.channels[k]:.3f} | "
                 f"{diag.measured[k]:.3f} | {diag.z[k]:+.2f} | "
                 f"{diag.channel_verdicts[k]} |")
    L.append("")
    L.append(f"**{diag.note}**")
    if plan is not None:
        L.append("")
        L.append("## Heal plan")
        L.append("")
        L.append(f"- {plan.gain_note}")
        L.append(f"- {plan.shim_note}")
        for c in plan.cautions:
            L.append(f"- ⚠️ {c}")
    return "\n".join(L)


# --------------------------------------------------------------------------- #
#  7 · Self-test — the closed forms the referee stands on
# --------------------------------------------------------------------------- #
def _self_test() -> None:                                   # pragma: no cover
    print("omnicore self-test…")
    # grammar: determinism + the flagship prompt
    p = parse_mission("Synthesize a lightweight, 4WD electric off-road "
                      "vehicle optimized for high-frequency bumpy terrain, "
                      "constrained by a $15,000 manufacturing budget and a "
                      "±2 mm weld-pull error shop floor.")
    assert p.maneuver == "curb_strike", p.maneuver
    assert p.budget_usd == 15000.0, p.budget_usd
    assert p.shop == "hand_weld" and p.weld_pull_mm == 2.0
    assert p.drive == "4wd" and "mass" in p.priorities
    assert parse_mission(p.text).summary() == p.summary()   # determinism
    q = parse_mission("")
    assert q.budget_usd is None and q.assumptions
    print("  grammar ok")

    # pareto: a hand-checkable 3-point case
    pts = [{"composure": 1.0, "laps": 10, "mass": 5, "cost": 100, "yield": .9},
           {"composure": 2.0, "laps": 9, "mass": 6, "cost": 110, "yield": .8},
           {"composure": 0.9, "laps": 11, "mass": 4, "cost": 90, "yield": .95}]
    assert pareto_mask(pts) == [False, False, True]
    print("  pareto ok")

    # twin: a synthetic baseline; a scaled signature must match itself
    base = TwinBaseline("step_steer",
                        {k: 1.0 for k in TWIN_CHANNELS},
                        {k: TWIN_CHANNELS[k][2] for k in TWIN_CHANNELS})
    sig = DefectSignature("x", "X", "s",
                          {k: v for k, v in zip(TWIN_CHANNELS,
                                                [3, 3, 2, 4, 0, -2, 1])})
    meas = {k: base.channels[k] + 0.8 * sig.dz[k] * base.bands[k]
            for k in TWIN_CHANNELS}
    d = diagnose(base, meas, [sig])
    assert d.suspect == "x" and abs(d.magnitude - 0.8) < 1e-6, d.summary()
    ok_meas = {k: base.channels[k] + 0.3 * base.bands[k]
               for k in TWIN_CHANNELS}
    d2 = diagnose(base, ok_meas, [sig])
    assert d2.suspect is None and not d2.drifting
    print("  twin match ok")

    # shim identity: t = s·tan θ
    class _R:                       # minimal stand-in for heal arithmetic
        ok = False
        actuator, bus = sf.ActuatorParams(), sf.BusParams()
        elec = type("E", (), {"M_cmd": np.zeros((0, 2))})()
    hp_plan = heal_plan(d2, _R(), camber_drift_deg=-0.5, bolt_span_mm=80.0)
    assert abs(hp_plan.shim_mm - 80.0 * math.tan(math.radians(0.5))) < 1e-9
    print("  shim arithmetic ok")

    # one tiny end-to-end sweep (single shop/actuator/volfrac, coarse)
    knobs = OmniKnobs(actuator_scales={"standard": 1.0}, volfracs=(0.4,),
                      compare_shops=False, genesis_starts=1,
                      genesis_yield_n=80, genesis_max_iter=5,
                      morph_h_mm=3.0, morph_max_iter=6, morph_betas=(2.0,),
                      morph_rounds=1, audit_samples=8, fan_cases=2,
                      band_camber_deg=0.35, band_toe_deg=0.20)
    res = run_omnicore(None, p, knobs)
    assert len(res.configs) == 1
    if res.configs[0].feasible:
        assert res.pareto_ids == [1] and res.knee_id == 1
    print("  sweep:", res.configs[0].label,
          "feasible" if res.configs[0].feasible else
          f"vetoed ({res.configs[0].vetoes[:1]})",
          f"{res.elapsed_s:.1f} s")
    md = render_omni_md(res)
    assert "Mission receipt" in md and "Engine ledger" in md
    json.loads(res.to_json())
    print("omnicore self-test passed.")


if __name__ == "__main__":                                  # pragma: no cover
    _self_test()
