# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  Module: morphmesh — the structural auto-synthesizer
# ============================================================================
"""
MorphMesh — generative structural topology grown from the TRANSIENT load fan
the Ghost Topology audit actually solved, and constrained — hard — to the
fabrication limits the shop actually declared in Stochastic Inversion.

WHY THIS MODULE EXISTS
----------------------
Two silent assumptions cripple every structural bracket a student team draws:

  1. THE STATIC LOAD CASE. A chassis tab or bellcrank web is sized against ONE
     hand-picked force — usually "worst Fy" from a spreadsheet. But Ghost
     Topology has already shown that under a transient event the member force
     ROTATES: the link deflects, the geometry migrates, and the force line the
     tab must react sweeps a FAN of directions with different amplitudes and
     dwell. A plate optimised for one arrow is structurally illiterate about
     the other thirty degrees of arc the same event visits.

  2. THE PERFECT MACHINIST. Generative-design tools that do consider topology
     produce organic 3D-printed lattices no workshop floor can weld, mill or
     waterjet — so the output is admired and then redrawn by hand. The shape
     was never manufacturable because the algorithm never knew what the shop
     could hold. Meanwhile Stochastic Inversion already carries exactly that
     knowledge: the asymmetric per-point error field of hand-welded tabs,
     jigged fixtures, or CNC'd plates.

MorphMesh is the join. It reads the per-instant member force vectors straight
off a GhostAudit (the deformed geometry Ghost Topology already solved — the
"shifting stress tensor" is free), compresses the event into an
exposure-weighted fan of load cases, and grows a volume-fraction (SIMP)
topology on a 2D plate domain — the waterjet/plasma-cut sheet bracket that IS
the FSAE fabrication reality — that carries the WHOLE fan, not one arrow.
Then the shop gets a veto: the declared fabrication class (seeded from the
same Stochastic Inversion ToleranceField presets — hand-welded tabs, jig,
CNC) sets a minimum buildable rib width and a heat-affected-zone rule at the
weld line, the generated shape is AUDITED against them by morphological
opening (a measurement, not a hope), and a shape with ribs the mill can't hold
or webs the welder will warp is REJECTED — the engine re-grows at a coarser
enforced length scale until the shape passes or the impossibility is named.

WHAT IT DOES
------------
  * THE TRANSIENT LOAD FAN. For a chosen member (UF/UR/LF/LR/TR/PR), each
    audited GhostInstant contributes the force vector the member applies to
    its chassis tab: magnitude = the solved ghost member force, direction =
    the DEFORMED link line at that instant (outboard point read off
    GhostInstant.ghost — the load migration is inherited, not re-derived).
    The best-fit plane of the fan becomes the bracket plane (out-of-plane
    share printed, never hidden); the instants are split into contiguous
    equal-exposure angular groups, each becoming one load case: direction =
    exposure-weighted mean, amplitude = the PEAK force in the group (design
    to the worst visitor, weight by the dwell).
  * GENERATIVE VOLUME-FRACTION TOPOLOGY. Classic density-based SIMP on a
    regular plane-stress Q4 grid: multi-load-case compliance objective
    (Σ w_k · fᵀu, one matrix factorisation shared by every case per
    iteration), cone density filter, smoothed-Heaviside projection with β
    continuation so the answer collapses to solid/void at a HELD length
    scale, optimality-criteria update under a volume-fraction budget.
    Material is stripped where no case's stress path travels and ribbed
    exactly along the migrating force lines.
  * THE MANUFACTURING-CONSTRAINED MESH. FabricationLimits carries the
    declared shop class: minimum rib width (kerf/bead floor + twice the
    positional accuracy of the class, so the worst-case as-cut web is still
    metal), a heat-affected-zone depth from the weld anchor, and a stiffer
    minimum rib inside it (thin cross-sections warp under the bead — a
    screening heuristic, stated as such, not a welding simulation). The
    filter radius ENFORCES the length scale during growth; the finished
    binary shape is then MEASURED by morphological opening with the required
    disk — any rib the disk cannot pass through is a violation. Violations
    above a sliver tolerance reject the shape and coarsen the next round.
  * THE STRESS-PATH MAP. Per element, the exposure-weighted stress tensor
    across the fan is reduced to a principal direction and magnitude — the
    field "where the load migration travels", shipped for the UI quiver and
    for the engineer's eye to sanity-check the ribs against.
  * THE STRUCTURAL SCREEN, INHERITED. The finished shape is re-solved at
    each case's PEAK amplitude, von Mises recovered per element, and the
    worst FoS judged against the same standing 1.5-on-yield rule the ghost
    and bracket audits divide by. Plane-stress scaling makes the fix
    arithmetic: FoS is linear in plate thickness, so an under-margin shape
    ships with the exact thickness that heals it.

THE HONESTY CONTRACT
--------------------
  * The domain is a 2D plane-stress PLATE — the waterjet/laser/plasma sheet
    bracket, tab and bellcrank web reality of a formula shop. It is NOT a 3D
    solid: out-of-plane bending, weld distortion mechanics and bolt preload
    are outside scope and said so. A bellcrank is served as a pivot-anchored
    plate with one loaded bore — the single-reaction idealisation, stated.
  * Linear elastic, quasi-static, small strain. The load fan inherits Ghost
    Topology's time-scale-separation argument whole — and its limits.
  * SIMP + OC + filter + projection is the textbook engine (Sigmund's 88-line
    lineage), not a novelty; the novelty is only WHAT feeds it (the transient
    ghost fan) and WHAT vetoes it (the declared shop field). Every step is
    checkable arithmetic; the same seed-free deterministic pipeline gives the
    same shape every run.
  * The length-scale audit is a MEASUREMENT on the binarised shape
    (morphological opening), so the pass/fail is earned, not assumed from
    the filter radius. A ≤1 % sliver tolerance absorbs binarisation
    staircase artifacts and is printed, not hidden.
  * Nothing in the public API raises. A failed run returns a flagged
    MorphResult whose verdict says what happened; an audit with no usable
    instants returns LOAD_STARVED instead of optimising zeros.
  * The output is a STARTING SHAPE for the CAD seat and the FEA queue — a
    pre-validated topology with its load provenance attached — never a
    certified part. The report footer says so.

No streamlit / pandas / plotly imports. Unit-testable headless.
Self-test:  python3 -m suspension.morphmesh
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field as _dcfield, asdict
from typing import Dict, List, Optional, Tuple

import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla
from scipy import ndimage as ndi

from . import flex as flexmod
from .kinematics import Hardpoints


# --------------------------------------------------------------------------- #
#  Verdicts — the module's whole vocabulary of outcomes
# --------------------------------------------------------------------------- #
VERDICTS = (
    "FORGEABLE",       # converged and passed the shop audit at the first length scale
    "COARSENED",       # shapes were REJECTED; a coarser rib pattern passed — premium printed
    "UNBUILDABLE",     # no shape within the volume budget satisfies the declared limits
    "LOAD_STARVED",    # the audit gave no usable member forces to grow from
    "SOLVER_LIMITED",  # the FE solve failed (degenerate anchor/domain) — named, not hidden
)

_FOS_RULE = 1.5        # the standing 1.5-on-yield rule, same as ghost/bracket audits
_SLIVER_TOL = 0.01     # ≤1 % violating pixels tolerated as binarisation staircase


# --------------------------------------------------------------------------- #
#  Member geometry — which hardpoints define each link's force line
# --------------------------------------------------------------------------- #
_MEMBER_INNER = {"UF": "upper_front_inner", "UR": "upper_rear_inner",
                 "LF": "lower_front_inner", "LR": "lower_rear_inner",
                 "TR": "tie_rod_inner"}
_MEMBER_OUTER = {"UF": "upper_outer", "UR": "upper_outer",
                 "LF": "lower_outer", "LR": "lower_outer",
                 "TR": "tie_rod_outer", "PR": "pushrod_outer"}
MEMBER_LABELS = {"UF": "UCA front leg", "UR": "UCA rear leg",
                 "LF": "LCA front leg", "LR": "LCA rear leg",
                 "TR": "tie rod", "PR": "pushrod"}


# --------------------------------------------------------------------------- #
#  The load fan — the transient event compressed into weighted load cases
# --------------------------------------------------------------------------- #
@dataclass
class LoadCase:
    """One arrow of the fan: an in-plane unit direction, a design amplitude
    (the PEAK member force among the instants grouped here), and the exposure
    weight (this group's share of Σ|F| over the event)."""
    dir2: np.ndarray          # (2,) unit vector in the bracket plane
    F_N: float                # peak |member force| in the group (design amplitude)
    weight: float             # exposure share, Σ groups = 1
    n_instants: int
    t_lo_s: float
    t_hi_s: float
    angle_deg: float          # atan2(dir2) — for the report

    def summary(self) -> dict:
        return {"angle_deg": round(self.angle_deg, 1), "F_N": round(self.F_N, 1),
                "weight": round(self.weight, 3), "n_instants": self.n_instants,
                "t_lo_ms": round(self.t_lo_s * 1e3, 1),
                "t_hi_ms": round(self.t_hi_s * 1e3, 1)}


@dataclass
class LoadFan:
    """The whole fan: cases + the plane they live in + what was dropped."""
    member: str
    cases: List[LoadCase]
    e1: np.ndarray            # (3,) bracket-plane basis (SAE mm frame)
    e2: np.ndarray
    normal: np.ndarray
    inplane_share: float      # Σ|in-plane| / Σ|F| — the honesty number
    span_deg: float           # angular arc between the RETAINED case arrows
    n_instants: int
    peak_F_N: float
    reversal_share: float = 0.0   # exposure fraction opposing the dominant arrow
    warnings: List[str] = _dcfield(default_factory=list)

    @property
    def ok(self) -> bool:
        return bool(self.cases) and self.peak_F_N > 1.0

    def summary(self) -> dict:
        return {"member": self.member,
                "cases": [c.summary() for c in self.cases],
                "inplane_share": round(self.inplane_share, 3),
                "span_deg": round(self.span_deg, 1),
                "n_instants": self.n_instants,
                "peak_F_N": round(self.peak_F_N, 1),
                "reversal_share": round(self.reversal_share, 3),
                "warnings": list(self.warnings)}


def load_fan_from_audit(audit, hp: Hardpoints, member: str = "LF",
                        n_cases: int = 4,
                        plane: Optional[Tuple[np.ndarray, np.ndarray]] = None
                        ) -> LoadFan:
    """
    Read the per-instant force the member applies to its chassis tab straight
    off a GhostAudit and compress it into ≤ n_cases exposure-weighted arrows.

    Direction per instant is the DEFORMED link line (inner hardpoint → the
    ghost outboard point at that instant): the load migration Ghost Topology
    solved is inherited, not re-derived. Force on the tab from member tension
    T is T·(outer − inner)/|outer − inner| — tension pulls the tab outboard,
    compression pushes it inboard, and a sign flip mid-event legitimately
    reverses the arrow (the fan may span > 180°; that is the physics).

    ``plane``: optional (e1, e2) 3-vector basis for the bracket plane. Default
    is the exposure-weighted best-fit plane of the fan itself; the dropped
    out-of-plane share is reported, never hidden.
    """
    member = str(member).upper()
    warnings: List[str] = []
    if member not in _MEMBER_OUTER:
        return LoadFan(member, [], np.eye(3)[0], np.eye(3)[1], np.eye(3)[2],
                       0.0, 0.0, 0, 0.0,
                       warnings=[f"unknown member '{member}' — allowed: "
                                 f"{', '.join(sorted(_MEMBER_OUTER))}"])

    # inner (chassis-fixed) end of the force line
    if member == "PR":
        inner = getattr(hp, "rocker_pushrod", None)
        if inner is None:
            inner = np.asarray(hp.upper_front_inner, float) + np.array([0., 0., 200.])
            warnings.append("no rocker defined — pushrod inner proxied 200 mm "
                            "above the upper-front pickup; treat the PR fan as "
                            "indicative only.")
    else:
        inner = getattr(hp, _MEMBER_INNER[member])
    inner = np.asarray(inner, float).reshape(3)

    instants = list(getattr(audit, "instants", []) or [])
    f3, tt = [], []
    for g in instants:
        mg = (g.margins or {}).get(member)
        T = float(mg["force_N"]) if mg else float(
            (g.load_path_shift or {}).get(member, {}).get("ghost_N", 0.0))
        outer = getattr(g.ghost, _MEMBER_OUTER[member], None)
        if outer is None:
            outer = getattr(hp, _MEMBER_OUTER[member], None)
        if outer is None:
            continue
        d = np.asarray(outer, float).reshape(3) - inner
        L = float(np.linalg.norm(d))
        if L < 1e-9:
            continue
        f3.append(T * d / L)
        tt.append(float(g.t))
    f3 = np.asarray(f3, float).reshape(-1, 3)
    tt = np.asarray(tt, float)

    mags = np.linalg.norm(f3, axis=1) if f3.size else np.zeros(0)
    exposure = float(mags.sum())
    if f3.shape[0] == 0 or exposure < 1.0 or float(mags.max(initial=0.0)) < 1.0:
        warnings.append("the audit carries no usable force history for this "
                        "member — nothing to grow a topology from.")
        return LoadFan(member, [], np.eye(3)[0], np.eye(3)[1], np.eye(3)[2],
                       0.0, 0.0, int(f3.shape[0]), float(mags.max(initial=0.0)),
                       warnings=warnings)

    # ---- the bracket plane: given, or the fan's own best-fit --------------- #
    if plane is not None:
        e1 = np.asarray(plane[0], float); e1 = e1 / np.linalg.norm(e1)
        e2 = np.asarray(plane[1], float)
        e2 = e2 - e1 * float(e1 @ e2)
        e2 = e2 / np.linalg.norm(e2)
    else:
        C = (f3 * mags[:, None]).T @ f3            # exposure-weighted scatter
        w_eig, V = np.linalg.eigh(C)
        e1, e2 = V[:, -1], V[:, -2]                # top-2 principal directions
    nrm = np.cross(e1, e2)

    v2 = np.stack([f3 @ e1, f3 @ e2], axis=1)      # in-plane components, N
    m2 = np.linalg.norm(v2, axis=1)
    inplane_share = float(m2.sum() / exposure)
    if inplane_share < 0.85:
        warnings.append(
            f"only {inplane_share*100:.0f}% of the load exposure lies in the "
            "bracket plane — this member's fan is genuinely 3D; the plate "
            "answer under-represents it (consider a different plane or a "
            "gusseted design).")

    keep = m2 > max(1.0, 0.02 * float(m2.max()))
    v2, m2, tk = v2[keep], m2[keep], tt[keep]
    if v2.shape[0] == 0:
        warnings.append("every instant's in-plane force is negligible.")
        return LoadFan(member, [], e1, e2, nrm, inplane_share, 0.0,
                       int(f3.shape[0]), float(mags.max()), warnings=warnings)

    # wrap-safe angles: rotate so the exposure-weighted mean direction sits at
    # 0° — a fan hugging the ±180° branch cut must not read as a 360° span
    theta = np.arctan2(v2[:, 1], v2[:, 0])
    mean_v = (v2 * m2[:, None]).sum(0)
    theta0 = math.atan2(mean_v[1], mean_v[0]) if np.linalg.norm(mean_v) > 1e-9 \
        else float(theta[int(np.argmax(m2))])
    dom_dir = np.array([math.cos(theta0), math.sin(theta0)])

    # ---- sign-partitioned, contiguous equal-exposure groups → cases -------- #
    # A tension↔compression flip reverses the arrow on the tab by 180°; a
    # vector mean across the flip would cancel it away, so each sign lobe is
    # grouped SEPARATELY — a ≥2 %-exposure reversal always keeps its arrow.
    n_cases = max(1, int(n_cases))
    total = float(m2.sum())
    signs = (v2 @ dom_dir) >= 0.0

    def _group(vv: np.ndarray, mm_: np.ndarray, tt_: np.ndarray,
               n_grp: int) -> List[LoadCase]:
        th = np.degrees(np.angle(np.exp(1j * (np.arctan2(vv[:, 1], vv[:, 0])
                                              - theta0))))
        o = np.argsort(th)
        vv, mm_, tt_ = vv[o], mm_[o], tt_[o]
        cum = np.concatenate([[0.0], np.cumsum(mm_)])
        tot = cum[-1]
        out: List[LoadCase] = []
        lo = 0
        for k in range(n_grp):
            target = tot * (k + 1) / n_grp
            hi = int(np.searchsorted(cum, target, side="left"))
            hi = max(hi, lo + 1)
            hi = min(hi, len(mm_)) if k < n_grp - 1 else len(mm_)
            if hi <= lo:
                continue
            seg_v, seg_m, seg_t = vv[lo:hi], mm_[lo:hi], tt_[lo:hi]
            mean = (seg_v * seg_m[:, None]).sum(0)
            if np.linalg.norm(mean) < 1e-9:
                mean = seg_v[int(np.argmax(seg_m))]
            d2 = mean / np.linalg.norm(mean)
            out.append(LoadCase(
                dir2=d2, F_N=float(seg_m.max()),
                weight=float(seg_m.sum() / total), n_instants=int(hi - lo),
                t_lo_s=float(seg_t.min()), t_hi_s=float(seg_t.max()),
                angle_deg=float(np.degrees(math.atan2(d2[1], d2[0]))) + 0.0
                if abs(math.atan2(d2[1], d2[0])) > 1e-12 else 0.0))
            lo = hi
        return out

    cases: List[LoadCase] = []
    for lobe in (signs, ~signs):
        share = float(m2[lobe].sum() / (total or 1.0))
        if share < 0.02 or not lobe.any():
            continue
        n_grp = max(1, int(round(n_cases * share)))
        cases.extend(_group(v2[lobe], m2[lobe], tk[lobe], n_grp))
    cases = [c for c in cases if c.weight >= 0.02]

    # merge cases whose directions the grouping split but the event didn't
    # (wrap-aware: −180° and +180° are the same arrow) — a direction-static
    # event honestly yields ONE arrow, not four clones
    merged: List[LoadCase] = []
    for c in sorted(cases, key=lambda c: -c.weight):
        hit = next((i for i, p in enumerate(merged)
                    if abs(np.degrees(np.angle(np.exp(1j * np.radians(
                        c.angle_deg - p.angle_deg))))) < 3.0), None)
        if hit is None:
            merged.append(c)
        else:
            p = merged[hit]
            merged[hit] = LoadCase(
                dir2=p.dir2 if p.F_N >= c.F_N else c.dir2,
                F_N=max(p.F_N, c.F_N), weight=p.weight + c.weight,
                n_instants=p.n_instants + c.n_instants,
                t_lo_s=min(p.t_lo_s, c.t_lo_s), t_hi_s=max(p.t_hi_s, c.t_hi_s),
                angle_deg=(p.angle_deg if p.F_N >= c.F_N else c.angle_deg))
    cases = merged
    wsum = sum(c.weight for c in cases) or 1.0
    for c in cases:
        c.weight /= wsum

    # span between the RETAINED arrows (an event with one direction has span
    # 0 whatever slivers were dropped) + the sign-reversal exposure, named
    def _adist(a: float, b: float) -> float:
        return abs(float(np.degrees(np.angle(np.exp(1j * np.radians(a - b))))))
    span = max((_adist(a.angle_deg, b.angle_deg)
                for i, a in enumerate(cases) for b in cases[i + 1:]),
               default=0.0)
    rev_share = float(m2[~signs].sum() / (total or 1.0))
    if rev_share >= 0.02:
        warnings.append(
            f"the member force REVERSES sign for {rev_share*100:.0f}% of the "
            "event's load exposure — the tab is loaded both ways; the fan "
            "carries both arrows.")

    return LoadFan(member, cases, e1, e2, nrm, inplane_share, span,
                   int(f3.shape[0]), float(m2.max()), rev_share,
                   warnings=warnings)


# --------------------------------------------------------------------------- #
#  FabricationLimits — the shop's declared veto, in millimetres
# --------------------------------------------------------------------------- #
@dataclass
class FabricationLimits:
    """
    What the floor can actually hold. Two numbers do the vetoing:

      min_rib_mm     : the thinnest structural web the process can produce and
                       hold: web floor (kerf / bead / hand-grind limit) plus
                       TWICE the positional accuracy of the class — each cut
                       edge lands within ±u, so a nominal-w rib survives as
                       ≥ w − 2u of metal; requiring that to clear the floor
                       gives min_rib = floor + 2u.
      min_rib_haz_mm : the stiffer rule inside the heat-affected zone
                       (haz_mm from the weld anchor): a delicate cross-section
                       next to the bead warps. The default 1.6× factor is a
                       SCREENING heuristic (distortion resistance grows
                       steeply with section), stated as such — not a welding
                       simulation. haz_mm = 0 disables the zone (bolted /
                       machined brackets).

    ``from_shop`` seeds all of it from the same class names Stochastic
    Inversion presets carry (hand_weld / jig_weld / cnc); ``from_tolerance_field``
    reads a declared ToleranceField directly — the per-point half-span of the
    shop's own error cloud IS the positional accuracy u (a screening
    identification, editable). Every number is a knob.
    """
    process: str = "hand_weld"
    min_rib_mm: float = 5.0
    haz_mm: float = 8.0
    min_rib_haz_mm: float = 8.0
    accuracy_mm: float = 1.5          # the u the limits were derived from
    web_floor_mm: float = 2.0

    _FLOORS = {"hand_weld": 2.0, "jig_weld": 1.5, "cnc": 1.0}
    _ACCURACY = {"hand_weld": 1.5, "jig_weld": 0.5, "cnc": 0.05}
    _HAZ = {"hand_weld": 8.0, "jig_weld": 6.0, "cnc": 0.0}

    @staticmethod
    def from_shop(shop: str = "hand_weld", haz_factor: float = 1.6
                  ) -> "FabricationLimits":
        if shop not in FabricationLimits._FLOORS:
            raise ValueError(f"Unknown shop class '{shop}'. Allowed: "
                             f"{', '.join(FabricationLimits._FLOORS)}.")
        u = FabricationLimits._ACCURACY[shop]
        floor = FabricationLimits._FLOORS[shop]
        rib = floor + 2.0 * u
        haz = FabricationLimits._HAZ[shop]
        return FabricationLimits(
            process=shop, min_rib_mm=rib, haz_mm=haz,
            min_rib_haz_mm=(rib * haz_factor if haz > 0 else rib),
            accuracy_mm=u, web_floor_mm=floor)

    @staticmethod
    def from_tolerance_field(fld, process: str = "declared field",
                             web_floor_mm: float = 2.0, haz_mm: float = 8.0,
                             haz_factor: float = 1.6) -> "FabricationLimits":
        """u = the largest per-axis half-span across the field's specs — the
        positional accuracy the shop itself declared it holds."""
        spans = []
        for spec in getattr(fld, "specs", {}).values():
            spans.append(float(np.max(spec.hi - spec.lo)) / 2.0)
        u = max(spans) if spans else 0.5
        rib = web_floor_mm + 2.0 * u
        return FabricationLimits(
            process=process, min_rib_mm=rib, haz_mm=haz_mm,
            min_rib_haz_mm=(rib * haz_factor if haz_mm > 0 else rib),
            accuracy_mm=u, web_floor_mm=web_floor_mm)

    def summary(self) -> dict:
        return {"process": self.process,
                "min_rib_mm": round(self.min_rib_mm, 2),
                "haz_mm": round(self.haz_mm, 2),
                "min_rib_haz_mm": round(self.min_rib_haz_mm, 2),
                "accuracy_mm": round(self.accuracy_mm, 3),
                "web_floor_mm": round(self.web_floor_mm, 2)}


# --------------------------------------------------------------------------- #
#  PlateDomain — the sheet the shape grows inside
# --------------------------------------------------------------------------- #
@dataclass
class PlateDomain:
    """
    A rectangular design plate, x right / y up, millimetres.

      anchor        : "bottom_edge" — the weld line (chassis tab); every node
                      on y=0 is fixed and the HAZ band grows up from it — or
                      "bores" — each anchor bore's ring nodes are fixed
                      (bolted bracket / bellcrank pivot; no HAZ unless
                      limits.haz_mm says otherwise, measured from the rings).
      load_bore     : (cx, cy, r) — the rod-end / clevis bore. Its interior
                      is a keep-out void; a keep-in solid annulus of
                      ring_mm surrounds it (the boss); each case's force is
                      spread over the ring nodes.
      ring_mm       : boss annulus width — grown to min_rib_mm at validation
                      if declared thinner (a boss the process can't hold is
                      the same defect as a rib it can't hold).
    """
    width_mm: float = 60.0
    height_mm: float = 80.0
    h_mm: float = 1.0                    # element size
    thickness_mm: float = 4.0
    material: str = "Steel 4130"
    yield_MPa: float = 460.0
    anchor: str = "bottom_edge"
    anchor_bores: List[Tuple[float, float, float]] = _dcfield(default_factory=list)
    load_bore: Tuple[float, float, float] = (30.0, 62.0, 5.0)
    ring_mm: float = 4.0
    name: str = "chassis tab"

    # ---- presets ---------------------------------------------------------- #
    @staticmethod
    def chassis_tab(width_mm: float = 60.0, height_mm: float = 80.0,
                    bore_r_mm: float = 5.0, thickness_mm: float = 4.0,
                    material: str = "Steel 4130", yield_MPa: float = 460.0,
                    h_mm: float = 1.0) -> "PlateDomain":
        """The hand-welded chassis tab: weld line along the bottom, rod-end
        bore up top on the centreline."""
        return PlateDomain(
            width_mm=width_mm, height_mm=height_mm, h_mm=h_mm,
            thickness_mm=thickness_mm, material=material, yield_MPa=yield_MPa,
            anchor="bottom_edge",
            load_bore=(width_mm / 2.0, height_mm - 3.0 * bore_r_mm, bore_r_mm),
            ring_mm=max(4.0, bore_r_mm * 0.8), name="chassis tab")

    @staticmethod
    def pivot_bracket(width_mm: float = 90.0, height_mm: float = 70.0,
                      pivot_r_mm: float = 6.0, bore_r_mm: float = 5.0,
                      thickness_mm: float = 5.0, material: str = "Aluminium 7075",
                      yield_MPa: float = 430.0, h_mm: float = 1.0
                      ) -> "PlateDomain":
        """The bellcrank-web idealisation: a fixed pivot bore, one loaded
        bore across the plate. Single-reaction plate — the scope note in the
        module docstring applies."""
        return PlateDomain(
            width_mm=width_mm, height_mm=height_mm, h_mm=h_mm,
            thickness_mm=thickness_mm, material=material, yield_MPa=yield_MPa,
            anchor="bores",
            anchor_bores=[(0.22 * width_mm, height_mm / 2.0, pivot_r_mm)],
            load_bore=(0.82 * width_mm, height_mm / 2.0, bore_r_mm),
            ring_mm=max(4.0, bore_r_mm * 0.8), name="pivot bracket (bellcrank web)")

    # ---- derived grid ----------------------------------------------------- #
    @property
    def nx(self) -> int:
        return max(6, int(round(self.width_mm / self.h_mm)))

    @property
    def ny(self) -> int:
        return max(6, int(round(self.height_mm / self.h_mm)))

    def cell_centers(self) -> Tuple[np.ndarray, np.ndarray]:
        """(ny, nx) x and y coordinates of element centres, mm."""
        xs = (np.arange(self.nx) + 0.5) * self.h_mm
        ys = (np.arange(self.ny) + 0.5) * self.h_mm
        X, Y = np.meshgrid(xs, ys)
        return X, Y


# --------------------------------------------------------------------------- #
#  The finite-element kernel — plane-stress Q4, derived by quadrature
# --------------------------------------------------------------------------- #
def _q4_ke_and_b(nu: float, h: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Element stiffness (E=1, t=1) by 2×2 Gauss quadrature, plus the
    strain–displacement matrix at the centroid and the unit constitutive D.
    Node order: bottom-left, bottom-right, top-right, top-left (CCW).
    Deriving KE numerically (instead of trusting a transcribed 88-line
    matrix) removes every node-ordering doubt at the cost of a microsecond."""
    D = (1.0 / (1.0 - nu * nu)) * np.array(
        [[1.0, nu, 0.0], [nu, 1.0, 0.0], [0.0, 0.0, (1.0 - nu) / 2.0]])
    gp = np.array([-1.0, 1.0]) / math.sqrt(3.0)
    KE = np.zeros((8, 8))

    def bmat(xi: float, eta: float) -> np.ndarray:
        dN = 0.25 * np.array([  # dN/dxi, dN/deta per node (CCW from BL)
            [-(1 - eta), -(1 - xi)],
            [+(1 - eta), -(1 + xi)],
            [+(1 + eta), +(1 + xi)],
            [-(1 + eta), +(1 - xi)]])
        dNxy = dN * (2.0 / h)               # J = h/2 · I for a square element
        B = np.zeros((3, 8))
        for a in range(4):
            B[0, 2 * a] = dNxy[a, 0]
            B[1, 2 * a + 1] = dNxy[a, 1]
            B[2, 2 * a] = dNxy[a, 1]
            B[2, 2 * a + 1] = dNxy[a, 0]
        return B

    detJ = (h / 2.0) ** 2
    for xi in gp:
        for eta in gp:
            B = bmat(xi, eta)
            KE += B.T @ D @ B * detJ
    return KE, bmat(0.0, 0.0), D


class _PlateFE:
    """Grid bookkeeping + assembly + shared-factorisation multi-case solve."""

    def __init__(self, dom: PlateDomain):
        self.dom = dom
        nx, ny, h = dom.nx, dom.ny, dom.h_mm
        self.nx, self.ny, self.h = nx, ny, h
        self.nn = (nx + 1) * (ny + 1)
        self.ndof = 2 * self.nn
        mat = flexmod.MATERIALS.get(dom.material)
        self.E0 = float(mat.E) if mat is not None else 205000.0   # MPa
        self.Emin = self.E0 * 1e-9
        self.rho = float(getattr(mat, "rho", 7850.0)) if mat is not None else 7850.0
        self.nu = 0.3
        self.KE, self.B0, self.D1 = _q4_ke_and_b(self.nu, h)

        # element → 8 dofs (node id = ix*(ny+1)+iy; element (ey, ex) row-major)
        ex, ey = np.meshgrid(np.arange(nx), np.arange(ny))       # (ny, nx)
        n1 = (ex * (ny + 1) + ey).ravel()                        # BL
        n2 = ((ex + 1) * (ny + 1) + ey).ravel()                  # BR
        n3 = ((ex + 1) * (ny + 1) + ey + 1).ravel()              # TR
        n4 = (ex * (ny + 1) + ey + 1).ravel()                    # TL
        self.edof = np.stack([2 * n1, 2 * n1 + 1, 2 * n2, 2 * n2 + 1,
                              2 * n3, 2 * n3 + 1, 2 * n4, 2 * n4 + 1], axis=1)
        self.iK = np.repeat(self.edof, 8, axis=1).ravel()
        self.jK = np.tile(self.edof, (1, 8)).ravel()

        # ---- masks: passive void (bores), passive solid (bosses) ---------- #
        X, Y = dom.cell_centers()
        self.pass_void = np.zeros((ny, nx), bool)
        self.pass_solid = np.zeros((ny, nx), bool)
        ring = max(dom.ring_mm, 0.0)
        bores = [dom.load_bore] + list(dom.anchor_bores)
        for (cx, cy, r) in bores:
            rr = np.hypot(X - cx, Y - cy)
            self.pass_void |= rr < r
            self.pass_solid |= (rr >= r) & (rr < r + ring)
        self.pass_solid &= ~self.pass_void
        self.active = ~(self.pass_void | self.pass_solid)

        # ---- anchors ------------------------------------------------------ #
        fixed_nodes: List[int] = []
        if dom.anchor == "bottom_edge":
            fixed_nodes = [ix * (ny + 1) + 0 for ix in range(nx + 1)]
        else:
            xn = np.arange(nx + 1) * h
            yn = np.arange(ny + 1) * h
            XN, YN = np.meshgrid(xn, yn, indexing="ij")          # (nx+1, ny+1)
            for (cx, cy, r) in dom.anchor_bores:
                rr = np.hypot(XN - cx, YN - cy)
                sel = (rr >= r * 0.5) & (rr <= r + 1.5 * h)
                ids = np.nonzero(sel.ravel())[0]
                fixed_nodes.extend(int(i) for i in ids)
        self.fixed_dofs = np.unique(np.concatenate(
            [[2 * n, 2 * n + 1] for n in fixed_nodes]).astype(int)) \
            if fixed_nodes else np.zeros(0, int)
        self.free = np.setdiff1d(np.arange(self.ndof), self.fixed_dofs)

        # ---- load ring nodes --------------------------------------------- #
        cx, cy, r = dom.load_bore
        xn = np.arange(nx + 1) * h
        yn = np.arange(ny + 1) * h
        XN, YN = np.meshgrid(xn, yn, indexing="ij")
        rr = np.hypot(XN - cx, YN - cy)
        sel = (rr >= max(r - 0.75 * h, 0.0)) & (rr <= r + ring + 0.5 * h)
        self.load_nodes = np.nonzero(sel.ravel())[0]
        if self.load_nodes.size == 0:                            # degenerate
            self.load_nodes = np.array(
                [int(round(cx / h)) * (ny + 1) + int(round(cy / h))])

        # ---- HAZ band (element mask) -------------------------------------- #
        self.haz_from = None
        if dom.anchor == "bottom_edge":
            self.haz_from = ("edge", None)
        elif dom.anchor_bores:
            self.haz_from = ("bores", list(dom.anchor_bores))

    # ------------------------------------------------------------------ #
    def force_vector(self, case: LoadCase) -> np.ndarray:
        F = np.zeros(self.ndof)
        n = self.load_nodes
        F[2 * n] = case.F_N * case.dir2[0] / n.size
        F[2 * n + 1] = case.F_N * case.dir2[1] / n.size
        return F

    def haz_mask(self, haz_mm: float) -> np.ndarray:
        """Element mask of the heat-affected band, from the anchor geometry."""
        mask = np.zeros((self.ny, self.nx), bool)
        if haz_mm <= 0 or self.haz_from is None:
            return mask
        X, Y = self.dom.cell_centers()
        if self.haz_from[0] == "edge":
            mask = Y <= haz_mm
        else:
            for (cx, cy, r) in self.haz_from[1]:
                mask |= np.hypot(X - cx, Y - cy) <= (r + haz_mm)
        return mask

    # ------------------------------------------------------------------ #
    def solve_cases(self, x_phys: np.ndarray, cases: List[LoadCase],
                    ) -> Tuple[List[np.ndarray], List[float]]:
        """One assembly + one factorisation, every case rides it."""
        t = self.dom.thickness_mm
        Ee = (self.Emin + (x_phys.ravel() ** 3) * (self.E0 - self.Emin)) * t
        sK = (self.KE.ravel()[None, :] * Ee[:, None]).ravel()
        K = sp.coo_matrix((sK, (self.iK, self.jK)),
                          shape=(self.ndof, self.ndof)).tocsc()
        Kff = K[self.free][:, self.free]
        lu = spla.splu(Kff)
        Us, cs = [], []
        for case in cases:
            F = self.force_vector(case)
            U = np.zeros(self.ndof)
            U[self.free] = lu.solve(F[self.free])
            Us.append(U)
            cs.append(float(F @ U))
        return Us, cs

    def element_strain(self, U: np.ndarray) -> np.ndarray:
        """(nel, 3) engineering strain at element centroids."""
        ue = U[self.edof]                                        # (nel, 8)
        return ue @ self.B0.T

    def von_mises(self, U: np.ndarray, x_solid: np.ndarray) -> np.ndarray:
        """(ny, nx) von Mises stress, MPa, on the given 0/1 field (void → 0).
        Plane stress: displacement already reflects thickness through the
        assembly, so σ here is the true membrane stress of the plate."""
        eps = self.element_strain(U)                             # (nel, 3)
        sig = eps @ (self.E0 * self.D1).T                        # MPa
        vm = np.sqrt(sig[:, 0] ** 2 - sig[:, 0] * sig[:, 1]
                     + sig[:, 1] ** 2 + 3.0 * sig[:, 2] ** 2)
        vm = vm.reshape(self.ny, self.nx) * (x_solid > 0.5)
        return vm

    def stress_tensor(self, U: np.ndarray) -> np.ndarray:
        """(nel, 3) [σxx, σyy, τxy] MPa (solid modulus — display field)."""
        return self.element_strain(U) @ (self.E0 * self.D1).T


# --------------------------------------------------------------------------- #
#  Filters — cone density filter (via convolution) + smoothed Heaviside
# --------------------------------------------------------------------------- #
def _cone_kernel(r_el: float) -> np.ndarray:
    R = max(1, int(math.ceil(r_el - 1e-9)))
    y, x = np.mgrid[-R:R + 1, -R:R + 1]
    w = np.maximum(0.0, r_el - np.hypot(x, y))
    return w


class _Filter:
    def __init__(self, r_el: float, shape: Tuple[int, int]):
        self.w = _cone_kernel(r_el)
        self.Hs = ndi.convolve(np.ones(shape), self.w, mode="constant")

    def forward(self, x: np.ndarray) -> np.ndarray:
        return ndi.convolve(x, self.w, mode="constant") / self.Hs

    def backward(self, d: np.ndarray) -> np.ndarray:
        return ndi.convolve(d / self.Hs, self.w, mode="constant")


def _project(xf: np.ndarray, beta: float, eta: float = 0.5
             ) -> Tuple[np.ndarray, np.ndarray]:
    den = math.tanh(beta * eta) + math.tanh(beta * (1.0 - eta))
    xb = (math.tanh(beta * eta) + np.tanh(beta * (xf - eta))) / den
    dxb = beta * (1.0 - np.tanh(beta * (xf - eta)) ** 2) / den
    return xb, dxb


def _disk(r_px: int) -> np.ndarray:
    r_px = max(1, int(r_px))
    y, x = np.mgrid[-r_px:r_px + 1, -r_px:r_px + 1]
    return (x * x + y * y) <= r_px * r_px + 1e-9


# --------------------------------------------------------------------------- #
#  The fabrication audit — a measurement, not a hope
# --------------------------------------------------------------------------- #
def fabrication_audit(solid: np.ndarray, h_mm: float,
                      limits: FabricationLimits,
                      haz: Optional[np.ndarray] = None,
                      protect: Optional[np.ndarray] = None) -> dict:
    """
    Morphological-opening length-scale check of a binary shape.

    A rib is buildable iff a disk of diameter min_rib fits through it: opening
    with that disk erases everything thinner, and any solid pixel the opening
    fails to recover is a violation. The HAZ band repeats the test with its
    stiffer disk. `protect` pixels (the bore bosses — enforced ≥ min_rib at
    domain validation, so they cannot legitimately fail) are excluded.

    Material flush with the plate boundary is judged by its IN-PLATE width
    (the plate edge is not free surface for a weld foot, and the sheet
    continues past the blank in the shop): the opening runs on an edge-
    replicated pad, so a boundary-touching strip is not falsely erased.
    """
    solid = solid.astype(bool)
    protect = np.zeros_like(solid) if protect is None else protect.astype(bool)
    n_solid = int(solid.sum())
    if n_solid == 0:
        return {"ok": False, "empty": True, "viol_frac": 1.0,
                "viol_haz_frac": 0.0, "n_viol": 0, "n_viol_haz": 0,
                "r_bulk_px": 0, "r_haz_px": 0}

    def _open_padded(B: np.ndarray, r_px: int) -> np.ndarray:
        pad = r_px + 1
        P = np.pad(B, pad, mode="edge")
        O = ndi.binary_opening(P, structure=_disk(r_px))
        return O[pad:-pad, pad:-pad]

    # a radius-r pixel disk has diameter (2r+1)·h — take the largest that
    # does NOT exceed the rule (clamped to one cell: a grid coarser than
    # min_rib/3 over-tests, in the conservative direction)
    def _r_for(rule_mm: float) -> int:
        return max(1, int(math.floor((rule_mm / h_mm - 1.0) / 2.0 + 1e-9)))

    r_bulk = _r_for(limits.min_rib_mm)
    opened = _open_padded(solid, r_bulk)
    viol = solid & ~opened & ~protect
    n_viol = int(viol.sum())

    n_viol_haz, r_haz = 0, 0
    if haz is not None and haz.any() and limits.haz_mm > 0:
        r_haz = _r_for(limits.min_rib_haz_mm)
        opened_h = _open_padded(solid, r_haz)
        viol_h = solid & haz.astype(bool) & ~opened_h & ~protect
        n_viol_haz = int(viol_h.sum())
        n_haz_solid = int((solid & haz).sum()) or 1
    else:
        n_haz_solid = 1

    viol_frac = n_viol / n_solid
    viol_haz_frac = n_viol_haz / n_haz_solid
    return {"ok": (viol_frac <= _SLIVER_TOL) and (viol_haz_frac <= _SLIVER_TOL),
            "empty": False,
            "viol_frac": float(viol_frac), "viol_haz_frac": float(viol_haz_frac),
            "n_viol": n_viol, "n_viol_haz": n_viol_haz,
            "r_bulk_px": r_bulk, "r_haz_px": r_haz}


# --------------------------------------------------------------------------- #
#  The result
# --------------------------------------------------------------------------- #
@dataclass
class MorphRound:
    """One growth attempt at one enforced length scale."""
    filter_mm: float
    iterations: int
    compliance: float           # Σ w_k fᵀu at unit-per-case peak loads, N·mm
    volfrac: float              # achieved physical volume fraction (active area)
    audit: dict
    accepted: bool


@dataclass
class MorphResult:
    verdict: str
    findings: List[dict]
    fan: Optional[LoadFan]
    domain_name: str
    domain_meta: dict
    limits: FabricationLimits
    rounds: List[MorphRound]
    density: np.ndarray          # (ny, nx) final physical density
    solid: np.ndarray            # (ny, nx) bool, binarised at 0.5
    mass_g: float
    compliance_history: List[float]
    fos: float                   # worst von Mises FoS across cases (peak loads)
    fos_case_deg: float          # angle of the governing case
    vm_max_MPa: float
    suggested_thickness_mm: Optional[float]
    stress_dir: np.ndarray       # (ny, nx, 2) exposure-weighted principal dir
    stress_mag: np.ndarray       # (ny, nx) its magnitude, MPa
    coarsen_premium: Optional[dict]   # {d_compliance_frac, d_filter_mm} when COARSENED
    n_solves: int
    warnings: List[str] = _dcfield(default_factory=list)

    @property
    def ok(self) -> bool:
        return self.verdict in ("FORGEABLE", "COARSENED")

    def summary(self) -> dict:
        return {
            "verdict": self.verdict, "domain": self.domain_name,
            "limits": self.limits.summary(),
            "fan": self.fan.summary() if self.fan is not None else None,
            "rounds": [{"filter_mm": round(r.filter_mm, 2),
                        "iterations": r.iterations,
                        "compliance": round(r.compliance, 3),
                        "volfrac": round(r.volfrac, 4),
                        "viol_frac": round(r.audit.get("viol_frac", 0.0), 4),
                        "viol_haz_frac": round(r.audit.get("viol_haz_frac", 0.0), 4),
                        "accepted": r.accepted} for r in self.rounds],
            "mass_g": round(self.mass_g, 1),
            "fos": round(self.fos, 3) if np.isfinite(self.fos) else None,
            "fos_case_deg": round(self.fos_case_deg, 1),
            "vm_max_MPa": round(self.vm_max_MPa, 1),
            "suggested_thickness_mm": (round(self.suggested_thickness_mm, 2)
                                       if self.suggested_thickness_mm else None),
            "coarsen_premium": self.coarsen_premium,
            "n_solves": self.n_solves,
            "findings": [dict(f) for f in self.findings],
            "warnings": list(self.warnings),
        }

    def to_json(self) -> str:
        return json.dumps(self.summary(), indent=2)

    # ---- exports ---------------------------------------------------------- #
    def cells_csv(self, h_mm: float) -> str:
        """x_mm,y_mm,density for every non-void cell — the shape as data."""
        ny, nx = self.density.shape
        lines = ["x_mm,y_mm,density"]
        ys, xs = np.nonzero(self.density > 0.05)
        for iy, ix in zip(ys, xs):
            lines.append(f"{(ix + 0.5) * h_mm:.2f},{(iy + 0.5) * h_mm:.2f},"
                         f"{self.density[iy, ix]:.3f}")
        return "\n".join(lines)

    def outline_segments(self, h_mm: float) -> List[Tuple[float, float, float, float]]:
        """Solid/void cell-edge boundary as (x1,y1,x2,y2) mm segments — the
        pixel-exact outline a CAD seat can trace (staircase resolution h)."""
        S = self.solid
        ny, nx = S.shape
        segs: List[Tuple[float, float, float, float]] = []
        P = np.pad(S, 1, constant_values=False)
        for iy in range(ny):
            for ix in range(nx):
                if not S[iy, ix]:
                    continue
                x0, y0 = ix * h_mm, iy * h_mm
                if not P[iy, ix + 1]:       # below
                    segs.append((x0, y0, x0 + h_mm, y0))
                if not P[iy + 2, ix + 1]:   # above
                    segs.append((x0, y0 + h_mm, x0 + h_mm, y0 + h_mm))
                if not P[iy + 1, ix]:       # left
                    segs.append((x0, y0, x0, y0 + h_mm))
                if not P[iy + 1, ix + 2]:   # right
                    segs.append((x0 + h_mm, y0, x0 + h_mm, y0 + h_mm))
        return segs


def _flagged(verdict: str, msg: str, fan=None, dom: Optional[PlateDomain] = None,
             limits: Optional[FabricationLimits] = None,
             warnings: Optional[List[str]] = None) -> MorphResult:
    ny = dom.ny if dom else 1
    nx = dom.nx if dom else 1
    z = np.zeros((ny, nx))
    return MorphResult(
        verdict=verdict,
        findings=[{"check": "run", "severity": "error", "message": msg,
                   "detail": {}}],
        fan=fan, domain_name=(dom.name if dom else ""),
        domain_meta=(asdict(dom) if dom else {}),
        limits=(limits or FabricationLimits()),
        rounds=[], density=z, solid=z.astype(bool), mass_g=0.0,
        compliance_history=[], fos=float("nan"), fos_case_deg=float("nan"),
        vm_max_MPa=float("nan"), suggested_thickness_mm=None,
        stress_dir=np.zeros((ny, nx, 2)), stress_mag=z,
        coarsen_premium=None, n_solves=0, warnings=list(warnings or []))


# --------------------------------------------------------------------------- #
#  The engine
# --------------------------------------------------------------------------- #
def _simp_round(fe: _PlateFE, cases: List[LoadCase], volfrac: float,
                r_mm: float, max_iter: int, betas: Tuple[float, ...],
                tol: float) -> Tuple[np.ndarray, List[float], int]:
    """One SIMP growth at one enforced length scale. Returns the physical
    density field, the compliance history, and the FE solve count."""
    ny, nx = fe.ny, fe.nx
    filt = _Filter(max(r_mm / fe.h, 1.001), (ny, nx))
    x = np.full((ny, nx), volfrac)
    x[fe.pass_void] = 0.0
    x[fe.pass_solid] = 1.0
    n_active = int(fe.active.sum()) or 1
    hist: List[float] = []
    n_solves = 0
    p = 3.0

    def physical(xd: np.ndarray, beta: float):
        xf = filt.forward(xd)
        xb, dxb = _project(xf, beta)
        xb = xb.copy()
        xb[fe.pass_void] = 0.0
        xb[fe.pass_solid] = 1.0
        return xb, dxb

    it_total = 0
    for beta in betas:
        for _ in range(max_iter):
            it_total += 1
            xb, dxb = physical(x, beta)
            Us, cs = fe.solve_cases(xb, cases)
            n_solves += len(cases)
            c = float(sum(w.weight * ck for w, ck in zip(cases, cs)))
            hist.append(c)

            # sensitivities on the physical field, chained back to the design
            dc_phys = np.zeros(ny * nx)
            for case, U in zip(cases, Us):
                ue = U[fe.edof]
                ce = np.einsum("ij,jk,ik->i", ue, fe.KE, ue)
                dc_phys += case.weight * ce
            t = fe.dom.thickness_mm
            dc_phys *= -p * (xb.ravel() ** (p - 1.0)) * (fe.E0 - fe.Emin) * t
            dc_phys = dc_phys.reshape(ny, nx)
            dc = filt.backward(dc_phys * dxb)
            dv = filt.backward(np.ones((ny, nx)) * dxb)
            dc[~fe.active] = 0.0

            # optimality criteria with bisection on the PHYSICAL volume
            l1, l2, move = 1e-9, 1e9, 0.2
            xnew = x
            while (l2 - l1) / (l1 + l2) > 1e-3:
                lam = 0.5 * (l1 + l2)
                step = x * np.sqrt(np.maximum(-dc, 0.0)
                                   / np.maximum(lam * np.maximum(dv, 1e-12), 1e-30))
                xnew = np.clip(np.clip(step, x - move, x + move), 0.0, 1.0)
                xnew[fe.pass_void] = 0.0
                xnew[fe.pass_solid] = 1.0
                xb_try, _ = physical(xnew, beta)
                if float(xb_try[fe.active].sum()) / n_active > volfrac:
                    l1 = lam
                else:
                    l2 = lam
            change = float(np.max(np.abs(xnew - x)))
            x = xnew
            if change < tol:
                break
    xb, _ = physical(x, betas[-1])
    return xb, hist, n_solves


def morph_component(dom: PlateDomain, cases: List[LoadCase],
                    limits: FabricationLimits, volfrac: float = 0.4,
                    fan: Optional[LoadFan] = None,
                    max_iter: int = 30,
                    betas: Tuple[float, ...] = (1.0, 2.0, 4.0, 8.0),
                    tol: float = 0.01, max_rounds: int = 4) -> MorphResult:
    """
    Grow the manufacturing-constrained topology. The whole pipeline:
    enforce → grow → binarise → MEASURE → accept or reject-and-coarsen.
    """
    warnings: List[str] = list(fan.warnings) if fan is not None else []
    findings: List[dict] = []

    if not cases:
        return _flagged("LOAD_STARVED",
                        "no load cases — the fan is empty; nothing to grow.",
                        fan, dom, limits, warnings)
    peak = max(c.F_N for c in cases)
    if peak < 1.0:
        return _flagged("LOAD_STARVED",
                        f"peak case amplitude {peak:.2f} N is negligible.",
                        fan, dom, limits, warnings)

    # boss annulus must itself be buildable — grow it, and say so
    if dom.ring_mm < limits.min_rib_mm:
        findings.append({
            "check": "domain", "severity": "info",
            "message": (f"bore boss ring grown {dom.ring_mm:.1f} → "
                        f"{limits.min_rib_mm:.1f} mm to meet the declared "
                        f"minimum rib ({limits.process})."),
            "detail": {"ring_mm": dom.ring_mm,
                       "min_rib_mm": limits.min_rib_mm}})
        dom = PlateDomain(**{**asdict(dom), "ring_mm": limits.min_rib_mm})

    try:
        fe = _PlateFE(dom)
    except Exception as e:                                     # noqa: BLE001
        return _flagged("SOLVER_LIMITED", f"domain construction failed: {e}",
                        fan, dom, limits, warnings)
    if fe.free.size == 0 or fe.fixed_dofs.size == 0:
        return _flagged("SOLVER_LIMITED",
                        "anchor leaves no free/fixed dof split — check the "
                        "anchor declaration.", fan, dom, limits, warnings)

    haz = fe.haz_mask(limits.haz_mm)
    protect = fe.pass_solid.copy()

    # length scale enforced from round 1: filter radius ≥ half the minimum rib
    r_mm = max(limits.min_rib_mm / 2.0, 1.5 * dom.h_mm)
    rounds: List[MorphRound] = []
    hist_all: List[float] = []
    n_solves = 0
    density = None

    for rnd in range(max_rounds):
        try:
            xb, hist, ns = _simp_round(fe, cases, volfrac, r_mm,
                                       max_iter, betas, tol)
        except Exception as e:                                 # noqa: BLE001
            return _flagged("SOLVER_LIMITED", f"FE/optimiser failed: {e}",
                            fan, dom, limits, warnings)
        n_solves += ns
        hist_all.extend(hist)
        solid = xb > 0.5
        audit = fabrication_audit(solid, dom.h_mm, limits, haz, protect)
        vol = float(xb[fe.active].sum()) / (int(fe.active.sum()) or 1)
        accepted = bool(audit["ok"])
        rounds.append(MorphRound(filter_mm=r_mm, iterations=len(hist),
                                 compliance=(hist[-1] if hist else float("nan")),
                                 volfrac=vol, audit=audit, accepted=accepted))
        density = xb
        if accepted:
            break
        findings.append({
            "check": "fabrication", "severity": "warn",
            "message": (f"round {rnd + 1} REJECTED: "
                        f"{audit['viol_frac']*100:.1f}% of the shape is thinner "
                        f"than the {limits.min_rib_mm:.1f} mm rib the shop can "
                        f"hold"
                        + (f", {audit['viol_haz_frac']*100:.1f}% of the weld "
                           f"zone under the {limits.min_rib_haz_mm:.1f} mm HAZ "
                           "rule" if audit["viol_haz_frac"] > _SLIVER_TOL else "")
                        + " — re-growing coarser."),
            "detail": dict(audit, filter_mm=r_mm)})
        r_mm *= 1.4

    solid = density > 0.5
    final = rounds[-1]

    # ---- the verdict ------------------------------------------------------ #
    if not final.accepted:
        limiter = ("the weld heat-affected zone"
                   if final.audit["viol_haz_frac"] > final.audit["viol_frac"]
                   else "the bulk rib width")
        findings.append({
            "check": "fabrication", "severity": "error",
            "message": (f"no shape within the {volfrac*100:.0f}% volume budget "
                        f"satisfies the declared {limits.process} limits — "
                        f"{limiter} is the binding constraint. Widen the "
                        "budget, thicken the plate, or improve the process "
                        "class."),
            "detail": final.audit})
        verdict = "UNBUILDABLE"
    elif len(rounds) > 1:
        verdict = "COARSENED"
    else:
        verdict = "FORGEABLE"

    coarsen_premium = None
    if verdict == "COARSENED" and np.isfinite(rounds[0].compliance) \
            and rounds[0].compliance > 0:
        dfrac = (final.compliance - rounds[0].compliance) / rounds[0].compliance
        coarsen_premium = {"d_compliance_frac": round(float(dfrac), 4),
                           "d_filter_mm": round(final.filter_mm
                                                - rounds[0].filter_mm, 2)}
        findings.append({
            "check": "fabrication", "severity": "info",
            "message": (f"the shop-buildable shape is "
                        f"{abs(dfrac)*100:.1f}% "
                        + ("more" if dfrac > 0 else "less")
                        + " compliant than the first (rejected) shape — the "
                          "stiffness premium paid for manufacturability."),
            "detail": coarsen_premium})

    # ---- structural screen on the finished shape --------------------------- #
    fos = float("inf")
    fos_deg = float("nan")
    vm_max = 0.0
    sdir = np.zeros((fe.ny, fe.nx, 2))
    smag = np.zeros((fe.ny, fe.nx))
    suggested_t = None
    if verdict in ("FORGEABLE", "COARSENED", "UNBUILDABLE") and solid.any():
        try:
            Us, _ = fe.solve_cases(solid.astype(float), cases)
            n_solves += len(cases)
            sig_acc = np.zeros((fe.ny * fe.nx, 3))
            for case, U in zip(cases, Us):
                vm = fe.von_mises(U, solid)
                m = float(vm.max())
                if m > 0:
                    f = dom.yield_MPa / m
                    if f < fos:
                        fos, fos_deg, vm_max = f, case.angle_deg, m
                sig_acc += case.weight * fe.stress_tensor(U)
            # exposure-weighted principal direction field — the stress-path map
            sxx, syy, sxy = sig_acc[:, 0], sig_acc[:, 1], sig_acc[:, 2]
            ang = 0.5 * np.arctan2(2.0 * sxy, sxx - syy)
            s1 = 0.5 * (sxx + syy) + np.sqrt(
                (0.5 * (sxx - syy)) ** 2 + sxy ** 2)
            sdir = np.stack([np.cos(ang), np.sin(ang)],
                            axis=1).reshape(fe.ny, fe.nx, 2)
            smag = np.abs(s1).reshape(fe.ny, fe.nx) * solid
            if np.isfinite(fos) and fos < _FOS_RULE:
                suggested_t = dom.thickness_mm * _FOS_RULE / fos
                findings.append({
                    "check": "structure", "severity": "warn",
                    "message": (f"worst von Mises FoS {fos:.2f} < {_FOS_RULE} "
                                f"(case at {fos_deg:.0f}°, "
                                f"{vm_max:.0f} MPa). Plane-stress FoS is "
                                f"linear in thickness: {suggested_t:.1f} mm "
                                "plate restores the rule."),
                    "detail": {"fos": fos, "vm_max_MPa": vm_max,
                               "suggested_thickness_mm": suggested_t}})
        except Exception as e:                                 # noqa: BLE001
            warnings.append(f"structural screen failed on the final shape: {e}")

    mass_g = float(solid.sum()) * (dom.h_mm ** 2) * dom.thickness_mm \
        * fe.rho * 1e-6  # mm³ · kg/m³ · 1e-9 → kg, ×1e3 → g

    return MorphResult(
        verdict=verdict, findings=findings, fan=fan,
        domain_name=dom.name, domain_meta=asdict(dom), limits=limits,
        rounds=rounds, density=density, solid=solid, mass_g=mass_g,
        compliance_history=hist_all, fos=fos, fos_case_deg=fos_deg,
        vm_max_MPa=vm_max, suggested_thickness_mm=suggested_t,
        stress_dir=sdir, stress_mag=smag, coarsen_premium=coarsen_premium,
        n_solves=n_solves, warnings=warnings)


# --------------------------------------------------------------------------- #
#  The one-call join
# --------------------------------------------------------------------------- #
def morph_from_audit(audit, hp: Hardpoints, member: str = "LF",
                     dom: Optional[PlateDomain] = None,
                     limits: Optional[FabricationLimits] = None,
                     n_cases: int = 4, volfrac: float = 0.4,
                     **kw) -> MorphResult:
    """GhostAudit → load fan → manufacturing-constrained topology."""
    dom = dom or PlateDomain.chassis_tab()
    limits = limits or FabricationLimits.from_shop("hand_weld")
    fan = load_fan_from_audit(audit, hp, member=member, n_cases=n_cases)
    if not fan.ok:
        return _flagged("LOAD_STARVED",
                        "the ghost audit yields no usable load fan for "
                        f"member {member} — " + "; ".join(fan.warnings[:2]),
                        fan, dom, limits, fan.warnings)
    return morph_component(dom, fan.cases, limits, volfrac=volfrac,
                           fan=fan, **kw)


# --------------------------------------------------------------------------- #
#  Markdown report — the sheet that goes to the CAD seat / the FEA queue
# --------------------------------------------------------------------------- #
_VERDICT_LINES = {
    "FORGEABLE": "🟢 FORGEABLE — grown, converged, and buildable at the shop's "
                 "own limits on the first attempt.",
    "COARSENED": "🟡 COARSENED — the unconstrained ribs were REJECTED by the "
                 "declared fabrication limits; a coarser pattern passed. The "
                 "premium is printed below.",
    "UNBUILDABLE": "🔴 UNBUILDABLE — no shape inside the volume budget "
                   "satisfies the declared shop limits.",
    "LOAD_STARVED": "⚪ LOAD_STARVED — the transient audit carries no usable "
                    "force history for this member.",
    "SOLVER_LIMITED": "🔴 SOLVER_LIMITED — the FE solve failed; check the "
                      "anchor and domain declaration.",
}


def render_morph_md(res: MorphResult) -> str:
    s = res.summary()
    L: List[str] = []
    L.append(f"# 🕸️🔩 MorphMesh — {s['domain']}")
    L.append("")
    L.append(_VERDICT_LINES.get(res.verdict, res.verdict))
    L.append("")
    if res.fan is not None and res.fan.cases:
        f = s["fan"]
        L.append(f"## The transient load fan — member {f['member']} "
                 f"({MEMBER_LABELS.get(f['member'], f['member'])})")
        L.append(f"- {f['n_instants']} audited instants, peak "
                 f"{f['peak_F_N']:.0f} N, fan span {f['span_deg']:.0f}°, "
                 f"{f['inplane_share']*100:.0f}% of exposure in the bracket "
                 "plane.")
        L.append("")
        L.append("| case | angle | peak force | exposure | instants | window |")
        L.append("|---|---|---|---|---|---|")
        for i, c in enumerate(f["cases"], 1):
            L.append(f"| {i} | {c['angle_deg']:+.0f}° | {c['F_N']:.0f} N | "
                     f"{c['weight']*100:.0f}% | {c['n_instants']} | "
                     f"{c['t_lo_ms']:.0f}–{c['t_hi_ms']:.0f} ms |")
        L.append("")
    lm = s["limits"]
    L.append("## The shop's declared limits")
    L.append(f"- process **{lm['process']}** — positional accuracy "
             f"±{lm['accuracy_mm']} mm, web floor {lm['web_floor_mm']} mm "
             f"→ minimum rib **{lm['min_rib_mm']} mm**"
             + (f"; HAZ {lm['haz_mm']} mm deep at the weld, minimum rib "
                f"inside it **{lm['min_rib_haz_mm']} mm**."
                if lm["haz_mm"] > 0 else "; no weld zone declared."))
    L.append("")
    L.append("## Growth rounds — enforce, grow, measure, accept/reject")
    L.append("| round | enforced scale | iters | compliance | vol | thin-rib "
             "viol | HAZ viol | outcome |")
    L.append("|---|---|---|---|---|---|---|---|")
    for i, r in enumerate(s["rounds"], 1):
        L.append(f"| {i} | {r['filter_mm']:.1f} mm | {r['iterations']} | "
                 f"{r['compliance']:.2f} | {r['volfrac']*100:.0f}% | "
                 f"{r['viol_frac']*100:.1f}% | {r['viol_haz_frac']*100:.1f}% | "
                 f"{'ACCEPTED' if r['accepted'] else 'REJECTED'} |")
    L.append("")
    if res.coarsen_premium:
        L.append(f"**Manufacturability premium:** the buildable shape is "
                 f"{abs(res.coarsen_premium['d_compliance_frac'])*100:.1f}% "
                 + ("more" if res.coarsen_premium['d_compliance_frac'] > 0
                    else "less")
                 + f" compliant than the first (rejected) shape, at "
                   f"+{res.coarsen_premium['d_filter_mm']:.1f} mm enforced "
                   "scale.")
        L.append("")
    if res.ok:
        L.append("## The finished shape")
        L.append(f"- mass **{s['mass_g']:.0f} g** at "
                 f"{res.domain_meta.get('thickness_mm', 0):.1f} mm "
                 f"{res.domain_meta.get('material', '')}")
        if s["fos"] is not None and np.isfinite(res.fos):
            L.append(f"- worst von Mises FoS **{s['fos']:.2f}** "
                     f"({s['vm_max_MPa']:.0f} MPa, governing case at "
                     f"{s['fos_case_deg']:.0f}°) vs the standing "
                     f"{_FOS_RULE} rule"
                     + (f" — **{s['suggested_thickness_mm']:.1f} mm plate "
                        "restores it**." if s["suggested_thickness_mm"]
                        else "."))
        L.append("")
    if s["findings"]:
        L.append("## Findings")
        for f in s["findings"]:
            L.append(f"- **[{f['severity']}]** {f['message']}")
        L.append("")
    if s["warnings"]:
        L.append("## Warnings")
        for w in s["warnings"]:
            L.append(f"- {w}")
        L.append("")
    L.append("---")
    L.append("*Scope: 2D plane-stress plate, linear elastic, quasi-static; "
             "the load fan inherits Ghost Topology's time-scale separation "
             "whole. The HAZ rule is a screening heuristic, not a welding "
             "simulation. This is a starting shape for the CAD seat and the "
             "FEA queue — validate before manufacturing.*")
    return "\n".join(L)


# --------------------------------------------------------------------------- #
#  Self-test
# --------------------------------------------------------------------------- #
if __name__ == "__main__":                                     # pragma: no cover
    print("MorphMesh self-test: two synthetic arrows on a small tab…")
    dom = PlateDomain.chassis_tab(width_mm=40, height_mm=50, h_mm=2.0,
                                  thickness_mm=4.0)
    cases = [
        LoadCase(np.array([0.35, -0.94]), 2500.0, 0.6, 10, 0.0, 0.4, -70.0),
        LoadCase(np.array([-0.5, -0.87]), 1800.0, 0.4, 8, 0.4, 0.8, -120.0),
    ]
    lim = FabricationLimits.from_shop("hand_weld")
    res = morph_component(dom, cases, lim, volfrac=0.45, max_iter=15)
    print(f"verdict={res.verdict} rounds={len(res.rounds)} "
          f"mass={res.mass_g:.0f} g fos={res.fos:.2f} solves={res.n_solves}")
    assert res.verdict in VERDICTS
    assert res.solid.any()
    a = fabrication_audit(res.solid, dom.h_mm, lim,
                          _PlateFE(dom).haz_mask(lim.haz_mm))
    print("audit:", {k: a[k] for k in ("ok", "viol_frac", "viol_haz_frac")})
    print(render_morph_md(res).splitlines()[2])
    print("OK.")
