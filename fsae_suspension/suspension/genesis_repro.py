# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  suspension/genesis_repro.py — the reproducibility layer for InverseGenesis.
# ============================================================================
"""
Everything an InverseGenesis result needs to be re-run byte for byte, and the
diagnostics a design report built on it needs but the engine does not
compute.

WHAT LIVES HERE
---------------
* ``GenesisManifest`` — every input of a corner run (geometry at full
  precision, static alignment, targets, legal boxes, spacing constraints,
  keep-outs, tolerance field, seed, sample counts, solver constants) as one
  JSON document, plus the outputs it produced. ``run()`` re-executes it,
  ``verify()`` checks a re-run against the recorded outputs, and
  ``inputs_sha256`` fingerprints the inputs so two people can tell whether
  they ran the same thing.
* ``cad_to_corner`` / ``corner_to_cad`` — the CAD (X lateral, Y up, Z
  forward) ↔ corner (x rearward, y outboard, z up from ground) transform,
  with the axle station and ground plane as explicit inputs.
* ``CapsuleObstacle`` / ``capsules_from_framegraph`` — frame tubes as
  keep-out capsules in the same ``clearances(points, probe)`` dialect the
  boundary filter already speaks.
* ``corner_diagnostics`` — the properties that are NOT channels: slopes,
  toe range, RC migration (chassis and above-ground), caster, KPI, scrub,
  contact-patch rise, side-view instant centre, anti-dive / anti-squat.
* ``camber_to_road`` — loaded-tyre camber relative to the road at a roll
  angle, and the gain that would hold a stated optimum.
* ``yield_breakdown`` / ``discordant_builds`` — per-channel failure
  fractions, the first-order worst case W, the zero-failure bound, and the
  paired comparison between two candidates.
* ``swept_clearance`` — every link against every obstacle capsule across
  travel, with the inboard-pickup exclusion.

All of it is deterministic. Nothing here changes what the engine computes;
it records it, re-runs it and measures what the engine does not.
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass, field as _dcfield
from typing import Any

import numpy as np

from .kinematics import Hardpoints, SuspensionKinematics
from .kinematik_stochastic import ToleranceField, ToleranceSpec, _perturbed
from . import inverse_genesis as ig

MANIFEST_SCHEMA = "kinematik.genesis.manifest/1"

_HP_POINTS = ("upper_front_inner", "upper_rear_inner", "lower_front_inner",
              "lower_rear_inner", "upper_outer", "lower_outer",
              "tie_rod_inner", "tie_rod_outer", "wheel_center",
              "contact_patch")
_HP_OPTIONAL = ("pushrod_outer", "rocker_pivot", "rocker_axis",
                "rocker_pushrod", "rocker_spring", "spring_inner")


def _kinematik_version() -> str:
    try:
        from . import __version__
        return str(__version__)
    except Exception:           # noqa: BLE001
        return "unknown"


# --------------------------------------------------------------------------- #
#  Frames
# --------------------------------------------------------------------------- #
def cad_to_corner(p_cad, axle_station_z: float, ground_y: float) -> np.ndarray:
    """CAD (X lateral +right, Y up, Z forward) → corner (x rearward,
    All coordinates in mm.
    y = X, z up from the ground plane). Determinant −1 by construction."""
    p = np.asarray(p_cad, float)
    out = np.empty_like(p)
    out[..., 0] = -(p[..., 2] - axle_station_z)
    out[..., 1] = p[..., 0]
    out[..., 2] = p[..., 1] - ground_y
    return out


def corner_to_cad(p_corner, axle_station_z: float, ground_y: float
                  ) -> np.ndarray:
    """Corner (x rearward, y outboard, z up from ground, mm) → CAD frame (mm); inverse of ``cad_to_corner``."""
    p = np.asarray(p_corner, float)
    out = np.empty_like(p)
    out[..., 0] = p[..., 1]
    out[..., 1] = p[..., 2] + ground_y
    out[..., 2] = axle_station_z - p[..., 0]
    return out


# --------------------------------------------------------------------------- #
#  Geometry helpers
# --------------------------------------------------------------------------- #
def _pt_seg_dist(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Distance from points p (n,3) to every segment a→b (m,3) → (n,m)."""
    ab = b - a                                       # (m,3)
    L2 = np.maximum(np.einsum("ij,ij->i", ab, ab), 1e-18)
    ap = p[:, None, :] - a[None, :, :]               # (n,m,3)
    t = np.clip(np.einsum("nmj,mj->nm", ap, ab) / L2, 0.0, 1.0)
    closest = a[None] + t[..., None] * ab[None]
    return np.linalg.norm(p[:, None, :] - closest, axis=2)


def seg_seg_dist(p1, q1, p2, q2) -> float:
    """Closest distance between segments p1q1 and p2q2 (Ericson §5.1.9). Distance in the input's length units (mm throughout KinematiK)."""
    p1, q1, p2, q2 = (np.asarray(v, float) for v in (p1, q1, p2, q2))
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = d1 @ d1, d2 @ d2, d2 @ r
    eps = 1e-12
    if a <= eps and e <= eps:
        return float(np.linalg.norm(p1 - p2))
    if a <= eps:
        s, t = 0.0, float(np.clip(f / e, 0.0, 1.0))
    else:
        c = d1 @ r
        if e <= eps:
            t, s = 0.0, float(np.clip(-c / a, 0.0, 1.0))
        else:
            b = d1 @ d2
            den = a * e - b * b
            s = float(np.clip((b * f - c * e) / den, 0.0, 1.0)) \
                if den > eps else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t, s = 0.0, float(np.clip(-c / a, 0.0, 1.0))
            elif t > 1.0:
                t, s = 1.0, float(np.clip((b - c) / a, 0.0, 1.0))
    return float(np.linalg.norm((p1 + d1 * s) - (p2 + d2 * t)))


# --------------------------------------------------------------------------- #
#  Frame tubes as keep-out capsules
# --------------------------------------------------------------------------- #
@dataclass
class CapsuleObstacle:
    """A set of capsules (segment + radius), corner frame, mm.

    Speaks the boundary filter's ``clearances(points, probe_radius_mm)``
    dialect: signed skin clearance to the nearest capsule, + clear.
    """
    a: np.ndarray                  # (m,3)
    b: np.ndarray                  # (m,3)
    radius: np.ndarray             # (m,)
    names: list[str] = _dcfield(default_factory=list)
    label: str = "frame tubes"

    def __post_init__(self):
        self.a = np.asarray(self.a, float).reshape(-1, 3)
        self.b = np.asarray(self.b, float).reshape(-1, 3)
        self.radius = np.broadcast_to(
            np.asarray(self.radius, float), (len(self.a),)).copy()
        if len(self.a) != len(self.b):
            raise ValueError("CapsuleObstacle: a and b differ in length.")
        if not self.names:
            self.names = [f"tube{i}" for i in range(len(self.a))]

    def clearances(self, points, probe_radius_mm: float = 0.0) -> np.ndarray:
        """Signed skin clearance (mm) from each point to the nearest capsule, minus the probe radius (mm); + is clear."""
        pts = np.asarray(points, float).reshape(-1, 3)
        if len(self.a) == 0:
            return np.full(len(pts), np.inf)
        d = _pt_seg_dist(pts, self.a, self.b) - self.radius[None, :]
        return d.min(axis=1) - probe_radius_mm

    def to_dict(self) -> dict:
        """Capsule set → dict; endpoints and radii in mm, corner frame."""
        return {"type": "capsules", "label": self.label,
                "names": list(self.names),
                "a": self.a.tolist(), "b": self.b.tolist(),
                "radius": self.radius.tolist()}


def capsules_from_framegraph(fg, axle_station_z: float, ground_y: float,
                             label: str = "frame tubes",
                             default_od_mm: float = 25.4
                             ) -> CapsuleObstacle:
    """FrameGraph (object or its ``as_dict``) in CAD axes → corner capsules.

    Node coordinates are taken as CAD (X lateral, Y up, Z forward), the
    convention of the STEP extraction; tube radius from the size table OD.
    """
    from .tubeframe import FrameGraph
    if isinstance(fg, dict):
        fg = FrameGraph.from_dict(fg)
    a, b, r, names = [], [], [], []
    for t in fg.tubes:
        a.append(cad_to_corner(fg.p(t.a), axle_station_z, ground_y))
        b.append(cad_to_corner(fg.p(t.b), axle_station_z, ground_y))
        try:
            od = float(fg.spec_of(t).od_mm)
        except Exception:        # noqa: BLE001
            od = default_od_mm
        r.append(0.5 * od)
        names.append(t.name)
    return CapsuleObstacle(np.array(a).reshape(-1, 3),
                           np.array(b).reshape(-1, 3),
                           np.array(r), names, label)


# --------------------------------------------------------------------------- #
#  (De)serialisation of the engine's inputs
# --------------------------------------------------------------------------- #
def hp_to_dict(hp: Hardpoints) -> dict:
    """Hardpoints → JSON-safe dict; coordinates in mm, static camber/toe in deg."""
    d: dict[str, Any] = {p: [float(v) for v in np.asarray(getattr(hp, p))]
                         for p in _HP_POINTS}
    for p in _HP_OPTIONAL:
        v = getattr(hp, p, None)
        if v is not None:
            d[p] = [float(x) for x in np.asarray(v)]
    d["pushrod_attach"] = getattr(hp, "pushrod_attach", "lower")
    d["static_camber"] = float(hp.static_camber)
    d["static_toe"] = float(hp.static_toe)
    return d


def hp_from_dict(d: dict) -> Hardpoints:
    """JSON dict → Hardpoints; coordinates in mm, static camber/toe in deg."""
    kw: dict[str, Any] = {}
    for k, v in d.items():
        if k in _HP_POINTS or k in _HP_OPTIONAL:
            kw[k] = None if v is None else np.asarray(v, float)
        elif k in ("static_camber", "static_toe"):
            kw[k] = float(v)
        elif k == "pushrod_attach":
            kw[k] = str(v)
    missing = [p for p in _HP_POINTS if p not in kw]
    if missing:
        raise ValueError(f"Hardpoints missing: {', '.join(missing)}")
    return Hardpoints(**kw)


def targets_to_dict(t: ig.GenesisTargets) -> dict:
    """GenesisTargets → dict; travel in mm, targets and bands in each channel's own units (deg or mm)."""
    return {"track_mm": float(t.track_mm),
            "curves": [{"channel": c.channel,
                        "travel_mm": c.travel_mm.tolist(),
                        "target": c.target.tolist(),
                        "band": c.band.tolist()} for c in t.curves]}


def targets_from_dict(d: dict) -> ig.GenesisTargets:
    """dict → GenesisTargets; travel in mm, targets and bands in each channel's units (deg or mm)."""
    return ig.GenesisTargets(
        curves=[ig.TargetCurve(c["channel"], c["travel_mm"], c["target"],
                               c["band"]) for c in d["curves"]],
        track_mm=float(d.get("track_mm", 1200.0)))


def linear_targets(stations, *, static_camber=None, camber_gain=None,
                   camber_band=0.30, toe=None, toe_band=0.08,
                   rc_height=None, rc_band=18.0, scrub=None, scrub_band=3.0,
                   track_mm: float = 1200.0) -> ig.GenesisTargets:
    """Targets written the way a design brief states them (deg, mm): static + gain·t for
    camber, constants for toe / RC height / scrub. Omit a channel with None."""
    st = np.asarray(stations, float)
    n = len(st)
    curves = []
    if camber_gain is not None:
        curves.append(ig.TargetCurve(
            "camber_deg", st, float(static_camber) + float(camber_gain) * st,
            np.full(n, camber_band)))
    if toe is not None:
        curves.append(ig.TargetCurve("toe_deg", st, np.full(n, float(toe)),
                                     np.full(n, toe_band)))
    if rc_height is not None:
        curves.append(ig.TargetCurve("rc_height_mm", st,
                                     np.full(n, float(rc_height)),
                                     np.full(n, rc_band)))
    if scrub is not None:
        curves.append(ig.TargetCurve("scrub_mm", st, np.full(n, float(scrub)),
                                     np.full(n, scrub_band)))
    return ig.GenesisTargets(curves=curves, track_mm=track_mm)


def _obstacle_to_dict(o) -> dict:
    if isinstance(o, ig.KeepOutBox):
        return {"type": "box", "label": o.label,
                "lo": o.lo.tolist(), "hi": o.hi.tolist()}
    if isinstance(o, CapsuleObstacle):
        return o.to_dict()
    raise TypeError(f"Keep-out of type {type(o).__name__} cannot be written "
                    "to a manifest; convert it to boxes or capsules first.")


def _obstacle_from_dict(d: dict):
    if d["type"] == "box":
        return ig.KeepOutBox(np.asarray(d["lo"]), np.asarray(d["hi"]),
                             label=d.get("label", "keep-out box"))
    if d["type"] == "capsules":
        return CapsuleObstacle(d["a"], d["b"], d["radius"],
                               d.get("names", []), d.get("label", "capsules"))
    raise ValueError(f"Unknown keep-out type '{d['type']}'.")


def _elasto_from_dict(d: dict | None):
    """dict -> elastokinematics.ElastoSpec (stiffness N/mm, forces N), or None."""
    if not d:
        return None
    from . import elastokinematics as _ek
    return _ek.ElastoSpec.from_dict(d)


def _property_bounds_to_dict(b: ig.SolvedPropertyBounds | None) -> dict | None:
    """SolvedPropertyBounds → dict; infinities as JSON null, lengths in mm.

    A one-sided bound carries a real infinity, which JSON cannot hold, so an
    absent side is written as null and read back as the infinity it was.
    """
    if b is None or not b.bounds:
        return None

    def _side(x: float):
        return None if not np.isfinite(x) else float(x)

    return {"bounds": [{"prop": p.prop, "lo": _side(p.lo), "hi": _side(p.hi),
                        "label": p.label} for p in b.bounds],
            "cg_height_mm": float(b.cg_height_mm),
            "wheelbase_mm": float(b.wheelbase_mm),
            "track_mm": float(b.track_mm),
            "brake_bias_front": float(b.brake_bias_front),
            "drive_bias_rear": float(b.drive_bias_rear),
            "travel_mm": [float(b.travel_mm[0]), float(b.travel_mm[1])],
            "n_nodes": int(b.n_nodes),
            **({"elasto": b.elasto.to_dict()}
               if getattr(b, "elasto", None) is not None else {})}


def _property_bounds_from_dict(d: dict | None) -> ig.SolvedPropertyBounds | None:
    """dict → SolvedPropertyBounds; null sides become infinities, lengths in mm."""
    if not d:
        return None
    return ig.SolvedPropertyBounds(
        bounds=[ig.PropertyBound(
            b["prop"],
            lo=float("-inf") if b.get("lo") is None else float(b["lo"]),
            hi=float("inf") if b.get("hi") is None else float(b["hi"]))
            for b in d.get("bounds", [])],
        cg_height_mm=float(d.get("cg_height_mm", 280.0)),
        wheelbase_mm=float(d.get("wheelbase_mm", 1630.0)),
        track_mm=float(d.get("track_mm", 1200.0)),
        brake_bias_front=float(d.get("brake_bias_front", 0.60)),
        drive_bias_rear=float(d.get("drive_bias_rear", 1.0)),
        travel_mm=tuple(d.get("travel_mm", (-25.0, 25.0))),
        n_nodes=int(d.get("n_nodes", 5)),
        elasto=_elasto_from_dict(d.get("elasto")))


def _envelope_to_dict(e: "ig.WheelEnvelope | None") -> dict | None:
    """WheelEnvelope → dict; lengths in mm."""
    if e is None:
        return None
    return {"pickup_radius_mm": float(e.pickup_radius_mm),
            "rim_radius_mm": float(e.rim_radius_mm),
            "tire_radius_mm": float(e.tire_radius_mm),
            "rim_half_width_mm": float(e.rim_half_width_mm),
            "tire_half_width_mm": float(e.tire_half_width_mm),
            "wheel_offset_mm": float(e.wheel_offset_mm),
            "clearance_mm": float(e.clearance_mm),
            "travel_mm": [float(e.travel_mm[0]), float(e.travel_mm[1])],
            "n_travel": int(e.n_travel),
            "rack_travel_mm": float(e.rack_travel_mm),
            "samples_per_link": int(e.samples_per_link),
            "links": list(e.links),
            "link_radius_mm": float(e.link_radius_mm),
            "rod_end_radius_mm": float(e.rod_end_radius_mm),
            "tire_profile": [[float(r), float(w)] for r, w in e.tire_profile],
            "joint_swing_limit_deg": e.joint_swing_limit_deg,
            "sectors": [{"label": q.label, "r_min_mm": q.r_min_mm,
                         "r_max_mm": q.r_max_mm,
                         "theta_min_deg": q.theta_min_deg,
                         "theta_max_deg": q.theta_max_deg,
                         "axial_min_mm": q.axial_min_mm,
                         "axial_max_mm": q.axial_max_mm}
                        for q in e.sectors]}


def _envelope_from_dict(d: dict | None) -> "ig.WheelEnvelope | None":
    """dict → WheelEnvelope; lengths in mm."""
    if not d:
        return None
    d = dict(d)
    d["travel_mm"] = tuple(d.get("travel_mm", (-25.0, 25.0)))
    d["links"] = tuple(d.get("links", ig.WheelEnvelope().links))
    # A manifest written before links had thickness recorded no radius, and
    # meant thin lines: replay it as written rather than under new defaults.
    d.setdefault("link_radius_mm", 0.0)
    d.setdefault("rod_end_radius_mm", 0.0)
    d["tire_profile"] = tuple(tuple(p) for p in d.get("tire_profile", ()))
    d["sectors"] = tuple(ig.WheelSector(**q) for q in d.get("sectors", ()))
    return ig.WheelEnvelope(**d)


def nodes_from_framegraph(fg, axle_station_z: float, ground_y: float
                          ) -> tuple[tuple[float, float, float], ...]:
    """FrameGraph nodes (CAD axes, mm) → corner-frame node coordinates, mm."""
    from .tubeframe import FrameGraph
    if isinstance(fg, dict):
        fg = FrameGraph.from_dict(fg)
    return tuple(tuple(float(c) for c in cad_to_corner(
        np.asarray(n.xyz_mm, float), axle_station_z, ground_y))
        for n in fg.nodes.values())


def volume_to_dict(v: ig.LegalVolume) -> dict:
    """LegalVolume → dict; box bounds, probe radius, clearance and spacing gaps in mm.

    ``properties`` is written only when solved-property bounds are declared,
    so a manifest from an unbounded run keeps the bytes, and therefore the
    inputs hash, it has always had.
    """
    out = {"boxes": {p: {"lo": lo.tolist(), "hi": hi.tolist()}
                     for p, (lo, hi) in sorted(v.boxes.items())},
           "keep_out": [_obstacle_to_dict(o) for o in v.keep_out],
           "probe_radius_mm": float(v.probe_radius_mm),
           "min_clearance_mm": float(v.min_clearance_mm),
           "spacings": [{"a": s.a, "b": s.b, "axis": s.axis,
                         "min_gap_mm": s.min_gap_mm, "label": s.label}
                        for s in v.spacings]}
    props = _property_bounds_to_dict(getattr(v, "properties", None))
    if props is not None:
        out["properties"] = props
    env = _envelope_to_dict(getattr(v, "wheel_envelope", None))
    if env is not None:
        out["wheel_envelope"] = env
    na = getattr(v, "node_attachment", None)
    if na is not None:
        out["node_attachment"] = {"nodes": [list(n) for n in na.nodes],
                                  "points": list(na.points),
                                  "max_offset_mm": float(na.max_offset_mm)}
    return out


def volume_from_dict(d: dict) -> ig.LegalVolume:
    """dict → LegalVolume; box bounds, probe radius, clearance and spacing gaps in mm."""
    return ig.LegalVolume(
        boxes={p: (np.asarray(b["lo"]), np.asarray(b["hi"]))
               for p, b in d["boxes"].items()},
        keep_out=[_obstacle_from_dict(o) for o in d.get("keep_out", [])],
        probe_radius_mm=float(d.get("probe_radius_mm", 0.0)),
        min_clearance_mm=float(d.get("min_clearance_mm", 0.0)),
        spacings=[ig.PointSpacing(**s) for s in d.get("spacings", [])],
        properties=_property_bounds_from_dict(d.get("properties")),
        wheel_envelope=_envelope_from_dict(d.get("wheel_envelope")),
        node_attachment=(ig.NodeAttachment(
            nodes=tuple(tuple(n) for n in d["node_attachment"]["nodes"]),
            points=tuple(d["node_attachment"]["points"]),
            max_offset_mm=float(d["node_attachment"]["max_offset_mm"]))
            if d.get("node_attachment") else None))


def boxes_about(hp: Hardpoints, half: dict[str, Any]
                ) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """Absolute boxes from per-point half-widths; a half-width may be a
    scalar or a 3-vector (use ``math.inf`` for a free axis, which is
    stored as ±1e6 mm so JSON stays finite)."""
    out = {}
    for p, h in half.items():
        h = np.broadcast_to(np.asarray(h, float), (3,)).copy()
        h[~np.isfinite(h)] = 1e6
        c = np.asarray(getattr(hp, p), float)
        out[p] = (c - h, c + h)
    return out


def field_to_dict(f: ToleranceField | None) -> dict | None:
    """ToleranceField → dict; per-axis lo/hi build tolerances in mm."""
    if f is None:
        return None
    return {"provenance": f.provenance, "calibrated": bool(f.calibrated),
            "specs": {p: {"lo": s.lo.tolist(), "hi": s.hi.tolist(),
                          "dist": s.dist}
                      for p, s in sorted(f.specs.items())}}


def field_from_dict(d: dict | None) -> ToleranceField | None:
    """dict → ToleranceField; per-axis lo/hi build tolerances in mm."""
    if d is None:
        return None
    return ToleranceField(
        {p: ToleranceSpec(np.asarray(s["lo"]), np.asarray(s["hi"]),
                          s.get("dist", "uniform"))
         for p, s in d["specs"].items()},
        provenance=d.get("provenance", "manifest"),
        calibrated=bool(d.get("calibrated", False)))


def field_with_overrides(shop: str, overrides: dict[str, float | tuple]
                         | None = None, **preset_kw) -> ToleranceField:
    """A shop preset with per-point overrides, e.g. put a welded rear
    toe-link tab in the tab class: ``{"tie_rod_inner": 0.5}``. A value may
    be a symmetric half-width or a (lo3, hi3) pair."""
    f = ToleranceField.preset(shop, **preset_kw)
    for p, v in (overrides or {}).items():
        if isinstance(v, (int, float)):
            f.specs[p] = ToleranceSpec.symmetric(float(v))
        else:
            lo, hi = v
            f.specs[p] = ToleranceSpec(np.asarray(lo, float),
                                       np.asarray(hi, float))
    if overrides:
        f.provenance += f" | per-point overrides: {sorted(overrides)}"
    return f


# --------------------------------------------------------------------------- #
#  The manifest
# --------------------------------------------------------------------------- #
@dataclass
class SearchSettings:
    seed: int = 0
    n_starts: int = 6
    n_yield: int = 4000
    n_verify_full: int = 0
    max_iter: int = 30
    step_mm: float = 0.25
    resilient_yield: float = 0.95
    tempered_yield: float = 0.80
    verify_agreement: float = 0.98

    def thresholds(self) -> ig.GenesisThresholds:
        """Verdict thresholds as fractions (dimensionless) of builds passing."""
        return ig.GenesisThresholds(self.resilient_yield,
                                    self.tempered_yield,
                                    self.verify_agreement)


@dataclass
class GenesisManifest:
    name: str
    hardpoints: dict
    targets: dict
    volume: dict
    tolerance: dict | None
    search: SearchSettings = _dcfield(default_factory=SearchSettings)
    #: provenance that does not enter the computation but must travel with it
    context: dict = _dcfield(default_factory=dict)
    #: filled by record(); compared by verify()
    recorded: dict | None = None
    kinematik_version: str = _dcfield(default_factory=_kinematik_version)
    schema: str = MANIFEST_SCHEMA

    # ---- construction ------------------------------------------------------ #
    @staticmethod
    def build(name: str, hp: Hardpoints, targets: ig.GenesisTargets,
              volume: ig.LegalVolume, fld: ToleranceField | None,
              search: SearchSettings | None = None,
              context: dict | None = None) -> GenesisManifest:
        """Assemble a manifest from engine objects; geometry in mm, angles in deg, tolerances in mm."""
        return GenesisManifest(
            name=name, hardpoints=hp_to_dict(hp),
            targets=targets_to_dict(targets), volume=volume_to_dict(volume),
            tolerance=field_to_dict(fld), search=search or SearchSettings(),
            context=dict(context or {}))

    def objects(self):
        """Rebuild (Hardpoints, GenesisTargets, LegalVolume, ToleranceField); all lengths in mm, angles in deg."""
        return (hp_from_dict(self.hardpoints),
                targets_from_dict(self.targets),
                volume_from_dict(self.volume),
                field_from_dict(self.tolerance))

    # ---- identity ---------------------------------------------------------- #
    def _inputs(self) -> dict:
        return {"hardpoints": self.hardpoints, "targets": self.targets,
                "volume": self.volume, "tolerance": self.tolerance,
                "search": self.search.__dict__}

    @property
    def inputs_sha256(self) -> str:
        blob = json.dumps(self._inputs(), sort_keys=True,
                          separators=(",", ":"))
        return hashlib.sha256(blob.encode()).hexdigest()

    # ---- IO ---------------------------------------------------------------- #
    def to_json(self, indent: int = 2) -> str:
        d = {"schema": self.schema, "name": self.name,
             "kinematik_version": self.kinematik_version,
             "inputs_sha256": self.inputs_sha256,
             "context": self.context, **self._inputs(),
             "recorded": self.recorded}
        return json.dumps(d, indent=indent, sort_keys=False)

    @staticmethod
    def from_json(text: str) -> GenesisManifest:
        """Parse a manifest; lengths in mm, angles in deg, yields as dimensionless fractions."""
        d = json.loads(text)
        if d.get("schema") != MANIFEST_SCHEMA:
            raise ValueError(f"Not a genesis manifest (schema "
                             f"{d.get('schema')!r}).")
        m = GenesisManifest(
            name=d["name"], hardpoints=d["hardpoints"],
            targets=d["targets"], volume=d["volume"],
            tolerance=d.get("tolerance"),
            search=SearchSettings(**d.get("search", {})),
            context=d.get("context", {}), recorded=d.get("recorded"),
            kinematik_version=d.get("kinematik_version", "unknown"))
        stored = d.get("inputs_sha256")
        if stored and stored != m.inputs_sha256:
            raise ValueError("Manifest inputs were edited after it was "
                             "written (sha256 mismatch). Remove the "
                             "'inputs_sha256' field to accept the edit.")
        return m

    # ---- execution --------------------------------------------------------- #
    def run(self) -> ig.GenesisResult:
        """Re-execute the manifest; returns candidates with fits in band fractions (dimensionless) and coordinates in mm."""
        hp, tg, vol, fld = self.objects()
        s = self.search
        return ig.inverse_genesis(
            hp, tg, vol, fld=fld, n_starts=s.n_starts, n_yield=s.n_yield,
            n_verify_full=s.n_verify_full, seed=s.seed,
            thresholds=s.thresholds(), max_iter=s.max_iter, step_mm=s.step_mm)

    def summarise(self, res: ig.GenesisResult) -> dict:
        """Outputs as a dict: hardpoints in mm, fit in band fractions and yields as dimensionless fractions."""
        _, _, vol, _ = self.objects()
        cands = []
        for c in res.candidates:
            cands.append({
                "hit": bool(c.hit), "verdict": c.verdict,
                "max_band_frac": float(c.max_band_frac),
                "worst_row": c.worst_row,
                "yield_frac": (None if c.yield_frac is None
                               else float(c.yield_frac)),
                "iterations": int(c.iterations),
                "clamped": list(c.clamped),
                "keepout_rejections": int(c.keepout_rejections),
                "shift_vec": [float(v) for v in c.shift_vec],
            })
        return {
            "ok": bool(res.ok), "reason": res.reason,
            "winner_hardpoints": (None if res.winner_hp is None
                                  else hp_to_dict(res.winner_hp)),
            "winner_verdict": (res.winner.verdict if res.winner else None),
            "winner_yield": (None if res.winner is None
                             or res.winner.yield_frac is None
                             else float(res.winner.yield_frac)),
            "resilience_premium": (None if res.resilience_premium is None
                                   else float(res.resilience_premium)),
            "candidates": cands,
            "coord_labels": vol.coord_labels(),
            "n_starts": int(res.n_starts),
            "inputs_sha256": self.inputs_sha256,
            "kinematik_version": _kinematik_version(),
        }

    def record(self, res: ig.GenesisResult) -> dict:
        """Store ``summarise(res)`` (mm, dimensionless fractions) as the manifest's recorded outputs."""
        self.recorded = self.summarise(res)
        return self.recorded

    def verify(self, res: ig.GenesisResult, tol: float = 1e-9
               ) -> tuple[bool, list[str]]:
        """Compare a re-run against the recorded outputs. Returns (same,
        Compares coordinates to 1e-6 mm and yields/fits (dimensionless) to ``tol``.
        differences)."""
        if not self.recorded:
            return False, ["Manifest has no recorded outputs to compare."]
        now = self.summarise(res)
        diffs: list[str] = []
        a, b = self.recorded, now
        if a.get("inputs_sha256") and a["inputs_sha256"] != b["inputs_sha256"]:
            diffs.append("inputs differ from the ones the outputs were "
                         "recorded with (sha256 mismatch)")
        if a["winner_verdict"] != b["winner_verdict"]:
            diffs.append(f"winner verdict {a['winner_verdict']} → "
                         f"{b['winner_verdict']}")
        if len(a["candidates"]) != len(b["candidates"]):
            diffs.append(f"candidate count {len(a['candidates'])} → "
                         f"{len(b['candidates'])}")
        for i, (ca, cb) in enumerate(zip(a["candidates"], b["candidates"])):
            for k in ("max_band_frac", "yield_frac"):
                va, vb = ca[k], cb[k]
                if (va is None) != (vb is None) or (
                        va is not None and abs(va - vb) > tol):
                    diffs.append(f"candidate {i + 1} {k}: {va} → {vb}")
            sa, sb = np.asarray(ca["shift_vec"]), np.asarray(cb["shift_vec"])
            if sa.shape != sb.shape or np.max(np.abs(sa - sb)) > 1e-6:
                diffs.append(f"candidate {i + 1} coordinates differ")
        wa, wb = a["winner_hardpoints"], b["winner_hardpoints"]
        if (wa is None) != (wb is None):
            diffs.append("winner presence differs")
        elif wa is not None:
            for p in _HP_POINTS:
                if np.max(np.abs(np.asarray(wa[p]) - np.asarray(wb[p]))) > 1e-6:
                    diffs.append(f"winner {p} differs")
        if a.get("kinematik_version") != b.get("kinematik_version"):
            diffs.append(f"(note) recorded on KinematiK "
                         f"{a.get('kinematik_version')}, re-run on "
                         f"{b.get('kinematik_version')}")
        real = [d for d in diffs if not d.startswith("(note)")]
        return (not real), diffs


# --------------------------------------------------------------------------- #
#  Diagnostics the channels do not see
# --------------------------------------------------------------------------- #
def _ic_2d(upper, lower, axes) -> np.ndarray:
    """Instant centre in the plane spanned by ``axes`` (two indices): each
    ball joint moves with v = a × (b − p_f); the IC is where the two lines
    through the joints, perpendicular to the projected velocities, meet."""
    i, j = axes
    rows, rhs = [], []
    for pf, pr, b in (upper, lower):
        v = np.cross(pr - pf, b - pf)
        vp = np.array([v[i], v[j]])
        rows.append(vp)
        rhs.append(vp @ np.array([b[i], b[j]]))
    M = np.array(rows)
    if abs(np.linalg.det(M)) < 1e-9:
        return np.array([np.nan, np.nan])
    return np.linalg.solve(M, np.array(rhs))


def corner_diagnostics(hp: Hardpoints, *, travel_mm: float = 25.0,
                       stations=None, track_mm: float = 1200.0,
                       axle: str = "front", wheelbase_mm: float | None = None,
                       cg_height_mm: float | None = None,
                       brake_bias_front: float | None = None,
                       n_dense: int = 201) -> dict:
    """Everything worth reporting about a corner that is not a channel.

    Slopes are least-squares over the stations (default −T, −T/2, 0, T/2, T);
    toe change is max − min over the stations. RC migration is given both in
    the chassis frame (the engine's frame) and above the displaced ground.
    Anti-dive is referenced to the contact patch (outboard brakes) and
    scaled by front brake bias; anti-squat to the wheel centre (inboard
    drive). Both need wheelbase and CG height; they are omitted otherwise.
    """
    st = (np.asarray(stations, float) if stations is not None
          else np.linspace(-travel_mm, travel_mm, 5))
    vals, ok = ig.curves_of(hp, st, track_mm=track_mm)
    if not ok:
        return {"ok": False}
    kin = SuspensionKinematics(hp)
    s0 = kin.solve_at_travel(0.0)
    dense = kin.sweep(travel_min=float(st.min()), travel_max=float(st.max()),
                      n=int(n_dense))
    tr = np.array([s.travel for s in dense])
    cpz = np.array([s.contact_patch[2] for s in dense])
    rc_dense = np.array([ig._rc_height_mm(s, track_mm) for s in dense])
    cpz_st = np.interp(st, tr, cpz)

    def slope(y):
        """Least-squares slope of ``y`` over the travel stations, in y-units per mm."""
        return float(np.polyfit(st, y, 1)[0])

    out = {
        "ok": True,
        "stations_mm": st.tolist(),
        "camber_deg": vals["camber_deg"].tolist(),
        "toe_deg": vals["toe_deg"].tolist(),
        "rc_height_mm": vals["rc_height_mm"].tolist(),
        "scrub_mm": vals["scrub_mm"].tolist(),
        "camber_gain_deg_per_mm": slope(vals["camber_deg"]),
        "bump_steer_deg_per_mm": slope(vals["toe_deg"]),
        "toe_change_deg": float(np.ptp(vals["toe_deg"])),
        "toe_min_deg": float(np.min(vals["toe_deg"])),
        "toe_max_deg": float(np.max(vals["toe_deg"])),
        "rc_height_static_mm": float(np.interp(0.0, st, vals["rc_height_mm"])),
        "rc_migration_chassis_mm_per_mm": slope(vals["rc_height_mm"]),
        "rc_migration_ground_mm_per_mm": slope(vals["rc_height_mm"] - cpz_st),
        "rc_above_ground_min_mm": float(np.min(rc_dense - cpz)),
        "caster_deg": float(s0.caster),
        "kpi_deg": float(s0.kpi),
        "scrub_static_mm": float(s0.scrub_radius),
        "contact_patch_rise_per_mm": float(np.polyfit(tr, cpz, 1)[0]),
        "ball_joint_z_mm": [float(s0.upper_outer[2]),
                            float(s0.lower_outer[2])],
    }
    # dense-vs-station check (does any curve leave its band between stations?)
    out["interp_error"] = {
        "camber_deg": float(np.max(np.abs(
            np.interp(st, tr, [s.camber for s in dense])
            - vals["camber_deg"]))),
        "toe_deg": float(np.max(np.abs(
            np.interp(st, tr, [s.toe for s in dense]) - vals["toe_deg"]))),
    }

    # side-view instant centre and anti-geometry
    p = {k: np.asarray(getattr(hp, k), float) for k in
         ("upper_front_inner", "upper_rear_inner",
          "lower_front_inner", "lower_rear_inner")}
    bu, bl = np.asarray(s0.upper_outer), np.asarray(s0.lower_outer)
    ic_sv = _ic_2d((p["upper_front_inner"], p["upper_rear_inner"], bu),
                   (p["lower_front_inner"], p["lower_rear_inner"], bl),
                   axes=(0, 2))
    out["side_view_ic_x_mm"] = float(ic_sv[0])
    out["side_view_ic_z_mm"] = float(ic_sv[1])
    cp, wc = np.asarray(s0.contact_patch), np.asarray(s0.wheel_center)
    ref = cp if axle == "front" else wc
    dx = ic_sv[0] - ref[0]
    tan_sv = float((ic_sv[1] - ref[2]) / dx) if abs(dx) > 1e-9 else math.nan
    out["side_view_tan"] = tan_sv
    out["side_view_reference"] = ("contact patch" if axle == "front"
                                  else "wheel centre")
    if wheelbase_mm and cg_height_mm and math.isfinite(tan_sv):
        k = tan_sv * wheelbase_mm / cg_height_mm
        if axle == "front":
            if brake_bias_front is not None:
                out["anti_dive_pct"] = 100.0 * k * float(brake_bias_front)
        else:
            # x is REARWARD, so the anti sense at the rear (SVIC forward of
            # and above the reference) is a NEGATIVE dx. The front branch's
            # sign applied here reported a pro-squat geometry as anti-squat
            # and vice versa. Use the kinematic core's path-slope methods,
            # which are sign-correct by virtual work, for both rear effects.
            kin = SuspensionKinematics(hp)
            out["anti_squat_pct"] = float(kin.anti_squat_pct(
                cg_height_mm, wheelbase_mm, 1.0, state=s0))
            if brake_bias_front is not None:
                out["anti_lift_pct"] = float(kin.anti_lift_pct(
                    cg_height_mm, wheelbase_mm,
                    1.0 - float(brake_bias_front), state=s0))
    return out


def camber_to_road(static_camber_deg: float, camber_gain_deg_per_mm: float,
                   roll_deg: float, track_mm: float,
                   optimum_deg: float | None = None) -> dict:
    """Table-7b arithmetic: outside-wheel bump from roll, camber to chassis,
    Angles in deg, track in mm, bump in mm, gains in deg/mm.
    camber to road, and the gain that would hold ``optimum_deg``."""
    bump = 0.5 * track_mm * math.tan(math.radians(roll_deg))
    change = camber_gain_deg_per_mm * bump
    to_chassis = static_camber_deg + change
    to_road = to_chassis + roll_deg
    d = {"bump_mm": bump, "camber_change_deg": change,
         "camber_to_chassis_deg": to_chassis, "camber_to_road_deg": to_road}
    if optimum_deg is not None and bump > 0:
        req = (optimum_deg - roll_deg - static_camber_deg) / bump
        d["gain_required_deg_per_mm"] = req
        d["ratio_to_delivered"] = (req / camber_gain_deg_per_mm
                                   if camber_gain_deg_per_mm else math.nan)
        d["error_from_optimum_deg"] = to_road - optimum_deg
    return d


# --------------------------------------------------------------------------- #
#  Yield, taken apart
# --------------------------------------------------------------------------- #
def _linear_rows(hp, targets, fld, n, seed, step_mm):
    r_fit, ok = targets.residual(hp)
    if not ok:
        return None, None, None, None
    J = ig._jacobian(hp, targets, fld.coords(), step_mm=step_mm)
    if J is None:
        return r_fit, None, None, None
    samples = fld.sample(n, seed=seed)
    return r_fit, J, samples, r_fit[None, :] + samples @ J.T


def pass_vector(hp, targets, fld, n=4000, seed=0, step_mm=0.25
                ) -> np.ndarray | None:
    """Per-build pass/fail on the engine's own samples (same seed → same
    builds), so two candidates can be compared pairwise."""
    _, _, _, rows = _linear_rows(hp, targets, fld, n, seed, step_mm)
    if rows is None:
        return None
    return np.all(np.abs(rows) <= 1.0, axis=1)


def yield_breakdown(hp: Hardpoints, targets: ig.GenesisTargets,
                    fld: ToleranceField, n: int = 4000, seed: int = 0,
                    step_mm: float = 0.25) -> dict:
    """Per-channel failure fractions, first-order worst case W, per-row
    Yields and failure fractions are dimensionless; headroom in standard deviations; step in mm.
    headroom in standard deviations, and the zero-failure bound."""
    r_fit, J, samples, rows = _linear_rows(hp, targets, fld, n, seed, step_mm)
    if rows is None:
        return {"ok": False}
    fail = np.abs(rows) > 1.0
    labels = targets.rows()
    per_ch = {}
    for ch in dict.fromkeys(c for c, _ in labels):
        idx = [i for i, (c, _) in enumerate(labels) if c == ch]
        per_ch[ch] = float(np.mean(np.any(fail[:, idx], axis=1)))
    passed = ~np.any(fail, axis=1)
    y = float(np.mean(passed))
    # first-order worst case over the tolerance box
    lo = np.concatenate([fld.specs[p].lo for p in sorted(fld.specs)])
    hi = np.concatenate([fld.specs[p].hi for p in sorted(fld.specs)])
    mid, half = 0.5 * (lo + hi), 0.5 * (hi - lo)
    worst_rows = np.abs(r_fit + J @ mid) + np.abs(J) @ half
    W = float(np.max(worst_rows))
    # headroom in σ, using the sample standard deviation of each row
    sd = np.std(samples @ J.T, axis=0)
    head = np.where(sd > 0, (1.0 - np.abs(r_fit)) / np.maximum(sd, 1e-15),
                    np.inf)
    gi = int(np.argmin(head))
    nfail = int(n - passed.sum())
    out = {
        "ok": True, "n": int(n), "seed": int(seed),
        "yield": y, "fails": nfail,
        "per_channel_fail_frac": per_ch,
        "worst_case_W": W,
        "worst_case_row": targets.row_labels()[int(np.argmax(worst_rows))],
        "guaranteed_first_order": W <= 1.0,
        "governing_row": targets.row_labels()[gi],
        "governing_headroom_sigma": float(head[gi]),
        "max_fit_residual": float(np.max(np.abs(r_fit))),
    }
    if nfail == 0:
        out["yield_lower_bound_95"] = 1.0 - 3.0 / n     # rule of three
    else:
        se = math.sqrt(y * (1 - y) / n)
        out["yield_se"] = se
    return out


def discordant_builds(pass_a: np.ndarray, pass_b: np.ndarray) -> dict:
    """Paired comparison on common random numbers: builds passing one
    candidate and failing the other, with the exact two-sided McNemar p."""
    a_only = int(np.sum(pass_a & ~pass_b))
    b_only = int(np.sum(~pass_a & pass_b))
    m = a_only + b_only
    if m == 0:
        p = 1.0
    else:
        k = min(a_only, b_only)
        p = min(1.0, 2.0 * sum(math.comb(m, i) for i in range(k + 1))
                / 2.0 ** m)
    return {"a_only": a_only, "b_only": b_only, "discordant": m,
            "p_two_sided": p}


def rounding_sensitivity(hp: Hardpoints, targets: ig.GenesisTargets,
                         fld: ToleranceField, points, cell_mm: float = 0.05,
                         n_trials: int = 64, n: int = 2000, seed: int = 0,
                         step_mm: float = 0.25) -> dict:
    """How much the yield estimate moves when ``points`` are displaced within
    a ±cell_mm rounding cell (uniform trials, deterministic). Use this before
    publishing coordinates rounded to 0.1 mm."""
    rng = np.random.default_rng(seed + 7919)
    ys = []
    for _ in range(int(n_trials)):
        offs = {p: rng.uniform(-cell_mm, cell_mm, 3) for p in points}
        pv = pass_vector(_perturbed(hp, offs), targets, fld, n, seed, step_mm)
        if pv is not None:
            ys.append(float(np.mean(pv)))
    if not ys:
        return {"ok": False}
    return {"ok": True, "min": min(ys), "max": max(ys),
            "mean": float(np.mean(ys)), "n_trials": len(ys),
            "cell_mm": cell_mm}


# --------------------------------------------------------------------------- #
#  Swept-volume clearance
# --------------------------------------------------------------------------- #
_LINKS = (("upper fore", "upper_front_inner", "upper_outer"),
          ("upper aft", "upper_rear_inner", "upper_outer"),
          ("lower fore", "lower_front_inner", "lower_outer"),
          ("lower aft", "lower_rear_inner", "lower_outer"),
          ("tie rod", "tie_rod_inner", "tie_rod_outer"))


def swept_clearance(hp: Hardpoints, obstacle: CapsuleObstacle, *,
                    travel_mm: float = 25.0, n_stations: int = 21,
                    link_radius_mm: float = 15.88 / 2,
                    exclusion_mm: float = 70.0) -> dict:
    """Every link, at every travel station, against every capsule.

    The first ``exclusion_mm`` of each link, measured from its inboard
    pickup, is excluded (overlap with the mounting tube is expected there
    by construction). Returns the per-link minimum and the global worst.
    Travel, link radius, exclusion and clearances in mm.
    """
    kin = SuspensionKinematics(hp)
    states = kin.sweep(travel_min=-travel_mm, travel_max=travel_mm,
                       n=int(n_stations))
    if any(not getattr(s, "converged", True) for s in states):
        return {"ok": False, "reason": "sweep did not converge"}
    per_link = {}
    worst = (math.inf, "", "", 0.0)
    for name, inner, outer in _LINKS:
        a0 = np.asarray(getattr(hp, inner), float)
        best = (math.inf, "", 0.0)
        for s in states:
            b = np.asarray(getattr(s, outer), float)
            L = np.linalg.norm(b - a0)
            if L <= exclusion_mm:
                continue
            a = a0 + (b - a0) * (exclusion_mm / L)
            for k in range(len(obstacle.a)):
                d = seg_seg_dist(a, b, obstacle.a[k], obstacle.b[k]) \
                    - obstacle.radius[k] - link_radius_mm
                if d < best[0]:
                    best = (d, obstacle.names[k], float(s.travel))
        per_link[name] = {"min_clearance_mm": float(best[0]),
                          "tube": best[1], "travel_mm": best[2]}
        if best[0] < worst[0]:
            worst = (best[0], name, best[1], best[2])
    return {"ok": True, "per_link": per_link,
            "min_clearance_mm": float(worst[0]), "link": worst[1],
            "tube": worst[2], "travel_mm": worst[3],
            "n_stations": int(n_stations), "exclusion_mm": exclusion_mm,
            "link_radius_mm": link_radius_mm}


# --------------------------------------------------------------------------- #
#  Structural screening of the links
# --------------------------------------------------------------------------- #
from . import loadpath as _lp


@dataclass
class TubeSpec:
    """A tube used for every link (the common FSAE single-size case)."""
    od_mm: float = 15.88
    wall_mm: float = 0.889
    yield_mpa: float = 460.0     # as-welded 4130, declared
    E_gpa: float = 205.0

    @property
    def ri(self):
        """Inner radius, mm."""
        return self.od_mm / 2 - self.wall_mm

    @property
    def area(self):
        """Wall cross-section area, mm²."""
        import math
        return math.pi * ((self.od_mm / 2) ** 2 - self.ri ** 2)

    @property
    def I(self):
        """Second moment of area, mm⁴."""
        import math
        return math.pi * ((self.od_mm / 2) ** 4 - self.ri ** 4) / 4


@dataclass
class LoadCaseSpec:
    """One contact-patch load vector.  ``mz_Nmm`` is the aligning torque."""
    name: str
    Fz: float
    Fy: float = 0.0
    Fx: float = 0.0
    mz_Nmm: float = 0.0

    def wheel_load(self) -> _lp.WheelLoad:
        """Contact-patch load: forces in N, aligning torque in N·mm."""
        return _lp.WheelLoad(Fx=self.Fx, Fy=self.Fy, Fz=self.Fz,
                             Mz=self.mz_Nmm)


def vehicle_load_cases(mass_kg: float = 300.0,
                     weight_dist_front: float = 0.48,
                     cg_height_mm: float = 280.0,
                     wheelbase_mm: float = 1630.0,
                     track_mm: float = 1210.0,
                     brake_bias_front: float = 0.60,
                     roll_share_front: float = 0.55,
                     aligning_torque_Nm: float = 50.0,
                     axle: str = "front",
                     lateral_g: float = 1.5) -> list[LoadCaseSpec]:
    """Five contact-patch load cases for one corner (forces in N, torque in
    N·mm), from the declared vehicle (mass kg, lengths mm, g-levels).

    - cornering at ``lateral_g``: Fz = static corner load + lateral load
      transfer weighted by the axle's roll-stiffness share; Fy = lateral_g·Fz;
      Mz = aligning torque.
    - 1.5g braking: Fz = static + longitudinal LT; Fx = bias × total braking / 2.
    - Combined 1.06g: lateral + longitudinal loads together, with aligning torque.
    - 3g vertical bump: purely vertical.
    - Kerb strike 2g vert + 1g long: Fz = 2× static; Fx = braking share.
    """
    import math
    g = 9.81
    m = float(mass_kg)
    af = float(weight_dist_front) if axle == "front" else 1.0 - float(weight_dist_front)
    rs = float(roll_share_front) if axle == "front" else 1.0 - float(roll_share_front)
    fz_s = m * g * af / 2        # static corner load
    lat_tr = lambda lat: m * g * lat * float(cg_height_mm) / float(track_mm)
    long_tr = lambda lon: m * g * lon * float(cg_height_mm) / float(wheelbase_mm)
    bias = float(brake_bias_front) if axle == "front" else 1.0 - float(brake_bias_front)
    mz = float(aligning_torque_Nm) * 1000.0

    lat_g = float(lateral_g)
    c1_fz = fz_s + rs * lat_tr(lat_g)
    c2_fz = fz_s + long_tr(1.5) / 2
    c3_fz = fz_s + rs * lat_tr(1.06) + long_tr(1.06) / 2
    c5_fz = fz_s * 2.0

    return [
        LoadCaseSpec(f"{lat_g:g}g corner", Fz=c1_fz, Fy=lat_g * c1_fz,
                     mz_Nmm=mz),
        LoadCaseSpec("1.5g braking", Fz=c2_fz, Fx=bias * m * g * 1.5 / 2),
        LoadCaseSpec("combined 1.06g", Fz=c3_fz, Fy=1.06 * c3_fz,
                     Fx=bias * m * g * 1.06 / 2, mz_Nmm=mz),
        LoadCaseSpec("3g vertical bump", Fz=fz_s * 3.0),
        LoadCaseSpec("kerb 2g+1g", Fz=c5_fz, Fx=bias * m * g * 1.0 / 2),
    ]


def structural_screening(hp: Hardpoints, tube: TubeSpec | None = None,
                         load_cases: list[LoadCaseSpec] | None = None,
                         fos_min: float = 1.5, axle: str = "front",
                         **load_case_kw) -> dict:
    """Member axial forces (N) and factors of safety (dimensionless) across a set of load cases.

    If ``load_cases`` is None, uses ``vehicle_load_cases(**load_case_kw)``.
    Screening is tension yield and pinned-pinned Euler buckling, the correct
    idealisation for a two-force member on spherical joints.
    """
    import math
    from .kinematics import SuspensionKinematics
    tube = tube or TubeSpec()
    kin = SuspensionKinematics(hp)
    state = kin.solve_at_travel(0.0)
    pts = _lp._member_geometry(kin, state)
    if load_cases is None:
        load_cases = vehicle_load_cases(axle=axle, **load_case_kw)

    def _length(m):
        if m not in pts:
            return math.nan
        p_out, p_in = pts[m]
        return float(np.linalg.norm(np.array(p_out) - np.array(p_in)))

    def _fos(F, L):
        if abs(F) < 0.01:
            return math.inf
        if F < 0:
            f_cr = math.pi ** 2 * tube.E_gpa * 1e3 * tube.I / L ** 2
            f_y = tube.yield_mpa * tube.area
            return min(f_y, f_cr) / abs(F)
        return tube.yield_mpa * tube.area / F

    members = [m for m in _lp.MEMBERS if m != "PR" and m in pts]
    lengths = {m: _length(m) for m in members}

    rows = []
    worst_fos = {m: math.inf for m in members}
    governing = {m: "" for m in members}

    for lc in load_cases:
        mf = _lp.solve_member_forces(kin, state, lc.wheel_load())
        for m in members:
            F = float(mf.forces.get(m, 0.0))
            L = lengths[m]
            if not math.isfinite(L):
                continue
            f = _fos(F, L)
            rows.append({"load_case": lc.name, "member": m,
                         "force_N": round(F, 1), "length_mm": round(L, 1),
                         "fos": round(f, 2),
                         "mode": "compression" if F < 0 else "tension",
                         "passes": f >= fos_min})
            if f < worst_fos[m]:
                worst_fos[m] = f
                governing[m] = lc.name

    worst_overall = min(worst_fos.values(), default=math.nan)
    governing_member = min(worst_fos, key=worst_fos.get) if worst_fos else ""

    return {
        "ok": True,
        "rows": rows,
        "worst_fos_per_member": {m: round(v, 2) for m, v in worst_fos.items()},
        "governing_case": governing,
        "worst_fos_overall": round(worst_overall, 2),
        "governing_member": governing_member,
        "all_pass": all(r["passes"] for r in rows),
        "tube": {"od_mm": tube.od_mm, "wall_mm": tube.wall_mm,
                 "yield_mpa": tube.yield_mpa, "E_gpa": tube.E_gpa},
        "fos_min": fos_min,
        "note": ("Two-force members on spherical joints: bending from the "
                 "pushrod attachment is not included.")
    }


# --------------------------------------------------------------------------- #
#  Compliance budget of the links
# --------------------------------------------------------------------------- #
def compliance_budget(hp: Hardpoints, axle: str = "front",
                      od_mm: float = 15.88, wall_mm: float = 0.889,
                      rod_end_lash_mm: float = 0.025,
                      lateral_g: float = 1.5,
                      **car_kw) -> dict:
    """Compliance steer and camber change at one load level.

    Uses the compliance module's uniform-tube path (axial stiffness only —
    a partial budget). Reports each as a fraction of the
    kinematic acceptance band, so the user can see how much band the load
    consumes before build scatter is added.
    """
    from .compliance import CompliantCorner, corner_wheel_load
    from .kinematics import SuspensionKinematics
    from .dynamics import VehicleDynamics, VehicleParams
    import math

    kin = SuspensionKinematics(hp)
    # build a minimal vehicle just to get wheel loads
    mass = float(car_kw.get("mass_kg", 300.0))
    af = float(car_kw.get("weight_dist_front", 0.48))
    cg = float(car_kw.get("cg_height_mm", 280.0))
    track = float(car_kw.get("track_mm", 1210.0))
    wb = float(car_kw.get("wheelbase_mm", 1630.0))
    rs = float(car_kw.get("roll_share_front", 0.55))
    mz = float(car_kw.get("aligning_torque_Nm", 50.0))
    axle_share = af if axle == "front" else 1.0 - af
    rs_share = rs if axle == "front" else 1.0 - rs
    g = 9.81
    fz = mass * g * axle_share / 2 + rs_share * mass * g * lateral_g * cg / track
    fy = lateral_g * fz
    vp = VehicleParams(mass=mass, cg_height=cg, wheelbase=wb,
                       track_front=track, track_rear=track,
                       weight_dist_front=af)
    veh = VehicleDynamics(vp)

    # kerb-strike load for worst compliance
    fz_kerb = mass * g * axle_share * 2 / 2
    fy_kerb, fx_kerb = 0.0, mass * g * 0.6 / 2

    from .loadpath import WheelLoad
    corner_load = WheelLoad(Fx=0, Fy=fy, Fz=fz, Mz=mz * 1000)
    kerb_load = WheelLoad(Fx=fx_kerb, Fy=fy_kerb, Fz=fz_kerb, Mz=0)

    cc = CompliantCorner.uniform_tube(hp, od_mm=od_mm, wall_mm=wall_mm)

    results = {}
    for name, load in (("cornering", corner_load), ("kerb strike", kerb_load)):
        try:
            r = cc.solve(load)
            s = r.summary()
            results[name] = {
                "compliance_toe_deg": round(r.compliance_toe, 4),
                "compliance_camber_deg": round(r.compliance_camber, 4),
                "toe_band_fraction": round(abs(r.compliance_toe) / 0.08, 3),
                "camber_band_fraction": round(abs(r.compliance_camber) / 0.30, 3),
                "max_link_extension_mm": round(max(
                    [abs(v) for v in s.get("member_deflection_mm", {}).values()]
                    or [0.0]), 4),
                "forces_N": {k: round(v, 1) for k, v in
                             s.get("member_forces_N", {}).items()},
            }
        except Exception as e:                   # noqa: BLE001
            results[name] = {"error": str(e)}
    results["note"] = ("Partial budget: axial stiffness only. Bracket "
                       "flex, chassis stiffness and upright compliance "
                       "are omitted and all act in the same direction.")
    return results
