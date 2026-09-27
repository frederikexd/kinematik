# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Elastokinematics at the chassis pickups: stiffness matrices, measured input,
load steps and the compliance Jacobian.

``compliance.py`` flexes the links: axial strain, tab stiffness in series and
non-linear joints, all acting along each link. What it cannot represent is the
frame node behind a pickup deflecting in any direction, including across the
link, which is where the kinematics are most sensitive. This module adds that
layer and couples it to the axial one:

* ``StiffnessField`` gives each chassis pickup a stiffness, as a declared
  scalar (N/mm, isotropic), a full 3x3 matrix in the corner frame (for example
  condensed from a frame finite-element model), or built from measured pull
  tests. ``provenance`` records which, so a declared value is never reported
  as measured.
* ``compliance_jacobian`` is the linear map from pickup displacement to the
  change in camber, toe, caster and kingpin inclination, by central
  differences with link lengths held.
* ``solve_elastokinematic`` applies the wheel load in steps. Within each step
  it re-resolves member forces on the geometry deflected so far, turns them
  into pickup deflections through the stiffness field and, if members are
  given, into link length changes through ``compliance.MemberStiffness``, and
  iterates to a fixed point before the next increment. It returns the
  non-linear result beside the Jacobian prediction and the difference, so the
  adequacy of a linear correction is measured each time rather than assumed.

Sign convention (loadpath): member force T > 0 is tension; a link in tension
pulls its chassis pickup toward the upright, along -u_hat, where u_hat runs
from the outboard point to the inboard one. A pickup of stiffness K moves by
d = K^-1 (-T u_hat).

Units: mm, N, N/mm (stiffness), deg (channels).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping, Sequence

import numpy as np

from .kinematics import Hardpoints, SuspensionKinematics
from . import loadpath as _lp

#: member -> chassis pickup it loads
MEMBER_PICKUP = {"UF": "upper_front_inner", "UR": "upper_rear_inner",
                 "LF": "lower_front_inner", "LR": "lower_rear_inner",
                 "TR": "tie_rod_inner"}
_MEMBER_TO_LENGTHKEY = {"UF": "upper_f", "UR": "upper_r", "LF": "lower_f",
                        "LR": "lower_r", "TR": "tie"}
PICKUPS = tuple(MEMBER_PICKUP.values())
CHANNELS = ("camber", "toe", "caster", "kpi")


@dataclass
class StiffnessField:
    """Series stiffness of node, bracket and bearing at each chassis pickup.

    ``default_n_per_mm`` applies to pickups not in ``per_point``; each
    ``per_point`` value is a scalar (isotropic, N/mm) or a 3x3 matrix (N/mm,
    corner frame). ``provenance`` is "declared", "measured" or "fea".
    """
    default_n_per_mm: float = 30000.0
    per_point: dict = field(default_factory=dict)
    provenance: str = "declared"

    def __post_init__(self):
        if float(self.default_n_per_mm) <= 0.0:
            raise ValueError("StiffnessField.default_n_per_mm must be > 0.")
        if self.provenance not in ("declared", "measured", "fea"):
            raise ValueError("provenance must be 'declared', 'measured' or 'fea'.")
        for k in self.per_point:
            if k not in PICKUPS:
                raise ValueError(f"StiffnessField: unknown pickup '{k}'.")
            self.matrix(k)                       # validate early

    def matrix(self, point: str) -> np.ndarray:
        """Stiffness matrix at a pickup, N/mm."""
        a = np.asarray(self.per_point.get(point, self.default_n_per_mm), float)
        if a.ndim == 0:
            if float(a) <= 0.0:
                raise ValueError(f"stiffness at {point} must be > 0.")
            return float(a) * np.eye(3)
        if a.shape != (3, 3):
            raise ValueError(f"stiffness at {point} must be a scalar or 3x3.")
        sym = 0.5 * (a + a.T)
        if np.any(np.linalg.eigvalsh(sym) <= 0.0):
            raise ValueError(f"stiffness matrix at {point} is not positive definite.")
        return sym

    def compliance(self, point: str) -> np.ndarray:
        """Compliance matrix at a pickup, mm/N."""
        return np.linalg.inv(self.matrix(point))

    def scaled(self, factor: float) -> "StiffnessField":
        """Every stiffness multiplied by ``factor`` (dimensionless)."""
        return StiffnessField(
            default_n_per_mm=self.default_n_per_mm * factor,
            per_point={k: np.asarray(v, float) * factor for k, v in self.per_point.items()},
            provenance=self.provenance)

    @classmethod
    def from_pull_tests(cls, hp: Hardpoints, tests: Mapping[str, tuple],
                        transverse_n_per_mm: float | None = None,
                        default_n_per_mm: float = 30000.0) -> "StiffnessField":
        """Build from measured in-situ pull tests.

        ``tests`` maps a pickup to (force N, deflection mm), both measured along
        its own link. That sets the stiffness along the link; a single pull says
        nothing across it, so the transverse stiffness is ``transverse_n_per_mm``
        if given, otherwise taken equal to the axial value. The matrix is built
        in the link frame and rotated into the corner frame.
        """
        pp = {}
        outer = {"upper_front_inner": "upper_outer", "upper_rear_inner": "upper_outer",
                 "lower_front_inner": "lower_outer", "lower_rear_inner": "lower_outer",
                 "tie_rod_inner": "tie_rod_outer"}
        for pt, (f, d) in tests.items():
            if pt not in PICKUPS:
                raise ValueError(f"from_pull_tests: unknown pickup '{pt}'.")
            if float(f) <= 0.0 or float(d) <= 0.0:
                raise ValueError(f"from_pull_tests: {pt} needs force and deflection > 0.")
            ka = float(f) / float(d)
            kt = float(transverse_n_per_mm) if transverse_n_per_mm else ka
            u = np.asarray(getattr(hp, outer[pt]), float) - np.asarray(getattr(hp, pt), float)
            u /= np.linalg.norm(u)
            P = np.outer(u, u)
            pp[pt] = ka * P + kt * (np.eye(3) - P)
        return cls(default_n_per_mm=default_n_per_mm, per_point=pp, provenance="measured")

    def to_dict(self) -> dict:
        """JSON-safe form; stiffness in N/mm."""
        return {"default_n_per_mm": float(self.default_n_per_mm),
                "per_point": {k: np.asarray(v, float).tolist()
                              for k, v in self.per_point.items()},
                "provenance": self.provenance}

    @classmethod
    def from_dict(cls, d: dict) -> "StiffnessField":
        return cls(default_n_per_mm=float(d.get("default_n_per_mm", 30000.0)),
                   per_point={k: np.asarray(v, float) for k, v in d.get("per_point", {}).items()},
                   provenance=d.get("provenance", "declared"))


@dataclass
class FrameTwist:
    """A global frame twist mode between the rack and the tie-rod plane.

    The pickup stiffness field is local: it cannot see the frame twisting
    between the axles. This carries the uniform-twist estimate of
    ``frame_twist_toe`` onto every trial geometry, so it can be bounded inside
    the search: the roll moment ``torque_Nm`` twists a frame of torsional
    stiffness ``kt_Nm_per_deg``; the share of that twist over
    ``separation_mm`` (rack mounts to tie-rod inner plane) moves the rack by
    that angle times ``lever_mm``, and the corner's own tie-rod sensitivity,
    solved per geometry, turns it into toe. Lengths mm.
    """
    kt_Nm_per_deg: float
    torque_Nm: float
    separation_mm: float
    lever_mm: float = 200.0
    wheelbase_mm: float = 1630.0

    def __post_init__(self):
        for n in ("kt_Nm_per_deg", "wheelbase_mm"):
            if float(getattr(self, n)) <= 0.0:
                raise ValueError(f"FrameTwist.{n} must be > 0.")
        for n in ("separation_mm", "lever_mm"):
            if float(getattr(self, n)) < 0.0:
                raise ValueError(f"FrameTwist.{n} must be >= 0.")

    def toe_deg(self, hp: Hardpoints) -> float:
        """Toe (deg) the twist puts into this corner."""
        J = compliance_jacobian(hp, points=("tie_rod_inner",))
        u = np.asarray(hp.tie_rod_outer, float) - np.asarray(hp.tie_rod_inner, float)
        u /= np.linalg.norm(u)
        toe_per_mm = float(abs(J[1] @ u))
        return abs(frame_twist_toe(self.torque_Nm, self.kt_Nm_per_deg,
                                   self.wheelbase_mm, self.separation_mm,
                                   self.lever_mm, toe_per_mm))

    def to_dict(self) -> dict:
        """Serialise: torsional rate in N·m/deg, torque in N·m, lengths in mm."""
        return {"kt_Nm_per_deg": float(self.kt_Nm_per_deg),
                "torque_Nm": float(self.torque_Nm),
                "separation_mm": float(self.separation_mm),
                "lever_mm": float(self.lever_mm),
                "wheelbase_mm": float(self.wheelbase_mm)}

    @classmethod
    def from_dict(cls, d: dict) -> "FrameTwist":
        """Rebuild from a dict: torsional rate in N·m/deg, torque in N·m, lengths in mm."""
        return cls(**{k: float(v) for k, v in d.items()})


def lash_deadband(hp: Hardpoints, lash_mm: Mapping[str, float],
                  h: float = 0.01) -> dict:
    """Worst-case channel movement (deg) the declared joint clearance allows.

    A joint with clearance lets its link change effective length by up to
    the clearance with no load at all, ahead of any stiffness. The deadband of
    each channel is the sum over members of |d channel / d length| times that
    member's total clearance: the worst case, every joint at the end of its
    play in the adverse direction. Returns {channel: deg}.
    """
    out = np.zeros(len(CHANNELS))
    base = None
    for m, c in lash_mm.items():
        if not c:
            continue
        key = _MEMBER_TO_LENGTHKEY[m]
        cp = _channels_of(_state(hp, None, {key: h})[1])
        cm = _channels_of(_state(hp, None, {key: -h})[1])
        out += np.abs((cp - cm) / (2.0 * h)) * float(c)
    return {ch: float(v) for ch, v in zip(CHANNELS, out)}


@dataclass
class ElastoSpec:
    """Load case and stiffness a compliance evaluation runs under.

    Contact-patch forces in N (loadpath SAE axes), aligning torque in N*mm,
    ``n_steps`` load increments. ``members`` optionally maps a member name to a
    ``compliance.MemberStiffness`` for link-side axial give in series.
    """
    stiffness: StiffnessField = field(default_factory=StiffnessField)
    Fx: float = 0.0
    Fy: float = 0.0
    Fz: float = 0.0
    Mz: float = 0.0
    n_steps: int = 5
    members: dict = field(default_factory=dict)
    #: shaft-reacted longitudinal force at the wheel centre (N), see
    #: loadpath.WheelLoad.Fx_wc (inboard drive / inboard brakes)
    Fx_wc: float = 0.0
    #: total radial clearance (mm) per member across both of its joints, the
    #: free play taken up before the joint carries load: {"TR": 0.05, ...}
    joint_lash_mm: dict = field(default_factory=dict)
    #: optional global frame twist between the rack and the tie-rod plane
    twist: "FrameTwist | None" = None

    def __post_init__(self):
        if int(self.n_steps) < 1:
            raise ValueError("ElastoSpec.n_steps must be >= 1.")
        self.joint_lash_mm = {str(k): float(v) for k, v in
                              dict(self.joint_lash_mm).items()}
        for k, v in self.joint_lash_mm.items():
            if k not in MEMBER_PICKUP:
                raise ValueError(f"ElastoSpec.joint_lash_mm: unknown member '{k}'.")
            if v < 0.0:
                raise ValueError("ElastoSpec.joint_lash_mm values must be >= 0 (mm).")
        if self.twist is not None and not isinstance(self.twist, FrameTwist):
            raise TypeError("ElastoSpec.twist takes a FrameTwist.")

    def load(self, frac: float = 1.0) -> _lp.WheelLoad:
        return _lp.WheelLoad(Fx=frac * self.Fx, Fy=frac * self.Fy,
                             Fz=frac * self.Fz, Mz=frac * self.Mz,
                             Fx_wc=frac * self.Fx_wc)

    def to_dict(self) -> dict:
        """JSON-safe; member stiffness maps are not serialised (declare in code).

        Fields added after the first manifest schema are written only when
        they differ from their defaults, so every earlier manifest keeps its
        input hash.
        """
        d = {"stiffness": self.stiffness.to_dict(), "Fx": float(self.Fx),
             "Fy": float(self.Fy), "Fz": float(self.Fz), "Mz": float(self.Mz),
             "n_steps": int(self.n_steps)}
        if self.Fx_wc:
            d["Fx_wc"] = float(self.Fx_wc)
        if self.joint_lash_mm:
            d["joint_lash_mm"] = dict(sorted(self.joint_lash_mm.items()))
        if self.twist is not None:
            d["twist"] = self.twist.to_dict()
        return d

    @classmethod
    def from_dict(cls, d: dict) -> "ElastoSpec":
        return cls(stiffness=StiffnessField.from_dict(d.get("stiffness", {})),
                   Fx=float(d.get("Fx", 0.0)), Fy=float(d.get("Fy", 0.0)),
                   Fz=float(d.get("Fz", 0.0)), Mz=float(d.get("Mz", 0.0)),
                   n_steps=int(d.get("n_steps", 5)),
                   Fx_wc=float(d.get("Fx_wc", 0.0)),
                   joint_lash_mm=dict(d.get("joint_lash_mm", {})),
                   twist=(FrameTwist.from_dict(d["twist"]) if d.get("twist")
                          else None))

    def wheel_load_magnitude(self) -> float:
        """|F| of the full wheel load, N (contact-patch and wheel-centre parts)."""
        return float(np.linalg.norm(self.load(1.0).force()))


def _state(hp: Hardpoints, pickup: dict | None, lengths: dict | None):
    kin = SuspensionKinematics(hp, length_deltas=lengths or None,
                               pickup_deltas=pickup or None)
    return kin, kin.solve_at_travel(0.0)


def _channels_of(st) -> np.ndarray:
    return np.array([st.camber, st.toe, st.caster, st.kpi], float)


def compliance_jacobian(hp: Hardpoints, points: Sequence[str] = PICKUPS,
                        h: float = 0.05) -> np.ndarray:
    """d(camber, toe, caster, kpi)/d(pickup displacement), deg/mm.

    Shape (4, 3*len(points)); columns are x, y, z per point. Central
    differences with link lengths held.
    """
    cols = []
    for p in points:
        for ax in range(3):
            e = np.zeros(3)
            e[ax] = h
            cp = _channels_of(_state(hp, {p: e}, None)[1])
            cm = _channels_of(_state(hp, {p: -e}, None)[1])
            cols.append((cp - cm) / (2.0 * h))
    return np.array(cols).T


@dataclass
class ElastoResult:
    """Outcome of a load-stepped elastokinematic solve (deg, mm, N)."""
    change: dict                   # channel -> deg, non-linear, load-stepped
    linear: dict                   # channel -> deg, Jacobian prediction
    linearity_error: dict          # channel -> |change - linear|, deg
    pickup_deflection_mm: dict     # pickup -> [dx, dy, dz]
    link_deflection_mm: dict       # member -> axial give (mm), if members given
    member_forces: dict            # member -> final axial force (N)
    converged: bool
    iterations: list               # fixed-point iterations per load step
    provenance: str
    #: every member's final axial force (N), pushrod included
    all_member_forces: dict = field(default_factory=dict)


def solve_elastokinematic(hp: Hardpoints, spec: ElastoSpec, tol_mm: float = 1e-6,
                          max_iter: int = 40, jacobian: bool = True) -> ElastoResult:
    """Apply ``spec``'s load in steps and solve the deflected corner.

    ``jacobian=False`` skips the linear comparison (linear and linearity_error
    are then empty), which is what the search uses: the non-linear result is
    the one bounded, and the Jacobian costs thirty extra solves.
    """
    _, s_rigid = _state(hp, None, None)
    base = _channels_of(s_rigid)
    pick = {p: np.zeros(3) for p in PICKUPS}
    lens: dict = {}
    link_defl: dict = {}
    iters, ok, forces, all_f = [], True, {}, {}
    for k in range(1, int(spec.n_steps) + 1):
        load = spec.load(k / int(spec.n_steps))
        for it in range(1, max_iter + 1):
            kin, st = _state(hp, pick, lens)
            mf = _lp.solve_member_forces(kin, st, load)
            new_pick, new_lens = {}, {}
            for m, p in MEMBER_PICKUP.items():
                T = float(mf.forces.get(m, 0.0))
                u = np.asarray(mf.axes[m], float)
                new_pick[p] = spec.stiffness.compliance(p) @ (-T * u)
                ms = spec.members.get(m)
                if ms is not None:
                    L = float(np.linalg.norm(np.asarray(mf.outboard[m], float)
                                             - np.asarray(getattr(hp, p), float)))
                    dl = float(ms.axial_deflection(T, L))
                    new_lens[_MEMBER_TO_LENGTHKEY[m]] = dl
                    link_defl[m] = dl
            step = max(float(np.linalg.norm(new_pick[p] - pick[p])) for p in PICKUPS)
            if new_lens or lens:
                step = max(step, max(abs(new_lens.get(q, 0.0) - lens.get(q, 0.0))
                                     for q in set(new_lens) | set(lens)))
            pick, lens = new_pick, new_lens
            if step < tol_mm:
                break
        else:
            ok = False
        iters.append(it)
        forces = {m: float(mf.forces.get(m, 0.0)) for m in MEMBER_PICKUP}
        all_f = {m: float(v) for m, v in mf.forces.items()}
    _, s_def = _state(hp, pick, lens)
    nl = _channels_of(s_def) - base
    if not jacobian:
        return ElastoResult(
            change={c: float(v) for c, v in zip(CHANNELS, nl)}, linear={},
            linearity_error={}, pickup_deflection_mm={p: pick[p].tolist() for p in PICKUPS},
            link_deflection_mm=link_defl, member_forces=forces, converged=ok,
            iterations=iters, provenance=spec.stiffness.provenance,
            all_member_forces=all_f)
    J = compliance_jacobian(hp)
    lin = J @ np.concatenate([pick[p] for p in PICKUPS])
    if lens:
        # the Jacobian covers pickups only; add the link-side part linearly
        _, s_len = _state(hp, None, lens)
        lin = lin + (_channels_of(s_len) - base)
    return ElastoResult(
        change={c: float(v) for c, v in zip(CHANNELS, nl)},
        linear={c: float(v) for c, v in zip(CHANNELS, lin)},
        linearity_error={c: float(abs(a - b)) for c, a, b in zip(CHANNELS, nl, lin)},
        pickup_deflection_mm={p: pick[p].tolist() for p in PICKUPS},
        link_deflection_mm=link_defl, member_forces=forces, converged=ok,
        iterations=iters, provenance=spec.stiffness.provenance,
        all_member_forces=all_f)


def required_uniform_stiffness(hp: Hardpoints, spec: ElastoSpec, channel: str,
                               budget_deg: float) -> float:
    """Uniform pickup stiffness (N/mm) holding ``channel`` inside ``budget_deg``.

    Uses the pickup field alone and the linear scaling of change with
    compliance, then confirms it with a second load-stepped solve.
    """
    base = ElastoSpec(stiffness=StiffnessField(spec.stiffness.default_n_per_mm),
                      Fx=spec.Fx, Fy=spec.Fy, Fz=spec.Fz, Mz=spec.Mz,
                      n_steps=spec.n_steps)
    ch = abs(solve_elastokinematic(hp, base).change[channel])
    if ch == 0.0:
        return 0.0
    return spec.stiffness.default_n_per_mm * ch / float(budget_deg)


def frame_twist_toe(torque_Nm: float, kt_Nm_per_deg: float, wheelbase_mm: float,
                    separation_mm: float, lever_mm: float,
                    toe_per_mm: float) -> float:
    """Toe (deg) from frame twist between a rack and its tie-rod plane.

    Uniform twist rate along the wheelbase: the relative rotation over
    ``separation_mm`` moves the rack by that angle times ``lever_mm``, and the
    tie-rod sensitivity ``toe_per_mm`` (deg/mm) turns that into toe.
    """
    rel = (torque_Nm / kt_Nm_per_deg) * separation_mm / wheelbase_mm
    return float(np.radians(rel) * lever_mm * toe_per_mm)
