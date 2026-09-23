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

    def __post_init__(self):
        if int(self.n_steps) < 1:
            raise ValueError("ElastoSpec.n_steps must be >= 1.")

    def load(self, frac: float = 1.0) -> _lp.WheelLoad:
        return _lp.WheelLoad(Fx=frac * self.Fx, Fy=frac * self.Fy,
                             Fz=frac * self.Fz, Mz=frac * self.Mz)

    def to_dict(self) -> dict:
        """JSON-safe; member stiffness maps are not serialised (declare in code)."""
        return {"stiffness": self.stiffness.to_dict(), "Fx": float(self.Fx),
                "Fy": float(self.Fy), "Fz": float(self.Fz), "Mz": float(self.Mz),
                "n_steps": int(self.n_steps)}

    @classmethod
    def from_dict(cls, d: dict) -> "ElastoSpec":
        return cls(stiffness=StiffnessField.from_dict(d.get("stiffness", {})),
                   Fx=float(d.get("Fx", 0.0)), Fy=float(d.get("Fy", 0.0)),
                   Fz=float(d.get("Fz", 0.0)), Mz=float(d.get("Mz", 0.0)),
                   n_steps=int(d.get("n_steps", 5)))


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


def solve_elastokinematic(hp: Hardpoints, spec: ElastoSpec, tol_mm: float = 1e-6,
                          max_iter: int = 40) -> ElastoResult:
    """Apply ``spec``'s load in steps and solve the deflected corner."""
    _, s_rigid = _state(hp, None, None)
    base = _channels_of(s_rigid)
    pick = {p: np.zeros(3) for p in PICKUPS}
    lens: dict = {}
    link_defl: dict = {}
    iters, ok, forces = [], True, {}
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
    _, s_def = _state(hp, pick, lens)
    nl = _channels_of(s_def) - base
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
        iterations=iters, provenance=spec.stiffness.provenance)


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
