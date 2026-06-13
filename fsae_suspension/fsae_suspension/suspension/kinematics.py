"""
Double-wishbone kinematics solver for Formula SAE suspension.

This is the engineering core of the tool. Given the 3D hardpoint locations of an
unequal-length double-wishbone corner, it solves the upright position as a function
of vertical wheel travel by enforcing the rigid-link constraints:

    - upper wishbone: upper outer ball joint stays at fixed length from BOTH
      upper-front and upper-rear chassis pickups (a circle/arc constraint)
    - lower wishbone: same for the lower ball joint
    - upright: the distance between upper and lower ball joints is rigid

We parameterise travel by the lower ball joint's vertical position and solve the
resulting nonlinear constraint system with a Levenberg-Marquardt least-squares step
(scipy.optimize.least_squares, method="lm") at each position. From the solved upright
pose we extract the kinematic outputs FSAE teams actually tune around:

    camber gain, toe (bump steer), caster, kingpin inclination (KPI), scrub radius,
    and the front-view instant-centre location.

Roll-centre height is derived from the instant centre at the vehicle level (see
dynamics.py), not here. Motion ratio is available via a separate method that
finite-differences the linkage. Anti-dive/anti-squat is NOT yet implemented — it is
on the roadmap, and is deliberately not reported rather than approximated.

All coordinates are SAE-style vehicle axes, in millimetres:
    x : rearward positive (vehicle longitudinal)
    y : right positive (lateral, toward the driver's right)
    z : upward positive (vertical)

A single "corner" is modelled. Left/right symmetry is handled by mirroring y.
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field, asdict
from scipy.optimize import least_squares


# --------------------------------------------------------------------------- #
#  Hardpoint container
# --------------------------------------------------------------------------- #
@dataclass
class Hardpoints:
    """3D pickup-point coordinates for one corner, mm, SAE axes (x rear, y right, z up)."""

    # Upper wishbone chassis pickups
    upper_front_inner: np.ndarray
    upper_rear_inner: np.ndarray
    # Lower wishbone chassis pickups
    lower_front_inner: np.ndarray
    lower_rear_inner: np.ndarray
    # Outboard ball joints (on the upright) at static ride height
    upper_outer: np.ndarray
    lower_outer: np.ndarray
    # Steering / tie rod
    tie_rod_inner: np.ndarray
    tie_rod_outer: np.ndarray
    # Wheel
    wheel_center: np.ndarray
    contact_patch: np.ndarray
    # Design intent at static ride height (deg). These set the wheel spin-axis
    # orientation, which the linkage then carries rigidly through travel & steer.
    static_camber: float = -1.5
    static_toe: float = 0.0

    @staticmethod
    def default() -> "Hardpoints":
        """A sane FSAE front-corner geometry (right side). Roughly a 1.55 m track car."""
        return Hardpoints(
            upper_front_inner=np.array([-100.0, 240.0, 290.0]),
            upper_rear_inner=np.array([130.0, 240.0, 290.0]),
            lower_front_inner=np.array([-110.0, 200.0, 120.0]),
            lower_rear_inner=np.array([140.0, 200.0, 120.0]),
            upper_outer=np.array([12.0, 540.0, 300.0]),
            lower_outer=np.array([-5.0, 575.0, 110.0]),
            tie_rod_inner=np.array([100.0, 230.0, 160.0]),
            tie_rod_outer=np.array([90.0, 560.0, 150.0]),
            wheel_center=np.array([0.0, 600.0, 228.0]),
            contact_patch=np.array([0.0, 605.0, 0.0]),
        )

    def as_dict(self):
        out = {}
        for k, v in asdict(self).items():
            if isinstance(v, np.ndarray):
                out[k] = v.tolist()
            else:
                out[k] = v
        return out

    @staticmethod
    def from_dict(d) -> "Hardpoints":
        vec_keys = {"upper_front_inner", "upper_rear_inner", "lower_front_inner",
                    "lower_rear_inner", "upper_outer", "lower_outer",
                    "tie_rod_inner", "tie_rod_outer", "wheel_center", "contact_patch"}
        kwargs = {}
        for k, v in d.items():
            kwargs[k] = np.array(v, float) if k in vec_keys else v
        return Hardpoints(**kwargs)

    def copy(self) -> "Hardpoints":
        return Hardpoints.from_dict(self.as_dict())


# --------------------------------------------------------------------------- #
#  Geometry helpers
# --------------------------------------------------------------------------- #
def _unit(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    return v / n if n > 1e-12 else v


def _rotation_from_to(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Rotation matrix rotating unit vector a onto unit vector b (Rodrigues)."""
    a, b = _unit(a), _unit(b)
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    if np.linalg.norm(v) < 1e-12:
        return np.eye(3) if c > 0 else -np.eye(3)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * (1.0 / (1.0 + c))


# --------------------------------------------------------------------------- #
#  Solved corner state
# --------------------------------------------------------------------------- #
@dataclass
class CornerState:
    travel: float                 # vertical wheel-centre travel from static, mm (+ = bump)
    upper_outer: np.ndarray
    lower_outer: np.ndarray
    tie_rod_outer: np.ndarray
    wheel_center: np.ndarray
    contact_patch: np.ndarray
    camber: float                 # deg, negative = top leans inboard
    toe: float                    # deg, positive = toe-out
    caster: float                 # deg
    kpi: float                    # deg
    scrub_radius: float           # mm
    instant_center: np.ndarray    # front-view IC (y,z) of the linkage, mm
    roll_center_height: float     # mm, computed at vehicle level
    converged: bool = True


# --------------------------------------------------------------------------- #
#  The kinematics solver
# --------------------------------------------------------------------------- #
class SuspensionKinematics:
    """Solves a double-wishbone corner over a range of wheel travel."""

    def __init__(self, hp: Hardpoints):
        self.hp = hp
        self._validate(hp)
        self._cache_static()

    @staticmethod
    def _validate(hp: "Hardpoints"):
        """Fail fast with a clear message rather than a cryptic solver error."""
        point_fields = [
            "upper_front_inner", "upper_rear_inner", "lower_front_inner",
            "lower_rear_inner", "upper_outer", "lower_outer",
            "tie_rod_inner", "tie_rod_outer", "wheel_center", "contact_patch",
        ]
        for name in point_fields:
            v = np.asarray(getattr(hp, name), float)
            if v.shape != (3,):
                raise ValueError(
                    f"Hardpoint '{name}' must be a 3D point [x, y, z]; "
                    f"got shape {v.shape}.")
            if not np.all(np.isfinite(v)):
                raise ValueError(f"Hardpoint '{name}' contains non-finite values: {v}.")
        # Degenerate geometry: ball joints must be distinct, wishbone arms non-zero.
        if np.linalg.norm(hp.upper_outer - hp.lower_outer) < 1e-6:
            raise ValueError("Upper and lower ball joints are coincident — "
                             "the upright would have zero length.")
        for inner, outer, label in [
            (hp.upper_front_inner, hp.upper_outer, "upper front"),
            (hp.upper_rear_inner, hp.upper_outer, "upper rear"),
            (hp.lower_front_inner, hp.lower_outer, "lower front"),
            (hp.lower_rear_inner, hp.lower_outer, "lower rear"),
        ]:
            if np.linalg.norm(np.asarray(inner, float) - np.asarray(outer, float)) < 1e-6:
                raise ValueError(f"The {label} wishbone has zero length "
                                 "(inner and outer points coincide).")

    def _cache_static(self):
        hp = self.hp
        # Rigid link lengths captured at static ride height.
        self.L_upper_f = np.linalg.norm(hp.upper_outer - hp.upper_front_inner)
        self.L_upper_r = np.linalg.norm(hp.upper_outer - hp.upper_rear_inner)
        self.L_lower_f = np.linalg.norm(hp.lower_outer - hp.lower_front_inner)
        self.L_lower_r = np.linalg.norm(hp.lower_outer - hp.lower_rear_inner)
        self.L_upright = np.linalg.norm(hp.upper_outer - hp.lower_outer)
        self.L_tie = np.linalg.norm(hp.tie_rod_outer - hp.tie_rod_inner)
        # Tie-rod outer is rigid to the upright: fixed distances to both ball joints.
        self.L_tro_lo = np.linalg.norm(hp.tie_rod_outer - hp.lower_outer)
        self.L_tro_uo = np.linalg.norm(hp.tie_rod_outer - hp.upper_outer)
        # Signed side of tro relative to the kingpin plane, to disambiguate the
        # two mirror solutions for the tie-rod outer position.
        kp0 = _unit(hp.upper_outer - hp.lower_outer)
        n0 = _unit(np.cross(kp0, np.array([1.0, 0.0, 0.0])))
        self._tro_side = np.sign(np.dot(hp.tie_rod_outer - hp.lower_outer, n0)) or 1.0

        # Rigid offsets of the wheel-carrier points relative to the upright frame.
        # The upright frame is defined by the lower ball joint (origin) and the
        # kingpin axis (lower->upper). We store body points in that local frame so
        # they ride rigidly with the upright as it moves.
        self.kp_axis_static = _unit(hp.upper_outer - hp.lower_outer)
        # Define a rigid upright frame from TWO physical points (lower & upper ball
        # joints) plus the tie-rod outer, which fixes rotation about the kingpin.
        self._R0, self._o0 = self._upright_pose(hp.lower_outer, hp.upper_outer, hp.tie_rod_outer)
        # All carrier points expressed in this static frame -> ride rigidly.
        self._wc_local = self._R0.T @ (hp.wheel_center - self._o0)
        self._cp_local = self._R0.T @ (hp.contact_patch - self._o0)
        # Static wheel spin axis (outboard +y) tilted by design camber & toe, then
        # frozen into the upright-local frame so it rides rigidly through travel.
        cam = np.radians(hp.static_camber)
        toe = np.radians(hp.static_toe)
        spin0 = _unit(np.array([np.sin(toe), np.cos(toe) * np.cos(cam), -np.sin(cam)]))
        self._spin_local = self._R0.T @ spin0

        self.static = self.solve_at_travel(0.0)

    def _upright_pose(self, lo, uo, tro):
        """
        Build a rigid orthonormal frame + origin for the upright from three of its
        physical points: lower ball joint (origin), kingpin axis (lo->uo) as local z,
        and the tie-rod outer to fix rotation about the kingpin (local x in-plane).
        Returns (R, origin) such that any local point p maps to origin + R @ p.
        """
        origin = lo.copy()
        z = _unit(uo - lo)
        ref = tro - lo
        x = _unit(ref - np.dot(ref, z) * z)
        y = np.cross(z, x)
        R = np.column_stack([x, y, z])
        return R, origin

    def _tro_local_static(self):
        return self._R0.T @ (self.hp.tie_rod_outer - self._o0)

    # --------------------------------------------------------------------- #
    def _residuals(self, q, target_lower_z):
        """
        Unknowns q = [lo(3), uo(3), tro(3)]
        Constraints (10 eqns, 9 unknowns -> least squares):
          lower outer on both lower-arm spheres                (2)
          upper outer on both upper-arm spheres                (2)
          upright rigid |uo-lo|                                (1)
          tie-rod outer rigid to upright (dist to lo and uo)   (2)
          tie-rod length to inner pickup                       (1)
          lower-outer z drives travel                          (1)
        """
        hp = self.hp
        lo, uo, tro = q[0:3], q[3:6], q[6:9]
        r = [
            np.linalg.norm(lo - hp.lower_front_inner) - self.L_lower_f,
            np.linalg.norm(lo - hp.lower_rear_inner) - self.L_lower_r,
            np.linalg.norm(uo - hp.upper_front_inner) - self.L_upper_f,
            np.linalg.norm(uo - hp.upper_rear_inner) - self.L_upper_r,
            np.linalg.norm(uo - lo) - self.L_upright,
            np.linalg.norm(tro - lo) - self.L_tro_lo,
            np.linalg.norm(tro - uo) - self.L_tro_uo,
            np.linalg.norm(tro - hp.tie_rod_inner) - self.L_tie,
            lo[2] - target_lower_z,
        ]
        return np.array(r)

    def solve_at_travel(self, travel_mm: float, seed=None) -> CornerState:
        """
        Solve the linkage at a given wheel travel. `seed` optionally provides a
        warm-start vector [lo, uo, tro] from a nearby solved position — passing the
        previous step's solution keeps the solver on the correct configuration branch
        and prevents it from jumping to the mirror (flipped-linkage) solution at large
        travel. When seed is None it starts from the static pose.
        """
        hp = self.hp
        target_lower_z = hp.lower_outer[2] + travel_mm
        q0 = seed if seed is not None else np.concatenate(
            [hp.lower_outer, hp.upper_outer, hp.tie_rod_outer])
        sol = least_squares(
            self._residuals, q0, args=(target_lower_z,),
            method="lm", max_nfev=400, xtol=1e-12, ftol=1e-12,
        )
        lo, uo, tro = sol.x[0:3], sol.x[3:6], sol.x[6:9]
        max_resid = float(np.max(np.abs(sol.fun)))
        converged = max_resid < 0.1

        # Rigid pose of the upright in the solved configuration.
        R, o = self._upright_pose(lo, uo, tro)
        wc = o + R @ self._wc_local
        cp = o + R @ self._cp_local
        spin = R @ self._spin_local       # current wheel spin axis (outboard)

        camber = self._camber(spin)
        toe = self._toe(spin)
        caster, kpi = self._caster_kpi(uo, lo)
        scrub = self._scrub_radius(uo, lo, cp)
        ic = self._instant_center(uo, lo)

        return CornerState(
            travel=travel_mm, upper_outer=uo, lower_outer=lo, tie_rod_outer=tro,
            wheel_center=wc, contact_patch=cp, camber=camber, toe=toe,
            caster=caster, kpi=kpi, scrub_radius=scrub,
            instant_center=ic, roll_center_height=np.nan, converged=converged,
        )

    # ------------------------ kinematic outputs -------------------------- #
    def _camber(self, spin):
        # Camber = lean of the wheel plane. Wheel plane normal = spin axis. The wheel
        # plane's tilt from vertical equals the spin axis tilt from horizontal in the
        # front (y-z) view. Top-inboard => negative camber (racing convention).
        s = _unit(spin)
        ang = np.degrees(np.arctan2(s[2], abs(s[1])))
        return -ang

    def _toe(self, spin):
        # Toe = steer of the wheel plane in the top (x-y) view. Spin axis points
        # outboard (+y); its fore/aft component gives toe. Positive = toe-out.
        s = _unit(spin)
        return np.degrees(np.arctan2(s[0], abs(s[1])))

    def _caster_kpi(self, uo, lo):
        kp = uo - lo
        # Caster: side-view kingpin lean. Positive when the top of the kingpin is
        # rearward of the bottom (x rear-positive in SAE), giving self-centering.
        caster = np.degrees(np.arctan2(kp[0], kp[2]))
        # KPI: front-view kingpin lean. Positive when top leans inboard (toward
        # centreline). On a right corner inboard is -y, so top has smaller y.
        kpi = np.degrees(np.arctan2(-kp[1], kp[2]))
        return caster, kpi

    def _scrub_radius(self, uo, lo, cp):
        # distance in ground plane between kingpin-axis ground intersection and contact patch
        kp = _unit(uo - lo)
        if abs(kp[2]) < 1e-9:
            return np.nan
        t = -lo[2] / kp[2]
        ground = lo + t * kp
        return float(cp[1] - ground[1])

    def _instant_center(self, uo, lo):
        """Front-view instant centre (y,z) from the two wishbone projections."""
        hp = self.hp
        # upper arm line in y-z (use mean of front/rear inner pickups)
        u_in = 0.5 * (hp.upper_front_inner + hp.upper_rear_inner)
        l_in = 0.5 * (hp.lower_front_inner + hp.lower_rear_inner)
        # upper line: through uo and u_in (in y-z)
        p1, d1 = np.array([uo[1], uo[2]]), np.array([u_in[1] - uo[1], u_in[2] - uo[2]])
        p2, d2 = np.array([lo[1], lo[2]]), np.array([l_in[1] - lo[1], l_in[2] - lo[2]])
        # solve p1 + t d1 = p2 + s d2
        A = np.column_stack([d1, -d2])
        if abs(np.linalg.det(A)) < 1e-9:
            return np.array([np.nan, np.nan])
        ts = np.linalg.solve(A, p2 - p1)
        ic = p1 + ts[0] * d1
        return ic

    # ------------------------- sweep & metrics --------------------------- #
    def sweep(self, travel_min=-30.0, travel_max=30.0, n=41):
        """
        Solve across a travel range. Marches outward from the static position in both
        directions, warm-starting each step from the previous solved pose so the
        solver stays on the physically correct branch instead of risking a jump to the
        mirror configuration at the extremes. Results are returned in ascending travel.
        """
        travels = np.linspace(travel_min, travel_max, n)
        # split into droop side (descending from 0) and bump side (ascending from 0)
        below = sorted([t for t in travels if t < 0], reverse=True)
        above = sorted([t for t in travels if t > 0])
        zero = [t for t in travels if t == 0]

        results = {}
        # solve static first as the anchor seed
        static = self.solve_at_travel(0.0)
        seed0 = np.concatenate([static.lower_outer, static.upper_outer,
                                static.tie_rod_outer])
        for t in zero:
            results[t] = static

        seed = seed0
        for t in above:
            st = self.solve_at_travel(t, seed=seed)
            seed = np.concatenate([st.lower_outer, st.upper_outer, st.tie_rod_outer])
            results[t] = st

        seed = seed0
        for t in below:
            st = self.solve_at_travel(t, seed=seed)
            seed = np.concatenate([st.lower_outer, st.upper_outer, st.tie_rod_outer])
            results[t] = st

        return [results[t] for t in travels]

    def motion_ratio(self, spring_inner, rocker_pivot, push_outer_local=None):
        """
        Estimate installation/motion ratio = wheel travel / spring travel via finite
        difference of the pushrod outer point projected onto the pushrod axis.
        If pushrod outer isn't given, use the lower outer ball joint as a proxy
        pickup (common direct-acting layout).
        """
        d = 5.0
        s_up = self.solve_at_travel(d)
        s_dn = self.solve_at_travel(-d)
        if push_outer_local is None:
            p_up, p_dn = s_up.lower_outer, s_dn.lower_outer
        else:
            p_up = s_up.lower_outer + push_outer_local
            p_dn = s_dn.lower_outer + push_outer_local
        axis = _unit(np.array(spring_inner) - np.array(rocker_pivot))
        spring_disp = np.dot(p_up - p_dn, axis)
        wheel_disp = s_up.wheel_center[2] - s_dn.wheel_center[2]
        if abs(spring_disp) < 1e-9:
            return np.nan
        return abs(wheel_disp / spring_disp)
