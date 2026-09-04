# ============================================================================
#  KinematiK — suspension/aero/underfloor.py
#  Quasi-1D underfloor channel model.
#
#  WHY THIS EXISTS. Neither of the other two solvers can answer the question a
#  floor is designed to answer.
#
#  The panel method has no circulation, so it does not see the mechanism at
#  all: on a representative FSAE floor it reported 13 N at 60 mm and 14 N at
#  30 mm — a ground-effect device that does not respond to ride height.
#
#  The vortex lattice does have circulation and is credible around 60 mm, but
#  it treats the floor as a lifting surface with a mirror image, and inviscid
#  thin-surface theory in ground effect diverges as the gap closes. Measured on
#  the same floor: 293 N at 60 mm, 1,043 N at 50 mm, 2,941 N at 40 mm. That is
#  not steepening, it is a singularity, and it sits directly on top of the
#  30–50 mm band FSAE floors actually run in.
#
#  A floor does not make downforce by being a wing. It makes downforce by being
#  a duct: the gap between the underside and the road converges to a throat,
#  the flow speeds up, the static pressure drops, and the diffuser recovers it.
#  That is a continuity-and-Bernoulli problem, and modelling it directly gives
#  the right behaviour where it matters — because the inlet and the throat both
#  shrink as the car is lowered, their RATIO tends to a constant, so the
#  suction saturates instead of running away.
#
#  WHAT IT DOES NOT DO. It is one-dimensional: no spanwise pressure variation,
#  no vortices off the edges, no yaw sensitivity. It assumes the diffuser stays
#  attached and says so loudly when the ramp angle makes that doubtful, which
#  is the single assumption most likely to be wrong on a real car. It is a
#  screening model for gap geometry, not a substitute for CFD.
# ============================================================================
"""Underfloor downforce from the gap distribution, by quasi-1D continuity."""

from __future__ import annotations

import math
from dataclasses import dataclass

#: Equivalent conical half-angle beyond which a plane diffuser is widely taken
#: to be at risk of separating. Below this a healthy boundary layer generally
#: stays attached; above it the pressure recovery this model assumes is
#: optimistic and the real part will make less downforce than predicted.
SEPARATION_ANGLE_DEG = 7.0

#: Fraction of the ideal inlet mass flow that actually goes under the car
#: rather than spilling around the sides. 1.0 would mean perfectly skirted.
#: 0.75 is a common preliminary figure for a floor with turned-up edges; it
#: scales the suction roughly linearly, so treat it as the model's main knob.
DEFAULT_DISCHARGE = 0.75

#: How much more than its own area the inlet may capture before the surplus
#: spills around the car instead of going under it. The throat suction draws
#: the streamtube in, so some over-capture is real; without a ceiling a large
#: diffuser would pump an unphysical amount through a small inlet.
SPILL_LIMIT = 1.45


@dataclass
class UnderfloorResult:
    c_lift: float                   #: negative = downforce, on `ref_area`
    downforce_N: float              #: at the speed and density supplied
    throat_x_frac: float            #: where the gap is tightest, 0 = nose
    area_ratio: float               #: inlet area / throat area
    diffuser_angle_deg: float       #: equivalent conical half-angle
    attached: bool                  #: False once the ramp is past the limit
    notes: str


def _lower_surface(mesh, nx: int, ny: int):
    """Height of the underside on an (x, y) grid.

    Binning downward-facing triangles rather than ray casting: trimesh's ray
    engine needs `rtree`, which is an optional dependency here, and a solver
    that works or not depending on whether an optional wheel installed is not
    a solver anyone can rely on. Every triangle whose normal points down is a
    candidate underside; the lowest one in each cell wins.
    """
    import numpy as np

    lo, hi = mesh.bounds
    xs = np.linspace(lo[0], hi[0], nx)
    ys = np.linspace(lo[1], hi[1], ny)

    cen = np.asarray(mesh.triangles_center, dtype=float)
    nz = np.asarray(mesh.face_normals, dtype=float)[:, 2]
    down = nz < -0.05                                   # skip near-vertical walls
    if not down.any():
        return xs, ys, np.full((nx, ny), np.nan)
    c = cen[down]

    ix = np.clip(np.searchsorted(xs, c[:, 0]) - 1, 0, nx - 1)
    iy = np.clip(np.searchsorted(ys, c[:, 1]) - 1, 0, ny - 1)
    z = np.full((nx, ny), np.inf)
    np.minimum.at(z, (ix, iy), c[:, 2])
    z[~np.isfinite(z)] = np.nan
    return xs, ys, z




def _solve_channel(xs_l, area_l, width_l, speed_ms, ref_area_m2, rho,
                   discharge):
    """The physics, given a duct: station positions, flow area and wetted
    width. Split out so the STL path and the parametric what-if path solve
    exactly the same equations — a sweep that used a second implementation
    would eventually disagree with the file it was meant to be exploring.
    """
    import numpy as np

    a_in = float(area_l[0])
    i_throat = i_throat_pre = int(np.argmin(area_l))
    a_throat = float(area_l[i_throat])
    if a_throat <= 1e-9:
        raise ValueError("gap closes completely")

    #  THE DIFFUSER PUMPS. The flow rate used to be set by the inlet alone,
    #  which made exit height irrelevant to the force — 348 N at a 60 mm exit
    #  and 347 N at 225 mm. That is not how a floor works: the duct discharges
    #  into the base wake, so it is the EXIT that fixes the mass flow, and a
    #  bigger exit pulls more air under the car and deepens the throat suction.
    #
    #  Exit condition: static pressure pinned near ambient by the wake, so the
    #  exit velocity is the freestream less the duct's losses — `discharge` is
    #  that loss factor. The inlet then has to supply it, and it can
    #  over-capture somewhat because the suction draws the streamtube in, but
    #  not without limit: past roughly SPILL_LIMIT the air goes around the car
    #  instead. Whichever of the two binds, binds.
    a_exit = float(area_l[-1])
    a_flow = min(discharge * a_exit, SPILL_LIMIT * discharge * a_in)
    v_ratio = a_flow / np.maximum(area_l, 1e-9)
    cp = 1.0 - v_ratio ** 2                             # Bernoulli, incompressible

    #  PRESSURE RECOVERY IS LIMITED, AND THE EXIT IS NOT A PLENUM.
    #
    #  Applying continuity all the way to the trailing edge makes the diffuser
    #  decelerate the flow well below freestream, which puts Cp ABOVE zero over
    #  the whole rear of the floor and turned the net force into lift: +71 N at
    #  80 mm on the test floor. That is not what a diffuser does. Its exit
    #  discharges into the base wake, which pins the pressure there at roughly
    #  ambient however much area the ramp has.
    #
    #  So continuity governs up to the throat, and aft of it the pressure
    #  recovers linearly toward ambient, reaching (1 - eta) of the throat value
    #  at the exit. eta is the diffuser effectiveness: 1.0 would be perfect
    #  recovery to ambient, 0 none at all.
    _eta = 0.80
    _aft = np.arange(len(area_l)) >= i_throat_pre
    if _aft.any():
        _cp_t = float(cp[i_throat_pre])
        _x = np.asarray(xs_l)[_aft]
        _f = ((_x - _x[0]) / max(_x[-1] - _x[0], 1e-9)) if len(_x) > 1 else 0.0
        cp[_aft] = _cp_t * (1.0 - _eta * _f)

    #  QUASI-1D CONTINUITY. The mass flow that gets under the car is the inlet
    #  area times the freestream, reduced by whatever spills around the edges.
    #  Both a_in and a_throat scale with ride height, so their ratio — and
    #  therefore the peak suction — tends to a constant as the car is lowered.
    #  That is exactly the saturation the lattice lacks.


    q = 0.5 * rho * speed_ms ** 2
    dx = np.gradient(xs_l)
    lift_N = float(np.sum(cp * q * width_l * dx))       # cp<0 under suction
    c_lift = lift_N / (q * float(ref_area_m2))

    #  (a_exit computed above, where the flow rate is set.)
    #  DIFFUSER CHECK. Equivalent conical half-angle of the expansion from the
    #  throat to the exit — the standard screen for whether the pressure
    #  recovery assumed above is physically available.
    x_t, x_e = float(xs_l[i_throat]), float(xs_l[-1])
    length = max(x_e - x_t, 1e-6)
    r_t, r_e = math.sqrt(a_throat / math.pi), math.sqrt(a_exit / math.pi)
    angle = math.degrees(math.atan2(r_e - r_t, length))
    attached = angle <= SEPARATION_ANGLE_DEG

    span = float(xs_l[-1] - xs_l[0]) or 1.0
    notes = (f"quasi-1D underfloor: throat at {100*(x_t-xs_l[0])/span:.0f}% of "
             f"floor length, inlet/throat area ratio {a_in/a_throat:.2f}, "
             f"discharge coefficient {discharge:.2f}. Incompressible "
             f"continuity plus Bernoulli — one-dimensional, so no spanwise "
             f"variation, no edge vortices and no yaw sensitivity.")
    if not attached:
        notes += (f" WARNING: the diffuser expands at an equivalent "
                  f"{angle:.1f}° half-angle, past the {SEPARATION_ANGLE_DEG:.0f}° "
                  f"where a plane diffuser is normally taken to separate. This "
                  f"model assumes the flow stays attached and recovers the "
                  f"pressure, so the real part will make LESS than this. Treat "
                  f"the number as an upper bound and shallow the ramp.")
    else:
        notes += (f" Diffuser half-angle {angle:.1f}°, inside the "
                  f"{SEPARATION_ANGLE_DEG:.0f}° attachment limit.")

    return UnderfloorResult(c_lift=c_lift, downforce_N=-lift_N,
                            throat_x_frac=(x_t - xs_l[0]) / span,
                            area_ratio=a_in / a_throat,
                            diffuser_angle_deg=angle, attached=attached,
                            notes=notes)


def solve(geometry_path: str, ride_height_mm: float, speed_ms: float,
          ref_area_m2: float, rho: float = 1.225,
          discharge: float = DEFAULT_DISCHARGE,
          nx: int = 80, ny: int = 40) -> UnderfloorResult:
    """Downforce from the shape of the gap between the underside and the road.

    `ride_height_mm` is the clearance at the LOWEST point of the underside,
    which for a floor with a throat is the throat itself — the number a team
    actually sets on the car.
    """
    import numpy as np
    import trimesh

    mesh = trimesh.load(geometry_path, force="mesh")
    if mesh.is_empty or len(mesh.faces) == 0:
        raise ValueError("empty mesh")

    xs, ys, zlow = _lower_surface(mesh, nx, ny)
    if not np.isfinite(zlow).any():
        raise ValueError("no underside found — is the STL closed?")

    h = float(ride_height_mm) / 1000.0
    #  Gap under each sample point: the throat sits on the road at `h`, so
    #  every other station is that much higher by its own relief.
    relief = zlow - np.nanmin(zlow)
    gap = relief + h                                    # (nx, ny), metres

    dy = float(ys[1] - ys[0])
    covered = np.isfinite(gap)
    width = covered.sum(axis=1) * dy                    # wetted width per x
    area = np.where(covered, gap, 0.0).sum(axis=1) * dy  # duct area per x

    live = width > 1e-6
    if live.sum() < 4:
        raise ValueError("underside too narrow to form a duct")
    xs_l, area_l, width_l = xs[live], area[live], width[live]

    return _solve_channel(xs_l, area_l, width_l, speed_ms, ref_area_m2,
                          rho, discharge)



# ---------------------------------------------------------------------- #
#  Parametric what-if
#
#  The STL path answers "what does THIS floor do". Iterating on a design asks
#  a different question — where should the throat go, how steep can the ramp
#  be, what does 5 mm lower buy — and answering that by exporting an STL per
#  idea is slow enough that teams stop asking. The duct solve is milliseconds,
#  so the parameters can be swept directly.
#
#  Both paths call _solve_channel, so a sweep cannot drift away from the file
#  it is meant to be exploring.
#
#  ONE THING THE SWEEP WILL SHOW YOU AND YOU SHOULD NOT BELIEVE. Exit height
#  barely moves the predicted force — 348 N at a 60 mm exit and 347 N at
#  225 mm — because the mass flow here is set by the inlet and the diffuser is
#  credited only with RECOVERING pressure, not with pumping the throat. A real
#  diffuser does both: a bigger exit pulls more air under the car and deepens
#  the throat suction. So read exit height for its effect on the SEPARATION
#  flag, which is modelled, and not for its effect on downforce, which is not.
#  Throat position and ride height are the two the model genuinely resolves.
# ---------------------------------------------------------------------- #

#: The four numbers a floor is actually iterated on, with sane FSAE bounds.
PARAMS = {
    "throat_frac":    (0.20, 0.75, "throat position (fraction of length)"),
    "inlet_rise_mm":  (0.0, 120.0, "inlet height above the throat (mm)"),
    "exit_rise_mm":   (0.0, 250.0, "diffuser exit height above the throat (mm)"),
    "ride_height_mm": (15.0, 90.0, "throat clearance to the road (mm)"),
}


def channel_from_params(length_m=1.55, width_m=0.80, throat_frac=0.45,
                        inlet_rise_mm=55.0, exit_rise_mm=115.0,
                        ride_height_mm=40.0, edge_fence=0.55, nx=120):
    """Duct geometry from the parameters, without going near a mesh.

    `ride_height_mm` is the clearance at the throat, so lowering the car moves
    the whole gap distribution down by a constant — which is what actually
    happens, and what makes the inlet/throat ratio climb.
    """
    import numpy as np

    xs = np.linspace(0.0, float(length_m), int(nx))
    xf = xs / float(length_m)
    tf = float(np.clip(throat_frac, 0.05, 0.95))

    relief = np.empty_like(xf)
    nose = xf < tf
    relief[nose] = (inlet_rise_mm / 1000.0) * (1.0 - xf[nose] / tf) ** 1.6
    aft = ~nose
    relief[aft] = (exit_rise_mm / 1000.0) * (
        (xf[aft] - tf) / max(1.0 - tf, 1e-6)) ** 1.35

    gap = relief + float(ride_height_mm) / 1000.0
    #  Turned-up edges: the same shape the STL generator uses, so a swept
    #  result and a solved export of the same numbers are comparable.
    #  Turned-up edges do two things: they seal the gap at the sides, and they
    #  take area out of the duct. The factor below is calibrated against the
    #  meshed floor in the sample set — a flat `width * gap` overstated the
    #  duct by about 55% and made the parametric sweep read roughly twice the
    #  force of solving the same floor as an STL, which is the worst possible
    #  discrepancy to leave inside one tool.
    #
    #      measured on fsae_floor at 40 mm    A_in     A_throat   A_exit
    #        meshed STL                       0.0413    0.0208    0.0660
    #        parametric, uncalibrated         0.0635    0.0267    0.1035
    #
    #  A single fence factor gets the inlet right and the throat wrong — a
    #  real turned-up edge takes more area out where the gap is deep than
    #  where it is tight, and chasing that with a second fitted term is
    #  curve-fitting one sample, not physics. So the shape here is nominal and
    #  `area_scale` is the honest fix: see calibration_scale(), which ties the
    #  sweep to the STL the member actually uploaded instead of to a generic
    #  floor. Sweeping perturbations around their own part is also the more
    #  useful thing to offer.
    width = np.full_like(xs, float(width_m))
    area = gap * width * (1.0 - 0.84 * edge_fence)
    return xs, area, width


def solve_parametric(speed_ms=20.0, ref_area_m2=1.24, rho=1.225,
                     discharge=DEFAULT_DISCHARGE, **geom) -> UnderfloorResult:
    """Same physics as :func:`solve`, on a duct described by numbers."""
    xs, area, width = channel_from_params(**geom)
    return _solve_channel(xs, area, width, speed_ms, ref_area_m2, rho,
                          discharge)


def sweep(param_x: str, values_x, param_y: str = None, values_y=None,
          speed_ms=20.0, ref_area_m2=1.24, discharge=DEFAULT_DISCHARGE,
          baseline=None, **fixed):
    """Solve a 1-D or 2-D grid and return rows ready for a table or heatmap.

    Each row carries `attached` as well as the force, because the highest
    downforce in a sweep is almost always the steepest diffuser — which is
    also the one most likely to separate. A sweep that reported only the
    number would steer every team to the same wrong corner.
    """
    rows = []
    ys = values_y if param_y else [None]
    for vy in ys:
        for vx in values_x:
            g = dict(fixed)
            g[param_x] = vx
            if param_y:
                g[param_y] = vy
            try:
                r = solve_parametric(speed_ms=speed_ms,
                                     ref_area_m2=ref_area_m2,
                                     discharge=discharge, **g)
            except Exception:                            # noqa: BLE001
                continue
            row = {param_x: vx, "C_L": round(r.c_lift, 4),
                   "downforce_N": round(r.downforce_N, 1),
                   "inlet/throat": round(r.area_ratio, 2),
                   "diffuser_deg": round(r.diffuser_angle_deg, 1),
                   "attached": r.attached}
            if param_y:
                row[param_y] = vy
            rows.append(row)

    #  RELATIVE, BECAUSE THAT IS WHAT THIS MODEL SUPPORTS.
    #
    #  The parametric duct is an idealisation of a meshed one and the two do
    #  not agree on magnitude — measured on the sample floor, the sweep runs
    #  about 1.5x the STL solve, and the gap widens as the car is lowered. I
    #  tried to calibrate it away by matching areas and it does not work: a
    #  uniform area scale cancels out of the velocity ratio entirely, and
    #  matching the inlet-to-throat ratio instead only closes part of it,
    #  because a turned-up edge takes area out of a 1-D duct in a way one
    #  fitted constant cannot describe. Chasing it further would be fitting one
    #  sample floor and calling it physics.
    #
    #  So the sweep reports CHANGE against its own baseline cell. That is the
    #  question being asked — which way do I move — and it is the part the
    #  model gets right: both paths are monotone in the same direction with the
    #  same ranking. Absolute newtons come from solving the STL.
    if rows:
        base = None
        if baseline is not None:
            base = next((r for r in rows
                         if all(r.get(k) == v for k, v in baseline.items())),
                        None)
        if base is None:
            base = min(rows, key=lambda r: abs(r["downforce_N"]
                                               - _median([q["downforce_N"]
                                                          for q in rows])))
        b = base["downforce_N"] or 1.0
        for r in rows:
            r["vs_baseline_pct"] = round(100.0 * (r["downforce_N"] - b) / b, 1)
    return rows


def _median(vals):
    v = sorted(vals)
    n = len(v)
    return v[n // 2] if n % 2 else 0.5 * (v[n // 2 - 1] + v[n // 2])


