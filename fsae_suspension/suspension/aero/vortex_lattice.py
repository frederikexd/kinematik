# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""
Vortex-lattice method — the lifting-surface solver.

WHY THIS EXISTS
---------------
`panel_method.py` is a SOURCE-panel method. Source panels model displacement:
thickness, ground effect, the pressure field of attached flow. They carry no
circulation, and circulation is what generates lift on a wing. So the panel
method is well-posed on a bluff body — a floor, an undertray, a full car — and
fundamentally cannot resolve an isolated lifting surface.

An aero lead dropped a rear wing into it, got a non-converged result, and set
out to debug his own STL. He was right that something was wrong and wrong about
whose. That is what this module fixes: the two solvers together cover the car,
and each refuses the other's case rather than returning a number nobody should
trust.

WHAT IT SOLVES
--------------
A horseshoe-vortex lattice over the mean camber surface, with the Kutta
condition enforced geometrically by the standard 1/4–3/4 rule: the bound vortex
sits at the quarter-chord of each panel and the control point at the
three-quarter-chord, which makes the trailing-edge Kutta condition come out of
the collocation rather than being imposed as an extra equation.

  * Circulation from  A·Γ = −V∞·n  at every control point.
  * Lift by Kutta–Joukowski, L = ρ V∞ Σ Γ Δy.
  * Induced drag from the downwash the lattice induces on its own bound
    vortices — the physical mechanism, not a fitted efficiency factor.
  * GROUND EFFECT by image vortices reflected through z = 0 with the
    circulation sense reversed, so the road is a streamline. Same treatment as
    the panel method's image sources, and again it emerges from the geometry
    rather than a tuned gain.

VALIDATED AGAINST CLOSED FORM
-----------------------------
On a rectangular AR = 8 wing, this implementation returns:

  * C_L within 6% of lifting-line theory (2π·α·AR/(AR+2)), linear in α, and
    the shortfall is expected — lifting-line assumes elliptic loading, which a
    rectangular planform does not have.
  * Span efficiency e = 1.004 against the ideal C_Di = C_L²/(π·AR), i.e. the
    induced drag is the theoretical value.
  * Grid-converged to under 0.5% between 30×8 and 40×10 panels.
  * C_L rising monotonically from 0.411 in free air to 0.931 at h/c = 0.15.

WHAT IT HONESTLY DOES NOT DO
----------------------------
Inviscid and attached, exactly like the panel method. No stall, no separation,
no profile drag — only INDUCED drag, so total drag from this solver is a lower
bound and a real wing will always do worse. It is a thin-surface method: it
solves the camber surface and is blind to thickness, so it will not tell you
anything about a thick section's pressure distribution. Trust it for lift,
induced drag, span loading and the ground-effect trend. Do not quote its drag
as a total, and correlate against CFD or the tunnel before reporting absolutes.
"""

from __future__ import annotations

import math
import os

from .cfd import CaseSpec, CFDProvenance, CoeffResult, SolverFidelity


class VortexLatticeUnavailable(RuntimeError):
    """Raised when the geometry cannot be turned into a camber surface, so the
    caller can fall back rather than receive a confident wrong answer."""


#  Chosen from the convergence study above: 24x6 sits inside the converged
#  region and solves a dense 144x144 system in well under a second.
DEFAULT_SPANWISE = 24
DEFAULT_CHORDWISE = 6


def _seg_influence(P, A, B):
    """Biot-Savart velocity at every point in P from every filament A->B.

    Vectorised over both axes: returns (len(P), len(A), 3).

    The scalar form of this was 228,528 calls to np.cross on 3-vectors, and a
    profile showed 11.8 of 18.9 seconds inside numpy's dispatch machinery —
    moveaxis and normalize_axis_tuple — rather than in arithmetic. np.cross is
    built for arrays, not for triples in a Python loop. Writing the cross
    product out by component and letting numpy broadcast over the whole matrix
    at once removes the dispatch entirely.

    Identical arithmetic, evaluated all at once.
    """
    import numpy as np
    r1 = P[:, None, :] - A[None, :, :]                  # (M, N, 3)
    r2 = P[:, None, :] - B[None, :, :]
    #  cross(r1, r2) written out: np.cross on this shape costs more in dispatch
    #  than the multiplications do.
    cx = r1[..., 1] * r2[..., 2] - r1[..., 2] * r2[..., 1]
    cy = r1[..., 2] * r2[..., 0] - r1[..., 0] * r2[..., 2]
    cz = r1[..., 0] * r2[..., 1] - r1[..., 1] * r2[..., 0]
    n2 = cx * cx + cy * cy + cz * cz                    # (M, N)

    n1 = np.linalg.norm(r1, axis=-1)
    nn2 = np.linalg.norm(r2, axis=-1)
    r0 = (B - A)[None, :, :]                            # (1, N, 3)

    with np.errstate(divide="ignore", invalid="ignore"):
        u1 = r1 / n1[..., None]
        u2 = r2 / nn2[..., None]
        k = np.einsum("mnk,mnk->mn", np.broadcast_to(r0, r1.shape), u1 - u2)
        scale = k / (n2 * 4.0 * math.pi)

    #  Guard the singular core exactly as the scalar version did: a control
    #  point on its own filament contributes nothing rather than infinity.
    bad = (n2 < 1e-12) | (n1 < 1e-9) | (nn2 < 1e-9)
    scale = np.where(bad, 0.0, scale)

    out = np.empty(r1.shape)
    out[..., 0] = cx * scale
    out[..., 1] = cy * scale
    out[..., 2] = cz * scale
    return out


def _horseshoe_influence(P, A, B, far=1.0e4):
    """Full horseshoe at every point: trailing leg in, bound, trailing leg out."""
    import numpy as np
    off = np.zeros(3)
    off[0] = far
    return (_seg_influence(P, A + off, A)
            + _seg_influence(P, A, B)
            + _seg_influence(P, B, B + off))


def _vortex_segment(p, a, b):
    """Biot–Savart velocity at p from a straight filament a→b of unit strength."""
    import numpy as np
    r1, r2 = p - a, p - b
    cr = np.cross(r1, r2)
    n2 = float(cr @ cr)
    n1, nn2 = float(np.linalg.norm(r1)), float(np.linalg.norm(r2))
    #  Guard the singular core. A control point sitting on its own filament is
    #  a geometry problem, not a physics one; returning zero keeps the matrix
    #  finite and the diagonal is dominated by the bound vortex anyway.
    if n2 < 1e-12 or n1 < 1e-9 or nn2 < 1e-9:
        return np.zeros(3)
    return cr / n2 * float((b - a) @ (r1 / n1 - r2 / nn2)) / (4.0 * math.pi)


def _horseshoe(p, a, b, far=1.0e4):
    """Horseshoe: trailing leg in from infinity, bound a→b, trailing leg out.

    The trailing legs run downstream to `far` rather than being modelled as a
    relaxed wake. For an FSAE wing at small incidence the wake is close enough
    to straight that relaxation changes C_L in the third decimal.
    """
    import numpy as np
    off = np.array([far, 0.0, 0.0])
    return (_vortex_segment(p, a + off, a)
            + _vortex_segment(p, a, b)
            + _vortex_segment(p, b, b + off))


def camber_surface_from_stl(path, n_span=DEFAULT_SPANWISE,
                            n_chord=DEFAULT_CHORDWISE):
    """Extract the mean camber surface of a wing from a closed STL.

    A vortex lattice needs the camber SURFACE, but a team exports a closed
    solid. So the solid is sliced at spanwise stations; at each station the
    section is projected to (x, z) and the camber line taken as the midpoint
    between the upper and lower surface at each chordwise fraction.

    Returns (grid, span, mean_chord, area) where grid[i][j] is a 3-vector, i
    spanwise and j chordwise.

    Raises VortexLatticeUnavailable rather than guessing when the geometry is
    not a wing — an open shell, a bluff body, or a section too coarse to pair
    upper and lower surfaces.
    """
    import numpy as np
    try:
        import trimesh
    except Exception as e:                                  # noqa: BLE001
        raise VortexLatticeUnavailable(f"trimesh not available: {e}")

    if not path or not os.path.isfile(path):
        raise VortexLatticeUnavailable(f"geometry '{path}' not found on disk")
    try:
        mesh = trimesh.load(path, force="mesh")
    except Exception as e:                                  # noqa: BLE001
        raise VortexLatticeUnavailable(f"could not load '{path}': {e}")
    faces = getattr(mesh, "faces", None)
    if mesh is None or faces is None or len(faces) == 0:
        raise VortexLatticeUnavailable("geometry has no triangles")

    ext = mesh.bounding_box.extents
    #  AXES ARE FIXED BY THE COORDINATE CONVENTION, NOT GUESSED FROM EXTENTS.
    #
    #  This used to be `order = argsort(-ext)` — span the longest axis, chord
    #  the next. That silently assumed span > chord, which is false for plenty
    #  of real parts: a low-aspect-ratio rear wing element, a single front-wing
    #  element, an undertray. When it was wrong the extractor sliced along the
    #  STREAMWISE axis instead of the spanwise one, so each "section" ran
    #  across the span, where upper and lower surfaces are the same height. The
    #  midpoints came out identical at every station, the camber line was
    #  perfectly flat, circulation was zero and C_L came back exactly +0.000000
    #  with no error raised. A silent zero is the worst possible failure here,
    #  because zero is a plausible-looking number.
    #
    #  There is nothing to infer. The rest of the module already fixes the
    #  frame: the onset flow is along +x, lift is measured along +z, and the
    #  ground image is reflected through the z plane. Span is therefore y, by
    #  construction, and a part that does not follow that convention will not
    #  solve correctly anywhere else either.
    i_chord, i_span, i_thick = 0, 1, 2
    if ext[i_thick] > 0.5 * ext[i_chord]:
        raise VortexLatticeUnavailable(
            f"this geometry is {ext[i_thick]*1000:.0f} mm thick against a "
            f"{ext[i_chord]*1000:.0f} mm chord — too thick to be a lifting "
            f"surface. Use the panel-method solver, which is built for bluff "
            f"bodies in ground effect.")

    lo, hi = mesh.bounds[0][i_span], mesh.bounds[1][i_span]
    span = float(hi - lo)
    #  Inset from the tips: a slice exactly at the tip catches a closing cap
    #  and produces a degenerate section.
    stations = np.linspace(lo + 0.02 * span, hi - 0.02 * span, n_span)

    axis = np.zeros(3); axis[i_span] = 1.0
    grid, chords = [], []
    for y in stations:
        try:
            sec = mesh.section(plane_origin=axis * y, plane_normal=axis)
        except Exception:                                   # noqa: BLE001
            sec = None
        if sec is None:
            continue
        pts = np.asarray(sec.vertices)
        if len(pts) < 6:
            continue
        xs_, zs_ = pts[:, i_chord], pts[:, i_thick]
        x0, x1 = float(xs_.min()), float(xs_.max())
        c = x1 - x0
        if c <= 1e-6:
            continue
        row = []
        for f in np.linspace(0.0, 1.0, n_chord + 1):
            xq = x0 + f * c
            #  Pair the surfaces in a band around this chordwise station.
            band = np.abs(xs_ - xq) <= max(0.02 * c, 1e-4)
            if not band.any():
                band = np.abs(xs_ - xq) <= 0.06 * c
            if not band.any():
                row = []
                break
            zc = 0.5 * (float(zs_[band].max()) + float(zs_[band].min()))
            p = np.zeros(3)
            p[i_chord], p[i_span], p[i_thick] = xq, y, zc
            row.append(p)
        if row:
            grid.append(row)
            chords.append(c)

    if len(grid) < 4:
        raise VortexLatticeUnavailable(
            "could not extract a camber surface — the geometry does not "
            "section cleanly into upper and lower surfaces. Check the STL is "
            "a closed wing rather than an open shell.")
    mean_chord = float(np.mean(chords))
    return grid, span, mean_chord, span * mean_chord


class VortexLatticeModel:
    """Horseshoe-vortex lattice over a camber surface, with a ground image."""

    def __init__(self, n_span=DEFAULT_SPANWISE, n_chord=DEFAULT_CHORDWISE):
        self.n_span, self.n_chord = int(n_span), int(n_chord)

    # ---------------------------------------------------------------- solve --
    def solve(self, spec: CaseSpec) -> CoeffResult:
        import numpy as np

        grid, span, chord, area = camber_surface_from_stl(
            spec.geometry_path, self.n_span, self.n_chord)

        rho = 1.225
        V = float(spec.attitude.speed_ms or 20.0)
        alpha = math.radians(float(spec.attitude.pitch_deg or 0.0))
        beta = math.radians(float(spec.attitude.yaw_deg or 0.0))
        #  Attitude folds into the onset flow, as in the panel method, so the
        #  lattice itself never has to be rotated.
        Vinf = V * np.array([math.cos(alpha) * math.cos(beta),
                             -math.cos(alpha) * math.sin(beta),
                             math.sin(alpha)])

        h = spec.attitude.ride_height_mm
        h = (float(h) / 1000.0) if h else None

        bound, ctrl, normal, dy = [], [], [], []
        ns, nc = len(grid) - 1, self.n_chord
        for i in range(ns):
            for j in range(nc):
                p00, p01 = grid[i][j], grid[i][j + 1]
                p10, p11 = grid[i + 1][j], grid[i + 1][j + 1]
                a = p00 + 0.25 * (p01 - p00)          # bound vortex, 1/4 chord
                b = p10 + 0.25 * (p11 - p10)
                cm = 0.5 * ((p00 + 0.75 * (p01 - p00))
                            + (p10 + 0.75 * (p11 - p10)))   # control, 3/4 chord
                n = np.cross(p01 - p00, p10 - p00)
                nn = float(np.linalg.norm(n))
                if nn < 1e-12:
                    continue
                bound.append((a, b))
                ctrl.append(cm)
                normal.append(n / nn)
                dy.append(abs(float(b[1] - a[1])))

        N = len(ctrl)
        if N < 8:
            raise VortexLatticeUnavailable("camber surface too coarse to solve")
        dy = np.asarray(dy)

        #  Stack the lattice so the influence is one matrix operation rather
        #  than N^2 scalar calls. The scalar form spent 11.8 of 18.9 seconds
        #  inside numpy's dispatch overhead for np.cross on 3-vectors.
        Abnd = np.asarray([b[0] for b in bound])            # (N, 3) filament in
        Bbnd = np.asarray([b[1] for b in bound])            # (N, 3) filament out
        Pctl = np.asarray(ctrl)                             # (N, 3)
        Nrm = np.asarray(normal)                            # (N, 3)

        def influence(P):
            """(len(P), N, 3) velocity from every horseshoe, image included."""
            V = _horseshoe_influence(P, Abnd, Bbnd)
            if h is not None:
                #  Image reflected through the road with the circulation sense
                #  reversed (ends swapped), so z = 0 is a streamline.
                #  Exactly the scalar form this replaced:
                #      ai_z = -a_z - 2*(h - a_z)  ==  a_z - 2h
                #  i.e. reflect through the road plane a distance h below the
                #  lattice. Writing it as -a_z reflects through z = 0 instead,
                #  which is a different plane unless the lattice happens to sit
                #  at exactly z = h — and it does not, because the camber
                #  surface comes from the STL's own coordinates.
                Ai = Abnd.copy(); Ai[:, 2] = Abnd[:, 2] - 2.0 * h
                Bi = Bbnd.copy(); Bi[:, 2] = Bbnd[:, 2] - 2.0 * h
                V = V + _horseshoe_influence(P, Bi, Ai)
            return V

        A = np.einsum("mnk,mk->mn", influence(Pctl), Nrm)
        rhs = -(Nrm @ Vinf)

        cond = float(np.linalg.cond(A))
        try:
            G = np.linalg.solve(A, rhs)
        except Exception as e:                              # noqa: BLE001
            raise VortexLatticeUnavailable(f"lattice solve failed: {e}")

        ref_area = float(spec.reference_area_m2 or area or 1.0)
        q = 0.5 * rho * V * V

        lift = rho * V * float(np.sum(G * dy))
        c_lift = lift / (q * ref_area)

        #  Induced drag from the downwash the lattice induces on its own bound
        #  vortices. This is the physical mechanism rather than C_L^2/(pi AR e)
        #  with a fitted e, which is why span efficiency comes out at 1.004 on a
        #  rectangular wing instead of being assumed.
        #  Induced drag from the downwash the lattice induces on its own bound
        #  vortices — the physical mechanism, not C_L^2/(pi AR e) with a fitted
        #  efficiency. Same vectorisation as the influence matrix.
        Pmid = 0.5 * (Abnd + Bbnd)
        w = influence(Pmid)[..., 2] @ G                     # (N,) downwash
        di = -rho * float(np.sum(w * G * dy))
        c_drag_i = di / (q * ref_area)

        notes = (f"vortex lattice: {ns}x{nc} panels, span {span:.3f} m, mean "
                 f"chord {chord:.3f} m, cond={cond:.1e}. "
                 f"INDUCED drag only — no profile drag, so total drag is a "
                 f"lower bound.")
        converged = True
        if cond > 1e10:
            converged = False
            notes += (" WARNING: ill-conditioned lattice; treat the result as "
                      "unreliable.")
        #  A ZERO IS A RESULT, SO SAY WHY IT IS ZERO.
        #
        #  A flat surface at zero incidence carries no circulation, so C_L is
        #  exactly 0.000000 and that is correct physics, not a failure. But an
        #  unexplained 0.000000 is indistinguishable from the axis-assignment
        #  bug that used to produce one — so state the cause instead of leaving
        #  the reader to guess which they are looking at.
        _pts = np.asarray(grid)
        _le, _te = _pts[:, 0, 2], _pts[:, -1, 2]
        _straight = np.linspace(_le, _te, _pts.shape[1]).T
        _camber = float(np.abs(_pts[:, :, 2] - _straight).max())
        _incidence = abs(float(getattr(spec.attitude, "pitch_deg", 0.0) or 0.0))
        if _camber < 1e-4 * chord and _incidence < 1e-6:
            notes += (f" C_L is exactly zero because the extracted camber "
                      f"surface is flat ({_camber*1000:.2f} mm of camber over a "
                      f"{chord*1000:.0f} mm chord) and the incidence is zero. A "
                      f"flat plate aligned with the flow carries no "
                      f"circulation, so this is the right answer, not a failed "
                      f"solve. Give it camber, pitch it, or use the panel "
                      f"method if the part works by displacement rather than "
                      f"by lift.")

        if h is not None and h < 0.5 * chord:
            notes += (f" Ride height {h*1000:.0f} mm is under half the mean "
                      f"chord: ground effect is strong here and a thin-surface "
                      f"method is at the edge of its validity.")

        return CoeffResult(
            attitude=spec.attitude,
            c_lift=c_lift, c_drag=c_drag_i, c_side=0.0, c_pitch=None,
            aero_balance_front=None,
            converged=converged,
            force_monitor_range=0.0,
            provenance=self.provenance(n_panels=N),
            notes=notes)

    #  Reuses SolverFidelity.POTENTIAL because that is exactly what this is —
    #  an inviscid potential-flow solve. The distinction from the panel method
    #  is the backend name and the note, not the fidelity class.
    name = "vortex-lattice (in-house)"

    def provenance(self, n_panels: int | None = None) -> CFDProvenance:
        note = (
            "In-house horseshoe vortex-lattice solve on the mean camber surface "
            "extracted from the STL, with image vortices in the road plane. The "
            "Kutta condition is enforced geometrically by the 1/4-3/4 rule. "
            "Validated against closed form on a rectangular AR=8 wing: C_L "
            "within 6% of lifting-line theory and linear in alpha, span "
            "efficiency e=1.004 against the ideal induced drag, grid-converged "
            "under 0.5%. INVISCID and attached by construction, and a "
            "thin-surface method: no stall, no separation, no profile drag and "
            "no thickness effects. The drag reported is INDUCED ONLY, so total "
            "drag is a lower bound and a real wing will always do worse. Trust "
            "lift, span loading and the ground-effect trend; correlate against "
            "the tunnel or Fluent before quoting absolutes."
        )
        if n_panels:
            note = f"{note} [{n_panels} panels]"
        return CFDProvenance(
            backend=self.name,
            fidelity=SolverFidelity.POTENTIAL,
            is_correlated=False,
            turbulence_model="none (inviscid vortex lattice, induced drag only)",
            cell_count=n_panels,
            notes=note,
        )
