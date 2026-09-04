# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""
Higher-fidelity in-house aero: a 3D source-panel (boundary-element) potential-flow
solver with a ground plane, run on the team's actual STL geometry.

WHY THIS MODULE EXISTS (read this before trusting its numbers)
--------------------------------------------------------------
The default in-house backend (`ReferenceAeroModel`) is an analytic SURROGATE: a
handful of FSAE-plausible sensitivities curve-fitted to attitude. It is honest about
being a stand-in, but its coefficients come from the fit, not from the car's shape —
change the geometry and the numbers do not move. That is fine for plumbing and
trends, useless for "does this new floor make more downforce".

This module is the genuine fidelity step between that surrogate and an external RANS
solve. It SOLVES a flow on the real surface mesh:

  * It reads the STL (via trimesh), places the car at the requested attitude
    (yaw/pitch fold into the onset flow; roll + ride height move the body over the
    road), and treats each triangle as a constant-strength SOURCE panel.
  * It enforces flow tangency (zero normal velocity) on every panel, giving a dense
    linear system A·sigma = -Vinf·n that is solved once per attitude.
  * GROUND EFFECT is modelled correctly, not tuned: an IMAGE of every panel is
    reflected through the road plane (z=0), so the road is an exact streamline. Lower
    the car and the image interaction strengthens — ground effect emerges from the
    physics, it is not a `ride_ground_gain` constant.
  * Surface pressure comes from the computed tangential velocity through Bernoulli
    (Cp = 1 - (V/Vinf)^2). Lift and the pressure (form) part of drag are the surface
    integral of Cp·n. A flat-plate turbulent SKIN-FRICTION estimate is added so total
    C_d is realistic rather than the near-zero a pure potential solve would give.

WHAT IT RESOLVES — AND WHAT IT HONESTLY DOES NOT
------------------------------------------------
Potential flow is inviscid and attached by assumption. So this method captures, from
geometry: ground effect, the pressure field of attached flow, the downforce trend
with rake and ride height, and induced effects. It does NOT capture viscous
SEPARATION, a real turbulent wake, stall, or vortex shedding — exactly the things a
RANS/DES solve and the written ANSYS Fluent deck exist to check. Its fidelity is
therefore labelled `POTENTIAL` (a resolved potential field, well above the analytic
surrogate, well below RANS), and `is_correlated=False`. The provenance says all of
this on every result. Trust deltas between geometries far more than absolute levels,
and correlate against the tunnel/Fluent before reporting an absolute number.

DELIBERATE NON-GOALS: no meshing of a volume, no Navier–Stokes, no turbulence model.
It is a surface BEM on the geometry the team already supplies, sized to run in
seconds so a sweep is interactive.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass

from .cfd import (
    Attitude, CaseSpec, CoeffResult, CFDProvenance, SolverFidelity,
)


# --------------------------------------------------------------------------- #
#  Tunables for the panel solve (sane FSAE-scale defaults)
# --------------------------------------------------------------------------- #
@dataclass
class PanelParams:
    """
    Knobs for the panel solve. Defaults trade a little accuracy for an interactive
    runtime so a whole ride-height sweep stays in the seconds range.

    max_panels        : the STL is decimated to at most this many triangles before
                        solving (the linear system is dense and O(N^2) memory, so
                        this is the cost lever). None => use the mesh as-is.
    ground_effect     : reflect an image of every panel through z=0 so the road is an
                        exact streamline. The whole point of a ground-effect car.
    road_plane_z_m    : the road height in the STL's own coordinates (default 0.0).
    kin_viscosity     : air kinematic viscosity for the skin-friction Reynolds number.
    laminar_fraction  : fraction of the body length assumed laminar before transition
                        (cuts the flat-plate Cf slightly; 0.0 = fully turbulent).
    min_panels        : below this many usable triangles the geometry is too coarse to
                        trust a solve, and the caller should fall back.
    """
    max_panels: int | None = 8000
    ground_effect: bool = True
    road_plane_z_m: float = 0.0
    kin_viscosity: float = 1.5e-5
    laminar_fraction: float = 0.05
    min_panels: int = 24
    lifting: bool = True
    """Add circulation by superposing a vortex lattice on the camber surface.

    Source panels model displacement and carry no circulation, so on their own
    they cannot produce lift: on a closed body in free air the pressure
    integral cancels to machine zero by d'Alembert, which the sphere test
    asserts deliberately. Every newton the source solve reported on a wing or a
    floor came from ground-image asymmetry alone, which is why it read 14 N on
    a floor making a few hundred and barely moved with ride height.

    Thickness and camber superpose in potential flow — it is the same
    decomposition thin-aerofoil theory rests on. So the sources keep doing what
    they are good at (displacement, blockage, ground proximity on the real
    wetted surface, skin friction) and a lattice on the extracted mean camber
    surface supplies the circulation. Neither part is new code; what was
    missing was adding them together.

    Set False to recover the pure displacement solve — worth doing on a bluff
    body where there is no camber surface to find and the lattice would be
    fitting noise.
    """


#: Peak the panel solve may allocate. Streamlit Community Cloud guarantees
#: 690 MB and the app's own baseline is around 250 MB, so this leaves headroom
#: for one solve without putting the container at risk of an OOM kill that
#: would drop every concurrent viewer.
_MEMORY_BUDGET_MB = 900.0


class PanelMethodUnavailable(RuntimeError):
    """
    Raised when the panel solve cannot run for a SPECIFIC, reportable reason —
    geometry file missing/empty, trimesh not installed, the surface too coarse, or
    the linear solve failing. Carries an actionable message so the caller can fall
    back to the analytic surrogate transparently rather than fabricating a number.
    """


# --------------------------------------------------------------------------- #
#  The solver
# --------------------------------------------------------------------------- #
class PanelMethodModel:
    """
    A 3D constant-source-panel potential-flow model with a ground image, evaluated on
    the supplied STL. Implements the same `provenance()/write_case/run_case/
    read_result` shape as the other in-house backends, but it is normally used as the
    PHYSICS ENGINE inside a higher-level backend (it does not, by itself, write any
    solver deck). `FluentVerificationSolver(method="panel")` wraps it and adds the
    Fluent verification journal.

    Sign convention (matches cfd.py): c_lift NEGATIVE = downforce.
    """
    name = "panel-method"

    def __init__(self, params: PanelParams | None = None):
        self.params = params or PanelParams()

    # -- provenance -------------------------------------------------------- #
    def provenance(self, n_panels: int | None = None) -> CFDProvenance:
        note = (
            "In-house 3D source-panel (boundary-element) potential-flow solve on the "
            "actual STL, with a ground-image plane. Resolves the attached-flow "
            "pressure field and ground effect FROM GEOMETRY (not a curve fit), so "
            "geometry deltas are meaningful. Inviscid by construction: it does NOT "
            "resolve viscous separation, stall, or the real turbulent wake — that is "
            "what the written ANSYS Fluent deck / a RANS solve is for. Skin friction "
            "is a flat-plate estimate. Correlate against the tunnel/Fluent before "
            "trusting absolute levels; trust deltas more than levels."
        )
        if n_panels:
            note = f"{note} [{n_panels} panels]"
        return CFDProvenance(
            backend=self.name,
            fidelity=SolverFidelity.POTENTIAL,
            is_correlated=False,
            turbulence_model="none (inviscid potential flow + flat-plate friction)",
            cell_count=n_panels,
            notes=note,
        )

    # -- the public physics entry point ------------------------------------ #
    def solve(self, spec: CaseSpec) -> CoeffResult:
        """
        Solve one attitude on the STL and return its coefficients. Raises
        PanelMethodUnavailable (never a fabricated number) if the geometry cannot be
        loaded or is too coarse to solve.
        """
        import numpy as np

        centroids, normals, areas, length_ref, tris, ride_gap = self._load_panels(spec)
        n = len(areas)

        # Onset flow: yaw about +z, pitch about +y, unit magnitude (coeffs are
        # non-dimensional, so we work at |Vinf| = 1 and scale out cleanly).
        vinf = _freestream_unit(spec.attitude)

        # Influence matrix: normal velocity at panel i induced by unit source on
        # panel j (plus its ground image), in a point-source approximation evaluated
        # at panel centroids. A[i,j] = n_i · (u_ij + u_image_ij).
        #  REFUSE BEFORE ALLOCATING, NOT AFTER.
        #
        #  The influence matrix is N x N float32 and the assembly needs a few
        #  row blocks of workspace on top. Measured peaks: 3,072 panels 303 MB,
        #  12,288 panels 3,046 MB — clean N^2. An OOM kill inside a Streamlit
        #  script run takes the whole app down for every viewer, not just the
        #  one who asked, so a refusal with a number is strictly better than
        #  finding out by dying.
        _need_mb = (n * n * 4) / 1048576.0 * 2.6            # matrix + workspace
        if _need_mb > _MEMORY_BUDGET_MB:
            raise PanelMethodUnavailable(
                f"this solve needs about {_need_mb:.0f} MB for a {n:,}-panel "
                f"influence matrix, over the {_MEMORY_BUDGET_MB:.0f} MB this "
                f"app can safely use. Pick a smaller mesh size limit, or "
                f"export a coarser STL — the cost is quadratic in panel count, "
                f"so dropping to {int(n * 0.7):,} panels roughly halves it.")

        A = self._influence_matrix(centroids, normals, areas, tris)

        # RHS: cancel the onset normal velocity on every panel (flow tangency).
        rhs = -(normals @ vinf)

        #  Solve A·sigma = rhs.
        #
        #  LU FIRST, LEAST-SQUARES ONLY AS A FALLBACK. The system is square and
        #  measured cond is 1e2 to 1e3, so an LU solve is the right tool.
        #  np.linalg.lstsq defaults to the SVD-based driver, which allocates
        #  U, S and V^T on top of the matrix — several times N^2 of workspace.
        #  On a 4220-panel undertray that was about 250 MB of the peak, in an
        #  app that has roughly a gigabyte for everything including Streamlit
        #  and every concurrent viewer.
        #
        #  lstsq is kept for the genuinely degenerate case, where it returns
        #  the minimum-norm answer rather than raising — the conditioning guard
        #  below then catches it and marks the result unconverged. So the
        #  robustness that lstsq was chosen for is preserved; it is simply no
        #  longer paid for on every well-posed solve.
        try:
            sigma = np.linalg.solve(A, rhs.astype(A.dtype, copy=False))
        except np.linalg.LinAlgError:
            try:
                sigma, *_ = np.linalg.lstsq(A, rhs, rcond=None)
            except np.linalg.LinAlgError as e:              # noqa: BLE001
                raise PanelMethodUnavailable(
                    f"panel linear solve failed: {e}")
        sigma = np.asarray(sigma, dtype=float)

        # CONDITIONING GUARD. lstsq never raises on a near-degenerate system; it
        # returns the minimum-norm answer to a problem that is no longer the one
        # posed, and this class then reported converged=True regardless ("a
        # direct solve, not an iteration"). That reasoning is wrong: a direct
        # solve cannot fail to *terminate*, but it can absolutely fail to be
        # meaningful.
        #
        # It is not theoretical. Sweeping rake on a subdivided box, pitch 0 deg
        # and 3 deg give max|sigma| ~2.4 and ~3.0; pitch 1.5 deg gives 233, a
        # hundredfold spurious mode, and C_L came back as -381. Non-monotone in
        # pitch, so it is a near-degenerate panel configuration rather than any
        # physical trend — and the sweep would have printed it as a data point.
        #
        # Flagged, not raised: the caller may still want the field for
        # inspection. But converged goes False so nothing downstream averages a
        # spurious point into an aero map.
        cond = float(np.linalg.cond(A)) if len(A) < 2500 else float("nan")
        sig_max = float(np.abs(sigma).max()) if sigma.size else 0.0
        ill = (np.isfinite(cond) and cond > 5.0e3) or sig_max > 50.0
        cond_note = ""
        if ill:
            cond_note = (f"  [WARNING: ill-conditioned panel system "
                         f"(cond={cond:.1e}, max|sigma|={sig_max:.1f}) — the "
                         f"least-squares answer is not trustworthy at this "
                         f"attitude. Re-mesh or nudge the attitude; do NOT put "
                         f"this point in an aero map.]")

        # Surface velocity = onset + induced; pressure from Bernoulli.
        # The ground image MUST be included here as well as in the influence
        # matrix — see _induced_velocity.
        v_ind = self._induced_velocity(centroids, normals, areas, sigma,
                                       ground_effect=self.params.ground_effect,
                                       road_plane_z_m=self.params.road_plane_z_m,
                                       tris=tris)
        v_surf = v_ind + vinf[None, :]
        # remove the normal component numerically (tangency is enforced only approx.)
        vn = np.einsum("ij,ij->i", v_surf, normals)
        v_tan = v_surf - vn[:, None] * normals
        speed2 = np.einsum("ij,ij->i", v_tan, v_tan)
        cp = 1.0 - speed2                                   # |Vinf| = 1

        # Force coefficient = -(1/Aref) ∮ Cp n dA, then split into lift/drag/side.
        # Aref/Lref come from the spec so the result is comparable to tunnel/Fluent.
        aref = max(spec.reference_area_m2, 1e-9)
        f = -(cp * areas)[:, None] * normals                # per-panel force vector
        F = f.sum(axis=0) / aref                            # [Fx, Fy, Fz] coeff

        # Drag is along the onset flow; lift is vertical (z); side is lateral (y).
        c_drag_pressure = float(F @ vinf)
        c_lift = float(F[2])                                # +z up => negative = downforce
        c_side = float(F[1])

        # Skin friction: flat-plate turbulent estimate over the wetted area, added to
        # the pressure drag so total C_d is physical (potential flow alone gives ~0).
        c_drag_friction = self._friction_cd(spec, areas, aref, length_ref)
        c_drag = c_drag_pressure + c_drag_friction

        #  CIRCULATION, SUPERPOSED. See PanelParams.lifting.
        lift_note = ""
        if self.params.lifting:
            c_lift_circ, c_drag_ind, lift_note, _circ_ok = \
                self._circulation_terms(spec)
            c_lift += c_lift_circ
            c_drag += c_drag_ind
            if not _circ_ok:
                ill = True

        # Aero balance: fraction of downforce ahead of the body mid-length. Use the
        # panel x-centroids and their vertical load to split front/rear.
        front = self._aero_balance(centroids, cp, areas, normals)

        # RESOLUTION GUARD — see _resolution_warning. Ground effect is the one
        # thing this module is for, and it is also where the point-source
        # approximation fails first.
        res_warn = self._resolution_warning(centroids, areas, ride_gap)

        return CoeffResult(
            attitude=spec.attitude,
            c_lift=c_lift, c_drag=c_drag, c_side=c_side,
            c_pitch=None,
            aero_balance_front=front,
            converged=not ill,              # direct solve; False only if ill-conditioned
            force_monitor_range=0.0,
            provenance=self.provenance(n_panels=n),
            notes=(f"panel solve: {n} panels, Cd(pressure)={c_drag_pressure:+.3f} "
                   f"+ Cd(friction)={c_drag_friction:.3f}" + lift_note
                   + res_warn + cond_note),
        )

    def _circulation_terms(self, spec):
        """Lift and induced drag from a lattice on the extracted camber surface.

        Returns (c_lift, c_drag_induced, note). Never raises: a body with no
        recoverable camber surface — a sphere, a nose cone, a rollhoop — is a
        pure displacement problem and the source solve already answered it, so
        the right result there is zero added lift and a note saying why.
        """
        try:
            from .vortex_lattice import VortexLatticeModel
            res = VortexLatticeModel().solve(spec)
        except Exception as e:                              # noqa: BLE001
            return 0.0, 0.0, (
                f"  [no circulation added: no lifting surface found "
                f"({str(e)[:80]}). Displacement only, which is the right "
                f"answer for a bluff body.]"), True

        note = (f"  [+ circulation: C_L {res.c_lift:+.4f} and induced drag "
                f"{res.c_drag:+.5f} from a lattice on the mean camber surface, "
                f"superposed on the source solve. Thickness and blockage come "
                f"from the panels, lift from the circulation.]")
        if not res.converged:
            #  The lift term now dominates the answer, so a lattice that did
            #  not converge makes the whole result untrustworthy — not just the
            #  part it contributed. Carrying `converged` back is the point.
            note += ("  [WARNING: the lattice did not converge at this "
                     "attitude, so the lift term — which now dominates the "
                     "result — is unreliable. Check the ride height against "
                     "the mean chord.]")
        return float(res.c_lift), float(res.c_drag), note, bool(res.converged)

    # -- CFDSolver-shaped convenience (physics only; no deck) -------------- #
    def run_case(self, spec: CaseSpec, workdir: str) -> CoeffResult:
        return self.solve(spec)

    def read_result(self, spec: CaseSpec, workdir: str) -> CoeffResult:
        return self.solve(spec)

    # ------------------------------------------------------------------ #
    #  Geometry loading + attitude placement
    # ------------------------------------------------------------------ #
    def _load_panels(self, spec: CaseSpec):
        """
        Load the STL, place it at the attitude (roll + ride height move the body;
        yaw/pitch are in the onset flow), decimate to the panel budget, and return
        per-panel centroids, unit normals, areas and a reference length. Raises
        PanelMethodUnavailable for any geometry problem so the caller can fall back.
        """
        import numpy as np

        path = spec.geometry_path
        if not path or not os.path.isfile(path):
            raise PanelMethodUnavailable(
                f"geometry '{path}' not found on disk — the panel method needs a real "
                "STL/surface mesh. Falling back to the analytic estimate.")
        try:
            import trimesh
        except Exception as e:                              # noqa: BLE001
            raise PanelMethodUnavailable(f"trimesh not available: {e}")

        try:
            mesh = trimesh.load(path, force="mesh")
        except Exception as e:                              # noqa: BLE001
            raise PanelMethodUnavailable(f"could not load geometry '{path}': {e}")
        if mesh is None or getattr(mesh, "faces", None) is None or len(mesh.faces) == 0:
            raise PanelMethodUnavailable(f"geometry '{path}' has no triangles to solve")

        #  REFUSE A THIN LIFTING SURFACE BEFORE SOLVING IT.
        #
        #  This is a SOURCE-only formulation: no doublets, no vortex lattice, no
        #  Kutta condition. Source panels model displacement — thickness, ground
        #  effect, the pressure field of attached flow — but they carry no
        #  circulation, and circulation is what generates lift on a wing. So an
        #  isolated wing is not a hard case for this method, it is the wrong
        #  method, and no amount of mesh refinement changes that.
        #
        #  Two symptoms, both measured on a 300 x 1200 mm test wing:
        #
        #    * The system goes ill-conditioned as the upper and lower surfaces
        #      approach each other and their influence coefficients converge.
        #      At 20 mm thickness, cond = 1.4e4 with a spurious source strength
        #      of 112; at 8 mm no better; at 40 mm it converges. Panel budget is
        #      irrelevant — 800, 2000 and 4000 give bit-identical results.
        #    * Even where it converges, the lift is wrong. The test wing returns
        #      POSITIVE lift at -4 deg.
        #
        #  A real user hit exactly this: dropped in a rear wing, got a
        #  non-converged result, and set out to troubleshoot his own setup. The
        #  solver had no way to tell him the geometry was the problem, so it let
        #  him spend his evening on a case that cannot work. Refusing up front,
        #  with the reason, is the only honest behaviour.
        #
        #  The threshold is deliberately generous: 25 mm catches wings and
        #  aerofoil sections while passing floors, undertrays, sidepods, noses
        #  and full cars, which are what this method is for.
        try:
            _ext = mesh.bounding_box.extents          # (dx, dy, dz) in metres
            _thin_mm = float(min(_ext)) * 1000.0
            _long_mm = float(max(_ext)) * 1000.0
            if _thin_mm < 25.0 and _long_mm > 6.0 * _thin_mm:
                raise PanelMethodUnavailable(
                    f"this geometry is {_thin_mm:.0f} mm across its thinnest "
                    f"axis, which makes it a thin lifting surface — a wing or "
                    f"an aerofoil section. This is a source-panel method with "
                    f"no circulation (no doublets, no Kutta condition), so it "
                    f"cannot produce trustworthy lift on one, and the linear "
                    f"system goes ill-conditioned as the surfaces approach. "
                    f"Refining the mesh does not help. Use it on the floor, "
                    f"undertray, sidepods or the full car, where displacement "
                    f"and the ground image carry the physics; size wings with "
                    f"a method that models circulation.")
        except PanelMethodUnavailable:
            raise
        except Exception:                                   # noqa: BLE001
            pass          # a bounding box we cannot read is not a reason to stop

        # Decimate to the panel budget to keep the dense solve interactive.
        mp = self.params
        if mp.max_panels and len(mesh.faces) > mp.max_panels:
            #  NO DECIMATION. CAP THE INPUT INSTEAD.
            #
            #  Quadric decimation was tried and it corrupts this geometry
            #  class. On a closed thin-walled undertray it returned a mesh that
            #  preserved area and volume to 100% while breaking watertightness,
            #  leaving degenerate faces, and skewing the normal distribution
            #  (662 up / 772 down on a box that must be symmetric). The solved
            #  answer went from a converging -0.0107 / -0.0093 / -0.0085 series
            #  on native meshes to +1.12 / -0.011 / +0.69 across budgets:
            #  different sign, different trend, pure noise.
            #
            #  Deleting faces instead is worse — it perforates a closed surface
            #  and the flow passes through the holes.
            #
            #  There is no safe way to reduce a closed mesh here, so the mesh
            #  is solved as supplied and the SIZE is capped instead. Memory is
            #  now the binding constraint rather than a correctness one, and
            #  chunked assembly brought a 4220-face undertray from 1740 MB to
            #  426 MB, so a few thousand faces is comfortable.
            if len(mesh.faces) > mp.max_panels:
                raise PanelMethodUnavailable(
                    f"this mesh has {len(mesh.faces):,} triangles, over the "
                    f"{mp.max_panels:,} this solver will accept. Reducing it "
                    f"here is not safe: decimation breaks thin-walled closed "
                    f"surfaces and silently changes the answer, so the mesh is "
                    f"solved exactly as supplied. Export a coarser STL from "
                    f"CAD instead — in SolidWorks, Save As > STL > Options and "
                    f"raise the deviation and angle tolerances. A few thousand "
                    f"triangles resolves a potential solve well.")

        # Place the body at attitude: roll, then PITCH, then ride-height translate.
        #
        # PITCH MUST BE GEOMETRIC WHEN THERE IS A GROUND PLANE. It used to be
        # folded into the onset flow vector along with yaw, on the free-air
        # identity that tilting the body and tilting the flow are the same
        # thing. That identity dies the moment a road exists: rotating the
        # freestream leaves the car's relationship to the road untouched, so
        # rake changed nothing about the underbody-to-road angle — and that
        # angle IS the ground-effect mechanism. The module advertises "the
        # downforce trend with rake and ride height"; ride height worked (it is
        # the translation below), rake did not, because with ground_effect on
        # the image system never saw it.
        #
        # Rake is the primary axis of every aero map. A sweep over it was
        # returning only the small free-air incidence effect.
        #
        # Yaw stays in the onset flow: the road is symmetric about z, so
        # rotating the body about z and rotating the flow about z ARE
        # equivalent even with the ground present.
        a = spec.attitude
        T = trimesh.transformations
        roll = T.rotation_matrix(math.radians(a.roll_deg), [1.0, 0.0, 0.0])
        mesh.apply_transform(roll)
        # +pitch = nose up. Rotate about +y (to the right); nose is -x here, so
        # a positive rotation about +y lifts the nose. Pivot about the body
        # centroid in x so pitch is rake, not a disguised heave.
        if abs(a.pitch_deg) > 1e-9:
            pivot = [float(mesh.centroid[0]), 0.0, 0.0]
            pitch = T.rotation_matrix(math.radians(a.pitch_deg), [0.0, 1.0, 0.0],
                                      pivot)
            mesh.apply_transform(pitch)
        #  PLACE THE BODY AT THE RIDE HEIGHT IT WAS ASKED FOR.
        #
        #  This used to be `dz = (ride_height_mm - 30)/1000`, a fixed offset
        #  that silently assumed every STL was authored with exactly 30 mm of
        #  clearance already built in. Nothing enforced that, so the clearance
        #  the solver actually used was (ride_height - 30 + whatever the STL's
        #  own lowest point happened to be). On a part exported resting on
        #  z = 0 — the natural way to export a floor — asking for 70 mm gave
        #  40 mm. The UI reported the request, the resolution guard measured
        #  the reality, and the two disagreed on screen: "70 mm" in the table
        #  next to "the 40 mm ride height" in the note.
        #
        #  Worse than the cosmetic mismatch, every ride-height sweep was
        #  offset by an unknown constant, so absolute C_L was attributed to the
        #  wrong clearance and two differently-authored STLs were not
        #  comparable at the same nominal height.
        #
        #  Measure the body and put it where it belongs. Taken AFTER roll and
        #  pitch, because both change which point is lowest — with rake applied
        #  the clearance must still mean the real gap to the road.
        gap_target = self.params.road_plane_z_m + a.ride_height_mm / 1000.0
        mesh.apply_translation([0.0, 0.0, gap_target - float(mesh.bounds[0][2])])
        #  The achieved gap is now the requested one by construction; carried
        #  out so the resolution guard reports the same number the UI shows
        #  rather than measuring its own.
        ride_gap = a.ride_height_mm / 1000.0

        centroids = np.asarray(mesh.triangles_center, dtype=float)
        normals = np.asarray(mesh.face_normals, dtype=float)
        areas = np.asarray(mesh.area_faces, dtype=float)
        #  The triangles themselves, in the SAME placed frame as the centroids,
        #  are needed for the near-field quadrature. Taken here rather than
        #  re-loading the STL so they cannot drift out of step with the
        #  attitude transform applied above.
        tris = np.asarray(mesh.triangles, dtype=float)

        # Drop degenerate panels (zero area / nan normal).
        good = (areas > 1e-12) & np.isfinite(normals).all(axis=1)
        centroids, normals, areas = centroids[good], normals[good], areas[good]
        tris = tris[good]
        if len(areas) < mp.min_panels:
            raise PanelMethodUnavailable(
                f"only {len(areas)} usable panels (< {mp.min_panels}); surface too "
                "coarse for a trustworthy panel solve")

        length_ref = float(spec.reference_length_m) if spec.reference_length_m else \
            float(centroids[:, 0].ptp() or 1.0)
        return centroids, normals, areas, length_ref, tris, ride_gap

    # ------------------------------------------------------------------ #
    #  Source-panel influence (point-source approx + ground image)
    # ------------------------------------------------------------------ #
    #: Sub-element quadrature stencil, built once. 2 levels = 16 congruent
    #: sub-triangles; 1 level was not enough to flatten the drift and 3 costs
    #: 4x for no measurable gain.
    _SUB_W = None            # filled in below the class body

    #: A pair counts as "near" when the centroid separation is under
    #: _NEAR_R * sqrt(area_j). 2.5 catches about 27 neighbours per panel and
    #: took the cambered step-to-step drift from 37/23/21/15% to 8/5/6/7%.
    #: 4.0 catches ~70 and gives the same answer, so 2.5 is the cheaper choice.
    _NEAR_R = 2.5

    # ------------------------------------------------------------------ #
    #  Near-field quadrature
    #
    #  WHY THIS EXISTS. Every panel is modelled as a POINT source at its
    #  centroid carrying the panel's area. For two panels separated by many
    #  panel widths that is accurate to second order. For immediate neighbours
    #  the separation IS the panel width, so the relative error stays O(1) no
    #  matter how fine the mesh — refining shrinks the panels and their spacing
    #  together and the near-field error never goes away.
    #
    #  On a FLAT body those errors sit symmetrically around each panel and
    #  cancel in the n-weighted pressure integral. On a CURVED one the surface
    #  bends the same way everywhere locally, the errors are one-signed, and
    #  they accumulate. Because a closed body's pressure integral is a near
    #  total cancellation (d'Alembert; the ground image is the only thing
    #  breaking it), that residue lands directly in C_L — and grows with panel
    #  count instead of shrinking.
    #
    #  Measured on a 1.2 x 0.7 m undertray at 150 mm, varying only camber:
    #
    #      faces    flat      camber 30 mm (before)   camber 30 mm (after)
    #       1256   -0.00576        -0.01470                -0.02009
    #       2316   -0.00527        -0.01908                -0.02114
    #       3844   -0.00521        -0.02430                -0.02251
    #       6200   -0.00522        -0.02854                -0.02424
    #      step Δ      0%          37/23/21/15%              8/5/6/7%
    #
    #  Flat always converged, which is why this went unnoticed: the box in the
    #  convergence table is flat. Camber is the entire reason an undertray
    #  makes downforce, so the shapes that matter were the ones that diverged.
    #
    #  WHAT REMAINS. The fix leaves a slow drift on cambered bodies — about 7%
    #  per refinement step, and it does not flatten with more quadrature levels
    #  or a wider near radius, so it is the flat-panel representation of a
    #  curved surface rather than the near field. The absolute level on a
    #  cambered part is therefore not a converged number. The RATIO between two
    #  geometries at matched mesh density is:
    #
    #      faces   C_L 20mm camber   C_L 30mm camber   ratio
    #       1256      -0.01320          -0.01791       1.356
    #       2316      -0.01401          -0.01910       1.364
    #       3844      -0.01471          -0.02047       1.392
    #       6200      -0.01580          -0.02212       1.401
    #
    #  20% drift in the levels, 35% in the difference, 3% in the ratio over a
    #  5x refinement. That is the screening use this solver actually supports.
    #
    #  The fix is to integrate the source over the actual triangle instead of
    #  collapsing it to its centroid, for near pairs only. Far pairs keep the
    #  cheap point form, so the cost is O(N * k) with k about 27 neighbours,
    #  not O(N^2).
    # ------------------------------------------------------------------ #

    @staticmethod
    def _sub_centroid_weights(levels: int = 2):
        """Barycentric centroids of the sub-triangles after `levels` rounds of
        midpoint subdivision. All 4**levels pieces are congruent, so each
        carries area/4**levels and only the positions are needed."""
        import numpy as np
        tris = [np.eye(3)]
        for _ in range(levels):
            out = []
            for t in tris:
                a, b, c = t
                ab, bc, ca = (a + b) / 2.0, (b + c) / 2.0, (c + a) / 2.0
                out += [np.array([a, ab, ca]), np.array([ab, b, bc]),
                        np.array([ca, bc, c]), np.array([ab, bc, ca])]
            tris = out
        return np.array([t.mean(axis=0) for t in tris])

    @staticmethod
    def _near_pairs(centroids, areas, radius_factor, block: int = 256):
        """(i, j) pairs, i != j, with |c_i - c_j| < radius_factor*sqrt(area_j).

        Was O(N^2) — blocked, but still built an (N/block, N) distance matrix
        per block. At 800 panels that was 330 ms, called twice per solve.

        Now uses scipy.spatial.cKDTree.query_ball_point: the KD-tree takes
        O(N log N) to build and each radius query is sublinear on average, so
        the total drops to ~30 ms at 800 panels and scales much more gently.
        The per-source radius (radius_factor * sqrt(area_j)) requires
        query_ball_point rather than query_pairs, because the threshold varies
        per source rather than being a single global value.
        `workers=-1` uses all available cores for the lookup.
        """
        import numpy as np
        try:
            from scipy.spatial import cKDTree
            thresh = radius_factor * np.sqrt(areas)
            tree = cKDTree(centroids)
            hits = tree.query_ball_point(centroids, thresh, workers=-1)
            counts = np.fromiter((len(h) for h in hits), dtype=np.intp,
                                 count=len(hits))
            src = np.repeat(np.arange(len(hits), dtype=np.intp), counts)
            tgt = np.concatenate(hits).astype(np.intp)
            mask = src != tgt
            return src[mask], tgt[mask]
        except ImportError:
            #  scipy not available: fall back to the blocked O(N^2) version.
            n = len(centroids)
            thresh = radius_factor * np.sqrt(areas)
            ii, jj = [], []
            for lo in range(0, n, block):
                hi = min(lo + block, n)
                d = np.linalg.norm(
                    centroids[lo:hi, None, :] - centroids[None, :, :], axis=2)
                m = d < thresh[None, :]
                m[np.arange(hi - lo), np.arange(lo, hi)] = False
                a, b = np.nonzero(m)
                ii.append(a + lo)
                jj.append(b)
            return np.concatenate(ii), np.concatenate(jj)

    @classmethod
    def _quad_normal_vel(cls, field_pts, field_normals, tris_j, areas_j,
                         chunk: int = 20000):
        """n_i . u from a constant source spread over triangle j."""
        import numpy as np
        w = cls._SUB_W
        k = w.shape[0]
        out = np.empty(len(field_pts), dtype=float)
        for lo in range(0, len(field_pts), chunk):
            hi = min(lo + chunk, len(field_pts))
            sub = np.einsum("kb,pbd->pkd", w, tris_j[lo:hi])
            diff = field_pts[lo:hi, None, :] - sub
            r2 = np.einsum("pkd,pkd->pk", diff, diff) + 1e-14
            inv = 1.0 / (4.0 * math.pi * np.power(r2, 1.5))
            ndot = np.einsum("pd,pkd->pk", field_normals[lo:hi], diff)
            out[lo:hi] = (ndot * inv).sum(axis=1) * areas_j[lo:hi] / k
        return out

    @classmethod
    def _quad_velocity(cls, field_pts, tris_j, sa_j, chunk: int = 20000):
        """Induced velocity VECTOR from a constant source over triangle j.

        This is the half that actually moves C_L: Cp is built from the surface
        velocity, so a per-panel velocity error survives straight into the
        force integral.
        """
        import numpy as np
        w = cls._SUB_W
        k = w.shape[0]
        out = np.empty((len(field_pts), 3), dtype=float)
        for lo in range(0, len(field_pts), chunk):
            hi = min(lo + chunk, len(field_pts))
            sub = np.einsum("kb,pbd->pkd", w, tris_j[lo:hi])
            diff = field_pts[lo:hi, None, :] - sub
            r2 = np.einsum("pkd,pkd->pk", diff, diff) + 1e-14
            inv = 1.0 / (4.0 * math.pi * np.power(r2, 1.5))
            out[lo:hi] = np.einsum("pk,pkd->pd", inv, diff) * (
                sa_j[lo:hi, None] / k)
        return out

    @staticmethod
    def _point_velocity(field_pts, src_pts, sa_j):
        """The single-point form, so the near-field fix can be applied as
        (quadrature - point) on top of the already-assembled far field."""
        import numpy as np
        diff = field_pts - src_pts
        r2 = np.einsum("pd,pd->p", diff, diff) + 1e-9
        inv = sa_j / (4.0 * math.pi * np.power(r2, 1.5))
        return diff * inv[:, None]

    def _influence_matrix(self, centroids, normals, areas, tris=None):
        """
        A[i,j] = normal velocity at panel i from a unit constant source on panel j
        (area-weighted point source at its centroid), plus the contribution of j's
        IMAGE reflected through the road plane when ground_effect is on. Self-term is
        the standard +1/2 source jump on the panel's own normal.
        """
        import numpy as np

        c = centroids
        n = len(c)

        #  ASSEMBLE IN ROW BLOCKS, NOT ALL AT ONCE.
        #
        #  The obvious form builds diff = c[:,None,:] - c[None,:,:], an
        #  (N, N, 3) array, and a second one for the image system — plus einsum
        #  temporaries of the same order. At 3072 panels that is over 600 MB of
        #  intermediates for a matrix that is only 75 MB, and it is what pushed
        #  peak RSS to 973 MB and got the hosted app OOM-killed.
        #
        #  Chunking by rows changes no arithmetic whatsoever: each block
        #  computes exactly the same entries, just BLOCK rows at a time, so the
        #  transient is (BLOCK, N, 3) instead of (N, N, 3). Results are
        #  bit-identical; only the peak memory moves.
        #
        #  512 rows keeps the transient near 12 MB per (N, 3) slab at N = 3000
        #  and costs nothing measurable in time — the work is the same, it is
        #  simply not all resident at once.
        BLOCK = 512
        #  float32 for the influence matrix.
        #
        #  It is the single largest allocation in the solve — 142 MB at 4220
        #  panels against 71 MB here — and this app has about a gigabyte for
        #  everything including Streamlit itself and every concurrent viewer.
        #
        #  Safe because the system is well conditioned: measured cond is 1e2 to
        #  1e3, so float32's ~7 significant figures lose at most three, leaving
        #  four. Checked directly rather than assumed: rounding the assembled
        #  matrix to float32 moved C_L by 0.0001% on a converged case
        #  (-0.00850434 vs -0.00850433). The conditioning guard already refuses
        #  anything where that margin would not hold.
        #
        #  The right-hand side and the solve are left in float64; only the
        #  matrix, which dominates memory, is narrowed.
        A = np.empty((n, n), dtype=np.float32)

        zr = self.params.road_plane_z_m
        if self.params.ground_effect:
            c_img = c.copy()
            c_img[:, 2] = 2.0 * zr - c_img[:, 2]       # reflect through z = zr
        else:
            c_img = None

        for lo in range(0, n, BLOCK):
            hi = min(lo + BLOCK, n)
            # r_ij = c_i - c_j  (vector from source j to field point i)
            diff = c[lo:hi, None, :] - c[None, :, :]       # (block, N, 3)
            A[lo:hi] = self._normal_vel_kernel(
                diff, normals[lo:hi], areas).astype(np.float32, copy=False)
            if c_img is not None:
                diff_img = c[lo:hi, None, :] - c_img[None, :, :]
                A[lo:hi] += self._normal_vel_kernel(
                    diff_img, normals[lo:hi],
                    areas).astype(np.float32, copy=False)
                del diff_img
            del diff

        #  Self-influence: a source panel induces +1/2 (area-scaled) on its own
        #  normal. Applied AFTER the image contribution, exactly as before — the
        #  image of a panel is a different panel and must not be overwritten.
        _diag = np.arange(n)
        if c_img is not None:
            #  preserve the image self-contribution, which the old code added
            #  on top of the 0.5 because fill_diagonal ran before the image term
            _img_self = A[_diag, _diag] - self._normal_vel_kernel(
                (c[:, None, :] - c[None, :, :])[_diag, _diag][:, None, :],
                normals, areas)[:, 0] if False else None
        A[_diag, _diag] = 0.5
        if c_img is not None:
            #  re-add the panel's own image, which is a real source below the
            #  road and not part of the self-jump
            d_self = c - c_img
            r2 = np.einsum("ij,ij->i", d_self, d_self) + 1e-12
            inv = 1.0 / (4.0 * math.pi * np.power(r2, 1.5))
            A[_diag, _diag] += np.einsum("ij,ij->i", normals, d_self) * inv * areas

        #  NEAR-FIELD CORRECTION. Overwrite the point-source entry for close
        #  pairs with the sub-element quadrature of the real triangle. The
        #  diagonal is left alone: the self term is the analytic +1/2 jump,
        #  which is exact and not a quadrature at all.
        if tris is not None and len(centroids) > 1:
            _ii, _jj = self._near_pairs(centroids, areas, self._NEAR_R)
            if len(_ii):
                A[_ii, _jj] = self._quad_normal_vel(
                    centroids[_ii], normals[_ii], tris[_jj],
                    areas[_jj]).astype(A.dtype, copy=False)
        return A

    @staticmethod
    def _normal_vel_kernel(diff, normals, areas):
        """n_i · u(r_ij) for a point source of strength = area_j, u = r / (4π |r|^3)."""
        import numpy as np
        r2 = np.einsum("ijk,ijk->ij", diff, diff) + 1e-12  # softened to avoid blowup
        inv = 1.0 / (4.0 * math.pi * np.power(r2, 1.5))
        # velocity vector field = diff * inv * area_j ; dot with field-point normal n_i
        ndotd = np.einsum("ik,ijk->ij", normals, diff)
        return ndotd * inv * areas[None, :]

    @staticmethod
    def _induced_velocity(centroids, normals, areas, sigma,
                          ground_effect: bool = False,
                          road_plane_z_m: float = 0.0,
                          tris=None):
        """Full induced velocity vector at each centroid (for the surface speed).

        GROUND IMAGE: this must mirror _influence_matrix exactly. It previously
        did not — the image was included when solving for sigma (the tangency
        condition saw the road) but omitted here, so the velocity field that
        produces Cp, and therefore every force this module reports, was computed
        as though the road were not there. The boundary condition knew about the
        ground and the pressures did not.

        That is not a small discrepancy in a module whose stated purpose is that
        "ground effect emerges from the physics". Ground effect IS the image
        system accelerating the flow under the floor; drop it from the velocity
        and the downforce it is supposed to predict largely disappears.

        The decisive check is the far-field limit: move the road away and a
        ground-effect solve must converge on the free-air solve. It did not —
        with the road 0.5 m from a 0.6 m body the old code still reported a
        spurious C_L of ~9e-4 against a free-air 0.0, and at FSAE ride heights
        it produced slight LIFT where the corrected solve produces downforce.
        Pinned by test_panel_method_recovers_free_air_when_the_road_is_far.
        """
        import numpy as np
        c = centroids
        n = len(c)

        #  Chunked for the same reason as _influence_matrix: the direct form
        #  builds two (N, N, 3) arrays plus einsum temporaries, which at a few
        #  thousand panels is hundreds of megabytes for a result that is only
        #  (N, 3). Identical arithmetic, one row block at a time.
        BLOCK = 512
        v = np.empty((n, 3), dtype=float)
        sa = sigma * areas

        if ground_effect:
            c_img = c.copy()
            c_img[:, 2] = 2.0 * road_plane_z_m - c_img[:, 2]
        else:
            c_img = None

        for lo in range(0, n, BLOCK):
            hi = min(lo + BLOCK, n)
            diff = c[lo:hi, None, :] - c[None, :, :]
            r2 = np.einsum("ijk,ijk->ij", diff, diff) + 1e-9
            inv = sa[None, :] / (4.0 * math.pi * np.power(r2, 1.5))
            v[lo:hi] = np.einsum("ij,ijk->ik", inv, diff)
            if c_img is not None:
                # Image of every source reflected through the road plane, same
                # strength (a source's image in a streamline plane is a source).
                d_img = c[lo:hi, None, :] - c_img[None, :, :]
                r2i = np.einsum("ijk,ijk->ij", d_img, d_img) + 1e-9
                invi = sa[None, :] / (4.0 * math.pi * np.power(r2i, 1.5))
                v[lo:hi] += np.einsum("ij,ijk->ik", invi, d_img)
                del d_img, r2i, invi
            del diff, r2, inv

        #  NEAR-FIELD CORRECTION, applied as (quadrature - point) on top of the
        #  far field already summed above. This half is the one that moves the
        #  answer: Cp comes from this velocity, and the pressure integral over
        #  a closed body cancels almost exactly, so a per-panel velocity error
        #  that does not shrink with refinement is what C_L ends up reporting.
        #
        #  Direct sources only. A panel's IMAGE sits at least twice the ride
        #  height away — hundreds of millimetres against panel sizes of tens —
        #  so it is far field by construction and the point form is accurate
        #  there. The existing resolution guard already refuses the regime
        #  where that stops being true.
        if tris is not None and n > 1:
            cls = PanelMethodModel
            _ii, _jj = cls._near_pairs(c, areas, cls._NEAR_R)
            if len(_ii):
                _corr = (cls._quad_velocity(c[_ii], tris[_jj], sa[_jj])
                         - cls._point_velocity(c[_ii], c[_jj], sa[_jj]))
                np.add.at(v, _ii, _corr)
        return v

    # ------------------------------------------------------------------ #
    #  Closures: skin friction + aero balance
    # ------------------------------------------------------------------ #
    def _friction_cd(self, spec, areas, aref, length_ref) -> float:
        """
        Flat-plate turbulent skin-friction drag referenced to Aref. Cf via the
        Schlichting 1/7-power correlation Cf = 0.074 Re^-0.2, scaled by wetted/ref
        area, lightly discounted for a laminar run length. Speed scales out of the
        coefficient but Re (hence Cf) depends on it, so we use the case speed.
        """
        import numpy as np
        v = max(float(spec.attitude.speed_ms), 1e-3)
        re = v * length_ref / max(self.params.kin_viscosity, 1e-12)
        cf = 0.074 / (re ** 0.2) if re > 1.0 else 0.01
        cf *= (1.0 - 0.85 * self.params.laminar_fraction)   # small transition discount
        wetted = float(np.sum(areas))
        return cf * wetted / max(aref, 1e-9)

    def _resolution_warning(self, centroids, areas, ride_gap=None) -> str:
        """Flag the mesh being too coarse for the ride height it is solving at.

        Every panel is collapsed to a POINT SOURCE at its centroid. That is a
        good approximation only when the distance to the field point is large
        compared with the panel. Under a car in ground effect the field point of
        interest is the road image, a distance of ~2x ride height away — so once
        the panel size approaches the ride height, the single dominant term in
        the whole ground-effect calculation is being evaluated where its own
        approximation is invalid.

        This is not hypothetical and it is not a small error. On a 12-panel box
        (340 mm panels) the ground contribution at 30 mm ride height came out
        SMALLER than at 100 mm — ride-height sensitivity inverted, which is the
        one trend teams actually take from this module. Subdividing to 170 mm
        panels restored monotone behaviour. The failure is silent: the solve
        converges, the numbers look plausible, and the trend is backwards.

        `max_panels` makes this worse, because decimating a fine STL to fit the
        budget is exactly how a mesh becomes too coarse for its ride height.

        Advisory only — it never blocks a solve, because the right call depends
        on what the user is asking of the answer.
        """
        if not self.params.ground_effect:
            return ""
        import numpy as np
        #  Use the gap the body was actually placed at, not the minimum
        #  CENTROID height — a centroid sits half a panel above the surface, so
        #  measuring it here reported a clearance that disagreed with the one
        #  the user typed by a mesh-dependent amount.
        gap = (float(ride_gap) if ride_gap is not None
               else float(np.min(centroids[:, 2])) - self.params.road_plane_z_m)
        if gap <= 0:
            return "  [WARNING: geometry intersects the road plane]"
        panel = float(np.sqrt(np.mean(areas)))
        if panel > gap:
            return (f"  [WARNING: mean panel size {panel * 1000:.0f} mm exceeds the "
                    f"{gap * 1000:.0f} mm ride height — panels are point sources at "
                    f"their centroids, so the ground image is being evaluated inside "
                    f"the range where that approximation fails. Ride-height trends "
                    f"can INVERT. Refine the mesh or raise max_panels.]")
        if panel > 0.4 * gap:
            return (f"  [note: mean panel size {panel * 1000:.0f} mm is a large "
                    f"fraction of the {gap * 1000:.0f} mm ride height; treat the "
                    f"ground-effect magnitude as indicative]")
        return ""

    @staticmethod
    def _aero_balance(centroids, cp, areas, normals) -> float | None:
        """
        Fraction of vertical aero load carried ahead of the body mid-length. Uses the
        per-panel vertical pressure load (Cp · area · n_z); returns None if there is
        effectively no vertical load to split.
        """
        import numpy as np
        x = centroids[:, 0]
        x_mid = 0.5 * (x.min() + x.max())
        load = -(cp * areas) * normals[:, 2]               # +ve = downforce contribution
        total = float(np.sum(load))
        if abs(total) < 1e-9:
            return None
        front = float(np.sum(load[x > x_mid]))             # +x is nose-forward
        rear = total - front

        #  A BALANCE IS ONLY A BALANCE WHEN BOTH ENDS PUSH THE SAME WAY.
        #
        #  front/total was returned clamped to [0, 1]. When one end makes
        #  downforce and the other makes lift, `total` is the small difference
        #  of two opposing loads and the ratio blows up — it came back exactly
        #  1.000 on one geometry and 0.094 on a near-identical one, both of
        #  them the clamp firing rather than a measurement. Clamping turned a
        #  meaningless number into a plausible-looking one, which is worse than
        #  returning nothing.
        #
        #  Report the split only when the ends agree in sign and the net is a
        #  real fraction of what each end carries. Otherwise None, which the UI
        #  already renders as an honest blank.
        if front * rear <= 0.0:
            return None
        if abs(total) < 0.2 * (abs(front) + abs(rear)):
            return None
        return float(min(max(front / total, 0.0), 1.0))


def _freestream_unit(att: Attitude):
    """Unit onset-flow vector with YAW ONLY folded in.

    Pitch is deliberately NOT here any more — it is applied to the geometry in
    _load_panels. Folding pitch into the flow and rotating the body are the same
    thing only in free air; with a road they are not, because tilting the flow
    leaves the underbody-to-road angle unchanged. Keeping pitch in both places
    would double-count it, so this function must stay yaw-only for as long as
    _load_panels rotates the mesh.

    Yaw legitimately stays here: the ground plane is symmetric under rotation
    about z, so yawing the body and yawing the flow remain equivalent.
    """
    import numpy as np
    yaw = math.radians(att.yaw_deg)
    v = np.array([math.cos(yaw), -math.sin(yaw), 0.0], dtype=float)
    n = np.linalg.norm(v)
    return v / (n if n > 1e-12 else 1.0)


#  Built once at import: the quadrature stencil is a fixed 16x3 array of
#  barycentric weights and does not depend on the mesh.
PanelMethodModel._SUB_W = PanelMethodModel._sub_centroid_weights(2)
