# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""
Tests for the higher-fidelity in-house aero: the 3D source-panel (boundary-element)
potential-flow solver (suspension/aero/panel_method.py) and its integration into
FluentVerificationSolver via the `method` switch.

These pin the behaviour that makes it a genuine fidelity step rather than a relabel:

  1. it SOLVES on the real STL — geometry that does not exist is an honest
     PanelMethodUnavailable, never a fabricated number,
  2. GROUND EFFECT emerges from the physics — lowering ride height increases
     downforce magnitude monotonically (the image-panel system, not a tuned term),
  3. total drag is physical — a flat-plate friction term is added to the (near-zero,
     by d'Alembert) potential pressure drag,
  4. it is honestly labelled POTENTIAL fidelity, UNCORRELATED, with notes that say it
     does not resolve separation/stall/wake,
  5. FluentVerificationSolver dispatches method='analytic'|'panel'|'auto' correctly,
     'auto' uses the panel solve when geometry is present and falls back to the
     analytic surrogate (recording why) when it is not,
  6. the Fluent verification deck is still written in every mode.

Run:  python -m pytest tests/test_panel_method.py
"""
import os
import tempfile

import pytest
import trimesh

from suspension.aero import (
    CaseSpec, Attitude, SolverFidelity,
    PanelMethodModel, PanelParams, PanelMethodUnavailable,
    FluentVerificationSolver,
)


# --------------------------------------------------------------------------- #
#  A refined plate STL we can actually solve (a box is only 12 triangles)
# --------------------------------------------------------------------------- #
def _plate_stl(z_lift_m=0.10):
    box = trimesh.creation.box(extents=[1.5, 0.6, 0.08])
    box = box.subdivide().subdivide().subdivide()       # -> hundreds of panels
    box.apply_translation([0.0, 0.0, z_lift_m])         # sit it above the road (z=0)
    d = tempfile.mkdtemp(prefix="panel_stl_")
    path = os.path.join(d, "plate.stl")
    box.export(path)
    return path


def _spec(stl, h=30.0, pitch=0.0, yaw=0.0, v=27.0):
    return CaseSpec(Attitude(pitch_deg=pitch, yaw_deg=yaw, ride_height_mm=h,
                             speed_ms=v),
                    stl, reference_area_m2=0.9, reference_length_m=1.5)


# --------------------------------------------------------------------------- #
#  1. Honest hole when there is no geometry
# --------------------------------------------------------------------------- #
def test_missing_geometry_is_an_honest_hole():
    m = PanelMethodModel()
    with pytest.raises(PanelMethodUnavailable):
        m.solve(_spec("does_not_exist.stl"))


def test_too_coarse_surface_refuses():
    # a raw box is 12 triangles — below the min_panels floor
    box = trimesh.creation.box(extents=[1.0, 0.5, 0.1])
    d = tempfile.mkdtemp(); path = os.path.join(d, "coarse.stl"); box.export(path)
    m = PanelMethodModel(PanelParams(min_panels=24))
    with pytest.raises(PanelMethodUnavailable):
        m.solve(_spec(path))


# --------------------------------------------------------------------------- #
#  2. Ground effect emerges from the image-panel physics
# --------------------------------------------------------------------------- #
def test_ground_effect_increases_downforce_as_ride_height_drops():
    stl = _plate_stl()
    m = PanelMethodModel(PanelParams(max_panels=2000, ground_effect=True))
    cls = [m.solve(_spec(stl, h=h)).c_lift for h in (80.0, 50.0, 30.0, 18.0)]
    # all downforce (negative), and magnitude grows monotonically as we get lower
    assert all(c < 0 for c in cls)
    mags = [abs(c) for c in cls]
    assert mags == sorted(mags), f"downforce should grow as ride height drops: {mags}"
    assert mags[-1] > mags[0] * 1.3      # a meaningful, not marginal, increase


def test_ground_effect_off_is_weaker_than_on():
    stl = _plate_stl()
    on = PanelMethodModel(PanelParams(max_panels=2000, ground_effect=True))
    off = PanelMethodModel(PanelParams(max_panels=2000, ground_effect=False))
    cl_on = on.solve(_spec(stl, h=18.0)).c_lift
    cl_off = off.solve(_spec(stl, h=18.0)).c_lift
    # the road image adds downforce; with it off there is less
    assert abs(cl_on) > abs(cl_off)


# --------------------------------------------------------------------------- #
#  3. Drag is physical: friction is added to the potential pressure drag
# --------------------------------------------------------------------------- #
def test_total_drag_includes_friction_and_is_positive():
    stl = _plate_stl()
    m = PanelMethodModel(PanelParams(max_panels=2000))
    r = m.solve(_spec(stl))
    assert r.c_drag is not None and r.c_drag > 0.0
    assert "Cd(friction)" in r.notes


# --------------------------------------------------------------------------- #
#  4. Honest labelling
# --------------------------------------------------------------------------- #
def test_provenance_is_potential_and_uncorrelated_and_candid():
    stl = _plate_stl()
    r = PanelMethodModel().solve(_spec(stl))
    prov = r.provenance
    assert prov.fidelity == SolverFidelity.POTENTIAL
    assert prov.is_correlated is False
    assert prov.cell_count is not None and prov.cell_count > 0   # panels recorded
    low = prov.notes.lower()
    assert "potential" in low
    assert "separation" in low or "wake" in low   # candid about what it misses


# --------------------------------------------------------------------------- #
#  5. FluentVerificationSolver method dispatch
# --------------------------------------------------------------------------- #
def test_analytic_method_is_geometry_insensitive():
    b = FluentVerificationSolver(method="analytic")
    wd = tempfile.mkdtemp()
    # even with a real STL, the analytic surrogate ignores it
    stl = _plate_stl()
    r = b.run_case(_spec(stl), wd)
    assert "analytic surrogate" in r.notes.lower()
    assert r.provenance.backend == "fluent"


def test_panel_method_uses_geometry_and_records_panels():
    b = FluentVerificationSolver(method="panel")
    stl = _plate_stl()
    r = b.run_case(_spec(stl), tempfile.mkdtemp())
    assert r.provenance.backend == "panel-method"
    assert r.provenance.cell_count and r.provenance.cell_count > 0


def test_panel_method_raises_without_geometry():
    b = FluentVerificationSolver(method="panel")
    with pytest.raises(PanelMethodUnavailable):
        b.run_case(_spec("car.stl"), tempfile.mkdtemp())


def test_auto_uses_panel_when_geometry_present():
    b = FluentVerificationSolver(method="auto")
    stl = _plate_stl()
    r = b.run_case(_spec(stl), tempfile.mkdtemp())
    assert r.provenance.backend == "panel-method"


def test_auto_falls_back_to_analytic_without_geometry_and_says_why():
    b = FluentVerificationSolver(method="auto")
    r = b.run_case(_spec("car.stl"), tempfile.mkdtemp())
    assert r.provenance.backend == "fluent"          # analytic provenance
    assert "panel solve unavailable" in r.notes.lower()
    assert r.c_lift is not None and r.c_lift < 0     # still a usable number


def test_bad_method_rejected():
    with pytest.raises(ValueError):
        FluentVerificationSolver(method="rans")


# --------------------------------------------------------------------------- #
#  6. The Fluent deck is still written in every mode
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("method,geom", [
    ("analytic", "car.stl"),
    ("auto", "car.stl"),
])
def test_fluent_deck_written_in_each_mode(method, geom):
    b = FluentVerificationSolver(method=method)
    wd = tempfile.mkdtemp()
    spec = _spec(geom)
    b.run_case(spec, wd)
    jou = os.path.join(wd, spec.case_name() + ".jou")
    assert os.path.isfile(jou)
    assert "ANSYS Fluent" in open(jou).read()


def test_fluent_deck_written_for_panel_mode_with_geometry():
    b = FluentVerificationSolver(method="panel")
    wd = tempfile.mkdtemp()
    spec = _spec(_plate_stl())
    b.run_case(spec, wd)
    jou = os.path.join(wd, spec.case_name() + ".jou")
    assert os.path.isfile(jou)


def _box_stl(extents, path, sub=3, lift=0.2):
    """A subdivided box at a given size, placed above the road."""
    m = trimesh.creation.box(extents=extents)
    for _ in range(sub):
        m = m.subdivide()
    m.apply_translation([0, 0, lift])
    m.export(path)
    return path


def _solve_box(extents, tmp_path, panels=1500):
    from suspension.aero.panel_method import PanelMethodModel, PanelParams
    from suspension.aero.cfd import CaseSpec, Attitude
    p = _box_stl(extents, str(tmp_path / "g.stl"))
    return PanelMethodModel(PanelParams(max_panels=panels)).solve(
        CaseSpec(attitude=Attitude(pitch_deg=-3, ride_height_mm=200,
                                   speed_ms=20),
                 geometry_path=p, reference_area_m2=0.5,
                 reference_length_m=1.55))


def test_thin_lifting_surface_is_refused_with_the_reason(tmp_path):
    """A source-panel method carries no circulation, so it cannot produce
    trustworthy lift on a wing. That is the wrong method rather than a hard
    case, and refining the mesh does not help.

    A user dropped in a rear wing, got a non-converged result, and set out to
    troubleshoot his own setup — the solver had no way to tell him the geometry
    was the problem. Measured on a 300 x 1200 mm test wing: 20 mm thickness
    gives cond = 1.4e4 with max|sigma| = 112, 8 mm is no better, 40 mm
    converges, and 800 / 2000 / 4000 panels give identical results.
    """
    from suspension.aero.panel_method import PanelMethodUnavailable
    with pytest.raises(PanelMethodUnavailable) as exc:
        _solve_box([0.30, 1.20, 0.020], tmp_path)
    msg = str(exc.value).lower()
    assert "circulation" in msg, "must name the cause, not just decline"
    assert "wing" in msg


@pytest.mark.parametrize("extents,label", [
    ([1.50, 0.90, 0.060], "undertray"),
    ([0.70, 0.30, 0.180], "sidepod"),
    ([2.80, 1.40, 0.90], "full car"),
])
def test_bluff_bodies_are_not_caught_by_the_guard(extents, label, tmp_path):
    """Floors, undertrays, sidepods and full cars are what this method is for.
    A guard that also refused them would remove the feature."""
    r = _solve_box(extents, tmp_path)
    assert r.converged, f"{label} should solve"


def test_oversized_mesh_is_refused_not_decimated(tmp_path):
    """Decimation was tried and removed because it corrupts this geometry.

    On a closed thin-walled undertray, quadric decimation preserved area and
    volume to 100% while breaking watertightness and skewing the normals
    (662 up / 772 down on a box that must be symmetric). The solved answer went
    from a cleanly converging -0.0107 / -0.0093 / -0.0085 series on native
    meshes to +1.12 / -0.011 / +0.69 across budgets: different sign, different
    trend, noise. Deleting faces instead perforates the surface and lets flow
    through the holes.

    So the mesh is solved exactly as supplied, and an oversized one is refused
    with instructions rather than silently altered.
    """
    from suspension.aero.panel_method import (PanelMethodModel, PanelParams,
                                              PanelMethodUnavailable)
    from suspension.aero.cfd import CaseSpec, Attitude
    m = trimesh.creation.box(extents=[0.30, 0.20, 0.10])
    for _ in range(4):
        m = m.subdivide()                      # 3072 faces
    m.apply_translation([0, 0, 0.15])
    p = str(tmp_path / "big.stl")
    m.export(p)
    spec = CaseSpec(attitude=Attitude(pitch_deg=0.0, ride_height_mm=100,
                                      speed_ms=20),
                    geometry_path=p, reference_area_m2=0.06,
                    reference_length_m=1.55)
    with pytest.raises(PanelMethodUnavailable) as exc:
        PanelMethodModel(PanelParams(max_panels=800)).solve(spec)
    assert "coarser" in str(exc.value).lower()


def test_mesh_is_solved_exactly_as_supplied(tmp_path):
    """The panel count solved must equal the triangles in the file. If any
    reduction creeps back in, this breaks."""
    from suspension.aero.panel_method import PanelMethodModel, PanelParams
    from suspension.aero.cfd import CaseSpec, Attitude
    m = trimesh.creation.box(extents=[0.30, 0.20, 0.10])
    for _ in range(3):
        m = m.subdivide()                      # 768 faces
    m.apply_translation([0, 0, 0.15])
    p = str(tmp_path / "m.stl")
    m.export(p)
    r = PanelMethodModel(PanelParams(max_panels=5000)).solve(
        CaseSpec(attitude=Attitude(pitch_deg=0.0, ride_height_mm=100,
                                   speed_ms=20),
                 geometry_path=p, reference_area_m2=0.06,
                 reference_length_m=1.55))
    assert r.provenance.cell_count == len(m.faces)


def test_grid_convergence_on_a_native_mesh(tmp_path):
    """The physics converges when the mesh is left alone. Refining a box from
    192 to 768 to 3072 faces gives -0.01065, -0.00931, -0.00850: monotone, one
    sign, shrinking steps. This is the property decimation destroyed."""
    from suspension.aero.panel_method import PanelMethodModel, PanelParams
    from suspension.aero.cfd import CaseSpec, Attitude
    vals = []
    for sub in (2, 3):
        m = trimesh.creation.box(extents=[0.30, 0.20, 0.10])
        for _ in range(sub):
            m = m.subdivide()
        m.apply_translation([0, 0, 0.15])
        p = str(tmp_path / f"m{sub}.stl")
        m.export(p)
        r = PanelMethodModel(PanelParams(max_panels=5000)).solve(
            CaseSpec(attitude=Attitude(pitch_deg=0.0, ride_height_mm=100,
                                       speed_ms=20),
                     geometry_path=p, reference_area_m2=0.06,
                     reference_length_m=1.55))
        vals.append(r.c_lift)
    assert all(v < 0 for v in vals), f"sign should be stable, got {vals}"
    assert abs(vals[1] - vals[0]) / abs(vals[0]) < 0.20, (
        f"refinement should move the answer by well under 20%, got {vals}")


# --------------------------------------------------------------------------- #
#  Vortex lattice — the lifting-surface solver
# --------------------------------------------------------------------------- #

def _wing_stl(path, span=2.40, chord=0.30, thick=0.030, nc=40, ns=40):
    """A closed rectangular wing with an elliptical thickness distribution."""
    import numpy as np
    xs = np.linspace(0, chord, nc)
    ys = np.linspace(-span / 2, span / 2, ns)

    def t(x):
        return thick * np.sqrt(np.clip(1 - ((x / chord - 0.5) / 0.5) ** 2, 0, 1))

    V, F, U, L = [], [], [], []

    def add(x, y, z):
        V.append((x, y, z))
        return len(V) - 1

    for x in xs:
        U.append([add(x, y, +t(x) / 2) for y in ys])
        L.append([add(x, y, -t(x) / 2) for y in ys])

    def q(a, b, c, d):
        F.append([a, b, c])
        F.append([a, c, d])

    for i in range(nc - 1):
        for j in range(ns - 1):
            q(U[i][j], U[i + 1][j], U[i + 1][j + 1], U[i][j + 1])
            q(L[i][j], L[i][j + 1], L[i + 1][j + 1], L[i + 1][j])
    for i in range(nc - 1):
        q(U[i][0], L[i][0], L[i + 1][0], U[i + 1][0])
        q(U[i][ns - 1], U[i + 1][ns - 1], L[i + 1][ns - 1], L[i][ns - 1])

    m = trimesh.Trimesh(vertices=np.array(V), faces=np.array(F), process=True)
    m.fix_normals()
    m.export(path)
    return m


def _vlm(path, alpha, h_mm, area):
    from suspension.aero.vortex_lattice import VortexLatticeModel
    from suspension.aero.cfd import CaseSpec, Attitude
    return VortexLatticeModel(n_span=24, n_chord=6).solve(
        CaseSpec(attitude=Attitude(pitch_deg=alpha, ride_height_mm=h_mm,
                                   speed_ms=20),
                 geometry_path=path, reference_area_m2=area,
                 reference_length_m=1.55))


def test_vlm_matches_lifting_line_and_is_linear(tmp_path):
    """C_L should be a constant fraction of lifting-line theory across alpha.

    Lifting-line assumes elliptic loading, which a rectangular planform does not
    have, so ~89% is the expected shortfall — what matters is that the ratio is
    the SAME at every incidence, i.e. the solve is linear as potential flow must
    be.
    """
    import math
    p = str(tmp_path / "ar8.stl")
    _wing_stl(p)
    AR, area = 8.0, 2.40 * 0.30
    ratios = []
    for al in (-2.0, -5.0, -8.0):
        r = _vlm(p, al, 3000, area)
        ll = 2 * math.pi * math.radians(abs(al)) * AR / (AR + 2)
        ratios.append(abs(r.c_lift) / ll)
    assert all(0.80 < x < 1.00 for x in ratios), ratios
    assert max(ratios) - min(ratios) < 0.02, f"not linear in alpha: {ratios}"


def test_vlm_induced_drag_is_physical(tmp_path):
    """Span efficiency against the ideal C_Di = C_L^2/(pi*AR).

    The drag comes from the downwash the lattice induces on its own bound
    vortices, not from C_L^2/(pi AR e) with a fitted e — so e falling out near
    unity is a check on the physics rather than a tautology.
    """
    import math
    p = str(tmp_path / "ar8.stl")
    _wing_stl(p)
    AR, area = 8.0, 2.40 * 0.30
    r = _vlm(p, -5.0, 3000, area)
    e = (r.c_lift ** 2) / (math.pi * AR * abs(r.c_drag))
    assert 0.85 < e < 1.10, f"span efficiency {e:.3f} is not physical"


def test_vlm_ground_effect_is_monotone(tmp_path):
    """Downforce must build as the wing approaches the road, and it must come
    from the image system rather than a tuned gain."""
    p = str(tmp_path / "ar8.stl")
    _wing_stl(p)
    area = 2.40 * 0.30
    vals = [_vlm(p, -4.0, h, area).c_lift for h in (2000, 600, 300, 150)]
    assert all(v < 0 for v in vals), f"should be downforce throughout: {vals}"
    assert all(vals[i + 1] < vals[i] for i in range(len(vals) - 1)), (
        f"ground effect not monotone: {vals}")


def test_vlm_zero_lift_at_zero_incidence(tmp_path):
    """A symmetric section at zero alpha must produce no lift. This is the
    check that caught a wrong image plane: reflecting through z = 0 instead of
    through the road a distance h below the lattice gave C_L = -0.997 here."""
    p = str(tmp_path / "ar8.stl")
    _wing_stl(p)
    r = _vlm(p, 0.0, 300, 2.40 * 0.30)
    assert abs(r.c_lift) < 1e-3, f"expected ~0, got {r.c_lift}"


def test_vlm_is_fast_enough_to_sweep(tmp_path):
    """The scalar form took 7.9 s per case — 228,528 np.cross calls on
    3-vectors, with 11.8 of 18.9 s inside numpy's dispatch overhead. Vectorised
    it is ~0.09 s. A ride-height sweep has to stay interactive."""
    import time
    p = str(tmp_path / "ar8.stl")
    _wing_stl(p)
    _vlm(p, -4.0, 300, 0.72)                       # warm any import cost
    t = time.time()
    _vlm(p, -4.0, 250, 0.72)
    assert time.time() - t < 2.0, "vortex lattice solve got slow again"


# ===========================================================================
#  VALIDATION AGAINST A CLOSED-FORM ANSWER
#
#  Everything above pins behaviour — that the solver refuses what it should,
#  labels itself honestly, trends the right way. None of it checks that the
#  numbers are RIGHT, because until now there was no case with a known answer
#  to check them against.
#
#  A sphere in unbounded potential flow has one: the surface speed is
#  1.5*V*sin(theta), so Cp = 1 - 2.25*sin^2(theta) exactly, and by symmetry
#  every force coefficient is zero. That exercises the whole chain — influence
#  matrix, linear solve, induced velocity, Cp, force integration — against
#  arithmetic rather than against itself.
# ===========================================================================
import math

import numpy as np

_SPHERE_R = 0.30


def _sphere_fields(subdivisions, tmp_path):
    """Solve a sphere in free air and return (cp_solved, cp_exact, areas,
    normals) at the panel centroids."""
    cap = {}
    orig = PanelMethodModel._induced_velocity

    def spy(c, n, a, sigma, ground_effect=False, road_plane_z_m=0.0, tris=None):
        v = orig(c, n, a, sigma, ground_effect, road_plane_z_m, tris)
        cap.update(c=np.asarray(c), n=np.asarray(n), a=np.asarray(a), v=v)
        return v

    PanelMethodModel._induced_velocity = staticmethod(spy)
    try:
        m = trimesh.creation.icosphere(subdivisions=subdivisions,
                                       radius=_SPHERE_R)
        p = str(tmp_path / f"sphere{subdivisions}.stl")
        m.export(p)
        aref = math.pi * _SPHERE_R ** 2
        res = PanelMethodModel(
            PanelParams(max_panels=40000, ground_effect=False)
        ).solve(CaseSpec(attitude=Attitude(ride_height_mm=3000.0, speed_ms=20.0),
                         geometry_path=p, reference_area_m2=aref,
                         reference_length_m=2 * _SPHERE_R))
    finally:
        PanelMethodModel._induced_velocity = staticmethod(orig)

    c, n, a, v = cap["c"], cap["n"], cap["a"], cap["v"]
    rel = c - c.mean(axis=0)
    cos_theta = rel[:, 0] / np.linalg.norm(rel, axis=1)      # +x is the onset
    v_surf = v + np.array([1.0, 0.0, 0.0])
    v_tan = v_surf - np.einsum("ij,ij->i", v_surf, n)[:, None] * n
    cp = 1.0 - np.einsum("ij,ij->i", v_tan, v_tan)
    cp_exact = 1.0 - 2.25 * (1.0 - cos_theta ** 2)
    return res, cp, cp_exact, a, n


def test_sphere_pressure_matches_the_closed_form_and_improves_with_mesh(tmp_path):
    """Cp on a sphere against 1 - 2.25 sin^2(theta), the exact potential-flow
    answer. Cp spans 2.25 here (+1 at the stagnation point to -1.25 at the
    equator), so the tolerance below is well under one percent of range."""
    errs = []
    for sub in (2, 3):
        _, cp, cp_exact, a, _ = _sphere_fields(sub, tmp_path)
        errs.append(math.sqrt(float((((cp - cp_exact) ** 2) * a).sum() / a.sum())))
    assert errs[0] < 0.02, f"coarse sphere Cp RMS error {errs[0]:.4f}"
    assert errs[1] < errs[0], f"refining made Cp worse: {errs}"
    assert errs[1] < 0.012, f"fine sphere Cp RMS error {errs[1]:.4f}"


def test_sphere_carries_no_net_force(tmp_path):
    """Symmetry gives zero lift and zero side force, and d'Alembert gives zero
    PRESSURE drag. Total C_d stays positive because friction is added on top —
    the pressure part is what must vanish."""
    res, cp, _, a, n = _sphere_fields(3, tmp_path)
    aref = math.pi * _SPHERE_R ** 2
    force = (-(cp * a)[:, None] * n).sum(axis=0) / aref
    assert abs(force[0]) < 1e-3, f"pressure drag {force[0]:+.5f}, expected 0"
    assert abs(res.c_lift) < 1e-4, f"sphere lift {res.c_lift:+.5f}"
    assert abs(res.c_side) < 1e-4, f"sphere side force {res.c_side:+.5f}"
    assert res.c_drag > 0.0, "total drag must stay positive once friction is in"


def test_coefficients_do_not_depend_on_speed(tmp_path):
    """A coefficient that moves with speed is a coefficient computed wrong.
    The force behind it must scale exactly with V^2."""
    p = _plate_stl()
    slow = PanelMethodModel(PanelParams(max_panels=4000)).solve(
        _spec(p, h=60.0, v=20.0))
    fast = PanelMethodModel(PanelParams(max_panels=4000)).solve(
        _spec(p, h=60.0, v=40.0))
    assert slow.c_lift == pytest.approx(fast.c_lift, rel=1e-6)
    d_slow = slow.downforce_N(1.225, 1.0, 20.0)
    d_fast = fast.downforce_N(1.225, 1.0, 40.0)
    assert d_fast / d_slow == pytest.approx(4.0, rel=1e-6)


def test_cambered_floor_grid_converges(tmp_path):
    """REGRESSION: the near-field fix in _influence_matrix / _induced_velocity.

    Panels were modelled as point sources at their centroids, which is exact in
    the far field and O(1) wrong for immediate neighbours however fine the
    mesh. On a flat body those errors cancel; on a cambered one they are
    one-signed and accumulate, and because a closed body's pressure integral
    almost entirely cancels, the residue landed straight in C_L. Measured on
    this shape before the fix, C_L ran -0.0147 -> -0.0191 -> -0.0243 over three
    refinements: growing with panel count, which is divergence, not
    convergence. Camber is the whole reason an undertray makes downforce, so
    this was the case that mattered.
    """
    def slab(nx, ny, camber=0.030, thick=0.040, length=1.20, width=0.70):
        xs = np.linspace(0.0, length, nx)
        ys = np.linspace(-width / 2.0, width / 2.0, ny)
        X, Y = np.meshgrid(xs, ys, indexing="ij")
        Z = -camber * np.sin(np.pi * (X / length) ** 0.85)
        top = np.stack([X, Y, Z + thick / 2], -1).reshape(-1, 3)
        bot = np.stack([X, Y, Z - thick / 2], -1).reshape(-1, 3)
        n = nx * ny

        def v(i, j, low=False):
            return i * ny + j + (n if low else 0)

        f = []
        for i in range(nx - 1):
            for j in range(ny - 1):
                f += [[v(i, j), v(i + 1, j), v(i + 1, j + 1)],
                      [v(i, j), v(i + 1, j + 1), v(i, j + 1)],
                      [v(i, j, 1), v(i + 1, j + 1, 1), v(i + 1, j, 1)],
                      [v(i, j, 1), v(i, j + 1, 1), v(i + 1, j + 1, 1)]]
        for i in range(nx - 1):
            for j in (0, ny - 1):
                a, b, c_, d = v(i, j), v(i + 1, j), v(i, j, 1), v(i + 1, j, 1)
                f += ([[a, c_, d], [a, d, b]] if j == 0 else
                      [[a, d, c_], [a, b, d]])
        for j in range(ny - 1):
            for i in (0, nx - 1):
                a, b, c_, d = v(i, j), v(i, j + 1), v(i, j, 1), v(i, j + 1, 1)
                f += ([[a, d, c_], [a, b, d]] if i == 0 else
                      [[a, c_, d], [a, d, b]])
        m = trimesh.Trimesh(vertices=np.vstack([top, bot]),
                            faces=np.array(f), process=True)
        m.update_faces(m.nondegenerate_faces())
        m.remove_unreferenced_vertices()
        m.fix_normals()
        m.apply_translation([0.0, 0.0, -m.bounds[0][2]])     # rest on z = 0
        return m

    cls = []
    for nx, ny in ((21, 15), (29, 20), (37, 26)):
        m = slab(nx, ny)
        assert m.is_watertight
        p = str(tmp_path / f"slab{nx}.stl")
        m.export(p)
        r = PanelMethodModel(PanelParams(max_panels=12000)).solve(
            CaseSpec(attitude=Attitude(ride_height_mm=150.0, speed_ms=20.0),
                     geometry_path=p, reference_area_m2=1.20 * 0.70,
                     reference_length_m=1.20))
        assert r.c_lift < 0.0, "a floor bowed toward the road must make downforce"
        cls.append(r.c_lift)

    steps = [abs(b - a) / max(abs(a), abs(b)) for a, b in zip(cls, cls[1:])]
    assert max(steps) < 0.15, (
        f"cambered floor is not grid-converged: C_L {cls}, steps {steps}")
