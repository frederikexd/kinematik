"""suspension/aero/vortex_lattice.py against a lattice written here from scratch: Biot-Savart for one filament with its
singular core, the horseshoe with straight trailing legs, panels by the 1/4-3/4 rule, the ground image with the sense
reversed, Kutta-Joukowski lift and the induced drag from the bound-vortex downwash. The camber extractor is checked
on a lofted, cambered section whose camber line is known at every station, and every refusal is checked word for
word."""
import math
import sys

import numpy as np
import pytest
import trimesh

from suspension.aero import vortex_lattice as vl
from suspension.aero.cfd import Attitude, CaseSpec, SolverFidelity


def _bs(p, a, b):
    r1, r2 = p - a, p - b
    c = np.cross(r1, r2)
    n2, n1, m2 = c @ c, np.linalg.norm(r1), np.linalg.norm(r2)
    if n2 < 1e-12 or n1 < 1e-9 or m2 < 1e-9:
        return np.zeros(3)
    return c / n2 * ((b - a) @ (r1 / n1 - r2 / m2)) / (4 * math.pi)


def _hs(p, a, b, far=1e4):
    o = np.array([far, 0.0, 0.0])
    return _bs(p, a + o, a) + _bs(p, a, b) + _bs(p, b, b + o)


RNG = np.random.default_rng(3)


def test_the_filaments_match_biot_savart():
    P = RNG.normal(size=(5, 3))
    A, B = RNG.normal(size=(4, 3)), RNG.normal(size=(4, 3))
    P[0] = A[1]                                       # on a filament end: the core is zeroed
    P[1] = 0.3 * A[2] + 0.7 * B[2]                    # on a filament: collinear, zero
    got = vl._seg_influence(P, A, B)
    want = np.array([[_bs(p, a, b) for a, b in zip(A, B)] for p in P])
    assert got.shape == (5, 4, 3) and np.allclose(got, want, rtol=1e-12, atol=1e-15)
    assert np.all(got[0, 1] == 0.0) and np.all(got[1, 2] == 0.0)
    for p in P:
        for a, b in zip(A, B):
            assert np.allclose(vl._vortex_segment(p, a, b), _bs(p, a, b), rtol=1e-12, atol=1e-15)
            assert np.allclose(vl._horseshoe(p, a, b), _hs(p, a, b), rtol=1e-12, atol=1e-15)
            assert np.allclose(vl._horseshoe(p, a, b, far=3.0), _hs(p, a, b, 3.0), rtol=1e-12, atol=1e-15)
    hs = vl._horseshoe_influence(P, A, B)
    assert np.allclose(hs, [[_hs(p, a, b) for a, b in zip(A, B)] for p in P], rtol=1e-12, atol=1e-15)
    assert np.allclose(vl._horseshoe_influence(P, A, B, far=2.5),
                       [[_hs(p, a, b, 2.5) for a, b in zip(A, B)] for p in P], rtol=1e-12, atol=1e-15)
    a, b = np.zeros(3), np.array([0.0, 1.0, 0.0])
    near = np.array([0.0, 0.5, 1e-7])                 # inside the core: n2 under 1e-12
    assert np.all(vl._vortex_segment(near, a, b) == 0.0) and np.all(vl._seg_influence(near[None], a[None], b[None]) == 0)
    assert np.all(vl._vortex_segment(a + [1e-10, 0, 0], a, b) == 0.0)
    assert np.all(vl._vortex_segment(b + [1e-10, 0, 0], a, b) == 0.0)
    assert np.all(vl._seg_influence((a + [1e-10, 0, 0])[None], a[None], b[None]) == 0.0)
    assert np.all(vl._seg_influence((b + [1e-10, 0, 0])[None], a[None], b[None]) == 0.0)
    edge = np.array([0.0, 2.0, 1e-5])                 # n2 just above the core, ends far: kept
    assert np.allclose(vl._vortex_segment(edge, a, b), _bs(edge, a, b)) and np.any(vl._vortex_segment(edge, a, b))


def test_the_defaults():
    assert (vl.DEFAULT_SPANWISE, vl.DEFAULT_CHORDWISE) == (24, 12)
    m = vl.VortexLatticeModel()
    assert (m.n_span, m.n_chord, m.name) == (24, 12, "vortex-lattice (in-house)")
    assert issubclass(vl.VortexLatticeUnavailable, RuntimeError)


# ---------------------------------------------------------------- the lattice, written here
def _grid(ns=7, nc=5, c=0.3, b=1.2, m=0.02, z0=0.0, twist=0.0):
    ys = np.linspace(-b / 2, b / 2, ns)
    xs = np.linspace(0.0, c, nc + 1)
    return [[np.array([x, y, z0 + m * 4 * (x / c) * (1 - x / c) - twist * x * abs(y)]) for x in xs] for y in ys]


def _lattice(grid, V=20.0, yaw=0.0, h=None, ref=None, area=None):
    g = np.asarray(grid, float)
    Vinf = V * np.array([math.cos(math.radians(yaw)), -math.sin(math.radians(yaw)), 0.0])
    pan = []
    for i in range(g.shape[0] - 1):
        for j in range(g.shape[1] - 1):
            p00, p01, p10, p11 = g[i, j], g[i, j + 1], g[i + 1, j], g[i + 1, j + 1]
            n = np.cross(p01 - p00, p10 - p00)
            if np.linalg.norm(n) < 1e-12:
                continue
            a, b = p00 + 0.25 * (p01 - p00), p10 + 0.25 * (p11 - p10)
            cp = 0.5 * (p00 + 0.75 * (p01 - p00) + p10 + 0.75 * (p11 - p10))
            pan.append((a, b, cp, n / np.linalg.norm(n), abs(b[1] - a[1])))

    def vel(p, a, b):
        v = _hs(p, a, b)
        if h is not None:
            v = v + _hs(p, b * [1, 1, -1], a * [1, 1, -1])
        return v
    M = np.array([[vel(cp, a, b) @ n for a, b, *_ in pan] for _, _, cp, n, _ in pan])
    G = np.linalg.solve(M, [-(n @ Vinf) for _, _, _, n, _ in pan])
    dy = np.array([d for *_, d in pan])
    S = ref or area
    q = 0.5 * 1.225 * V * V
    L = 1.225 * V * np.sum(G * dy)
    w = np.array([sum(vel(0.5 * (a + b), aa, bb)[2] * gk for (aa, bb, *_), gk in zip(pan, G)) for a, b, *_ in pan])
    D = -1.225 * np.sum(w * G * dy)
    return L / (q * S), D / (q * S), len(pan), np.linalg.cond(M)


def _solve(monkeypatch, grid, span=1.2, chord=0.3, area=0.36, **att):
    seen = {}

    def fake(path, ns, nc):
        seen.update(path=path, ns=ns, nc=nc)
        return [list(r) for r in grid], span, chord, area
    monkeypatch.setattr(vl, "camber_surface_from_stl", fake)
    ref = att.pop("ref", None)
    spec = CaseSpec(attitude=Attitude(**att), geometry_path="wing.stl", reference_area_m2=ref)
    r = vl.VortexLatticeModel(n_span=7, n_chord=len(grid[0]) - 1).solve(spec)
    return r, seen


def test_free_air_lift_and_induced_drag(monkeypatch):
    g = _grid()
    r, seen = _solve(monkeypatch, g, ride_height_mm=0.0, speed_ms=25.0)
    cl, cd, n, cond = _lattice(g, V=25.0, area=0.36)
    assert seen == dict(path="wing.stl", ns=7, nc=5)
    assert r.c_lift == pytest.approx(cl, rel=1e-9) and r.c_drag == pytest.approx(cd, rel=1e-9) and cl > 0 and cd > 0
    assert (r.c_side, r.c_pitch, r.aero_balance_front, r.converged, r.force_monitor_range) == (0.0, None, None, True, 0.0)
    assert r.attitude.speed_ms == 25.0 and r.provenance.cell_count == n == 30
    assert r.notes == (f"vortex lattice: 6x5 panels, span 1.200 m, mean chord 0.300 m, cond={cond:.1e}. INDUCED drag "
                       "only — no profile drag, so total drag is a lower bound.")


def test_ground_effect_yaw_and_a_reference_area(monkeypatch):
    g = _grid(twist=0.05)
    r, _ = _solve(monkeypatch, g, ride_height_mm=120.0, yaw_deg=6.0, speed_ms=18.0, ref=0.5)
    gg = np.asarray(g)
    gg[:, :, 2] += 0.12 - gg[:, 0, 2].mean()
    cl, cd, n, cond = _lattice(gg, V=18.0, yaw=6.0, h=0.12, ref=0.5)
    assert r.c_lift == pytest.approx(cl, rel=1e-9) and r.c_drag == pytest.approx(cd, rel=1e-9)
    free = _lattice(gg, V=18.0, yaw=6.0, ref=0.5)[0]
    assert abs(cl - free) > 1e-3 * abs(free) and r.converged is True
    assert "Ride height" not in r.notes and "WARNING" not in r.notes


def test_the_default_speed_and_area(monkeypatch):
    g = _grid()
    r, _ = _solve(monkeypatch, g, ride_height_mm=0.0, speed_ms=0.0)
    assert r.c_lift == pytest.approx(_lattice(g, V=20.0, area=0.36)[0], rel=1e-9)
    r1, _ = _solve(monkeypatch, g, area=0.0, ride_height_mm=0.0)
    assert r1.c_lift == pytest.approx(_lattice(g, V=20.0, area=1.0)[0], rel=1e-9)


def _rot(g, roll=0.0, pitch=0.0):
    P = np.asarray(g, float).reshape(-1, 3).copy()
    if roll:
        c, s = math.cos(math.radians(roll)), math.sin(math.radians(roll))
        y, z = P[:, 1].copy(), P[:, 2].copy()
        P[:, 1], P[:, 2] = c * y - s * z, s * y + c * z
    if pitch:
        x0 = 0.5 * (P[:, 0].min() + P[:, 0].max())
        c, s = math.cos(math.radians(pitch)), math.sin(math.radians(pitch))
        x, z = P[:, 0] - x0, P[:, 2].copy()
        P[:, 0], P[:, 2] = c * x + s * z + x0, -s * x + c * z
    return P.reshape(np.asarray(g).shape)


def test_roll_and_pitch_turn_the_extracted_surface(monkeypatch):
    g = _grid(z0=0.05)
    for roll, pitch in ((4.0, 0.0), (0.0, -3.0), (3.0, 5.0)):
        r, _ = _solve(monkeypatch, g, ride_height_mm=0.0, roll_deg=roll, pitch_deg=pitch)
        assert r.c_lift == pytest.approx(_lattice(_rot(g, roll, pitch), area=0.36)[0], rel=1e-9)
        assert r.c_drag == pytest.approx(_lattice(_rot(g, roll, pitch), area=0.36)[1], rel=1e-9)


def test_the_flat_plate_and_the_clearance_notes(monkeypatch):
    flat = _grid(m=0.0)
    r, _ = _solve(monkeypatch, flat, ride_height_mm=0.0)
    assert abs(r.c_lift) < 1e-12
    assert r.notes.endswith(" C_L is exactly zero because the extracted camber surface is flat (0.00 mm of camber over "
                            "a 300 mm chord) and the incidence is zero. A flat plate aligned with the flow carries no "
                            "circulation, so this is the right answer, not a failed solve. Give it camber, pitch it, or "
                            "use the panel method if the part works by displacement rather than by lift.")
    pitched, _ = _solve(monkeypatch, flat, ride_height_mm=0.0, pitch_deg=2.0)
    assert "exactly zero" not in pitched.notes
    tiny = _grid(m=3e-5 / 0.96 * 0.9)                    # camber just under 1e-4 of the chord: still "flat"
    assert "exactly zero" in _solve(monkeypatch, tiny, ride_height_mm=0.0)[0].notes
    small = _grid(m=3e-5 / 0.96 * 1.1)
    assert "exactly zero" not in _solve(monkeypatch, small, ride_height_mm=0.0)[0].notes
    low, _ = _solve(monkeypatch, _grid(), ride_height_mm=12.0)
    assert low.converged is False and low.notes.endswith(
        " WARNING: ride height 12 mm is 0.040 of the mean chord. Below about 0.05 the ground image dominates and |C_L| "
        "runs away with falling ride height — the magnitude here is a numerical artefact, not downforce. Do not quote "
        "it, and do not compare geometries at this clearance.")
    mid, _ = _solve(monkeypatch, _grid(), ride_height_mm=24.0)
    assert mid.converged is True and mid.notes.endswith(
        " Ride height 24 mm is 0.080 of the mean chord: ground effect is strong and the absolute level is indicative. "
        "Deltas between geometries at this same attitude are still usable.")
    edge = _solve(monkeypatch, _grid(), ride_height_mm=15.0)[0]           # exactly 0.05: the milder note
    assert edge.converged is True and "0.050 of the mean chord: ground effect" in edge.notes
    top = _solve(monkeypatch, _grid(), ride_height_mm=30.0)[0]            # exactly 0.10: no note
    assert "Ride height" not in top.notes
    nochord = _solve(monkeypatch, _grid(), chord=0.0, ride_height_mm=12.0)[0]
    assert "ride height" not in nochord.notes.lower()


def test_refusals_and_an_ill_conditioned_lattice(monkeypatch):
    coarse = _grid(ns=3, nc=3)                                            # 2 x 3 = 6 panels
    with pytest.raises(vl.VortexLatticeUnavailable) as e:
        _solve(monkeypatch, coarse, ride_height_mm=0.0)
    assert str(e.value) == "camber surface too coarse to solve"
    dup = _grid(ns=5, nc=3)
    dup.insert(2, [p.copy() for p in dup[2]])                             # a zero-width strip: its panels drop out
    r, _ = _solve(monkeypatch, dup, ride_height_mm=0.0)
    assert r.provenance.cell_count == 12 and r.c_lift == pytest.approx(_lattice(dup, area=0.36)[0], rel=1e-9)
    eight = _grid(ns=3, nc=4)
    assert _solve(monkeypatch, eight, ride_height_mm=0.0)[0].provenance.cell_count == 8
    monkeypatch.setattr(np.linalg, "cond", lambda A: 2e10)
    bad, _ = _solve(monkeypatch, _grid(), ride_height_mm=0.0)
    assert bad.converged is False and bad.notes.endswith(
        "lower bound. WARNING: ill-conditioned lattice; treat the result as unreliable.")
    monkeypatch.setattr(np.linalg, "cond", lambda A: 1e10)
    assert _solve(monkeypatch, _grid(), ride_height_mm=0.0)[0].converged is True

    def boom(A, b):
        raise np.linalg.LinAlgError("Singular matrix")
    monkeypatch.setattr(np.linalg, "solve", boom)
    with pytest.raises(vl.VortexLatticeUnavailable) as e:
        _solve(monkeypatch, _grid(), ride_height_mm=0.0)
    assert str(e.value) == "lattice solve failed: Singular matrix"


def test_the_provenance():
    p = vl.VortexLatticeModel().provenance()
    assert (p.backend, p.fidelity, p.is_correlated, p.cell_count) == ("vortex-lattice (in-house)",
                                                                       SolverFidelity.POTENTIAL, False, None)
    assert p.turbulence_model == "none (inviscid vortex lattice, induced drag only)"
    assert p.notes.startswith("In-house horseshoe vortex-lattice solve on the mean camber surface extracted from the "
                              "STL, with image vortices in the road plane.")
    assert p.notes.endswith("Trust lift, span loading and the ground-effect trend; correlate against the tunnel or "
                            "Fluent before quoting absolutes.")
    assert vl.VortexLatticeModel().provenance(42).notes == p.notes + " [42 panels]"
    assert vl.VortexLatticeModel().provenance(42).cell_count == 42


# ---------------------------------------------------------------- the camber extractor
C, M, T = 0.3, 0.015, 0.012
XS = np.linspace(0.0, C, 121)
CAMB = M * 4 * (XS / C) * (1 - XS / C)
HALF = T * 2 * np.sqrt(np.clip(XS / C * (1 - XS / C), 0, None)) + 0.0004


def _wing(path, y0=-0.5, y1=0.6, thick=1.0):
    loop = np.r_[np.c_[XS, CAMB + thick * HALF], np.c_[XS[::-1], (CAMB - thick * HALF)[::-1]]]
    K = len(loop)
    V = [[x, y, z] for y in (y0, y1) for x, z in loop] + [[loop[:, 0].mean(), y0, loop[:, 1].mean()],
                                                          [loop[:, 0].mean(), y1, loop[:, 1].mean()]]
    F = []
    for k in range(K):
        a, b = k, (k + 1) % K
        F += [[a, b, K + b], [a, K + b, K + a], [2 * K, b, a], [2 * K + 1, K + a, K + b]]
    trimesh.Trimesh(np.array(V), np.array(F), process=False).export(path)
    return path


def _camber_at(xq):
    up = np.interp(xq, XS, CAMB + HALF)
    lo = np.interp(xq, XS, CAMB - HALF)
    return 0.5 * (up + lo)


def test_the_camber_surface_of_a_lofted_section(tmp_path):
    grid, span, chord, area = vl.camber_surface_from_stl(_wing(str(tmp_path / "w.stl")), n_span=6, n_chord=8)
    assert span == pytest.approx(1.1) and chord == pytest.approx(C) and area == pytest.approx(1.1 * C)
    assert len(grid) == 6 and all(len(r) == 9 for r in grid)
    ys = np.linspace(-0.5 + 0.022, 0.6 - 0.022, 6)
    for i, row in enumerate(grid):
        for j, p in enumerate(row):
            f = j / 8
            xq = 1e-4 * C if j == 0 else (C - 1e-4 * C if j == 8 else f * C)
            assert p == pytest.approx([xq, ys[i], _camber_at(xq)], abs=2e-7)          # STL stores float32
    g2 = vl.camber_surface_from_stl(str(tmp_path / "w.stl"))[0]
    assert len(g2) == 24 and len(g2[0]) == 13


def test_the_extractor_refuses_what_is_not_a_wing(tmp_path, monkeypatch):
    def why(*a, **k):
        with pytest.raises(vl.VortexLatticeUnavailable) as e:
            vl.camber_surface_from_stl(*a, **k)
        return str(e.value)
    assert why(str(tmp_path / "missing.stl")) == f"geometry '{tmp_path / 'missing.stl'}' not found on disk"
    assert why("") == "geometry '' not found on disk"
    box = tmp_path / "box.stl"
    trimesh.creation.box(extents=(0.3, 1.0, 0.2)).export(str(box))
    assert why(str(box)) == ("this geometry is 200 mm thick against a 300 mm chord — too thick to be a lifting surface. "
                             "Use the panel-method solver, which is built for bluff bodies in ground effect.")
    slab = tmp_path / "slab.stl"
    trimesh.creation.box(extents=(0.3, 1.0, 0.15)).export(str(slab))       # exactly half the chord: still a wing
    assert len(vl.camber_surface_from_stl(str(slab), n_span=5, n_chord=4)[0]) == 5
    w = _wing(str(tmp_path / "w.stl"))
    assert why(w, n_span=3) == ("could not extract a camber surface — the geometry does not section cleanly into upper "
                                "and lower surfaces. Check the STL is a closed wing rather than an open shell.")
    assert len(vl.camber_surface_from_stl(w, n_span=4, n_chord=4)[0]) == 4
    monkeypatch.setitem(sys.modules, "trimesh", None)
    assert why(w).startswith("trimesh not available: ")


def test_an_empty_mesh_has_no_triangles(tmp_path, monkeypatch):
    p = _wing(str(tmp_path / "w.stl"))
    monkeypatch.setattr(trimesh, "load", lambda *a, **k: trimesh.Trimesh())
    with pytest.raises(vl.VortexLatticeUnavailable) as e:
        vl.camber_surface_from_stl(p)
    assert str(e.value) == "geometry has no triangles"


def test_a_mesh_that_will_not_load(tmp_path, monkeypatch):
    p = _wing(str(tmp_path / "w.stl"))

    def bad(*a, **k):
        raise ValueError("not an STL")
    monkeypatch.setattr(trimesh, "load", bad)
    with pytest.raises(vl.VortexLatticeUnavailable) as e:
        vl.camber_surface_from_stl(p)
    assert str(e.value) == f"could not load '{p}': not an STL"


# ---------------------------------------------------------------- added: guards, notes and the extractor fallbacks
def test_a_point_just_off_a_filament_end_is_still_guarded():
    A = np.array([0.0, 0.0, 0.0])
    B = np.array([0.0, 2e4, 0.0])
    P = A + np.array([1e-10, 0.0, 0.0])                   # |r1| < 1e-9 but |r1 x r2|^2 = 4e-12
    assert float(np.cross(P - A, P - B) @ np.cross(P - A, P - B)) > 1e-12
    assert np.all(vl._vortex_segment(P, A, B) == 0.0)
    assert np.all(vl._seg_influence(P[None], A[None], B[None]) == 0.0)
    Q = B + np.array([1e-10, 0.0, 0.0])                   # the same at the far end
    assert np.all(vl._vortex_segment(Q, A, B) == 0.0)
    assert np.all(vl._seg_influence(Q[None], A[None], B[None]) == 0.0)
    on = np.array([[0.0, 1.0, 0.0]])                       # on the filament itself
    assert np.all(vl._seg_influence(on, A[None], B[None]) == 0.0)
    off = np.array([[0.3, 1.0, 0.2]])
    assert vl._seg_influence(off, A[None], B[None])[0, 0] == pytest.approx(_bs(off[0], A, B), rel=1e-12)


def test_a_collapsed_chordwise_column_is_skipped_panel_by_panel(monkeypatch):
    g = _grid(ns=6, nc=5)
    for row in g:
        row[1] = row[0].copy()                             # first chordwise panel of every strip has no area
    r, _ = _solve(monkeypatch, g, ride_height_mm=0.0)
    cl, cd, n, _ = _lattice(g, area=0.36)
    assert r.provenance.cell_count == n == 20
    assert r.c_lift == pytest.approx(cl, rel=1e-9) and r.c_drag == pytest.approx(cd, rel=1e-9)


def test_ride_height_is_measured_from_the_leading_edge(monkeypatch):
    g = _grid(z0=0.04, twist=0.05)
    r, _ = _solve(monkeypatch, g, ride_height_mm=150.0)
    gg = np.asarray(g)
    gg[:, :, 2] += 0.15 - gg[:, 0, 2].mean()
    cl, cd, _, _ = _lattice(gg, h=0.15, area=0.36)
    assert r.c_lift == pytest.approx(cl, rel=1e-9) and r.c_drag == pytest.approx(cd, rel=1e-9)


def test_flat_but_lifted_or_pitched_plates(monkeypatch):
    lifted = _grid(m=0.0, z0=0.05)
    r, _ = _solve(monkeypatch, lifted, ride_height_mm=0.0)
    assert "exactly zero because the extracted camber surface is flat (0.00 mm" in r.notes
    tilt, _ = _solve(monkeypatch, _grid(m=0.0), ride_height_mm=0.0, pitch_deg=0.5)
    assert "exactly zero" not in tilt.notes


def _bump(peak, ns=7, nc=4, c=0.3):
    g = _grid(ns=ns, nc=nc, c=c, m=0.0)
    for row in g:
        row[2] = row[2] + np.array([0.0, 0.0, peak])
    return g


def test_the_note_numbers(monkeypatch):
    r, _ = _solve(monkeypatch, _bump(9.949e-4), chord=10.0, ride_height_mm=0.0)
    assert r.notes == (
        f"vortex lattice: 6x4 panels, span 1.200 m, mean chord 10.000 m, cond={r.notes.split('cond=')[1][:7]}. "
        "INDUCED drag only — no profile drag, so total drag is a lower bound. C_L is exactly zero because the "
        "extracted camber surface is flat (0.99 mm of camber over a 10000 mm chord) and the incidence is zero. A flat "
        "plate aligned with the flow carries no circulation, so this is the right answer, not a failed solve. Give it "
        "camber, pitch it, or use the panel method if the part works by displacement rather than by lift.")
    low, _ = _solve(monkeypatch, _grid(), chord=13.0, ride_height_mm=600.0)
    assert " WARNING: ride height 600 mm is 0.046 of the mean chord." in low.notes
    assert low.notes.startswith("vortex lattice: ")
    mid, _ = _solve(monkeypatch, _grid(), chord=8.0, ride_height_mm=600.0)
    assert " Ride height 600 mm is 0.075 of the mean chord:" in mid.notes
    assert mid.notes.startswith("vortex lattice: ")


def test_the_provenance_note_word_for_word():
    p = vl.VortexLatticeModel().provenance()
    assert p.notes == (
        "In-house horseshoe vortex-lattice solve on the mean camber surface extracted from the STL, with image vortices "
        "in the road plane. The Kutta condition is enforced geometrically by the 1/4-3/4 rule. Validated against "
        "closed form on a rectangular AR=8 wing: C_L within 6% of lifting-line theory and linear in alpha, span "
        "efficiency e=1.004 against the ideal induced drag, grid-converged under 0.5%. INVISCID and attached by "
        "construction, and a thin-surface method: no stall, no separation, no profile drag and no thickness effects. "
        "The drag reported is INDUCED ONLY, so total drag is a lower bound and a real wing will always do worse. Trust "
        "lift, span loading and the ground-effect trend; correlate against the tunnel or Fluent before quoting "
        "absolutes.")


def _prism(path, xz, y0=-0.5, y1=0.5):
    """Extrude a closed (x, z) polygon along y, capped at both ends."""
    xz = np.asarray(xz, float)
    K = len(xz)
    V = [[x, y, z] for y in (y0, y1) for x, z in xz]
    V += [[xz[:, 0].mean(), y0, xz[:, 1].mean()], [xz[:, 0].mean(), y1, xz[:, 1].mean()]]
    F = []
    for k in range(K):
        a, b = k, (k + 1) % K
        F += [[a, b, K + b], [a, K + b, K + a], [2 * K, b, a], [2 * K + 1, K + a, K + b]]
    trimesh.Trimesh(np.array(V), np.array(F), process=False).export(path)
    return path


HEX = [(0.0, 0.0), (0.0625, 0.01), (0.1875, 0.0125), (0.25, 0.0), (0.1875, -0.002), (0.0625, -0.004)]


def test_a_six_vertex_section_is_enough(tmp_path):
    grid, span, chord, _ = vl.camber_surface_from_stl(_prism(str(tmp_path / "h.stl"), HEX), n_span=5, n_chord=4)
    assert len(grid) == 5 and all(len(r) == 5 for r in grid)
    assert chord == pytest.approx(0.25) and span == pytest.approx(1.0)
    mid = grid[2][1]                                       # x = 0.0625: a vertex column, upper 0.01, lower -0.004
    assert mid[2] == pytest.approx(0.003, abs=1e-7)


def test_a_scene_file_is_loaded_as_one_mesh(tmp_path):
    a = trimesh.load(_prism(str(tmp_path / "a.stl"), HEX, -0.5, 0.0))
    b = trimesh.load(_prism(str(tmp_path / "b.stl"), HEX, 0.0, 0.5))
    path = str(tmp_path / "two.glb")
    trimesh.Scene([a, b]).export(path)
    grid, span, chord, _ = vl.camber_surface_from_stl(path, n_span=6, n_chord=4)
    assert len(grid) == 6 and span == pytest.approx(1.0) and chord == pytest.approx(0.25)


def test_a_scene_with_no_faces_and_a_section_that_throws(tmp_path, monkeypatch):
    p = _prism(str(tmp_path / "h.stl"), HEX)
    real = trimesh.load
    monkeypatch.setattr(trimesh, "load", lambda *a, **k: trimesh.Scene())
    with pytest.raises(vl.VortexLatticeUnavailable) as e:
        vl.camber_surface_from_stl(p)
    assert str(e.value) == "geometry has no triangles"
    monkeypatch.setattr(trimesh, "load", real)

    def boom(self, *a, **k):
        raise ValueError("no")
    monkeypatch.setattr(trimesh.Trimesh, "section", boom)
    with pytest.raises(vl.VortexLatticeUnavailable) as e:
        vl.camber_surface_from_stl(p, n_span=5, n_chord=4)
    assert str(e.value).startswith("could not extract a camber surface")


def test_the_thickness_refusal_in_millimetres(tmp_path):
    box = str(tmp_path / "b.stl")
    trimesh.creation.box(extents=(1.0, 2.0, 0.6)).export(box)
    with pytest.raises(vl.VortexLatticeUnavailable) as e:
        vl.camber_surface_from_stl(box)
    assert str(e.value).startswith("this geometry is 600 mm thick against a 1000 mm chord —")


def _no_outline(monkeypatch):
    def boom(self):
        raise ValueError("no outline")
    monkeypatch.setattr(trimesh.path.Path3D, "discrete", property(boom))


SPARSE = [(0.0, 0.0), (0.0703125, 0.01), (0.125, 0.0125), (0.1796875, 0.008), (0.25, 0.0),
          (0.1796875, 0.0), (0.125, -0.001), (0.0703125, -0.002)]


def _band_z(xq, xz, c=0.25):
    xz = np.asarray(xz)
    d = np.abs(xz[:, 0] - xq)
    band = d <= max(0.02 * c, 1e-4)
    if not band.any():
        band = d <= 0.06 * c
    if not band.any():
        return None
    return 0.5 * (xz[band, 1].max() + xz[band, 1].min())


def test_without_an_outline_the_vertex_band_is_used(tmp_path, monkeypatch):
    p = _prism(str(tmp_path / "s.stl"), SPARSE)
    _no_outline(monkeypatch)
    grid, _, chord, _ = vl.camber_surface_from_stl(p, n_span=5, n_chord=4)
    assert chord == pytest.approx(0.25) and len(grid) == 5
    mesh = trimesh.load(p)
    for row in grid:
        assert len(row) == 5
        sec = mesh.section(plane_origin=[0.0, row[0][1], 0.0], plane_normal=[0.0, 1.0, 0.0])
        xz = np.asarray(sec.vertices)[:, [0, 2]]
        x0, x1 = xz[:, 0].min(), xz[:, 0].max()
        c = x1 - x0
        xs = [x0 + 1e-4 * c, x0 + 0.25 * c, x0 + 0.5 * c, x0 + 0.75 * c, x1 - 1e-4 * c]
        for x, pnt in zip(xs, row):
            assert pnt[0] == pytest.approx(x, abs=1e-12)
            assert pnt[2] == pytest.approx(_band_z(x, xz, c), abs=1e-12)


def test_a_station_with_no_nearby_vertex_drops_the_whole_section(tmp_path, monkeypatch):
    holey = [(0.0, 0.0), (0.0703125, 0.01), (0.125, 0.0125), (0.2109375, 0.008), (0.25, 0.0),
             (0.2109375, 0.0), (0.125, -0.001), (0.0703125, -0.002)]
    p = _prism(str(tmp_path / "s.stl"), holey)
    _no_outline(monkeypatch)
    with pytest.raises(vl.VortexLatticeUnavailable) as e:
        vl.camber_surface_from_stl(p, n_span=5, n_chord=4)
    assert str(e.value).startswith("could not extract a camber surface")


def _outline(monkeypatch, extra_first=(), extra_last=()):
    real = trimesh.path.Path3D.discrete

    def disc(self):
        loops = [np.asarray(d) for d in real.fget(self)]
        y = float(loops[0][0, 1])
        add = lambda pts: [np.asarray([[x, y, z] for x, z in q]) if np.ndim(q) == 2 else np.asarray(q)
                           for q in pts]
        return add(extra_first) + loops + add(extra_last)
    monkeypatch.setattr(trimesh.path.Path3D, "discrete", property(disc))


def test_odd_outline_loops(tmp_path, monkeypatch):
    p = _prism(str(tmp_path / "h.stl"), HEX)
    base = vl.camber_surface_from_stl(p, n_span=5, n_chord=4)[0]
    _outline(monkeypatch, extra_first=[[(0.1, 5.0)], np.array([0.1, 0.2, 0.3])])
    same = vl.camber_surface_from_stl(p, n_span=5, n_chord=4)[0]
    assert np.allclose(np.asarray(same), np.asarray(base), atol=1e-12)
    monkeypatch.undo()
    _outline(monkeypatch, extra_last=[[(-1.0, 1.0), (2.0, 1.0)]])          # a two-point loop over the whole chord
    hi = np.asarray(vl.camber_surface_from_stl(p, n_span=5, n_chord=4)[0])
    lower = np.interp(hi[0, :, 0], [0.0, 0.0625, 0.1875, 0.25], [0.0, -0.004, -0.002, 0.0])
    assert hi[0, :, 2] == pytest.approx(0.5 * (1.0 + lower), abs=1e-7)
    monkeypatch.undo()
    _outline(monkeypatch, extra_last=[[(0.0625, 5.0), (0.0625, 6.0)]])    # vertical, exactly on a station
    v = np.asarray(vl.camber_surface_from_stl(p, n_span=5, n_chord=4)[0])
    assert v[0, 1, 2] == pytest.approx(0.5 * (6.0 - 0.004), abs=1e-7)
    assert v[0, 2, 2] == pytest.approx(np.asarray(base)[0, 2, 2], abs=1e-12)


def test_six_section_vertices_pass_and_five_are_skipped(tmp_path, monkeypatch):
    tri = _prism(str(tmp_path / "t.stl"), [(0.0, 0.0), (0.25, 0.01), (0.25, -0.01)])
    assert len(trimesh.load(tri).section(plane_origin=[0, 0.1, 0], plane_normal=[0, 1, 0]).vertices) == 6
    assert len(vl.camber_surface_from_stl(tri, n_span=5, n_chord=4)[0]) == 5
    hexa = _prism(str(tmp_path / "h.stl"), HEX)
    real = trimesh.Trimesh.section
    calls = []

    class Few:
        vertices = np.zeros((5, 3))

    def sec(self, *a, **k):
        calls.append(1)
        return Few() if len(calls) == 1 else real(self, *a, **k)
    monkeypatch.setattr(trimesh.Trimesh, "section", sec)
    grid = vl.camber_surface_from_stl(hexa, n_span=6, n_chord=4)[0]
    assert len(grid) == 5 and grid[0][0][1] > -0.45


def test_missing_and_degenerate_sections_are_skipped(tmp_path, monkeypatch):
    hexa = _prism(str(tmp_path / "h.stl"), HEX)
    real = trimesh.Trimesh.section

    class Sliver:
        vertices = np.array([[0.1, 0.0, z] for z in np.linspace(0.0, 0.01, 8)])

    for first in (None, Sliver()):
        calls = []

        def sec(self, *a, **k):
            calls.append(1)
            return first if len(calls) == 1 else real(self, *a, **k)
        monkeypatch.setattr(trimesh.Trimesh, "section", sec)
        grid = vl.camber_surface_from_stl(hexa, n_span=6, n_chord=4)[0]
        assert len(grid) == 5 and grid[0][0][1] > -0.45
        monkeypatch.undo()
