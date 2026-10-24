# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""suspension/aero/underfloor.py, the quasi-1D floor duct, against its own physics: a constant-area duct makes no
force, suction is discharge * (1 - (A_flow/A)^2) up to the throat, force scales with dynamic pressure, the exit sets
the flow until the inlet spills, the diffuser half-angle screen, and the STL path against the parametric one on the
same floor. SI units (m, m^2, N, Pa) except where a name says mm."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension.aero import underfloor as uf                     # noqa: E402

trimesh = pytest.importorskip("trimesh")


def _duct(areas, width=0.8, length=1.5):
    xs = np.linspace(0.0, length, len(areas))
    return xs, np.asarray(areas, float), np.full(len(areas), width)


# --------------------------------------------------------------------------- #
#  The channel solve
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("discharge", [0.5, 0.75, 1.0])
def test_a_constant_area_duct_makes_no_force(discharge):
    xs, a, w = _duct([0.03] * 40)
    r = uf._solve_channel(xs, a, w, 20.0, 1.24, 1.225, discharge)
    assert r.downforce_N == 0.0 and r.c_lift == 0.0 and r.area_ratio == 1.0


def test_suction_at_the_throat_is_continuity_and_bernoulli():
    """Ahead of the throat Cp = discharge * (1 - (A_flow/A)^2) with A_flow the exit area (under the spill limit);
    the force is the pressure integrated over the wetted floor. Checked here by integrating that Cp independently."""
    a_in, a_t, a_ex = 0.040, 0.020, 0.044
    n = 61
    xs = np.linspace(0.0, 1.5, n)
    a = np.interp(xs, [0.0, 0.6, 1.5], [a_in, a_t, a_ex])
    w = np.full(n, 0.8)
    r = uf._solve_channel(xs, a, w, 20.0, 1.24, 1.225, 0.75)
    i_t = int(np.argmin(a))
    cp = 0.75 * (1 - (min(a_ex, uf.SPILL_LIMIT * a_in) / a) ** 2)
    f = (xs[i_t:] - xs[i_t]) / (xs[-1] - xs[i_t])
    cp[i_t:] = cp[i_t] * (1 - 0.8 * f)                           # 80% recovery toward ambient at the exit
    q = 0.5 * 1.225 * 20.0 ** 2
    lift = np.sum(cp * q * w * np.gradient(xs))
    assert r.downforce_N == pytest.approx(-lift, rel=1e-12)
    assert r.c_lift == pytest.approx(lift / (q * 1.24), rel=1e-12)
    assert r.area_ratio == pytest.approx(a_in / a_t) and r.throat_x_frac == pytest.approx(xs[i_t] / 1.5)


def test_force_scales_with_dynamic_pressure_and_linearly_with_discharge():
    base = uf.solve_parametric(speed_ms=20.0)
    assert uf.solve_parametric(speed_ms=40.0).downforce_N == pytest.approx(4 * base.downforce_N, rel=1e-12)
    assert uf.solve_parametric(speed_ms=40.0).c_lift == pytest.approx(base.c_lift, rel=1e-12)
    assert uf.solve_parametric(rho=1.0).downforce_N == pytest.approx(base.downforce_N / 1.225, rel=1e-12)
    assert uf.solve_parametric(discharge=0.375).downforce_N == pytest.approx(base.downforce_N / 2, rel=1e-12)


def test_the_exit_sets_the_flow_until_the_inlet_spills():
    """A bigger exit pulls more air under the car and deepens the suction, until the exit asks for more than
    SPILL_LIMIT times the inlet can supply; past that the force stops growing."""
    f = [uf.solve_parametric(exit_rise_mm=e).downforce_N for e in (40.0, 60.0, 90.0)]
    assert f[0] < f[1] < f[2]
    a_in = uf.channel_from_params()[1][0]
    big = [uf.solve_parametric(exit_rise_mm=e) for e in (200.0, 250.0)]
    assert uf.channel_from_params(exit_rise_mm=200.0)[1][-1] > uf.SPILL_LIMIT * a_in
    assert big[0].downforce_N == pytest.approx(big[1].downforce_N, rel=1e-12)


def test_lowering_the_car_raises_the_suction_without_bound_while_the_inlet_has_a_ramp():
    """With an inlet ramp, the inlet/throat ratio is about (rise + h)/h, so it grows without limit as h -> 0 and so
    does the force. The model has no ground-effect stall: below about 20 mm on this floor treat it as an upper
    bound."""
    hs = [60.0, 40.0, 20.0, 10.0]
    rs = [uf.solve_parametric(ride_height_mm=h) for h in hs]
    assert all(a.downforce_N < b.downforce_N for a, b in zip(rs, rs[1:]))
    assert all(a.area_ratio < b.area_ratio for a, b in zip(rs, rs[1:]))
    flat = [uf.solve_parametric(ride_height_mm=h, inlet_rise_mm=0.0) for h in (40.0, 10.0)]
    assert flat[1].area_ratio == pytest.approx(1.0) and flat[0].area_ratio == pytest.approx(1.0)


def test_the_diffuser_screen_is_the_equivalent_conical_half_angle():
    xs, a, w = _duct(np.interp(np.linspace(0, 1, 50), [0, 0.4, 1], [0.04, 0.02, 0.06]))
    r = uf._solve_channel(xs, a, w, 20.0, 1.24, 1.225, 0.75)
    i_t = int(np.argmin(a))
    ang = math.degrees(math.atan2(math.sqrt(a[-1] / math.pi) - math.sqrt(a[i_t] / math.pi), xs[-1] - xs[i_t]))
    assert r.diffuser_angle_deg == pytest.approx(ang)
    assert r.attached == (ang <= uf.SEPARATION_ANGLE_DEG)
    steep = uf.solve_parametric(exit_rise_mm=250.0, throat_frac=0.7)
    assert not steep.attached and "WARNING" in steep.notes and "upper bound" in steep.notes
    gentle = uf.solve_parametric(exit_rise_mm=40.0)
    assert gentle.attached and "inside the" in gentle.notes


def test_a_closed_gap_is_refused():
    xs, a, w = _duct([0.03, 0.0, 0.03, 0.04])
    with pytest.raises(ValueError, match="gap closes"):
        uf._solve_channel(xs, a, w, 20.0, 1.24, 1.225, 0.75)


def test_a_duct_that_only_converges_has_no_diffuser_to_recover_over():
    """Throat at the trailing edge: no ramp, so continuity holds all the way. The flow is set by the exit, the
    smallest area, so everywhere ahead of it the air is slower than at the exit and the pressure is above ambient:
    a floor that only converges makes lift."""
    xs, a, w = _duct(np.linspace(0.04, 0.02, 30))
    r = uf._solve_channel(xs, a, w, 20.0, 1.24, 1.225, 0.75)
    cp = 0.75 * (1 - (0.02 / a) ** 2)
    q = 0.5 * 1.225 * 400.0
    assert r.downforce_N == pytest.approx(-np.sum(cp * q * w * np.gradient(xs)), rel=1e-12)
    assert r.throat_x_frac == 1.0 and r.downforce_N < 0


def test_turned_up_edges_take_area_out_of_the_duct():
    xs, a0, w = uf.channel_from_params(edge_fence=0.0)
    _, a1, _ = uf.channel_from_params(edge_fence=0.55)
    assert np.allclose(a1, a0 * (1 - 0.84 * 0.55)) and np.all(w == 0.8)
    assert np.min(a0) == pytest.approx(0.04 * 0.8, rel=2e-3)            # throat = ride height x width


# --------------------------------------------------------------------------- #
#  The STL path
# --------------------------------------------------------------------------- #
def _floor_stl(path, relief_fn, length=1.5, width=0.8, nx=91, ny=21, z0=0.1):
    """A downward-facing floor surface z = z0 + relief(x), triangulated on a grid, closed into a thin slab."""
    xs, ys = np.linspace(0, length, nx), np.linspace(-width / 2, width / 2, ny)
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    Z = z0 + relief_fn(X)
    verts = np.column_stack([X.ravel(), Y.ravel(), Z.ravel()])
    faces = []
    for i in range(nx - 1):
        for j in range(ny - 1):
            a, b, c, d = i * ny + j, (i + 1) * ny + j, (i + 1) * ny + j + 1, i * ny + j + 1
            faces += [[a, c, b], [a, d, c]]                      # wound so the normal points down
    mesh = trimesh.Trimesh(vertices=verts, faces=np.array(faces), process=False)
    assert (mesh.face_normals[:, 2] < 0).all()
    mesh.export(path)
    return mesh


def test_a_flat_underside_gives_no_force_from_the_stl(tmp_path):
    p = tmp_path / "flat.stl"
    _floor_stl(p, lambda x: 0.0 * x)
    r = uf.solve(str(p), ride_height_mm=40.0, speed_ms=20.0, ref_area_m2=1.24)
    assert r.downforce_N == pytest.approx(0.0, abs=1e-9) and r.area_ratio == pytest.approx(1.0)


def test_the_stl_and_parametric_paths_agree_on_the_same_floor(tmp_path):
    """The same relief profile, meshed finely and described by numbers (no edge fences), gives the same throat
    position, area ratio and force within the STL path's sampling: it takes the lowest point in each grid cell, which
    reads the steep inlet slightly low and so the spill-limited flow slightly low (about 5% here)."""
    kw = dict(length_m=1.5, width_m=0.8, throat_frac=0.45, inlet_rise_mm=55.0, exit_rise_mm=90.0, ride_height_mm=40.0,
              edge_fence=0.0)
    xs, area, _ = uf.channel_from_params(nx=400, **kw)
    relief = area / 0.8 - 0.040
    p = tmp_path / "floor.stl"
    _floor_stl(p, lambda x: np.interp(x, xs, relief), nx=321, ny=161)
    stl = uf.solve(str(p), ride_height_mm=40.0, speed_ms=20.0, ref_area_m2=1.24, nx=80, ny=40)
    par = uf.solve_parametric(speed_ms=20.0, ref_area_m2=1.24, **kw)
    assert stl.throat_x_frac == pytest.approx(par.throat_x_frac, abs=0.03)
    assert stl.area_ratio == pytest.approx(par.area_ratio, rel=0.05)
    assert stl.downforce_N == pytest.approx(par.downforce_N, rel=0.10)


def test_meshes_that_cannot_form_a_duct_are_refused(tmp_path):
    up = tmp_path / "up.stl"
    m = trimesh.creation.box(extents=[1.0, 0.8, 0.1])
    keep = m.face_normals[:, 2] > 0.5
    trimesh.Trimesh(vertices=m.vertices, faces=m.faces[keep]).export(up)
    with pytest.raises(ValueError, match="no underside"):
        uf.solve(str(up), 40.0, 20.0, 1.24)
    narrow = tmp_path / "narrow.stl"
    _floor_stl(narrow, lambda x: 0.0 * x, length=0.002, nx=2)
    with pytest.raises(ValueError, match="too narrow"):
        uf.solve(str(narrow), 40.0, 20.0, 1.24)
    empty = tmp_path / "empty.stl"
    empty.write_text("solid empty\nendsolid empty\n")
    with pytest.raises(ValueError):
        uf.solve(str(empty), 40.0, 20.0, 1.24)


# --------------------------------------------------------------------------- #
#  The sweep
# --------------------------------------------------------------------------- #
def test_a_sweep_reports_change_against_its_baseline_and_flags_separation():
    rows = uf.sweep("ride_height_mm", [30.0, 40.0, 50.0], baseline={"ride_height_mm": 40.0})
    base = next(r for r in rows if r["ride_height_mm"] == 40.0)
    assert base["vs_baseline_pct"] == 0.0
    for r in rows:
        assert r["vs_baseline_pct"] == pytest.approx(100 * (r["downforce_N"] - base["downforce_N"])
                                                     / base["downforce_N"], abs=0.06)
        assert set(r) >= {"C_L", "downforce_N", "inlet/throat", "diffuser_deg", "attached"}
    grid = uf.sweep("throat_frac", [0.3, 0.5], "exit_rise_mm", [60.0, 250.0])
    assert len(grid) == 4 and {r["exit_rise_mm"] for r in grid} == {60.0, 250.0}
    assert any(not r["attached"] for r in grid) and any(r["attached"] for r in grid)


def test_a_sweep_skips_cells_that_cannot_be_solved_and_defaults_to_the_median_cell():
    rows = uf.sweep("ride_height_mm", [0.0, 30.0, 40.0, 50.0], inlet_rise_mm=55.0, nx=101)   # throat on a station
    assert [r["ride_height_mm"] for r in rows] == [30.0, 40.0, 50.0]      # 0 mm: the throat closes
    assert next(r for r in rows if r["ride_height_mm"] == 40.0)["vs_baseline_pct"] == 0.0
    assert uf.sweep("ride_height_mm", [0.0], inlet_rise_mm=0.0, exit_rise_mm=0.0) == []
    assert uf._median([3, 1, 2]) == 2 and uf._median([4, 1, 3, 2]) == 2.5
