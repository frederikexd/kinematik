"""suspension/aero/underfloor.py against the quasi-1D channel written out here: continuity with the spill limit,
Bernoulli with the discharge coefficient, the linear aft recovery, the strip-summed lift, the equivalent conical
diffuser angle, the parametric duct shape, the underside binning on hand-placed triangles, and the sweep table with
its baseline percentages. Every note and refusal is checked word for word."""
import math

import numpy as np
import pytest
import trimesh

from suspension.aero import underfloor as uf


def test_the_constants_and_the_parameter_table():
    assert (uf.SEPARATION_ANGLE_DEG, uf.DEFAULT_DISCHARGE, uf.SPILL_LIMIT) == (7.0, 0.75, 1.15)
    assert uf.PARAMS == {"throat_frac": (0.20, 0.75, "throat position (fraction of length)"),
                         "inlet_rise_mm": (0.0, 120.0, "inlet height above the throat (mm)"),
                         "exit_rise_mm": (0.0, 250.0, "diffuser exit height above the throat (mm)"),
                         "ride_height_mm": (15.0, 90.0, "throat clearance to the road (mm)")}


def _chan(xs, area, width, V, S, rho, cd):
    xs, area, width = (np.asarray(a, float) for a in (xs, area, width))
    it = int(np.argmin(area))
    a_in, a_t, a_e = area[0], area[it], area[-1]
    vr = min(a_e, 1.15 * a_in) / area
    cp = cd * (1 - vr ** 2)
    aft = np.arange(len(area)) >= it
    x = xs[aft]
    f = (x - x[0]) / (x[-1] - x[0]) if len(x) > 1 else 0.0
    cp[aft] = cp[it] * (1 - 0.8 * f)
    q = 0.5 * rho * V * V
    L = float(np.sum(cp * q * width * np.gradient(xs)))
    ang = math.degrees(math.atan2(math.sqrt(a_e / math.pi) - math.sqrt(a_t / math.pi), xs[-1] - xs[it]))
    span = xs[-1] - xs[0]
    return dict(c_lift=L / (q * S), downforce_N=-L, throat_x_frac=(xs[it] - xs[0]) / span, area_ratio=a_in / a_t,
                diffuser_angle_deg=ang, attached=ang <= 7.0)


def _check(r, want):
    for k, v in want.items():
        assert getattr(r, k) == pytest.approx(v, rel=1e-12, abs=1e-15), k


def _note(xt_pct, ratio, cd, ang):
    head = (f"quasi-1D underfloor: throat at {xt_pct:.0f}% of floor length, inlet/throat area ratio {ratio:.2f}, "
            f"discharge coefficient {cd:.2f}. Incompressible continuity plus Bernoulli — one-dimensional, so no "
            "spanwise variation, no edge vortices and no yaw sensitivity.")
    if ang > 7.0:
        return head + (f" WARNING: the diffuser expands at an equivalent {ang:.1f}° half-angle, past the 7° where a "
                       "plane diffuser is normally taken to separate. This model assumes the flow stays attached and "
                       "recovers the pressure, so the real part will make LESS than this. Treat the number as an "
                       "upper bound and shallow the ramp.")
    return head + f" Diffuser half-angle {ang:.1f}°, inside the 7° attachment limit."


def test_the_channel_physics():
    xs = [0.0, 0.2, 0.5, 0.9, 1.3, 1.5]
    area = [0.06, 0.04, 0.02, 0.03, 0.05, 0.2]           # exit > 1.15 inlet: the spill limit binds
    width = [0.8, 0.8, 0.7, 0.7, 0.75, 0.8]
    r = uf._solve_channel(xs, area, width, 22.0, 1.3, 1.18, 0.7)
    w = _chan(xs, area, width, 22.0, 1.3, 1.18, 0.7)
    _check(r, w)
    assert r.c_lift < 0 and r.downforce_N > 0 and r.attached is False
    assert r.notes == _note(100 * 0.5 / 1.5, 3.0, 0.7, w["diffuser_angle_deg"])
    gentle = [0.06, 0.04, 0.02, 0.021, 0.022, 0.03]       # the exit limits the flow
    r = uf._solve_channel(xs, gentle, width, 15.0, 1.0, 1.225, 0.75)
    w = _chan(xs, gentle, width, 15.0, 1.0, 1.225, 0.75)
    _check(r, w)
    assert r.attached is True and r.notes == _note(100 * 0.5 / 1.5, 3.0, 0.75, w["diffuser_angle_deg"])


def test_a_throat_at_the_exit_and_a_closed_gap():
    xs = [0.0, 0.5, 1.0, 1.5]
    area = [0.05, 0.04, 0.03, 0.02]
    r = uf._solve_channel(xs, area, [1.0] * 4, 20.0, 1.0, 1.2, 0.75)
    _check(r, _chan(xs, area, [1.0] * 4, 20.0, 1.0, 1.2, 0.75))
    assert r.diffuser_angle_deg == 0.0 and r.throat_x_frac == 1.0 and r.attached is True
    with pytest.raises(ValueError) as e:
        uf._solve_channel(xs, [0.05, 0.0, 0.03, 0.04], [1.0] * 4, 20.0, 1.0, 1.2, 0.75)
    assert str(e.value) == "gap closes completely"


def _duct(length_m=1.55, width_m=0.80, throat_frac=0.45, inlet_rise_mm=55.0, exit_rise_mm=115.0,
          ride_height_mm=40.0, edge_fence=0.55, nx=120):
    xs = np.linspace(0.0, length_m, nx)
    tf = min(max(throat_frac, 0.05), 0.95)
    rel = []
    for x in xs / length_m:
        if x < tf:
            rel.append(inlet_rise_mm / 1000 * (1 - x / tf) ** 1.6)
        else:
            rel.append(exit_rise_mm / 1000 * ((x - tf) / (1 - tf)) ** 1.35)
    gap = np.array(rel) + ride_height_mm / 1000
    return xs, gap * width_m * (1 - 0.84 * edge_fence), np.full(nx, float(width_m))


@pytest.mark.parametrize("kw", [{}, dict(length_m=1.2, width_m=0.6, throat_frac=0.3, inlet_rise_mm=80.0,
                                         exit_rise_mm=200.0, ride_height_mm=25.0, edge_fence=0.2, nx=37),
                                dict(throat_frac=0.01), dict(throat_frac=0.99)])
def test_the_parametric_duct(kw):
    xs, area, width = uf.channel_from_params(**kw)
    wx, wa, ww = _duct(**kw)
    assert xs == pytest.approx(wx, rel=1e-12) and area == pytest.approx(wa, rel=1e-12, abs=1e-15)
    assert width.tolist() == ww.tolist()


def test_the_parametric_solve():
    r = uf.solve_parametric()
    xs, a, w = _duct()
    _check(r, _chan(xs, a, w, 20.0, 1.24, 1.225, 0.75))
    r = uf.solve_parametric(speed_ms=30.0, ref_area_m2=1.1, rho=1.1, discharge=0.6, throat_frac=0.6,
                            exit_rise_mm=60.0, nx=50)
    xs, a, w = _duct(throat_frac=0.6, exit_rise_mm=60.0, nx=50)
    _check(r, _chan(xs, a, w, 30.0, 1.1, 1.1, 0.6))


# ------------------------------------------------------------------ the underside, binned
def _tri(cx, cy, cz, down=True, s=0.01):
    a, b, c = (cx - s, cy - s, cz), (cx + 2 * s, cy - s, cz), (cx - s, cy + 2 * s, cz)
    return (a, c, b) if down else (a, b, c)


def _mesh(tris):
    V = np.array([v for t in tris for v in t], float)
    F = np.arange(len(V)).reshape(-1, 3)
    return trimesh.Trimesh(V, F, process=False)


def test_the_underside_binning():
    tris = [_tri(0.1, 0.1, 0.05), _tri(0.1, 0.1, 0.03), _tri(0.5, 0.3, 0.04), _tri(0.9, 0.5, 0.06),
            _tri(0.5, 0.5, 0.0, down=False), _tri(-0.2, -0.2, 0.2), _tri(1.2, 0.8, 0.07)]
    m = _mesh(tris)
    xs, ys, z = uf._lower_surface(m, 6, 4)
    lo, hi = m.bounds
    assert xs == pytest.approx(np.linspace(lo[0], hi[0], 6)) and ys == pytest.approx(np.linspace(lo[1], hi[1], 4))
    want = np.full((6, 4), np.nan)
    cen = np.asarray(m.triangles_center)
    for c, n in zip(cen, np.asarray(m.face_normals)):
        if n[2] < -0.05:
            i = min(max(int(np.searchsorted(xs, c[0])) - 1, 0), 5)
            j = min(max(int(np.searchsorted(ys, c[1])) - 1, 0), 3)
            want[i, j] = c[2] if np.isnan(want[i, j]) else min(want[i, j], c[2])
    np.testing.assert_array_equal(z, want)
    assert np.isfinite(z).sum() == 5
    up = _mesh([_tri(0.1, 0.1, 0.0, down=False), _tri(0.5, 0.5, 0.0, down=False)])
    xs, ys, z = uf._lower_surface(up, 3, 2)
    assert z.shape == (3, 2) and np.isnan(z).all()


def _floor(path, nx=8, ny=5, z=lambda x: 0.0, L=1.0, W=0.5):
    tris = []
    for i in range(nx):
        for j in range(ny):
            x, y = (i + 0.5) * L / nx, (j + 0.5) * W / ny
            tris.append(_tri(x, y, z(x), s=0.2 * L / nx))
    tris.append(_tri(0.5 * L, 0.5 * W, 0.3, down=False))
    _mesh(tris).export(path)
    return path


def _expected(path, h_mm, V, S, rho=1.225, cd=0.75, nx=80, ny=40):
    m = trimesh.load(path, force="mesh")
    xs, ys, zl = uf._lower_surface(m, nx, ny)
    gap = zl - np.nanmin(zl) + h_mm / 1000
    dy = ys[1] - ys[0]
    cov = np.isfinite(gap)
    width = cov.sum(1) * dy
    area = np.where(cov, gap, 0.0).sum(1) * dy
    live = width > 1e-6
    return _chan(xs[live], area[live], width[live], V, S, rho, cd)


def test_the_mesh_solve(tmp_path):
    p = _floor(str(tmp_path / "f.stl"), nx=10, ny=4, z=lambda x: 0.04 * (x - 0.4) ** 2 + 0.01)
    r = uf.solve(p, 30.0, 25.0, 1.2, nx=12, ny=6)
    _check(r, _expected(p, 30.0, 25.0, 1.2, nx=12, ny=6))
    r = uf.solve(p, 20.0, 18.0, 1.0, rho=1.1, discharge=0.6, nx=20, ny=8)
    _check(r, _expected(p, 20.0, 18.0, 1.0, rho=1.1, cd=0.6, nx=20, ny=8))
    r = uf.solve(p, 40.0, 20.0, 1.0)
    _check(r, _expected(p, 40.0, 20.0, 1.0))


def test_a_scene_is_read_as_one_mesh(tmp_path):
    p = _floor(str(tmp_path / "f.stl"), nx=10, ny=4, z=lambda x: 0.05 * x)
    a = trimesh.load(p)
    b = a.copy()
    b.apply_translation([0.0, 0.0, 0.5])
    glb = str(tmp_path / "two.glb")
    trimesh.Scene([a, b]).export(glb)
    r = uf.solve(glb, 30.0, 20.0, 1.0, nx=12, ny=6)
    assert np.isfinite(r.c_lift)


def test_the_refusals(tmp_path):
    p = str(tmp_path / "e.stl")
    _mesh([_tri(0.1, 0.1, 0.0)]).export(p)
    empty = str(tmp_path / "empty.ply")
    trimesh.Trimesh(vertices=np.zeros((3, 3)), faces=np.zeros((0, 3), int)).export(empty)
    with pytest.raises(ValueError) as e:
        uf.solve(empty, 30.0, 20.0, 1.0)
    assert str(e.value) == "empty mesh"
    up = str(tmp_path / "up.stl")
    _mesh([_tri(0.1, 0.1, 0.0, down=False), _tri(0.6, 0.4, 0.0, down=False)]).export(up)
    with pytest.raises(ValueError) as e:
        uf.solve(up, 30.0, 20.0, 1.0)
    assert str(e.value) == "no underside found — is the STL closed?"
    narrow = _floor(str(tmp_path / "n.stl"), nx=3, ny=2)
    with pytest.raises(ValueError) as e:
        uf.solve(narrow, 30.0, 20.0, 1.0, nx=3, ny=3)
    assert str(e.value) == "underside too narrow to form a duct"
    four = _floor(str(tmp_path / "four.stl"), nx=12, ny=2)
    assert np.isfinite(uf.solve(four, 30.0, 20.0, 1.0, nx=5, ny=3).c_lift)      # the last bin never fills: 4 live
    with pytest.raises(ValueError):
        uf.solve(four, 30.0, 20.0, 1.0, nx=4, ny=3)


# ------------------------------------------------------------------ the sweep table
def _row(px, vx, r, py=None, vy=None):
    row = {px: vx, "C_L": round(r.c_lift, 4), "downforce_N": round(r.downforce_N, 1),
           "inlet/throat": round(r.area_ratio, 2), "diffuser_deg": round(r.diffuser_angle_deg, 1),
           "attached": r.attached}
    if py:
        row[py] = vy
    return row


def test_a_one_dimensional_sweep_against_its_median():
    vals = [20.0, 30.0, 40.0, 50.0, 60.0]
    rows = uf.sweep("ride_height_mm", vals, speed_ms=25.0, ref_area_m2=1.1, discharge=0.7, exit_rise_mm=150.0)
    rs = [uf.solve_parametric(speed_ms=25.0, ref_area_m2=1.1, discharge=0.7, exit_rise_mm=150.0,
                              ride_height_mm=v) for v in vals]
    base = sorted(round(r.downforce_N, 1) for r in rs)[2]
    for row, v, r in zip(rows, vals, rs):
        want = _row("ride_height_mm", v, r)
        want["vs_baseline_pct"] = round(100.0 * (want["downforce_N"] - base) / base, 1)
        assert row == want
    assert any(r["vs_baseline_pct"] == 0.0 for r in rows)


def test_a_two_dimensional_sweep_with_a_named_baseline_and_failures():
    rows = uf.sweep("ride_height_mm", [0.0, 30.0, 50.0], "exit_rise_mm", [80.0, 160.0], inlet_rise_mm=0.0,
                    baseline={"ride_height_mm": 50.0, "exit_rise_mm": 80.0})
    assert [(r["ride_height_mm"], r["exit_rise_mm"]) for r in rows] == [(30.0, 80.0), (50.0, 80.0), (30.0, 160.0),
                                                                        (50.0, 160.0)]
    b = next(r for r in rows if r["ride_height_mm"] == 50.0 and r["exit_rise_mm"] == 80.0)["downforce_N"]
    for r in rows:
        s = uf.solve_parametric(ride_height_mm=r["ride_height_mm"], exit_rise_mm=r["exit_rise_mm"], inlet_rise_mm=0.0)
        want = _row("ride_height_mm", r["ride_height_mm"], s, "exit_rise_mm", r["exit_rise_mm"])
        want["vs_baseline_pct"] = round(100.0 * (want["downforce_N"] - b) / b, 1)
        assert r == want
    miss = uf.sweep("ride_height_mm", [20.0, 30.0, 40.0, 50.0], baseline={"ride_height_mm": 99.0})
    dn = sorted(r["downforce_N"] for r in miss)
    med = 0.5 * (dn[1] + dn[2])
    base = min(miss, key=lambda r: abs(r["downforce_N"] - med))["downforce_N"]
    assert [r["vs_baseline_pct"] for r in miss] == [round(100 * (r["downforce_N"] - base) / base, 1) for r in miss]
    assert uf.sweep("ride_height_mm", [0.0], inlet_rise_mm=0.0) == []
    assert uf.sweep("ride_height_mm", [30.0, 40.0], speed_ms=0.0) == []           # q = 0: every point fails
    still = uf.sweep("ride_height_mm", [30.0, 40.0], speed_ms=0.01)
    assert [r["downforce_N"] for r in still] == [0.0, 0.0] and [r["vs_baseline_pct"] for r in still] == [-100.0,
                                                                                                         -100.0]


def test_the_median():
    assert uf._median([3.0, 1.0, 2.0]) == 2.0 and uf._median([4.0, 1.0, 3.0, 2.0]) == 2.5 and uf._median([7.0]) == 7.0


def test_offset_and_short_channels():
    from tests._golden import check
    res = {}
    xs = np.array([0.5, 0.9, 1.4, 2.1])
    res["offset"] = uf._solve_channel(xs, [0.05, 0.02, 0.03, 0.045], [1.0, 1.0, 1.1, 1.2], 20.0, 1.1, 1.2, 0.75)
    try:
        res["two"] = uf._solve_channel(np.array([0.3, 1.3]), [0.04, 0.05], [1.0, 1.0], 20.0, 1.0, 1.2, 0.75)
    except Exception as e:                                      # noqa: BLE001
        res["two"] = f"{type(e).__name__}: {e}"
    rows = uf.sweep("ride_height_mm", [20.0, 30.0, 40.0, 50.0], baseline={"ride_height_mm": 20.0})
    res["base20"] = rows
    check("underfloor_edges", res)
