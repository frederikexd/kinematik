"""Characterization suite for suspension/aero/panel_method.py (mutation gate); see tests/_golden.py.

Small meshes (an 80-face icosphere, a 48-face subdivided slab) keep each dense solve to milliseconds; the circulation
lattice is replaced by a stand-in except for one real call."""
import json
import sys
import types

import numpy as np
import pytest
import trimesh

from suspension.aero import cfd
from suspension.aero import panel_method as pm
from tests._golden import check, plain

PM = pm.PanelMethodModel


def _norm(x, tmp_path):
    return json.loads(json.dumps(plain(x)).replace(str(tmp_path), "<T>"))


@pytest.fixture()
def geo(tmp_path):
    out = {}
    out["sphere"] = str(tmp_path / "s.stl")
    trimesh.creation.icosphere(1, radius=0.3).export(out["sphere"])
    out["slab"] = str(tmp_path / "b.stl")
    trimesh.creation.box(extents=[1.0, 0.5, 0.1]).subdivide().export(out["slab"])
    out["tiny"] = str(tmp_path / "t.stl")
    trimesh.creation.box(extents=[1.0, 0.5, 0.1]).export(out["tiny"])
    out["thin"] = str(tmp_path / "th.stl")
    trimesh.creation.box(extents=[0.4, 0.3, 0.01]).subdivide().subdivide().export(out["thin"])
    out["junk"] = str(tmp_path / "j.stl")
    open(out["junk"], "w").write("not a mesh at all")
    return out


def _spec(path, **att):
    kw = {k: att.pop(k) for k in list(att) if k in ("reference_area_m2", "reference_length_m")}
    return cfd.CaseSpec(cfd.Attitude(**att), path, **kw)


class _FakeVLM:
    mode = "ok"

    def solve(self, spec):
        if self.mode == "raise":
            raise RuntimeError("no camber surface")
        return types.SimpleNamespace(c_lift=-0.25, c_drag=0.012, converged=self.mode == "ok")


def test_solves(geo, tmp_path, monkeypatch):
    res = {}
    P = pm.PanelParams
    cases = {
        "sphere_free": (P(ground_effect=False, lifting=False), _spec(geo["sphere"])),
        "sphere_ground": (P(lifting=False), _spec(geo["sphere"], ride_height_mm=40.0, speed_ms=30.0)),
        "slab": (P(lifting=False), _spec(geo["slab"], pitch_deg=2.0, yaw_deg=5.0, roll_deg=1.0,
                                         reference_area_m2=0.5, reference_length_m=None)),
        "slab_road": (P(lifting=False, road_plane_z_m=0.05, kin_viscosity=1e-5, laminar_fraction=0.3),
                      _spec(geo["slab"], ride_height_mm=10.0)),
        "slab_lowref": (P(lifting=False, ground_effect=False), _spec(geo["slab"], reference_area_m2=0.0)),
    }
    for k, (p, s) in cases.items():
        res[k] = PM(p).solve(s)
    monkeypatch.setattr("suspension.aero.vortex_lattice.VortexLatticeModel", _FakeVLM)
    for mode in ("ok", "bad", "raise"):
        monkeypatch.setattr(_FakeVLM, "mode", mode)
        res["lift_" + mode] = PM().solve(_spec(geo["slab"]))
    res["run_read"] = [PM(P(lifting=False)).run_case(_spec(geo["sphere"]), "x"),
                       PM(P(lifting=False)).read_result(_spec(geo["sphere"]), "x")]
    res["prov"] = [PM().provenance(), PM().provenance(0), PM().provenance(77)]
    res["name"] = [PM.name, pm._MEMORY_BUDGET_MB, PM._NEAR_R, P()]
    check("panel_solves", _norm(res, tmp_path))


def test_real_lattice_call(geo, tmp_path):
    check("panel_lattice", _norm({"r": PM().solve(_spec(geo["slab"]))}, tmp_path))


def test_solver_fallbacks(geo, tmp_path, monkeypatch):
    res = {}
    P = pm.PanelParams(lifting=False)
    s = _spec(geo["sphere"])
    with monkeypatch.context() as m:
        m.setattr(pm, "_MEMORY_BUDGET_MB", 0.01)
        with pytest.raises(pm.PanelMethodUnavailable) as e:
            PM(P).solve(s)
        res["memory"] = str(e.value)
    with monkeypatch.context() as m:
        m.setitem(sys.modules, "scipy.linalg", None)
        res["no_scipy"] = PM(P).solve(s)
    import scipy.linalg as sl

    def lu_fail(*a, **k):
        raise np.linalg.LinAlgError("singular")

    with monkeypatch.context() as m:
        m.setattr(sl, "lu_factor", lu_fail)
        res["lstsq"] = PM(P).solve(s)
        m.setattr(np.linalg, "lstsq", lu_fail)
        with pytest.raises(pm.PanelMethodUnavailable) as e:
            PM(P).solve(s)
        res["lstsq_fail"] = str(e.value)
    with monkeypatch.context() as m:
        m.setattr(sl, "lu_solve", lambda lu, b, **k: np.full(b.shape, 80.0))
        res["ill_sigma"] = PM(P).solve(s)
    with monkeypatch.context() as m:
        real = sl.lu_factor
        m.setattr(sl, "lu_factor", lambda A, **k: (lambda lu: (lu[0] * np.where(np.eye(len(A)) > 0, 1e-6, 1.0)
                                                              if False else lu[0], lu[1]))(real(A, **k)))
        res["ok_again"] = PM(P).solve(s).converged
    check("panel_fallbacks", _norm(res, tmp_path))


def test_load_panels(geo, tmp_path, monkeypatch):
    res = {}
    P = pm.PanelParams
    bad = {"missing": _spec(str(tmp_path / "none.stl")), "empty_path": _spec(""), "junk": _spec(geo["junk"]),
           "thin": _spec(geo["thin"]), "tiny": _spec(geo["tiny"])}
    for k, s in bad.items():
        with pytest.raises(pm.PanelMethodUnavailable) as e:
            PM(P(lifting=False)).solve(s)
        res[k] = str(e.value).replace(str(tmp_path), "<T>")
    with pytest.raises(pm.PanelMethodUnavailable) as e:
        PM(P(max_panels=40)).solve(_spec(geo["slab"]))
    res["cap"] = str(e.value)
    res["cap_none"] = PM(P(max_panels=None, lifting=False)).solve(_spec(geo["slab"])).c_lift
    res["min_panels"] = PM(P(min_panels=12, lifting=False)).solve(_spec(geo["tiny"])).c_lift
    m = PM(P(lifting=False))
    for k, att in {"flat": {}, "pitch": dict(pitch_deg=3.0), "pitch_tiny": dict(pitch_deg=1e-10),
                   "roll": dict(roll_deg=4.0, ride_height_mm=55.0)}.items():
        c, n, a, L, tris, gap = m._load_panels(_spec(geo["slab"], **att))
        res["lp_" + k] = [c[::7], n[::7], a[::7], L, tris[::11], gap]
    res["lp_ref"] = m._load_panels(_spec(geo["slab"], reference_length_m=0.7))[3]
    with monkeypatch.context() as mp:
        mp.setitem(sys.modules, "trimesh", None)
        with pytest.raises(pm.PanelMethodUnavailable) as e:
            m.solve(_spec(geo["slab"]))
        res["no_trimesh"] = str(e.value)
    with monkeypatch.context() as mp:
        mp.setattr(trimesh, "load", lambda *a, **k: types.SimpleNamespace(faces=[]))
        with pytest.raises(pm.PanelMethodUnavailable) as e:
            m.solve(_spec(geo["slab"]))
        res["no_faces"] = str(e.value).replace(str(tmp_path), "<T>")
        mp.setattr(trimesh, "load", lambda *a, **k: None)
        with pytest.raises(pm.PanelMethodUnavailable):
            m.solve(_spec(geo["slab"]))

        def broken(*a, **k):
            raise ValueError("bad facet")

        mp.setattr(trimesh, "load", broken)
        with pytest.raises(pm.PanelMethodUnavailable) as e:
            m.solve(_spec(geo["slab"]))
        res["load_err"] = str(e.value).replace(str(tmp_path), "<T>")
    real_load = trimesh.load

    def no_bbox(*a, **k):
        mesh = real_load(*a, **k)
        return types.SimpleNamespace(__class__=type(mesh), **{}) if False else _NoBox(mesh)

    with monkeypatch.context() as mp:
        mp.setattr(trimesh, "load", no_bbox)
        res["no_bbox"] = m.solve(_spec(geo["thin"])).c_lift
    check("panel_load", _norm(res, tmp_path))


class _NoBox:
    """Proxy whose bounding box cannot be read (the thin-plate guard must then stand aside)."""

    def __init__(self, mesh):
        object.__setattr__(self, "_m", mesh)

    @property
    def bounding_box(self):
        raise RuntimeError("no box")

    def __getattr__(self, k):
        return getattr(self._m, k)


def test_kernels_and_helpers(monkeypatch):
    res = {}
    rng = np.random.default_rng(3)
    c = rng.normal(size=(9, 3)) * 0.2
    c[:, 2] += 0.5
    n = rng.normal(size=(9, 3))
    n /= np.linalg.norm(n, axis=1)[:, None]
    a = rng.uniform(0.001, 0.01, 9)
    tris = c[:, None, :] + rng.normal(size=(9, 3, 3)) * 0.03
    sig = rng.normal(size=9)
    res["w"] = [PM._sub_centroid_weights(0), PM._sub_centroid_weights(1), PM._SUB_W.shape]
    res["near"] = [PM._near_pairs(c, a, 20.0), PM._near_pairs(c, a, 0.0)]
    with monkeypatch.context() as m:
        m.setitem(sys.modules, "scipy.spatial", None)
        res["near_np"] = [PM._near_pairs(c, a, 20.0, block=4), PM._near_pairs(c, a, 3.0)]
    res["qn"] = [PM._quad_normal_vel(c, n, tris, a), PM._quad_normal_vel(c, n, tris, a, chunk=4)]
    res["qv"] = [PM._quad_velocity(c, tris, sig * a), PM._quad_velocity(c, tris, sig * a, chunk=2)]
    res["pv"] = PM._point_velocity(c, c[::-1], sig * a)
    res["nk"] = PM._normal_vel_kernel(c[:3, None, :] - c[None, :, :], n[:3], a)
    for ge in (True, False):
        for z in (0.0, 0.1):
            mdl = PM(pm.PanelParams(ground_effect=ge, road_plane_z_m=z))
            res[f"A_{ge}_{z}"] = [mdl._influence_matrix(c, n, a), mdl._influence_matrix(c, n, a, tris)]
            res[f"v_{ge}_{z}"] = [PM._induced_velocity(c, n, a, sig, ge, z),
                                  PM._induced_velocity(c, n, a, sig, ge, z, tris)]
    res["A1"] = PM()._influence_matrix(c[:1], n[:1], a[:1], tris[:1])
    res["v1"] = PM._induced_velocity(c[:1], n[:1], a[:1], sig[:1], True, 0.0, tris[:1])
    sp = lambda v: types.SimpleNamespace(attitude=cfd.Attitude(speed_ms=v))
    res["fric"] = [PM()._friction_cd(sp(v), a, 0.5, L) for v in (20.0, 0.0, 1e-9) for L in (1.0, 1e-6)] + [
        PM(pm.PanelParams(kin_viscosity=0.0, laminar_fraction=0.5))._friction_cd(sp(10.0), a, 0.0, 2.0)]
    m0 = PM(pm.PanelParams(ground_effect=False))
    m1 = PM()
    res["resw"] = [m0._resolution_warning(c, a, 0.1), m1._resolution_warning(c, a, 0.0),
                   m1._resolution_warning(c, a, -0.1), m1._resolution_warning(c, a, 0.05),
                   m1._resolution_warning(c, a, 0.15), m1._resolution_warning(c, a, 0.2),
                   m1._resolution_warning(c, a, 1.0),
                   m1._resolution_warning(c, a), PM(pm.PanelParams(road_plane_z_m=0.9))._resolution_warning(c, a)]
    xs = np.array([[1.0, 0, 0], [0.5, 0, 0], [-0.5, 0, 0], [-1.0, 0, 0]])
    up = np.tile([0.0, 0.0, 1.0], (4, 1))
    A4 = np.ones(4)
    res["bal"] = [PM._aero_balance(xs, np.array(cp), A4, up) for cp in
                  ([1.0, 1.0, 1.0, 1.0], [0.0, 0.0, 0.0, 0.0], [1.0, 1.0, -1.0, -1.0], [2.0, 1.0, 0.5, 0.5],
                   [1.0, 1.0, -0.9, -0.9], [1.0, 1.0, -0.5, -0.5], [-1.0, 0.2, 1.0, 1.0], [5.0, 0.0, 0.0, 0.2])]
    res["free"] = [pm._freestream_unit(cfd.Attitude(yaw_deg=y)) for y in (0.0, 10.0, -90.0)]
    check("panel_kernels", res)


def _stl(tmp_path, name, mesh):
    p = str(tmp_path / name)
    mesh.export(p)
    return p


def test_characterized_blocks_and_guards(tmp_path, monkeypatch):
    P = pm.PanelParams
    res = {}
    big = trimesh.creation.icosphere(2, radius=0.3)
    big.apply_translation([0.35, 0.0, 0.0])
    path = _stl(tmp_path, "big.stl", big)
    for ge in (True, False):
        r = PM(P(ground_effect=ge, lifting=False)).solve(_spec(path, pitch_deg=2.5, roll_deg=1.5, ride_height_mm=60.0))
        res[f"big_{ge}"] = [r.c_lift, r.c_drag, r.c_side, r.aero_balance_front, r.converged, r.notes]
    res["sub_w"] = [PM._sub_centroid_weights(1), PM._sub_centroid_weights(2)]
    rng = np.random.default_rng(3)
    tris = rng.normal(size=(5, 3, 3))
    fp, fn = rng.normal(size=(5, 3)) + 3.0, rng.normal(size=(5, 3))
    res["quad"] = [PM._quad_normal_vel(fp, fn, tris, np.full(5, 0.2), chunk=2),
                   PM._quad_velocity(fp, tris, np.full(5, 0.3), chunk=2)]
    c = rng.normal(size=(9, 3))
    a = np.full(9, 0.5)
    res["near"] = [list(map(np.ndarray.tolist, PM._near_pairs(c, a, 2.5)))]
    monkeypatch.setitem(sys.modules, "scipy.spatial", None)
    res["near"].append(list(map(np.ndarray.tolist, PM._near_pairs(c, a, 2.5, block=2))))
    monkeypatch.undo()
    cen = np.array([[-1.0, 0, 0], [-0.2, 0, 0], [0.4, 0, 0], [1.0, 0, 0]])
    nz = np.array([[0, 0, 1.0]] * 4)
    res["balance"] = [PM._aero_balance(cen + 3.0, np.array(cp), np.ones(4), nz)
                      for cp in ([0.5, 0.2, 0.3, 0.1], [0.5, -0.2, 0.3, -0.9], [1e-12, 0, 0, 0])]
    res["vinf"] = [pm._freestream_unit(cfd.Attitude(yaw_deg=y)) for y in (0.0, 7.5, -30.0)]
    msgs = {}
    for k, ext in {"thin_long": [0.16, 0.08, 0.02498], "just_thick": [0.16, 0.08, 0.0255],
                   "seven": [0.13, 0.06, 0.02], "short": [0.10, 0.06, 0.02], "edge": [0.11995, 0.06, 0.02]}.items():
        p = _stl(tmp_path, f"{k}.stl", trimesh.creation.box(extents=ext))
        try:
            PM(P(lifting=False, min_panels=4))._load_panels(_spec(p))
            msgs[k] = "loaded"
        except pm.PanelMethodUnavailable as e:
            msgs[k] = str(e)[:60]
    box = _stl(tmp_path, "box12.stl", trimesh.creation.box(extents=[0.5, 0.4, 0.3]))
    for cap in (12, 11):
        try:
            PM(P(lifting=False, max_panels=cap, min_panels=4))._load_panels(_spec(box))
            msgs[f"cap{cap}"] = "loaded"
        except pm.PanelMethodUnavailable as e:
            msgs[f"cap{cap}"] = str(e)[:60]
    res["msgs"] = msgs
    n = 320
    need = n * n * 4 / 1048576.0 * 3.3
    for k, budget in (("over", need * 0.99), ("under", need * 1.1)):
        monkeypatch.setattr(pm, "_MEMORY_BUDGET_MB", budget)
        try:
            PM(P(lifting=False)).solve(_spec(path))
            res[f"mem_{k}"] = "solved"
        except pm.PanelMethodUnavailable as e:
            res[f"mem_{k}"] = str(e)
    check("panel_blocks", _norm(res, tmp_path))


def test_characterized_defaults_and_messages(geo, tmp_path, monkeypatch):
    res = {}

    class LongFail:
        def solve(self, spec):
            raise RuntimeError("x" * 79 + "ABCDEFGH")
    monkeypatch.setattr("suspension.aero.vortex_lattice.VortexLatticeModel", LongFail)
    res["circ"] = PM(pm.PanelParams(lifting=True))._circulation_terms(_spec(geo["sphere"]))
    rng = np.random.default_rng(7)
    c = rng.normal(size=(6, 3)) * 0.2 + [0, 0, 0.3]
    n = rng.normal(size=(6, 3))
    n /= np.linalg.norm(n, axis=1)[:, None]
    a = np.full(6, 0.01)
    sig = rng.normal(size=6)
    res["v_dflt"] = PM._induced_velocity(c, n, a, sig)
    tris2 = np.stack([c[:2] + d for d in (np.zeros(3), [0.05, 0, 0], [0, 0.05, 0])], axis=1)
    c2 = np.array([[0.0, 0.0, 0.3], [0.04, 0.0, 0.3]])
    res["two"] = [PM._induced_velocity(c2, n[:2], np.full(2, 0.004), sig[:2], True, 0.0, tris2),
                  PM(pm.PanelParams(ground_effect=True))._influence_matrix(c2, np.array([[0, 0, 1.0]] * 2),
                                                                          np.full(2, 0.004), tris2)]
    m1 = PM(pm.PanelParams(ground_effect=True))
    res["note"] = m1._resolution_warning(c, np.full(6, 0.36), 1.0)
    check("panel_defaults", _norm(res, tmp_path))
