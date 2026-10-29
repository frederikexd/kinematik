"""Characterization suite for suspension/chassis.py (mutation gate); see tests/_golden.py.

Meshes are trimesh primitives written to temporary files, so the loaders, the unit guess, the decimator, the swept
clearance and the envelope checks are pinned on geometry whose answers can be read off by hand."""
import json
import sys
import types

import numpy as np
import pytest
import trimesh

from suspension import chassis as ch
from suspension.kinematics import Hardpoints, SuspensionKinematics
from tests._golden import check, plain

HP = Hardpoints.default()


@pytest.fixture(autouse=True)
def _bounded_memory():
    """Cap the address space so a runaway sampling grid raises MemoryError instead of tripping the OOM killer."""
    import resource
    old = resource.getrlimit(resource.RLIMIT_AS)
    resource.setrlimit(resource.RLIMIT_AS, (2_500_000_000, old[1]))
    yield
    resource.setrlimit(resource.RLIMIT_AS, old)


def _norm(x, tmp_path):
    return json.loads(json.dumps(plain(x)).replace(str(tmp_path), "<T>"))


def _write(tmp_path, name, mesh):
    p = str(tmp_path / name)
    mesh.export(p)
    return p


def _fake_cascadio(src_mesh, log):
    def step_to_glb(path, glb_path, tol_linear, tol_angular):
        log.append((path.endswith(".step"), glb_path.endswith(".glb"), tol_linear, tol_angular))
        src_mesh.export(glb_path)
    return types.SimpleNamespace(step_to_glb=step_to_glb)


def test_lazy_proxy():
    lz = ch._LazyTrimesh()
    assert object.__getattribute__(lz, "_m") is None
    assert lz.Trimesh is trimesh.Trimesh and object.__getattribute__(lz, "_m") is trimesh
    assert lz.util is trimesh.util


def test_characterized_loaders(tmp_path, monkeypatch):
    res = {}
    box = trimesh.creation.box(extents=[100.0, 60.0, 40.0])
    stl = _write(tmp_path, "b.stl", box)
    m = ch.load_chassis(stl, offset=(10.0, -5.0, 2.0), scale=2.0)
    res["chassis"] = [ch.mesh_summary(m), ch.mesh_summary(ch.load_chassis(stl))]
    two = trimesh.Scene([trimesh.creation.box(extents=[10.0, 10.0, 10.0]),
                         trimesh.creation.box(extents=[5.0, 5.0, 5.0], transform=trimesh.transformations.translation_matrix([50, 0, 0]))])
    glb = _write(tmp_path, "two.glb", two)
    res["two"] = ch.mesh_summary(ch.load_chassis(glb))
    log = []
    monkeypatch.setitem(sys.modules, "cascadio", _fake_cascadio(box, log))
    step = str(tmp_path / "p.step")
    open(step, "w").write("ISO-10303-21;")
    res["step_chassis"] = ch.mesh_summary(ch.load_chassis(step, scale=0.5))
    sphere = trimesh.creation.icosphere(3, radius=0.2)
    monkeypatch.setitem(sys.modules, "cascadio", _fake_cascadio(sphere, log))
    part = ch.load_part_mesh(step, max_faces=500)
    res["step_part"] = [part["size_mm"], part["unit_scale"], part["triangles"] <= 500, len(part["verts"])]
    stp = str(tmp_path / "q.STP")
    open(stp, "w").write("ISO-10303-21;")
    res["stp_chassis"] = ch.mesh_summary(ch.load_chassis(stp))
    res["log"] = log
    monkeypatch.setitem(sys.modules, "cascadio", None)
    for fn in (ch.load_chassis, ch.load_part_mesh):
        with pytest.raises(ValueError) as e:
            fn(step)
        res.setdefault("no_cascadio", []).append(str(e.value))
    parts = {"mm": trimesh.creation.box(extents=[300.0, 80.0, 50.0]), "inch": trimesh.creation.box(extents=[12.0, 3.0, 2.0]),
             "m": trimesh.creation.box(extents=[0.3, 0.1, 0.05])}
    for k, mesh in parts.items():
        p = ch.load_part_mesh(_write(tmp_path, k + ".stl", mesh))
        res["part_" + k] = p
    p = ch.load_part_mesh(_write(tmp_path, "s.stl", parts["mm"]), unit_scale=0.1)
    res["part_explicit"] = [p["size_mm"], p["unit_scale"], p["verts"][:3]]
    p = ch.load_part_mesh(_write(tmp_path, "off.stl", trimesh.creation.box(
        extents=[200.0, 100.0, 60.0], transform=trimesh.transformations.translation_matrix([500, 20, -30]))))
    res["part_recentred"] = [np.min(p["verts"], axis=0), np.max(p["verts"], axis=0)]
    real_load = trimesh.load
    for k, ret in {"mixed_scene": trimesh.Scene([trimesh.creation.box(extents=[8.0, 4.0, 2.0]),
                                                 trimesh.PointCloud([[0.0, 0.0, 0.0], [90.0, 0.0, 0.0]])]),
                   "empty_scene": trimesh.Scene(), "bare_mesh": trimesh.creation.box(extents=[20.0, 10.0, 5.0]),
                   "empty_mesh": trimesh.Trimesh()}.items():
        monkeypatch.setattr(trimesh, "load", lambda *a, _r=ret, **kw: _r.copy() if hasattr(_r, "copy") else _r)
        for fn in (ch.load_chassis, ch.load_part_mesh):
            try:
                out = fn(stl)
                res[f"{k}_{fn.__name__}"] = ch.mesh_summary(out) if fn is ch.load_chassis else out["size_mm"]
            except (ValueError, TypeError) as e:
                res[f"{k}_{fn.__name__}"] = f"{type(e).__name__}: {e}"
    monkeypatch.setattr(trimesh, "load", real_load)
    res["guess"] = [ch._guess_unit_scale(s) for s in ([], [0.0], [-2.0], [4.9], [5.0], [39.9], [40.0], [300.0, 2.0])]
    check("chassis_loaders", _norm(res, tmp_path))


def test_characterized_decimation(monkeypatch):
    sph = trimesh.creation.icosphere(4, radius=100.0)
    res = {"quadric": None}
    try:
        res["quadric"] = len(ch._decimate_mesh(sph.copy(), 300).faces) <= 400
    except Exception:
        res["quadric"] = "unavailable"

    def boom(self, *a, **k):
        raise RuntimeError("no simplifier")
    monkeypatch.setattr(trimesh.Trimesh, "simplify_quadric_decimation", boom, raising=False)
    monkeypatch.setattr(trimesh.Trimesh, "simplify_quadratic_decimation", boom, raising=False)
    out = ch._decimate_mesh(sph.copy(), 300)
    res["cluster"] = [len(out.faces), out.bounds]
    out2 = ch._decimate_mesh(sph.copy(), 5000)
    res["cluster_loose"] = [len(out2.faces), out2.bounds]
    flat = trimesh.Trimesh(vertices=[[0, 0, 0], [1e-9, 0, 0], [0, 1e-9, 0]], faces=[[0, 1, 2]], process=False)
    res["degenerate"] = len(ch._decimate_mesh(flat, 1).faces)
    monkeypatch.setattr(trimesh.Trimesh, "simplify_quadric_decimation", lambda self, n: None, raising=False)
    monkeypatch.setattr(trimesh.Trimesh, "simplify_quadratic_decimation",
                        lambda self, n: trimesh.creation.box(extents=[1.0, 1.0, 1.0]), raising=False)
    res["old_name"] = len(ch._decimate_mesh(sph.copy(), 300).faces)
    check("chassis_decimate", res)


def _frame_box(centre, extents):
    return trimesh.creation.box(extents=extents, transform=trimesh.transformations.translation_matrix(centre))


def test_characterized_checks():
    kin = SuspensionKinematics(HP)
    res = {}
    res["segs"] = [[n, p, q] for n, p, q in ch._link_segments(kin.static, HP)]
    p, q = np.array([0.0, 0.0, 0.0]), np.array([100.0, 50.0, 20.0])
    res["sample"] = [ch._sample_segment(p, q, 5), ch._sample_segment(p, q, 6, skip_start_mm=30.0)]
    pts, names = ch.sweep_link_points(kin, -10.0, 15.0, 3, samples_per_link=4, mount_exclude_mm=40.0)
    res["sweep"] = [pts, names]
    res["sweep_one"] = ch.sweep_link_points(kin, 0.0, 0.0, 1, samples_per_link=2, mount_exclude_mm=100.0)
    res["sweep_none"] = ch.sweep_link_points(kin, 0.0, 0.0, 1, samples_per_link=3, mount_exclude_mm=5000.0)
    pd, nd = ch.sweep_link_points(kin)
    res["sweep_dflt"] = [pd.shape, nd[:3], nd[-3:], pd[::97]]
    ufi = np.asarray(HP.upper_front_inner, float)
    frame = _frame_box(ufi + [0.0, -20.0, 0.0], [600.0, 30.0, 400.0])
    res["fit"] = [ch.fit_check(HP, frame), ch.fit_check(HP, frame, tol_mm=200.0)]
    lo = np.asarray(kin.static.lower_outer, float)
    for k, centre in {"hit": lo, "tight": lo + [0.0, 0.0, -90.0], "clear": lo + [0.0, 900.0, 0.0]}.items():
        res["clear_" + k] = ch.clearance_check(kin, _frame_box(centre, [80.0, 80.0, 80.0]), -10.0, 10.0, 3,
                                               warn_mm=60.0)
    res["clear_dflt"] = ch.clearance_check(kin, _frame_box(lo + [0.0, 0.0, -36.0], [60.0, 60.0, 60.0]))
    res["corners"] = ch._box_corners((1.0, 2.0, 3.0), (10.0, 20.0, 30.0))
    res["surf"] = [ch._box_surface_points((1.0, 2.0, 3.0), (45.0, 10.0, 6.0), 20.0),
                   ch._box_surface_points((0.0, 0.0, 0.0), (5.0, 5.0, 5.0), 0.5),
                   ch.envelope_box_points((0.0, 0.0, 0.0), (30.0, 10.0, 10.0))]
    cage = trimesh.creation.box(extents=[1000.0, 600.0, 400.0])
    tube = trimesh.util.concatenate([_frame_box([0.0, 0.0, -150.0], [1000.0, 20.0, 20.0]),
                                     _frame_box([590.0, 390.0, 290.0], [20.0, 20.0, 20.0]),
                                     _frame_box([-590.0, -390.0, -290.0], [20.0, 20.0, 20.0])])
    for k, (mesh, o, s, kw) in {
            "outside": (cage, (-600.0, 0.0, 0.0), (100.0, 50.0, 50.0), {}),
            "inside_solid": (cage, (-50.0, -50.0, -50.0), (100.0, 100.0, 100.0), {}),
            "tight": (tube, (-100.0, -50.0, -135.0), (200.0, 100.0, 50.0), {"warn_mm": 10.0}),
            "clear": (tube, (-100.0, -50.0, -100.0), (200.0, 100.0, 50.0), {"warn_mm": 10.0}),
            "neg_size": (tube, (100.0, 50.0, -50.0), (-200.0, -100.0, 50.0), {"name": "motor", "sample_step_mm": 50.0}),
            "dflt": (tube, (-100.0, -50.0, -137.0), (200.0, 100.0, 50.0), {})}.items():
        res["env_" + k] = ch.envelope_fit_check(mesh, o, s, **kw)
    res["env_step"] = ch.envelope_fit_check(tube, (-100.0, -50.0, -100.0), (200.0, 100.0, 50.0), sample_step_mm=35.0)
    res["box_step"] = ch.envelope_box_points((0.0, 0.0, 0.0), (30.0, 10.0, 10.0), step_mm=7.0)
    res["env_coarse"] = ch.envelope_fit_check(tube, (-37.0, -53.0, -128.0), (200.0, 100.0, 50.0), sample_step_mm=95.0)

    def no_contains(self, pts):
        raise RuntimeError("no ray backend")
    real_contains = trimesh.Trimesh.contains
    trimesh.Trimesh.contains = no_contains
    try:
        res["env_no_rays"] = ch.envelope_fit_check(cage, (-50.0, -50.0, -50.0), (100.0, 100.0, 100.0))
    finally:
        trimesh.Trimesh.contains = real_contains
    open_tube = tube.copy()
    open_tube.faces = open_tube.faces[:-2]
    res["env_open"] = ch.envelope_fit_check(open_tube, (-100.0, -50.0, -135.0), (200.0, 100.0, 50.0))
    res["sheet"] = ch.manufacturing_sheet(HP, kin)
    check("chassis_checks", json.loads(json.dumps(plain(res))))
