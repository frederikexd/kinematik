import os
"""Characterization suite for suspension/integration.py (mutation gate); see tests/_golden.py. Parts loaded from
STL, OBJ and GLB (scaled, rotated about each axis, offset), every refusal (type, unreadable, empty, metres), the
summary of open and closed meshes, clearance checks that reject on bounding boxes, collide, sit tight and clear
(surface sampling seeded so the verdict is repeatable), and part records with and without a density."""
import numpy as np
import pytest
import trimesh

from suspension import integration as I
from tests._golden import plain
from tests._golden_lazy import check_lazy


@pytest.fixture
def seeded(monkeypatch):
    real = trimesh.sample.sample_surface
    monkeypatch.setattr(trimesh.sample, "sample_surface", lambda mesh, n, **kw: real(mesh, n, seed=7))


def _box(ext, at=(0, 0, 0)):
    b = trimesh.creation.box(extents=ext)
    b.apply_translation(at)
    return b


def _load(path, **kw):
    try:
        m = I.load_part(str(path), **kw)
        return [plain(I.part_summary(m)), plain(np.round(m.vertices[:4], 6))]
    except ValueError as e:
        msg = str(e)                         # the library's own detail differs between trimesh versions
        return {"raised": msg.split(" (details: ")[0], "detailed": " (details: " in msg}


def _cases(tmp):
    stl = tmp / "box.stl"
    _box((100, 50, 20)).export(stl)
    obj = tmp / "box.obj"
    _box((80, 40, 30), (10, 0, 0)).export(obj)
    glb = tmp / "box.glb"
    trimesh.Scene([_box((60, 60, 60)), _box((10, 10, 10), (100, 0, 0))]).export(glb)
    small = tmp / "small.stl"
    _box((0.1, 0.05, 0.02)).export(small)
    for name, p, kw in (("stl", stl, {}), ("obj", obj, {"offset": (5.0, -3.0, 2.0)}), ("glb", glb, {}),
                        ("scaled", small, {"scale": 1000.0}), ("rx", stl, {"rotate_deg": (90.0, 0.0, 0.0)}),
                        ("ry", stl, {"rotate_deg": (0.0, 45.0, 0.0)}), ("rz", stl, {"rotate_deg": (0.0, 0.0, 30.0)}),
                        ("all", stl, {"scale": 2.0, "rotate_deg": (10.0, 20.0, 30.0), "offset": (1.0, 2.0, 3.0)}),
                        ("metres", small, {}), ("half", stl, {"scale": 0.04})):
        yield f"load.{name}", _load(p, **kw)
    bad = tmp / "bad.stl"
    bad.write_bytes(b"not an stl at all")
    txt = tmp / "part.sldprt"
    txt.write_text("x")
    empty = tmp / "empty.obj"
    empty.write_text("# nothing\n")
    step = tmp / "bad.step"
    step.write_text("ISO-10303-21; garbage")
    for name, p in (("type", txt), ("bad", bad), ("empty", empty), ("upper", tmp / "BOX.STL"), ("step", step)):
        yield f"load.err.{name}", _load(p)
    (tmp / "UP.STL").write_bytes(stl.read_bytes())
    yield "load.upper_ext", _load(tmp / "UP.STL")
    ref = _box((400, 300, 200))
    for name, part, kw in (("far", _box((50, 50, 50), (1000, 0, 0)), {}), ("near_bbox", _box((50, 50, 50), (228, 0, 0)), {}),
                           ("inside", _box((50, 50, 50)), {}), ("overlap", _box((50, 50, 50), (210, 0, 0)), {}),
                           ("tight", _box((50, 50, 50), (227.0, 0, 0)), {"warn_mm": 5.0}),
                           ("clear_close", _box((50, 50, 50), (230.0, 0, 0)), {"warn_mm": 10.0}),
                           ("sample", _box((50, 50, 50), (210, 0, 0)), {"sample_target": 900}),
                           ("exact_bbox", _box((50, 50, 50), (230.0, 0, 0)), {"warn_mm": 5.0})):
        yield f"check.{name}", plain(I.interference_check(part, ref, **kw))
    fine = trimesh.creation.icosphere(subdivisions=4, radius=30.0)
    fine.apply_translation((205, 0, 0))
    yield "check.many_faces", plain(I.interference_check(fine, ref, sample_target=1000))
    opn = _box((10, 10, 10))
    opn.update_faces(np.arange(len(opn.faces) - 2))
    yield "summary.open", plain(I.part_summary(opn))
    for name, m, d in (("dense", _box((100, 100, 100)), 7850.0), ("none", _box((100, 100, 100)), None),
                       ("open", opn, 2700.0), ("offset", _box((10, 20, 30), (5, 5, 5)), 1000.0)):
        yield f"record.{name}", plain(I.part_record("brakes", name, m, d))
    yield "teams", I.TEAMS
    fresh = I._LazyTrimesh()
    yield "lazy", [object.__getattribute__(fresh, "_m") is None, fresh.creation.__name__,
                   object.__getattribute__(fresh, "_m").__name__]
    import cascadio
    calls = []

    def fake(src, dst, tol_linear, tol_angular):
        calls.append([os.path.basename(src), dst.endswith(".glb"), os.path.exists(dst), tol_linear, tol_angular])
        with open(dst, "wb") as fh:
            fh.write(glb.read_bytes())
    real = cascadio.step_to_glb
    cascadio.step_to_glb = fake
    try:
        for ext in ("step", "stp", "STEP"):
            sp = tmp / f"part.{ext}"
            sp.write_text("ISO-10303-21;")
            yield f"load.fake_step.{ext}", [_load(sp, offset=(1.0, 0.0, 0.0)), calls[-1:]]
    finally:
        cascadio.step_to_glb = real
    real_load = trimesh.load
    for name, geo in (("points", trimesh.Trimesh(vertices=[[0, 0, 0], [1, 0, 0], [0, 1, 0]], faces=[], process=False)),
                      ("one_face", trimesh.Trimesh(vertices=[[0, 0, 0], [100, 0, 0], [0, 100, 0]], faces=[[0, 1, 2]],
                                                   process=False)),
                      ("one_vertex", trimesh.Trimesh(vertices=[[0, 0, 0]], faces=[], process=False))):
        trimesh.load = (lambda g: lambda *a, **k: trimesh.Scene([g]))(geo)
        try:
            yield f"load.geo.{name}", _load(stl)
        finally:
            trimesh.load = real_load
    for side in (5.0, 5.5):
        pth = tmp / f"cube{side}.stl"
        _box((side, side, side)).export(pth)
        yield f"load.size.{side}", _load(pth)
    yield "check.defaults", [plain(I.interference_check(_box((50, 50, 50), (x, 0, 0)), ref)) for x in (229.5, 230.5,
                                                                                                   225.5)]
    cone = trimesh.creation.cone(radius=40.0, height=90.0)
    yield "record.cone", plain(I.part_record("brakes", "cone", cone, 7850.0))


def test_every_part_load_and_check(tmp_path, seeded):
    check_lazy("integration", _cases(tmp_path), rel=1e-6, abs_=1e-6)
