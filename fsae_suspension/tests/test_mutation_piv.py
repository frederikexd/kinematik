"""Characterization suite for suspension/aero/piv.py (mutation gate); see tests/_golden.py."""
import json
import os

import numpy as np
import pytest

from suspension.aero import piv
from suspension.aero.cfd import Attitude
from suspension.aero.windtunnel import GroundState
from tests._golden import check, plain

O = piv.SheetOrientation


def _prov(**kw):
    base = dict(facility="tunnel", plane=piv.LaserSheetPlane(), dt_us=20.0, magnification_m_per_px=1e-4)
    base.update(kw)
    return piv.PIVProvenance(**base)


def test_planes_and_provenance():
    res = {}
    planes = [piv.LaserSheetPlane(), piv.LaserSheetPlane(O.XY_PLAN, 0.05, 2.0), piv.LaserSheetPlane(O.YZ_CROSS, -0.3)]
    res["planes"] = [[p.in_plane_axes(), p.out_of_plane_axis(), p.label()] for p in planes]
    provs = {"base": _prov(), "heavy": _prov(particle_diameter_um=12.0), "fast": _prov(dt_us=200.0),
             "slow": _prov(dt_us=1.0), "fixed": _prov(ground_state=GroundState.FIXED_FLOOR, window_px=16,
                                                       camera_fps=5000.0, laser="LED", notes="n"),
             "zero": _prov(freestream_ms=0.0, magnification_m_per_px=0.0, dt_us=0.0),
             "edge_lo": _prov(dt_us=20.0, freestream_ms=20.0, magnification_m_per_px=1e-3),
             "edge_hi": _prov(dt_us=80.0, freestream_ms=20.0, magnification_m_per_px=1e-4)}
    for k, p in provs.items():
        res[k] = [p.stokes_number(), p.stokes_number(1000.0, 2e-5, 0.01), p.stokes_number(flow_length_m=0.0),
                  p.particle_shift_px(), p.timing_ok(), p.seeding_ok(), p.seeding_ok(0.01), p.status()]
    st = _prov(particle_diameter_um=3.0)
    res["st_edge"] = [st.stokes_number(), st.seeding_ok(st.stokes_number()), st.seeding_ok(st.stokes_number() * 0.999)]
    check("piv_prov", res)


def _speckle(shape=(48, 64), seed=1, n=300):
    rng = np.random.default_rng(seed)
    img = np.zeros(shape)
    yy, xx = np.mgrid[:shape[0], :shape[1]]
    for y, x in zip(rng.uniform(0, shape[0], n), rng.uniform(0, shape[1], n)):
        img += np.exp(-((yy - y) ** 2 + (xx - x) ** 2) / 1.5)
    return img


def test_correlation_core():
    res = {}
    a = _speckle((32, 32))
    for k, (dy, dx) in {"zero": (0, 0), "right": (0, 3), "up": (-2, 1), "big": (5, -6)}.items():
        b = np.roll(np.roll(a, dy, axis=0), dx, axis=1)
        res[k] = [piv._window_displacement(a, b, 1.2), piv._window_displacement(a, b, 50.0)]
    res["flat"] = piv._window_displacement(np.ones((8, 8)), a[:8, :8], 1.2)
    res["flat_b"] = piv._window_displacement(a[:8, :8], np.ones((8, 8)), 1.2)
    res["noise"] = piv._window_displacement(np.random.default_rng(2).normal(size=(16, 16)),
                                            np.random.default_rng(3).normal(size=(16, 16)), 1.05)
    c = np.array([[1.0, 2.0, 1.0], [2.0, 6.0, 3.0], [1.0, 2.5, 1.0]])
    res["sub"] = [piv._gaussian_subpixel(c, 1, 1), piv._gaussian_subpixel(c, 0, 0), piv._gaussian_subpixel(c, 2, 2),
                  piv._gaussian_subpixel(c, 1, 0), piv._gaussian_subpixel(c, 0, 1),
                  piv._gaussian_subpixel(np.array([[1.0, 1.0, 1.0], [1.0, 1.0, 1.0], [1.0, 1.0, 1.0]]), 1, 1),
                  piv._gaussian_subpixel(np.array([[1.0, 0.0, 1.0], [0.5, 2.0, 1.0], [1.0, 1.0, 1.0]]), 1, 1),
                  piv._gaussian_subpixel(np.array([[1.0, 1e-9, 1.0], [1e-9, 2.0, 50.0], [1.0, 9.0, 1.0]]), 1, 1),
                  piv._gaussian_subpixel(np.array([[1.0, 2.0, 1.0], [-1.0, 3.0, 2.0], [1.0, 0.0, 1.0]]), 1, 1)]
    check("piv_core", res)


def test_processor_and_fields():
    res = {}
    a = _speckle()
    b = np.roll(np.roll(a, -1, axis=0), 2, axis=1)
    pair = piv.FramePair(a, b, 20.0)
    res["pair"] = pair.shape
    for args in ((np.zeros((4, 4)), np.zeros((4, 5)), 1.0), (np.zeros(4), np.zeros(4), 1.0)):
        with pytest.raises(ValueError) as e:
            piv.FramePair(*args)
        res.setdefault("pair_err", []).append(str(e.value))
    prov = _prov(window_px=16)
    procs = {"d": piv.PIVProcessor(window_px=16), "ov0": piv.PIVProcessor(16, 0.0, 1.1),
             "ov75": piv.PIVProcessor(16, 0.75, 2.0), "big": piv.PIVProcessor(32, 0.5)}
    for k, p in procs.items():
        f = p.process(pair, prov, Attitude(), (0.1, -0.05)) if k == "d" else p.process(pair, prov)
        res[k] = [f.xs, f.ys, f.u, f.v, f.valid, f.shape, f.speed(), f.coverage(), f.status()]
    for ov in (-0.1, 1.0):
        with pytest.raises(ValueError, match=r"^overlap must be in \[0, 1\)"):
            piv.PIVProcessor(16, ov).process(pair, prov)
    for dt in (0.0, -1.0):
        with pytest.raises(ValueError, match="^dt_us must be positive"):
            piv.PIVProcessor(16).process(piv.FramePair(a, b, dt), prov)
    res["tiny"] = piv.PIVProcessor(64).process(piv.FramePair(a[:8, :8], b[:8, :8], 5.0), prov).shape
    g = np.zeros((2, 3))
    vf = piv.VelocityField(piv.LaserSheetPlane(), g, g, g + 1.0, g, np.array([[1, 0, 1], [0, 0, 0]]))
    res["vf"] = [vf.speed(), vf.coverage(), vf.status(),
                 piv.VelocityField(piv.LaserSheetPlane(), np.zeros((0,)), np.zeros(0), np.zeros(0), np.zeros(0),
                                   np.zeros(0)).coverage()]
    with pytest.raises(ValueError, match="^xs, ys, u, v, valid must all share one shape"):
        piv.VelocityField(piv.LaserSheetPlane(), g, g, g, g, np.zeros((3, 2)))
    res["sep"] = piv.separation_mask(np.array([-1.0, 0.0, 1.0, -2.0]), np.array([True, True, True, False]))
    check("piv_fields", res)


def _grid(nx=5, ny=4, x0=0.0, x1=0.4, y0=0.0, y1=0.3):
    xs, ys = np.meshgrid(np.linspace(x0, x1, nx), np.linspace(y0, y1, ny))
    return xs, ys


def test_cfd_slice_and_correlation():
    res = {}
    xs, ys = _grid()
    u = 20.0 + 5.0 * xs - 3.0 * ys
    v = 1.0 + 2.0 * ys
    pl = piv.LaserSheetPlane()
    asc = piv.CFDFieldSlice(pl, xs, ys, u, v)
    desc = piv.CFDFieldSlice(pl, xs[:, ::-1], ys[::-1, :], u[::-1, ::-1], v[::-1, ::-1])
    one_d = piv.CFDFieldSlice(pl, xs[0], ys[:, 0], u, v)
    qx, qy = np.meshgrid(np.array([-0.1, 0.0, 0.05, 0.2, 0.4, 0.45]), np.array([0.0, 0.11, 0.3, 0.31]))
    res["grid"] = [asc.on_grid(qx, qy), desc.on_grid(qx, qy), one_d.on_grid(qx, qy)]
    flat = piv.CFDFieldSlice(pl, np.zeros((2, 2)) + [[0.0, 0.0]], np.array([[0.0, 0.0], [0.1, 0.1]]),
                             np.ones((2, 2)), np.ones((2, 2)))
    res["degenerate"] = flat.on_grid(np.array([[0.0]]), np.array([[0.05]]))
    pv = lambda uu, vv, valid=None, prov=True: piv.VelocityField(
        pl, xs, ys, uu, vv, np.ones(xs.shape, bool) if valid is None else valid,
        attitude=Attitude(), provenance=_prov() if prov else None)
    sep_u = u.copy()
    sep_u[0, :2] = -3.0
    cases = {"match": (pv(u, v), asc, None), "mag": (pv(u * 1.3, v), asc, None),
             "angle": (pv(u, v + 8.0), asc, None), "piv_sep": (pv(sep_u, v), asc, None),
             "cfd_sep": (pv(u, v), piv.CFDFieldSlice(pl, xs, ys, sep_u, v), None),
             "both_sep": (pv(sep_u, v), piv.CFDFieldSlice(pl, xs, ys, sep_u, v), None),
             "all_sep": (pv(-u, v), piv.CFDFieldSlice(pl, xs, ys, -u, v), None),
             "tol": (pv(u * 1.05, v + 2.0), asc, {"mag_rms_pct": 1.0, "angle_rms_deg": 1.0}),
             "noprov": (pv(u, v, prov=False), asc, None),
             "none": (pv(u, v, valid=np.zeros(xs.shape, bool)), asc, None),
             "none_noprov": (pv(u, v, valid=np.zeros(xs.shape, bool), prov=False), asc, None),
             "outside": (pv(u, v), piv.CFDFieldSlice(pl, xs + 5.0, ys, u, v), None),
             "nan_cfd": (pv(u, v), piv.CFDFieldSlice(pl, xs, ys, u * np.nan, v), None)}
    for k, (p, c, tol) in cases.items():
        res[k] = piv.correlate_field(p, c, tol).as_dict()
    with pytest.raises(ValueError) as e:
        piv.correlate_field(pv(u, v), piv.CFDFieldSlice(piv.LaserSheetPlane(O.XY_PLAN), xs, ys, u, v))
    res["orient"] = str(e.value)
    res["tol"] = piv.DEFAULT_FIELD_TOL
    check("piv_correlate", res)


def test_plan_and_rig(tmp_path):
    res = {}
    plan = piv.AcquisitionPlan(piv.LaserSheetPlane(O.XY_PLAN, 0.02), Attitude(pitch_deg=1.0), 25.0, 8e-5)
    plan2 = piv.AcquisitionPlan(piv.LaserSheetPlane(), Attitude(), 10.0, 1e-4, 50, 2.0, 24, 30.0, "note")
    res["prov"] = [plan.to_provenance("T1"), plan2.to_provenance("T2", GroundState.FIXED_FLOOR)]
    res["text"] = [plan.plan_text(), plan2.plan_text()]
    rig = piv.OfflinePIVRig()
    d = tmp_path / "w"
    p = rig.write_plan(plan2, str(d))
    rig.write_plan(plan2, str(d))
    res["file"] = [os.path.basename(p), open(p).read(), rig.name]
    with pytest.raises(piv.RigUnavailable) as e:
        rig.acquire(plan, str(d))
    res["acq"] = str(e.value)
    res["proto"] = isinstance(rig, piv.PIVRig)
    check("piv_plan", json.loads(json.dumps(plain(res)).replace(str(tmp_path), "<T>")))


def test_characterized_edges(tmp_path):
    res = {}
    img = _speckle((15, 15), seed=4, n=40)
    shifted = np.roll(img, (2, -1), axis=(0, 1))
    res["odd"] = piv._window_displacement(img, shifted, 1.2)
    tiny = _speckle((3, 3), seed=5, n=3)
    res["tiny"] = piv._window_displacement(tiny, np.roll(tiny, 1, axis=1), 1.2)
    fr = _speckle((9, 9), seed=6, n=20)
    pair = piv.FramePair(fr, np.roll(fr, 1, axis=1), 20.0)
    for k, proc in (("w4", piv.PIVProcessor(4, 0.5)), ("w2", piv.PIVProcessor(2, 0.5))):
        f = proc.process(pair, _prov())
        res[k] = [f.xs, f.ys, f.u, f.v, f.valid, f.speed(), f.coverage()]
    bad = _prov(particle_diameter_um=12.0, dt_us=500.0, window_px=40, ground_state=GroundState.FIXED_FLOOR)
    res["status"] = bad.status()
    plan = piv.AcquisitionPlan(piv.LaserSheetPlane(), Attitude(), 10.0, 0.002)
    res["plan"] = plan.plan_text()
    path = piv.OfflinePIVRig().write_plan(plan, str(tmp_path / "w2"))
    res["path_in_workdir"] = path.startswith(str(tmp_path / "w2"))
    xs, ys = _grid(nx=6, ny=5)
    u = 20.0 + np.sin(9.0 * xs) * np.cos(7.0 * ys)
    v = 1.0 + xs * ys * 30.0
    pl = piv.LaserSheetPlane()
    nl = piv.CFDFieldSlice(pl, xs, ys, u, v)
    nld = piv.CFDFieldSlice(pl, xs[:, ::-1], ys[::-1, :], u[::-1, ::-1], v[::-1, ::-1])
    qx, qy = np.meshgrid(np.array([0.0, 0.03, 0.37, 0.4]), np.array([0.0, 0.02, 0.28, 0.3]))
    res["nl"] = [nl.on_grid(qx, qy), nld.on_grid(qx, qy)]
    pxs, pys = _grid(nx=3, ny=2)
    pu = np.array([[0.0, 0.5, 20.0], [20.0, -2.0, 20.0]])
    pv_ = np.ones((2, 3))
    valid = np.array([[True, True, True], [True, True, False]])
    field = piv.VelocityField(pl, pxs, pys, pu, pv_, valid, provenance=_prov())
    cfd_u = np.array([[0.5, 0.0, 20.0], [20.0, 20.0, -1.0]])
    res["corr"] = [piv.correlate_field(field, piv.CFDFieldSlice(pl, pxs, pys, cu, pv_)).as_dict()
                   for cu in (cfd_u, pu, np.array([[20.0, 20.0, 20.0], [20.0, 20.0, 20.0]]),
                              np.array([[20.0, 20.0, 20.0], [-1.0, 20.0, 20.0]]))]
    check("piv_edges", json.loads(json.dumps(plain(res)).replace(str(tmp_path), "<T>")))
