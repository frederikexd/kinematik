"""Closed-form checks of the two aero solvers.

  python scripts/aero_closed_form.py

1. Source-panel method on a sphere in free air, against the potential-flow
   surface pressure Cp = 1 - (9/4) sin^2(theta).
2. Vortex lattice on a flat rectangular AR 8 wing in free air, against
   lifting-line theory (elliptic loading), Helmbold's lifting-surface estimate,
   and an independent flat-plate lattice written here from Katz & Plotkin that
   shares no code with the solver.

The numbers this prints are the ones quoted in VALIDATION.md and PHYSICS.md
§10 and §11.
"""
from __future__ import annotations

import math
import os
import sys
import tempfile

import numpy as np
import trimesh

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from suspension.aero.cfd import Attitude, CaseSpec  # noqa: E402
from suspension.aero.panel_method import (  # noqa: E402
    PanelMethodModel,
    PanelParams,
    _freestream_unit,
)
from suspension.aero.vortex_lattice import VortexLatticeModel  # noqa: E402


def sphere_cp_error(subdivisions: int = 3, radius: float = 0.5) -> dict:
    """Panel-method Cp on an icosphere against the analytic distribution."""
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "sphere.stl")
        trimesh.creation.icosphere(subdivisions=subdivisions, radius=radius).export(path)
        pm = PanelMethodModel(PanelParams(ground_effect=False, lifting=False,
                                          max_panels=None))
        spec = CaseSpec(attitude=Attitude(ride_height_mm=1000.0),
                        geometry_path=path, reference_area_m2=math.pi * radius ** 2)
        c, nrm, ar, _l, tris, _g = pm._load_panels(spec)
    v = _freestream_unit(spec.attitude)
    a = np.asarray(pm._influence_matrix(c, nrm, ar, tris), dtype=float)
    sigma = np.linalg.solve(a, -(nrm @ v))
    vi = pm._induced_velocity(c, nrm, ar, sigma, ground_effect=False,
                              road_plane_z_m=0.0, tris=tris)
    vs = vi + v
    vt = vs - (vs * nrm).sum(1)[:, None] * nrm
    cp = 1.0 - (vt * vt).sum(1)
    ctr = c - (c * ar[:, None]).sum(0) / ar.sum()
    cos = (ctr @ v) / np.linalg.norm(ctr, axis=1)
    err = np.abs(cp - (1.0 - 2.25 * (1.0 - cos ** 2)))
    return {"panels": len(ar), "max_dcp": float(err.max()),
            "mean_dcp": float(err.mean()), "cp_min": float(cp.min())}


def _segment(p, a, b):
    r1, r2 = p - a, p - b
    c = np.cross(r1, r2)
    c2 = (c * c).sum(-1)
    n1 = np.linalg.norm(r1, axis=-1)[..., None]
    n2 = np.linalg.norm(r2, axis=-1)[..., None]
    ok = c2 > 1e-12
    k = np.where(ok, ((b - a) * (r1 / n1 - r2 / n2)).sum(-1)
                 / (4 * np.pi * np.where(ok, c2, 1.0)), 0.0)
    return c * k[..., None]


def independent_flat_plate_cl(ar: float, alpha_deg: float, ns: int, nc: int,
                              chord: float = 0.2) -> float:
    """Flat-plate horseshoe lattice (Katz & Plotkin), free air, small angle."""
    span = ar * chord
    ys = np.linspace(-span / 2, span / 2, ns + 1)
    xs = np.linspace(0.0, chord, nc + 1)
    a, b, p, dy = [], [], [], []
    for i in range(ns):
        for j in range(nc):
            xq = xs[j] + 0.25 * (xs[j + 1] - xs[j])
            a.append([xq, ys[i], 0.0])
            b.append([xq, ys[i + 1], 0.0])
            p.append([xs[j] + 0.75 * (xs[j + 1] - xs[j]), 0.5 * (ys[i] + ys[i + 1]), 0.0])
            dy.append(ys[i + 1] - ys[i])
    a, b, p, dy = (np.asarray(x) for x in (a, b, p, dy))
    pm, ab, bb = p[:, None, :], a[None], b[None]
    far = np.array([1.0e4, 0.0, 0.0])
    w = (_segment(pm, ab + far, ab) + _segment(pm, ab, bb)
         + _segment(pm, bb, bb + far))[..., 2]
    gamma = np.linalg.solve(w, -np.full(len(p), math.sin(math.radians(alpha_deg))))
    return float(2.0 * (gamma * dy).sum() / (span * chord))


def wing_check(ar: float = 8.0, alpha_deg: float = 4.0, ns: int = 40, nc: int = 10,
               ride_height_mm: float = 50000.0) -> dict:
    """KinematiK's lattice on a thin rectangular wing STL."""
    chord = 0.2
    span = ar * chord
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "wing.stl")
        trimesh.creation.box(extents=[chord, span, 0.002]).export(path)
        r = VortexLatticeModel(ns, nc).solve(CaseSpec(
            attitude=Attitude(pitch_deg=alpha_deg, ride_height_mm=ride_height_mm),
            geometry_path=path, reference_area_m2=span * chord))
    cl = abs(r.c_lift)
    a = math.radians(alpha_deg)
    return {"cl": cl,
            "lifting_line": 2 * math.pi * a * ar / (ar + 2),
            "helmbold": 2 * math.pi * ar / (2 + math.sqrt(ar * ar + 4)) * a,
            "independent": independent_flat_plate_cl(ar, alpha_deg, ns, nc),
            "e": cl * cl / (math.pi * ar * r.c_drag)}


def main() -> None:
    print("Sphere, source panels, free air (analytic Cp = 1 - 9/4 sin^2 theta)")
    for sub in (3, 4):
        s = sphere_cp_error(sub)
        print(f"  {s['panels']:5d} panels: Cp_min {s['cp_min']:+.3f} (analytic -1.250), "
              f"max |dCp| {s['max_dcp']:.4f}, mean |dCp| {s['mean_dcp']:.4f}")
    print("\nRectangular AR 8 wing, vortex lattice, alpha 4 deg")
    for ns, nc in ((30, 8), (40, 10), (60, 12)):
        w = wing_check(ns=ns, nc=nc)
        print(f"  {ns}x{nc}: C_L {w['cl']:.4f} | lifting line {w['lifting_line']:.4f} "
              f"({w['cl'] / w['lifting_line'] - 1:+.1%}) | Helmbold {w['helmbold']:.4f} "
              f"({w['cl'] / w['helmbold'] - 1:+.1%}) | independent lattice "
              f"{w['independent']:.4f} ({w['cl'] / w['independent'] - 1:+.1%}) | e {w['e']:.3f}")
    g = wing_check(ride_height_mm=30.0)
    print(f"\n  ground effect, h/c = 0.15: C_L {g['cl']:.4f} "
          f"({g['cl'] / wing_check()['cl']:.2f}x free air)")


if __name__ == "__main__":
    main()
