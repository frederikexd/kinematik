# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Actuation of the final corners placed against everything it can hit, not only the frame.

The pushrod acts at the lower ball joint, so from there it must clear the upright, the hub and outer joint, every link
and, at the rear, the halfshaft, inner joint and differential, at every wheel position. The ChassisV10 tubes come from
the STEP file; the other parts have no CAD here and are declared envelopes (capsules, mm):

* upright: the line between the ball joints less 35 mm at each end (the joint housings, where the pushrod clevis and
  the links attach), radius 20; steering arm: from that line to the tie-rod outer joint, radius 12; front hub (bearing
  boss): the 45 mm inboard of the wheel centre, radius 40; rear hub and outer joint: the 60 mm inboard, radius 45;
* rim: the 127 mm inner radius over the rim's 178 mm width, as 36 capsules of 40 mm radius parallel to the spin axis
  (within 2 mm of the cylinder);
* links: their own axes, radius 7.9 (tie rod 9.5 front); all move with the wheel (actuation.actuation_metrics,
  wheel_obstacles);
* rear: halfshaft from the wheel centre to an inner joint 150 mm from the centreline, radius 15 (its outer end moves
  with the wheel), inner joint 130 to 190 mm from the centreline, radius 40, differential within 120 mm, radius 75;
* front: steering rack housing along the tie-rod inner line, radius 20.

Clearance is surface to surface: the pushrod is 7.9 mm in radius, the rocker arms 10 mm and the coilover 30 mm; the
pushrod's first 30 mm (its rod end and clevis at the ball joint) and the coilover's end eyes (25 mm at each end) are left
out, and every member keeps 5 mm from every tube and envelope at every wheel position. The rocker axis is free, so the rocker can lean with a pushrod that passes fore or aft of the hub.

usage: python scripts/actuation_package.py front|rear hp_in.json hp_out.json [lean,lean,... | refine]"""
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension import actuation as act                                                  # noqa: E402
from suspension.genesis_analysis import frame_graph_from_step, parse_step_tubes           # noqa: E402
from suspension.genesis_repro import capsules_from_framegraph, hp_from_dict, hp_to_dict   # noqa: E402

AZ = {"front": 950.0, "rear": -680.0}
G = -76.0
R_PR, R_ROCK, R_COIL, SKIP, EYE = 7.9, 10.0, 30.0, 30.0, 25.0
SPRUNG = {"front": (60.1, 2.6), "rear": (66.1, 3.0)}
HUB = {"front": (45.0, 40.0), "rear": (60.0, 45.0)}
RIM_R, RIM_HALF = 127.0, 89.0                                   # rim inner radius and half-width (mm), as the wheel envelope            # inboard length and radius of the hub envelope (mm)
#: ChassisV10.STEP (not in the repository): $KINEMATIK_V10_STEP, else data/ChassisV10.STEP, else the author's path
STEP = next((p for p in (Path(os.environ["KINEMATIK_V10_STEP"]) if os.environ.get("KINEMATIK_V10_STEP") else None,
                         ROOT / "data" / "ChassisV10.STEP", Path("/home/claude/step/v10.step")) if p is not None and p.exists()),
            ROOT / "data" / "ChassisV10.STEP")
PTS = ("rocker_pivot", "rocker_axis", "rocker_pushrod", "rocker_spring", "spring_inner")


def frame_obstacles(axle):
    fg = frame_graph_from_step(parse_step_tubes(STEP.read_text()))
    cap = capsules_from_framegraph(fg, AZ[axle], G)
    return [(np.asarray(a), np.asarray(b), float(r)) for a, b, r in zip(cap.a, cap.b, cap.radius)]


def spin_axis(hp):
    """The wheel's spin axis at static height, outboard (unit)."""
    from suspension.kinematics import SuspensionKinematics
    kin = SuspensionKinematics(hp)
    st = kin.solve_at_travel(0.0)
    Rm, _ = kin._upright_pose(st.lower_outer, st.upper_outer, st.tie_rod_outer)
    a = Rm @ np.asarray(kin._spin_local, float)
    return a / np.linalg.norm(a)


def envelopes(hp, axle):
    """(wheel_obstacles, chassis_obstacles) for ``hp`` at static height."""
    g = lambda k: np.asarray(getattr(hp, k), float)                        # noqa: E731
    wc = g("wheel_center"); s = float(np.sign(wc[1])); yo = spin_axis(hp)
    tr_r = 9.5 if axle == "front" else 7.9
    lo, uo, to = g("lower_outer"), g("upper_outer"), g("tie_rod_outer")
    k = (uo - lo) / np.linalg.norm(uo - lo)
    foot = lo + float((to - lo) @ k) * k                     # the steering arm's root on the upright line
    wheel = [(lo + 35.0 * k, uo - 35.0 * k, 20.0, "wheel"),
             (foot, to - 11.0 * (to - foot) / np.linalg.norm(to - foot), 12.0, "wheel"),
             (wc - HUB[axle][0] * yo, wc, HUB[axle][1], "wheel")]
    e1 = np.cross(yo, [1.0, 0.0, 0.0]); e1 /= np.linalg.norm(e1); e2 = np.cross(yo, e1)
    for th in np.radians(np.arange(0.0, 360.0, 10.0)):
        c = wc + (RIM_R + 40.0) * (np.cos(th) * e1 + np.sin(th) * e2)
        wheel.append((c - RIM_HALF * yo, c + RIM_HALF * yo, 40.0, "wheel"))
    for o, i, r in (("upper_outer", "upper_front_inner", 7.9), ("upper_outer", "upper_rear_inner", 7.9),
                    ("lower_outer", "lower_front_inner", 7.9), ("lower_outer", "lower_rear_inner", 7.9),
                    ("tie_rod_outer", "tie_rod_inner", tr_r)):
        wheel.append((g(o), g(i), r, "link"))
    chassis = []
    if axle == "rear":
        inner = np.array([wc[0], s * 150.0, wc[2]])
        wheel.append((wc - 60.0 * yo, inner, 15.0, "link"))
        chassis += [(np.array([wc[0], s * 130.0, wc[2]]), np.array([wc[0], s * 190.0, wc[2]]), 40.0),
                    (np.array([wc[0], -120.0, wc[2]]), np.array([wc[0], 120.0, wc[2]]), 75.0)]
    else:
        ti = g("tie_rod_inner")
        chassis.append((np.array([ti[0], -ti[1], ti[2]]), ti, 20.0))
    return wheel, chassis


def lean_seed(hp, deg):
    """The rocker assembly turned about the lateral line through the pushrod's outer mount by ``deg``."""
    import copy
    h = copy.deepcopy(hp)
    c, s_ = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    Ry = np.array([[c, 0.0, s_], [0.0, 1.0, 0.0], [-s_, 0.0, c]])
    o = np.asarray(hp.pushrod_outer, float)
    for p in ("rocker_pivot", "rocker_pushrod", "rocker_spring", "spring_inner"):
        setattr(h, p, o + Ry @ (np.asarray(getattr(hp, p), float) - o))
    h.rocker_axis = Ry @ np.asarray(hp.rocker_axis, float)
    return h


def direct_seed(hp, dxdz, dydz):
    """The rocker assembly turned about the pushrod's outer mount by the least rotation that points the pushrod along
    (dx/dz, dy/dz, 1) in the corner frame (y outboard), so a pushrod can be aimed past the hub before synthesis."""
    import copy
    o = np.asarray(hp.pushrod_outer, float)
    a = np.asarray(hp.rocker_pushrod, float) - o; a /= np.linalg.norm(a)
    s = float(np.sign(np.asarray(hp.wheel_center, float)[1]))
    b = np.array([dxdz, s * dydz, 1.0]); b /= np.linalg.norm(b)
    v = np.cross(a, b); c = float(a @ b); vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    Rm = np.eye(3) + vx + vx @ vx / (1.0 + c)
    h = copy.deepcopy(hp)
    for p in ("rocker_pivot", "rocker_pushrod", "rocker_spring", "spring_inner"):
        setattr(h, p, o + Rm @ (np.asarray(getattr(hp, p), float) - o))
    h.rocker_axis = Rm @ np.asarray(hp.rocker_axis, float)
    return h


def upright_seed(hp, along_mm=20.0, inboard_mm=30.0, dxdz=0.0, dydz=-0.7, fore_mm=0.0):
    """The pushrod moved onto the upright: its outer mount ``along_mm`` up the kingpin line from the lower ball joint,
    ``inboard_mm`` inboard of it and ``fore_mm`` forward (-x), the rocker assembly aimed from there (direct_seed)."""
    import copy
    h = copy.deepcopy(hp)
    lo, uo = np.asarray(hp.lower_outer, float), np.asarray(hp.upper_outer, float)
    k = (uo - lo) / np.linalg.norm(uo - lo)
    s = float(np.sign(np.asarray(hp.wheel_center, float)[1]))
    n = np.array([0.0, -s, 0.0]); n -= (n @ k) * k; n /= np.linalg.norm(n)
    shift = lo + along_mm * k + inboard_mm * n + np.array([-fore_mm, 0.0, 0.0]) - np.asarray(hp.pushrod_outer, float)
    for p in ("pushrod_outer", "rocker_pivot", "rocker_pushrod", "rocker_spring", "spring_inner"):
        setattr(h, p, np.asarray(getattr(hp, p), float) + (shift if p == "pushrod_outer" else 0.0))
    h.pushrod_attach = "upright"
    return direct_seed(h, dxdz, dydz)


def steer_coupling(hp, rack_mm):
    """Spring length change (mm) at static height when the rack moves ``rack_mm`` each way (front only)."""
    from suspension.kinematics import SuspensionKinematics
    out = []
    L0 = None
    for r in (0.0, rack_mm, -rack_mm):
        kin = SuspensionKinematics(hp, pickup_deltas={"tie_rod_inner": np.array([0.0, r, 0.0])})
        L, th, ok = kin.spring_length_at(kin.solve_at_travel(0.0), seed=0.0)
        if L0 is None:
            L0 = L
        else:
            out.append(float(L - L0))
    return out


def package(hp, axle, leans=tuple(range(-50, 51, 5)), box_mm=60.0, n_best=2):
    """Screen rocker-assembly leans about the lateral line through the pushrod's outer mount, then optimise the
    best ``n_best`` with every actuation point and the rocker axis free (bounded least squares, as
    actuation.synthesize_actuation)."""
    frame = frame_obstacles(axle)
    wheel, chassis = envelopes(hp, axle)
    obst = frame + chassis
    ms, hz = SPRUNG[axle]
    kw = dict(obstacles=obst, wheel_obstacles=wheel, pushrod_skip_mm=SKIP, spring_skip_mm=EYE,
              member_radii=(R_PR, R_ROCK, R_ROCK, R_COIL))
    t = act.ActuationTargets.derived()
    screen = []
    for d in leans:
        h0 = (upright_seed(hp, *d) if isinstance(d, tuple) and len(d) >= 4 else
              direct_seed(hp, *d) if isinstance(d, tuple) else lean_seed(hp, d))
        m = act.actuation_metrics(h0, t.travel_mm, **kw)
        r = act._residual(m, t)
        screen.append((float(np.sum(r ** 2)), d, h0))
        print(f"seed lean {d}: residual {screen[-1][0]:.3g} clearance {m.clearance_mm:.1f}", flush=True)
    screen.sort(key=lambda z: z[0])
    best = None
    for sc0, d, h0 in screen[:n_best]:
        pts = (("pushrod_outer",) + PTS) if getattr(h0, "pushrod_attach", "lower") == "upright" else PTS
        res = act.synthesize_actuation(h0, points=pts, box_mm=box_mm, pushrod_box_mm=15.0, axis_box=0.6, n_starts=1,
                                       seed=0, ride_hz=hz, sprung_corner_kg=ms, **kw)
        sc = float(np.sum(np.asarray(res.residual) ** 2)) if len(res.residual) else math.inf
        m = res.metrics
        print(f"lean {d}: ok {res.ok} score {sc:.3g} clearance {m.clearance_mm:.1f} mr {m.mr_static:.3f} "
              f"spread {m.spread:.3f} oop {m.out_of_plane_deg:.1f} tr {np.round(m.transmission_deg, 1).tolist()} "
              f"stroke {m.stroke_used_mm:.1f} arm offset {m.arm_offset_mm:.1f}", flush=True)
        if best is None or sc < best[0]:
            best = (sc, res, d)
        if res.ok:
            break
    return best


def _solve(h0, pts, t, kw, box_mm, push_box, axis_box, max_nfev=200):
    """Bounded least squares on the synthesis targets with one clearance hinge per member (pushrod, both rocker arms,
    coilover), so the search sees every member that is short of clearance, not only the closest."""
    from scipy.optimize import least_squares
    x0 = np.concatenate([np.asarray(getattr(h0, p), float) for p in pts])
    half = np.concatenate([np.full(3, push_box if p == "pushrod_outer" else (axis_box if p == "rocker_axis" else box_mm))
                           for p in pts])

    def res(x):
        m = act.actuation_metrics(act._set(h0, x, pts), t.travel_mm, **kw)
        r = act._residual(m, t, shrink=0.98)
        mc = m.member_clearance_mm if (m.ok and m.member_clearance_mm) else (-50.0,) * 4
        hinge = [max(0.0, t.min_clearance_mm * 1.02 - c) / 5.0 for c in mc]
        return np.concatenate([r, hinge, 0.002 * (x - x0) / half])
    sol = least_squares(res, x0, bounds=(x0 - half, x0 + half), diff_step=1e-3, max_nfev=max_nfev, xtol=1e-6, ftol=1e-8)
    h = act._set(h0, sol.x, pts)
    m = act.actuation_metrics(h, t.travel_mm, **kw)
    return h, m, act._residual(m, t)


def refine(hp, axle, rounds=4, box_mm=30.0):
    """Restart the bounded least squares from its own result until every target is met or it stops improving."""
    frame = frame_obstacles(axle)
    wheel, chassis = envelopes(hp, axle)
    ms, hz = SPRUNG[axle]
    kw = dict(obstacles=frame + chassis, wheel_obstacles=wheel, pushrod_skip_mm=SKIP, spring_skip_mm=EYE,
              member_radii=(R_PR, R_ROCK, R_ROCK, R_COIL))
    best = None
    h = hp
    pts = (("pushrod_outer",) + PTS) if getattr(hp, "pushrod_attach", "lower") == "upright" else PTS
    t = act.ActuationTargets.derived()
    for i in range(rounds):
        hh, m, r = _solve(h, pts, t, kw, box_mm, 10.0, 0.3)
        k_w = (2 * math.pi * hz) ** 2 * ms / 1000.0
        res = act.ActuationResult(ok=bool(np.all(r <= 1e-6)), hp=hh, metrics=m, residual=r.tolist(), seed_metrics=m,
                                  spring_rate_N_per_mm=k_w / m.mr_static ** 2 if m.ok else math.nan,
                                  ride_frequency_hz=[hz * v / m.mr_static for v in m.mr] if m.ok else [])
        sc = float(np.sum(r ** 2))
        print(f"refine {i}: ok {res.ok} score {sc:.3g} clearance {m.clearance_mm:.1f} mr {m.mr_static:.3f} "
              f"spread {m.spread:.3f} oop {m.out_of_plane_deg:.1f} tr {np.round(m.transmission_deg, 1).tolist()} "
              f"stroke {m.stroke_used_mm:.1f} arm offset {m.arm_offset_mm:.1f}", flush=True)
        if best is not None and sc >= best[0] - 1e-9:
            break
        best = (sc, res, "refined")
        h = res.hp
        if res.ok:
            break
    return best


if __name__ == "__main__":
    axle, src, dst = sys.argv[1], sys.argv[2], sys.argv[3]
    d = json.loads(Path(src).read_text())
    hp = hp_from_dict(d["recorded"]["winner_hardpoints"] if "recorded" in d else d)
    if len(sys.argv) > 4 and sys.argv[4] == "refine":
        sc, res, lean = refine(hp, axle)
    else:
        leans = None
        if len(sys.argv) > 4 and sys.argv[4] == "upright":         # pushrod on the upright beside the ball joint
            if axle == "front":                                   # on the kingpin side: no fore offset, so steer barely moves it
                leans = tuple((30.0, b, dx, dy, f) for b in (25.0, 35.0) for f in (-10.0, -20.0) for dx in (0.0, 0.15)
                              for dy in (-0.45, -0.55))
            else:                                                 # behind the halfshaft, clear of the forward upper legs
                leans = tuple((20.0, 25.0, dx, dy, f) for f in (-30.0, -45.0) for dx in (0.0, 0.2) for dy in (-0.6, -0.8))
            sc, res, lean = package(hp, axle, leans=leans, n_best=3)
        elif len(sys.argv) > 4 and sys.argv[4] == "direct":          # aim the pushrod: dx/dz and dy/dz grids
            leans = tuple((dx, dy) for dx in (-0.3, -0.45, -0.6, -0.8, -1.0) for dy in (-0.8, -1.0, -1.2, -1.5))
            sc, res, lean = package(hp, axle, leans=leans, n_best=2)
        else:
            leans = tuple(float(v) for v in sys.argv[4].split(",")) if len(sys.argv) > 4 else None
            sc, res, lean = package(hp, axle, **({"leans": leans, "n_best": len(leans)} if leans else {}))
    m = res.metrics
    out = hp_to_dict(res.hp)
    Path(dst).write_text(json.dumps(out, indent=1, default=float))
    print(json.dumps({"ok": res.ok, "lean_seed_deg": lean, "residual": res.residual, "clearance_mm": m.clearance_mm,
                      "mr_static": m.mr_static, "spread": m.spread, "out_of_plane_deg": m.out_of_plane_deg,
                      "transmission_deg": m.transmission_deg, "stroke_used_mm": m.stroke_used_mm,
                      "spring_N_per_mm": res.spring_rate_N_per_mm, "ride_hz": res.ride_frequency_hz,
                      "rocker_axis": list(map(float, res.hp.rocker_axis)),
                      "pushrod_attach": getattr(res.hp, "pushrod_attach", "lower"),
                      "steer_coupling_spring_mm": steer_coupling(res.hp, 27.0) if axle == "front" else None},
                     indent=1, default=float))
