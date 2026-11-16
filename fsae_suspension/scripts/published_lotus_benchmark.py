# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""KinematiK against Lotus Suspension Analysis (SHARK) results published by other teams: someone else's model,
someone else's definitions of camber, toe, caster, kingpin inclination and damper ratio.

Sources (hardpoints and results transcribed from the papers, not adjusted):
  A  Biswal, Prasanth, Sakhamuri, Selhi, "Design and Optimization of the Steering System of a Formula SAE Car Using
     Solidworks and Lotus Shark", Proc. WCE 2016, pp. 1062-1066, fig. 5 (hardpoints) and fig. 7 (bump table).
  B  Singh, Patil, Arwari, Waingankar, "Optimization of Suspension Parameters using LOTUS Shark Suspension Analyser",
     IJERT 9(3), NTASU-2020 (2021), table 2 (front hardpoints) and the text (camber and KPI at -30 and +30 mm).

Frames: both papers use Lotus' x rearward, z up; B is a left corner (y negative), mirrored here to y outboard.
Bump is matched on wheel-centre travel. Static camber and toe come from each paper's spindle and wheel-centre points.

Limits, set before KinematiK was run on these corners and not changed after:
  camber, caster, kingpin inclination 0.01 deg (the papers print 4 decimals; transcription and rounding);
  damper ratio 0.005 (printed to 3 decimals).
Where a printed value disagrees, Project Chrono solves the same published hardpoints independently (``chrono`` mode,
under the Chrono environment); benchmarks/published_lotus/chrono_check.json records it. If Chrono agrees with KinematiK,
the published table does not follow from its own hardpoints, and the row stays a FAIL with that reason beside it.

Scoring rules, also set beforehand: A's toe column is printed without decimals from +1 mm on (e.g. "836"), so toe is
compared only from -25 to 0 mm, in degrees. A row whose printed value breaks the smooth trend of its column by more than
the limit (a typo in the source, e.g. A's camber at +25) is reported as a source error, not scored, and listed.
"""
from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "benchmarks" / "published_lotus"
sys.path.insert(0, str(ROOT))

LIMITS = {"camber": 0.01, "toe": 0.01, "caster": 0.01, "kpi": 0.01, "damper_ratio": 0.005}

# --- A: WCE 2016, fig. 5 (mm) -----------------------------------------------------------------------------------
A_PTS = {"lower_front_inner": (-11.3, 260, 138.794), "lower_rear_inner": (149.02, 260, 138.794),
         "lower_outer": (-11.33, 565.58, 137.23), "upper_front_inner": (11.33, 322.85, 366.12),
         "upper_rear_inner": (149.02, 322.85, 366.12), "upper_outer": (11.33, 538.36, 396.17),
         "pushrod_outer": (20.8, 555, 137.5), "rocker_pushrod": (33.76, 304.13, 317.79),
         "tie_rod_outer": (95, 508, 165), "tie_rod_inner": (95, 193.3575, 162),
         "spring_inner": (40.14, 253.78, 84.67), "rocker_spring": (40.14, 253.78, 275.17),
         "spindle": (0, 565.58, 266.7), "wheel_center": (0, 610, 266.7),
         "rocker_pivot": (33.86, 303.37, 274.29), "rocker_axis_2": (40.14, 303.37, 274.29)}
# fig. 7: bump (mm): camber, toe, castor, kingpin (deg), damper ratio
A_BUMP = """-25 0.6838 0.0535 4.9953 5.3125 1.593
-24 0.6597 0.0489 4.9955 5.337 1.58
-23 0.6354 0.0445 4.9957 5.3617 1.556
-22 0.6107 0.0403 4.9959 5.3867 1.544
-21 0.5858 0.0363 4.9961 5.412 1.532
-20 0.5607 0.0325 4.9963 5.4375 1.52
-19 0.5352 0.0289 4.9966 5.4632 1.509
-18 0.5095 0.0255 4.9968 5.4892 1.497
-17 0.4835 0.0233 4.997 5.5155 1.486
-16 0.4573 0.0193 4.9972 5.542 1.475
-15 0.4308 0.0166 4.9974 5.5687 1.464
-14 0.404 0.014 4.9979 5.623 1.453
-13 0.3769 0.0116 4.9981 5.6505 1.442
-12 0.3496 0.0095 4.9984 5.6783 1.431
-11 0.322 0.00075 4.9946 5.7063 1.421
-10 0.2941 0.0058 4.9984 5.7346 1.41
-9 0.2659 0.0042 4.9986 5.7632 1.4
-8 0.2375 0.0029 4.9989 5.792 1.389
-7 0.2088 0.0018 4.91 5.8211 1.379
-6 0.1798 0.0009 4.94 5.8504 1.369
-5 0.1505 0.0002 4.96 5.88 1.359
-4 0.121 -0.0003 4.99 5.9098 1.349
-3 0.0912 -0.0005 5.002 5.9399 1.339
-2 0.0611 -0.0006 5.004 5.9703 1.33
-1 0.0307 -0.0004 5.007 6.0009 1.32
0 0 0 5.001 6.0319 1.31
1 -0.031 6 5.0012 6.063 1.301
2 -0.0622 14 5.0015 6.0945 1.291
3 -0.0937 25 5.0018 6.1262 1.282
4 -0.1225 38 5.0021 6.1581 1.273
5 -0.1576 52 5.0024 6.1904 1.264
6 -0.19 70 5.0027 6.2229 1.255
7 -0.2227 89 5.003 6.2557 1.246
8 -0.2557 111 5.0033 6.2887 1.237
9 -0.289 135 5.0036 6.3221 1.228
10 -0.3225 161 5.0039 6.3557 1.219
11 -0.35645 189 5.0043 6.3895 1.21
12 -0.3905 220 5.0046 6.4327 1.201
13 -0.425 253 5.0049 6.48581 1.193
14 -0.4597 289 5.0052 6.4929 1.184
15 -0.4948 327 5.0056 6.5279 1.176
16 -0.5301 367 5.007 6.5632 1.156
17 -0.5658 409 5.0073 6.5987 1.137
18 -0.6017 454 5.0077 6.6246 1.159
19 -0.638 501 5.0081 6.6707 1.15
20 -0.6746 551 5.0085 6.7071 1.142
21 -0.7115 603 5.0088 6.7439 1.133
22 -0.7487 657 5.0092 6.7809 1.125
23 -0.7862 714 5.0096 6.8182 1.116
24 -0.824 774 5.0099 6.8558 1.108
25 -0.8261 836 5.0102 6.8932 1.1"""

# --- B: IJERT 2021, table 2, front (mm); a left corner, mirrored to y outboard ------------------------------------
B_PTS = {"lower_front_inner": (-150, -213.8, 145.8), "lower_rear_inner": (150, -213.8, 145.8),
         "lower_outer": (-5.79, -539.8, 145.8), "upper_front_inner": (-150, -242.2, 280.8),
         "upper_rear_inner": (150, -242.2, 280.8), "upper_outer": (5.79, -528.22, 311.4),
         "tie_rod_outer": (74.69, -508.85, 178.5), "tie_rod_inner": (80, -208.52, 178.5),
         "spindle": (0, -534.17, 226.44), "wheel_center": (0, -592.02, 228.46)}
#: text: camber -0.9596 at -30 mm and -3.0576 at +30 mm; KPI 2.9929 at -30 mm and 4.8666 at +30 mm
B_POINTS = {"camber": {-30: -0.9596, 30: -3.0576}, "kpi": {-30: 2.9929, 30: 4.8666}}


def _static(pts):
    s = np.subtract(pts["wheel_center"], pts["spindle"])
    return float(-np.degrees(np.arctan2(s[2], s[1]))), float(-np.degrees(np.arctan2(s[0], s[1])))   # camber, toe


def corner(pts, mirror=False):
    from suspension.kinematics import Hardpoints
    p = {k: np.array([v[0], -v[1] if mirror else v[1], v[2]], float) for k, v in pts.items()}
    cam, toe = _static(p)
    kw = {k: p[k] for k in ("lower_front_inner", "lower_rear_inner", "lower_outer", "upper_front_inner",
                            "upper_rear_inner", "upper_outer", "tie_rod_outer", "tie_rod_inner", "wheel_center")}
    kw["contact_patch"] = np.array([p["wheel_center"][0], p["wheel_center"][1], 0.0])
    hp = replace(Hardpoints.default(), static_camber=cam, static_toe=-toe, **kw)
    if "rocker_pivot" in p:
        hp = replace(hp, pushrod_outer=p["pushrod_outer"], rocker_pivot=p["rocker_pivot"],
                     rocker_axis=p["rocker_axis_2"] - p["rocker_pivot"], rocker_pushrod=p["rocker_pushrod"],
                     rocker_spring=p["rocker_spring"], spring_inner=p["spring_inner"], pushrod_attach="lower")
    return hp


def curves(hp, travel):
    from suspension.kinematics import SuspensionKinematics
    kin = SuspensionKinematics(hp)
    st = kin.sweep(-45, 45, 361)
    dz = np.array([s.wheel_center[2] for s in st]) - hp.wheel_center[2]
    col = {"camber": [s.camber for s in st], "toe": [-s.toe for s in st], "caster": [s.caster for s in st],
           "kpi": [s.kpi for s in st]}
    out = {k: np.interp(travel, dz, v) for k, v in col.items()}
    if hp.rocker_pivot is not None:
        lens, seed = [], 0.0
        for s in st:
            L, seed, _ = kin.spring_length_at(s, seed=seed)
            lens.append(L)
        L = np.interp(travel, dz, lens)
        out["damper_ratio"] = 1.0 / np.abs(np.gradient(L, travel))      # Lotus: wheel travel per damper travel
    return out


def compare_a():
    rows = np.array([[float(x) for x in ln.split()] for ln in A_BUMP.splitlines()])
    t = rows[:, 0]
    k = curves(corner(A_PTS), t)
    cols = {"camber": 1, "toe": 2, "caster": 3, "kpi": 4, "damper_ratio": 5}
    res = {}
    for name, c in cols.items():
        lotus = rows[:, c].copy()
        use = np.ones_like(t, bool) if name != "toe" else t <= 0
        # a printed value that breaks its column's own smooth trend by more than the limit is a source error
        fit = np.polyval(np.polyfit(t[use], lotus[use], 4), t)
        typo = use & (np.abs(lotus - fit) > LIMITS[name])
        scored = use & ~typo
        d = np.abs(k[name] - lotus)
        res[name] = {"worst": float(d[scored].max()), "at": float(t[scored][np.argmax(d[scored])]),
                     "scored": int(scored.sum()), "source_errors": t[typo].tolist(),
                     "passed": bool(d[scored].max() <= LIMITS[name])}
    return res


def compare_b():
    hp = corner(B_PTS, mirror=True)
    k = curves(hp, np.array([-30.0, 0.0, 30.0]))
    res = {}
    for name, pts in B_POINTS.items():
        d = {tr: float(np.interp(tr, [-30, 0, 30], k[name]) - v) for tr, v in pts.items()}
        res[name] = {"kinematik": {tr: float(np.interp(tr, [-30, 0, 30], k[name])) for tr in pts}, "lotus": pts,
                     "worst": max(abs(x) for x in d.values()), "passed": all(abs(x) <= LIMITS[name] for x in d.values())}
    return res


CHECK_TRAVEL = (-30.0, -25.0, -12.0, 0.0, 12.0, 25.0, 30.0)


def chrono_check():
    """Camber, toe-in, caster and KPI (deg) of both published corners at CHECK_TRAVEL, solved by Chrono alone.
    Run with the Chrono environment's python after ``python scripts/published_lotus_benchmark.py export``."""
    import json
    import pychrono as ch
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import chrono_benchmark as cb
    import random_geometry_benchmark as rg
    res = {}
    for name in ("A", "B"):
        w = json.loads((OUT / f"corner_{name}.json").read_text())
        P = {k: np.asarray(v, float) for k, v in w.items() if isinstance(v, list) and len(v) == 3}
        tip = rg._tip(w)
        s = cb._system()
        ground = cb._body(s, fixed=True)
        lca, uca, up = cb._body(s), cb._body(s), cb._body(s)
        mot = ch.ChLinkMotorRotationAngle()
        mot.Initialize(lca, ground, cb._frame_z_along(P["lower_front_inner"], P["lower_rear_inner"] - P["lower_front_inner"]))
        fn = ch.ChFunctionConst(0.0)
        mot.SetAngleFunction(fn)
        rev = ch.ChLinkLockRevolute()
        rev.Initialize(uca, ground, cb._frame_z_along(P["upper_front_inner"], P["upper_rear_inner"] - P["upper_front_inner"]))
        s1, s2 = ch.ChLinkLockSpherical(), ch.ChLinkLockSpherical()
        s1.Initialize(up, lca, ch.ChFramed(cb.V(P["lower_outer"])))
        s2.Initialize(up, uca, ch.ChFramed(cb.V(P["upper_outer"])))
        for x in (mot, rev, s1, s2):
            s.Add(x)
        cb._distance(s, up, ground, P["tie_rod_outer"], P["tie_rod_inner"])
        rows = []
        for sgn in (1, -1):
            for b in (lca, uca, up):
                b.SetPos(ch.ChVector3d(0, 0, 0)); b.SetRot(ch.ChQuaterniond(1, 0, 0, 0))  # noqa: E702
            for a in np.radians(np.arange(0, 12.01, 0.05)) * sgn:
                fn.SetConstant(float(a))
                for _ in range(3):
                    s.DoAssembly(ch.POSITION, 100, 1e-10, 1e-14, 1e-12)
                g = {p: cb.A(up.TransformPointLocalToParent(cb.V(q))) for p, q in
                     (("wc", P["wheel_center"]), ("tip", tip), ("lo", P["lower_outer"]), ("uo", P["upper_outer"]))}
                rows.append((g["wc"][2] - P["wheel_center"][2], *rg._wheel_angles(g["wc"], g["tip"]),
                             *rg._axis_angles(g["lo"], g["uo"])))
        o = np.array(sorted(rows))
        res[name] = {q: [float(np.interp(t, o[:, 0], o[:, i])) for t in CHECK_TRAVEL]
                     for i, q in enumerate(("camber", "toe", "caster", "kpi"), start=1)}
    (OUT / "chrono_check.json").write_text(json.dumps({"travel_mm": CHECK_TRAVEL, **res}, indent=1) + "\n")
    print(f"wrote {OUT / 'chrono_check.json'}")


def export():
    import json
    from suspension.genesis_repro import hp_to_dict
    OUT.mkdir(parents=True, exist_ok=True)
    for name, (pts, m) in {"A": (A_PTS, False), "B": (B_PTS, True)}.items():
        d = hp_to_dict(corner(pts, m))
        (OUT / f"corner_{name}.json").write_text(json.dumps(
            {k: ([float(x) for x in v] if hasattr(v, "__len__") and not isinstance(v, str) else v) for k, v in d.items()},
            indent=1) + "\n")


def chrono_agrees(tol=1e-3) -> dict:
    """Largest |KinematiK - Chrono| (deg) per corner over CHECK_TRAVEL, from the recorded Chrono check."""
    import json
    rec = json.loads((OUT / "chrono_check.json").read_text())
    t = np.array(CHECK_TRAVEL)
    out = {}
    for name, (pts, m) in {"A": (A_PTS, False), "B": (B_PTS, True)}.items():
        k = curves(corner(pts, m), t)
        out[name] = max(float(np.max(np.abs(np.asarray(k[q]) - np.asarray(rec[name][q])))) for q in rec[name])
    return out


if __name__ == "__main__" and sys.argv[1:2] == ["export"]:
    export()
elif __name__ == "__main__" and sys.argv[1:2] == ["chrono"]:
    chrono_check()
elif __name__ == "__main__":
    print("A (WCE 2016, front, Lotus SHARK):")
    for n, r in compare_a().items():
        print(f"  {n:13s} worst {r['worst']:.4f} at {r['at']:+.0f} mm over {r['scored']} rows  "
              f"{'PASS' if r['passed'] else 'FAIL'}  source errors at {r['source_errors']}")
    print("B (IJERT 2021, front, Lotus SHARK):")
    for n, r in compare_b().items():
        print(f"  {n:13s} KinematiK {r['kinematik']}  Lotus {r['lotus']}  {'PASS' if r['passed'] else 'FAIL'}")
