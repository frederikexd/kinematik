# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Random-geometry benchmark: KinematiK against Project Chrono on corners nobody chose.

The other benchmarks run on a handful of named corners. This one draws N corners at random around front_v11_final and
rear_v11_final (every pickup moved independently, uniformly, within the ranges below), and compares camber, toe,
caster, kingpin inclination, scrub, mechanical trail, the contact-patch path and the damper length over -25..+25 mm of
wheel-centre travel. The limits are those set for Adams View before any run here and are not changed by the results.

Three stages, because Chrono and KinematiK live in different environments:

    python scripts/random_geometry_benchmark.py draw 200          # corners + KinematiK's curves -> benchmarks/random_geometry/
    ./chrono/bin/python scripts/random_geometry_benchmark.py chrono   # Chrono's curves for the same corners
    python scripts/random_geometry_benchmark.py compare           # worst difference per measure -> summary.json

The corners and curves (about 10 MB) are not committed: the seed regenerates them exactly. summary.json is the record.

What it does not do: both sides use KinematiK's definitions of each measure (from the same hardpoints), so it finds
solver and implementation errors, not a wrong definition. That needs a model built by someone else.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "benchmarks" / "random_geometry"
BASES = ("front_v11_final", "rear_v11_final")
TRAVEL = np.arange(-25.0, 26.0, 1.0)
SEED = 20261116
#: Set before the first run, equal to the Adams View limits; never tuned to the results.
LIMITS = {"camber": 0.005, "toe_in": 0.005, "caster": 0.005, "kpi": 0.005,
          "scrub": 0.05, "trail": 0.05, "cp_dx": 0.05, "cp_dz": 0.05, "damper_len": 0.05}
UNITS = {k: ("deg" if k in ("camber", "toe_in", "caster", "kpi") else "mm") for k in LIMITS}
#: How far each pickup may move from the base corner (mm, uniform per axis).
SPREAD = {"upper_front_inner": 20, "upper_rear_inner": 20, "lower_front_inner": 20, "lower_rear_inner": 20,
          "upper_outer": 15, "lower_outer": 15, "tie_rod_inner": 15, "tie_rod_outer": 15,
          "pushrod_outer": 10, "rocker_pushrod": 10, "rocker_spring": 10, "spring_inner": 10}


def _base(name):
    return json.loads((ROOT / "manifests" / f"{name}.recorded.genesis.json").read_text())["recorded"]["winner_hardpoints"]


def _road_point(lo, uo, cp):
    kp = np.asarray(uo, float) - np.asarray(lo, float)
    return np.asarray(lo, float) + (cp[2] - lo[2]) / kp[2] * kp


def _axis_angles(lo, uo):
    """Caster (deg, top rearward +) and KPI (deg, top inboard +) of the kingpin axis lower -> upper ball joint."""
    k = np.asarray(uo, float) - np.asarray(lo, float)
    return math.degrees(math.atan(k[0] / k[2])), math.degrees(math.atan(-k[1] / k[2]))


def _tip(w):
    cam, toe = math.radians(w.get("static_camber", 0.0)), math.radians(w.get("static_toe", 0.0))
    spin = np.array([math.sin(toe), math.cos(toe) * math.cos(cam), -math.sin(cam)])
    return np.asarray(w["wheel_center"], float) + 100.0 * spin / np.linalg.norm(spin)


def _wheel_angles(wc, tip):
    d = np.asarray(tip) - np.asarray(wc)
    return -math.degrees(math.atan(d[2] / d[1])), -math.degrees(math.atan(d[0] / d[1]))   # camber, toe-in


# ---------------------------------------------------------------- stage 1: draw + KinematiK
def kinematik_curves(w):
    sys.path.insert(0, str(ROOT))
    from suspension.genesis_repro import hp_from_dict
    from suspension.kinematics import SuspensionKinematics
    hp = hp_from_dict(w)
    kin = SuspensionKinematics(hp)
    st = kin.sweep(-40, 40, 321)
    if not all(s.converged for s in st):
        return None
    dz = np.array([s.wheel_center[2] for s in st]) - hp.wheel_center[2]
    if dz.min() > -25 or dz.max() < 25 or np.any(np.diff(dz) <= 0):
        return None
    cp = np.array([s.contact_patch for s in st])
    road = np.array([_road_point(s.lower_outer, s.upper_outer, s.contact_patch) for s in st])
    lens, seed = [], 0.0
    for s in st:
        L, seed, ok = kin.spring_length_at(s, seed=seed)
        if not ok:
            return None
        lens.append(L)
    cols = {"camber": np.array([s.camber for s in st]), "toe_in": -np.array([s.toe for s in st]),
            "caster": np.array([s.caster for s in st]), "kpi": np.array([s.kpi for s in st]),
            "scrub": np.array([s.scrub_radius for s in st]), "trail": cp[:, 0] - road[:, 0],
            "cp_dx": cp[:, 0] - hp.contact_patch[0], "cp_dz": cp[:, 2] - hp.contact_patch[2],
            "damper_len": np.array(lens)}
    out = {k: np.interp(TRAVEL, dz, v) for k, v in cols.items()}
    out["damper_len"] = out["damper_len"] - out["damper_len"][TRAVEL == 0.0][0]
    return out


def draw(n):
    rng = np.random.default_rng(SEED)
    OUT.mkdir(parents=True, exist_ok=True)
    corners, curves, rejected, bad = [], [], 0, []
    i = 0
    while len(corners) < n:
        base = _base(BASES[i % len(BASES)])
        i += 1
        w = json.loads(json.dumps(base))
        for p, r in SPREAD.items():
            if p in w:
                w[p] = [float(x) for x in np.asarray(w[p], float) + rng.uniform(-r, r, 3)]
        c = kinematik_curves(w)
        if c is None:
            rejected += 1
            bad.append(w)
            continue
        w["_base"] = BASES[(i - 1) % len(BASES)]
        corners.append(w)
        curves.append({k: v.tolist() for k, v in c.items()})
    (OUT / "corners.json").write_text(json.dumps(corners))
    (OUT / "kinematik.json").write_text(json.dumps(curves))
    (OUT / "rejected.json").write_text(json.dumps(rejected))
    (OUT / "rejected_corners.json").write_text(json.dumps(bad))      # check these in Chrono before calling it a pass
    print(f"{n} corners drawn ({rejected} rejected: did not solve over ±25 mm in KinematiK)")


# ---------------------------------------------------------------- stage 2: Chrono
def chrono_curves(w):
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import chrono_benchmark as cb
    import pychrono as ch
    P = {k: np.asarray(v, float) for k, v in w.items() if isinstance(v, list) and len(v) == 3}
    tip = _tip(w)
    s = cb._system()
    ground = cb._body(s, fixed=True)
    lca, uca, up, rk = cb._body(s), cb._body(s), cb._body(s), cb._body(s)
    mot = ch.ChLinkMotorRotationAngle()
    mot.Initialize(lca, ground, cb._frame_z_along(P["lower_front_inner"], P["lower_rear_inner"] - P["lower_front_inner"]))
    fn = ch.ChFunctionConst(0.0)
    mot.SetAngleFunction(fn)
    rev = ch.ChLinkLockRevolute()
    rev.Initialize(uca, ground, cb._frame_z_along(P["upper_front_inner"], P["upper_rear_inner"] - P["upper_front_inner"]))
    rrev = ch.ChLinkLockRevolute()
    rrev.Initialize(rk, ground, cb._frame_z_along(P["rocker_pivot"], P["rocker_axis"]))
    s1, s2 = ch.ChLinkLockSpherical(), ch.ChLinkLockSpherical()
    s1.Initialize(up, lca, ch.ChFramed(cb.V(P["lower_outer"])))
    s2.Initialize(up, uca, ch.ChFramed(cb.V(P["upper_outer"])))
    for x in (mot, rev, rrev, s1, s2):
        s.Add(x)
    cb._distance(s, up, ground, P["tie_rod_outer"], P["tie_rod_inner"])
    carrier = up if w.get("pushrod_attach", "upright") == "upright" else lca
    cb._distance(s, carrier, rk, P["pushrod_outer"], P["rocker_pushrod"])
    angles = np.radians(np.linspace(-12.0, 12.0, 481))
    order = np.argsort(np.abs(angles), kind="stable")
    res, dead = {}, set()
    for n, k in enumerate(order):
        if np.sign(angles[k]) in dead:                 # past the end of this direction's travel
            continue
        if n == 0 or np.sign(angles[k]) != np.sign(angles[order[n - 1]]):
            for b in (lca, uca, up, rk):
                b.SetPos(ch.ChVector3d(0, 0, 0)); b.SetRot(ch.ChQuaterniond(1, 0, 0, 0))  # noqa: E702
        fn.SetConstant(float(angles[k]))
        for _ in range(3):
            s.DoAssembly(ch.POSITION, 100, 1e-10, 1e-14, 1e-12)
        g = {p: cb.A(up.TransformPointLocalToParent(cb.V(P[p]))) for p in
             ("wheel_center", "contact_patch", "lower_outer", "upper_outer", "tie_rod_outer")}
        g["tip"] = cb.A(up.TransformPointLocalToParent(cb.V(tip)))
        g["rocker_spring"] = cb.A(rk.TransformPointLocalToParent(cb.V(P["rocker_spring"])))
        g["rocker_pushrod"] = cb.A(rk.TransformPointLocalToParent(cb.V(P["rocker_pushrod"])))
        g["pushrod_outer"] = cb.A(carrier.TransformPointLocalToParent(cb.V(P["pushrod_outer"])))
        for a, b in (("tie_rod_outer", "tie_rod_inner"), ("pushrod_outer", "rocker_pushrod")):
            far = P[b] if b == "tie_rod_inner" else g[b]
            if abs(np.linalg.norm(g[a] - far) - np.linalg.norm(P[a] - P[b])) > 1e-6:
                dead.add(np.sign(angles[k]))
                break
        if np.sign(angles[k]) in dead:
            continue
        cp, lo, uo = g["contact_patch"], g["lower_outer"], g["upper_outer"]
        road = _road_point(lo, uo, cp)
        cam, toe = _wheel_angles(g["wheel_center"], g["tip"])
        cas, kpi = _axis_angles(lo, uo)
        res[k] = (g["wheel_center"][2] - P["wheel_center"][2], cam, toe, cas, kpi, None, cp[0] - road[0],
                  cp[0] - P["contact_patch"][0], cp[2] - P["contact_patch"][2],
                  float(np.linalg.norm(g["rocker_spring"] - P["spring_inner"])), cp[1] - road[1])
    arr = np.array([[x if x is not None else np.nan for x in res[k]] for k in sorted(res)], float)
    srt = np.argsort(arr[:, 0])
    if arr[srt[0], 0] > -25 or arr[srt[-1], 0] < 25:
        return None
    names = ("camber", "toe_in", "caster", "kpi", None, "trail", "cp_dx", "cp_dz", "damper_len", "scrub")
    out = {nm: np.interp(TRAVEL, arr[srt, 0], arr[srt, i]) for i, nm in enumerate(names, start=1) if nm}
    out["damper_len"] = out["damper_len"] - out["damper_len"][TRAVEL == 0.0][0]
    return out


def chrono():
    corners = json.loads((OUT / "corners.json").read_text())
    curves = []
    for i, w in enumerate(corners):
        c = chrono_curves(w)
        curves.append(None if c is None else {k: v.tolist() for k, v in c.items()})
        if i % 20 == 0:
            print(f"{i}/{len(corners)}", flush=True)
    (OUT / "chrono.json").write_text(json.dumps(curves))
    print(f"wrote {OUT / 'chrono.json'}")


# ---------------------------------------------------------------- stage 3: compare
def compare() -> dict:
    kin = json.loads((OUT / "kinematik.json").read_text())
    chr_ = json.loads((OUT / "chrono.json").read_text())
    worst = {k: 0.0 for k in LIMITS}
    where = {k: None for k in LIMITS}
    failed, unsolved = [], [i for i, c in enumerate(chr_) if c is None]
    for i, (a, b) in enumerate(zip(kin, chr_)):
        if b is None:
            continue
        bad = False
        for k in LIMITS:
            d = float(np.max(np.abs(np.asarray(a[k]) - np.asarray(b[k]))))
            if d > worst[k]:
                worst[k], where[k] = d, i
            bad |= d > LIMITS[k]
        if bad:
            failed.append(i)
    n = len(kin) - len(unsolved)
    return {"corners": len(kin), "compared": n, "chrono_unsolved": unsolved, "failed": failed,
            "worst": worst, "worst_corner": where, "passed": not failed and not unsolved}


def main(argv):
    if argv[:1] == ["draw"]:
        draw(int(argv[1]) if len(argv) > 1 else 200)
    elif argv[:1] == ["chrono"]:
        chrono()
    else:
        r = compare()
        print(f"{r['compared']} of {r['corners']} corners compared; Chrono did not solve {len(r['chrono_unsolved'])}")
        for k, lim in LIMITS.items():
            print(f"  {k:11s} worst {r['worst'][k]:.2e} {UNITS[k]:3s} (limit {lim})  corner {r['worst_corner'][k]}")
        print("PASS" if r["passed"] else f"FAIL on corners {r['failed'][:20]}")
        r.update(seed=SEED, limits=LIMITS, spread_mm=SPREAD, travel_mm=[-25, 25],
                 kinematik_rejected=json.loads((OUT / "rejected.json").read_text()) if (OUT / "rejected.json").exists() else None)
        (OUT / "summary.json").write_text(json.dumps(r, indent=1) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
