# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""KinematiK against MathWorks' quarter-car K&C test rig (Simscape Vehicle Templates): someone else's model, someone
else's metric definitions, measured the way a K&C machine measures them.

scripts/matlab/kk_knc_simscape.m loads front_v11_final and rear_v11_final into the Templates' double-wishbone rig, runs
its K&C sequence and writes MathWorks' metrics table (sm_car_knc_calc_susp_metrics) and the toe and camber curves to
benchmarks/knc_simscape/. This script compares them with KinematiK.

How MathWorks defines each metric (from sm_car_knc_calc_susp_metrics.m), and what it is compared with:
  Toe, Camber          static, + toe in, + top out           KinematiK's static toe (sign flipped) and camber
  Caster, KPI          from camber change at +-toe while     KinematiK's kingpin axis (lower to upper ball joint)
                       steering, as an alignment rig does
  Caster Trail         steering axis from a circle fit of     KinematiK's mechanical trail at the road (the contact
                       the wheel centre's path while          patch is directly below the wheel centre in x on these
                       steering, projected to the ground      corners, so the definitions coincide)
  Scrub Radius         the same axis, measured from the       KinematiK's road-plane intersection measured from the
                       point BELOW THE WHEEL CENTRE           same point. KinematiK's printed scrub is measured from
                                                              the contact patch, 5 mm outboard here: a definition
                                                              difference declared before the run, also reported raw
  Bump Steer/Camber    secant over +-10 mm of wheel centre    the same secant on KinematiK's curves (KinematiK prints a
                       travel, deg/m                          least-squares slope over the travel; also reported)
  Roll Steer/Camber    change between +-tan(2 deg) x wheel    the same on KinematiK's curves
                       centre y of travel
  Damper Ratio         secant over +-10 mm, damper on the     KinematiK's direct-acting damper on the same two points
                       lower wishbone (the rig's layout)
  toe and camber       over the whole post sweep, change      KinematiK's curves at the same wheel-centre heights
  curves               from ride height

Harness history, disclosed: the first two runs were not of these corners as designed. The rig's PostLoadFcn reloads
MathWorks' own corner whenever the model loads, so the rear (run 1) and both corners (run 2) ran MathWorks' default
FSAE corner; and in run 1 the front sat 5.23 mm below the height the metric script calls design (the dataset's tyre
radius). The script now loads the model before setting the hardpoints, raises the corner rigidly by that 5.23 mm, and
checks the assembled camber, toe and wheel-centre position. Since run 2, KinematiK's trail and scrub are taken on the
rig's own ground plane, read from MathWorks' reference file. The limits below were not changed.

Limits, set before the first run and not changed after:
  static toe and camber 0.01 deg; curves 0.01 deg; caster and KPI 0.05 deg (the steer method's own approximation);
  trail and scrub 0.5 mm (circle fit); bump steer and camber 0.05 deg/m; roll steer and camber 0.01 deg;
  damper ratio 0.005.

What the run showed (results.json, benchmarks/knc_simscape/README.md): every directly measured quantity passes on both
corners: bump steer and camber within 0.00003 deg/m, roll steer and camber and the curves within 0.00001 deg, damper
ratio within 0.0034. In the steering test, KinematiK at the rig's own rack positions
with the wheel centre held at design height, as the rig holds it, reproduces the rig's toe to 2e-7 deg, its wheel-centre
path to 4e-5 mm and its camber to 2e-5 deg, taking camber as the inclination of the wheel plane (ISO 8855), which the
rig measures. KinematiK prints the front-view projection, as Lotus does; at 12 deg of steer that is 0.06 deg from the
rig. KinematiK now also gives the inclination (suspension.kinematics.camber_inclination).
Caster, KPI, trail and scrub fail. They are not measured by the rig; MathWorks estimates them from the steering test.
MathWorks' own formulas, applied to KinematiK's motion, give back the rig's caster, hub offsets and trail to 0.0001
(mathworks_formulas), so the gaps are those estimates, not the motion. The trail formula adds the hub offset where the
geometry subtracts it, so it appears to have a sign error; with the sign the geometry gives it lands within 0.26 mm of
KinematiK's trail. The scrub formula uses MathWorks' KPI estimate; with KinematiK's KPI it lands within 0.11 mm of
KinematiK's scrub. MathWorks' KPI estimate uses wheel-frame angles the harness did not log, so it is not reproduced
here. The fails stay fails.
"""
from __future__ import annotations

import csv
import functools
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
BENCH = ROOT / "benchmarks" / "knc_simscape"
CORNERS = ("front_v11_final", "rear_v11_final")
LIMITS = {"Toe": 0.01, "Camber": 0.01, "Caster": 0.05, "KPI": 0.05, "Caster Trail": 0.5, "Scrub Radius": 0.5,
          "Bump Steer": 0.05, "Bump Camber": 0.05, "Roll Steer": 0.01, "Roll Camber": 0.01, "Damper Ratio": 0.005,
          "toe curve": 0.01, "camber curve": 0.01}
Z_BUMP_MM = 10.0
ROLL_DEG = 2.0


def _winner(name):
    return json.loads((ROOT / "manifests" / f"{name}.recorded.genesis.json").read_text())["recorded"]["winner_hardpoints"]


def _shock(w):
    """The damper points kk_knc_simscape.m gives the rig (KinematiK frame, mm)."""
    lo, fi, ri = (np.asarray(w[k], float) for k in ("lower_outer", "lower_front_inner", "lower_rear_inner"))
    mid = 0.5 * (fi + ri)
    bot = mid + 0.7 * (lo - mid) + np.array([0.0, 0.0, 15.0])
    return bot, bot + np.array([0.0, -60.0, 300.0])


def kinematik(name, tyre_radius_mm: float | None = None) -> dict:
    """KinematiK's side of every compared quantity, and its toe-in and camber curves (deg) against wheel-centre
    travel (mm)."""
    from suspension.genesis_repro import hp_from_dict
    from suspension.kinematics import SuspensionKinematics
    w = _winner(name)
    hp = hp_from_dict(w)
    kin = SuspensionKinematics(hp)
    st = kin.sweep(-60, 60, 481)
    dz = np.array([s.wheel_center[2] for s in st]) - hp.wheel_center[2]
    toe_in = -np.array([s.toe for s in st])
    cam = np.array([s.camber for s in st])
    f_toe = lambda z: float(np.interp(z, dz, toe_in))                                  # noqa: E731
    f_cam = lambda z: float(np.interp(z, dz, cam))                                     # noqa: E731
    s0 = kin.solve_at_travel(0.0)
    lo, uo, cp = (np.asarray(x, float) for x in (s0.lower_outer, s0.upper_outer, s0.contact_patch))
    kp = uo - lo
    # the ground the rig measures trail and scrub on: one tyre radius below the wheel centre (MathWorks' reference
    # file); KinematiK's own contact patch otherwise
    gz = float(s0.wheel_center[2]) - tyre_radius_mm if tyre_radius_mm else float(cp[2])
    road = lo + (gz - lo[2]) / kp[2] * kp
    wc = np.asarray(s0.wheel_center, float)
    z_roll = math.tan(math.radians(ROLL_DEG)) * float(hp.wheel_center[1])
    # direct-acting damper on the rig's two points
    bot, top = _shock(w)
    d = dict(w, actuation="direct", pushrod_attach="lower", pushrod_outer=list(bot), spring_inner=list(top))
    for k in ("rocker_pivot", "rocker_axis", "rocker_pushrod", "rocker_spring"):
        d.pop(k, None)
    kd = SuspensionKinematics(hp_from_dict(d))
    L = lambda z: kd.spring_length_at(kd.solve_at_travel(z))[0]                         # noqa: E731
    return {
        "Toe": -float(hp.static_toe), "Camber": float(hp.static_camber),
        "Caster": float(s0.caster), "KPI": float(s0.kpi),
        "Caster Trail": float(cp[0] - road[0]),
        "Scrub Radius": float(wc[1] - road[1]),                       # from the point below the wheel centre
        "scrub_printed": float(s0.scrub_radius), "cp_offset_mm": float(cp[1] - wc[1]),
        "Bump Steer": (f_toe(Z_BUMP_MM) - f_toe(-Z_BUMP_MM)) / (2 * Z_BUMP_MM / 1000.0),
        "Bump Camber": (f_cam(Z_BUMP_MM) - f_cam(-Z_BUMP_MM)) / (2 * Z_BUMP_MM / 1000.0),
        "Roll Steer": f_toe(z_roll) - f_toe(-z_roll), "Roll Camber": f_cam(z_roll) - f_cam(-z_roll),
        "Damper Ratio": (L(-Z_BUMP_MM) - L(Z_BUMP_MM)) / (2 * Z_BUMP_MM),
        "curves": (dz, toe_in, cam)}


def mathworks(name) -> dict | None:
    m, c = BENCH / f"{name}_metrics.csv", BENCH / f"{name}_curves.csv"
    if not (m.exists() and c.exists()):
        return None
    with m.open() as fh:
        rows = {r["Names"]: float(r["Values"]) for r in csv.DictReader(fh) if r["Values"] not in ("", "NaN")}
    with c.open() as fh:
        cur = np.array([[float(r["pz_mm"]), float(r["toe_in_deg"]), float(r["camber_deg"])] for r in csv.DictReader(fh)])
    rows["curves"] = cur
    ref = BENCH / f"{name}_reference.csv"
    if ref.exists():
        with ref.open() as fh:
            r = next(csv.DictReader(fh))
        rows["tire_radius_mm"] = float(r["tire_radius_mm"])
    return rows


def compare(name) -> dict | None:
    mw = mathworks(name)
    if mw is None:
        return None
    k = kinematik(name, mw.get("tire_radius_mm"))
    out = {}
    for q in ("Toe", "Camber", "Caster", "KPI", "Caster Trail", "Scrub Radius", "Bump Steer", "Bump Camber",
              "Roll Steer", "Roll Camber", "Damper Ratio"):
        if q in mw:
            out[q] = {"mathworks": mw[q], "kinematik": k[q], "diff": abs(mw[q] - k[q]), "limit": LIMITS[q],
                      "passed": abs(mw[q] - k[q]) <= LIMITS[q]}
    out["Scrub Radius"]["kinematik_printed"] = k["scrub_printed"]
    out["Scrub Radius"]["definition_offset_mm"] = k["cp_offset_mm"]
    dz, toe_in, cam = k["curves"]
    pz, mt, mc = mw["curves"][:, 0], mw["curves"][:, 1], mw["curves"][:, 2]
    use = (pz >= dz.min()) & (pz <= dz.max())
    mc0 = float(np.interp(0.0, *zip(*sorted(zip(pz, mc)))))
    mt0 = float(np.interp(0.0, *zip(*sorted(zip(pz, mt)))))
    dt = np.abs((mt[use] - mt0) - (np.interp(pz[use], dz, toe_in) - np.interp(0.0, dz, toe_in)))
    dc = np.abs((mc[use] - mc0) - (np.interp(pz[use], dz, cam) - np.interp(0.0, dz, cam)))
    for q, d in (("toe curve", dt), ("camber curve", dc)):
        out[q] = {"diff": float(d.max()), "limit": LIMITS[q], "passed": float(d.max()) <= LIMITS[q],
                  "range_mm": [float(pz[use].min()), float(pz[use].max())]}
    return {"name": name, "rows": out, "passed": all(r["passed"] for r in out.values())}


# ---------------------------------------------------------------- the steering test, where the fails come from
STEER_T = (27.5, 48.6)          # the rig's steering test at design height (s): rack 0 -> +13 -> -13 -> 0 mm


def steer_extract(name, every: int = 1) -> Path | None:
    """The steering test at design height, cut from the 3.4 MB run log into <corner>_steer.csv (committed)."""
    ts = BENCH / f"{name}_timeseries.csv"
    if not ts.exists():
        return None
    a = np.genfromtxt(ts, delimiter=",", names=True)
    a = a[(a["t_s"] >= STEER_T[0]) & (a["t_s"] <= STEER_T[1])][::every]
    out = BENCH / f"{name}_steer.csv"
    with out.open("w") as fh:
        fh.write(",".join(a.dtype.names) + "\n")
        for r in a:
            fh.write(",".join(f"{r[k]:.9g}" for k in a.dtype.names) + "\n")
    return out


def _steered(name, racks_mm):
    """KinematiK's corner at each rack position (the rig's sign: + rack moves the inner tie-rod point inboard) with the
    wheel centre held at its design height, as the rig holds it, in the rig's frame: x forward, y outboard, mm."""
    from suspension.genesis_repro import hp_from_dict
    from suspension.kinematics import SuspensionKinematics, camber_inclination
    hp = hp_from_dict(_winner(name))
    rows = []
    for r in racks_mm:
        k = SuspensionKinematics(hp, pickup_deltas={"tie_rod_inner": np.array([0.0, -float(r), 0.0])})
        tr, s = 0.0, k.solve_at_travel(0.0)
        for _ in range(8):                         # travel at which the steered wheel centre is back at design height
            err = float(s.wheel_center[2] - hp.wheel_center[2])
            if abs(err) < 1e-9:
                break
            tr -= err
            s = k.solve_at_travel(tr)
        wc = np.asarray(s.wheel_center, float)
        # camber two ways: the ISO 8855 inclination the rig measures, and KinematiK's front-view projection
        rows.append((float(r), -wc[0], wc[1], wc[2] - hp.wheel_center[2], -float(s.toe),
                     float(camber_inclination(s.camber, s.toe)), float(s.camber)))
    return np.array(rows)


@functools.lru_cache(maxsize=None)
def _rig_steer(name):
    """The rig's steering test (<corner>_steer.csv) and KinematiK at the same rack samples; None if not run."""
    f = BENCH / f"{name}_steer.csv"
    if not f.exists():
        return None
    m = np.genfromtxt(f, delimiter=",", names=True)
    return m, _steered(name, m["rack_mm"])


def steer_motion(name) -> dict | None:
    """The rig's steered wheel against KinematiK's at the same rack travel: toe, camber (KinematiK's ISO 8855
    inclination, and its printed front-view projection), wheel-centre x and y."""
    if _rig_steer(name) is None:
        return None
    m, k = _rig_steer(name)
    x0, y0 = float(m["px_mm"][0]), float(m["py_mm"][0])
    out = {"toe_in": np.abs(m["toe_in_deg"] - k[:, 4]), "camber": np.abs(m["camber_deg"] - k[:, 5]),
           "camber_front_view": np.abs(m["camber_deg"] - k[:, 6]),
           "wc_x": np.abs((m["px_mm"] - x0) - (k[:, 1] - k[0, 1])), "wc_y": np.abs((m["py_mm"] - y0) - (k[:, 2] - k[0, 2]))}
    return {q: float(np.nanmax(v)) for q, v in out.items()} | {"max_toe_deg": float(np.max(np.abs(m["toe_in_deg"])))}


def _pratt(xy):
    """Circle centre by Pratt's algebraic fit: MathWorks' CircleFitByPratt, ported line for line."""
    xy = np.asarray(xy, float)
    c = xy.mean(axis=0)
    X, Y = xy[:, 0] - c[0], xy[:, 1] - c[1]
    Z = X * X + Y * Y
    Mxy, Mxx, Myy, Mxz, Myz, Mzz = (float(np.mean(v)) for v in (X * Y, X * X, Y * Y, X * Z, Y * Z, Z * Z))
    Mz = Mxx + Myy
    cov = Mxx * Myy - Mxy * Mxy
    A2 = 4 * cov - 3 * Mz * Mz - Mzz
    A1 = Mzz * Mz + 4 * cov * Mz - Mxz * Mxz - Myz * Myz - Mz ** 3
    A0 = Mxz * Mxz * Myy + Myz * Myz * Mxx - Mzz * cov - 2 * Mxz * Myz * Mxy + Mz * Mz * cov
    xn, yn = 0.0, 1e20
    for _ in range(20):
        yo, yn = yn, A0 + xn * (A1 + xn * (A2 + 4 * xn * xn))
        if abs(yn) > abs(yo):
            xn = 0.0
            break
        xo, xn = xn, xn - yn / (A1 + xn * (2 * A2 + 16 * xn * xn))
        if abs((xn - xo) / xn) < 1e-12:
            break
    det = xn * xn - xn * Mz + cov
    return c + np.array([Mxz * (Myy - xn) - Myz * Mxy, Myz * (Mxx - xn) - Mxz * Mxy]) / det / 2


def _swing_caster(toe, cam, q):
    """MathWorks' caster: the samples nearest +-q of toe-in, their toe taken as +-q +- the miss, then
    atan2(sin cam(-q) - sin cam(+q), sin toe(+q) - sin toe(-q)) (sm_car_knc_calc_susp_metrics, SAE 850219)."""
    i_n, i_p = int(np.argmin(np.abs(toe + q))), int(np.argmin(np.abs(toe - q)))
    t_n, t_p = abs(toe[i_n] + q) - q, abs(toe[i_p] - q) + q
    r = math.radians
    return math.degrees(math.atan2(math.sin(r(cam[i_n])) - math.sin(r(cam[i_p])), math.sin(r(t_p)) - math.sin(r(t_n))))


def mathworks_formulas(name, kpi_deg: float | None = None) -> dict | None:
    """MathWorks' estimates (sm_car_knc_calc_susp_metrics) computed on KinematiK's motion at the rig's own rack samples,
    instead of on the rig's: caster by the swing formula at +-1 deg of toe; the steering axis at wheel-centre height
    from a Pratt circle fit of the wheel centre's xy path within +-10 mm in x; trail as MathWorks writes it,
    hubLong + tan(caster) h, and with the sign the geometry gives, -hubLong + tan(caster) h (x forward, caster top
    rearward, trail + with the axis ahead of the contact patch); scrub as |hubLat| - tan(KPI) h with the KPI given."""
    ref = mathworks(name)
    if _rig_steer(name) is None or ref is None:
        return None
    m, k = _rig_steer(name)
    h = ref.get("tire_radius_mm", 233.23)
    x0, y0 = k[int(np.argmin(np.abs(m["rack_mm"]))), 1:3]
    use = np.abs(k[:, 1] - x0) <= 10.0
    cx, cy = _pratt(k[use, 1:3])
    caster = _swing_caster(k[:, 4], k[:, 5], 1.0)
    hub_long, hub_lat = float(x0 - cx), float(y0 - cy)
    t = math.tan(math.radians(caster)) * h
    out = {"Caster": caster, "Hub Long Offset": hub_long, "Hub Lat Offset": hub_lat, "Caster Trail": hub_long + t,
           "trail_geometric": -hub_long + t}
    if kpi_deg is not None:
        out["scrub_from_kpi"] = abs(hub_lat) - math.tan(math.radians(kpi_deg)) * h
    return out



def record() -> dict:
    """Every comparison and both diagnostics, as benchmarks/knc_simscape/results.json."""
    res = {}
    for n in CORNERS:
        r = compare(n)
        if r is None:
            continue
        sm, f = steer_motion(n), mathworks_formulas(n, r["rows"]["KPI"]["kinematik"])
        res[n] = {"passed": r["passed"], "rows": r["rows"], "steering_motion_worst": sm, "mathworks_formulas_on_kinematik": f}
    (BENCH / "results.json").write_text(json.dumps(res, indent=1) + "\n")
    return res


def main():
    if sys.argv[1:2] == ["record"]:
        record()
        print(f"wrote {BENCH / 'results.json'}")
        return
    for n in CORNERS:
        r = compare(n)
        if r is None:
            print(f"{n}: no MathWorks results yet in {BENCH.relative_to(ROOT)}")
            continue
        print(f"{n}: {'PASS' if r['passed'] else 'FAIL'}")
        for q, x in r["rows"].items():
            mwv = f"{x['mathworks']:.4f}" if "mathworks" in x else "curve"
            kv = f"{x['kinematik']:.4f}" if "kinematik" in x else ""
            print(f"  {q:13s} MathWorks {mwv:>10s}  KinematiK {kv:>10s}  diff {x['diff']:.4f} (limit {x['limit']})  "
                  f"{'PASS' if x['passed'] else 'FAIL'}")
        steer_extract(n)
        sm = steer_motion(n)
        if sm:
            print(f"  steering test, rig vs KinematiK at the same rack (to {sm['max_toe_deg']:.1f} deg of toe), worst: "
                  f"toe {sm['toe_in']:.5f} deg, camber {sm['camber']:.5f} deg (front-view form: "
                  f"{sm['camber_front_view']:.5f}), wheel centre x {sm['wc_x']:.4f} y {sm['wc_y']:.4f} mm")
        mw = mathworks(n)
        f = mathworks_formulas(n, r["rows"]["KPI"]["kinematik"])
        if f:
            print("  MathWorks' formulas on KinematiK's motion: " + ", ".join(
                f"{q} {f[q]:.4f} (rig {mw[q]:.4f})" for q in ("Caster", "Hub Long Offset", "Hub Lat Offset", "Caster Trail")
                if q in mw) + f"; trail with the geometric sign {f['trail_geometric']:.4f}; scrub by MathWorks' formula "
                f"with KinematiK's KPI {f['scrub_from_kpi']:.4f}")


if __name__ == "__main__":
    main()
