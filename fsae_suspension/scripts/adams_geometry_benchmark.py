"""KinematiK against Adams View: anti-dive / anti-squat path, scrub radius, mechanical trail and the rocker.

The kinematics benchmark (adams_view_benchmark.py) compares camber, toe, caster, kingpin inclination and the wheel-centre
path. This one adds what the reports quote and that benchmark does not reach, on the same two paper corners
(front_v11_final, rear_v11_final), the same jack (wheel-centre travel -25 to +25 mm in 50 steps) and the same joints:

  * the contact patch's side-view path (cp_dx against cp_dz), which anti-dive and anti-squat are computed from;
  * scrub radius and mechanical trail: where the kingpin axis (lower to upper ball joint) meets the road plane through
    the contact patch, laterally (scrub) and longitudinally (trail) from the contact patch;
  * the rocker: a pushrod (spherical at its outer end, Hooke at the rocker so it cannot spin), a rocker on a revolute
    about its declared axis, and the damper length from the rocker's spring point to the chassis spring mount.

Adams writes one Spread Sheet file per measure (<corner>_geo__<measure>.tab, File > Export > Spread Sheet), which the
command file does itself at the end of the run. The limits below were set before any Adams result existed.
Frame: x rearward, y outboard (right-hand corner), z up from the ground; mm and degrees.

    python scripts/adams_geometry_benchmark.py --write   # the command files, benchmarks/adams_geometry/<corner>_geo.cmd
    python scripts/adams_geometry_benchmark.py           # compare, exit 1 on any failure (skips corners not yet run)
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
import adams_view_benchmark as avb                                                    # noqa: E402
from suspension.genesis_repro import hp_from_dict                                     # noqa: E402
from suspension.kinematics import SuspensionKinematics                                # noqa: E402

BENCH = ROOT / "benchmarks" / "adams_geometry"
CORNERS = ("front_v11_final", "rear_v11_final")
TRAVEL = avb.TRAVEL
MEASURES = ("travel", "cp_dx", "cp_dz", "scrub", "trail", "damper_len")
#: Set before any Adams result: path and lengths to 0.05 mm (as the wheel-centre path), the derived anti-geometry to
#: 0.1 percentage point at ride height.
LIMITS = {"cp_dx": 0.05, "cp_dz": 0.05, "scrub": 0.05, "trail": 0.05, "damper_len": 0.05, "anti_pct": 0.1}
UNITS = {"cp_dx": "mm", "cp_dz": "mm", "scrub": "mm", "trail": "mm", "damper_len": "mm", "anti_pct": "%"}
#: The paper's car, for anti-dive (front, outboard brakes, contact-patch path) and anti-squat (rear, inboard drive,
#: wheel-centre path is not used here: both are taken from the contact patch so one path serves both corners).
CAR = {"cg_height_mm": 280.0, "wheelbase_mm": 1630.0, "brake_bias_front": 0.73}


def _hp(name: str) -> tuple[dict, str]:
    return avb._winner(name)


def _road_point(lo, uo, cp):
    """Where the kingpin axis (lo -> uo) meets the horizontal road plane through the contact patch (mm)."""
    kp = np.asarray(uo, float) - np.asarray(lo, float)
    t = (cp[2] - lo[2]) / kp[2]
    return np.asarray(lo, float) + t * kp


def kinematik_curves(name: str) -> dict[str, np.ndarray]:
    """KinematiK's side of every measure at TRAVEL (mm of wheel-centre travel): a warm-started sweep of 281 states over
    -35..+35 mm of lower-ball-joint travel, interpolated, as in the kinematics benchmark."""
    w, _ = _hp(name)
    hp = hp_from_dict(w)
    kin = SuspensionKinematics(hp)
    st = kin.sweep(-35, 35, 281)
    if not all(s.converged for s in st):
        raise RuntimeError(f"{name}: KinematiK sweep did not converge")
    dz = np.array([s.wheel_center[2] for s in st]) - hp.wheel_center[2]
    cp = np.array([s.contact_patch for s in st])
    road = np.array([_road_point(s.lower_outer, s.upper_outer, s.contact_patch) for s in st])
    lens, seed = [], 0.0
    for s in st:
        L, seed, ok = kin.spring_length_at(s, seed=seed)
        if not ok:
            raise RuntimeError(f"{name}: rocker did not solve at {s.wheel_center[2]:.1f} mm")
        lens.append(L)
    cols = {"cp_dx": cp[:, 0] - hp.contact_patch[0], "cp_dz": cp[:, 2] - hp.contact_patch[2],
            "scrub": np.array([s.scrub_radius for s in st]),
            "trail": cp[:, 0] - road[:, 0],
            "damper_len": np.array(lens)}
    out = {k: np.interp(TRAVEL, dz, v) for k, v in cols.items()}
    out["damper_len"] = out["damper_len"] - out["damper_len"][TRAVEL == 0.0][0]
    out["anti_pct"] = np.array([kin.anti_dive_pct(CAR["cg_height_mm"], CAR["wheelbase_mm"],
                                                  CAR["brake_bias_front"])])
    return out


def anti_from_path(cp_dx, cp_dz) -> float:
    """Anti-dive (%) from a contact-patch path, as KinematiK defines it: tan(phi) = -dx/dz at ride height."""
    i = int(np.argmin(np.abs(TRAVEL)))
    slope = (cp_dx[i + 1] - cp_dx[i - 1]) / (cp_dz[i + 1] - cp_dz[i - 1])
    return float(-slope * CAR["wheelbase_mm"] / CAR["cg_height_mm"] * CAR["brake_bias_front"] * 100.0)


# ---------------------------------------------------------------- command file
def command_file(name: str) -> str:
    """The Adams View command file: the kinematics benchmark's corner plus the contact patch, the rocker and the
    damper, with every measure exported to <corner>_geo__<measure>.tab when the run ends."""
    w, sha = _hp(name)
    P = {k: np.asarray(w[k], float) for k in ("contact_patch", "pushrod_outer", "rocker_pivot", "rocker_axis",
                                              "rocker_pushrod", "rocker_spring", "spring_inner")}
    carrier = "upright" if w.get("pushrod_attach", "upright") == "upright" else "lca"
    r = P["rocker_pushrod"] - P["pushrod_outer"]
    r /= np.linalg.norm(r)
    a = np.cross(r, [0, 0, 1.0])
    a /= np.linalg.norm(a)
    b = np.cross(r, a)
    o_pi = avb._euler_from(np.column_stack([a, np.cross(r, a), r]))          # Hooke: pins normal to the pushrod
    o_rj = avb._euler_from(np.column_stack([np.cross(b, r), b, r]))
    o_piv = avb._euler313(P["rocker_axis"])
    G, U, CP, O = "{M}.upright.u_lbj", "{M}.upright.u_ubj", "{M}.upright.u_cp", "{M}.ground.g_o"
    t = f"((DZ({CP},{O})-DZ({G},{O}))/DZ({U},{G}))"
    measures = [("cp_dx", f"DX({CP},{{M}}.ground.g_cp0)"), ("cp_dz", f"DZ({CP},{{M}}.ground.g_cp0)"),
                ("scrub", f"DY({CP},{O})-(DY({G},{O})+{t}*DY({U},{G}))"),
                ("trail", f"DX({CP},{O})-(DX({G},{O})+{t}*DX({U},{G}))"),
                ("damper_len", "DM({M}.rocker.r_spr,{M}.ground.g_spr)")]
    markers = [("upright", "u_cp", P["contact_patch"]), ("ground", "g_cp0", P["contact_patch"]),
               ("ground", "g_o", np.zeros(3)),
               (carrier, "c_pr", P["pushrod_outer"]), ("pushrod", "pr_out", P["pushrod_outer"]),
               ("pushrod", "pr_in", P["rocker_pushrod"], o_pi), ("rocker", "r_pr", P["rocker_pushrod"], o_rj),
               ("rocker", "r_piv", P["rocker_pivot"], o_piv), ("ground", "g_piv", P["rocker_pivot"], o_piv),
               ("rocker", "r_spr", P["rocker_spring"]), ("ground", "g_spr", P["spring_inner"])]
    cons = [f"constraint create joint spherical joint_name={{M}}.pr_sph i_marker_name={{M}}.pushrod.pr_out "
            f"j_marker_name={{M}}.{carrier}.c_pr",
            "constraint create joint hooke joint_name={M}.pr_hk i_marker_name={M}.pushrod.pr_in "
            "j_marker_name={M}.rocker.r_pr",
            "constraint create joint revolute joint_name={M}.rk_rev i_marker_name={M}.rocker.r_piv "
            "j_marker_name={M}.ground.g_piv"]
    body = avb.command_file(f"{name}_geo", w=w, sha=sha, extra_markers=markers, extra_measures=measures,
                            extra_parts=("pushrod", "rocker"), extra_constraints=cons)
    head = (f"! KinematiK corner {name}: contact-patch path, scrub, trail and the rocker's damper length over wheel "
            f"travel -25 to +25 mm. Pushrod on the {carrier}: spherical there, Hooke at the rocker; rocker on a "
            f"revolute about its axis. Writes {name}_geo__<measure>.tab for {', '.join(MEASURES)}.\n")
    tail = "\n".join(avb.export_commands(f"{name}_geo_v4", MEASURES, f"{name}_geo")) + "\n"
    return head + body + tail


def adams_curves(name: str) -> dict[str, np.ndarray] | None:
    """Adams' side, from the exported Spread Sheet files, at TRAVEL; None until the corner has been run."""
    files = {m: BENCH / f"{name}_geo__{m}.tab" for m in MEASURES}
    table = BENCH / f"{name}_geo.tab"            # or one PostProcessor table (File > Export > Table) of all six
    raw = {}
    if all(f.exists() for f in files.values()):
        for m, f in files.items():
            model, _rs, _t, v = avb.read_spreadsheet(f)
            if not model.startswith(f"{name}_geo"):
                raise ValueError(f"{f.name} was exported from model {model!r}")
            raw[m] = v
    elif table.exists():
        model, cols = avb.read_table(table)
        if not model.startswith(f"{name}_geo"):
            raise ValueError(f"{table.name} was exported from model {model!r}")
        missing = [m for m in MEASURES if m not in cols]
        if missing:
            raise ValueError(f"{table.name} lacks {', '.join(missing)}")
        raw = {m: cols[m] for m in MEASURES}
    else:
        return None
    trav = raw.pop("travel")
    out = {m: np.interp(TRAVEL, trav, v) for m, v in raw.items()}
    out["damper_len"] = out["damper_len"] - out["damper_len"][TRAVEL == 0.0][0]
    out["anti_pct"] = np.array([anti_from_path(out["cp_dx"], out["cp_dz"])])
    return out


def chrono_curves(name: str) -> dict[str, np.ndarray] | None:
    """Project Chrono's side (scripts/chrono_geometry_benchmark.py, chrono_geometry.csv) at TRAVEL; None if not run."""
    f = BENCH / "chrono_geometry.csv"
    if not f.exists():
        return None
    rows = [ln.split(",") for ln in f.read_text().splitlines()[1:] if ln.startswith(name + ",")]
    if not rows:
        return None
    a = np.array([[float(x) for x in r[1:]] for r in rows])
    if not np.allclose(a[:, 0], TRAVEL):
        raise ValueError(f"chrono_geometry.csv does not hold {name} at -25..+25 mm in 1 mm steps")
    out = dict(zip(("cp_dx", "cp_dz", "scrub", "trail", "damper_len"), a[:, 1:].T))
    out["anti_pct"] = np.array([anti_from_path(out["cp_dx"], out["cp_dz"])])
    return out


def compare(name: str, tool: str = "adams") -> dict | None:
    adams = adams_curves(name) if tool == "adams" else chrono_curves(name)
    if adams is None:
        return None
    kk = kinematik_curves(name)
    worst = {q: float(np.max(np.abs(adams[q] - kk[q]))) for q in LIMITS}
    return {"corner": name, "tool": tool, "worst": worst, "passed": all(worst[q] <= LIMITS[q] for q in LIMITS)}


def main(argv: list[str]) -> int:
    if "--write" in argv:
        BENCH.mkdir(parents=True, exist_ok=True)
        for n in CORNERS:
            (BENCH / f"{n}_geo.cmd").write_text(command_file(n))
            print(f"wrote benchmarks/adams_geometry/{n}_geo.cmd")
        return 0
    ok, ran = True, 0
    qs = list(LIMITS)
    print(f"{'corner':28s}" + "".join(f"{q + ' (' + UNITS[q] + ')':>18s}" for q in qs))
    print(f"{'limit':28s}" + "".join(f"{LIMITS[q]:18.3f}" for q in qs))
    for tool in ("adams", "chrono"):
        for n in CORNERS:
            r = compare(n, tool)
            label = f"{n} {tool}"
            if r is None:
                print(f"{label:28s}  not run yet")
                continue
            ran += 1
            ok &= r["passed"]
            print(f"{label:28s}" + "".join(f"{r['worst'][q]:18.6f}" for q in qs)
                  + ("  PASS" if r["passed"] else "  FAIL"))
    return 0 if ok and ran else (1 if ran else 0)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
