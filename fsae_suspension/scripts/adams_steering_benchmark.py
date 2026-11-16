"""KinematiK against Adams View and Project Chrono: road-wheel steer over rack travel, lock and Ackermann.

The kinematics benchmark sweeps wheel travel with the rack fixed. This one holds the corner at ride height (the lower
wishbone locked at its static angle, as KinematiK's solve_at_travel(0) does) and drives the tie-rod inner end with a
rack on a translational joint along y, from -30 to +30 mm in 0.25 mm steps. Measured: toe (Adams' + toe-in) and camber
of the steered wheel. From the toe curve, read the way KinematiK reads its own (a mirrored axle: the left wheel is this
corner at the opposite rack), the steering-wheel angle that reaches the declared lock and the Ackermann at it.

Corners: front_v11_final and front_v11_joints50, with the rack c-factor, lock, lock basis, wheelbase and track their
manifests declare. Limits, set before any Adams or Chrono result: toe and camber to 0.005 deg (as the kinematics
benchmark), steering-wheel angle at lock to 0.1 deg, Ackermann at lock to 0.1 percentage point.

    python scripts/adams_steering_benchmark.py --write   # benchmarks/adams_steering/<corner>_steer.cmd
    python scripts/adams_steering_benchmark.py           # compare (Adams tables and chrono_steering.csv), exit 1 on fail
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
import adams_view_benchmark as avb                                                    # noqa: E402
from suspension.dynamic_ackermann import ackermann_pct, lock_report                   # noqa: E402
from suspension.genesis_repro import GenesisManifest, hp_from_dict                    # noqa: E402
from suspension.kinematics import SuspensionKinematics                                # noqa: E402
from suspension.steering_margin import steer_effort                                   # noqa: E402

BENCH = ROOT / "benchmarks" / "adams_steering"
CORNERS = ("front_v11_final", "front_v11_joints50")
RACK = np.round(np.arange(-30.0, 30.0001, 0.25), 6)
MEASURES = ("rack_pos", "toe_in", "camber")  # "rack" is the part: Adams names share one namespace
LIMITS = {"toe_in": 0.005, "camber": 0.005, "lock_sw_deg": 0.1, "ackermann_lock_pct": 0.1}
UNITS = {"toe_in": "deg", "camber": "deg", "lock_sw_deg": "deg", "ackermann_lock_pct": "%"}


def declared(name: str) -> dict:
    """The manifest's rack c-factor (mm/rev), lock (deg), lock basis, wheelbase and kingpin track (mm)."""
    txt = (avb.MAN / f"{name}.recorded.genesis.json").read_text()
    m = GenesisManifest.from_json(txt)
    se = steer_effort(m)
    p = (m.volume or {}).get("properties", {}) if isinstance(m.volume, dict) else {}
    return {"c": float(se.rack_c_mm_per_rev), "lock": float(se.lock_road_wheel_deg), "basis": se.lock_basis,
            "wheelbase": float(p.get("wheelbase_mm", 1630.0)), "track": float(p.get("track_mm", 1210.0))}


def kinematik_curves(name: str) -> dict:
    """KinematiK's toe-in and camber (deg) at every RACK, and its own lock and Ackermann (lock_report)."""
    w, _ = avb._winner(name)
    hp = hp_from_dict(w)
    toe, cam = [], []
    for r in RACK:
        s = SuspensionKinematics(hp, pickup_deltas={"tie_rod_inner": np.array([0.0, float(r), 0.0])}) \
            .solve_at_travel(0.0)
        toe.append(-s.toe)
        cam.append(s.camber)
    d = declared(name)
    lr = lock_report(hp, d["c"], d["lock"], wheelbase_mm=d["wheelbase"], kingpin_track_mm=d["track"],
                     lock_basis=d["basis"])
    return {"toe_in": np.array(toe), "camber": np.array(cam), "lock_sw_deg": lr["wheel_lock_deg"],
            "ackermann_lock_pct": lr["ackermann_pct_lo_and_lock"][1]}


def lock_from_curve(name: str, toe_in) -> tuple[float, float]:
    """Steering-wheel angle at lock (deg) and Ackermann there (%), read off a toe-in curve over RACK the way
    dynamic_ackermann.lock_report reads the linkage: right wheel at +rack, left wheel = this corner at -rack."""
    d = declared(name)
    toe = -np.asarray(toe_in, float)                          # + toe-out = right turn on this right corner
    t0 = float(np.interp(0.0, RACK, toe))

    def split(r):
        dR = float(np.interp(r, RACK, toe)) - t0
        dL = -(float(np.interp(-r, RACK, toe)) - t0)
        return (dR, dL) if dR + dL >= 0 else (-dL, -dR)        # (inner, outer)

    def reach(r):
        i, o = split(r)
        return i if d["basis"] == "inner" else 0.5 * (i + o)

    lo, hi = 0.0, float(RACK[-1])
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if reach(mid) < d["lock"] else (lo, mid)
    i, o = split(hi)
    return 360.0 * hi / d["c"], ackermann_pct(i, o, d["wheelbase"], d["track"])


# ---------------------------------------------------------------- command file
def command_file(name: str) -> str:
    """Adams View: the corner at ride height (lower wishbone held at its static angle), steered by a rack."""
    w, sha = avb._winner(name)
    P = {k: np.array(w[k], float) for k in avb.POINTS}
    cam, toe = math.radians(w.get("static_camber", 0.0)), math.radians(w.get("static_toe", 0.0))
    spin = np.array([math.sin(toe), math.cos(toe) * math.cos(cam), -math.sin(cam)])
    tip = P["wheel_center"] + 100.0 * spin / np.linalg.norm(spin)
    o_lca = avb._euler313(P["lower_rear_inner"] - P["lower_front_inner"])
    o_uca = avb._euler313(P["upper_rear_inner"] - P["upper_front_inner"])
    o_rack = avb._euler313([0.0, 1.0, 0.0])                   # translational joint slides along its markers' z
    r = P["tie_rod_outer"] - P["tie_rod_inner"]
    r /= np.linalg.norm(r)
    a = np.cross(r, [0, 0, 1.0])
    a /= np.linalg.norm(a)
    b = np.cross(r, a)
    o_ui = avb._euler_from(np.column_stack([a, np.cross(r, a), r]))
    o_uj = avb._euler_from(np.column_stack([np.cross(b, r), b, r]))
    m = f"{name}_steer_v3"
    M = "." + m
    n = len(RACK) - 1
    L = [f"! KinematiK corner {name} (inputs SHA-256 {sha[:12]}) for Adams View: steering benchmark, rack -30 to +30 mm "
         f"in {n} steps at ride height (lower wishbone held at its static angle). Writes {name}_steer__<measure>.tab "
         f"for {', '.join(MEASURES)}.",
         "! Frame: x rearward, y outboard (right-hand corner), z up from the ground; mm and degrees.",
         "default units length=mm mass=kg force=newton time=second angle=degrees",
         f"model create model_name={m}"]
    for part in ("lca", "uca", "upright", "tierod", "rack"):
        L.append(f"part create rigid_body name_and_position part_name={M}.{part} "
                 "location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0")

    def mk(part, mname, loc, ori=(0.0, 0.0, 0.0)):
        L.append(f"marker create marker_name={M}.{part}.{mname} location={avb._f(loc)} orientation={avb._f(ori)}")
    mk("ground", "g_lca", P["lower_front_inner"], o_lca); mk("lca", "lca_piv", P["lower_front_inner"], o_lca)  # noqa: E702
    mk("ground", "g_uca", P["upper_front_inner"], o_uca); mk("uca", "uca_piv", P["upper_front_inner"], o_uca)  # noqa: E702
    mk("lca", "lca_out", P["lower_outer"]); mk("upright", "u_lbj", P["lower_outer"])                          # noqa: E702
    mk("uca", "uca_out", P["upper_outer"]); mk("upright", "u_ubj", P["upper_outer"])                          # noqa: E702
    mk("upright", "u_tro", P["tie_rod_outer"]); mk("tierod", "tr_out", P["tie_rod_outer"])                    # noqa: E702
    mk("tierod", "tr_in", P["tie_rod_inner"], o_ui); mk("rack", "r_tri", P["tie_rod_inner"], o_uj)            # noqa: E702
    mk("rack", "r_sl", P["tie_rod_inner"], o_rack); mk("ground", "g_sl", P["tie_rod_inner"], o_rack)          # noqa: E702
    mk("upright", "u_wc", P["wheel_center"]); mk("upright", "u_tip", tip)                                      # noqa: E702
    L += [f"constraint create joint revolute joint_name={M}.lca_rev i_marker_name={M}.lca.lca_piv j_marker_name={M}.ground.g_lca",
          f"constraint create joint revolute joint_name={M}.uca_rev i_marker_name={M}.uca.uca_piv j_marker_name={M}.ground.g_uca",
          f"constraint create joint spherical joint_name={M}.lbj i_marker_name={M}.upright.u_lbj j_marker_name={M}.lca.lca_out",
          f"constraint create joint spherical joint_name={M}.ubj i_marker_name={M}.upright.u_ubj j_marker_name={M}.uca.uca_out",
          f"constraint create joint spherical joint_name={M}.tro i_marker_name={M}.upright.u_tro j_marker_name={M}.tierod.tr_out",
          f"constraint create joint hooke joint_name={M}.tri i_marker_name={M}.tierod.tr_in j_marker_name={M}.rack.r_tri",
          f"constraint create joint translational joint_name={M}.rack_trans i_marker_name={M}.rack.r_sl "
          f"j_marker_name={M}.ground.g_sl",
          f"constraint create motion_generator motion_name={M}.lca_hold joint_name={M}.lca_rev "
          'time_derivative=displacement function="0.0"',
          f"constraint create motion_generator motion_name={M}.rack_motion joint_name={M}.rack_trans "
          'time_derivative=displacement function="60.0*time - 30.0"']
    T, W = f"{M}.upright.u_tip", f"{M}.upright.u_wc"
    for mname, fn in (("rack_pos", f"DY({M}.rack.r_sl,{M}.ground.g_sl)"),
                      ("toe_in", f"-RTOD*ATAN(DX({T},{W})/DY({T},{W}))"),
                      ("camber", f"-RTOD*ATAN(DZ({T},{W})/DY({T},{W}))")):
        L.append(f'measure create function measure_name={M}.{mname} function="{fn}"')
    L += [f"default model model_name={M}", "simulation single_run reset",
          f"simulation single_run transient type=kinematic end_time=1.0 number_of_steps={n} model_name={M} "
          "initial_static=no"]
    L += avb.export_commands(m, MEASURES, f"{name}_steer")
    return "\n".join(L) + "\n"


# ---------------------------------------------------------------- results
def adams_curves(name: str) -> dict | None:
    files = {q: BENCH / f"{name}_steer__{q}.tab" for q in ("toe_in", "camber")}
    if not all(f.exists() for f in files.values()):
        return None
    raw = {}
    for q, f in files.items():
        model, _rs, t, v = avb.read_spreadsheet(f)
        if not model.startswith(f"{name}_steer"):
            raise ValueError(f"{f.name} was exported from model {model!r}")
        raw[q] = v
    pos = BENCH / f"{name}_steer__rack_pos.tab"
    # the rack is driven at 60*time - 30 mm; its own measure, when exported, is used to check that
    rack = avb.read_spreadsheet(pos)[3] if pos.exists() else 60.0 * np.asarray(t, float) - 30.0
    return {q: np.interp(RACK, rack, v) for q, v in raw.items()}


def chrono_curves(name: str) -> dict | None:
    f = BENCH / "chrono_steering.csv"
    if not f.exists():
        return None
    rows = [ln.split(",") for ln in f.read_text().splitlines()[1:] if ln.startswith(name + ",")]
    if not rows:
        return None
    a = np.array([[float(x) for x in r[1:]] for r in rows])
    if not np.allclose(a[:, 0], RACK):
        raise ValueError(f"chrono_steering.csv does not hold {name} at the benchmark's rack stations")
    return {"toe_in": a[:, 1], "camber": a[:, 2]}


def compare(name: str, tool: str = "adams") -> dict | None:
    other = adams_curves(name) if tool == "adams" else chrono_curves(name)
    if other is None:
        return None
    kk = kinematik_curves(name)
    sw, ack = lock_from_curve(name, other["toe_in"])
    worst = {"toe_in": float(np.max(np.abs(other["toe_in"] - kk["toe_in"]))),
             "camber": float(np.max(np.abs(other["camber"] - kk["camber"]))),
             "lock_sw_deg": abs(sw - kk["lock_sw_deg"]), "ackermann_lock_pct": abs(ack - kk["ackermann_lock_pct"])}
    return {"corner": name, "tool": tool, "worst": worst, "passed": all(worst[q] <= LIMITS[q] for q in LIMITS),
            "lock_sw_deg": sw, "ackermann_lock_pct": ack}


def main(argv: list[str]) -> int:
    if "--write" in argv:
        BENCH.mkdir(parents=True, exist_ok=True)
        for n in CORNERS:
            (BENCH / f"{n}_steer.cmd").write_text(command_file(n))
            print(f"wrote benchmarks/adams_steering/{n}_steer.cmd")
        return 0
    ok, ran = True, 0
    print(f"{'corner':30s}" + "".join(f"{q + ' (' + UNITS[q] + ')':>24s}" for q in LIMITS))
    print(f"{'limit':30s}" + "".join(f"{LIMITS[q]:24.3f}" for q in LIMITS))
    for tool in ("adams", "chrono"):
        for n in CORNERS:
            r = compare(n, tool)
            if r is None:
                print(f"{n + ' ' + tool:30s}  not run yet")
                continue
            ran += 1
            ok &= r["passed"]
            print(f"{n + ' ' + tool:30s}" + "".join(f"{r['worst'][q]:24.6f}" for q in LIMITS)
                  + ("  PASS" if r["passed"] else "  FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
