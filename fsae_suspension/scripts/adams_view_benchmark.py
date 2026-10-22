# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""KinematiK against Adams View on the three corners of the paper (front_v11_final, rear_v11_final, front_v11_joints50).

write():   the Adams View command file for a corner, built from its manifest's recorded hardpoints. Rigid lower and
           upper wishbones on revolute inner pivots, spherical ball joints, a tie rod with a spherical joint at the upright
           and a Hooke joint at the fixed rack end, and a jack that drives wheel-centre height from -25 to +25 mm in 50
           steps through an inplane primitive (wheel centre free in x and y). Kinematic analysis, no masses.
compare(): reads the table Adams View exported (PostProcessor, File > Export > Table, spreadsheet) and compares it with
           KinematiK's sweep of the same manifest at every 1 mm of wheel-centre travel.

The limits were set before any Adams result existed. Adams reports toe as + toe-in; KinematiK's own output is + toe-out
and is flipped here. Frame: x rearward, y outboard (right-hand corner), z up from the ground; mm and degrees.

    python scripts/adams_view_benchmark.py            # compare all three corners, exit 1 on any failure
    python scripts/adams_view_benchmark.py --write    # regenerate the command files from the manifests
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension.genesis_repro import hp_from_dict                                     # noqa: E402
from suspension.kinematics import SuspensionKinematics                                # noqa: E402

BENCH = ROOT / "benchmarks" / "adams_view"
MAN = ROOT / "manifests"
CORNERS = ("front_v11_final", "rear_v11_final", "front_v11_joints50")
TRAVEL = np.arange(-25.0, 26.0, 1.0)
QUANTITIES = ("camber", "toe_in", "caster", "kpi", "wc_dx", "wc_dy")
LIMITS = {"camber": 0.01, "toe_in": 0.005, "caster": 0.01, "kpi": 0.01, "wc_dx": 0.05, "wc_dy": 0.05}
UNITS = {"camber": "deg", "toe_in": "deg", "caster": "deg", "kpi": "deg", "wc_dx": "mm", "wc_dy": "mm"}
POINTS = ("upper_front_inner", "upper_rear_inner", "upper_outer", "lower_front_inner", "lower_rear_inner",
          "lower_outer", "tie_rod_inner", "tie_rod_outer", "wheel_center")


def _winner(name: str) -> tuple[dict, str]:
    d = json.loads((MAN / f"{name}.recorded.genesis.json").read_text())
    return d["recorded"]["winner_hardpoints"], d["inputs_sha256"]


def kinematik_curves(name: str) -> dict[str, np.ndarray]:
    """KinematiK on wheel-centre travel -25..+25 mm: a warm-started sweep of 281 states over -35..+35 mm of
    lower-ball-joint travel, interpolated onto wheel-centre travel."""
    w, _ = _winner(name)
    hp = hp_from_dict(w)
    st = SuspensionKinematics(hp).sweep(-35, 35, 281)
    if not all(s.converged for s in st):
        raise RuntimeError(f"{name}: KinematiK sweep did not converge")
    wc = np.array([s.wheel_center for s in st])
    dz = wc[:, 2] - hp.wheel_center[2]
    cols = {"camber": np.array([s.camber for s in st]),
            "toe_in": -np.array([s.toe for s in st]),
            "caster": np.array([s.caster for s in st]),
            "kpi": np.array([s.kpi for s in st]),
            "wc_dx": wc[:, 0] - hp.wheel_center[0],
            "wc_dy": wc[:, 1] - hp.wheel_center[1]}
    return {k: np.interp(TRAVEL, dz, v) for k, v in cols.items()}


def read_table(path: Path) -> tuple[str, dict[str, np.ndarray]]:
    """An Adams View PostProcessor table export: model name on line 1, quoted headers on line 3, then rows."""
    lines = [ln for ln in path.read_text().splitlines()]
    model = lines[0].strip().strip('"')
    heads = [h.strip().strip('"').strip() for h in lines[2].split("\t")]
    rows = [[float(x) for x in ln.split("\t")] for ln in lines[3:] if ln.strip()]
    data = np.array(rows)
    return model, {h: data[:, i] for i, h in enumerate(heads)}


def compare(name: str) -> dict:
    model, adams = read_table(BENCH / f"{name}.tab")
    if not model.startswith(name):
        raise ValueError(f"{name}.tab was exported from model {model!r}, not from {name}")
    if not np.allclose(adams["travel"], TRAVEL, atol=1e-6):
        raise ValueError(f"{name}.tab does not hold wheel-centre travel -25..+25 mm in 1 mm steps")
    kk = kinematik_curves(name)
    worst = {q: float(np.max(np.abs(adams[q] - kk[q]))) for q in QUANTITIES}
    return {"corner": name, "model": model, "worst": worst,
            "passed": all(worst[q] <= LIMITS[q] for q in QUANTITIES)}


# ---------------------------------------------------------------- command file
def _euler313(z) -> tuple[float, float, float]:
    """Adams body-313 angles (deg) whose z axis points along z."""
    z = np.asarray(z, float) / np.linalg.norm(z)
    th = math.degrees(math.acos(max(-1.0, min(1.0, z[2]))))
    ps = math.degrees(math.atan2(z[0], -z[1])) if abs(math.sin(math.radians(th))) > 1e-12 else 0.0
    return ps, th, 0.0


def _euler_from(R) -> tuple[float, float, float]:
    th = math.degrees(math.acos(max(-1.0, min(1.0, R[2, 2]))))
    return math.degrees(math.atan2(R[0, 2], -R[1, 2])), th, math.degrees(math.atan2(R[2, 0], R[2, 1]))


def _rot313(ps, th, ph) -> np.ndarray:
    p, t, q = (math.radians(a) for a in (ps, th, ph))
    Rz = lambda a: np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])  # noqa: E731
    Rx = lambda a: np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])  # noqa: E731
    return Rz(p) @ Rx(t) @ Rz(q)


def _f(v) -> str:
    return ", ".join(f"{c:.4f}" for c in v)


def command_file(name: str) -> str:
    w, sha = _winner(name)
    P = {k: np.array(w[k], float) for k in POINTS}
    cam, toe = math.radians(w.get("static_camber", 0.0)), math.radians(w.get("static_toe", 0.0))
    spin = np.array([math.sin(toe), math.cos(toe) * math.cos(cam), -math.sin(cam)])  # static spin axis, outboard
    tip = P["wheel_center"] + 100.0 * spin / np.linalg.norm(spin)
    o_lca = _euler313(P["lower_rear_inner"] - P["lower_front_inner"])
    o_uca = _euler313(P["upper_rear_inner"] - P["upper_front_inner"])
    r = P["tie_rod_outer"] - P["tie_rod_inner"]
    r /= np.linalg.norm(r)
    a = np.cross(r, [0, 0, 1.0])
    a /= np.linalg.norm(a)
    b = np.cross(r, a)
    # Hooke joint: the cross pins are the I marker's x axis and the J marker's y axis, both normal to the rod
    RI, RJ = np.column_stack([a, np.cross(r, a), r]), np.column_stack([np.cross(b, r), b, r])
    o_ui, o_uj = _euler_from(RI), _euler_from(RJ)
    assert np.allclose(_rot313(*o_ui), RI, atol=1e-9) and np.allclose(_rot313(*o_uj), RJ, atol=1e-9)
    m = f"{name}_v4"
    M = "." + m
    L = [f"! KinematiK corner {name} (inputs SHA-256 {sha[:12]}) for Adams View: kinematic benchmark, "
         "parallel wheel travel -25 to +25 mm.",
         "! No masses: a kinematic sweep does not use them.",
         "! Frame: x rearward, y outboard (right-hand corner), z up from the ground; mm and degrees.",
         "! Tie rod: spherical at the upright, Hooke (universal) joint at the fixed rack end (locks its spin). The jack "
         "drives wheel-centre height through an inplane primitive, leaving the wheel centre free in x and y.",
         "default units length=mm mass=kg force=newton time=second angle=degrees",
         f"model create model_name={m}"]
    for part in ("lca", "uca", "upright", "tierod", "jack"):
        L.append(f"part create rigid_body name_and_position part_name={M}.{part} "
                 "location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0")

    def mk(part, mname, loc, ori=(0.0, 0.0, 0.0)):
        L.append(f"marker create marker_name={M}.{part}.{mname} location={_f(loc)} orientation={_f(ori)}")
    mk("ground", "g_lca", P["lower_front_inner"], o_lca); mk("lca", "lca_piv", P["lower_front_inner"], o_lca)  # noqa: E702
    mk("ground", "g_uca", P["upper_front_inner"], o_uca); mk("uca", "uca_piv", P["upper_front_inner"], o_uca)  # noqa: E702
    mk("lca", "lca_out", P["lower_outer"]); mk("upright", "u_lbj", P["lower_outer"])                          # noqa: E702
    mk("uca", "uca_out", P["upper_outer"]); mk("upright", "u_ubj", P["upper_outer"])                          # noqa: E702
    mk("upright", "u_tro", P["tie_rod_outer"]); mk("tierod", "tr_out", P["tie_rod_outer"])                    # noqa: E702
    mk("tierod", "tr_in", P["tie_rod_inner"], o_ui); mk("ground", "g_tri", P["tie_rod_inner"], o_uj)          # noqa: E702
    mk("upright", "u_wc", P["wheel_center"]); mk("upright", "u_tip", tip)                                      # noqa: E702
    mk("ground", "g_wc0", P["wheel_center"]); mk("jack", "j_pad", P["wheel_center"])                           # noqa: E702
    mk("jack", "j_slide", P["wheel_center"]); mk("ground", "g_jack", P["wheel_center"])                        # noqa: E702
    L += [f"constraint create joint revolute joint_name={M}.lca_rev i_marker_name={M}.lca.lca_piv j_marker_name={M}.ground.g_lca",
          f"constraint create joint revolute joint_name={M}.uca_rev i_marker_name={M}.uca.uca_piv j_marker_name={M}.ground.g_uca",
          f"constraint create joint spherical joint_name={M}.lbj i_marker_name={M}.upright.u_lbj j_marker_name={M}.lca.lca_out",
          f"constraint create joint spherical joint_name={M}.ubj i_marker_name={M}.upright.u_ubj j_marker_name={M}.uca.uca_out",
          f"constraint create joint spherical joint_name={M}.tro i_marker_name={M}.upright.u_tro j_marker_name={M}.tierod.tr_out",
          f"constraint create joint hooke joint_name={M}.tri i_marker_name={M}.tierod.tr_in j_marker_name={M}.ground.g_tri",
          f"constraint create joint translational joint_name={M}.jack_trans i_marker_name={M}.jack.j_slide "
          f"j_marker_name={M}.ground.g_jack",
          f"constraint create primitive_joint inplane jprim_name={M}.jack_pad i_marker_name={M}.upright.u_wc "
          f"j_marker_name={M}.jack.j_pad",
          f"constraint create motion_generator motion_name={M}.jack_motion joint_name={M}.jack_trans "
          'time_derivative=displacement function="50.0*time - 25.0"']
    T, W, U, G = f"{M}.upright.u_tip", f"{M}.upright.u_wc", f"{M}.upright.u_ubj", f"{M}.upright.u_lbj"
    for mname, fn in (("travel", f"DZ({W},{M}.ground.g_wc0)"),
                      ("camber", f"-RTOD*ATAN(DZ({T},{W})/DY({T},{W}))"),
                      ("toe_in", f"-RTOD*ATAN(DX({T},{W})/DY({T},{W}))"),
                      ("caster", f"RTOD*ATAN(DX({U},{G})/DZ({U},{G}))"),
                      ("kpi", f"RTOD*ATAN(-DY({U},{G})/DZ({U},{G}))"),
                      ("wc_dx", f"DX({W},{M}.ground.g_wc0)"),
                      ("wc_dy", f"DY({W},{M}.ground.g_wc0)")):
        L.append(f'measure create function measure_name={M}.{mname} function="{fn}"')
    L += [f"default model model_name={M}", "simulation single_run reset",
          f"simulation single_run transient type=kinematic end_time=1.0 number_of_steps=50 model_name={M} "
          "initial_static=no"]
    return "\n".join(L) + "\n"


def main(argv: list[str]) -> int:
    if "--write" in argv:
        for n in CORNERS:
            (BENCH / f"{n}.cmd").write_text(command_file(n))
            print(f"wrote benchmarks/adams_view/{n}.cmd")
        return 0
    ok = True
    print(f"{'corner':20s}" + "".join(f"{q + ' (' + UNITS[q] + ')':>16s}" for q in QUANTITIES))
    print(f"{'limit':20s}" + "".join(f"{LIMITS[q]:16.3f}" for q in QUANTITIES))
    for n in CORNERS:
        r = compare(n)
        ok &= r["passed"]
        print(f"{n:20s}" + "".join(f"{r['worst'][q]:16.6f}" for q in QUANTITIES) + ("  PASS" if r["passed"] else "  FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
