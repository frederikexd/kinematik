# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Direct-acting dampers (no rocker) against the kinematics benchmark tools.

Corners (benchmarks/direct_acting/corners/*.json, written by --write from front_v11_final):
  direct_upright  damper from the upright's pushrod pickup up to a chassis mount: six two-force members
  direct_lower    damper from the lower wishbone (KinematiK's default placement) to a chassis mount: two-body statics

Quantities, limits registered in benchmarks/direct_acting/limits.json before any run:
  damper   damper length change from static at every 1 mm of wheel-centre travel, -25..+25 mm (the motion ratio)
  loads    axial member forces at the design position under the four link-load cases; on direct_lower only the
           determinate members UF, UR, TR and the damper PR

Tools:
  independent  damper: scripts/independent_kinematics.py's rigid-upright solve (no shared code), the damper end carried
               on the upright or rotated with the lower wishbone about its pivot axis. loads: the displacement-method
               solve of scripts/independent_link_loads.py (direct_upright; direct_lower is NOT RUN there, its two-body
               statics is checked against virtual work in tests/test_direct_acting.py).
  adams_view   adams/<corner>_mr.cmd (kinematic sweep, measure damper_len) and adams/<corner>_loads.cmd (zero-DOF
               static model, measures T_*); results: adams/<corner>_mr.tab and adams/<corner>_loads.tab.
  motionsolve  motionsolve/<corner>_mr.adm/.acf and <corner>_loads.adm/.acf (Adams Solver datasets); results entered
               in motionsolve_damper.csv and motionsolve_loads.csv.
  ard          inputs.csv in; results in ard_damper.csv and ard_loads.csv.

A tool without results is NOT RUN, never passed.

    python scripts/direct_acting_benchmark.py           # compare; exit 1 on any failure
    python scripts/direct_acting_benchmark.py --write   # corners, decks, inputs sheet and templates
"""
from __future__ import annotations

import csv
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
import adams_view_benchmark as avb                                                       # noqa: E402
import independent_kinematics as ik                                                      # noqa: E402
import independent_link_loads as ind                                                     # noqa: E402
import link_load_benchmark as llb                                                        # noqa: E402
from suspension import genesis_analysis as ga, loadpath as lp                            # noqa: E402
from suspension.genesis_repro import hp_from_dict                                        # noqa: E402
from suspension.kinematics import SuspensionKinematics                                   # noqa: E402

BENCH = ROOT / "benchmarks" / "direct_acting"
CORNERS = ("direct_upright", "direct_lower")
REG = json.loads((BENCH / "limits.json").read_text())
LIMITS = REG["limits"]
CASES = llb.CASES
TRAVEL = np.arange(-25.0, 26.0, 1.0)
HAND_TRAVEL = np.arange(-25.0, 26.0, 5.0)
TOOLS = ("independent", "adams_view", "motionsolve", "chrono", "matlab", "ard")
ALL6 = ("UF", "UR", "LF", "LR", "TR", "PR")
LOAD_MEMBERS = {"direct_upright": ALL6, "direct_lower": ("UF", "UR", "TR", "PR")}


# ---------------------------------------------------------------- the corners
def build_corner(name: str) -> dict:
    """The benchmark corner, generated from front_v11_final's recorded hardpoints."""
    w = dict(llb._winner("front_v11_final"))
    for k in ("rocker_pivot", "rocker_axis", "rocker_pushrod", "rocker_spring"):
        w.pop(k, None)
    w["actuation"] = "direct"
    if name == "direct_upright":
        w["pushrod_attach"] = "upright"
        out = np.asarray(w["pushrod_outer"], float)
        inn = out + np.array([0.0, -190.0, 240.0])
    else:
        w["pushrod_attach"] = "lower"
        out, inn = ga.default_direct_damper(hp_from_dict(w))
    w["pushrod_outer"] = [round(float(c), 4) for c in out]
    w["spring_inner"] = [round(float(c), 4) for c in inn]
    return w


def corner(name: str) -> dict:
    return json.loads((BENCH / "corners" / f"{name}.json").read_text())


# ---------------------------------------------------------------- KinematiK
def kinematik_damper(name: str) -> np.ndarray:
    """Damper length change from static (mm) at TRAVEL of the wheel centre: warm-started sweep over -35..+35 mm of
    lower-ball-joint travel, interpolated onto wheel-centre travel (as the Adams kinematics benchmark does)."""
    w = corner(name)
    hp = hp_from_dict(w)
    st = SuspensionKinematics(hp).sweep(-35, 35, 281)
    if not all(s.converged for s in st):
        raise RuntimeError(f"{name}: KinematiK sweep did not converge")
    inn = np.asarray(w["spring_inner"], float)
    L = np.array([np.linalg.norm(inn - np.asarray(s.pushrod_outer)) for s in st])
    dz = np.array([s.wheel_center[2] for s in st]) - hp.wheel_center[2]
    Lw = np.interp(TRAVEL, dz, L)
    return Lw - Lw[TRAVEL == 0.0][0]


def kinematik_loads(name: str) -> dict:
    kin = SuspensionKinematics(hp_from_dict(corner(name)))
    s0 = kin.solve_at_travel(0.0)
    out = {}
    for c, d in CASES.items():
        mf = lp.solve_member_forces(kin, s0, lp.WheelLoad(Fx=d["Fx"], Fy=d["Fy"], Fz=d["Fz"], Mz=d["Mz"],
                                                         Fx_wc=d["Fx_wc"]))
        out[c] = {m: mf.forces[m] for m in LOAD_MEMBERS[name]}
    return out


# ---------------------------------------------------------------- independent
def independent_damper(name: str) -> np.ndarray:
    w = corner(name)
    c = ik.Corner(w)
    wc0, out0 = np.asarray(w["wheel_center"], float), np.asarray(w["pushrod_outer"], float)
    inn = np.asarray(w["spring_inner"], float)
    k = c.lri - c.lfi
    k = k / np.linalg.norm(k)

    def damper_outer(x):
        if w["pushrod_attach"] == "upright":
            return c._move(x, out0)
        lo = c._move(x, c.lo0)                         # lower wishbone: rotate about its pivot axis with the ball joint
        a, b = c.lo0 - c.lfi, lo - c.lfi
        a, b = a - (a @ k) * k, b - (b @ k) * k
        ang = np.arctan2(k @ np.cross(a, b), a @ b)
        return Rotation.from_rotvec(ang * k).apply(out0 - c.lfi) + c.lfi

    trav = np.linspace(-35.0, 35.0, 281)
    order = np.argsort(np.abs(trav), kind="stable")
    xs, prev = {}, {}
    for i in order:
        t = float(trav[i])
        near = None if t == 0.0 else min(prev, key=lambda u: abs(u - t))
        xs[i] = c.solve(t, None if near is None else prev[near])
        prev[t] = xs[i]
    L = np.array([np.linalg.norm(inn - damper_outer(xs[i])) for i in range(len(trav))])
    dz = np.array([c._move(xs[i], wc0)[2] for i in range(len(trav))]) - wc0[2]
    Lw = np.interp(TRAVEL, dz, L)
    return Lw - Lw[TRAVEL == 0.0][0]


def independent_loads(name: str) -> dict | None:
    if name != "direct_upright":
        return None
    w = corner(name)
    return {c: {m: f for m, f in ind.member_forces(w, d).items()} for c, d in CASES.items()}


# ---------------------------------------------------------------- reading results
def _table(path: Path, name: str):
    model, t = avb.read_table(path)
    if not model.startswith(name):
        raise ValueError(f"{path.name} was exported from model {model!r}, not from {name}")
    return t


def _csv_rows(path: Path, name: str):
    if not path.exists():
        return []
    return [r for r in csv.DictReader(ln for ln in path.read_text().splitlines() if not ln.startswith("#"))
            if r["corner"] == name]


def tool_damper(tool: str, name: str):
    """(travel, damper length change from static) or None."""
    if tool == "independent":
        return TRAVEL, independent_damper(name)
    if tool == "adams_view":
        p = BENCH / "adams" / f"{name}_mr.tab"
        if not p.exists():
            return None
        t = _table(p, name)
        trv, L = np.round(t["travel"], 6), t["damper_len"]
    else:
        rows = _csv_rows(BENCH / f"{tool}_damper.csv", name)
        if not rows or any(not r["damper_mm"].strip() for r in rows):
            return None
        trv = np.array([float(r["travel_mm"]) for r in rows])
        L = np.array([float(r["damper_mm"]) for r in rows])
    if not np.any(trv == 0.0):
        raise ValueError(f"{tool} {name}: no row at zero travel")
    return trv, L - L[trv == 0.0][0]


def tool_loads(tool: str, name: str):
    mem = LOAD_MEMBERS[name]
    if tool == "independent":
        return independent_loads(name)
    if tool == "adams_view":
        p = BENCH / "adams" / f"{name}_loads.tab"
        if not p.exists():
            return None
        t = _table(p, name)
        return {c: {m: float(t[f"T_{m}"][int(np.argmin(np.abs(t["Time"] - i)))]) for m in mem}
                for i, c in enumerate(CASES)}
    rows = _csv_rows(BENCH / f"{tool}_loads.csv", name)
    if not rows or any(not r[m].strip() for r in rows for m in mem):
        return None
    return {r["case"]: {m: float(r[m]) for m in mem} for r in rows}


def compare(tool: str, name: str) -> dict:
    lim, out = LIMITS[tool], {"tool": tool, "corner": name}
    d = tool_damper(tool, name)
    if d is None:
        out["damper"] = "NOT RUN"
    else:
        trv, ref = d
        kk = np.interp(trv, TRAVEL, kinematik_damper(name))
        worst = float(np.max(np.abs(kk - ref)))
        out["damper"] = "PASS" if worst <= lim["damper_mm"] else "FAIL"
        out["damper_worst_mm"] = worst
    f = tool_loads(tool, name)
    if f is None:
        out["loads"] = "NOT RUN"
    else:
        kk = kinematik_loads(name)
        ratio, worst, where = 0.0, 0.0, None
        for c in CASES:
            for m in LOAD_MEMBERS[name]:
                e = abs(kk[c][m] - f[c][m])
                r = e / (lim["abs_N"] + lim["rel"] * abs(f[c][m]))
                if r > ratio:
                    ratio, worst, where = r, e, (c, m)
        out.update(loads="PASS" if ratio <= 1.0 else "FAIL", loads_worst_N=worst, loads_ratio=ratio, where=where)
    return out


# ---------------------------------------------------------------- Adams View decks
def mr_command_file(name: str) -> str:
    w = corner(name)
    part = "upright" if w["pushrod_attach"] == "upright" else "lca"
    sha = hashlib.sha256((BENCH / "corners" / f"{name}.json").read_bytes()).hexdigest()
    txt = avb.command_file(f"{name}_mr", w=w, sha=sha,
                           extra_markers=[(part, "dmp_out", w["pushrod_outer"]),
                                          ("ground", "g_dmp", w["spring_inner"])],
                           extra_measures=[("damper_len", f"DM({{M}}.{part}.dmp_out,{{M}}.ground.g_dmp)")])
    head = (f"! KinematiK direct-acting corner {name}: damper length over wheel travel -25 to +25 mm. Damper outer on "
            f"the {part}, chassis mount fixed. Export travel and damper_len as {name}_mr.tab.\n")
    return head + txt


def _links(w: dict, name: str):
    """(member, outer point, inner point, part the outer end is on)."""
    on = "upright" if w["pushrod_attach"] == "upright" else "lca"
    base = [(m, w[o], w[i], "upright") for m, (o, i) in ind.ENDS.items() if m != "PR"]
    if on == "lca":
        base = [b for b in base if b[0] not in ("LF", "LR")]
    return base + [("PR", w["pushrod_outer"], w["spring_inner"], on)], on == "lca"


def _sched(key):
    v = [CASES[c][key] for c in CASES]
    return f"IF(time-0.5: {v[0]}, {v[0]}, IF(time-1.5: {v[1]}, {v[1]}, IF(time-2.5: {v[2]}, {v[2]}, {v[3]})))"


def _link_frames(po, pi):
    r = (np.asarray(pi, float) - np.asarray(po, float))
    r /= np.linalg.norm(r)
    a = np.cross(r, [0, 0, 1.0]); a /= np.linalg.norm(a)                                      # noqa: E702
    b = np.cross(r, a)
    oi = avb._euler_from(np.column_stack([a, np.cross(r, a), r]))
    oj = avb._euler_from(np.column_stack([np.cross(b, r), b, r]))
    return r, oi, oj


def loads_command_file(name: str) -> str:
    w = corner(name)
    P = {k: np.asarray(v, float) for k, v in w.items() if isinstance(v, list) and len(v) == 3}
    links, has_lca = _links(w, name)
    m = f"{name}_loads_v1"
    M, f = "." + m, avb._f
    L = [f"! KinematiK direct-acting corner {name} for Adams View: member forces at the design position.",
         "! Links spherical at the moving body, Hooke at the chassis" + (
             "; the lower wishbone is a rigid body on a revolute pivot carrying the damper" if has_lca else "")
         + ": zero degrees of freedom. T_PR is the damper force.",
         "! Load cases one per step, by time: " + ", ".join(f"{i}={c}" for i, c in enumerate(CASES)) + ".",
         "! Frame: x rearward, y outboard (right-hand corner), z up from the ground; mm, N, N*mm.",
         "default units length=mm mass=kg force=newton time=second angle=degrees",
         f"model create model_name={m}"]
    parts = ["upright"] + (["lca"] if has_lca else []) + [x[0].lower() for x in links]
    L += [f"part create rigid_body name_and_position part_name={M}.{p} location=0.0, 0.0, 0.0 "
          "orientation=0.0, 0.0, 0.0" for p in parts]

    def mk(part, mname, loc, ori=(0.0, 0.0, 0.0)):
        L.append(f"marker create marker_name={M}.{part}.{mname} location={f(loc)} orientation={f(ori)}")
    mk("ground", "g_ref", (0.0, 0.0, 0.0))
    mk("upright", "u_cp", P["contact_patch"]); mk("upright", "u_wc", P["wheel_center"])       # noqa: E702
    if has_lca:
        o = avb._euler313(P["lower_rear_inner"] - P["lower_front_inner"])
        mk("ground", "g_lca", P["lower_front_inner"], o); mk("lca", "lca_piv", P["lower_front_inner"], o)  # noqa: E702
        mk("lca", "lca_out", P["lower_outer"]); mk("upright", "u_lbj", P["lower_outer"])              # noqa: E702
        L += [f"constraint create joint revolute joint_name={M}.lca_rev i_marker_name={M}.lca.lca_piv "
              f"j_marker_name={M}.ground.g_lca",
              f"constraint create joint spherical joint_name={M}.lbj i_marker_name={M}.upright.u_lbj "
              f"j_marker_name={M}.lca.lca_out"]
    meas = []
    for mem, po, pi, on in links:
        r, oi, oj = _link_frames(po, pi)
        p = mem.lower()
        mk(on, f"m_{p}", po); mk(p, f"{p}_out", po)                                                  # noqa: E702
        mk(p, f"{p}_in", pi, oi); mk("ground", f"g_{p}", pi, oj)                                      # noqa: E702
        L += [f"constraint create joint spherical joint_name={M}.{p}_sph i_marker_name={M}.{on}.m_{p} "
              f"j_marker_name={M}.{p}.{p}_out",
              f"constraint create joint hooke joint_name={M}.{p}_hk i_marker_name={M}.{p}.{p}_in "
              f"j_marker_name={M}.ground.g_{p}"]
        I, J = f"{M}.{on}.m_{p}", f"{M}.{p}.{p}_out"
        meas.append((f"T_{mem}", f"FX({I},{J})*({r[0]:.9f}) + FY({I},{J})*({r[1]:.9f}) + FZ({I},{J})*({r[2]:.9f})"))
    L += [f"force create direct force_vector force_vector_name={M}.patch_load i_marker_name={M}.upright.u_cp "
          f"j_part_name={M}.ground ref_marker_name={M}.ground.g_ref "
          f'x_force_function="{_sched("Fx")}" y_force_function="{_sched("Fy")}" z_force_function="{_sched("Fz")}"',
          f"force create direct force_vector force_vector_name={M}.shaft_load i_marker_name={M}.upright.u_wc "
          f"j_part_name={M}.ground ref_marker_name={M}.ground.g_ref "
          f'x_force_function="{_sched("Fx_wc")}" y_force_function="0.0" z_force_function="0.0"',
          f"force create direct torque_vector torque_vector_name={M}.aligning i_marker_name={M}.upright.u_cp "
          f"j_part_name={M}.ground ref_marker_name={M}.ground.g_ref "
          f'x_torque_function="0.0" y_torque_function="0.0" z_torque_function="{_sched("Mz")}"']
    L += [f'measure create function measure_name={M}.{n} function="{fn}"' for n, fn in meas]
    L += [f"default model model_name={M}", "simulation single_run reset",
          f"simulation single_run transient type=kinematic end_time=3.0 number_of_steps=3 model_name={M} "
          "initial_static=no"]
    return "\n".join(L) + "\n"


# ---------------------------------------------------------------- Adams Solver datasets (MotionSolve)
def _deg(o):
    return ", ".join(f"{a:.4f}D" for a in o)


def loads_adm(name: str) -> str:
    w = corner(name)
    P = {k: np.asarray(v, float) for k, v in w.items() if isinstance(v, list) and len(v) == 3}
    links, has_lca = _links(w, name)
    f = avb._f
    L = [f"KinematiK {name} direct-acting member forces (Adams Solver dataset, runs in MotionSolve)",
         "! Load cases one per step, by time: " + ", ".join(f"{i}={c}" for i, c in enumerate(CASES))
         + ". REQUEST k, F2 = axial force (+ tension); T_PR is the damper.",
         "UNITS/FORCE=NEWTON, MASS=KILOGRAM, LENGTH=MILLIMETER, TIME=SECOND",
         "PART/1, GROUND", "MARKER/1, PART=1, QP=0, 0, 0",
         "MARKER/2, PART=1, FLOATING", "MARKER/3, PART=1, FLOATING", "MARKER/4, PART=1, FLOATING",
         "PART/2", f"MARKER/20, PART=2, QP={f(P['contact_patch'])}", f"MARKER/21, PART=2, QP={f(P['wheel_center'])}"]
    joints, reqs = [], []
    if has_lca:
        o = avb._euler313(P["lower_rear_inner"] - P["lower_front_inner"])
        L += ["PART/9", f"MARKER/90, PART=9, QP={f(P['lower_front_inner'])}, REULER={_deg(o)}",
              f"MARKER/91, PART=1, QP={f(P['lower_front_inner'])}, REULER={_deg(o)}",
              f"MARKER/92, PART=9, QP={f(P['lower_outer'])}", f"MARKER/93, PART=2, QP={f(P['lower_outer'])}"]
        joints += ["JOINT/90, REVOLUTE, I=90, J=91", "JOINT/91, SPHERICAL, I=93, J=92"]
    for k, (mem, po, pi, on) in enumerate(links):
        r, oi, oj = _link_frames(po, pi)
        pid, base = 3 + k, 100 + 10 * k
        on_id = 9 if on == "lca" else 2
        L += [f"PART/{pid}", f"MARKER/{base}, PART={on_id}, QP={f(po)}", f"MARKER/{base + 1}, PART={pid}, QP={f(po)}",
              f"MARKER/{base + 2}, PART={pid}, QP={f(pi)}, REULER={_deg(oi)}",
              f"MARKER/{base + 3}, PART=1, QP={f(pi)}, REULER={_deg(oj)}"]
        joints += [f"JOINT/{base}, SPHERICAL, I={base}, J={base + 1}",
                   f"JOINT/{base + 1}, HOOKE, I={base + 2}, J={base + 3}"]
        I, J = base, base + 1
        reqs += [f"REQUEST/{k + 1}, TITLE=T_{mem}",
                 f", F2 = FX({I},{J})*({r[0]:.9f}) + FY({I},{J})*({r[1]:.9f}) + FZ({I},{J})*({r[2]:.9f})"]
    L += joints
    L += ["VFORCE/1, I=20, JFLOAT=2, RM=1",
          f", FX = {_sched('Fx')}\\", f", FY = {_sched('Fy')}\\", f", FZ = {_sched('Fz')}",
          "VFORCE/2, I=21, JFLOAT=3, RM=1", f", FX = {_sched('Fx_wc')}\\", ", FY = 0\\", ", FZ = 0",
          "VTORQUE/1, I=20, JFLOAT=4, RM=1", ", TX = 0\\", ", TY = 0\\", f", TZ = {_sched('Mz')}"]
    return "\n".join(L + reqs + ["OUTPUT/REQSAVE", "END"]) + "\n"


def mr_adm(name: str) -> str:
    """The kinematic sweep of the Adams View kinematics benchmark as an Adams Solver dataset, plus the damper:
    REQUEST/1 F2 = wheel-centre travel, F3 = damper length."""
    w = corner(name)
    P = {k: np.asarray(v, float) for k, v in w.items() if isinstance(v, list) and len(v) == 3}
    f = avb._f
    o_l = avb._euler313(P["lower_rear_inner"] - P["lower_front_inner"])
    o_u = avb._euler313(P["upper_rear_inner"] - P["upper_front_inner"])
    _, oi, oj = _link_frames(P["tie_rod_outer"], P["tie_rod_inner"])
    on = 4 if w["pushrod_attach"] == "upright" else 2
    L = [f"KinematiK {name} direct-acting damper over wheel travel (Adams Solver dataset, runs in MotionSolve)",
         "! Parts: 2 lower wishbone, 3 upper wishbone, 4 upright, 5 tie rod, 6 jack. Jack drives wheel-centre height "
         "-25..+25 mm through an inplane primitive. REQUEST/1: F2 travel, F3 damper length.",
         "UNITS/FORCE=NEWTON, MASS=KILOGRAM, LENGTH=MILLIMETER, TIME=SECOND",
         "PART/1, GROUND",
         f"MARKER/10, PART=1, QP={f(P['lower_front_inner'])}, REULER={_deg(o_l)}",
         f"MARKER/11, PART=1, QP={f(P['upper_front_inner'])}, REULER={_deg(o_u)}",
         f"MARKER/12, PART=1, QP={f(P['tie_rod_inner'])}, REULER={_deg(oj)}",
         f"MARKER/13, PART=1, QP={f(P['wheel_center'])}",
         f"MARKER/14, PART=1, QP={f(P['wheel_center'])}",
         f"MARKER/15, PART=1, QP={f(P['spring_inner'])}",
         "PART/2", f"MARKER/20, PART=2, QP={f(P['lower_front_inner'])}, REULER={_deg(o_l)}",
         f"MARKER/21, PART=2, QP={f(P['lower_outer'])}",
         "PART/3", f"MARKER/30, PART=3, QP={f(P['upper_front_inner'])}, REULER={_deg(o_u)}",
         f"MARKER/31, PART=3, QP={f(P['upper_outer'])}",
         "PART/4", f"MARKER/40, PART=4, QP={f(P['lower_outer'])}", f"MARKER/41, PART=4, QP={f(P['upper_outer'])}",
         f"MARKER/42, PART=4, QP={f(P['tie_rod_outer'])}", f"MARKER/43, PART=4, QP={f(P['wheel_center'])}",
         "PART/5", f"MARKER/50, PART=5, QP={f(P['tie_rod_outer'])}",
         f"MARKER/51, PART=5, QP={f(P['tie_rod_inner'])}, REULER={_deg(oi)}",
         "PART/6", f"MARKER/60, PART=6, QP={f(P['wheel_center'])}", f"MARKER/61, PART=6, QP={f(P['wheel_center'])}",
         f"MARKER/70, PART={on}, QP={f(P['pushrod_outer'])}",
         "JOINT/1, REVOLUTE, I=20, J=10", "JOINT/2, REVOLUTE, I=30, J=11",
         "JOINT/3, SPHERICAL, I=40, J=21", "JOINT/4, SPHERICAL, I=41, J=31", "JOINT/5, SPHERICAL, I=42, J=50",
         "JOINT/6, HOOKE, I=51, J=12", "JOINT/7, TRANSLATIONAL, I=61, J=14",
         "JPRIM/1, INPLANE, I=43, J=60",
         "MOTION/1, JOINT=7, TRANSLATION, FUNCTION=50.0*TIME - 25.0",
         "REQUEST/1, TITLE=damper", ", F2 = DZ(43,13)\\", ", F3 = DM(70,15)",
         "OUTPUT/REQSAVE", "END"]
    return "\n".join(L) + "\n"


def acf(stem: str, end: float, steps: int) -> str:
    return f"{stem}.adm\n{stem}\nSIMULATE/KINEMATICS, END={end}, STEPS={steps}\nSTOP\n"


# ---------------------------------------------------------------- sheets
def inputs_sheet() -> str:
    pts = ["upper_front_inner", "upper_rear_inner", "upper_outer", "lower_front_inner", "lower_rear_inner",
           "lower_outer", "tie_rod_inner", "tie_rod_outer", "wheel_center", "contact_patch"]
    rows = ["# Direct-acting corners. mm; x rearward, y outboard, z up from the ground. No rocker.",
            "# damper_outer is on the body named in damper_on; damper_inner is the chassis mount.",
            "corner,point,x,y,z"]
    for n in CORNERS:
        w = corner(n)
        rows += [f"{n},{p}," + ",".join(f"{c:.4f}" for c in w[p]) for p in pts]
        rows += [f"{n},damper_outer," + ",".join(f"{c:.4f}" for c in w["pushrod_outer"]),
                 f"{n},damper_inner," + ",".join(f"{c:.4f}" for c in w["spring_inner"])]
    rows += ["", "corner,damper_on,static_camber,static_toe"]
    rows += [f"{n},{'upright' if corner(n)['pushrod_attach'] == 'upright' else 'lower wishbone'},"
             f"{corner(n)['static_camber']},{corner(n)['static_toe']}" for n in CORNERS]
    rows += ["", "case,Fx,Fy,Fz,Fx_wc,Mz"]
    rows += [f"{c},{d['Fx']},{d['Fy']},{d['Fz']},{d['Fx_wc']},{d['Mz']}" for c, d in CASES.items()]
    return "\n".join(rows) + "\n"


def damper_template(tool: str) -> str:
    trv = TRAVEL if tool == "motionsolve" else HAND_TRAVEL
    head = (f"# {tool}: damper length (mm) at each wheel-centre travel (mm, + bump). Fill every row of a corner or "
            "leave the corner blank.\n")
    return head + "corner,travel_mm,damper_mm\n" + "".join(f"{n},{t:g},\n" for n in CORNERS for t in trv)


def loads_template(tool: str) -> str:
    head = (f"# {tool}: axial force per member, N, + tension, design position. direct_lower: UF, UR, TR and the "
            "damper PR only (leave LF, LR blank). Fill every needed cell of a corner or leave it blank.\n")
    rows = ["corner,case," + ",".join(ALL6)] + [f"{n},{c}," + "," * 5 for n in CORNERS for c in CASES]
    return head + "\n".join(rows) + "\n"


def write_all():
    (BENCH / "corners").mkdir(exist_ok=True)
    (BENCH / "adams").mkdir(exist_ok=True)
    (BENCH / "motionsolve").mkdir(exist_ok=True)
    for n in CORNERS:
        (BENCH / "corners" / f"{n}.json").write_text(json.dumps(build_corner(n), indent=1) + "\n")
    for n in CORNERS:
        (BENCH / "adams" / f"{n}_mr.cmd").write_text(mr_command_file(n))
        (BENCH / "adams" / f"{n}_loads.cmd").write_text(loads_command_file(n))
        (BENCH / "motionsolve" / f"{n}_mr.adm").write_text(mr_adm(n))
        (BENCH / "motionsolve" / f"{n}_mr.acf").write_text(acf(f"{n}_mr", 1.0, 50))
        (BENCH / "motionsolve" / f"{n}_loads.adm").write_text(loads_adm(n))
        (BENCH / "motionsolve" / f"{n}_loads.acf").write_text(acf(f"{n}_loads", 3.0, 3))
    (BENCH / "inputs.csv").write_text(inputs_sheet())
    for t in ("motionsolve", "ard"):
        for kind, fn in (("damper", damper_template), ("loads", loads_template)):
            p = BENCH / f"{t}_{kind}.csv"
            if not p.exists():
                p.write_text(fn(t))


def main(argv):
    if "--write" in argv:
        write_all()
        print("wrote benchmarks/direct_acting/")
        return 0
    ok = True
    for t in TOOLS:
        for n in CORNERS:
            r = compare(t, n)
            ok &= "FAIL" not in (r["damper"], r["loads"])
            d = r["damper"] + (f" ({r['damper_worst_mm']:.2e} mm)" if "damper_worst_mm" in r else "")
            lo = r["loads"] + (f" ({r['loads_worst_N']:.2e} N at {r['where']})" if "loads_worst_N" in r else "")
            print(f"{t:12s} {n:15s} damper {d:28s} loads {lo}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
