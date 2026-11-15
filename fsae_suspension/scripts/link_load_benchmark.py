# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""KinematiK's link loads against the tools the kinematics was benchmarked against.

Corners: front_v11_final, rear_v11_final, front_v11_joints50 (the Adams View kinematics corners), at the design
position. Load cases and limits: benchmarks/link_loads/limits.json, registered before any comparison was run.

  independent  scripts/independent_link_loads.py: displacement method, SciPy fsolve, no shared code. Runs here.
  adams_view   write(): an Adams View command file per corner. Each member is a link body, spherical at the upright
               and Hooke at the chassis, so the model has zero degrees of freedom; the four load cases are applied one
               per output step (time 0, 1, 2, 3) and the axial force in each link is a measure. compare() reads the
               table exported from the PostProcessor (same export as the kinematics benchmark): <corner>.tab.
  motionsolve  adm_file()/acf_file(): the same model as an Adams Solver dataset, which MotionSolve runs (or MotionView
               imports as a solver deck). Results go into benchmarks/link_loads/motionsolve_results.csv.
  ard          results entered by hand from Applied Racing Dynamics into benchmarks/link_loads/ard_results.csv
               (template written by --write), + tension, N.
  Lotus is registered in limits.json but has no run: no licence, and the paper used for the Lotus kinematics
  benchmark does not hold usable member forces (scripts/arevalo_link_load_check.py).

A tool with no result file is reported as NOT RUN, never as passed.

    python scripts/link_load_benchmark.py           # compare every tool that has results; exit 1 on any failure
    python scripts/link_load_benchmark.py --write   # regenerate the Adams command files and the CSV templates
"""
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
import adams_view_benchmark as avb                                                       # noqa: E402
import independent_link_loads as ind                                                     # noqa: E402
from suspension import loadpath as lp                                                    # noqa: E402
from suspension.genesis_repro import hp_from_dict                                        # noqa: E402
from suspension.kinematics import SuspensionKinematics                                   # noqa: E402

BENCH = ROOT / "benchmarks" / "link_loads"
MAN = ROOT / "manifests"
CORNERS = avb.CORNERS
REG = json.loads((BENCH / "limits.json").read_text())
CASES = REG["load_cases"]
LIMITS = REG["limits"]
MEMBERS = ind.MEMBERS
TOOLS = ("independent", "adams_view", "motionsolve", "chrono", "matlab", "ard")


def _winner(name):
    return json.loads((MAN / f"{name}.recorded.genesis.json").read_text())["recorded"]["winner_hardpoints"]


def kinematik(name: str) -> dict:
    """{case: {member: N}} from KinematiK's loadpath at the design position."""
    kin = SuspensionKinematics(hp_from_dict(_winner(name)))
    s0 = kin.solve_at_travel(0.0)
    out = {}
    for c, d in CASES.items():
        mf = lp.solve_member_forces(kin, s0, lp.WheelLoad(Fx=d["Fx"], Fy=d["Fy"], Fz=d["Fz"], Mz=d["Mz"],
                                                         Fx_wc=d["Fx_wc"]))
        out[c] = {m: mf.forces[m] for m in MEMBERS}
    return out


def independent(name: str) -> dict:
    hp = _winner(name)
    return {c: ind.member_forces(hp, d) for c, d in CASES.items()}


def read_adams(name: str) -> dict | None:
    """Adams View results: the per-measure Spread Sheet files adams/<corner>__T_xx.tab the command file writes, or
    one PostProcessor table <corner>.tab."""
    files = {m: BENCH / "adams" / f"{name}__T_{m}.tab" for m in MEMBERS}
    if all(f.exists() for f in files.values()):
        out = {}
        for m, f in files.items():
            model, rs, tt, q = avb.read_spreadsheet(f)
            if not model.startswith(name) or rs != f"T_{m}":
                raise ValueError(f"{f.name} holds {model}.{rs}, not {name} T_{m}")
            for c in CASES:
                out.setdefault(c, {})[m] = float(q[int(np.argmin(np.abs(tt - list(CASES).index(c))))])
        return out
    p = BENCH / f"{name}.tab"
    if not p.exists():
        return None
    model, t = avb.read_table(p)
    if not model.startswith(name):
        raise ValueError(f"{p.name} was exported from model {model!r}, not from {name}")
    out = {}
    for c in CASES:
        i = int(np.argmin(np.abs(t["Time"] - list(CASES).index(c))))
        out[c] = {m: float(t[f"T_{m}"][i]) for m in MEMBERS}
    return out


def read_csv(tool: str, name: str) -> dict | None:
    p = BENCH / f"{tool}_results.csv"
    if not p.exists():
        return None
    rows = [r for r in csv.DictReader(ln for ln in p.read_text().splitlines() if not ln.startswith("#"))
            if r["corner"] == name]
    if not rows or any(not r[m].strip() for r in rows for m in MEMBERS):
        return None                                          # template not filled in for this corner
    return {r["case"]: {m: float(r[m]) for m in MEMBERS} for r in rows}


def reference(tool: str, name: str) -> dict | None:
    if tool == "independent":
        return independent(name)
    if tool == "adams_view":
        return read_adams(name)
    return read_csv(tool, name)


def compare(tool: str, name: str) -> dict:
    """Worst |KinematiK - tool| and the worst ratio to its limit, over every case and member."""
    ref = reference(tool, name)
    if ref is None:
        return {"tool": tool, "corner": name, "status": "NOT RUN"}
    kk, lim = kinematik(name), LIMITS[tool]
    worst, ratio, where = 0.0, 0.0, None
    for c in CASES:
        for m in MEMBERS:
            d = abs(kk[c][m] - ref[c][m])
            r = d / (lim["abs_N"] + lim["rel"] * abs(ref[c][m]))
            if r > ratio:
                worst, ratio, where = d, r, (c, m)
    return {"tool": tool, "corner": name, "worst_N": worst, "ratio": ratio, "where": where,
            "status": "PASS" if ratio <= 1.0 else "FAIL"}


# ---------------------------------------------------------------- Adams View command file
def command_file(name: str) -> str:
    w = _winner(name)
    P = {k: np.asarray(v, float) for k, v in w.items() if isinstance(v, list) and len(v) == 3}
    m = f"{name}_loads_v1"
    M = "." + m
    f = avb._f
    L = [f"! KinematiK corner {name} for Adams View: link-load benchmark at the design position.",
         "! Each member is a link, spherical at the upright and Hooke at the chassis: zero degrees of freedom.",
         "! Load cases one per step, by time: " + ", ".join(f"{i}={c}" for i, c in enumerate(CASES)) + ".",
         "! Frame: x rearward, y outboard (right-hand corner), z up from the ground; mm, N, N*mm.",
         "default units length=mm mass=kg force=newton time=second angle=degrees",
         f"model create model_name={m}",
         f"part create rigid_body name_and_position part_name={M}.upright location=0.0, 0.0, 0.0 "
         "orientation=0.0, 0.0, 0.0"]
    for mem in MEMBERS:
        L.append(f"part create rigid_body name_and_position part_name={M}.{mem.lower()} "
                 "location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0")

    def mk(part, mname, loc, ori=(0.0, 0.0, 0.0)):
        L.append(f"marker create marker_name={M}.{part}.{mname} location={f(loc)} orientation={f(ori)}")
    mk("ground", "g_ref", (0.0, 0.0, 0.0))
    mk("upright", "u_cp", P["contact_patch"]); mk("upright", "u_wc", P["wheel_center"])       # noqa: E702
    meas = []
    for mem in MEMBERS:
        o, i = ind.ENDS[mem]
        po, pi = P[o], P[i]
        r = (pi - po) / np.linalg.norm(pi - po)
        a = np.cross(r, [0, 0, 1.0]); a /= np.linalg.norm(a)                                    # noqa: E702
        b = np.cross(r, a)
        RI, RJ = np.column_stack([a, np.cross(r, a), r]), np.column_stack([np.cross(b, r), b, r])
        oi, oj = avb._euler_from(RI), avb._euler_from(RJ)
        assert np.allclose(avb._rot313(*oi), RI, atol=1e-9) and np.allclose(avb._rot313(*oj), RJ, atol=1e-9)
        p = mem.lower()
        mk("upright", f"u_{p}", po); mk(p, f"{p}_out", po)                                     # noqa: E702
        mk(p, f"{p}_in", pi, oi); mk("ground", f"g_{p}", pi, oj)                                # noqa: E702
        L += [f"constraint create joint spherical joint_name={M}.{p}_sph i_marker_name={M}.upright.u_{p} "
              f"j_marker_name={M}.{p}.{p}_out",
              f"constraint create joint hooke joint_name={M}.{p}_hk i_marker_name={M}.{p}.{p}_in "
              f"j_marker_name={M}.ground.g_{p}"]
        I, J = f"{M}.upright.u_{p}", f"{M}.{p}.{p}_out"
        meas.append((f"T_{mem}", f"FX({I},{J})*({r[0]:.9f}) + FY({I},{J})*({r[1]:.9f}) + FZ({I},{J})*({r[2]:.9f})"))

    def sched(key):                                     # the case value at each integer time
        v = [CASES[c][key] for c in CASES]
        return f"IF(time-0.5: {v[0]}, {v[0]}, IF(time-1.5: {v[1]}, {v[1]}, IF(time-2.5: {v[2]}, {v[2]}, {v[3]})))"
    L += [f"force create direct force_vector force_vector_name={M}.patch_load i_marker_name={M}.upright.u_cp "
          f"j_part_name={M}.ground ref_marker_name={M}.ground.g_ref "
          f'x_force_function="{sched("Fx")}" y_force_function="{sched("Fy")}" z_force_function="{sched("Fz")}"',
          f"force create direct force_vector force_vector_name={M}.shaft_load i_marker_name={M}.upright.u_wc "
          f"j_part_name={M}.ground ref_marker_name={M}.ground.g_ref "
          f'x_force_function="{sched("Fx_wc")}" y_force_function="0.0" z_force_function="0.0"',
          f"force create direct torque_vector torque_vector_name={M}.aligning i_marker_name={M}.upright.u_cp "
          f"j_part_name={M}.ground ref_marker_name={M}.ground.g_ref "
          f'x_torque_function="0.0" y_torque_function="0.0" z_torque_function="{sched("Mz")}"']
    for n, fn in meas:
        L.append(f'measure create function measure_name={M}.{n} function="{fn}"')
    L += [f"default model model_name={M}", "simulation single_run reset",
          f"simulation single_run transient type=kinematic end_time=3.0 number_of_steps=3 model_name={M} "
          "initial_static=no"]
    L += ["! Write each measure to <corner>__T_xx.tab in Adams' working directory."]
    L += avb.export_commands(m, [f"T_{x}" for x in MEMBERS], name)
    return "\n".join(L) + "\n"


# ---------------------------------------------------------------- Adams Solver dataset (MotionSolve)
def adm_file(name: str) -> str:
    """The link-load model as an Adams Solver dataset: ground, upright, six links (spherical at the upright, Hooke at
    the chassis), the four load cases one per step, and one REQUEST per member whose F2 is the axial force."""
    w = _winner(name)
    P = {k: np.asarray(v, float) for k, v in w.items() if isinstance(v, list) and len(v) == 3}
    f = avb._f
    deg = lambda o: ", ".join(f"{a:.4f}D" for a in o)                                   # noqa: E731
    L = [f"KinematiK {name} link-load benchmark (Adams Solver dataset, runs in MotionSolve)",
         "! Frame: x rearward, y outboard (right-hand corner), z up from the ground; mm, N, N*mm.",
         "! Load cases one per step, by time: " + ", ".join(f"{i}={c}" for i, c in enumerate(CASES)) + ".",
         "UNITS/FORCE=NEWTON, MASS=KILOGRAM, LENGTH=MILLIMETER, TIME=SECOND",
         "PART/1, GROUND",
         "MARKER/1, PART=1, QP=0, 0, 0",
         "MARKER/2, PART=1, FLOATING",
         "MARKER/3, PART=1, FLOATING",
         "MARKER/4, PART=1, FLOATING",
         "PART/2",
         f"MARKER/20, PART=2, QP={f(P['contact_patch'])}",
         f"MARKER/21, PART=2, QP={f(P['wheel_center'])}"]
    joints, reqs = [], []
    for k, mem in enumerate(MEMBERS):
        o, i = ind.ENDS[mem]
        po, pi = P[o], P[i]
        r = (pi - po) / np.linalg.norm(pi - po)
        a = np.cross(r, [0, 0, 1.0]); a /= np.linalg.norm(a)                                    # noqa: E702
        b = np.cross(r, a)
        oi = avb._euler_from(np.column_stack([a, np.cross(r, a), r]))
        oj = avb._euler_from(np.column_stack([np.cross(b, r), b, r]))
        pid, base = 3 + k, 100 + 10 * k
        L += [f"PART/{pid}",
              f"MARKER/{base}, PART=2, QP={f(po)}",
              f"MARKER/{base + 1}, PART={pid}, QP={f(po)}",
              f"MARKER/{base + 2}, PART={pid}, QP={f(pi)}, REULER={deg(oi)}",
              f"MARKER/{base + 3}, PART=1, QP={f(pi)}, REULER={deg(oj)}"]
        joints += [f"JOINT/{base}, SPHERICAL, I={base}, J={base + 1}",
                   f"JOINT/{base + 1}, HOOKE, I={base + 2}, J={base + 3}"]
        I, J = base, base + 1
        reqs += [f"REQUEST/{k + 1}, TITLE=T_{mem}",
                 f", F2 = FX({I},{J})*({r[0]:.9f}) + FY({I},{J})*({r[1]:.9f}) + FZ({I},{J})*({r[2]:.9f})"]

    def sched(key):
        v = [CASES[c][key] for c in CASES]
        return f"IF(time-0.5: {v[0]}, {v[0]}, IF(time-1.5: {v[1]}, {v[1]}, IF(time-2.5: {v[2]}, {v[2]}, {v[3]})))"
    L += joints
    L += ["VFORCE/1, I=20, JFLOAT=2, RM=1",
          f", FX = {sched('Fx')}\\", f", FY = {sched('Fy')}\\", f", FZ = {sched('Fz')}",
          "VFORCE/2, I=21, JFLOAT=3, RM=1",
          f", FX = {sched('Fx_wc')}\\", ", FY = 0\\", ", FZ = 0",
          "VTORQUE/1, I=20, JFLOAT=4, RM=1",
          ", TX = 0\\", ", TY = 0\\", f", TZ = {sched('Mz')}"]
    L += reqs
    L += ["OUTPUT/REQSAVE", "END"]
    return "\n".join(L) + "\n"


def acf_file(name: str) -> str:
    return f"{name}.adm\n{name}\nSIMULATE/KINEMATICS, END=3.0, STEPS=3\nSTOP\n"


def inputs_sheet() -> str:
    """Hardpoints and load cases in one sheet, for entering the corners into ARD (or any other tool) by hand."""
    pts = ["upper_front_inner", "upper_rear_inner", "upper_outer", "lower_front_inner", "lower_rear_inner",
           "lower_outer", "tie_rod_inner", "tie_rod_outer", "pushrod_outer", "rocker_pushrod", "wheel_center",
           "contact_patch"]
    rows = ["# Hardpoints (mm; x rearward, y outboard, z up from the ground) and load cases (N, N*mm; + tension out).",
            "# Pushrod: from pushrod_outer on the upright to a FIXED chassis point at rocker_pushrod.",
            "# Loads: Fx Fy Fz at the contact patch, Fx_wc at the wheel centre, Mz about vertical.",
            "corner,point,x,y,z"]
    for n in CORNERS:
        w = _winner(n)
        rows += [f"{n},{p}," + ",".join(f"{c:.4f}" for c in w[p]) for p in pts]
    rows += ["", "case,Fx,Fy,Fz,Fx_wc,Mz"]
    rows += [f"{c},{d['Fx']},{d['Fy']},{d['Fz']},{d['Fx_wc']},{d['Mz']}" for c, d in CASES.items()]
    return "\n".join(rows) + "\n"


def csv_template(tool: str) -> str:
    head = (f"# {tool}: axial force per member, N, + tension, at the design position. Hardpoints from "
            "manifests/<corner>.recorded.genesis.json; loads from limits.json (contact patch, Fx_wc at the wheel "
            "centre, Mz about z). Fill every cell for a corner or leave the corner blank.\n")
    rows = ["corner,case," + ",".join(MEMBERS)]
    rows += [f"{n},{c}," + "," * (len(MEMBERS) - 1) for n in CORNERS for c in CASES]
    return head + "\n".join(rows) + "\n"


def main(argv):
    if "--write" in argv:
        (BENCH / "motionsolve").mkdir(exist_ok=True)
        for n in CORNERS:
            (BENCH / f"{n}.cmd").write_text(command_file(n))
            (BENCH / "motionsolve" / f"{n}.adm").write_text(adm_file(n))
            (BENCH / "motionsolve" / f"{n}.acf").write_text(acf_file(n))
        (BENCH / "inputs.csv").write_text(inputs_sheet())
        for t in ("motionsolve", "ard"):
            p = BENCH / f"{t}_results.csv"
            if not p.exists():
                p.write_text(csv_template(t))
        print("wrote benchmarks/link_loads/*.cmd and the CSV templates")
        return 0
    ok = True
    for t in TOOLS:
        for n in CORNERS:
            r = compare(t, n)
            if r["status"] == "NOT RUN":
                print(f"{t:12s} {n:20s} NOT RUN")
                continue
            ok &= r["status"] == "PASS"
            print(f"{t:12s} {n:20s} worst {r['worst_N']:.2e} N  {r['ratio']:.2e} of limit at {r['where']}  {r['status']}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
