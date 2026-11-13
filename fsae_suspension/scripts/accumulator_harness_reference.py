#!/usr/bin/env python3
# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Regenerate the accumulator and harness references (docs/ACCUMULATOR_HARNESS_BENCHMARK_PLAN.md).

  h1   ngspice:   harness wire voltage drop with copper's temperature coefficient   -> h1_ngspice_wire_drop.json
  a1   PyBaMM:    pack terminal voltage, current and charge along a lap             -> a1_pybamm_pack_lap.json
  a2   PyBaMM:    lumped pack temperature with cooling over 30 minutes              -> a2_pybamm_pack_thermal.json
  a3   liionpack: a 140s3p pack network, cell by cell, with interconnects           -> a3_liionpack_network.json

Nothing here imports KinematiK: the references are built from the tools and the plan alone. Needs ngspice on PATH
and `pip install pybamm liionpack`. tests/test_accumulator_harness_benchmark.py checks KinematiK against the JSON.
"""
import argparse
import json
import math
import re
import subprocess
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "benchmarks" / "electrical" / "accu_harness"
RHO_IACS = 1.7241e-8          # Ω·m, annealed copper at 100 % IACS, 20 °C
TC1 = 0.00393                 # 1/K


def _write(name, d):
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / name).write_text(json.dumps(d, indent=1) + "\n", encoding="utf-8")
    print("wrote", OUT / name)


def _ngspice_version():
    out = subprocess.run(["ngspice", "--version"], capture_output=True, text=True).stdout
    m = re.search(r"ngspice-(\d+)", out)
    return f"ngspice-{m.group(1)}" if m else "ngspice"


def h1():
    rows = []
    current = 10.0
    for awg in (22, 18, 14, 10, 6):
        d_m = 0.127e-3 * 92.0 ** ((36 - awg) / 39.0)
        area = math.pi * d_m * d_m / 4.0
        for length_m in (0.5, 2.0, 5.0):
            r20 = RHO_IACS * length_m / area
            for temp_c in (20.0, 60.0, 105.0):
                net = (f"* wire {awg} AWG {length_m} m at {temp_c} C\n"
                       f"I1 0 n1 DC {current}\n"
                       f"R1 n1 0 {r20!r} tc1={TC1}\n"
                       f".options tnom=20\n.temp {temp_c}\n"
                       ".control\nop\nprint v(n1)\n.endc\n.end\n")
                with tempfile.NamedTemporaryFile("w", suffix=".cir", delete=False) as f:
                    f.write(net)
                out = subprocess.run(["ngspice", "-b", f.name], capture_output=True, text=True).stdout
                v = float(re.search(r"v\(n1\)\s*=\s*([-+0-9.eE]+)", out).group(1))
                rows.append({"awg": awg, "length_m": length_m, "temp_c": temp_c, "current_a": current,
                             "drop_v": v})
    _write("h1_ngspice_wire_drop.json", {"tool": _ngspice_version(), "rho_ohm_m": RHO_IACS, "tc1_per_k": TC1,
                                         "rows": rows})


def _ecm(pybamm, ocv_soc, capacity_ah, r_ohm, soc0, *, heat_capacity=1.0e5, cooling=0.0, ambient_k=298.15):
    model = pybamm.equivalent_circuit.Thevenin(options={"number of rc elements": 0})
    p = model.default_parameter_values
    soc_pts, v_pts = (np.asarray(x, dtype=float) for x in ocv_soc)

    def ocv(soc):
        return pybamm.Interpolant(soc_pts, v_pts, soc, interpolator="linear", extrapolate=True)

    p.update({
        "Open-circuit voltage [V]": ocv,
        "R0 [Ohm]": r_ohm,
        "Cell capacity [A.h]": capacity_ah,
        "Nominal cell capacity [A.h]": capacity_ah,
        "Initial SoC": soc0,
        "Entropic change [V/K]": 0.0,
        "Cell thermal mass [J/K]": heat_capacity,
        "Cell-jig heat transfer coefficient [W/K]": cooling,
        "Jig thermal mass [J/K]": 1.0e15,
        "Jig-air heat transfer coefficient [W/K]": 0.0,
        "Ambient temperature [K]": ambient_k,
        "Initial temperature [K]": ambient_k,
        "Upper voltage cut-off [V]": 1.0e6,
        "Lower voltage cut-off [V]": 0.0,
    }, check_already_exists=False)
    return model, p


def _run_steps(pybamm, model, params, steps):
    exp = pybamm.Experiment(steps)
    sim = pybamm.Simulation(model, parameter_values=params, experiment=exp)
    sol = sim.solve()
    first = []
    all_steps = [st for cyc in sol.cycles for st in cyc.steps]    # each experiment entry is its own cycle
    for s in all_steps:
        first.append((float(s["Voltage [V]"].entries[0]), float(s["Current [A]"].entries[0]),
                      float(s["SoC"].entries[0]), float(s["Cell temperature [K]"].entries[0])))
    last = all_steps[-1]
    end = (float(last["Voltage [V]"].entries[-1]), float(last["Current [A]"].entries[-1]),
           float(last["SoC"].entries[-1]), float(last["Cell temperature [K]"].entries[-1]))
    return first, end


def a1():
    import pybamm
    lap = json.loads((ROOT / "benchmarks" / "electrical" / "spice_pack_lap.json").read_text())["inputs"]
    p, t = np.asarray(lap["p_terminal_w"], float), np.asarray(lap["t_s"], float)
    model, params = _ecm(pybamm, lap["ocv_soc"], lap["capacity_ah"], lap["r_pack_ohm"], lap["soc0"])
    #  KinematiK holds each point's power over the interval that follows it; so does each experiment step.
    steps = [pybamm.step.power(float(p[k]), duration=float(t[k + 1] - t[k]), period=float(t[k + 1] - t[k]))
             for k in range(p.size - 1)]
    first, end = _run_steps(pybamm, model, params, steps)
    first.append(end)                         # the state at the last point
    _write("a1_pybamm_pack_lap.json", {
        "tool": f"PyBaMM {pybamm.__version__} (Thevenin ECM, no RC pairs, power steps)",
        "inputs": {"source": "spice_pack_lap.json"},
        "v_terminal_v": [r[0] for r in first], "i_a": [r[1] for r in first], "soc": [r[2] for r in first],
        "note": "the last point's voltage and current belong to the last step's power, not the last point's"})


def _thermal_profile():
    blocks = []       # (current A, seconds): acceleration, cruise, regen, rest, repeated to 30 minutes
    pattern = [(160.0, 8), (60.0, 20), (-45.0, 6), (100.0, 12), (20.0, 14)]
    total = 0
    while total < 1800:
        for cur, dur in pattern:
            if total >= 1800:
                break
            dur = min(dur, 1800 - total)
            blocks.append((cur, dur))
            total += dur
    return blocks


def a2():
    import pybamm
    blocks = _thermal_profile()
    c, h, r, amb = 40000.0, 10.0, 0.1, 298.15
    model, params = _ecm(pybamm, ([0.0, 1.0], [500.0, 600.0]), 1.0e4, r, 0.9,
                         heat_capacity=c, cooling=h, ambient_k=amb)
    steps = [pybamm.step.current(cur, duration=float(dur), period=1.0) for cur, dur in blocks]
    sol = pybamm.Simulation(model, parameter_values=params, experiment=pybamm.Experiment(steps)).solve()
    times, temps = [], []
    for s in (st for cyc in sol.cycles for st in cyc.steps):
        tt, tk = s["Time [s]"].entries, s["Cell temperature [K]"].entries
        times.extend(float(x) for x in tt)
        temps.extend(float(x) for x in tk)
    grid = np.arange(0.0, 1800.0 + 0.5, 1.0)
    ref = np.interp(grid, np.asarray(times), np.asarray(temps))
    _write("a2_pybamm_pack_thermal.json", {
        "tool": f"PyBaMM {pybamm.__version__} (Thevenin ECM lumped thermal, jig held at ambient)",
        "inputs": {"blocks_a_s": blocks, "heat_capacity_j_k": c, "cooling_w_k": h, "r_pack_ohm": r,
                   "ambient_k": amb, "dt_s": 1.0},
        "t_s": grid.tolist(), "temp_k": ref.tolist()})


def a3():
    import liionpack as lp
    ns, np_, r_cell, v_cell, i_pack = 140, 3, 0.0128, 4.0, 100.0
    cases = {}
    for label, rc, rb in (("ideal", 1e-9, 1e-9), ("realistic", 2e-4, 1e-4)):
        net = lp.setup_circuit(Np=np_, Ns=ns, Ri=r_cell, Rc=rc, Rb=rb, Rt=1e-9, I=i_pack, V=v_cell)
        v_node, i_batt = lp.solve_circuit(net)
        src = net[net.desc.str.startswith("I")].iloc[0]
        cases[label] = {"rc_ohm": rc, "rb_ohm": rb,
                        "terminal_v": float(v_node[int(src.node1)] - v_node[int(src.node2)]),
                        "cell_currents_a": [float(-x) for x in i_batt]}
    _write("a3_liionpack_network.json", {
        "tool": f"liionpack {lp.__version__} (modified nodal analysis, parallel strings, terminals at one end)",
        "inputs": {"n_series": ns, "n_parallel": np_, "r_cell_ohm": r_cell, "v_cell_v": v_cell,
                   "i_pack_a": i_pack},
        "cases": cases})


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("steps", nargs="*", default=["h1", "a1", "a2", "a3"])
    for step in ap.parse_args().steps:
        {"h1": h1, "a1": a1, "a2": a2, "a3": a3}[step]()
