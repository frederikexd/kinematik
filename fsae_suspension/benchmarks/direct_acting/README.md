# Direct-acting dampers against the kinematics benchmark tools

The direct-acting option (no rocker: the damper runs from the upright or lower wishbone straight to the chassis),
benchmarked on two corners built from front_v11_final:

| Corner | Damper outer end | Statics |
|---|---|---|
| `direct_upright` | the upright (the corner's pushrod pickup) | six two-force members |
| `direct_lower` | the lower wishbone, KinematiK's default placement | two bodies: the wishbone carries the damper |

Two quantities, limits in `limits.json`, registered on 14 Nov 2026 before any comparison:

- **Damper length** change from static at every 1 mm of wheel-centre travel, −25 to +25 mm (every 5 mm for hand
  entry). Its slope is the motion ratio.
- **Member forces** at the design position under the four link-load cases (`../link_loads/limits.json`). On
  `direct_lower` only UF, UR, TR and the damper (PR) are compared: the split of the wishbone's pivot reaction between its
  two legs is statically indeterminate in every rigid-body tool, so LF and LR are not a fair comparison.

| Tool | Damper limit | Force limit | direct_upright | direct_lower |
|---|---|---|---|---|
| Independent (no shared code) | 0.001 mm | 0.5 N + 0.1 % | **Pass**: damper 1e-13 mm, forces 0.008 N | **Pass**: damper 1e-13 mm; forces not run (checked against virtual work in `tests/test_direct_acting.py`) |
| Project Chrono 9.0.1 (`scripts/chrono_benchmark.py`) | 0.01 mm | 0.5 N + 0.1 % | **Pass**: damper 1.1e-6 mm, forces 4.7e-7 N | **Pass**: damper 7.2e-6 mm, forces 4.1e-7 N |
| Adams View 2026.1 Student Edition (run by the author, `adams/`) | 0.01 mm | 0.5 N + 0.1 % | **Pass**: damper 9.2e-5 mm, forces 6.1e-4 N | **Pass**: damper 6.9e-5 mm, forces 1.0e-3 N |
| MATLAB Simscape Multibody R2025b (`scripts/matlab/kk_bench_simscape.m`, run by the author) | 0.01 mm | 0.5 N + 0.1 % | **Pass**: damper 9.4e-7 mm, forces 4.7e-7 N | **Pass**: damper 4.3e-6 mm, forces 4.1e-7 N |
| Altair MotionSolve | 0.01 mm | 0.5 N + 0.1 % | Not run yet | Not run yet |
| Applied Racing Dynamics | 0.05 mm | 2 N + 0.5 % | Not run yet | Not run yet |

## Running each tool

**Adams View.** For each corner import `adams/<corner>_mr.cmd` (File > Import > Adams View Command File), run, then
PostProcessor > File > Export > Table with `travel` and `damper_len`, saved as `adams/<corner>_mr.tab`. Then import
`adams/<corner>_loads.cmd`, run, and export `T_UF` … `T_PR` as `adams/<corner>_loads.tab`. T_PR is the damper force.

**MotionSolve.** Run `motionsolve/<corner>_mr.acf` and `<corner>_loads.acf` (or MotionView > File > Import > Solver
Deck). From HyperGraph: REQUEST/1 of the `_mr` run gives travel (F2) and damper length (F3) per step, into
`motionsolve_damper.csv`; the `_loads` run gives T_* (F2) at time 0, 1, 2, 3 = bump, corner, brake, corner_drive, into
`motionsolve_loads.csv`.

**ARD.** Build both corners from `inputs.csv` with a direct-acting damper between `damper_outer` and `damper_inner`.
Enter the damper length at each travel in `ard_damper.csv` and the member forces in `ard_loads.csv` (+ tension).

None of the decks has been run in Adams or MotionSolve yet. If a tool rejects a line, fix the generator
(`scripts/direct_acting_benchmark.py`), not the file.

What this shows when complete: the motion ratio and damper force of a direct-acting corner agree with independent
solvers on the same rigid-link idealisation. What it does not show: damper friction, bushing compliance, or that a built
car matches its hardpoints.


MATLAB note: the Simscape script needed several fixes before its first complete run, all in how it built the models or read their outputs, none to the limits: unique block names, R2025b parameter names, a custom inertia, reading the sensed constraint force as a vector (the first run kept only its vertical component, which itself matched KinematiK's forces times each link's vertical direction to six digits), frame-port paths for the Kinematics Solver, driving wheel-centre height directly, and revolute pivots that cannot depend on Simscape's Rigid Transform conventions. Each fix is a separate commit.

Re-run: `python scripts/direct_acting_benchmark.py` and `pytest tests/test_direct_acting_benchmark.py`.
