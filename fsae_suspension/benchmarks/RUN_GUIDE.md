# KinematiK benchmark run guide

Everything to run in Adams View, MotionSolve and ARD, in order. Limits were registered before any run
(`limits.json` in each folder): do not change them. When a tool rejects a line on import, note the line and send it
back; the generator gets fixed, not the file.

Label every published result with the licence it was run on, e.g. "Adams View 2026.1 Student Edition, academic
research".

## What is being benchmarked

| Benchmark | Corners | Quantity | Folder |
|---|---|---|---|
| A. Link loads | front_v11_final, rear_v11_final, front_v11_joints50 | Axial force in UF, UR, LF, LR, TR, PR under 4 load cases | `link_loads/` |
| B. Direct-acting motion ratio | direct_upright, direct_lower | Damper length over wheel travel −25…+25 mm | `direct_acting/` |
| C. Direct-acting loads | direct_upright (all 6), direct_lower (UF, UR, TR, damper) | Axial force under the same 4 load cases | `direct_acting/` |

Load cases, one per step (time 0, 1, 2, 3): bump, corner, brake, corner_drive. Forces in N, + tension.

## 1. Adams View (8 runs)

For each `.cmd` file: **File > Import > Adams View Command File**, select it, let it run. Then
**PostProcessor > File > Export > Table** (spreadsheet), pick the measures listed, save under the name given, in the
same folder as the `.cmd`.

| Import | Export measures | Save as |
|---|---|---|
| `link_loads/front_v11_final.cmd` | T_UF T_UR T_LF T_LR T_TR T_PR | `link_loads/front_v11_final.tab` |
| `link_loads/rear_v11_final.cmd` | same | `link_loads/rear_v11_final.tab` |
| `link_loads/front_v11_joints50.cmd` | same | `link_loads/front_v11_joints50.tab` |
| `direct_acting/adams/direct_upright_mr.cmd` | travel, damper_len | `direct_acting/adams/direct_upright_mr.tab` |
| `direct_acting/adams/direct_lower_mr.cmd` | travel, damper_len | `direct_acting/adams/direct_lower_mr.tab` |
| `direct_acting/adams/direct_upright_loads.cmd` | T_UF T_UR T_LF T_LR T_TR T_PR | `direct_acting/adams/direct_upright_loads.tab` |
| `direct_acting/adams/direct_lower_loads.cmd` | T_UF T_UR T_TR T_PR | `direct_acting/adams/direct_lower_loads.tab` |

(7 files; the kinematics benchmark's 3 `.tab` files are already done.)

Check before exporting: the loads runs have 4 output steps (time 0–3); the `_mr` runs have 51 (travel −25…+25).

## 2. MotionSolve (7 runs)

Install: Altair Student Edition via your school email (siemens.com > Altair Student Edition). MotionSolve is in the
solver package, MotionView/HyperGraph in the desktop package.

Run each `.acf` with the MotionSolve solver (or MotionView > **File > Import > Solver Deck**, pick the `.adm`, run a
kinematic analysis to the end time in the `.acf`). Read the requests in **HyperGraph** and type or paste them into the
CSV.

| Run | Read | Into |
|---|---|---|
| `link_loads/motionsolve/<corner>.acf` (3 corners) | REQUEST T_UF…T_PR, component F2, at time 0, 1, 2, 3 | `link_loads/motionsolve_results.csv` |
| `direct_acting/motionsolve/<corner>_mr.acf` (2) | REQUEST damper: F2 = travel, F3 = damper length, every step | `direct_acting/motionsolve_damper.csv` |
| `direct_acting/motionsolve/<corner>_loads.acf` (2) | REQUEST T_*, F2, at time 0–3 | `direct_acting/motionsolve_loads.csv` |

## 3. MATLAB Simscape Multibody (one command, R2025b)

1. Apply the patch to the repo (`git am …`), so `scripts/matlab/kk_bench_simscape.m` sits in the repo.
2. In MATLAB: `cd` to `fsae_suspension/scripts/matlab`, then run `kk_bench_simscape`.
   (From anywhere else: `kk_bench_simscape('<path to fsae_suspension>')`.)
3. It checks a closed-form case first, then builds and solves every corner. It writes
   `link_loads/matlab_results.csv`, `direct_acting/matlab_damper.csv` and `matlab_loads.csv` itself: nothing to copy.
4. If it stops with an error, send the whole message. When a block parameter name differs in R2025b, the error
   lists every parameter that block has, which is what's needed to fix it.

## 4. Applied Racing Dynamics (by hand)

Build the corners from the inputs sheets, apply the loads, and type the results into the CSVs (N, + tension; mm).

| Build from | Enter results in |
|---|---|
| `link_loads/inputs.csv` (pushrod to a fixed point at rocker_pushrod) | `link_loads/ard_results.csv` |
| `direct_acting/inputs.csv` (direct-acting damper between damper_outer and damper_inner) | `direct_acting/ard_damper.csv` (damper length every 5 mm), `direct_acting/ard_loads.csv` |

Watch ARD's sign convention: these sheets are + tension. If ARD cannot output member forces or damper length, leave
the sheet blank: it reports NOT RUN, which is the honest result.

## 5. Send back

The `.tab` files and filled CSVs (MATLAB writes its own CSVs into the repo: commit them or send them). Then in the repo:

```
python scripts/link_load_benchmark.py
python scripts/direct_acting_benchmark.py
```

Each prints PASS, FAIL or NOT RUN per tool and corner. A FAIL is published like the Stanford result.

## Already done (no action)

- Independent solve, link loads: pass, 0.017 N worst.
- Independent solve, direct-acting: motion ratio pass on both corners (1e-13 mm); loads pass on direct_upright (0.008 N).
- Project Chrono 9.0.1 (run here, `scripts/chrono_benchmark.py`): pass on all three benchmarks, including the
  lower-wishbone direct-acting loads (forces within 5e-7 N, damper within 7e-6 mm).
- MATLAB Simscape Multibody R2025b (run by the author): pass on all three benchmarks (forces within 5e-7 N, damper
  within 5e-6 mm).
- Lotus: no licence; Arevalo et al. (2018) member forces checked and not usable (`scripts/arevalo_link_load_check.py`).
