# Validation

What has been checked, how, what it found, and what has not been checked yet. Every
number here can be re-run from this repository; the derivations are in
[PHYSICS.md](PHYSICS.md) and the section numbers below point there. Every benchmark test, listed by role and
feature, is in [BENCHMARK.md](BENCHMARK.md).

## Summary

| Check | Against | Result | Re-run |
|---|---|---|---|
| Suspension kinematics | Adams View 2026.1, the paper's three corners | Camber, toe, caster, KPI and wheel-centre path within 0.00003° and 0.00003 mm at every 1 mm over ±25 mm (limits 0.005–0.01° and 0.05 mm, set before the run) | `scripts/adams_view_benchmark.py`, `tests/test_adams_view_benchmark.py` |
| Suspension kinematics | Lotus Suspension Analysis, two published corners | Camber in bump within 0.009°, toe within 0.0003°, camber in roll within 0.008° | `scripts/lotus_benchmark.py`, `tests/test_lotus_benchmark.py` |
| Suspension kinematics | Applied Racing Dynamics (ARD), front and rear corners | Static caster, KPI, scrub, motion ratio and camber change at ±20 mm agree to ARD's display precision; every larger difference traced to a definition | §32.4 |
| Suspension kinematics | Independent rigid-body solve (SciPy `fsolve`, no shared code) | Camber and toe within 1e-12° on three corners | `scripts/independent_kinematics.py` |
| Synthesis reproducibility | Re-runs by SIT Formula Racing and Elbee Racing on their own machines | Identical to the recorded outputs: coordinates to 1e-6 mm, fits and yield to 1e-9 | `scripts/verify_run.py` |
| Published numbers | Recomputed from the manifests | 499 of 499 claims match | `scripts/paper_check.py`, weekly in CI |
| Final corners | Re-solved from the manifests | Match the recorded outputs | `scripts/verify_run.py`, weekly in CI |
| Statics and load cases | Virtual work, global equilibrium, closed-form beams | Pass | §32.1, `tests/test_load_case_physics.py` |
| Panel method | Sphere in potential flow | Cp minimum −1.248 against −1.250; mean error 0.009, largest 0.028 | `scripts/aero_closed_form.py` |
| Vortex lattice | Rectangular AR 8 wing | C_L 11% under lifting-line, 4% under an independent lattice (open, see below) | `scripts/aero_closed_form.py` |
| Code | About 3,500 tests, lint, CodeQL, dependency audit, bandit | Green on every push | badges in the README |

## Kinematics

**Adams View.** The three corners of the paper (`front_v11_final`, `rear_v11_final`, `front_v11_joints50`) built
in Adams View 2026.1 Student Edition from command files generated from their manifests: rigid wishbones on
revolute pivots, spherical ball joints, a tie rod with a Hooke joint at the fixed rack end, and the wheel centre
driven from −25 to +25 mm in 1 mm steps. The limits were set before the runs. Largest differences over all three
corners: camber 0.000027°, toe 0.000005°, caster 0.000010°, KPI 0.000006°, wheel-centre x 0.000025 mm and y 0.000027
mm, the size of KinematiK's interpolation and Adams' solver tolerance. The models were built from the command files,
not from the Adams/Car template, and this checks the solver against the solver, not against a built car. Files,
exports and limits: [benchmarks/adams_view](fsae_suspension/benchmarks/adams_view/README.md).

**Lotus Suspension Analysis.** Hardpoints from Arevalo, Medina and Valladolid,
*Ingenius* 20 (2018) 95–106, solved in KinematiK and compared with the Lotus
curves published there. Full tables in PHYSICS.md §32.5. Largest differences:
camber in bump 0.009°, toe in bump 0.0003°, camber in roll 0.008°, camber in
steer 0.050°, roll-centre height 3.3 mm. The Lotus values are read off the
paper's figures, so they carry reading error; the roll-centre gap also depends on
a tyre radius the source does not state.

**Applied Racing Dynamics.** Two corners entered into ARD's suspension model and
compared at static and ±20 mm (PHYSICS.md §32.4). Static values agree to ARD's
display precision. Toe change, roll-centre height over travel and rear caster and
KPI over travel differ because the two tools define them differently (static
camber in the spin axis, road versus body-frame datum, heave with the tyre on the
ground); each was confirmed with an independent solve. The comparison found one
real error, scrub measured on z = 0 instead of the road plane, which was fixed.

**Independent solve.** `scripts/independent_kinematics.py` treats the upright as
one rigid body with six unknowns and shares no code with the solver.

## Reproducibility

Every synthesis run writes a manifest with SHA-256 hashes of its inputs and
outputs. SIT Formula Racing (Singapore Institute of Technology) and Jonathan
Iglesias of Elbee Racing re-ran `front_v11_final` from the published code and got
the recorded result to 1e-6 mm. The `reproduce` workflow repeats this weekly for
the front and rear corners and recomputes all 499 published numbers from the
manifests. This shows the result reproduces; it is not a physical test of a car.

## Corrections the checks forced

Nine errors were found by these checks and fixed, each now pinned by a test:
lateral load-case signs, pushrod statics, scrub datum, instant-centre
construction, anti-dive sign, anti-squat path, degenerate roll centre, the tyre
model's camber scaling and curvature sign, and the damping-ratio rate (PHYSICS.md §32.3). In October 2026 three
more were fixed: EV energy at constant speed, brake rotor spin speed, and tyre gas
heating (`tests/test_energy_rotor_gas_physics.py`).

## Aero

The panel method matches the potential-flow sphere closely. The vortex lattice
reads 11% under lifting-line theory on a rectangular AR 8 wing; part of that is
expected (lifting line assumes elliptic loading), but it also reads 4% under an
independent flat-plate lattice on the same grid, and that gap is not explained
yet. Earlier text quoted 6% and a span efficiency of 1.004; those did not
reproduce and have been replaced with the measured values. Both solvers are
inviscid: no separation, stall or profile drag. Compare designs with them; do not
quote absolute downforce.

## Not validated yet

- **No measured car.** Nothing has been correlated with an instrumented vehicle,
  a K&C rig or a wind tunnel.
- **Tyre data.** The default tyre is a synthetic Magic Formula set, not a TTC fit.
  Load your own coefficients for real numbers.
- **Quasi-static.** Lap time, load transfer and thermal models are quasi-static or
  lumped. Transient results are trends.
- **ARD lap time.** A load-transfer-balance study in ARD's lap simulator is
  pending.
- **Vortex-lattice lift gap.** The 4% gap to an independent lattice above.
- **Most non-suspension modules** (brakes, accumulator, electronics, structures)
  are checked against closed forms and their own tests, not against another tool.
- **Powertrain, EV and combustion.** Checked against closed forms only (BENCHMARK.md).
  The combustion side has no engine model: the torque curve is declared, and fuel
  rests on one brake thermal efficiency. The comparison against GT-SUITE,
  Simulink, OptimumLap and PyBaMM is planned in
  [docs/POWERTRAIN_BENCHMARK_PLAN.md](docs/POWERTRAIN_BENCHMARK_PLAN.md) and not run.

## Peer review

A manuscript describing the inverse synthesis is in preparation and is in the process of being
peer reviewed. The software is archived at Zenodo,
[doi:10.5281/zenodo.23450788](https://doi.org/10.5281/zenodo.23450788).

## How this was built

KinematiK is written by one author. That is why every claim
above comes with a script that re-runs it: the checks, not the author, are the
evidence. If one of them is wrong, open an issue.
