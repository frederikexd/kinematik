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
| The tests themselves | Mutation testing of the kinematics and statics; property-based invariants on random corners | all 1,679 deliberate errors caught (floor 99%); 14 laws hold on 100 random cases each; core solvers and seven physics, model and workbook modules at 100% statement and branch coverage | `.github/workflows/mutation.yml`, `tests/test_invariants_property.py` |

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

## Testing the tests

A suite that only agrees with the code it was written against proves little, so the tests are tested too.

**Mutation testing.** `mutmut` makes 1,679 small deliberate errors in `suspension/kinematics.py` and
`suspension/loadpath.py` (a flipped sign, a swapped operator, a wrong index) and runs the kinematics, statics,
property and benchmark suites against each. All 1,679 are caught (it was 70.7% before the property tests, 74.8% before
`tests/test_mutation_gaps.py` and 93.7% before the equivalent mutants were marked). The survivors were sorted by what they change. The float conversions that about
90 of them dropped now happen in one helper, `_vec`, instead of at 45 call sites, and the solver settings (iteration
limits, tolerances) are marked not to mutate, since any setting that converges gives the same answer; the rest were
real gaps, and `tests/test_mutation_gaps.py` closes the largest groups: the pushrod-carrying wishbone's own balance and the split
of its pivot reactions, every compliance link-length shim, the default corner pinned value by value, a save-and-reload
of every field, the validation messages, the rocker rotation, the motion-ratio functions against finite differences
of the solved linkage, and the side-view centre against the solved ball-joint motion. Dead code those mutants were
pointing at (cached rocker lengths and frames nothing read, a residual that was computed and discarded) was removed.
Of the 117 that then survived, 19 were already caught: mutmut 3 keeps the result of a mutant whose function has not
changed, so tests added since did not count until those results were cleared. The rest fell in two groups. Lines whose
every mutant gives the same answer are marked `# pragma: no mutate` with the reason on the line: a `<` against `<=` on
a floating-point threshold that no real input meets exactly, a slice past the end of a fixed-length array, the sign of
a solve parameter that is never read, a residual that is a sum of zeros on a balanced solve, and one row of the
pivot-axis cross-product matrix (any two rows fix the reaction up to a multiple of the axis, which the leg split then
removes); two redundant pieces of code were deleted instead. Every other survivor was a missing check, now in
`tests/test_mutation_gaps.py`: a rocker that does not close gives no motion ratio, a failed linkage solve is marked
unconverged, each solve starts from the seed it is given, the bending arm is clipped to the leg, points are stored as
floats, the default sweep and curve ranges, a state without a pushrod point falls back to the strut, and notes add up
rather than replace each other. That removed 189 mutants and left none surviving. The score is held at a 99% floor
monthly (`.github/workflows/mutation.yml`), and the run lists every survivor.

**Property-based invariants.** `tests/test_invariants_property.py` draws 100 corners at random around the published
ones, with every pickup moved independently by up to 4 mm and the pushrod on the upright or either wishbone, and
random wheel loads including the halfshaft's wheel-centre force and moment. On every one: links and the upright stay
rigid and the pushrod rides its own body; moving the whole corner changes no angle, anti-geometry or swing arm;
stepping to a travel and solving straight to it agree; member forces are linear in the load and balance it on the
chassis; the pushrod force equals virtual work; an independent one-variable rocker solve gives the same spring length
and motion ratio; a compliant link carries its stretch exactly; and inboard anti-lift is anti-squat times the brake
bias. The examples are derandomized, so a failure reproduces exactly.

**Coverage.** The four core solvers, `kinematics.py`, `loadpath.py` (statics), `compliance.py` and
`elastokinematics.py`, are at 100% of statements and branches, and a CI job holds them there on every push. Reaching
that took tests of what the corner studies never exercise: degenerate geometry (parallel arms, a joint on its own pivot
axis, a horizontal kingpin, no pickup stagger), inputs that must be refused, the fall-backs (no rocker, a
non-determinate member set, a solve that runs out of iterations), joint linearisation and damping, and the required
stiffness for a budget (`tests/test_core_solver_edges.py`). Each checks an answer against a closed form or an
independent calculation, not only that the line ran. Two helpers that nothing called were removed.

Five physics modules are held at 100% the same way: the engine and cooling model (`powertrain/engine.py`), the
underfloor (`aero/underfloor.py`), the fit forecast (`fit_forecast.py`), rotor thermal (`brake_thermal.py`) and the EV
electrical check (`ev_electrical_check.py`). So is the full-car 3D model (`fullcar3d.py`), whose tests check the mesh
primitives against closed forms (box and cylinder volumes, the NACA section thickness, closed meshes) and every body
against its documented sizing law; they found six drawing bugs, also in the changelog. The EV workbook round trip
(`ev_excel_roundtrip.py`) is held there too, tested on a workbook built to the stock layout (the stock file is not in
the repository): the propulsion chain at constant speed against its closed form, the profile and block rows the
dependent sheets read, and the parameter read-back when cached values, sheets or labels are missing. Their tests check Darcy–Weisbach and Hagen–Poiseuille losses, the
Borda–Carnot expansion, the pump and system operating point, ε-NTU, energy conservation in the thermal transient,
`x = at²/2` and the load-transfer traction limit at launch, zero force from a constant-area duct, force proportional to
dynamic pressure, the radiative and convective balance solved back for the power, and the pack and fuse arithmetic by
hand. Writing them found six bugs, listed in the changelog.

Two coverage figures are reported on every push from the same test run. Everything in `suspension/` and `powertrain/`
is at 87.5% (it was 81.8% before the physics-module, full-car and EV workbook tests), held at an 87% floor. The
engineering code alone is at 89.0%, held at 88.5%: that figure leaves out the twelve modules that compute nothing about the car (sign-in and its
screens, workspaces and project history in the database, report storage and Google Drive export, usage analytics and
its cache, the anonymous visitor id, preview mode and the hosting memory guard). Those modules are still tested and
still counted in the first figure; the list is in `.github/workflows/tests.yml`, so anyone can see what was left out.
The largest gaps left in the engineering code are the briefing concept figures (`brief_visuals.py`, 0%), the binary
Altium reader (`pcb_altium_binary.py`, 17%), the process library (`process_library.py`, 18%) and the chassis mesh code
(`chassis.py`, 52%).

**Pinned reproduction.** The weekly reproduce run installs the exact versions in `requirements-reproduce.lock`, so the
published numbers are recomputed on a fixed environment, not on whatever numpy and scipy were released that week.

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
