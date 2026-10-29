# Changelog


## Unreleased


## 1.3.0 (2026-10-29)

**CI runs the same on any machine.** Fixes found when the new suites first ran on GitHub's runners:
- `contact_patch.QuarterCar.is_stable()` decided an undamped corner from rounding noise (its poles sit on the
  imaginary axis, at +-1e-15 depending on the CPU's LAPACK) and could call it stable; a pole now counts as decaying
  only when it is clearly left of the axis, so an undamped corner is reported as not stable on every machine;
- the panel solver prints a pressure drag that rounds to zero as `+0.000` (it printed `-0.000` on some CPUs), and a
  straight-back load arrow in `morphmesh` reads +180 deg rather than either sign of the branch cut;
- golden files are compared at a relative 1e-7 in the suites whose results come from iterative solves, and at 1e-6
  for the panel method (part of its solve runs in float32); every other suite keeps 1e-9;
- tests no longer depend on pytest before 9.2 (no `pytest.approx` on nested data) or on `lxml` being installed
  (openpyxl writes an empty cell as `<v />` without it);
- the mutation gate fails a group outright when mutmut cannot start (its clean run of the tests failed) instead of
  scoring it 0%, and three groups that were just under 99% on Python 3.12 had equivalent mutants removed.

**Every physics module held at a 99% mutation floor.** `mutation_targets.toml` now lists 80 groups covering 83 files
in `suspension/` and `powertrain/`, and the shared floor is 99.0% per file (it was 90%); 60,223 of 60,443 mutants are
caught (99.6%). 81 `tests/test_mutation_*.py` suites pin them, most as golden-master characterizations:
`tests/_golden.py` records a function's full output to `tests/golden/*.json` and compares at a relative 1e-9, with
booleans compared by type (`KINEMATIK_GOLDEN_RECORD=1` re-records). Equivalent mutants were removed by rewriting where
the code could say the same thing without them (default-equal arguments dropped, `partial(np.asarray, dtype=float)`
helpers instead of dtype arguments, module-level constants for defaults and einsum subscripts, guards that could never
fire deleted, single-line statements where a pragma had to apply) and otherwise marked
`# pragma: no mutate (reason)` on the line: float boundaries no input meets exactly, solver caps and tolerances, block
sizes and warm-start seeds. Results are unchanged; the existing suites pass as before. Smaller changes made on the way:
- `chassis.py`: the STEP and mesh loading shared by `load_chassis` and `load_part_mesh` is one helper, `_load_scene`;
  the static envelope check no longer tests `min_clear < 0` on an unsigned distance;
- `morphmesh.py`: `load_fan_from_audit` drops the empty-history checks that the peak test already covers and merges
  cases through one `big` pick; `_PlateFE` reads a material's density with a steel default for unknown materials;
- `simulforge.py`: an empty run's telemetry is built by one `_empty_telemetry()`; the response lag is the first
  maximum of the cross-correlation (as before) computed with `argmax`; the actuator force is applied without a
  zero-moment branch (a zero moment adds exactly zero);
- the chassis tests cap the address space, since mutmut counts a mutant that is killed by the out-of-memory killer as
  a survivor.

**Five physics modules at 100% coverage, and the bugs that turned up.** `powertrain/engine.py`,
`suspension/aero/underfloor.py`, `suspension/fit_forecast.py`, `suspension/brake_thermal.py` and
`suspension/ev_electrical_check.py` are now at 100% of statements and branches, held by a second step in the
`core-coverage` job. The five new test files check closed forms (Darcy–Weisbach, Haaland against Colebrook,
Borda–Carnot, ε-NTU, the pump operating point, `x = at²/2`, the traction limit with load transfer, the convective and
radiative balance) rather than only running the lines. Fixed along the way:
- The endurance equilibrium used the rotating-disc correlation while the duct-gain calculator used a calibrated set's
  fitted `h0 + h1·v`, so the gain it asked for did not give the target temperature. Both now use `convection_h`. The
  uncalibrated example rotor's equilibrium moves from 302 °C to 350 °C; the calculator now returns a gain of 1.00 for
  that temperature.
- The equilibrium's Newton solve started from the convective estimate and could stop far from the answer at high power
  (at 1e15 W the balance was off by a factor of 2e14). It now starts from the lower of the convective and radiative
  bounds.
- `engine.py` read the subsystem masses from the integration ledger under the wrong key and fell back to 230 kg; a car
  declaring 95 kg now gets 95 kg.
- The EV fuse warning printed the fuse limit twice instead of the lap's peak power against it.
- The EV check found the speed-vs-time sheet only by its exact name; it now uses the same alias and content lookup as
  the other sheets.
- With no single-cell current given, the EV check fell back to capacity over parallel count (5 A) instead of the fuse
  rating over parallel count.
- The underfloor header claimed the force saturates at low ride height; it does not (784 N at 40 mm, 1.9 kN at 20 mm,
  17 kN at 5 mm), and the model has no ground-effect stall, so read results below about 20 mm as an upper bound.
Removed two conditions that could never be false (`underfloor` aft recovery, `fit_forecast` deductions).

**Per-file mutation gate; elastokinematics and compliance join at 99.0% and 97.0%.** `mutation_targets.toml` lists the
files held to a mutation floor and the tests that pin each one; `scripts/mutation_gate.py` scores every file on its own
against 90% (the core group holds itself at 99%), and `.github/workflows/mutation.yml` runs only the groups a push
touches, every group monthly as parallel jobs, and (on a manual run with `baseline: true`) every module in
`suspension/` and `powertrain/` as its own report-only job. `suspension/elastokinematics.py` went from 78.2% to 99.0%
and `suspension/compliance.py` from 68.5% to 97.0%, with `tests/test_mutation_compliance.py` (24 tests) and these code
changes:
- the float conversions go through `kinematics._vec`;
- `solve_elastokinematic` reads loadpath's pickup forces directly: the fallback that rebuilt them from the axial force
  could not run, since loadpath reports a force on every pickup; an unused axis and the unused `_PICK_MEMBER` map were
  removed;
- `CompliantCorner.solve` no longer keeps a `length_deltas` copy nothing read; `_member_length` takes the outboard point
  from the state by its own field name instead of through an identity map;
- `damping_summary` no longer runs a compliance solve whose result it did not use (a joint's loss per cycle depends on
  amplitude and frequency only); its arguments are unchanged.
The mutmut working copy now includes `data/`, which the chassis-model test reads.

**Mutation testing at 100%.** All 1,679 deliberate errors in the kinematics and statics are caught; the floor is raised
from 92% to 99%. Of the 117 survivors left at 93.7%, 19 were stale results (mutmut 3 keeps a result while the function is
unchanged, so newer tests had not been counted). Lines whose every mutant is equivalent carry `# pragma: no mutate` with
the reason: floating-point thresholds where `<` and `<=` differ only on exact equality, slices past the end of a
9-entry or 6-row array, the unused sign in the instant-centre solves, the balance residual (a sum of zeros), one row of
the pivot-axis cross-product matrix, and the two multi-line solver calls, which now use a `start`/`end` block because a
trailing pragma covers only a statement's first line. The redundant `if self._has_rocker` around `_pushrod_outer_at` and
the restated `pushrod_attach="lower"` default were removed. `_solve_rocker_angle` takes its seed explicitly. Eight new
tests in `tests/test_mutation_gaps.py` cover the rest: a rocker that does not close, an unconverged solve, the seed
passed to each solve, the bending arm clipped at both ends of the leg, float storage, the default ranges, the strut
fallback for a state without a pushrod point, and notes that add up.

**Mutation testing at 93.7%.** The full run catches 1,751 of 1,868 deliberate errors (it was 70.7%, then 74.8%); the
floor is raised from 70% to 92%. The survivors were sorted by what they change. The float conversions about 90 of them
dropped now go through one helper (`_vec`) instead of 45 call sites, the solver settings are marked not to mutate (any
converging setting gives the same answer), and the new `tests/test_mutation_gaps.py` (58 tests, added to the mutation
selection) closes the real gaps, each by an independent calculation:
- the wishbone that carries the pushrod: the corner balances with every chassis reaction, the wishbone balances on
  its own with the ball-joint force the upright implies, and the pivot reactions are split for the least force
  across the legs (sign errors in that solve used to pass), for a pushrod on the lower and a pullrod on the upper arm;
- every compliance link-length shim lengthens that link and only that link; a chassis-tab shift keeps the unloaded
  length;
- the default corner pinned field by field, with its documented outputs;
- every hardpoint field through a save and reload, including an unknown key first;
- the validation messages; the rocker's right-hand rotation; spring length, motion ratio (and its cache) and the
  motion-ratio curve against finite differences of the solved linkage; the direct-acting proxy;
- the side-view centre against finite differences of the solved ball joints, at static and in bump, with the pivot
  axes square and angled in plan, and with one arm square and the other staggered; the anti-geometry at a given state
  and with its default biases;
- a corner drawn in metres solves the same as in millimetres, so no "is this zero" threshold is tied to a unit;
- the sweep and the motion-ratio curve warm-start each step from the previous solution, as their docstrings say;
- the exact notes the force solver attaches, and the flag for a near-singular linkage.
Removed code nothing read (cached rocker arm and spring lengths, a wishbone frame and kingpin axis, a tie-rod side
flag, a discarded force sum) and the `getattr` fallbacks for attributes that always exist (`WheelLoad` fields,
`_has_rocker`); `spring_length_at` now always places the pushrod from the solved joints; the two-force solve now
uses the same `_applied_moment` as the wishbone solve. Corrected `_side_view_swing_arm`'s docstring, which said the
default geometry's velocity and hand constructions differ (1908.6 against 2050.4 mm): with pivot axes that have no
lateral component they are the same line, both 1908.6 mm; they part only when the axes are angled in plan.

**The exported EV workbook is corrected, not just refilled.** `power_draw.audit_workbook()` pins thirteen defects in
the stock power-draw workbook to their cells; the round trip used to hand every one of them back. By default
(`correct_workbook=True`) the exported file now:
- adds `KX_RoadLoad`, a live road-load model on the profile rows (`a = dv/dt`, `F = m a + ½ρCdA v² + Crr m g`,
  `P = MAX(F v, 0)/η`, `I = P/V`, with mass, CdA, Crr, air density, efficiency and pack voltage as editable cells);
- points the ElecPropulsion current and power blocks at it, in all fifteen gear columns (pack current does not depend
  on the gear), replacing `V · PF · RPM / 1000` and the power formula that multiplied by efficiency and applied
  `√3 · PF` to the DC bus;
- divides wheel speed by the ratio cell instead of multiplying, so gear 7 turns the motor 7× the wheel, not 1/7;
- pulls block ranges that ran one row long back to their blocks (the RPM statistics, the EMF range), relabels
  "Min Power (W)" as kW and "Joule Heat (kW)" as W, sets date-formatted gear cells to General;
- writes pack resistance as `S · R_cell / P`, drops the stray fuse factor from ThermalLoad's max joule heat, turns
  "Sum of Joule Heat" into the energy `Σ P dt` in kWh, and gives BearingBlowOut `TAN(RADIANS(…))` and the pitch
  radius instead of the diameter;
- rewrites the EMF columns, which multiplied volts × kW × ohms, as the pack's terminal voltage under load,
  `V_pack − I · R_pack` with the road-load current: of the two voltages they could have meant, it is the only one the
  workbook holds the inputs for (a motor back-EMF needs a V/rpm constant the file does not have), and it uses the same
  pack voltage and resistance cells they already referenced;
- lists every change with its before, after and reason on a `KX_Corrections` sheet.
On a workbook in the stock layout, `audit_workbook` goes from thirteen findings to none. A second export does not
correct twice, cells it cannot recognise are left alone and said so, and `correct_workbook=False` keeps the old
behaviour. The audit's own checks now tell a corrected cell from a defective one (a divided ratio, `$B$16/2`,
`SUMPRODUCT`), and its reader checks test behaviour instead of asserting it. The formula evaluator no longer treats
text that quotes a formula as one.

**EV pack current from the road load, in the gear the workbook selects.** The workbook round trip reported pack
current from the workbook's own `V · PF · RPM / 1000`, which has no load or acceleration in it (a launch and a cruise
at the same speed read the same, and almost any lap "blew" a 50 A fuse), and it used the first gear column although
ThermalLoad, EMFs and BearingBlowOut select gear 7. It now uses the same road load as the Electrical Feasibility
Check, `I = max(m a + ½ρCdA v² + Crr m g, 0) · v / (η V_pack)` with `a = dv/dt` and that check's mass, CdA, Crr and
drivetrain efficiency, and reads the gear from `ROUND(1/ThermalLoad!B2, 0)` (stored with the workbook parameters, with
a stated fallback to gear 1 when there is no selector). The ratio cells hold 1/N, so the motor turns N times the
wheel; the verdict now also checks motor speed, torque and power against the declared limits in that gear. The
workbook's formula is kept as `workbook_current_a` and in the summary block for comparison. Both electrical checks
now credit no draw while coasting or braking: the feasibility check used to keep drag and rolling power on the pack
through every braking zone.

**The EV workbook round trip at 100% coverage, and six bugs in what it reports.** `suspension/ev_excel_roundtrip.py`
was at 51%; the new `tests/test_ev_excel_roundtrip.py` (37 tests) builds a workbook to the stock layout (live gear
formulas in three stacked blocks, dependent sheets that read them by absolute row, a CHOOSECOLS array formula, a pack
sheet read by label) and checks the propulsion chain at constant speed against its closed form, the rows written and
left alone, the rebuilt blocks of a flattened file, and the read-back paths. Fixed:
- The minimum-feasible-pack advisor showed nameplate energy in its "Energy (usable)" row, beside the current pack's
  usable energy, overstating the minimum by 1/0.92. It now reports the usable energy the lap draws, and sizes the
  nameplate capacity from it.
- With no series count in the pack sheet, the advisor sized a 504 V pack as one cell in series; the count is now
  derived from the pack and cell voltages.
- The integration ledger received "Current Across a Single Cell" as the pack's peak current, low by the parallel
  count. It now takes the propulsion sheet's pack current, or the cell current times the parallel strings.
- Stored workbook parameters read uncached `=1/7` gear ratios as 1:1 (the defect already fixed for the workbook path,
  still present in the database path); they are now read from the formula, and a zero ratio is reported.
- The pack heat map spread the modelled peak over an invented ±15% "gradient across the stack", printing cell
  temperatures above the modelled peak. Every cell carries the same current in this model, so the grid now shows the
  one modelled peak and says that a spread needs measured cell temperatures.
- A speed profile with title rows above its data was written over those rows when its extent came from the populated
  rows; it now starts at the first data row.
The workbook path and the database path now share one evaluation of the propulsion chain, so they cannot disagree,
and the comment that called the rectangle-rule energy sum trapezoidal is corrected.

**The full-car 3D model at 100% coverage, and six drawing bugs.** `suspension/fullcar3d.py` was at 7%; the new
`tests/test_fullcar3d.py` (69 tests) checks the mesh primitives against closed forms (a box's volume, a cylinder's
radius and polygon volume, the NACA thickness of the wing section, closed watertight meshes), every body against its
documented sizing law, the CG marker against a hand-weighted mean, the heat map against the ledger checks, and the
helpers that read a figure back. It joins the 100% step of the `core-coverage` job. Fixed:
- Every cylinder (tires, rims, brake discs, the motor, roll hoops, frame tubes) was triangulated with two different
  diagonals per side quad, so half of each quad was drawn twice and a sliver along every edge was left open. The
  meshes are now closed.
- The spring-rate ride-height cue was backwards: a softer spring lowered the wheels and the ground, so the body ended
  up higher above the ground. Now the frame, driver, wings, pods, motor, accumulator, logger and the inboard pickups
  drop by the static sag, and the wheels and ground stay at z = 0.
- Hiding or moving a suspension member ("Upright", "Tie rod", …) reached only the first corner, because the legend
  name is set once; it now reaches all four.
- A ledger passed in its `as_dict()` form was read at the wrong level, so every subsystem fell back to its nominal
  body. Dict ledgers are now converted first.
- The live influence read-out said the accumulator is "not drawn" when nothing is declared (a placeholder box is
  drawn), still called the frame a monocoque, and said "no power/envelope" when a powertrain envelope was declared.
- The CAD reconcile check listed an axis as changed when it was inside the tolerance it had just passed (it measured
  5% of the stand-in instead of the larger of the two sizes).
An unknown heat-map metric now falls back to the heat map with its own label, a comment that claimed the renderer
reads `part_anchor()` (it does not; the rear wing is drawn 122 mm further back) is corrected, and defensive branches
that could not run were removed.

**Two coverage figures.** The test job now reports coverage twice from one run: everything in `suspension/` and
`powertrain/` (87.5% with the full-car and EV workbook tests, floor raised from 81% to 87%), and the engineering code
alone (89.0%, floor 88.5%), which leaves
out the twelve account, cloud, telemetry and hosting modules listed in `.github/workflows/tests.yml`. Nothing is hidden
from the first figure.

**The core solvers at 100% coverage.** `kinematics.py`, `loadpath.py`, `compliance.py` and `elastokinematics.py` are
now at 100% of statements and branches, held by a new `core-coverage` CI job. The new `tests/test_core_solver_edges.py`
(47 tests) checks the edges against closed forms or independent calculations: parallel equal arms put the front-view
centre at infinity, unstaggered pickups give an infinite side-view swing arm, a non-determinate member set is flagged
and solved by least squares, the direct-acting motion-ratio proxy, joint linearisation and damping energy, rear and
inboard-drive corner loads, frame-twist toe against a re-solved tie-rod sensitivity, a coupled frame flexibility equal
to the point stiffnesses, the required stiffness for a budget, and every input the solvers must refuse. Removed
`SuspensionKinematics._tro_local_static`, which nothing called.

**Testing the tests.** Four additions aimed at the question of whether the tests would notice a wrong answer.
`tests/test_invariants_property.py` checks 14 laws of the kinematics and statics with `hypothesis` on randomly
perturbed corners (100 derandomized geometries and loads each, with the pushrod on the upright, the lower or the upper
wishbone): rigid links, a pushrod that rides its own body, no dependence on where the corner sits or the path taken,
statics linear in the load, chassis forces that balance the wheel load including the halfshaft's wheel-centre force
and moment, virtual work, an independent rocker solve for the spring length and motion ratio, compliance stretch
carried exactly, side-view geometry, and inboard anti-lift equal to anti-squat times the brake bias. Mutation testing
(`mutmut`, `.github/workflows/mutation.yml`, monthly) deliberately breaks `suspension/kinematics.py` and
`suspension/loadpath.py` and requires the suites to catch it: 70.7% of 2,355 mutants caught (it was 60.6% before the
property tests), held at a 70% floor. Coverage of `suspension/` and `powertrain/` (81.8%) is reported on every push
with an 81% floor. The weekly reproduce run now installs the exact versions in `requirements-reproduce.lock`, so a new
numpy or scipy cannot move a published number without a commit. The README shows the three suspension benchmarks and
how to re-run them. Removed `kinematics._rotation_from_to`, which nothing called.

**Preview mode: five minutes without an account.** With accounts on (`KINEMATIK_ENABLE_LOGIN`), the sign-in screen
now offers "👀 Preview KinematiK": the full app for five minutes, a countdown in the sidebar and a banner with a
"Create free account" button, a toast at one minute left, then the "Like what you see?" screen with Create account
preselected. The preview's project store is in memory, held in the visitor's own session (`suspension/preview.py`),
so a preview never reads or writes a workspace, the shared legacy project row or another visitor's preview. One
preview per browser session; invited visitors go straight to sign-up. Tests in `tests/test_preview.py`.


## 1.2.0 (2026-10-22)

The combustion powertrain (beta), the EV / ICE switch and the Adams View benchmark. Also since 1.1.0: EV energy
at constant speed, brake rotor spin speed and tyre gas heating fixed (`tests/test_energy_rotor_gas_physics.py`);
`networkx` added for the vortex-lattice mesher; tracked caches and telemetry removed from the repository. Every
result in the paper and supplement is unchanged: they cite v1.1.0, and the claim check passes on this release.

**EV / ICE slide switch.** The ⚡ Powertrain tab (was EV Powertrain) opens with a switch: left shows the electric
panels, right shows the combustion panel in their place instead of an expander above them. The CAD import stays on
both sides. Skipping the EV body in ICE mode is safe: no code outside the tab reads a variable it defines.

**Combustion (ICE) powertrain at parity with the EV side.** New `suspension/ice_powertrain.py` and the ⛽ panel at the
top of the powertrain tab (`ui/ice_powertrain.py`): the engine curve against the intake restrictor's choked air flow
(20.0 mm gasoline, 19.0 mm E85), gear ratios and shift points, a clutch launch with timed shifts over 75 m, a
final-drive sweep, the QSS lap on the best-gear envelope with engine inertia and a per-shift penalty, fuel per lap and
for the endurance against the tank and the fuel limit, CO2 for the efficiency event (and for an EV's kWh), engine
heat into the cooling loop, engine braking against the brake balance, an ICE feasibility gate and an engine-option
comparison. The torque curve is declared; until a team loads one, a representative restricted 600 cc inline-four
shape is used and flagged. Choked flow delegates to `throttle_dynamics.compressible_mass_flow`, registered in
`physics_owners.py`. 27 tests (`tests/test_ice_powertrain.py`), PHYSICS.md §25.5, FEATURES.md, BENCHMARK.md and
`docs/POWERTRAIN_BENCHMARK_PLAN.md` (the EV and ICE cases to run against GT-SUITE, Simulink, OptimumLap and PyBaMM).

**Adams View benchmark and a benchmark index.** The paper's three corners solved in Adams View 2026.1 agree with
KinematiK at every 1 mm over ±25 mm of wheel-centre travel: largest differences 0.000027° and 0.000027 mm against
limits of 0.005–0.01° and 0.05 mm set before the runs. The command files (generated from the manifests), the Adams
exports and the comparison are in `benchmarks/adams_view/`, `scripts/adams_view_benchmark.py` and
`tests/test_adams_view_benchmark.py`. New `BENCHMARK.md` lists every benchmark test by role and feature, and says
which features have none. The claim check gains 24 claims for the benchmark (499 in all), and 22 quotes are brought in
line with the current text (American spelling, two doubled words, and four reworded sentences); no value changed.

## 1.1.0 and earlier

**Load cases corrected; the final corners re-run.** Two load-case inconsistencies the paper disclosed are fixed in
code rather than in text. (1) The cornering cases carried a declared 50 N*m aligning moment at the lateral-force peak,
where the pneumatic trail has collapsed; `genesis_repro.vehicle_load_cases` now gives each case the tire's own moment,
`tire_aligning_moment_Nmm` (cosine-law pneumatic trail, brush t0 at 83 kPa and a 180 mm patch, at the slip that
carries the case's lateral force): -7.5 N*m front and -8.4 N*m rear at 1.5 g, -18.7 N*m front combined, -18.3 N*m
rear combined traction. A number passed as `aligning_torque_Nm` still declares a fixed moment. (2) The rear traction
and braking shock cases applied the longitudinal force at the contact patch, as for outboard drive and brakes;
`robustness.shock_cases` now takes `inboard_drive` and `inboard_brakes` (default True for the rear) and applies it at
the wheel centre (`Fx_wc`), and rear braking unloads the rear wheels. `front_v11_final` (2d35663ecd27, was
2e7072ce2268), `rear_v11_final` (1d2b4a0a7fca, was 8818c4b890d1) and `front_v11_joints50` (3678a72d786b, was
b6399630caba) were re-run on the corrected cases: each returns the same winner (every pickup within 0.0 mm), with
the same grade and yield. The rear link factor rises from 2.24 to 4.63 (lower-front leg -3.8 kN instead of -7.8 kN;
2.89 on 5/8 x 0.035 in legs throughout), bearings unchanged at 2.08; front cornering compliance toe becomes -0.012 deg
(was +0.034) on the final front and -0.018 deg (was +0.062) on the front to build; braking toe, the front link
factors and bearings are unchanged. `front_v11_from_85mm` (41c4fcdd9766, was 3bcacea5c507) and `front_v11_from_delivered` (1d9bb6277536, was
3079657a19c5), which carry every input of the final front, were re-run on the same cases and record the same
results (RESILIENT at 113 mm; NO_FIT at 1.26); only their cornering compliance toe moves. The replaced records are in
`manifests/superseded`; the node brief is rehashed (4f1c95fff3ab) and `scripts/paper_claims.py` quotes the current paper and supplement.

**The roll gradient is derived, not declared.** `target_derivation.roll_gradient_for_friction` gives the softest roll
gradient at which the ride frequency still holds the 1.06 g combined case inside the jounce budget at a stated tire
friction: with the springs at 2.6/3.0 Hz and the final corners' anti-geometry, the rear binds at 0.788 deg/g at mu 1.82
(the car tire's published belt friction), the front at 0.868, so the declared 0.78 deg/g is that limit rounded down.
`springs_only_roll_gradient` shows the springs alone give 1.36 deg/g, so the bars are needed for travel as well as
balance. Paper section 2 now opens with the chain from requirement to number for every target, citing the travel rule
(FS T2.5.1) and the hairpin rule (FSAE, 9 m outside diameter).

**A ceiling on Ackermann for the front to build.** `scripts/front_with_joints.py` now declares Ackermann at lock
between 75 and 100% (it had a floor only). The re-search reaches every wall in 260 evaluations and returns
`front_v11_joints50` (inputs b6399630caba), RESILIENT at 99.35%: 85.3% Ackermann at lock (was 143.8%), 115.6 deg of
wheel at the hairpin (was 120.0), wheel clearance +0.71 mm (was +0.45), braking toe 0.249 deg at 50 kN/mm per end
unchanged. The price is a quicker ratio, 4.88:1 against 5.28:1: 8.3 N*m at the hand and a 21.5 mm trail limit, so its
alternative upright moves the spindle 2.5 mm forward (`review_checks.ALT_BUILD_MM`, every wall held, +0.15 mm
clearance). The earlier record (7d226fcc1090) is in `manifests/superseded`; the node brief is rehashed (2699ec59d2cb).

**An independent forward solver.** `scripts/independent_kinematics.py` solves the corner as one rigid upright (six
unknowns, SciPy fsolve), sharing no code with `suspension/kinematics.py`; on the three final corners camber and toe
agree to 1e-12 deg over +/-25 mm and roll-centre heights to 0.6 mm. `scripts/export_points.py` writes each corner's
points and curves so an outside team with OptimumKinematics or another package can check them.

**A manifest re-run reports its result where it was started.** In the browser studio, ▶ Run this manifest now shows
the grade, build yield and fit right under the button, with the recorded grade beside it, whether the re-run matches
the record and what the start search did; the full results panel stays at the bottom of the tab
(`ui/inverse_genesis._manifest_run_summary`, `tests/test_manifest_run_summary.py`).

**The brake result against grip.** `review_checks.brake_grip()` gives the closed form at constant friction: the
balanced bias b = w_f + mu h/L, where a/g = mu; the 60 -> 73% gain is the same 24% at mu 1.55 and 13 to 24% (balanced
bias) across road mu 1.09 to 1.43. Tests: `tests/test_review_items.py`.

**What it changes on the car.** `review_checks.car_gains()` sets the delivered ChassisV10 front against the front to
build on the same rack, tire, load cases, actuation and 50 kN/mm link ends: braking limit 1.27 to 1.57 g (31.0 to
25.0 m from 100 km/h), toe-out per wheel in a 1.5 g stop 0.80 to 0.249 deg (brake imbalance held inside the toe band
20% to 64%), 137 to 120 deg of wheel at the hairpin, Ackermann force loss 2.8 to 0.8% at 1.5 g. Paper section 3.4.

**Every front check repeated on the front to build.** `scripts/review_checks.py` gains `front_to_build()`: Table 4's
rows, the anti-dive friction threshold, the steering margin, full lock, the alternative upright, the hairpin
Ackermann cost and the half-scale model predictions (`half_scale()`, defined at full scale with a declared model
tolerance) for front_v11_final and front_v11_joints50. On the front to build the 2.0 mm alternative upright would
break the wheel clearance, so its alternative is 0.5 mm; its 144% Ackermann at lock costs 0.8% (1.5 g) to 1.4%
(1.0 g) of front lateral force at the hairpin. The half-scale predictions printed earlier came from an earlier front
and are replaced. The node brief names `front_to_build` (front_v11_joints50).

**The final front re-searched with its joints declared.** `scripts/front_with_joints.py` declares every input of
front_v11_final with 50 kN/mm at every link end in both compliance cases and the start search. From
front_v11_final, which then breaks only the braking-toe wall, the pattern search reaches every wall in 811
evaluations (pickups moved 0.5 to 33 mm) and InverseGenesis returns `front_v11_joints50`, RESILIENT at 99.75%:
toe 0.249 deg in the 1.5 g stop and 0.056 deg in the corner at 50 kN/mm per end. The node brief carries it as the
front to build unless a pulled test tab and joint measure above 494 kN/mm per end.

**Three checks the walls do not make.** `suspension/compliance_margin.py` puts the joints, tabs and upright arms at
both ends of every link in series along it (one stiffness per end) and finds the end stiffness each compliance wall
needs, for any manifest. On the final front the cornering toe wall holds above 30 kN/mm per end; the braking toe
wall, met with rigid joints, holds only above about 494 kN/mm (0.43 deg at 50 kN/mm), almost all from the
lower-front leg; the node brief now requires pull tests and a re-search with the measured stiffness.
`steering_margin.lock_check()` evaluates the steering at full lock (moment arms about the steering axis, weight
jacking by virtual work): 8.3 N*m at the hairpin. `scripts/yield_full_check.py` re-solves sampled builds: 2000 of
2000 pass at jig tolerance on both final corners; at hand weld the full solves give 78.0% front and 93.2% rear
against 80.5% and 94.0% to first order. The 59.7% and 93.3% quoted earlier were those of front_v11_hairpin_walls
and rear_v11_walls. `scripts/review_checks.py` gathers the three.

**A start that breaks a wall is searched from, for any corner.** `GenesisManifest.start_search` declares the
pattern search (`genesis_seed.SeedSearch`) as part of a manifest: `run()` first moves the declared hardpoints until
every wall holds, then runs InverseGenesis from there, and `verify()` checks that a re-run repeats the start
search's evaluation count, wall result and moves. It enters the inputs hash only when declared, so every earlier
manifest keeps its hash. `GenesisManifest.restarted()` copies a manifest's targets, boxes, walls, tolerance and
search settings onto another start. `GenesisManifest.search_start()` gives the hardpoints the search itself started from, which
every candidate's `shift_vec` is measured from. `genesis_seed.walls_broken()` names the walls a start breaks. In the browser
studio, Inverse Genesis offers the start search (on by default) whenever the start breaks a wall, records it in the
manifest and reports its moves; before, such a run ended NO_FIT. Keep-outs, spacings and bolt access now enter the
pattern search's cost by depth (`SeedSearch.graded_keepouts`, default on), so it can walk out of them; the
published searches were made with the earlier flat penalty and declare `graded_keepouts: false`. The two searches
from a failing start (`front_v11_from_85mm`, `front_v11_from_delivered`) are re-recorded with the start search
declared, so a re-run repeats both stages; their earlier records are in `manifests/superseded`.

**The steering margin for any front manifest.** `suspension/steering_margin.py` takes the loads, effort limit and
ratio from a manifest's own `SteerEffort` and geometry: `margin()` gives the hand torque, the trail limit and the
mechanical trail each tire needs at an assumed trail; `offset_check()` re-evaluates every wall and band with the
spindle moved along x; `offset_for_trail()` finds the offset for a trail. `scripts/steering_margin.py` keeps only this
car's declarations.

**Re-run checks say what they compare.** `GenesisManifest.verify` is not a byte comparison: coordinates must agree to
1e-6 mm and fits and yields to 1e-9. Its docstring and the app's re-run message now say so.

**The steering-effort margin and a pre-verified alternative upright.** `scripts/steering_margin.py` states the
final front's margin on the 10 N*m effort wall at each tire level (declared set; R25B shape at road friction 1.43
and 1.55; the car tire's belt friction) at the brush-theory trail and at a 25.4 mm (1 in) trail, and re-evaluates
the final front with the spindle 2.0 mm forward of the kingpin axis (mechanical trail 12.7 to 10.7 mm; caster, kingpin and every
pickup unchanged): every wall and band holds and a 25.4 mm trail stays under 10 N*m on the declared set and at road
friction 1.43. The 3.1 mm offset the R25B shape at 1.55 would need breaks the wheel-clearance wall, and the script
reports it. The node brief carries the alternative upright.

**The search from a failing start.** `scripts/design_from_delivered.py` declares every input of the final
front with a different start (the delivered front, or the 85 mm-base ChassisV11 candidate) and runs the two
recorded stages: `genesis_seed` until every wall holds, then InverseGenesis. From the 85 mm candidate, which fails
the link and braking-toe walls, it reaches a front that meets every wall, RESILIENT at 100%
(`front_v11_from_85mm`); from the delivered front it stalls at 1.30 times the band and InverseGenesis returns
NO_FIT (`front_v11_from_delivered`): the search is local. `SeedSearch.walls_only` stops the first stage as soon
as the walls hold, wherever the curves are. The lock solve is shared between the lock, swing and misalignment
properties (the same numbers, computed once).

**A damper-length wall and a braking compliance wall.** The actuation synthesis
can now be told which damper it is placing (`ActuationTargets.damper_extended_mm`
with `stroke_mm`): the spring may not top out before full droop and must stay
inside the usable stroke at full bump. Left unset, targets and residuals are as
before. InverseGenesis gains `compliance_toe_braking_deg`, the elastokinematic
toe under a second declared load case (`SolvedPropertyBounds.elasto_braking`),
written to a manifest only when declared so earlier hashes stand.
`robustness.overtravel_check(..., damper_extended_mm=)` counts the damper's
margin to top-out at full droop instead of treating the whole stroke as usable.
`scripts/final_corners_v3.py` declares the final corners with both walls (an
Öhlins TTX25 MkII FSAE, 200 mm / 57 mm; front braking toe within ±0.25 deg,
reached by moving the lower-front pickup along its own pivot axis, which leaves
every kinematic curve unchanged). The ride-frequency default is now the derived
2.6 Hz front set-up.

**Solved-property bounds are enforced inside the InverseGenesis search.** An
inverse solver is indifferent to any property it is not told about: the four
curve channels (camber, toe, roll-centre height, scrub) never carried
anti-squat, anti-dive, roll-centre migration rate, caster or kingpin
inclination, so the engine would happily return a corner that hit every drawn
curve at -5 deg of caster or -102% anti-squat. Those properties could be
computed afterwards and the field screened, but a screen can only report that
nothing it happened to produce survived; it cannot steer the search.

`PropertyBound` and `SolvedPropertyBounds` now put them on the same wall as
the keep-out volumes. A declared range is evaluated on every trial geometry
and a step that leaves it is refused, raising lambda and retrying, exactly as
a keep-out violation is refused — a constraint, not a penalty. Anti-dive and
anti-squat need a vehicle to be meaningful, so the CG height, wheelbase and
brake/drive bias are declared alongside the bounds and travel into the
manifest with them.

Three honesty properties are preserved rather than assumed, and are pinned by
tests. The wall is absolute in both directions, so it refuses a step out of
the band and does not walk in from outside it: with bounds declared the
multi-start draws property-feasible starts (deterministically, from the same
seed) and, when it cannot find any, the result says the bound is unreachable
*from this legal volume* rather than unreachable full stop. A non-finite
property — a side-view instant centre at infinity — counts as a violation,
not as a free pass. And declaring no bounds is bit-for-bit the previous
behaviour: nothing is computed, nothing is enforced, and the solver pays
nothing, so every existing run reproduces exactly.

Only the properties actually bounded are evaluated, which is what keeps this
affordable inside the loop; the report gains a delivered-vs-bound-vs-margin
table for the winner and counts property refusals separately from keep-out
refusals, because "a pickup sits inside the exhaust" and "this corner would
deliver -51% anti-squat" are different facts about the design.

New: `suspension/inverse_genesis.py` (`PropertyBound`, `SolvedPropertyBounds`,
`properties_of`, `SOLVED_PROPERTIES`, `LegalVolume.properties`,
`LegalVolume.property_violations`, `Candidate.property_rejections`,
`Candidate.properties`, `GenesisResult.property_bounds`) and
`tests/test_inverse_genesis_property_bounds.py`.


## What changed in this build (`0.33.0-dual-eda`).

**PCB Doctor reads three formats.** KiCad `.kicad_pcb` (v5–v10), Altium/Protel
ASCII, and native binary `.PcbDoc` with no export step — which matters because
Altium saves binary by default, so it is the file a member actually has. One
`PcbBoard` model behind all three, so every check, the viewer, the fix engine
and the report stay written once. Binary is read-only on purpose.

**Seven silent wrong-answer bugs fixed**, none of which raised an error and none
findable without real files: Altium net IDs numbering from 0 in some exports and
1 in others; KiCad 10 dropping numeric net IDs, so v10 boards parsed as zero
segments and read as empty files; teardrops faking pours and suppressing real
copper-open findings; pads inheriting the wrong copper side; endpoints 11 µm
apart splitting a routed net into islands; tracks landing on a via's annulus
rather than its centre; and a `raise warning '%s'` printing "feature pcb**s**".

**Copper-open false alarms 8.0% → 1.3%** across fifteen real boards, in measured
steps with no board regressing: endpoint welding at 50 µm, mid-trace pad
attachment, and via-annulus attachment using the via's own radius from the file
rather than a tuned tolerance.

**Undeclared currents report MISSING** instead of being assigned a default and
failed against it. A working commercial board previously came back 12 FAIL /
10 WARN, none of it actionable.

**Per-team analytics.** Events carry a `workspace_id`, with back-fill for events
emitted before the workspace resolves. New views: `v_workspace_weekly`,
`v_workspace_adoption`, `v_feature_traction`.

**Three database fixes**, one of them a cross-workspace access hole — see
Database setup above, and run them in the order given.

**UI.** Dual-format upload identified by content rather than extension;
`diagnose()` and `board_svg()` cached on a fingerprint (≈10 s of dead UI per
click on the largest board, now 0.7 ms); viewer suppressed above 6000 segments
because the largest real board emits 6.6 MB of inline SVG; upload ceiling 24 →
64 MB after a real 27 MB board was rejected; an unreadable file no longer hides
the Trace Prescriber. Architecture Synthesis Pareto chart now plots the same
deduplicated rows as its own table — it was drawing the raw front, so three
table rows became six markers with every label doubled and clipped.

---

## What changed in this build (`0.32.0-omnicore`)

**🪐 OmniCore — new tab (Design & Sizing)**
- New `suspension/omnicore.py`: the vehicle-synthesis referee. `parse_mission` is a deterministic grammar over the mission sentence — budget regex taking the largest money-looking number (a `±2 mm` tolerance is not money), stated tolerance → the worst shop class whose declared accuracy still covers it (`±2 mm` → hand_weld, with `weld…pull` context read as systematic tab pull), terrain words → the transient manoeuvre, priority words → knee weights, drive words disclosed as carried-not-varied — with a full receipt of every token consumed (and what it meant), every default assumed, and every word ignored; same text in, same spec out, forever. `run_omnicore` fans the mission over shop × actuator × volume-fraction sharing sub-solves on purpose (one InverseGenesis per shop with the mission's weld pull in that shop's `ToleranceField`; one SimulForge co-solve + Ghost transient audit per actuator; one MorphMesh growth per triple, at printed screening fidelity), reuses every engine's own verdicts unchanged (KNIFE_EDGE and authority become flags; NO_FIT, brownouts, UNBUILDABLE and the mission budget become **named vetoes**), assembles the five-axis scorecard (∫|roll|dt composure, pack-vs-event-draw laps, structure mass, decision-scope cost, Monte-Carlo build yield), and referees it: hand-checkable `pareto_mask`, priority-weighted `knee_pick` over the feasible set, and `dominance_receipts` naming the exact configuration that beats every dominated one on all axes at once. Engine failures veto by name — flagged, never raised — and the engine-call ledger is printed. The self-healing twin: `twin_baseline` declares the banded healthy telemetry envelope (V_min, sag, peak draw, lag, event energy, authority, peak roll), `defect_signatures` re-runs the manoeuvre once per Degradation preset (the Saboteur's signature idea pointed at the running car), `diagnose` band-normalises measured drift and cosine-matches it (≥ 0.75 names a suspect with magnitude, below it says "matches nothing" honestly; inside-band drift names nobody), and `heal_plan` is closed-form arithmetic: gain de-rate from the deliverable moment at the measured sagged bus (I = duty·V/R, M = Kt·gear·eff·I capped, gains scaled so the peak nominal command fits — predictability, not performance, and a hardware de-rate is called out as unfixable by gains), plus a printable camber shim t = s·tan|θ| pointing at the Stochastic tab's verified Alignment Prescription. Markdown reports for both halves. Pure Python + NumPy over the engines already in the repo, headless, deterministic, self-tested (`python3 -m suspension.omnicore`).
- New `ui/omnicore.py` under the `ui/` strangulation pattern: the mission text box with the flagship prompt as placeholder, the parse receipt (consumed / assumed / ignored) shown before anything runs, sweep knobs (compare shops, tab count, events per lap, base cost, bands), live progress through the lattice, the front figure (cost × composure, marker size = mass, colour = laps, Pareto ring, knee star, vetoed configs greyed with their veto on hover), the full scorecard table, named vetoes and flags per configuration, dominance receipts, the engine-call ledger, JSON + markdown downloads, and the twin panel: build baseline + signatures, type the measured channels, get the banded verdicts, the named suspect (or the honest miss), and the heal plan with new gains and the shim spec. Reads the live hardpoints when the Kinematics tab has set them — one geometry, every engine, one referee, on purpose.
- Registered in the lazy public-API facade (`suspension/__init__.py`): `omnicore` submodule plus `MissionSpec`, `parse_mission`, `OmniKnobs`, `OmniResult`, `run_omnicore`, `render_omni_md`, `twin_baseline`, `defect_signatures`, `twin_diagnose`, `heal_plan`, `render_twin_md` — the lazy-init sync test stays green.
- Wired into the shell (tab meta, the Design & Sizing category next to MorphMesh, the suspension / electrics / chassis role tabs, both description maps, the new `vg_referee` verify-goal, the feature bullets, and the render container) following the strangulation pattern; a broken tab is isolated in `try/except` and can never take the studio down.
- `tests/test_omnicore.py`: 28 tests pinning the grammar (flagship prompt field-by-field, determinism, the receipt accounting for every word class, an empty prompt disclosing every default, tolerance-is-not-money, `$15k`, the accuracy → shop-class boundaries, unknown words landing in `ignored`), the Pareto arithmetic on hand cases (strict domination, trade-offs keeping both, ties surviving, axis senses respected), the knee actually listening to mission priorities, dominance receipts naming the dominator and staying silent on a clean front, the twin (a 0.8×-scaled copy of a signature matched at cosine ≈ 1 with the magnitude read back, inside-band drift naming nobody, orthogonal drift matching nothing honestly, the NOMINAL/WATCH/DEGRADED band ladder), the heal algebra against its closed forms (t = s·tan|θ| both signs, the gain de-rate from first principles, the screening caution always printed), the flagged-not-raised doctrine with every engine broken on purpose (vetoes by name, no exception, an empty front warned not thrown), and one coarse end-to-end sweep with rendered reports.

*(Previous build `0.31.0-morphmesh` below.)*

## What changed in this build (`0.31.0-morphmesh`)

**🕸️🔩 MorphMesh — new tab (Design & Sizing)**
- New `suspension/morphmesh.py`: the structural auto-synthesizer. `load_fan_from_audit` reads the per-instant member force straight off a Ghost Topology transient audit — magnitude *and* the deformed link line it acts along — and compresses it into ≤ N exposure-weighted `LoadCase` arrows on the chassis tab: exposure-weighted best-fit bracket plane (dropped out-of-plane share reported, never hidden), wrap-safe angles (the fan is rotated so the exposure-weighted mean direction sits at 0°, so a fan hugging the ±180° branch cut cannot read as a 360° span), and sign-partitioned grouping so a tension↔compression flip keeps its own opposing arrow with the reversal named — a direction-static event collapses to one case on its own. `FabricationLimits` derives the shop's veto from the same class names Stochastic Inversion presets carry (hand_weld / jig_weld / cnc: minimum rib = web floor + 2× positional accuracy, a stricter stated-heuristic rule inside the HAZ band) or reads a declared `ToleranceField`'s own half-span as the accuracy. `PlateDomain` declares the sheet (chassis-tab and pivot-bracket presets: anchors, load bore, keep-in boss ring — a boss thinner than the minimum rib is grown and the finding named). `morph_component` runs the enforce → grow → binarise → **measure** → accept-or-reject-and-coarsen loop: a multi-case SIMP growth (Q4 plane-stress elements derived by quadrature, density filter → smoothed-Heaviside projection with β-continuation, OC update bisected on the *physical* volume, one factorisation shared by all cases per solve) minimises exposure-weighted compliance against the whole fan at once; the binarised shape is then measured by morphological opening (edge-replicated padding so an anchor-flush weld foot is never falsely eaten) against the declared limits, and a shape with ribs the floor can't hold is **REJECTED** and re-grown coarser, up to four rounds, with the **premium of buildability** (compliance given up, feature size gained) printed when coarsening was forced. The survivor is re-solved at peak fan amplitudes and screened by von Mises against the standing 1.5-on-yield rule (a failing screen prints the linear thickness fix), and ships with the exposure-weighted stress-path map, mass, cells CSV, a pixel-exact outline for the CAD seat, JSON summary, and a markdown report carrying the full rejection ledger. Verdicts FORGEABLE / COARSENED / UNBUILDABLE / LOAD_STARVED / SOLVER_LIMITED — flagged, never raised. Pure Python + scipy, headless, deterministic, self-tested (`python3 -m suspension.morphmesh`).
- New `ui/morphmesh.py` under the `ui/` strangulation pattern: event / corner / member selectors, plate-domain presets with editable dimensions and mesh size, the shop-veto editor seeded from the shop class with every number a knob, growth-budget controls, verdict banner with the per-verdict blurb, metrics (mass, worst FoS with its governing case, shop rejections, FE solves), the premium-of-buildability callout, the grown-shape figure (density field, pixel-exact outline, stress-path whiskers, anchors + HAZ band, the load fan drawn at the bore), the fan table, the shop's rejection ledger, findings, and four downloads (cells CSV, outline CSV, JSON, report). Builds the `GhostCorner` exactly as the Phantom Envelope tab does and reads the live hardpoints when the Kinematics tab has set them — one geometry, another consumer, on purpose.
- New `demo_morphmesh.py`: the end-to-end story — a curb strike audited, the fan condensed, the tab grown for the hand shop (a real round-1 rejection and the printed premium), the same tab re-grown for a CNC shop with the compliance delta named, and the deliverables.
- Registered in the lazy public-API facade (`suspension/__init__.py`): `morphmesh` submodule plus `LoadCase`, `LoadFan`, `load_fan_from_audit`, `FabricationLimits`, `PlateDomain`, `fabrication_audit`, `MorphRound`, `MorphResult`, `morph_component`, `morph_from_audit`, `render_morph_md` — the lazy-init sync test stays green.
- Wired into the shell (tab meta, the Design & Sizing category next to InverseGenesis, the suspension and chassis role tabs, both description maps, the vg_build verify-goal next to Stochastic Inversion, the feature bullets, and the render container) following the strangulation pattern; a broken tab is isolated in `try/except` and can never take the studio down.
- `tests/test_morphmesh.py`: 20 tests pinning the Q4 element (symmetry, exactly three rigid-body modes, stiffer-deflects-less), the fabrication audit as a measurement (thick passes / thin fails, the edge-flush weld foot not falsely eaten, the HAZ disk stricter and localised, bosses protected), the shop-class arithmetic and the ToleranceField-declared accuracy, the fan's fidelity to the deformed link line, the wrap fix, the sign reversal keeping its own named arrow, out-of-plane and starved audits flagged not raised, the forgeable growth respecting bores, the reject-and-coarsen contract and its priced premium, UNBUILDABLE naming the binding constraint, the structural screen pricing the thickness fix, a multi-case shape serving both arrows, the end-to-end join, and export round-trips.

*(Previous build `0.29.0-genesis` below.)*

## What changed in this build (`0.29.0-genesis`)

**🧬 InverseGenesis — new tab (Design & Sizing)**
- New `suspension/inverse_genesis.py`: the stochastic inverse engine. `GenesisTargets`/`TargetCurve` state the intent as curves over travel with per-station acceptance bands (zero bands refused out loud — they ask for a probability-zero car); `curves_of` samples every channel from one dense warm-started sweep, failing whole geometries the solver can't follow. `LegalVolume` is the physics-informed boundary filter: per-point boxes (clamped coordinates named), plus keep-out volumes speaking the Phantom Envelope `clearances(points, probe)` dialect — carved envelopes of neighbouring assemblies or the new `KeepOutBox` (exact signed AABB distance) — enforced as a wall: a violating step is rejected, never penalised. `genesis_solve` is the deterministic reverse-gradient core: damped Gauss–Newton on the band-weighted residual, each iteration one central-difference Jacobian through the full nonlinear corner solver, λ-adapted, clamped, keep-out-screened. `inverse_genesis` runs multi-start (deterministic, seeded, deduped by basin) and the build-yield co-optimizer: every curve-hitting candidate priced for yield against a Stochastic Inversion `ToleranceField` through its own sensitivity matrix, with the candidate's fit residual spending band headroom before scatter — knife-edge optima rejected, the resilience premium printed, the winning yield verified by full nonlinear solves with the pass/fail agreement (self-demoting below the floor). Jointly unsatisfiable declarations return the binding band/box/keep-out by name and the closest legal car, never a fabricated optimum. Verdicts RESILIENT / TEMPERED / KNIFE_EDGE / NO_FIT. Pure Python, headless, deterministic, self-tested (`python3 -m suspension.inverse_genesis`).
- New `ui/inverse_genesis.py` under the `ui/` strangulation pattern: per-channel curve editors seeded from the live nominal's own curves (the sheet solves before the first edit) with band floors, movable-point and freedom controls, a keep-out box editor with probe radius and clearance, Stochastic Inversion shop presets for the field, verdict banner with resilience-premium metric, winner shift table, per-channel nominal/generated/band overlay charts, the candidate family table ("what a point of yield costs in fit"), markdown handover, and an apply button that writes the generated `Hardpoints` back to the live session — one geometry, another consumer *and the first producer*, on purpose.
- Registered in the lazy public-API facade (`suspension/__init__.py`): `inverse_genesis` submodule plus `TargetCurve`, `GenesisTargets`, `KeepOutBox`, `LegalVolume`, `GenesisThresholds`, `Candidate`, `GenesisResult`, `curves_of`, `genesis_solve`, `build_yield`, `inverse_genesis_run`, `render_genesis_md` — the lazy-init sync test stays green.
- Wired into the shell (tab meta, the Design & Sizing category next to Stochastic Inversion, the suspension role tabs, both description maps, the susp_geo goal routing with feature bullets, and the render container) following the strangulation pattern; a broken tab is isolated in `try/except` and can never take the studio down.
- `tests/test_inverse_genesis.py`: 24 tests pinning the forward map against the direct solver, intent validation (zero band, duplicate channel, unknown channel refused), box clamps named, `KeepOutBox` signed distance inside/outside/probed, the keep-out wall (a walled-off truth position still returns a violation-free geometry), hidden-geometry recovery, determinism of the solve and byte-identical reports, no-shift on an already-satisfied nominal, yield bounds, the winner never out-yielded by another hit, the fit-residual/headroom coupling (a worse fit can only cost yield), the CNC-out-yields-hand-weld control case, verification pricing, the named unsatisfiable verdict, and the honest fit-only degradation when no field is declared.
- `docs/BOTTLENECKS.md` gains bottleneck **#18 — the design loop that only runs forward**, the first inverse engine to give the packaging reality a wall and the shop a veto.

*(Previous build `0.28.0-stochastic` below.)*

## What changed in this build (`0.28.0-stochastic`)

**🎲🛡️ Stochastic Inversion — new tab (Design & Sizing)**
- New `suspension/kinematik_stochastic.py`: the manufacturing-yield audit and the metrology feedback loop. Asymmetric per-point, per-axis tolerance fields (`ToleranceSpec` lo/hi bounds, uniform or truncated-normal, closed-form mean/variance; shop presets hand-weld / jig-weld / CNC with a directional weld-pull bias applied to the welded wishbone inners only — tie-rod inner and all outboard joints are machining-class, a physics distinction, not a convenience). One central-difference sensitivity matrix (40 corner solves, refused if the nominal itself won't solve) propagates thousands of sampled cars in microseconds, and every linear-mode yield is **priced live** by a full-nonlinear verification subsample whose pass/fail agreement prints with the result — below 98 % it demotes itself with a warning. Verdicts ROBUST / MARGINAL / FRAGILE / SOLVER_LIMITED (perturbed geometries that fail to solve mean the nominal sits near a kinematic singularity). Per-metric fail fractions, expected bias E[Δ] = J·μ, p05/p95 spreads, first-order variance attribution naming the tab that drives each metric. `robust_nudge` solves the band-weighted least-squares nominal shift that re-centres an asymmetric cloud, clamps to declared per-point freedom (clamps named), honestly refuses a centred field, and verifies with full solves judged against the **original** design intent via an explicit target — so the re-centred nominal cannot quietly move the goalposts. `alignment_prescription` takes pasted as-built coordinates (a >25 mm shift is refused as a units/frame slip and never shimmed), the real adjusters (point, axis, range, shim step), solves at the as-built geometry, quantises and clamps the moves, then **re-solves the shimmed car in full** so the printed residual is the one the car will carry; metrics outside the adjusters' reach are named unreachable. RESTORED / PARTIAL / UNSHIMMABLE. Pure Python, headless, deterministic (seeded), self-tested (`python3 -m suspension.kinematik_stochastic`).
- New `ui/kinematik_stochastic.py` under the `ui/` strangulation pattern: shop-preset + weld-pull error-field editor with every per-point bound editable, acceptance-band and engine controls, verdict banner with per-verdict blurb, four metrics (yield, verification agreement, worst metric, solver failures), and four sub-tabs — per-metric spread, variance attribution, the robust nudge with freedom and verification controls plus a markdown handover, and the Alignment Prescription with a CSV as-built parser, adjuster editor, verified shim table and download.
- Registered in the lazy public-API facade (`suspension/__init__.py`): `kinematik_stochastic` submodule plus `ToleranceSpec`, `ToleranceField`, `YieldSpec`, `StochasticThresholds`, `StochasticResult`, `Sensitivity`, `sensitivity`, `stochastic_sweep`, `RobustNudge`, `robust_nudge`, `Adjuster`, `Prescription`, `alignment_prescription`, `render_stochastic_md`, `render_prescription_md` — guarded by the existing lazy-init and public-API export tests, still green.
- Wired into the shell (tab meta, the Design & Sizing category, the suspension role tabs, both description maps, the verify-goal list, and the render container) following the strangulation pattern; a broken tab is isolated in `try/except` and can never take the studio down.
- `tests/test_kinematik_stochastic.py`: 28 tests pinning the field moments and bounds, sampler determinism, preset physics (tie-rod inner is machining-class), the Jacobian against direct solves and the bump-steer↔tie-rod coupling, byte-identical reports across runs, yield monotonicity in field size, the linearisation's printed price and its agreement with full mode, attribution normalisation, every verdict boundary, the centred-field refusal, the nudge's bias re-centring and the goalpost fix (verified-vs-original ≈ predicted), freedom clamps named, the prescription restoring an injected 1.4 mm weld error to RESTORED, shim quantisation and clamping, an unreachable metric named, the metres-as-mm refusal, and adjuster axis normalisation.
- `docs/BOTTLENECKS.md` gains bottleneck **#17 — the car nobody welds**, the first audit to treat hardpoints as the distribution the shop actually delivers and to close the metrology loop back to a shim stack.

*(Previous build `0.27.0-ghost` below.)*

## What changed in this build (`0.27.0-ghost`)

**👻🔩 Ghost Topology — new tab (Design & Sizing)**
- New `suspension/ghost_topology.py`: the deformed-geometry audit — the join that closes the tyre-compliance loop the siloed rigid-kinematics → static-FEA workflow leaves open. Walks a transient overload and solves the ghost topology at each audited instant: geometry drift vs rigid intent at that instant's travel (camber, toe, instant-centre, roll-centre height, contact patch), member load-path migration (same wheel load through rigid vs ghost geometry, share shift summing to zero), transient FoS per member on yield **and** pinned-pinned Euler buckling (the honest column for a spherical-jointed two-force member), and the tyre-force feedback loop gain **measured** by contraction — |g| < 1 gives the geometric-series closed-loop force with the gain reported as the stability margin, |g| ≥ 1 is flagged divergent with open-loop values, no fabricated fixed point. Verdicts worst-first: FEEDBACK_DIVERGENT / COMPLIANCE_INVERTED / MARGIN_BREACHED / COMPLIANCE_DEGRADED / RIGID_FAITHFUL. The zero-FEA trick is time-scale separation, stated and priced: the quasi-static compliance solve evaluated algebraically along the load history, 25 N-quantized solve cache, a few solves per event. It prices its own limits — sub-5 ms load edges flagged per instant as a structural-dynamics question it refuses to answer, a member past yield voiding the elastic geometry beyond that instant. The body→corner sign mapping is applied once, documented, and a failed transient is refused rather than audited as zeros. Pure Python, headless, deterministic, self-tested (`python3 -m suspension.ghost_topology`).
- New `ui/ghost_topology.py` under the `ui/` strangulation pattern (physics stays in `suspension/`, this only orchestrates and draws): manoeuvre picker (step steer / snap oversteer / brake→throttle / curb strike), corner picker with the sign-mapping help, audited-instants slider, link tube / material / yield and chassis-tab-stiffness editor, tyre-sensitivity overrides, the verdict banner with per-verdict blurb, four metrics (worst FoS, peak Δcamber, Δtoe, loop gain with the margin-to-instability delta), the findings list, and four sub-tabs (geometry vs intent, transient margins vs the 1.5/1.0 lines, load-path shift at the worst instant, the instant table) plus a markdown handover export. Reads the live hardpoints when the Kinematics tab has set them.
- Registered in the lazy public-API facade (`suspension/__init__.py`): `ghost_topology` submodule plus `GhostCorner`, `ghost_audit`, `ghost_audit_transient`, `MemberSection`, `TireSensitivity`, `GhostThresholds`, `GhostAudit`, `GhostInstant`, `render_ghost_md`, `uniform_sections` — guarded by the existing lazy-init and public-API export tests, still green.
- Wired into the shell (tab meta, the Design & Sizing category, the render container) following the strangulation pattern; a broken tab is isolated in `try/except` and can never take the studio down.
- `tests/test_ghost_topology.py`: 24 tests pinning the closed forms (Euler Pcr, yield FoS, buckling-governed compression), the roll-centre construction, the feedback sign paths, the fixed-point self-consistency and the measured-divergence flag, stiffness monotonicity, load-path bookkeeping, the travel baseline and its clamp, every verdict boundary (stock corner RIGID_FAITHFUL, soft corner MARGIN_BREACHED at the load peak, very-soft corner COMPLIANCE_INVERTED outranking margin), the solve cache with per-instant timestamps, the fast-edge quasi-static flag, the transient sign mapping, failed-transient refusal, and the markdown report.
- `docs/BOTTLENECKS.md` gains bottleneck **#16 — the rigid lie under load**, the first audit to close the tyre-compliance feedback loop.

*(Previous build `0.26.0-fusebox` below.)*

## What changed in this build (`0.26.0-fusebox`)

**⛓️ Fusebox — new shared-spine tab (Checks & Integration)**
- New `suspension/fusebox.py`: the failure-order audit. Overload chains as ordered element lists (FoS at the element's own load share, evidence grade + staleness pricing σ via the exact Proof Engine band law — one pedigree law, fifth consumer), first-failure probabilities from deterministic fixed-grid quadrature of the minimum of independent normals with the pairwise closed-form printed for napkin checks, verdicts FUSED / COIN-FLIP / INVERTED / UNFUSED / BREACH-RISK against a sha256-sealed Fuse Charter (one confidence, one forbidden-first tolerance, designations per path), three exact fix levers per rival (soften-the-fuse floored at FoS 1.10, stiffen-the-rival, sharpen-the-grade — with honest infeasibility messages when z·r ≥ 1 means no metal can fix an unknown), the probability-weighted overload bill in $ and days, incident judging (AS-DESIGNED / SURPRISE / BREACH, every break banking the free capacity datum; a tampered charter refuses to judge), four seeded FSAE-EV archetypes whose grades tell the true story, and a pinnable markdown Fuse Map. Pure stdlib, deterministic end to end, self-tested (`python3 -m suspension.fusebox`).
- New `ui/fusebox.py` under the `ui/` strangulation pattern: pecking-order board with per-path expected bills, path & element editor (seeds resettable), fix-arithmetic panel, charter sealing + incident judging.
- Wired into all seven shell registries (tab meta, category grouping, shared spine, both description maps, the how-it-works bullets, and the render container); a broken tab is isolated in `try/except` and can never take the studio down.
- `tests/test_fusebox.py`: 27 tests pinning the quadrature against the closed form, determinism (byte-identical probabilities and markdown), the flagship 27 % coin flip, every verdict at its documented constants (BREACH-RISK outranking all, empty paths and missing severities as blind spots never passes), fix roots satisfying the pairwise equation to display precision, the fuse floor, the no-metal-fixes-an-unknown infeasibility, seal tamper-refusal, and incident verdicts with the free datum.
- `docs/BOTTLENECKS.md` gains bottleneck **#15 — the failure order nobody chose**, the first physical-consequence audit alongside the four epistemic ones.

*(Previous build `0.25.0-phantom` below.)*

## What changed in this build (`0.25.0-phantom`)

**👻 Phantom Car — new shared-spine tab (Checks & Integration)**
- New `suspension/phantom_car.py`: the margin audit. Each consumer *discloses* the design value its sizing actually uses (a built-in FSAE-EV consumption map seeds the form at nominal on purpose); every hedge is priced in σ of that quantity's own evidence-graded, staleness-inflated band — the exact Proof Engine ledger, third consumer, zero new physics. One sha256-sealed Margin Charter percentile judges the lot: **ALIGNED / STACKED** (excess priced as releasable envelope in the quantity's own units) **/ UNDER-COVERED / NAKED** (naming the evidence grade it's naked on) **/ ANTI-HEDGED**. The **two-cars detector** flags assumed values >1σ apart on the same quantity (the Integration contradiction check, applied to *assumptions* for the first time); **β = √(Σz²)** is the FORM reliability index, stating the odds each stacked load case defends against; and the **three-cars comparison** (nominal / coherent-percentile / phantom) prices, per objective, the design envelope currently spent defending cars the deck's own σ says cannot exist — reported honestly as envelope, never as promised savings. A charter with a broken seal refuses to judge, out loud. Pure stdlib, deterministic end to end, self-tested (`python3 -m suspension.phantom_car`).
- New `ui/phantom_car.py` under the `ui/` strangulation pattern: sealed-charter panel, disclosure editor (seed / demo / hand-edit), verdict board with releasable-and-exposed pricing, the two-cars detector, β per consumer, the three-cars table per objective, honest blind-spot and unresolved-key reporting, and a pinnable Margin Docket markdown export. Shares the `proof_pedigree` session map with the Proof Planner and the Saboteur — one pedigree, three consumers, on purpose.
- Wired into all seven shell registries (tab meta, category grouping, shared spine, the two description maps, the how-it-works bullets, and the render container) following the strangulation pattern; a broken tab is isolated in `try/except` and can never take the studio down.
- `tests/test_phantom_car.py`: 22 tests pinning determinism (audit and docket byte-identical), the verdict thresholds at their documented constants (ALIGNED at the charter percentile, NAKED at nominal, STACKED with releasable envelope, ANTI-HEDGED on a favourable value), the two-cars detector (contradiction named with both consumers, quiet within 1σ), β = √(Σz²) over stacked worst cases only, the phantom as the union of adverse extremes, seal tamper-evidence (an edited charter refuses to judge and computes nothing else), and honesty (unresolved keys and unaudited consumers reported, seeds never fabricating prudence).
- `docs/BOTTLENECKS.md` gains bottleneck **#13 — the margin nobody adds up**, the conservatism mirror-image of #12, closing the triangle with the Proof Engine and the Saboteur.

*(Previous build `0.24.0-saboteur` below.)*

## What changed in this build (`0.24.0-saboteur`)

**🧨 Saboteur — new shared-spine tab (Checks & Integration)**
- New `suspension/saboteur.py`: mutation catalog (thousandfold/imperial unit slips, frame Z flip, kilo-prefix slips, dropped & double-counted roll-up terms, each with its real-world story), sabotage sweep over a shadow copy of the uncertainty ledger, silent-killer classification against the objective's own 3σ envelope, tripwire catalog with per-wire deck-consistency tolerances and real-world read instructions, greedy detectability set-cover (runs until nothing catchable remains; a caller-imposed cap charges its victims to the blind-spot list), sha256-sealed pre-flight sheets, cosine fingerprinting of tripped patterns against predicted corruption signatures, honest-blind-spot reporting, markdown export. Pure stdlib, deterministic end to end, self-tested.
- New `ui/saboteur.py` under the `ui/` strangulation pattern: kill board, coverage before/after, sheet sealing, judge-a-run panel with named suspects. Shares the `proof_pedigree` session map with the Proof Planner — one pedigree, two consumers, on purpose. Can optionally judge the sweep against an open validation contract's acceptance band, answering: *could garbage hand you a PASS?*
- `tests/test_saboteur.py`: 22 tests pinning determinism, per-class detection (z-flip caught by the CG wire, lb·ft by torque-per-power, dropped terms by rolled-up mass), honesty (blind spots reported and charged, unavailable wires never offered, verified pedigrees never shrink tripwire tolerances), seal tampering refused, fingerprint correctness (injected corruption identified at cosine > 0.99, magnitude 1.0×), and uncatalogued errors admitted rather than misattributed.
- `docs/BOTTLENECKS.md` gains bottleneck **#12 — the garbage that flatters the envelope**, closing the residual #8 left open.

*(Previous build `0.23.0-frames` below.)*

## What changed in build (`0.23.0-frames`)

**🧭 Frames & Datums — new shared-spine tab (Checks & Integration)**
- New `coordinate_frames.py`: frame registry (ISO 8855, SAE J670, ISO 4130, KinematiK internal, SolidWorks-typical, custom-from-words with derived +z guaranteeing right-handedness), exact point/vector/rotation-sense transforms via one `frame → world → frame` path, floating datums resolved live from wheelbase / weight split / CG height, datum-drift detection, CSV + SolidWorks Curve-Through-XYZ I/O, sign-convention linter (below-ground / mirror asymmetry / unit sniff / envelope), judge-ready charter markdown. Pure Python, streamlit only imported inside `render()`, exact-identity self-tests.
- Tab body wired with hard isolation (`try/except`) so the convention tool can never take the studio down; live hardpoint provider filters the session hardpoint dict to 3-vectors and maps keys to human labels.
- Declaring a charter logs a Decision (`team=integration`, tags `coordinates,standard`) so the convention and its rationale survive into the Registry and next season's handover.

**Frame tags on everything leaving the platform**
- `_generic_dxf_bytes` annotation block now stamps the declared convention onto every generic DXF (aero sections, mount plates, radiator faces, brackets, gussets); the brake-rotor DXF (which builds its own R12 file) stamps the same line.
- `project.build_handover_markdown` gained `frame_tag=""` and renders a **Coordinate convention** section before the weight budget; the app passes the long-form tag, which explicitly reads **UNDECLARED** when no charter exists — formal documents never silently omit the convention.
- Integration tab banner states the declared convention above the ledger, or nudges to declare one.

**Hardpoint editor mislabel fixed**
- The editor header claimed "SAE x-rear y-right z-up". SAE J670 is Z-**down**; the internal frame is ISO 4130-style. Header corrected and now points to Frames & Datums for conversion — the tool no longer commits the exact mislabel the tab was built to end.

*(Previous build `0.22.0-unified` — team CAD library ⇄ 3D model quick-assembly preview, mission briefing onboarding, `v_retention` two-phase identity rewrite, `v_time_to_first_result` anchor fix, visitor identity fixes, `session_start` deferral — see Git history for the full notes.)*

---
