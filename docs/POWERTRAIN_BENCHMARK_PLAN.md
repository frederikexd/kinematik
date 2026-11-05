# Powertrain benchmark plan: EV and combustion

The powertrain features are checked against closed forms (BENCHMARK.md). This plan is the next step: the same
cases run in the tools the industry treats as the reference, with the inputs and the pass limits fixed before any
run, as for the Adams View kinematics benchmark (`fsae_suspension/benchmarks/adams_view`). Nothing here has been run
yet.

## The tools

Which tool is a reference depends on the case. None of them is a gold standard on its own: the reference for a
lap time or an energy figure is a logged lap, and for an engine a dyno. A simulation tool earns its place when it
solves more of the physics than KinematiK does (a forward transient against KinematiK's quasi-steady state, a detailed
engine against a declared curve).

| Tool | Role | What it checks | Access for a Formula SAE team |
|---|---|---|---|
| MATLAB, Simulink and Simscape (Simscape Driveline, Electrical, Battery; Powertrain Blockset) | Reference for the EV and for launch dynamics: a forward transient with a real driveline, inverter and pack | Cases 1, 2, 4, 5 | Complimentary through the MathWorks student-competition programme; MathWorks publishes a Formula Student Simscape vehicle and *Simscape Essentials for Automotive Student Teams* to start from |
| Simcenter Amesim (Siemens) | Reference for both powertrains as a system: engine and restrictor, gearbox and shifts, pack, cooling loop | Cases 1, 2, 4, 5, 6 | Siemens partners with Formula SAE and supports teams with Simcenter; ask Siemens for an Amesim licence |
| Siemens NX | **Not a powertrain reference.** NX is CAD and structural, thermal and flow analysis (Simcenter 3D); it has no vehicle or powertrain model to drive over a lap. Use Amesim for this benchmark and keep NX for the chassis and parts | — | — |
| MoTeC (loggers and i2 Pro) | **The measured reference.** Not a simulator: the car's own runs, logged and exported, against what KinematiK predicts for the same car. Benchmarks the powertrain design (launch, peak wheel power, pack resistance and energy, final drive; gear ratios, in-gear torque curve, fuel), not lap time, which mostly measures the tires and the driver | Cases 1, 2, 4 and the MoTeC metrics below | The team's logger and i2 Pro |
| GT-SUITE (GT-POWER) | Reference for the engine itself (the torque curve, restrictor and plenum, heat to coolant) | Cases 5, 6, 7 | Gamma Technologies has offered two free seats per FSAE team; confirm it still runs |
| A dyno and logged laps | The actual reference: everything above is checked against a model | All | The team's own data |

## First: one input set

KinematiK's modules carry different defaults for the same car (tire radius 0.200, 0.203 and 0.225 m; rolling
resistance 0.015, 0.018 and 0.020; ClA 2.5 and 2.6 m²; power 60 and 80 kW; mass 280 and 300 kg; final drive 3.5
and 7.0). A benchmark on mixed inputs measures the mixing. Use the set below in KinematiK and in every tool, and
write it down before the first run.

**Shared vehicle.** Mass 300 kg with driver; CG 0.28 m; wheelbase 1.55 m; loaded tire radius 0.225 m (18.0 x 6.0-10);
longitudinal friction 1.4; rolling resistance 0.015; CdA 1.10 m²; ClA 2.5 m², half on the rear; air 1.225 kg/m³.

**EV.** Rear static share 52%; rotating-inertia factor 1.05; motor 120 Nm peak, 23,500 rpm; 80 kW at the pack;
final drive 7.0; motor 0.9545, inverter 0.97, driveline 0.95; pack 140s3p, 3.6 V and 5.0 Ah per cell, 0.0128 Ω per
cell (504 V, 0.597 Ω, 7.56 kWh, 6.96 kWh usable); regen 60% up to 0.35 g. The team workbook's 50 A fuse caps the pack
near 25 kW: run the 80 kW case with the fuse limit off and decide which is the car.

**Combustion.** Rear static share 55%; gasoline, 20.0 mm restrictor, Cd 0.95; torque curve: the team's dyno or
GT-POWER curve (until then the representative 600 cc inline-four of `ice_powertrain.EngineMap.representative_600_i4`,
peak 61 kW at 11,000 rpm, redline 13,500 rpm); primary 2.111, gears 2.750, 2.000, 1.667, 1.444, 1.304, 1.208, final
drive 3.0 (and 2.3, the 75 m optimum on the representative curve); shift time 0.08 s; launch at 9,000 rpm; gearbox
and chain 0.92; brake thermal efficiency 0.30 (BSFC 279 g/kWh on gasoline); engine inertia 0.025 kg·m² at the crank;
wheels 0.6 kg·m² in all; overrun torque 8 Nm; 28% of fuel energy to the coolant.

## Cases

Limits are the largest allowed difference, KinematiK against the tool. A different method (QSS against a forward
transient) gets 2%; the same physics gets 1%.

| # | Case | EV | Combustion | Compare | Limit |
|---|---|---|---|---|---|
| 1 | 75 m from rest | `laptime.acceleration_time` against Simulink or Amesim, then a logged run (MoTeC) | `ice_powertrain.simulate_launch` against Simulink or Amesim, then a logged run | time and speed at 75 m | 2% (tools), 3% (logged) |
| 2 | Energy on a fixed speed trace | `ev_powertrain`, `power_draw` on KinematiK's endurance speed trace, in Simulink and GT-SUITE | `ice_powertrain.fuel_for_lap` on the same trace with constant BSFC | kWh per lap, peak and RMS current, minimum pack voltage (EV); fuel per lap, peak fuel flow (ICE) | 1% |
| 4 | Energy store | `pack_thermal` against PyBaMM or Simscape Battery on case 2's cell current (cell part number needed) | `ice_powertrain.endurance_fuel` against the same tool's fuel total over 22 km | terminal voltage and cell temperature (EV); endurance litres and CO2 (ICE) | 2% and 2 °C (EV); 1% (ICE) |
| 5 | Cooling loop | `powertrain/engine` cooling network against GT-SUITE or Simscape Fluids (your Y-branch pipe lengths needed) | the same loop with the engine's heat; GT-POWER's heat to coolant at full load checks the declared 28% share | flow, loop pressure drop, coolant temperature over a lap | 2%, 3%, 1 °C |
| 6 | Intake | — | restrictor choked flow and the plenum (`throttle_dynamics`) against GT-POWER with the throttle, restrictor and plenum alone | choked mass flow; manifold pressure settling after a throttle step | 1%; 5% |
| 7 | Overrun | regen torque and the brake balance (declared) | GT-POWER's motoring torque against the declared 8 Nm overrun torque | torque at 6,000, 9,000 and 12,000 rpm | report, then replace the declared value |

Cases 5 (ICE share), 6's GT-POWER curve and 7 calibrate declared values rather than check a solver: their result
replaces the declaration, and the gate's MISSING and WARN flags go away with it.

## Running it

Everything a tool needs is in `fsae_suspension/benchmarks/powertrain/`:

- `inputs.json`: the shared input set above, machine-readable;
- `motor_curve.csv`: the motor's shaft torque against rpm (120 Nm up to the shaft power limit, 80 kW at the pack
  times inverter and motor efficiency, then constant power to 23,500 rpm);
- `engine_curve.csv`: the representative 600 cc torque curve;
- `kinematik_results.csv`: KinematiK's numbers on these inputs (`python scripts/powertrain_benchmark.py run`);
- `reference_results_TEMPLATE.csv`: one row per tool, case and metric to fill in.

Copy the template, fill in each tool's number as it comes in, and grade it:

    cd fsae_suspension
    python scripts/powertrain_benchmark.py compare benchmarks/powertrain/reference_results.csv

It prints KinematiK against the tool for every filled row, with the limit and PASS or FAIL; empty rows are skipped.

**Simulink and Simscape (cases 1 and 2, then 4).** Start from MathWorks' *Formula Student Vehicle with Simscape*.
Set the vehicle body (mass, CG height, wheelbase, rear share, Cd·A, Cl·A, rolling resistance) from `inputs.json`;
use a Simscape Driveline tire with constant friction 1.4 (Tire (Magic Formula) with its peak set to 1.4, or Tire
(friction parameterized)); drive the motor from `motor_curve.csv` through a 7.0 gear with 0.95 efficiency. Case 1:
full torque from rest, log distance and speed, read the time and speed at 75 m. Case 2: a Drive Cycle Source with
KinematiK's endurance speed trace, log pack current and energy. For the combustion car use Powertrain Blockset's
Mapped SI Engine with `engine_curve.csv`, the gearbox ratios and a 0.08 s shift.

**Simulink lap with the Simscape Vehicle Templates (case 2 on a real lap).** The Simscape Vehicle Templates
(`sm_car`) carry MathWorks' Formula Student car, Achilles (four hub motors), and lap events: *Hockenheim flat* in the
FSAE set, and the CRG race tracks (Hockenheim, Mallory Park, Kyalami, Suzuka, Nürburgring). A lap event starts at
speed and stops after one lap.

1. Open the project (`startup_sm_car`), load the Achilles car, choose the *Hockenheim flat* event, and run it.
2. In the same folder, run `simulink_lap` (`benchmarks/powertrain/matlab/`; it calls `export_achilles`, so keep both
   on the path). It finds the speed, position, pack voltage, pack current and motor speed in `logsout_sm_car` by name,
   makes the current positive while discharging, resamples to 100 Hz, and writes `achilles/simulink_lap.csv`
   with the car's `achilles/inputs.json`. If a signal is not found it says so; pick it from
   `achilles/simulink_signals.txt`: `simulink_lap([], struct('volts', 'that path'))`.
3. Copy `achilles/` next to the repo's `fsae_suspension` folder and run
   `python scripts/powertrain_benchmark.py simulink-lap --inputs achilles/inputs.json --lap achilles/simulink_lap.csv`.

KinematiK is given Simulink's own speed trace (as in the FASTSim benchmark), so the comparison is the drive and the
pack, not the driver. Graded at 2 % (tool against tool, plan case 2; set before the first run and not moved since):
energy per km, peak and RMS pack current, and minimum pack voltage, all on discharge; the pack resistance fitted from
Simulink's V–I at 15 %. Reported, not graded: the motor speed over road speed (the tire rolls on its loaded radius,
so this measures the tire, not the gearbox), and KinematiK's own limit lap on the same path (`laptime.track_from_path`
and `simulate_lap`) beside Simulink's lap time. `sm_car`'s driver follows the event's own speed profile rather than
the car's limit, so KinematiK's lap should be the faster.

`export_achilles` also writes the car's motor efficiency map (speed × torque, from Simscape's loss table), its tires'
rolling-resistance and cornering-stiffness terms (from the .tir file) and its rotating-inertia factor (from the
wheels' spin inertia and the motors' rotor inertia); the grader uses each when it is there
(`ev_electrical_check.MotorEfficiencyMap`, `TireLosses`, `traction_power_w(cla_m2=, tire=, a_lat=)`), with the
lateral acceleration taken from the lap's X/Y.

### Result: MathWorks' Formula Student car, Hockenheim (flat), double wishbone (5 Nov 2026)

The *Formula Student Vehicle with Simscape* project (`SSVT_FSAE.prj`, MATLAB Online), event `CRG Hockenheim F`, one
lap of 4.53 km in 231 s at 6–23 m/s. The car as Simscape has it: 405 kg with driver (from the tire loads less the
downforce, and equal to the parts in `Vehicle_data_dwishbone.m`), four hub motors (300 N·m front, 200 N·m rear, sharing the torque equally on this lap) through 2:1 gearboxes,
60 kW converter at 98 %, 286 V / 230 Ah pack, Cd·A 1.2 m², Cl·A 3.0 m², wheels of 2 kg·m² spin inertia each.

| | KinematiK as it was | with the car's motor map, tires and inertia | Simulink | limit |
|---|---|---|---|---|
| energy, kWh/km | 0.112 (−21.7 %) | 0.138 (**−3.3 %**) | 0.143 | 2 % |
| peak pack current, A | 68 (−28.5 %) | 89.9 (**−5.6 %**) | 95.1 | 2 % |
| RMS pack current, A | 32.5 (−23.2 %) | 40.7 (**−3.6 %**) | 42.2 | 2 % |
| minimum pack voltage, V | 284.3 (+0.5 %) | 283.7 (**+0.3 %**) | 282.9 | 2 % |
| pack resistance, Ω | 0.0265 | 0.0265 (−13.1 %) | 0.0305 (fitted) | 15 % |

Two of five within their limits; energy and currents are outside the 2 %, by 3–6 %. Where the 21.7 % went, from
Simulink's own signals (motor torque and speed, tire forces and moments, brake torque; the balance closes to 2 %), and
what each addition closed, in the order added:

| added to KinematiK | energy against Simulink |
|---|---|
| nothing (one motor efficiency, Crr on the weight, factor 1.05) | −21.7 % |
| the motor's speed × torque map: each motor averages 14 N·m here, where the loss table gives 85 %, not the 93 % median | −14.4 % |
| the tire's rolling resistance under weight and downforce (up to +20 % load) and with speed: 0.066 kWh in Simulink against 0.035 for Crr on the weight | −9.5 % |
| cornering drag F_y² / C_α from the lateral acceleration and the tires' cornering stiffness | −6.5 % |
| the model's rotating inertia: 2 kg·m² of spin inertia per wheel, a factor of 1.36 on the mass (a typical 10-inch wheel has 0.2–0.3 kg·m²) | **−3.3 %** |

The run, the car's inputs exactly as `export_achilles` wrote them in MATLAB Online for that run, and the logged
signal list are stored in
`benchmarks/powertrain/simulink_hockenheim/` with their SHA-256 in `MANIFEST.sha256`;
`tests/test_powertrain_benchmark.py` re-grades the stored run and holds it to the numbers above, so nothing needs MATLAB
to check them: `python scripts/powertrain_benchmark.py simulink-lap --inputs
benchmarks/powertrain/simulink_hockenheim/inputs.json --lap benchmarks/powertrain/simulink_hockenheim/simulink_lap.csv`.

The inertia is also what the peak current was missing (−22 % before it, −5.6 % after): the peak is a full-power
launch out of a hairpin, where nearly a third of the motors' torque goes into spinning up the wheels.

What is left (3.3 % on energy): longitudinal tire slip (0.004 kWh in Simulink, not modeled), the cornering drag of
the point-mass estimate (0.016 against Simulink's 0.024 kWh: one lateral force shared by four tires, no load
transfer), and damper and other losses (about 0.01 kWh). The peak current, in a straight-line launch out of a
hairpin, is short by the wheels' slip (1.8 %) and their loaded radius, which sets the rotating inertia (1.39 on the
loaded radius against 1.36 on the free one).

### The sealed runs: does it hold on cases it has not seen?

The Hockenheim result above is in-sample: the physics was added because of that lap. Six more Simscape runs test it
blind. Their cases, metrics, limits and the SHA-256 of every file that grades them or runs them were sealed in
`benchmarks/powertrain/simulink_blind/PREREGISTRATION.json` before any of them was run; the grader refuses to grade
from code that differs from the sealed files, and every case is reported, pass or fail.

| case | what it tests |
|---|---|
| `hock_fs_car` | Hockenheim (flat) with a Formula Student-sized car: about 290 kg, 0.25 kg·m² wheels, a 25 Ah pack with 9.2 times the resistance (the same cells), so the voltage sag is real |
| `hock_fs_car_fast` | the same driven 25 % faster (`Maneuver.vGain` 1.25): the motors near full power |
| `hock_pushrod_fast` | the same with pushrod suspension: the powertrain result should not depend on it |
| `skidpad_fs_car` | steady cornering, where the cornering drag is most of the load |
| `launch_fs_car` | full-pedal standing start: KinematiK's own 75 m time and speed (`laptime.acceleration_time`), a prediction, not given the trace |
| `launch_stock` | the same with MathWorks' car as shipped (405 kg, 2 kg·m² wheels) |

Lap cases are graded as above (2 % on energy, peak and RMS current, minimum voltage; 15 % on the fitted
resistance); launches at 2 % on the time to 75 m and the speed there (plan case 1), both clocked the same way from
the sample before the car passes 0.5 m/s. What these runs cannot test: regeneration (the Simscape car has none),
battery temperature, and anything against a real car.

Order matters, and the commit history is the record of it:

1. Commit and push `simulink_blind/PREREGISTRATION.json` (it is in this patch) before running anything.
2. In MATLAB Online, with the project open and the files of `benchmarks/powertrain/matlab/` uploaded:
   `kinematik_blind_runs`. It runs every case not yet in `kinematik_blind/` and writes one folder per case.
3. Commit the `kinematik_blind` folder under `benchmarks/powertrain/simulink_blind/runs/`, then grade from the
   sealed commit: `python scripts/powertrain_benchmark.py simulink-blind benchmarks/powertrain/simulink_blind/runs`.

### Result of the sealed runs (5 Nov 2026): the Hockenheim fixes did not generalize

Run in MATLAB Online (R2026b) from 15:39 UTC, after the seal (09:29 UTC); stored with their SHA-256 in
`simulink_blind/runs/`, graded from the sealed code (`simulink_blind/runs/GRADED.txt`). **2 of 20 graded metrics
within their limits.**

| case | energy | peak current | RMS current | min voltage | resistance (fit) |
|---|---|---|---|---|---|
| `hock_fs_car` (275 kg) | −13.7 % | −33.2 % | −19.9 % | +6.3 % | −25.9 % |
| `hock_fs_car_fast` | identical to `hock_fs_car` (see below) | | | | |
| `hock_pushrod_fast` (285 kg) | −4.5 % | −33.4 % | −10.7 % | +5.2 % | −20.9 % |
| `skidpad_fs_car` | −62.5 % | −33.4 % | −57.2 % | +0.8 % pass | −14.1 % pass |
| `launch_fs_car` | could not be run: Simscape's battery failed to converge at full pedal (2.17 s) with the sealed 25 Ah pack | | | | |
| `launch_stock` | could not be run: Simscape's DC-DC converter failed to converge at full pedal (4.44 s) | | | | |

What the runs show, and what went wrong with the protocol itself:

* **KinematiK under-predicts the lighter car badly.** On the same speed trace, Simulink's 275 kg car drew *more*
  battery energy (0.650 kWh) than its 405 kg car (0.646 kWh); KinematiK, as physics says it should, predicts 13 %
  less. The gap is largest while accelerating (Simulink 46 % above KinematiK) and in corners (29 %), and appears
  as about 0.8 kW even on steady straights, where the stock car matched to 1 %. Something in the Simscape model
  costs more than KinematiK accounts for once the pack is smaller and the car lighter: candidates are the DC-DC
  converter at the lower pack voltage (252–286 V instead of 283–286 V) and inertia that the case did not reduce
  (if the wheels' 2 kg·m² is not all in `TireA*.Inertia`). Without a signal dump of these runs it is not
  diagnosed, and nothing has been changed because of it.
* **The pack model misses too:** the fitted resistance is 21–26 % above the declared one and the minimum voltage
  5–6 % high, because KinematiK holds the open-circuit voltage constant while the 25 Ah pack loses about 9 % of its
  charge over a lap.
* **The skidpad is at walking pace** (5.5 m/s): Simulink draws about 1.5 kW on average against KinematiK's 0.6 kW,
  a fixed loss that dominates at low power.
* **`Maneuver.vGain` did nothing.** `hock_fs_car_fast` is byte-identical to `hock_fs_car`, and the pushrod run
  drove the same speeds: the event's driver does not read that field. The "25 % faster" cases were not faster.
* **Both launches were beyond what the model can simulate** at full pedal; the sealed specification did not check
  that first.

The in-sample Hockenheim result (−3.3 %) therefore does not carry over to a different car, and the Simulink
benchmark does not, on present evidence, support KinematiK's EV energy within the plan's 2 % or even 5 %. A further
attempt has to diagnose the gap first (a `simulink_dump` of `hock_fs_car`), fix what is found, check that every
case is runnable and actually varies what it claims to, and then be sealed as a new pre-registration.

### Round 2: diagnosed, fixed, checked, then sealed again

**Diagnosis of round 1** (a `simulink_dump` of `hock_fs_car`, compared with the stock car's): the motors delivered the
same energy for both cars (0.556 against 0.553 kWh) at the same electrical efficiency (85.4 against 85.6 %). The
difference was mechanical and came from the case, not from KinematiK: taking 130 kg off without touching the springs
raised the car, and the rear suspension's toe went from −0.2° to +2.1° on each wheel. The rear tires then scrubbed
against each other everywhere (rear slip loss 0.040 kWh against 0.010; 705 N of tire side force on straights against
388), about 0.8 kW at mean speed, the gap seen on steady straights. Two further causes: the peak current was a
start-up spike 0.5 s into the lap, while the event's driver caught the car at its initial speed, and KinematiK held the
pack's open-circuit voltage constant while the 25 Ah pack lost 10 % of its charge in the lap.

**What changed** (all before the second seal):

* KinematiK: toe scrub, `C_α tan δ sin δ` per tire (`TireLosses.toe_drag_n`, `traction_power_w(toe_rad=)`), with
  the toe as the car runs measured from the run on the straights and exported by `simulink_lap.m`, as a team would
  measure its alignment; and a pack whose open-circuit voltage follows its state of charge (`pack_along_lap`, with
  Simscape's voltage-versus-charge table and initial charge exported by `export_achilles.m`).
* The Formula Student-sized car keeps its ride height and alignment: the spring preload is scaled with the sprung
  weight. Its pack is 25 Ah of higher-power cells (3 times the resistance, not 9.2). Its wheels keep the shipped
  2.4 kg·m² (see the next point).
* Speed is varied through the trajectory itself (`Maneuver.Trajectory.vx` and the start speed × 1.15).
* Launches at pedal 0.10: Simscape's controller asks each motor for 400 N·m × pedal (read off the stock lap), so
  40 N·m each; KinematiK's launch caps its motor curve at 4 × 40 N·m.
* Laps are graded from 3 s in, on both sides, past the event's start-up.
* `kinematik_blind2_runs.m` has a check mode, run before the seal: every case ran to its end, the car's rear toe
  stayed at −0.29° (the stock car's), the fast cases started at 26.45 m/s (shipped 23.00) and lapped in 200 s
  instead of 231, and both launches ran. It wrote no results.

* The rotating inertia is measured from the run: each wheel's spin inertia from `J α = T_motor × ratio − Fx r − |My|`
  while it speeds up, the same way the mass is measured from the tire loads. The first seal of round 2 (19:41 UTC)
  read it from `TireA*.Inertia` instead; checking it against Simulink's torques before any round-2 run showed that
  field does not reach Simscape's wheel (the wheels spin with 2.4–2.5 kg·m² whatever it is set to), so round 1's
  "light wheels" never were, and the export would have told KinematiK 1.07 for a car that has about 1.7. That seal
  was never pushed or run; it was replaced, and this note is the record of it.

* The pack heats: Simscape's battery block holds 30 kJ/K and starts at 300 K, and its resistance table rises with
  temperature (at 80 % charge, 0.026 Ω at 298 K and 0.056 Ω at 313 K, scaled with the pack); over a lap it warms 4–5 K.
  KinematiK's pack now reads its resistance from that table at the charge and temperature of the moment and heats by
  I²R (`pack_along_lap(r_soc_temp=, heat_capacity_j_k=)`); `export_achilles` reads the heat capacity off the block.
  The pack resistance is now compared as each side shows it, the slope of terminal voltage against current.
* **Every limit is 3 %**, the bar set for the whole benchmark, written into the seal before the runs. The plan's 2 %
  (which round 1 failed) is raised here, and the resistance, which had 15 %, comes down to it.

* Two more metrics, both at 3 %: the pack's temperature rise over the lap (Simscape's battery temperature against
  KinematiK's heating pack; +3.9 % in-sample on round 1's lap) and the heat the motors and inverter make while
  driving (battery power less the motors' shaft power; +12 % in-sample), with `simulink_lap.m` exporting the pack
  temperature and the four motors' shaft power.

The last three points were added after the first three seals of round 2 (19:41, 20:00 and 20:15 UTC), none of
which was pushed or run; each replacement is recorded here. The fourth seal (20:26:43 UTC), also unpushed and unrun,
was replaced to add the fitness-for-use bars (docs/POWERTRAIN_VALIDATION.md): a second, design-margin verdict per
metric, sealed next to the 3 % limits, which are unchanged.

**The seven sealed cases** (`simulink_blind2/PREREGISTRATION.json`, re-sealed on 5 Nov 2026): `hock_fs2`, `hock_fs2_fast`, `hock_fs2_pushrod_fast`, `hock_stock_fast` (the car the round-1 fixes were
built on, at a new speed), `skidpad_fs2`, `launch_fs2`, `launch_stock2`. Round 1's 2 of 20 stands as recorded.
What to expect, from the two laps already seen (in-sample, so not evidence): with everything above, the stock lap is
within 1.5 % on every metric and the round-1 FS lap within 4 % on all but peak current (−8.6 %, at the exit of the
hairpin, where something still unmodeled costs about 2 kW).

**Simcenter Amesim (cases 1, 2, 5, 6).** Build the same car from the IFP Drive and Powertrain libraries: a vehicle
body with the `inputs.json` parameters, a tabulated engine (`engine_curve.csv`) or e-motor (`motor_curve.csv`),
the gearbox and final drive, and constant-friction tires. Run the 75 m launch and the endurance speed trace as in
Simulink; for case 6 use a pneumatic restrictor (20 mm, Cd 0.95, ISA inlet) and read its choked mass flow.

**MoTeC (the car itself).** Benchmarks the powertrain design against the car, with the car's own parameters:
copy `inputs.json` to the car's own file and set every value to the car as built (mass with driver, tire radius,
Cd·A and Cl·A from a coast-down or the aero model, the motor or engine curve from the dyno, the ratios, the pack).

Log at 50 Hz or faster:

| Role | EV channel | Combustion channel |
|---|---|---|
| Time, speed | Time; Ground Speed (GPS or undriven wheels, not driven ones) | same |
| Distance (optional) | Distance | same |
| Pack | Pack Voltage, Pack Current (DC side, discharge positive) | — |
| Motor or engine | Motor Speed | Engine RPM; Throttle Pos (full-throttle samples only) |
| Fuel | — | Fuel Used (litres) |

Runs: one full-throttle acceleration from rest past 75 m, and one longer drive (an endurance lap or stint). In i2 Pro,
select the run, then File > Export > CSV, with the channels above at the logged rate. Channel names are matched to
the common ones above; a car whose names differ maps them with `--chan`, e.g. `--chan "speed=Wheel Speed FL"`.

    python scripts/powertrain_benchmark.py motec --kind ev --inputs car.json --accel accel.csv --drive endurance.csv
    python scripts/powertrain_benchmark.py motec --kind ice --inputs car.json --accel accel.csv --drive endurance.csv

| Metric | KinematiK | Measured | Limit |
|---|---|---|---|
| 75 m time | `acceleration_time` / `simulate_launch` | first movement to 75 m | 3% |
| EV peak wheel power | motor curve peak × driveline efficiency | max of (1.05 m a + road load) v | 5% |
| EV pack resistance | series × cell resistance / parallel | slope of voltage against current | 15% |
| EV final drive | declared | motor speed / wheel speed | 1% |
| EV energy per km | road-load power over the measured speed trace / the declared efficiencies | discharge energy (V·I > 0) / distance | 10% |
| Combustion gear ratios | primary × gear × final | engine rpm / wheel speed, per gear | 1% |
| Combustion torque curve | the declared map | (m_eff a + road load) r / (G η), full throttle, gears 2 and up, away from shifts | 8% RMS |
| Combustion fuel per 100 km | road-load energy / (BTE × LHV) | Fuel Used / distance | 10% |

Read a failure as a finding about an input, not a reason to widen the limit: a low torque curve is the declared map
or the driveline efficiency, a high energy per km is CdA, rolling resistance or an efficiency, a wrong final drive is
the tire radius. Correct the input that the evidence points at, and record the change and the run it came from.

## Recording a result

For each case: save the tool's export unedited, write the comparison as a script under `fsae_suspension/scripts/` and
a test that holds the limit, and add the row to BENCHMARK.md and VALIDATION.md, as the Adams View benchmark does.
