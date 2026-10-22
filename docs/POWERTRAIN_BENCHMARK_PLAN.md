# Powertrain benchmark plan: EV and combustion

The powertrain features are checked against closed forms (BENCHMARK.md). This plan is the next step: the same
cases run in the tools the industry treats as the reference, with the inputs and the pass limits fixed before any
run, as for the Adams View kinematics benchmark (`fsae_suspension/benchmarks/adams_view`). Nothing here has been run
yet.

## The tools

| Tool | Benchmarks | Access for a Formula SAE team |
|---|---|---|
| GT-SUITE (GT-POWER, GT-DRIVE, cooling, battery) | Engine and restrictor, launch, fuel and energy, cooling loop | Gamma Technologies has offered two free seats per FSAE team, EV or combustion (2019 announcement; confirm it still runs) |
| MATLAB and Simulink (Powertrain Blockset, Simscape Battery, Simscape Fluids, Motor Control Blockset) | Launch, fuel and energy, pack, cooling loop | Complimentary to Formula Student teams through the MathWorks competition request form |
| OptimumLap | 75 m and lap time | Free QSS lap simulator from OptimumG: the closest like-for-like check of KinematiK's QSS solver |
| PyBaMM | Cell voltage and heating under the current trace | Free, open source |
| AVL VSM RACE | Full-vehicle dynamics, for torque split and differential results | AVL's university partnership: one free licence a year for selected teams, applications by email |
| Ricardo WAVE | Alternative to GT-POWER for the engine | No current Formula Student offer found; ask Ricardo |

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
| 1 | 75 m from rest | `powertrain/engine.simulate_launch`, `laptime.acceleration_time` against OptimumLap, then Simulink or GT-SUITE | `ice_powertrain.simulate_launch` against OptimumLap (gears and shift time), then GT-DRIVE or Simulink with the same curve and gearbox | time, speed at 75 m; peak pack current (EV); shift speeds (ICE) | 2% |
| 2 | Energy on a fixed speed trace | `ev_powertrain`, `power_draw` on KinematiK's endurance speed trace, in Simulink and GT-SUITE | `ice_powertrain.fuel_for_lap` on the same trace with constant BSFC | kWh per lap, peak and RMS current, minimum pack voltage (EV); fuel per lap, peak fuel flow (ICE) | 1% |
| 3 | Lap time | `laptime.simulate_lap` against OptimumLap, same track and constant friction | `ice_powertrain.simulate_lap` against OptimumLap | lap time, minimum corner speeds | 2% |
| 4 | Energy store | `pack_thermal` against PyBaMM or Simscape Battery on case 2's cell current (cell part number needed) | `ice_powertrain.endurance_fuel` against the same tool's fuel total over 22 km | terminal voltage and cell temperature (EV); endurance litres and CO2 (ICE) | 2% and 2 °C (EV); 1% (ICE) |
| 5 | Cooling loop | `powertrain/engine` cooling network against GT-SUITE or Simscape Fluids (your Y-branch pipe lengths needed) | the same loop with the engine's heat; GT-POWER's heat to coolant at full load checks the declared 28% share | flow, loop pressure drop, coolant temperature over a lap | 2%, 3%, 1 °C |
| 6 | Intake | — | restrictor choked flow and the plenum (`throttle_dynamics`) against GT-POWER with the throttle, restrictor and plenum alone | choked mass flow; manifold pressure settling after a throttle step | 1%; 5% |
| 7 | Overrun | regen torque and the brake balance (declared) | GT-POWER's motoring torque against the declared 8 Nm overrun torque | torque at 6,000, 9,000 and 12,000 rpm | report, then replace the declared value |

Cases 5 (ICE share), 6's GT-POWER curve and 7 calibrate declared values rather than check a solver: their result
replaces the declaration, and the gate's MISSING and WARN flags go away with it.

## Recording a result

For each case: save the tool's export unedited, write the comparison as a script under `fsae_suspension/scripts/` and
a test that holds the limit, and add the row to BENCHMARK.md and VALIDATION.md, as the Adams View benchmark does.
