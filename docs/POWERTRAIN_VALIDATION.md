# What KinematiK's powertrain is validated against, and what it is not

One page for a reviewer. Each claim links to the benchmark that grades it; every number below is reproduced by a
script and held by a test in CI, from reference files pinned by SHA-256. Misses are listed with the hits.

## The evidence

| Claim | Evidence | Kind | Result | Where |
|---|---|---|---|---|
| EV energy on a drive cycle | Argonne D3, BMW i3, 7 graded dyno segments; road load from coast-down, efficiencies held out | **measured** | MAE 3.7 %, worst +6.6 %; 6 of 7 within 5 % | GT_SUITE_BENCHMARK.md |
| EV launch | Argonne D3, i3 wide-open-throttle runs; motor curve from one run, the other predicted | **measured** | 0–60 mph within 1 % | GT_SUITE_BENCHMARK.md |
| ICE fuel on a drive cycle | Argonne D3, 2019 Blazer, 6 graded segments; Willans line from the stairs test alone | **measured** | MAE 3.8 %, worst −6.3 % | GT_SUITE_BENCHMARK.md |
| Pack sag and I²R loss at Formula Student scale | Team A-FA, FSK 2023 endurance telemetry | **measured** | sag −3.7 %, I²R loss −4.7 % | GT_SUITE_BENCHMARK.md |
| EV energy and ICE power and fuel against GT-SUITE | 2 papers (BMW i3, 12 t truck) and 3 more (e-two-wheeler, Avanza, Proton); inputs the papers omit are assumed and banded | tool, published | 41 of 44 within 5 %; misses: i3 US06 −7.0 %, Avanza 0–100 −9.9 %, Proton gear sensitivity | GT_SUITE_BENCHMARK.md |
| The energy chain implemented right: road load, drivetrain, motor map, regeneration, aux, engine map, idle fuel | NREL FASTSim 3.1.0, 5 vehicles × 4 cycles, every input stated by FASTSim, nothing assumed | tool, exact inputs | 20 of 20; MAE 0.02 %, worst 0.04 % | FASTSIM_BENCHMARK.md |
| Energy, current and pack voltage of a four-motor Formula Student EV over a full lap, given the speed | MathWorks' Formula Student car in Simscape, Hockenheim | tool, multibody | energy −3.3 %, peak current −5.6 %, RMS −3.6 % (outside the 2 % limit); voltage +0.3 %, resistance −13 % (within) | POWERTRAIN_BENCHMARK_PLAN.md |
| The same, on cases the code has not seen | Six Simscape runs sealed before they were run (4 ran; both launches failed in Simscape) | tool, **blind** | **round 1: 2 of 20 within limits** (diagnosed: the case changed the car's alignment); **round 2 sealed at 3 %, pending** | POWERTRAIN_BENCHMARK_PLAN.md |

## Every element against 3 %

The bar a reader asked for: is each element of the EV and ICE modules within 3 % of an independent reference? The
honest answer per element, with the strongest reference it has. "Within" is a measured result; "pending" is a test
sealed and not yet run; anything else says why not.

| Module | Element (KinematiK function) | Strongest reference | Result | Within 3 %? |
|---|---|---|---|---|
| EV | Road load: mass, rolling resistance, drag, wheel inertia (`traction_power_w`) | FASTSim, identical inputs, 5 vehicles × 4 cycles | ≤ 0.03 % | **yes** (implementation) |
| EV | Drive efficiency, motor map, regeneration, auxiliaries (`traction_power_w`, `motor_efficiency`) | FASTSim, identical inputs | ≤ 0.04 % | **yes** (implementation) |
| EV | Energy on a measured drive cycle | Argonne i3, 7 dyno segments, efficiencies held out | mean 3.7 %, worst 6.6 % | **no**: 6 of 7 within 5 %; the dyno repeats to about 2–3 % itself |
| EV | Motor speed × torque map, tire rolling resistance under load, cornering drag, toe scrub (`MotorEfficiencyMap`, `TireLosses`) | Simscape Formula Student car, blind round 2 | round 1 failed (2/20, diagnosed); in-sample now −1.5 % on energy | **pending** (round 2, 3 %) |
| EV | Pack: I²R, voltage falling with charge, resistance with temperature, self-heating (`battery_internal_power_w`, `pack_along_lap`) | Simscape, blind round 2; A-FA telemetry (measured) | A-FA sag −3.7 %, I²R −4.7 % (constant-voltage model); Simscape in-sample: voltage 0.0 to +0.5 %, peak current +1.2 to −8.6 % | **pending** (round 2); **no** on A-FA with the old model |
| EV | Launch, 75 m and 0–60 mph (`laptime.acceleration_time`, `powertrain.engine.simulate_launch`) | Argonne i3 WOT runs (measured); Simscape part-pedal launches | i3 0–60 within 1 % | **yes** on the i3; Simscape **pending** |
| EV | Energy and top speed against published GT-SUITE models | 3 papers (i3, two-wheeler, Avanza) | i3 UDDS/HWY −2.0 / −1.3 %, US06 −7.0 %; two-wheeler top speed −2.7 %; Avanza top speed −4.1 %, 0–100 −9.9 % | **partly**. US06: the paper is itself 34 % above Argonne's measurement of the same car; Avanza: the paper does not publish its motor curve. Neither can honestly be made to agree within 3 %; both are reported as reference uncertainty, not tuned |
| EV | Lap time (`laptime.simulate_lap`) | none graded | — | **not validated**: no published Formula Student lap gives both the car's parameters and a measured lap. The closest, SAE 2019-01-0163 (Queen's University Belfast), compares its simulator with DUT12's FSG 2012 times (acceleration 3.454 s, endurance lap 78.297 s), but Delft published only the layout (four AMK hub motors, Hoosier 18 × 6-10 tires): the paper estimated the mass, gearing and motor data and used FSG's track file, which is not public. Needs a logged lap of a known car |
| EV | Battery temperature (`pack_temperature_along`, `pack_along_lap` heating) | Team A-FA FSK 2023 endurance, BMS cell temperatures (measured): heat capacity and cooling fitted on stint 1, stint 2 predicted; Simscape pack, blind round 2 | stint 2: RMS 0.34 K, worst 0.85 K over 2,119 readings; rise 7.20 K against 6.50 K measured (+10.8 %) | **no** on the rise, but the BMS reports whole degrees, so a 6.5 K rise is only known to about ±8 %: 3 % cannot be resolved in this data. The RMS is under the sensor's step. Simscape **pending** (round 2, 3 %) |
| EV | Heat from the motors and inverter, driving (`traction_power_w` with the motor map) | Simscape, blind round 2 (battery power minus the motors' shaft power) | in-sample on round 1's lap: +12 % | **pending** (round 2, 3 %) |
| ICE | Engine power and fuel at steady speed (`fuel_for_lap`, constant BTE) | GT-SUITE truck paper, every speed as calibration point | power within 1 %, fuel MAE 2.2 % | **yes** |
| ICE | Fuel through an engine map (`PowerEfficiencyMap`) | FASTSim, identical inputs, 2 vehicles × 4 cycles | ≤ 0.02 % | **yes** (implementation) |
| ICE | Fuel through a Willans line (`WillansLine`, `fit_willans`) | Argonne Blazer, 6 segments, line from the stairs test alone (measured) | mean 3.8 %, worst −6.3 % (1.9 % against the meter it was fitted to) | **no**: the line cannot follow a measured map at light load; the engine-map option is the route under 3 % |
| ICE | Gearing and launch (`Gearbox`, `simulate_launch`) | GT-SUITE Proton paper | 402 m time within 1.6 % (worst of 25); gear sensitivity off by 1.21× | **partly**: the time yes, the sensitivity no (the paper's shift schedule is not published) |
| ICE | Restrictor choked flow (`choked_mass_flow_kg_s`) | closed-form isentropic flow | exact by construction | **yes** (closed form; no measured restrictor yet) |
| ICE | Formula Student engine fuel map, coolant heat, ICE launch from a clutch | none open | — | **not validated**: read in full, the two Queen's University Belfast papers hold no fuel map. SAE 2008-32-0079 gives the unrestricted Yamaha R6's torque and airflow (plots only, no fuel); the restricted engine appears only as a simulation. SAE 2012-32-0027 gives one measured transient number, 231 ml for a 64 s Formula Student lap replayed on a dynamometer (±0.2 % meter), the measured savings of two map changes (leanest-for-best-torque at full load −5.7 %, over-run fuel cut −20.2 %) and the car's 3.702 l for the 2011 22 km endurance; the lap's torque and speed exist only as plots and no efficiency or BSFC data is given, so a prediction would rest on digitized curves and an assumed engine efficiency: a plausibility check (±10–15 %), not a 3 % grade. No open coolant-heat or clutch-launch data was found |

## Fitness for use: the bar a pre-validation tool has to meet

KinematiK sizes a car before it is built: pack capacity, fuses and cabling, cooling, gearing. The 3 % column above
is a research bar. A design decision needs a different bar: the error a decision can carry without changing it,
with misses on the side that leaves margin. These bars were written down, and sealed into round 2's
pre-registration (`fitness_for_use` in `benchmarks/powertrain/simulink_blind2/PREREGISTRATION.json`), **before any
round-2 result existed**. The 3 % research limits in the same file are unchanged; the grader reports both verdicts
for every metric, and neither is relaxed afterwards.

| Quantity | Fit for use within | Why that much | Sealed for round 2 |
|---|---|---|---|
| Energy per lap or km (sizes the pack) | ±5 % | packs are sized with 10–20 % reserve for ageing, temperature and the driver; FSAE endurance packs with a 5 % shortfall finish | ±5 % |
| Fuel per lap (sizes the tank) | ±5 % | same reserve logic; the Argonne Blazer test is at 3.8 % mean | (no ICE case in Simscape) |
| Peak and RMS current (fuses, cabling, contactors) | ±10 %, and an under-prediction outside the bar is flagged as unsafe | parts are rated with ≥ 20 % margin; under-predicting current is the dangerous direction, over-predicting only costs weight | ±10 %, over-prediction safe |
| Minimum pack voltage (motor field weakening, BMS cut-off) | ±3 % | the margin to the cell cut-off is small, so this one keeps the tight bar | ±3 % |
| Pack resistance | ±10 % | feeds the voltage and heat rows, which carry their own bars | ±10 % |
| Pack temperature rise (cooling, derating) | ±10 % of the rise or ±0.5 K, the larger | cell limits are set with several kelvin of margin; BMS sensors report whole degrees | ±10 % / ±0.5 K |
| Motor and inverter heat while driving | ±10 % | cooling is sized with margin; the row is a difference of two large powers | ±10 % |
| Launch, 75 m time and speed | ±3 % | 3 % of a 4 s run is 0.12 s, about the spread between two runs of the same car | ±3 % |
| Lap time | ±3–5 %, and the right order between designs | a lap simulator is used to rank designs; the ranking matters more than the absolute time | (not graded: Simscape's driver is not at the limit) |
| A design change (a delta: mass, gear, aero) | right sign, size within about 25 % | the decision is which way to go and roughly how far | (not graded in round 2) |

A metric can pass the fitness bar and fail the 3 % bar: that says the tool is usable for the decision but not yet as
accurate as the research claim. Both are reported.

## How to read it

* **Measured beats simulated.** The Argonne and A-FA rows are the ones that say KinematiK matches real hardware.
  The tool rows say it computes what established tools compute; that rules out implementation errors but not shared
  physics errors.
* **FASTSim checks the arithmetic, not the physics.** It is given identical inputs and agrees to 0.04 %: the code
  does what it says. It cannot show that the physics is complete, because FASTSim leaves out the same things a
  point-mass model leaves out (cornering, tire slip, load transfer).
* **The Hockenheim lap is in-sample.** The motor map, tire losses, cornering drag and rotating inertia were added
  because that lap showed KinematiK 21.7 % short; they are physics, with every value taken from the model's own
  files and nothing fitted, but the −3.3 % was measured on the lap that prompted them. The sealed runs tested
  whether they generalize, and they did not: on a lighter car with a smaller pack, KinematiK is 5–14 % short on
  Hockenheim energy and 33 % short on peak current.
* **Assumed inputs.** The GT-SUITE papers omit motor maps, BSFC maps and rolling resistance; those rows carry bands
  from the assumed values, and a pass means the paper's value is inside KinematiK's band.

## Not validated

* **EV energy and current of a Formula Student-sized car against Simulink.** Round 1 failed (above; diagnosed as
  the case's own alignment change, plus a start-up spike and a constant pack voltage); round 2 is sealed and not yet
  run.

* **Lap time against a real car.** KinematiK's lap time is checked only against itself and, ungraded, against
  Simscape's driver, which drives below the limit. A logged lap with a known car is needed.
* **Battery temperature against a car with a known pack.** The A-FA test fits the pack's heat capacity and cooling
  on one stint (they are not published), so it validates the model's form, not a prediction from a datasheet. The
  Argonne hot and cold tests are reported, not graded (no cabin model).
* **A Formula Student engine's fuel map.** The Willans line is graded on a 3.6 L road V6. SAE 2008-32-0079 and
  2012-32-0027 were read in full and hold no fuel map for the restricted 600 cc engine (see the table above); the
  one measured lap-fuel figure in 2012-32-0027 supports a plausibility check, not a grade. ICE coolant heat likewise
  has no open measured reference.
* **Regeneration on a Formula Student car.** Regeneration is graded against FASTSim (exactly) and Argonne's i3
  (measured), but the Simscape car has no regenerative braking.
* **ICE launch.** The Blazer's launches use a torque converter and all-wheel drive, which KinematiK's clutch launch
  does not model; not graded.
* **Tire slip and load transfer** in the energy model: what remains of the Hockenheim gap (longitudinal slip,
  unequal tire loads in corners, damper losses).
