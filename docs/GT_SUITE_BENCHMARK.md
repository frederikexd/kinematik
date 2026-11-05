# Powertrain benchmark: GT-SUITE papers and Argonne measurements

KinematiK's EV and ICE powertrain modules are graded against two published GT-SUITE models and, for the EV, against
Argonne National Laboratory's dynamometer measurements of the same car.

The second tool is NREL's FASTSim, where every vehicle input is stated, so the comparison has no assumption band:
20 of 20 vehicle-and-cycle runs agree within 0.04 % ([FASTSIM_BENCHMARK.md](FASTSIM_BENCHMARK.md)).

```
cd fsae_suspension
python scripts/gt_suite_benchmark.py                  # GT-SUITE comparison, writes benchmarks/gt_suite/results.csv
python scripts/gt_suite_benchmark.py --seal           # the same, plus the SHA-256 of results.csv (blind runs)
python scripts/gt_suite_benchmark.py argonne          # against Argonne's measured BMW i3
python scripts/gt_suite_benchmark.py argonne-extract RAW_DIR   # re-cut the Argonne segments from the raw D3 files
python scripts/gt_suite_benchmark.py argonne-ice      # the ICE against Argonne's measured 2019 Chevrolet Blazer
python scripts/gt_suite_benchmark.py argonne-ice-extract RAW_DIR   # re-cut it from the TDMS files (needs npTDMS)
python scripts/gt_suite_benchmark.py fsk2023 PATH/data.msgpack      # the pack model on a Formula Student car (needs msgpack)
python scripts/gt_suite_benchmark.py preregister CASE   # seal predictions for a paper before entering its results
python scripts/gt_suite_benchmark.py grade CASE         # grade the sealed predictions once the results are in
pytest tests/test_gt_suite_benchmark.py               # the findings and safeguards below, run in CI
```

## References

| Module | Reference | What is compared |
|---|---|---|
| `ev_electrical_check.traction_power_w`, `battery_internal_power_w` | Argonne D3, 2014 BMW i3 BEV, chassis dynamometer, May 2015 | Measured battery energy per km and its trace, 7 hot-start cycles at 25 °C, HVAC off |
| `powertrain.engine.simulate_launch` | Argonne D3, BMW i3, two wide-open-throttle standing starts | 0–30 and 0–60 mph |
| the same | SAE 2021-01-1245, BMW i3 60 Ah, GT-SUITE | Battery energy per km on the EPA UDDS, HWY and US06 cycles at 25 °C (Table 6) |
| `ice_powertrain.fuel_for_lap`, `WillansLine`, `fit_willans` | Argonne D3, 2019 Chevrolet Blazer 3.6 L V6, chassis dynamometer, Aug 2019 | Fuel per cycle (Argonne's bag values), 6 hot-start segments at 25 °C, predicted blind from the stairs test |
| the same | SAE 2012-36-0247, 12 t diesel truck, GT-SUITE |
| `ice_powertrain.simulate_launch` | JAITA 5(1) 2024, Proton Saga 1.3 L, GT-SUITE (GT-Drive) | 0–402 m time of 25 gear sets (Table 5) |
| `ev_electrical_check.fuse_limited_speed` | Materials Research Proceedings 49 (2025), electric two-wheeler, GT-SUITE | Top speed (Table 3) |
| `powertrain.engine.simulate_launch`, `fuse_limited_speed`, `traction_power_w` | Cylinder 11(1) 2025, converted Toyota Avanza EV, GT-SUITE | 0–100 km/h, top speed, NEDC energy (single gear) | Engine power (Figure 28) and fuel economy without accessories (Table 1) at 40–90 km/h |

Every input is in `benchmarks/gt_suite/references.json`, each with its table, figure or test number. Inputs the papers
do not give are under `assumed`, with a nominal value and a band.

## Results

### EV against the measured car (Argonne)

Road load from Argonne's coast-down targets (Crr 0.0106, CdA 0.730 m²). Auxiliaries measured at standstill (≈186 W).
Drive and regeneration efficiencies fitted on six segments, the seventh predicted, for each of the seven.

| Segment | Measured Wh/km | KinematiK | Error | Trace RMS |
|---|---|---|---|---|
| 61505019 HWY | 115.2 | 110.8 | −3.8 % | 3.9 % |
| 61505019 UDDS | 96.6 | 94.4 | −2.3 % | 1.6 % |
| 61505019 US06 | 154.0 | 161.3 | +4.7 % | 2.3 % |
| 61505020 US06 | 152.4 | 162.5 | +6.6 % | 3.4 % |
| 61505021 UDDS | 96.4 | 94.7 | −1.8 % | 1.5 % |
| 61505021 HWY | 116.3 | 111.0 | −4.5 % | 4.0 % |
| 61505021 UDDS | 96.7 | 94.5 | −2.2 % | 1.8 % |

MAE 3.7 %, bias −0.5 %, worst +6.6 %; 6 of 7 within 5 %.

The fitted efficiencies are effective values, not component efficiencies: drive ≈ 0.98 and regeneration ≈ 0.62. A
coast-down already contains the drivetrain's spin losses, so the drive efficiency that remains is close to 1, and
regeneration has to cover the same losses on the way back plus the friction brakes the i3 blends in at low speed.

### EV launch against the measured car (Argonne, BMW i3)

Two wide-open-throttle standing starts from Argonne test 61505016, at the logger's 10 Hz. KinematiK's
`powertrain.engine.simulate_launch` runs the i3 at its test weight, with Argonne's coast-down road load and the motor
speed to road speed ratio read from the log (an effective rolling radius of 0.339 m). Assumed: driveline efficiency
0.95 (the paper's differential) and rotating inertia 2 % of the mass. The dyno holds the car, so traction is not
tested here.

| | Run 1 measured | KinematiK | Run 2 measured | KinematiK |
|---|---|---|---|---|
| 0–30 mph, motor curve from the other run | 3.32 s | 3.16 s (−4.9 %) | 3.20 s | 3.26 s (+1.8 %) |
| 0–60 mph, motor curve from the other run | 7.50 s | 7.43 s (−1.0 %) | 7.47 s | 7.43 s (−0.5 %) |
| 0–60 mph, spec sheet only (250 Nm, 125 kW constant) | 7.50 s | 6.62 s (−11.8 %) | 7.47 s | 6.62 s (−11.4 %) |

The car itself varies by 3.8 % run to run on 0–30 mph and 0.5 % on 0–60 mph, which bounds how tightly 0–30 can be
graded.

What it shows: the launch dynamics are right, and the spec sheet is not enough. The torque curve measured at the
wheels (back to the motor through the ratio and 0.95) holds about 230 Nm to 4,250 rpm, peaks near 114 kW around
5,000–6,000 rpm and falls to 60–95 kW by 9,250 rpm. A curve built from the published 250 Nm and 125 kW held constant
to 11,400 rpm is 10–12 % fast for exactly that reason. **For design work, load a real motor curve** (the
manufacturer's continuous and peak curves, or a dyno sweep) instead of peak torque and power alone. The Avanza's
−9.9 % against GT-SUITE (below) is the same size and the same direction, and KinematiK ran it from peak values only,
so this is the likely explanation there too; the paper does not give its motor map to confirm it.

### The GT-SUITE paper against the same measurements

| Cycle | Measured (mean) | GT-SUITE, paper | KinematiK on the paper's inputs |
|---|---|---|---|
| UDDS | 96.6 | 116.0 (+20 %) | 113.7 (+18 %) |
| HWY | 115.7 | 144.0 (+24 %) | 142.1 (+23 %) |
| US06 | 153.2 | 205.0 (+34 %) | 190.7 (+25 %) |

The paper's GT-SUITE energy figures sit 20–34 % above Argonne's measurements of the same car. Its road-load inputs
(Crr 0.015) give about 40 % more rolling force than the coast-down. KinematiK agrees with GT-SUITE when given the same
inputs (UDDS −2.0 %, HWY −1.3 %, US06 −7.0 %, every GT-SUITE value inside the band of the assumptions). So the paper is
a reference for model against model, and Argonne is the reference for the car.

### ICE against the measured car (Argonne, 2019 Chevrolet Blazer)

Road load from Argonne's coast-down targets (Crr 0.0075, CdA 1.084 m²), full-load torque curve from the
wide-open-throttle test, published 9T65 ratios checked against the logged shaft speeds.

**Fuel model: a Willans line** (`ice_powertrain.WillansLine`, fitted with `fit_willans`). The engine burns
(brake power + friction and pumping power) / indicated efficiency, so its efficiency rises with load as a real engine's
does. Everything it needs comes from one test, the steady-speed stairs (61908021): the line from the engine's logged
torque, speed and fuel flow (η_indicated 0.408, FMEP 56 kPa + 52.5 kPa per 1000 rpm), and the driveline efficiency
against the coast-down road load (0.942). The six graded cycles are then predicted blind; only the idle and overrun
fuel rates are measured on the other segments.

| Segment | Measured g (bag) | Willans | Error | One BTE (before) | Error |
|---|---|---|---|---|---|
| UDDS | 884.7 | 835.7 | −5.5 % | 755.1 | −14.7 % |
| UDDS | 887.3 | 831.1 | −6.3 % | 746.9 | −15.8 % |
| HWY | 827.1 | 806.4 | −2.5 % | 847.3 | +2.4 % |
| HWY | 813.1 | 808.4 | −0.6 % | 853.1 | +4.9 % |
| US06 | 1055.5 | 1000.0 | −5.3 % | 1124.8 | +6.6 % |
| US06 | 1038.1 | 1012.8 | −2.4 % | 1145.4 | +10.3 % |

Willans: MAE 3.8 %, bias −3.8 %, worst −6.3 %. One BTE (fitted on the other five): MAE 9.1 %, worst −15.8 %.

The remaining bias is the fuel meter. The Willans line is fitted to the logged direct fuel flow, which reads 1.5–3.8 %
below Argonne's bag fuel (carbon balance, the certification method). Against the meter it was fitted with, the
prediction is within MAE 1.9 %, bias −1.2 %, worst −3.7 %.

The logs show why one BTE could not work: the engine's own efficiency is 0.14 at 1–5 kW, 0.22 at 5–10 kW, 0.28 at
10–20 kW and 0.31–0.32 above 20 kW.

**For a race car.** Give the engine its fuel map once: upload a dyno sweep on the ICE screen (rpm, torque or power,
and fuel flow or BSFC at three or more throttle openings; a template is on the screen), or in code
`EngineMap(..., fuel_map=fuel_map_from_csv(text, displacement_cc, lhv).line)`. The screen shows the fitted line, how
closely it meets the points and its efficiency against power, and warns when the efficiency, the friction or the fit
look wrong. Every fuel calculation
(`fuel_for_lap`, endurance fuel, the efficiency score, engine comparisons) uses it. Without a logged rpm trace, the
engine speed is that of the gear with the most drive force, the gear a driver on the limit is in. That matters on a
Formula Student lap: part-throttle corners at high rpm in a low gear cost the friction and pumping power that one
full-load BTE leaves out. Here the benchmark passes the Blazer's logged rpm instead, because its automatic picks gears
for economy. Without a fuel map, `fuel_for_lap` still uses one declared BTE, as before.

### EV pack at Formula Student scale (Team A-FA, Formula Student Korea 2023 endurance)

On-car telemetry from a real Formula Student EV in competition: 3.85 kWh over two endurance stints, 28.9 kW peak,
state of charge 94 → 58 %. The team published it at github.com/luftaquila/a-fa-telemetry-endurance-viewer. The
repository has no license, so the data is not copied here: `fsk2023` reads your own clone and refuses any file
whose SHA-256 differs from the one pinned in `references.json`.

Pack resistance and a linear open-circuit voltage are fitted on stint 1; on stint 2 the open-circuit voltage is
read at rest, and `battery_internal_power_w` predicts current and terminal voltage from the measured power.

| Stint 2 | Measured | KinematiK | Error |
|---|---|---|---|
| Voltage sag above 30 A (mean) | 9.07 V | 8.73 V | −3.7 % |
| I²R loss over the stint | 78.0 Wh | 74.3 Wh | −4.7 % |

Resistance 0.146 Ω from stint 1 against 0.137 Ω fitted on stint 2 alone; loaded voltage within 3.1 V RMS (about 1 %),
current within 0.5 A RMS. The open-circuit voltage is not KinematiK's to predict (it is an input); a straight line
fitted on stint 1 and extended to stint 2 misses by about 8 V, because the cell curve steepens at low charge. The
BMS pack voltage in this log is filtered and the inverter's torque estimate gives more mechanical than electrical
power, so neither is used. Without the car's mass, aero and gearing, the road-load side cannot be graded on it.

**Pack temperature.** The BMS logs its hottest and coolest cell in whole degrees; their mean is the pack's
temperature here (28 → 41 °C over the endurance). KinematiK's lumped pack (`pack_temperature_along`,
C dT/dt = I²R − hA (T − T_ambient), R the stint-1 resistance, the ambient the pack at the start) has its heat capacity
and cooling fitted on stint 1 alone (28.3 kJ/K, 7.4 W/K; RMS 0.33 K), and stint 2 is predicted from its own current:

| Stint 2 | Measured | KinematiK | Error |
|---|---|---|---|
| Temperature rise | 6.50 K | 7.20 K | +10.8 % |
| Temperature, RMS over 2,119 readings | | | 0.34 K (worst 0.85 K) |

The rise misses 3 % and 5 %, but it is known only to the sensor's step: two whole-degree readings averaged, so
about ±0.5 K on a 6.5 K rise (±8 %). Along the stint the model stays within 0.34 K RMS, under that step. The heat
capacity and cooling are fitted, not taken from the pack's datasheet (unpublished), so this checks the model's form
on held-out data, not a prediction from first principles. This test was designed after a look at the temperature
range in the log.

### ICE against the GT-SUITE truck

Calibrated at each of the six speeds in turn, worst error on the other five: power within 2.0 %, fuel within 4.1 %
(MAE 2.2 %, bias +0.3 %). Engine power follows GT-SUITE within 1 % from any calibration speed.

### More GT-SUITE EV papers

**Electric two-wheeler** (220 kg, 5 kW, every input in Table 1): KinematiK's top speed from the road-load power
balance is 78.3 km/h against GT-SUITE's 80.44 (−2.7 %); the motor speed there, from the gear ratio and wheel radius,
matches GT-SUITE's 5,927 rpm exactly. This is the closest to Formula Student scale of the papers (mass about that of
an FS car, far less power).

**Converted Toyota Avanza EV** (1,250 kg, 60 kW, single gear). Tire radius (0.30 m, which reproduces the paper's
automatic top speed at the gearbox's 6,000 rpm input limit), rolling resistance (0.008–0.012) and driveline
efficiency are assumed.

| Single gear | GT-SUITE | KinematiK | Error |
|---|---|---|---|
| Top speed | 190 km/h | 182.2 km/h | −4.1 % |
| 0–100 km/h (`powertrain.engine.simulate_launch`) | 14.0 s | 12.6 s | −9.9 % (fail) |
| NEDC energy | 117.0 Wh/km | 85.8 with regeneration, 113.1 without | not graded |

KinematiK's launch reaches 100 km/h 1.4 s sooner with the full 220 Nm and 60 kW; the paper does not give what
limits GT-SUITE's launch (the pack is 268.8 V against a motor rated 260–410 V, which may hold power below peak). The
NEDC figure lies inside KinematiK's band over the assumptions, but the paper does not say whether regeneration is
on, and the answer turns on it, so it is reported, not graded. The automatic-transmission results (13.3 s, 191.3
km/h, 8.77 km/kWh) are reported only: KinematiK's EV drivetrain is single-speed.

The NEDC trace is built from the UNECE R83 breakpoints (`nedc.csv`, 10.97 km against the official 11.0 km).

**Checked and not graded:** the same authors' battery-temperature study (Cylinder 11(1) 2025) gives peak pack
temperatures on NEDC and WLTP but not the cell resistance, thermal mass, starting or ambient temperature, or the
convection it used, so a heat model cannot be checked against it.

### ICE gearing against GT-SUITE (Proton Saga, 25 gear sets)

The paper's GT-SUITE model (correlated to the car within 0.3 % on NEDC fuel and 1.5 % on 0–100 km/h) gives the
0–402 m time for an L25 array of four-speed gear sets. Frontal area, final drive, the torque curve's shape and the
automatic's launch are not published, so they are assumed (`references.json`), and the torque curve is scaled once
so the standard set gives the paper's base 22.82 s.

The car is a four-speed automatic, which upshifts at road speeds set by its control unit, not where the next gear
pulls harder. Its schedule is not published, so KinematiK's `Gearbox(upshift_kmh=...)` takes the schedule of the
same class of Toyota four-speed automatic from another paper (Setiawan and Farid, Cylinder 11(1) 2025, Table 3:
25, 45, 65 km/h), unchanged.

| | Automatic's schedule (graded) | Shifting at maximum force (race) |
|---|---|---|
| 402 m time, worst of 25 | −1.6 % (MAE 0.5 %) | +3.4 % (MAE 1.1 %) |
| Rank correlation with GT-SUITE | 0.93 | 0.35 |
| Gear effect (KinematiK's change from the base set ÷ GT-SUITE's) | 1.21 (95 % interval 1.02–1.39) | 0.42 |

With the automatic's shifting, KinematiK ranks the 25 sets as GT-SUITE does and gets the size of the gear effect
within 21 %; the size still fails the 5 % tolerance. It depends on the schedule, which the paper does not give (a
30/55/80 km/h schedule gives 1.10), so it is not tuned further. The race-shifting column is kept as the diagnostic:
it is how KinematiK models a Formula Student car, and it is a different car from the paper's.

### What the two remaining misses are

**i3 on US06 (−7.0 %).** Tested and ruled out: a motor loss model that varies with torque and speed (copper loss
∝ torque², iron loss ∝ speed), with its two coefficients fitted on the UDDS and highway cycles, predicts US06 at
−9.4 %, further off, not closer. The paper's own numbers do not hold together on this cycle: its GT-SUITE model
claims Argonne agreement within 2 % of state of charge, yet its 205 Wh/km is 34 % above Argonne's measured US06 for
the same car (153 Wh/km; see the Argonne section), against 20–24 % on the other two cycles. KinematiK is not changed
to chase that number.

**Avanza 0–100 km/h (−9.9 %).** KinematiK ran it from the paper's peak torque and power only, held constant above
base speed. On the measured i3 (above) that same idealisation is 10–12 % fast, while a measured motor curve brings
KinematiK within 0.5–1.0 % on 0–60 mph. The Avanza miss is the same size and sign, so the motor curve is the likely
cause; the paper does not publish GT-SUITE's motor map, so it cannot be confirmed, and the row stays graded as a miss.

## Papers checked and not graded

A paper can be graded only if it prints, as numbers, both the inputs KinematiK needs and results KinematiK
computes. These were read in full and could not be:

| Paper | Why not |
|---|---|
| SAE 2006-32-0070, restricted Yamaha R6 (Schenck dyno) | Torque and power only; no fuel flow |
| SAE 2005-01-0025, Honda CBR600 F4i for FSAE (GT-POWER, WAVE) | Engine set-up and injector calibration; volumetric efficiency only as a plot; no fuel map |
| SAE 2016-36-0123, ethanol Honda 600 for FSAE | Ricardo WAVE, not GT-SUITE; curves only as plots |
| SAE 2019-01-2369, Isuzu eNPR-HD electric truck (GT-SUITE) | Results normalised to the field test (field = 100) for confidentiality; coast-down coefficients and drive traces not published |
| SAE 2026-26-0362, EV thermal control for range (GT-SUITE) | Grille-shutter and fan strategy; results as plots, no cell or pack thermal inputs |
| IJRASET 10(6) 2022, GSX-R600 exhaust for FS (GT-SUITE) | Exhaust geometry; KinematiK takes the torque curve as an input and has no gas-dynamics model. Its 70 kW peak sits at KinematiK's choked-flow limit for a 20 mm restrictor (70.8 g/s), a consistency check only |
| Bradshaw 2019, Oxford Brookes FS V-twin thesis (GT-POWER) | Engine breathing and combustion. Its choked-flow figure (2.04 kg/s) uses a 76 mm throat area, not 20 mm; its 61.8 kW from 46.4 g/s of air implies a 45 % brake efficiency |
| Cylinder 11(1) 2025, Avanza EV battery temperature (GT-SUITE) | No cell resistance, thermal mass, starting temperature or convection |
| MDPI Energies 13(14):3694, supercharged KTM 690 FS engine (GT-POWER) | Fuel use at full load only, with a flat torque curve that cannot separate friction from efficiency |
| MDPI Machines 8(3):51, EV lightweighting | MATLAB/Simulink, not GT-SUITE |

The gaps these leave (a Formula Student engine's part-load fuel use, battery temperature, a GT-SUITE EV against
test data) need data that is not published in the GT-SUITE literature found so far. Measured references cover two
of them where GT-SUITE does not: Argonne's i3 and Blazer, and Team A-FA's Formula Student pack.

## The rules that keep it honest

1. **Pinned inputs.** `MANIFEST.sha256` holds the SHA-256 of `references.json`, the three cycles and every Argonne
   segment (EV and ICE). The script refuses to run (exit 2) when a file no longer matches. To change one deliberately, edit it,
   then run `cd benchmarks/gt_suite && sha256sum references.json udds.csv hwfet.csv us06.csv nedc.csv argonne/*.csv
   argonne_ice/*.csv > MANIFEST.sha256`; the change then shows up in review as both a value and a hash.
2. **The extraction is checked against Argonne's own numbers.** Each segment's energy matches the DC energy on
   Argonne's summary sheet within 0.2 %. Test 61505028 did not (up to 6 % apart) and is left out.
3. **No grading on the calibration data.** Efficiencies are fitted on the other segments (Argonne) and Crr and BTE on
   one speed with the worst over all six reported (truck). The auxiliaries are measured, not fitted.
4. **Bands, not just points.** The GT-SUITE comparison is also computed at every corner of the assumed bands
   (efficiency 0.85–0.93, air density ±3 %, pack resistance 0.06–0.13 Ω).
5. **Traces, not only totals.** The cumulative energy is compared over each whole cycle (RMS error as a share of the
   total), so errors that cancel over a cycle still show. The papers' plotted traces were not digitized: Argonne's
   10 Hz measurements replace them.
6. **Statistics, not only pass or fail.** Mean absolute error, bias and worst case per module.
7. **Tolerance, and why.** 5 %: tighter than the truck paper's own error against its tests (up to 7–10 % fuel) and
   than the gap between a cold and a hot start of the same UDDS at 25 °C (104 against 97 Wh/km, 7 %); hot repeats
   of one cycle agree within 1 %.

## Not covered yet

* **Temperature and HVAC.** Argonne's −18 °C, −6 °C and 35 °C tests (HVAC set to 22 °C) are reported but not graded:
  KinematiK has no battery thermal or cabin climate model and under-predicts them by 3–71 %. The cold-start 25 °C
  UDDS is 9 % under for the same reason.
* **Formula Student engine fuel map.** The Willans line is validated on a 3.6 L road V6. A restricted 600 cc engine
  needs its own dyno sweep with fuel flow. SAE 2006-32-0070 (restricted Yamaha R6, Schenck dyno) was checked: it
  reports torque and power only, mapped at 13:1 air-fuel ratio, with no fuel flow, so it cannot calibrate a fuel map.
* **ICE acceleration.** The Blazer's wide-open-throttle runs are in the data, but an automatic with a torque
  converter and all-wheel drive launches unlike the clutch launch `simulate_launch` models, so they are not graded.
* **ICE transients.** The truck's acceleration runs need its torque curve and gear ratios, which the paper shows only
  as a 3D surface and a part number.
* **Accessory loads on the ICE.** The truck's fuel figures with accessories are not graded.
* **Formula Student scale, vehicle level.** The A-FA log checks the pack at Formula Student scale, not the road
  load, because the car's mass, aero and gearing are not published with it. The Simulink Achilles kit (`benchmarks/powertrain/matlab/simulink_benchmark.m`) and MoTeC
  logs (`scripts/powertrain_benchmark.py motec`) cover FS cars.

## Pre-registered papers (a single developer, no second person)

An independent run needs someone who has not seen KinematiK's numbers. Working alone from published papers, the
substitute is pre-registration: KinematiK's predictions for a paper are sealed, from a committed release, before
the paper's results are typed in, and the seal is pushed to GitHub so its time is public.

1. Add the paper to `benchmarks/gt_suite/papers.json` with its `type`, `source` and `inputs`, and no `results`.
   Types: `ev_cycle` (a vehicle on standard cycles, energy per km) and `fuel_map_holdout` (a fuel map fitted on some
   dyno points, fuel flow predicted at the others). Commit.
2. `python scripts/gt_suite_benchmark.py preregister CASE`. It refuses if the case already has results or if the
   working tree has uncommitted changes, so every prediction names a commit. It writes
   `preregistered/CASE.json` (commit, UTC time, a hash of the inputs, the predictions) and its SHA-256 into
   `preregistrations.json`.
3. Commit and push those two files. Only then read off the paper's results.
4. Add `results` to the case and run `python scripts/gt_suite_benchmark.py grade CASE`. It grades the sealed
   predictions, not a fresh run, and also shows what the current model gives.

A test fails if any paper with results lacks a seal, if the sealed file changed after registration, or if the
inputs changed after sealing. When the model changes later, every paper is re-graded and both the sealed and the
current numbers are reported, so a fix cannot be tuned to one paper unseen.

What this does not do: it cannot stop the one person who enters the inputs from having glanced at the results.
The docs and every graded paper are labelled "pre-registered, single developer"; the GitHub history of each seal
is what a reader checks.

### With a second person, later

1. The KinematiK side runs `python scripts/gt_suite_benchmark.py --seal` (or `preregister`) and publishes the
   SHA-256.
2. The other person runs the reference tool on the same inputs and sends their values first.
3. The results file is published and checked against the SHA-256.

## Argonne data

`benchmarks/gt_suite/argonne_ice/` holds the Blazer segments the same way, from the TDMS "Extended Datasets"
(Conventional vehicles, 2019 Chevrolet Blazer). `benchmarks/gt_suite/argonne/` holds 1 s block means (energy and distance kept exactly) of the segments listed in
`references.json`, cut from the raw files with `argonne-extract`. The raw files come from
https://www.anl.gov/taps/downloadable-dynamometer-database (Electric vehicles, 2014 BMW i3 BEV, "All Data").

This data is from the Downloadable Dynamometer Database and was generated at the Advanced Mobility Technology
Laboratory (AMTL) at Argonne National Laboratory under the funding and guidance of the U.S. Department of Energy (DOE).
