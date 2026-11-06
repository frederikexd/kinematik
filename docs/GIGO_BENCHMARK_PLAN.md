# Garbage-in benchmark

**Result (6 Nov 2026, seal `4c18a8a`, run with its hash as the seed): at entry, KinematiK caught 0 of 120 harmful
input errors; as a second opinion beside FASTSim it caught 99 of 99; no false alarm on any valid car.** Details
below, under Result.

The powertrain benchmark (POWERTRAIN_VALIDATION.md) shows KinematiK computes the right answer from the right inputs.
KinematiK's pitch is more than that: it should stop wrong inputs from reaching, or quietly coming back from, an
established tool. This plan tests that claim with the same discipline: the corruptions, the decks and the pass
criteria are sealed before anything is run, and every miss is reported.

## What exists today (read from the code, 5 Nov 2026)

* **Entry checks.** The powertrain functions refuse inputs with no meaning (a negative restrictor diameter, a
  motor map with efficiencies outside (0, 1], gear ratios that do not fall). They do not judge whether a meaningful
  value is plausible: 150 hp typed into a kW field, or a pack's 60 Ah typed into a kWh field, is accepted.
* **Workbook audits** (`power_draw.audit_pack_sheet`, `audit_workbook`): cross-checks inside a team's Excel pack
  sheet.
* **Proof Engine** (`proof_engine.py`): a run outside the ledger's ±3σ envelope is DISCREPANT.
* **Saboteur** (`saboteur.py`): ten corruption classes (length, mass, power and torque unit slips, Z flips, dropped
  or doubled roll-up terms) and ten tripwires (mass, CG height, specific power, torque per power, implied pack
  voltage, ...). Its own figures ("about 8 % caught by the envelope, 100 % with four wires") come from its own
  catalog on its own representative deck: a self-test, not an independent benchmark. Few of its classes are
  powertrain-specific.

## The question, in two gates

1. **Gate 1, at entry.** The wrong number is given to KinematiK. Does it refuse it (ValueError) or warn, beyond what
   it says on the clean deck? A crash is reported, not counted as a catch.
2. **Gate 2, second opinion.** The deck in KinematiK is right; the copy given to the established tool is wrong (a
   transcription error). Does the tool's result disagree with KinematiK's by more than 1 % (clean decks agree to
   0.04 %)?

**When the same wrong number is in both** (a misread datasheet), Gate 2 cannot see it, and neither can Saboteur's
tripwires or the Proof Engine's envelope: both compare a run with the deck, and here the deck itself is wrong. Only a
check of the value against physics or the rest of the deck can catch it, which is Gate 1. So a separate third gate
in the first draft is folded into Gate 1, and Gate 1's score is also the score for common-mode errors.

## Decks

The five FASTSim vehicles of FASTSIM_BENCHMARK.md (three EVs, two combustion), on the same four cycles, with
FASTSim 3.1.0 as the established tool in the loop: FASTSim runs locally, so the whole benchmark runs in one command,
no MATLAB. And the Formula Student car of the Simscape round 2 (`hock_fs2` and `launch_fs2` inputs): it has the inputs a
road-car deck lacks (motor torque, gear ratio, motor speed, cell voltage and capacity), so the Formula
Student-specific errors are tested there, for Gate 1 and harm.

**False alarms are tested on unusual cars too.** A Formula Student car is an outlier (276 kg, four motors, a 25 Ah
pack): a check tuned on road cars could flag it. Gate 1's clean set is the five FASTSim decks plus every Simscape
deck the powertrain benchmark holds (the seven round-2 decks, Formula Student and stock, and the stock Hockenheim
deck). A check that flags any of them is a false alarm.

## Corruptions: three sets, so the test cannot be steered

A list written by the people who wrote the checks can drift toward what the checks catch. Three sets guard
against it, each reported on its own:

1. **Systematic.** Every powertrain input, crossed with every confusion its dimension allows, from a fixed table:
   SI and imperial (kW/hp, N·m/lb·ft, kg/lb, m/in), prefix slips (×1000, ÷1000), percent for fraction (×100),
   diameter for radius (×2), inverse (1/x). Exhaustive, so nothing is picked.
2. **Random.** Typing errors (a dropped or shifted decimal point, two digits swapped, a digit dropped) on inputs
   drawn at random. The generator's seed is the commit hash of the pushed seal, which no one knows until the seal
   is on GitHub, so the sample cannot be chosen.
3. **Curated.** The documented real-world errors below, as the readable headline.

## Curated catalog (to be sealed)

Each is a documented, common error; each names the input it strikes. Applied one at a time, to every vehicle it
applies to.

| Key | Error | Decks | Input |
|---|---|---|---|
| lb_for_kg | the mass from a US scale, in pounds (×2.205) | all | vehicle mass |
| driver_dropped | the 75 kg driver left out | all | vehicle mass |
| cd_for_cda | the drag coefficient typed where CdA belongs (÷ frontal area) | FASTSim | drag |
| crr_percent | rolling resistance in percent (×100) | FASTSim | Crr |
| dia_for_radius | wheel diameter typed as the radius (×2) | all | wheel radius |
| hp_for_kw | a horsepower rating typed as kW (×1.341) | all | motor, engine or pack power |
| aux_kilo | 300 W of auxiliaries typed as 0.3 (÷1000) | FASTSim | auxiliary power |
| eff_percent | drive efficiency typed in percent (×100) | all | transmission or inverter efficiency |
| eff_twice | the drive efficiency applied twice (squared) | all | transmission or inverter efficiency |
| lbft_for_nm | motor torque from a US datasheet, in lb·ft (÷1.356) | Formula Student | motor peak torque |
| ratio_inverted | gear ratio entered as its inverse | Formula Student | final drive |
| rpm_for_rads | motor top speed in rad/s typed into rpm (÷9.549) | Formula Student | motor top speed |
| cell_for_pack_v | a cell's 3.6 V typed where the pack voltage belongs | Formula Student | pack voltage |
| mah_for_ah | cell capacity in mAh typed into Ah (×1000) | Formula Student | cell capacity |

## What is measured

For every (corruption, vehicle, cycle):

* **Harm.** FASTSim decks: how far the corrupted deck moves FASTSim's energy or fuel on any cycle; harmful beyond 5 %
  (the fitness-for-use bar). The Formula Student deck (Simscape round 2's `hock_fs2` and `launch_fs2` inputs, no
  tool to rerun): how far it moves KinematiK's own Simscape-validated energy (5 %), peak and RMS current (10 %),
  minimum pack voltage (3 %) or 75 m time (3 %). Harmless corruptions are reported, not scored. A deck FASTSim
  refuses to run is counted apart: the tool caught it.
* **Gate 1** and **Gate 2** as above (Gate 2 on the FASTSim decks only).

Reported: the share of harmful corruptions each gate catches, the share no gate catches (the silent ones, listed by
name), and false alarms on the clean decks.

## Honesty rules

* The catalog above is written by the same people who wrote the checks. It is drawn from documented error types
  (Saboteur's stories plus the common powertrain unit slips), and it is sealed before any check is changed.
* **Gate 1 is measured on the code as it is.** If the result shows gaps and checks are added, that is a new, sealed
  round, reported beside this one.
* **Predictions, in numbers, sealed with the catalog** (so a good result cannot be called expected afterwards):
  Gate 2 catches at least 95 % of harmful corruptions with no false alarm; Gate 1 catches under 30 % (only values
  with no meaning are refused today) with no false alarm. A low Gate 1 is a finding, not a failure of the
  benchmark.
* **Anyone can rerun it**: one command, FASTSim pinned at 3.1.0, every deck and corruption in the repo with
  checksums, the results held by a test in CI.

## Seal record

* 6 Nov 2026, 00:04:00 UTC: first seal, replaced before it was pushed or run. A dry check showed its cell-voltage
  error did nothing on the Formula Student deck, whose pack is exported as one lumped cell (series 1), so "pack
  voltage typed as the cell voltage" multiplied by 1. The error became "a cell's 3.6 V typed where the pack
  voltage belongs"; nothing else changed.
* 6 Nov 2026, 00:04:35 UTC: second seal, also replaced before it was pushed or run. A check that every corruption
  changes its deck found 14 that do not (a zero idle-fuel power scaled, a driveline efficiency of 1 squared, typing
  errors that give the same number, such as two equal digits swapped). The runner now records these as "no change"
  and leaves them out of every count, instead of scoring them as harmless.
* 6 Nov 2026, 00:04:56 UTC: third seal, **pushed** as commit `abfa38d` and run with that hash as the seed. The run
  died before writing any result: FASTSim's compiled core crashed outright (a segmentation fault) on one of the
  corrupted decks, taking the whole benchmark process with it. No result was produced or seen. The runner now runs
  each FASTSim deck in a process of its own, so a crash is recorded as FASTSim refusing that deck. That changes
  the sealed runner, so the benchmark is sealed again; the pushed seal is kept as
  `benchmarks/gigo/PREREGISTRATION.superseded-abfa38d.json`, and the random errors are drawn from the new seal's
  commit, not from `abfa38d`.

## Result (6 Nov 2026)

Run at 00:12 UTC with the hash of the pushed seal (`4c18a8ae…`) as the seed; graded by the sealed code
(`benchmarks/gigo/runs.json`, `MANIFEST.sha256`, `GRADED.txt`).

| Set | Errors that changed the deck | Harmful (beyond the fitness bar) | Gate 1, at entry | Gate 2, second opinion | Caught by either |
|---|---|---|---|---|---|
| Systematic | 160 | 74 | 0 of 74 | 61 of 61 | 61 of 74 |
| Random typing errors | 49 | 19 | 0 of 19 | 15 of 15 | 15 of 19 |
| Curated | 56 | 27 | 0 of 27 | 23 of 23 | 23 of 27 |
| **All** | **265** | **120** | **0 of 120 (0 %)** | **99 of 99 (100 %)** | **99 of 120 (82 %)** |

No false alarm from either gate on any clean deck (the five FASTSim cars, the seven Simscape round-2 decks and the
stock Hockenheim deck). All four sealed predictions held. 16 errors left their value unchanged and are not
counted; FASTSim itself refused 31 corrupted decks (an efficiency typed in percent, a thousandfold power or mass
slip), which are not counted as harmful.

**What it means.**

* **Second opinion works.** When KinematiK holds the right deck, every harmful error that reached FASTSim was
  caught, with no false alarm: the 0.04 % agreement on clean decks makes any real error stand out.
* **Entry checks do not.** KinematiK accepted every harmful wrong value given to it: a mass in pounds or in grams,
  a wheel radius in millimetres, an efficiency of 93 instead of 0.93, an inverted gear ratio, the driver left out
  of the mass. It refused 5, all efficiency maps typed in percent (values above 1, which have no meaning);
  FASTSim refused those 5 too. Two more rows are marked "refused" in `runs.json` but were not KinematiK checking
  anything: a tire radius in inches or centimetres made KinematiK's own 75 m launch never reach 75 m, silently,
  and the benchmark's clocking then raised. The sealed grading counts no harm for them because no metric came
  back; they are silent, harmful errors too (the same radius in millimetres moved the results by over 1,000×), so
  23, not 21, harmful errors got through on the Formula Student deck.
* **Every harmful error that got through is on the Formula Student deck** (21 graded, plus the 2 above), where no
  external tool was rerun, so only Gate 1 could have caught them. That is also the case of a misread datasheet
  entered in both tools: today nothing in KinematiK catches it.

**Next, as a new sealed round:** plausibility checks at entry (ranges for mass, efficiencies, drag and lift areas,
rolling resistance, wheel radius, grip and gear ratio, with Formula Student cars inside the ranges), tested on a new
seed, with the false-alarm set as the guard against checks that would flag a valid car.

## Round 2: plausibility checks at entry (sealed before running)

**What changed in KinematiK.** `suspension/plausibility.py` holds, for each powertrain input, the range any real
vehicle of its class falls in, why the range is there, and the slip a value outside it usually is. Two classes:
`GENERIC` (an e-scooter to a 40 t truck; catches thousandfold, percent and length-unit slips) and `FSAE` (a Formula
Student car, from the rules and the cars in this repository; also catches pounds for kilograms, ft² for m², a
diameter for a radius and an inverted ratio). A value outside its range raises a `PlausibilityWarning` naming the
likely slip; a drive or gearbox efficiency above 1 is refused. The checks run where inputs enter: the road-load
function, the ICE vehicle, gearbox, engine map and fuel function, and the loader of a Formula Student deck exported
from Simscape.

**What did not change.** The catalog of errors, the decks, the harm bars and the gates. The random typing errors
are drawn again, from the commit hash of this round's seal, so they are new.

**One fix to the harness.** In round 1 a Formula Student launch that never reached 75 m made the benchmark's own
clocking raise, and the row was marked "refused" although KinematiK had checked nothing. Now such a launch is a
result (75 m never reached: infinitely slow, so harmful), not a refusal.

**The honest caveat.** The ranges were written after round 1, by the people who saw which errors got through. Each
range is set by the vehicles of its class, not by the errors (a Formula Student car's wheel is 10 to 13 inches;
the rules cap a pack at 600 V and set a 1525 mm minimum wheelbase), and the false-alarm guard is unchanged: every
existing test and benchmark deck (the FASTSim, GT-SUITE, Argonne, A-FA and Simscape decks) runs with warnings as
errors, and none raises one. The fresh random set is the part no one has seen.

**Predictions, sealed:** Gate 1 catches between 50 and 90 % of the harmful errors overall, and at least 75 % on the
Formula Student deck (where no second opinion is possible); Gate 2 at least 95 %; no false alarm from either gate.
Expected misses, stated now: a factor of 2.2 (pounds) or 1.34 (horsepower) on a road car, whose generic range spans
a scooter to a truck; the driver left out of the mass (16 %, still a plausible car); torque in lb·ft and motor
speed in rad/s on the Formula Student car (both still plausible values).
