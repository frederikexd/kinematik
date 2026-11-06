# Garbage-in benchmark

**Round 1 (seal `4c18a8a`): at entry, KinematiK caught 0 of 120 harmful input errors; as a second opinion beside
FASTSim, 99 of 99. Round 2 (seal `383bda9`, with plausibility checks at entry): 65 of 119 at entry, 18 of 20 on the
Formula Student deck, 99 of 99 as a second opinion, 117 of 119 by either. Round 3 (seal `8a02c0f`, decks the checks
were never written against): 64 of 83 at entry on two held-out Formula Student cars (77 %, 95 % CI 67–85 %), 23 of
23 as a second opinion on a held-out road car; no false alarm on any valid car in any round.** Details below.

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

## Result of round 2 (6 Nov 2026)

Run at 00:36 UTC with the hash of the pushed seal (`383bda91…`) as the seed; graded by the sealed code
(`benchmarks/gigo/round2/runs.json`, `MANIFEST.sha256`, `GRADED.txt`).

| Set | Errors | Harmful | Gate 1, at entry | Gate 2, second opinion | Caught by either |
|---|---|---|---|---|---|
| Systematic | 160 | 74 | 45 of 74 | 61 of 61 | 74 of 74 |
| Random typing errors (new draw) | 53 | 19 | 9 of 19 | 15 of 15 | 18 of 19 |
| Curated | 56 | 26 | 11 of 26 | 23 of 23 | 25 of 26 |
| **All** | **269** | **119** | **65 of 119 (55 %)** | **99 of 99 (100 %)** | **117 of 119 (98 %)** |

On the Formula Student deck, where no second opinion is possible, the entry checks caught **18 of 20** harmful
errors (round 1: 0 of 21). No false alarm on any clean deck. All four sealed predictions held: Gate 1 overall
55 % (sealed: 50–90 %), Formula Student deck 90 % (sealed: at least 75 %), Gate 2 100 % (at least 95 %), no false
alarm. Gate 1 also flagged 66 wrong values too small to cross the harm bar.

**What still gets through at entry**, as predicted before the run:

* On road cars, factors their wide range cannot see: a mass in pounds (×2.2 or ÷2.2), a drag area in ft², the drag
  coefficient typed for CdA, horsepower for kW, an efficiency applied twice. Every one of these was caught by the
  second opinion.
* On the Formula Student car, two errors that leave a believable car: the 75 kg driver left out of the mass (16 %
  off) and an inverter efficiency typed 0.89 instead of 0.98 (a dropped digit, 23 % off). No range can catch these;
  only a measurement (corner scales, an inverter datasheet) can.

**What it means.** With both gates, KinematiK caught 117 of the 119 harmful input errors in this benchmark, and
on a Formula Student car, the class it is built for, the entry checks alone catch 9 in 10. The remaining misses
are values that are wrong but physically plausible; they are listed by name.

## Round 3: held-out decks (sealed before running)

**Correction to round 2.** Round 2's plan says every existing test and benchmark deck runs with plausibility warnings
as errors. It did not: the repository's pytest settings made only deprecation warnings errors, so a plausibility
warning in a test would have passed unseen. Round 2's own false-alarm count is unaffected (the harness records the
warnings on every clean deck itself and found none). Checked afterwards with the warning made an error across every
related test: four tests raised it, all feeding textbook numbers on purpose (grip of 100 for a lossless launch, a
0.5 W toy engine map, a 0.3 drivetrain efficiency and a 0.5 ratio testing edge cases); no benchmark car (FASTSim,
GT-SUITE, Argonne, A-FA, Simscape) raised one. The warning is now an error in `pyproject.toml`, and those four tests
carry a mark saying their numbers are deliberate, so the claim holds from here on.

**Why a third round.** The entry ranges were written after round 1, and rounds 1 and 2 used the same decks. Round 3
tests the checks on decks they were not written against:

* `fastsim:bolt`: the 2020 Chevrolet Bolt EV bundled with FASTSim 3.1.0, as bundled (its thermal models on), with
  FASTSim in the loop. On the clean deck KinematiK agrees with FASTSim within 0.04 % on all four cycles, a sixth
  car for the FASTSim benchmark that was never used to build anything.
* `fsae_ev` and `fsae_ice`: the Formula Student EV and combustion car of the powertrain benchmark's own deck
  (`benchmarks/powertrain/inputs.json`, written at the start of the powertrain benchmark, before any of this): a
  300 kg car with a 140s3p 504 V pack and a 7:1 reduction, and a restricted 600 cc four with six gears. Harm is
  measured by KinematiK's own Simscape-validated models.

**What KinematiK gained for it, before the seal:** the whole-deck check now covers the combustion section (brake
thermal efficiency, redline, the engine curve's peak torque and power) and two cross-checks between inputs that must
agree: the declared motor peak torque and top speed against the motor curve, and the redline against the engine
curve (within 5 %). These target unit slips on a single field that leave a believable value.

**Reported with 95 % confidence intervals** (Wilson), per deck and overall.

**Predictions, sealed:** Gate 1 catches at least 75 % of the harmful errors on the two Formula Student decks; between
50 and 95 % overall; Gate 2 (on the Bolt) at least 95 %; no false alarm from either gate.

Before the seal, the harness was smoke-tested on four corrupted rows (the Bolt's mass in pounds; the combustion
car's torque in lb·ft, redline in rad/s and efficiency in percent) to check that every deck runs end to end. That
test caught a harness bug: the Bolt was added to the shared vehicle list, and its corrupted FASTSim runs, made in a
separate process, could not see it. It is now added only where it is used.

## Result of round 3 (6 Nov 2026)

Run at 02:14 UTC with the hash of the pushed seal (`8a02c0f0…`) as the seed; graded by the sealed code
(`benchmarks/gigo/round3/runs.json`, `MANIFEST.sha256`, `GRADED.txt`). 95 % confidence intervals are Wilson's.

| Held-out deck | Harmful errors | Gate 1, at entry | Gate 2, second opinion |
|---|---|---|---|
| Formula Student EV (`fsae_ev`) | 35 | 29 (83 %, CI 67–92 %) | none to compare with |
| Formula Student combustion car (`fsae_ice`) | 48 | 35 (73 %, CI 59–83 %) | none to compare with |
| **Both Formula Student decks** | **83** | **64 (77 %, CI 67–85 %)** | |
| Chevrolet Bolt EV (`fastsim:bolt`) | 23 | 8 (35 %, CI 19–55 %) | **23 (100 %, CI 86–100 %)** |
| **All** | **106** | **72 (68 %, CI 59–76 %)** | |

No false alarm on any of the three clean decks; all four sealed predictions held (Formula Student at entry 77 %,
sealed at least 75 %; overall 68 %, sealed 50–95 %; second opinion 100 %, sealed at least 95 %). The checks also
flagged 40 wrong values too small to cross the harm bar. On the Bolt, entry plus second opinion caught 23 of 23.

**So the entry checks generalize, about as well as on the decks they were written after** (round 2's Formula Student
deck: 18 of 20; here 64 of 83). The interval is the honest headline: on a Formula Student car KinematiK catches
roughly two thirds to five sixths of harmful input errors at entry.

**What got through, by name**, and what it shows:

* **The ICE gearbox's primary reduction** is not checked at all: entered as its inverse it moved the result 67 %,
  and a shifted decimal point made the launch never reach 75 m, silently. Neither range nor cross-check covers it.
  This is the biggest single gap found, and it was found only because the decks were new.
* **The ICE launch speed** in rad/s (12 %) is not checked.
* **Errors that leave a believable car**, as in round 2: the driver left out of the mass, an efficiency applied
  twice or with a dropped digit, a mistyped grip (16 %), a downforce area or pack resistance typo (4–6 %), an
  engine curve in lb·ft (4 %).
* **One crash:** a dropped digit turned the EV's mass into 0. KinematiK warned about it (so it counts as caught) and
  then divided by it. A mass of 0 should be refused outright.

**Next, as a new sealed round on fresh decks:** check the primary reduction and the launch speed (against the
engine curve and redline, as the redline is now), refuse a zero or negative mass, and leave the plausible-but-wrong
class to measurement (corner scales, datasheets), which no range can replace.

## After round 3: three fixes, not yet measured

Made after round 3's result, from the gaps it named; no sealed round has measured them yet, so round 3's numbers
stay the record until one does.

* **A vehicle mass of zero or less is refused** where a deck enters KinematiK, instead of being warned about and
  then divided by.
* **The combustion car's primary reduction is checked** (1 to 4, a direct drive to a motorcycle engine's primary
  gears), and so is the gearing as a whole: the top-gear speed at redline must lie between 20 and 70 m/s for a
  Formula Student car (10 to 150 m/s for any car). An inverted or mistyped primary, gear or final drive moves it out.
* **The launch speed is checked against the redline:** between a fifth of the redline and the redline, which a
  launch speed typed in rad/s is not.

Every deck in the repository still passes clean. To claim these in numbers, run round 4 on decks the checks have not
seen, sealed the same way.
