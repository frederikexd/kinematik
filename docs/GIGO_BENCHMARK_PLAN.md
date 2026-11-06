# Garbage-in benchmark

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
