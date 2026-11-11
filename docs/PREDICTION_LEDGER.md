# Prediction ledger: KinematiK against real cars

The benchmarks show that KinematiK's logic matches other tools. They don't show that it describes real cars. The
ledger tests that, against measurements nobody here controls: official Formula Student results.

For each car, KinematiK predicts the skidpad time and the 75 m acceleration time from the car data the team itself
published. The predictions are sealed, then compared with the times the organizers measured at the event.

Code: `fsae_suspension/suspension/ledger.py`, run with `fsae_suspension/scripts/ledger.py`. Records:
`benchmarks/ledger/<event>/`. Tests: `fsae_suspension/tests/test_ledger.py`.

## Two kinds of entry

- **Blind.** Sealed and pushed to GitHub before the competition runs. The commit time proves the predictions
  existed before the results. This is the real test.
- **Hindcast.** Sealed after the results were already public: a past event, or a case from a paper. The protocol
  below still forbids choosing anything per car or tuning anything to results. Readers only have our word that the
  results did not shape the protocol, though, so a hindcast is development evidence, not proof.

Each record states which kind it is, and so does its scorecard.

## The protocol (version 1)

Version 2, which adds calibration across years, is [further down](#protocol-version-2-calibrated-on-one-year-tested-on-another).

Every input comes from the team's public car data, from the competition rules, or from KinematiK's own defaults.
The same defaults apply to every car. Nothing is chosen per car, and nothing is tuned to results.

| Input | Source |
|---|---|
| Mass, weight split | Published: front + rear weight with a 68 kg driver (FSG car data) |
| Wheelbase, tracks | Published |
| Drive | Published: 4 motors means all-wheel drive, fewer means rear-wheel drive |
| Power | Rule: motors × published power per motor, capped at the FS accumulator limit (80 kW; 85 kW in 2012) |
| Overall ratio, redline | Published: the primary ratio (primary × secondary only when the primary is below 5; rear ratio when front and rear differ) and the max motor rpm |
| Wheel radius | Published tire diameter / 2 × 0.97 (loaded) |
| CG height | Assumed: KinematiK default (300 mm) |
| Tire grip | Assumed: KinematiK `default_tire()` |
| Aero | Assumed: KinematiK defaults, ClA 2.6 m², CdA 1.1 m² |
| Motor torque | Assumed: enough for 2 g at the wheels, so the launch is grip-limited (teams don't publish peak torque) |
| Skidpad | `laptime.skidpad_time`, radius 9.125 m |
| Acceleration | `laptime.acceleration_time`, 75 m from rest |
| Scoring exclusions | Skidpad runs whose left and right laps differ by more than 10 %, or whose best time includes a penalty. They are listed, but left out of the summary. |

## How a record is sealed

A sealed record contains the specs, every model input with its source (published, rule or assumed), the
predictions, the protocol, a SHA-256 fingerprint of the code that made them (`ledger.py`, `laptime.py`,
`dynamics.py`, `tiremodel.py`) and the time. The record's own SHA-256 is stored with it.

`verify` checks two things: that the record hasn't been changed since it was sealed, and that today's code still
reproduces its predictions. `score` refuses to score a record that has been changed.

```
python fsae_suspension/scripts/ledger.py seal   --specs benchmarks/ledger/<event>/specs.csv --event "<event>" \
       --kind blind --out benchmarks/ledger/<event>/SEALED.json      # then commit and push it
python fsae_suspension/scripts/ledger.py score  --record benchmarks/ledger/<event>/SEALED.json \
       --results benchmarks/ledger/<event>/results.csv --out benchmarks/ledger/<event>
python fsae_suspension/scripts/ledger.py verify --record benchmarks/ledger/<event>/SEALED.json
```

## Entry 1: FSG 2026 (hindcast, 25 cars)

- **Specs.** 25 cars, from the 2026 car data on each team's formulastudent.de page. Every row has its URL in
  `specs.csv`.
- **Results.** The official FSG26 Skidpad Results v2 and Acceleration Results v2, both dated 2026-08-16. The
  links are in `results.csv`.
- **Sealing.** The record was sealed after those results were public, so this entry is a hindcast. Protocol v1 was
  written before any prediction was compared with the results, and it was not changed after.

| Event | Cars scored | Mean abs. error | Median | Bias | Within 5 % | Rank correlation |
|---|---|---|---|---|---|---|
| Skidpad | 22 (3 excluded) | 9.7 % | 7.8 % | −9.7 % | 3 | 0.45 |
| Acceleration | 25 | 7.7 % | 5.0 % | −5.0 % | 13 | 0.32 |

A negative bias means KinematiK predicted faster than the official time. Every car and every miss is listed in
[`benchmarks/ledger/fsg2026/SCORECARD.md`](../benchmarks/ledger/fsg2026/SCORECARD.md).

### What it shows

- **Skidpad: the defaults are optimistic for the field.** Every prediction is faster than the official time. The
  fastest official car (TU München, 4.678 s) is within 2.5 %. The field average is 9.7 % slower than predicted.
  The predictions spread little (4.56 to 4.70 s) because every car gets the same tire, CG height and aero; only
  mass, weight split and track differ. That limits the rank correlation to 0.45.
- **Acceleration: about half within 5 %.** 13 of 25 cars are within 5 %. Of the 10 fastest officially, 9 are
  within 3.5 %; the tenth is TU Delft, covered below. The cars that were much slower than predicted are spread down the order. A prediction is what the
  published spec allows, so a large negative miss also includes launch, traction control and driver. This
  hindcast can't separate those from model error.
- **Two misses show the published data contradicting the official time.** TU Delft (+16.8 %) published 13,250 rpm
  at an 11.84 ratio. That caps the car at 23.1 m/s on 16-inch tires, yet its official run averaged 22.0 m/s over
  75 m, which needs a much higher top speed. UGent (+17.0 %) published 11,600 rpm at 12.83, a 18.7 m/s cap, against
  an official average of 16.7 m/s. In both cases the published rpm or ratio can't be right. The ledger catches
  inconsistent inputs as well as model error.

### What it does not show

It does not show that KinematiK predicts these cars' times within any band. This is a hindcast on public specs,
with tire, CG height and aero assumed the same for every car. The honest reading is a first measurement of how far
default KinematiK sits from real Formula Student cars: optimistic by about 10 % on skidpad and 5 % on
acceleration.

## Entry 2: TU Delft DUT12 at FSG 2012, the paper case (hindcast)

Doyle, Cunningham, White and Early (2019), *Lap Time Simulation Tool for the Development of an Electric Formula
Student Car*, SAE Technical Paper 2019-01-0163, validate their simulator on TU Delft's DUT12. Their acceleration
result is 3.494 s against the official 3.454 s (+1.17 %). They estimated some unpublished parameters, which they
do not list.

KinematiK used only DUT12's public FSG car data, under the same protocol. It predicts **3.267 s, −5.4 %** against
the official 3.454 s (FSG 2012 FSE Acceleration results). That is consistent with the −5.0 % acceleration bias on
FSG 2026. The paper's simulator was closer. It also had the team's own parameter estimates, where KinematiK had only
the public spec sheet.

## Protocol version 2: calibrated on one year, tested on another

Version 1 left a shared offset and a few misses caused by the teams' own data. Version 2 keeps v1's physics and adds
five rules. Each rule is the same for every car, and each is fitted only on a training competition, never on the one
being predicted. Code: `fsae_suspension/suspension/ledger_v2.py`, run with `fsae_suspension/scripts/ledger_v2.py`.
Records: `benchmarks/ledger/protocol-v2/`. Tests: `fsae_suspension/tests/test_ledger_v2.py` (100 % statement and
branch coverage).

1. **Shared calibration.** One skidpad time factor per tire compound group (Hoosier R20, LC0, other) and one
   acceleration factor. Each is the median of official ÷ predicted over the training competition's clean runs. A
   group with fewer than 4 training cars uses the pooled factor.
2. **Self-contradicting data is set aside.** A published value outside the plausible Formula Student range counts
   as not published (Dortmund 2025 lists a 155 mm wheelbase and 124 mm track). A redline and ratio that cap the car
   below 25 m/s count as a data error: the motor map is dropped and the power limit applies. This sets aside TU
   Delft 2026 (23.1 m/s cap) and UGent 2026 (18.7 m/s).
3. **Published data v1 ignored.** The tire compound (rule 1 groups by it) and metric tire sizes such as
   "175/518 R13", where 518 is the diameter in mm.
4. **Team track record, skidpad only.** A team's official ÷ calibrated prediction from the previous competition,
   shrunk halfway to 1, multiplies this year's skidpad prediction. It uses only results that are public before the
   event.
5. **Skidpad as driven.** The path radius is the rules' 7.625 m inner circle plus 0.25 m plus half the car's own
   track, instead of a fixed 9.125 m. The faster of the left and right laps is scored as a second channel.

**Aero is not used.** Teams' aero entries proved unreliable: TU Delft and GFR run wings in 2026, but their car data
doesn't say so.

### Cross-year test

The test data is FSG 2025, with 21 teams from the formulastudent.de team pages (`benchmarks/ledger/fsg2025/`) and
the official FSG25 results, plus FSG 2026. Each year is predicted with factors fitted only on the other year.
Forward (fit on 2025, predict 2026) is how the blind FSG 2027 test will run. Both years' results were already public,
so both directions are hindcasts. Full tables: [`CROSS_YEAR.md`](../benchmarks/ledger/protocol-v2/CROSS_YEAR.md).

| Test | Event | v1 mean abs. error | v2 mean abs. error | v2 within 5 % | v2 rank corr. |
|---|---|---|---|---|---|
| Forward (2025 → 2026) | Skidpad | 9.7 % | **3.2 %** (median 2.0 %) | 18 of 22 | 0.60 |
| Forward (2025 → 2026) | Acceleration | 7.7 % | 6.8 % | 6 of 25 | 0.48 |
| Reverse (2026 → 2025) | Skidpad | 8.7 % | **4.2 %** | 13 of 19 | 0.82 |
| Reverse (2026 → 2025) | Acceleration | 12.2 % | 10.3 % | 7 of 18 | 0.79 |

What this shows:

- **Skidpad: the error is more than halved in both directions.** The bias is gone (−0.9 % forward, +1.9 %
  reverse). The rank correlation rises from 0.45 to 0.60 forward. The team record adds 0.7 points forward and
  0.5 reverse on top of the shared factors.
- **Acceleration: v2 does not fix it.** The fitted factor moved between years: 1.106 from 2025 and 1.037 from
  2026. A factor fitted on one year therefore over-corrects the other. The forward mean error falls from 7.7 % to
  6.8 %, but only 6 of 25 cars are within 5 %, against 13 of 25 under v1, and the bias flips from −5.0 % to +3.5 %.
  With the team record also applied to acceleration, forward got worse (7.7 %) while reverse got better (9.5 %),
  so the record is left out for acceleration. The acceleration gap is launch, traction control and driver: a public
  spec sheet doesn't carry those, and a single shared factor doesn't capture them either.
- **One caveat on rule 2.** The 25 m/s threshold was set after seeing the TU Delft and UGent misses in v1. It sits
  well below what any 80 kW car reaches in 75 m (about 30 m/s), so it shouldn't catch a correct spec. But it was
  not fixed blind. FSG 2027 is the first test where it is.

### Why some cars will always miss

No spec-based model can predict a bad day on track. At FSG 2025, ARUS (4 × 35 kW) ran acceleration in 6.06 s and
Schanzer (4 × 35 kW) in 6.00 s, while the top cars with the same layout ran about 3.45 s. Those runs were limp mode,
a cautious driver or a traction problem, and a published spec sheet carries none of that. A ledger that put every
car within 5 % would have to be tuned to the results, and then it would prove nothing. The scorecards list those
cars as misses.

### A challenger, registered and not adopted

The acceleration factor is the median ratio, so the bad days pull it up and push the cars that ran to their spec
too slow. Calibrating to the fastest quarter of the training field instead (`CHALLENGER_ACCEL_QUANTILE`, 0.25)
targets the cars that achieved their spec:

| Acceleration | v2 (median) | Challenger (fastest quarter) |
|---|---|---|
| Forward (2025 → 2026): mean abs. error | 6.8 % | 4.5 % |
| Forward: within 5 % | 6 of 25 | 18 of 25 |
| Reverse (2026 → 2025): mean abs. error | 10.3 % | 10.7 % |
| Reverse: within 5 % | 7 of 18 | 7 of 18 |

The challenger was proposed after the 2026 test had been seen. Choosing it on that evidence would be tuning to the
test, so v2 keeps the median it was declared with. Both are frozen and both will be sealed blind for FSG 2027. The
blind result decides whether the challenger becomes protocol v3, and both scores are published either way.

### The registered tests: data nobody here had looked at

Before any FSG 2023 or 2024 result was downloaded, and before any autocross result was compared with a
prediction, [`REGISTRATION.md`](../benchmarks/ledger/protocol-v2/REGISTRATION.md) fixed three things: the two
frozen calibrations (by SHA-256), the tests, and the decision rule. The results were public, so this is still a
hindcast. What it removes is the chance to fit the protocol to the data after seeing it. All numbers are in
[`REGISTERED_RESULTS.md`](../benchmarks/ledger/protocol-v2/REGISTERED_RESULTS.md).

The 2023 and 2024 car data was taken from the same team pages as the 2025 and 2026 data (each page holds every
season). Only teams already in the ledger were included, and only seasons the team page lists.

**Test A: frozen v2 on FSG 2023 and 2024, no team record.**

| Event | 2023 | 2024 | Pooled |
|---|---|---|---|
| Skidpad | 4.3 %, 9 of 13 within 5 % | 5.3 % (median 1.7 %), 8 of 13 | 4.8 %, 17 of 26 |
| Acceleration | 4.3 %, 8 of 11 | 8.8 %, 9 of 18 | 7.1 %, 17 of 29 |

- **Skidpad.** The registered expectation was a mean error at or below 5 % in each year. 2023 met it; 2024 missed
  by 0.3 points. Most 2024 cars were close (median 1.7 %), but a few were far off: PWR Wrocław ran 7.17 s.
- **The challenger lost.** Pooled acceleration came out at 7.7 % with 14 of 29 within 5 %, against v2's 7.1 % and
  17 of 29. By the registered rule, v2 stands. The 2026 gain that suggested the challenger did not carry to
  data it hadn't seen. This is the reason the challenger wasn't adopted on the 2026 numbers alone.

**What the calibration factor is.** Each year was fitted on its own:

| Year | Skidpad factor | Acceleration factor |
|---|---|---|
| 2023 | 1.127 | 1.057 |
| 2024 | 1.157 | 1.108 |
| 2025 | 1.141 | 1.106 |
| 2026 | 1.121 | 1.037 |

- **Skidpad: a stable physical offset, not a fudge factor.** The factor stays within ±1.6 % of 1.137 over four
  years and 67 scored skidpad runs. Skidpad time scales with one over the square root of lateral acceleration, so a factor
  of 1.137 means the car's effective grip is 1/1.137² ≈ 0.77 of the model's. The model's default tire and aero
  hold 1.69 g on the FSG skidpad. The official times imply about 1.3 g as driven (median; best 1.65 g). The
  factor absorbs what the default tire doesn't model: the real tire on FSG asphalt at that temperature, a driver
  short of the limit, and a line longer than the ideal circle. Anyone can check this, because it is one number,
  and it is the same for every car and every year.
- **Acceleration: unstable, so it isn't one physical effect.** The factor ranges from 1.037 to 1.108. A shared
  number can't capture launch, traction control and driver, and that is why acceleration stays at about 7 %.

**Test B: autocross relative pace, FSG 2023 to 2026.** Each car's lap on KinematiK's representative autocross was
scaled by the median of the other cars at the same event. That predicts each car's time from its spec relative to
the field. It was compared with a spec-blind baseline that predicts every car at the field median.

| Year | Cars | KinematiK | Spec-blind baseline | Rank correlation |
|---|---|---|---|---|
| 2023 | 9 | 6.9 % | 7.9 % | 0.72 |
| 2024 | 10 | 8.1 % | 6.0 % | −0.59 |
| 2025 | 12 | 12.6 % | 11.1 % | 0.12 |
| 2026 | 15 | 10.0 % | 10.6 % | 0.60 |

**A negative result.** From public specs alone, KinematiK does not predict autocross pace better than assuming
every car is average. It wins in two years and loses in two, and the rank correlation runs from −0.59 to 0.72.
Between competitive FS cars, autocross pace is decided by things a spec sheet doesn't carry: tires and their
temperature, aero (not used here), setup and driver. This test does not show that KinematiK's lap simulation is
wrong. It shows that a public spec sheet is not enough input to rank autocross pace. A team that loads its own
tire data, aero map and track gives the model what this test withheld. That is the next test (below).

### Where each open point stands

| Point | Status |
|---|---|
| All hindcast | Partly closed. Tests A and B were registered before their data was seen. FSG 2027 is the first truly blind test, and v2 and the challenger are frozen and pushed for it. |
| Calibration looks like a fudge factor | Closed for skidpad: the factor is stable to ±1.6 % across four years and maps to effective grip. Open for acceleration: its factor moves by year. |
| Acceleration weak | Open. 7.1 % pooled on unseen years. The registered challenger did not fix it. |
| Small sample | Reduced: four FSG years, 67 scored skidpad runs and 72 acceleration runs. Still one competition. |
| Autocross / endurance, real suspension | Autocross from public specs fails against a spec-blind baseline (above). A team's own car is needed, and the blind team test below is ready for it. Not yet run. |

### Frozen for FSG 2027

Two calibration files are in `benchmarks/ledger/protocol-v2/`. Both are fitted on FSG 2025 and 2026 together:

- `CALIBRATION.json` is v2. Skidpad factors: R20 1.121, LC0 1.075, other 1.176. Acceleration factor: 1.067 (median).
- `CALIBRATION_challenger.json` is the same, except that acceleration is 1.032 (fastest quarter).

Each file carries the team record from FSG 2026, the code fingerprint and its own SHA-256. `seal` refuses a
calibration file that has been changed. Both are pushed before any FSG 2027 data exists, so the FSG 2027 entry tests
them blind.

## Next: the blind test (FSG 2027)

FSG 2027 has no teams registered yet. When teams publish their 2027 car data, the steps are:

1. Copy the car data into `benchmarks/ledger/fsg2027/specs.csv`.
2. Seal it under v1, v2 and the v2 challenger, and commit and push all three records **before the dynamic
   events start**:

   ```
   python fsae_suspension/scripts/ledger.py seal --specs benchmarks/ledger/fsg2027/specs.csv \
          --event "FSG 2027" --kind blind --out benchmarks/ledger/fsg2027/SEALED.json
   python fsae_suspension/scripts/ledger_v2.py seal --calibration benchmarks/ledger/protocol-v2/CALIBRATION.json \
          --specs benchmarks/ledger/fsg2027/specs.csv --event "FSG 2027" --kind blind \
          --out benchmarks/ledger/fsg2027/SEALED_v2.json
   python fsae_suspension/scripts/ledger_v2.py seal \
          --calibration benchmarks/ledger/protocol-v2/CALIBRATION_challenger.json \
          --specs benchmarks/ledger/fsg2027/specs.csv --event "FSG 2027" --kind blind \
          --out benchmarks/ledger/fsg2027/SEALED_v2_challenger.json
   ```

3. When the official results are out, copy them into `results.csv`. Then run `score` on all three records.

Hits and misses are published either way. The expectation, from the forward and registered tests, is a skidpad error
of about 3 to 5 % and an acceleration error of about 7 %.

## Next: a team's own car, blind, on its next test day

The ledger tests what public data allows. Prevalidation means a team's own car, with its tire data, measured mass
and CG, aero and powertrain, and the decision between setups. No public dataset has both a full FS car
definition and measured runs. The closest found are these:
- TU/e's URE11 report measured damper travel for three anti-squat settings but didn't publish the hardpoints.
- PoliTo's SC18 thesis logged the car but didn't reproduce its hardpoint table and ran no setup A/B.

So this test needs a team.

`fsae_suspension/scripts/team_test.py` makes it one command
([how-to](../benchmarks/team-test/README.md); code `suspension/team_test.py`; tests `tests/test_team_test.py`):

1. The team writes a plan: base car plus two or more setups it will run.
2. KinematiK seals the predictions for every setup and every A vs B difference.
3. The team pushes the record, or posts its SHA-256 publicly, before the test day. That makes it blind on the
   team's own schedule, not a year away.
4. After the test, `score` reports the absolute error and, for each pair of setups, whether KinematiK called the
   direction right and how far off the size of the change was. If the plan includes the car's hardpoints, it
   also scores camber and toe against wheel travel, measured in the garage with an angle gauge, in degrees
   against a tolerance fixed in advance. Measured differences within the team's stated
   noise count as ties. Private tire files enter only as a hash.

This is the test that decides whether KinematiK ranks designs correctly when the inputs are good. Until a team
has run it, that is not shown. If FSG 2027 misses that, the scorecard will say so.
