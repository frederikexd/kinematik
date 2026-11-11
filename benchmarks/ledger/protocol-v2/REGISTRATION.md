# Registration: tests on data not yet looked at

Registered 2026-11-07T11:41:50Z, before any FSG 2023 or FSG 2024 result, and before any autocross result of FSG 2023, 2024 or 2025,
was downloaded for this project. (The FSG 2026 autocross table had been downloaded, but not compared with any
prediction.) The results were public, so these are still hindcasts. What this file fixes is that the protocol,
the two calibrations and the decision rule were set before the data was seen. The file's SHA-256 is in the
commit that adds it.

## Frozen inputs

- `CALIBRATION.json` (v2, acceleration at the median): sha256 `b0bcdc04cd9bbc58098160afcf1743430fb717ca5c1df12f76db9a066eca2389`
- `CALIBRATION_challenger.json` (acceleration at the fastest quarter): sha256 `88667c6b5e5e5e7462cb5f324c22be2b6447420af98eb0cabb043ca54dc0edef`
- `suspension/ledger_v2.py`: sha256 `dcdfae3e9a2909c4480bb05bf4a35310250ae676243f23f19489130aa9a88d66`

## Test A: skidpad and acceleration, FSG 2023 and FSG 2024

1. Cars: every team in `fsg2025/` or `fsg2026/` whose FSG team page lists car data for that year, and that has an
   official FSG result for that year. Car data is copied as published, with the same fields and conventions as
   `fsg2025/specs.csv`. Nothing is chosen per car.
2. Predictions: `ledger_v2.predict(physics(spec), calibration, record=None)`, with both frozen calibrations and no
   team record. The frozen record comes from FSG 2026, after these events, so it is not used.
3. Scoring: as in `ledger.score`, with the same exclusions (penalty in the best run; skidpad left/right split over
   10 %).
4. Decision rule for the acceleration challenger: it replaces the median in a protocol v3 only if, pooled over FSG
   2023 and 2024, it has the lower mean absolute error AND at least as many cars within 5 %. Otherwise v2 stands.
   This test does not settle the question; FSG 2027 still runs blind with both.
5. Expectation, stated in advance: skidpad mean absolute error at or below 5 % in each year.

## Test B: autocross relative pace, FSG 2023 to 2026

The autocross track changes every year and is not published as a map, so an absolute time can't be predicted. What
can be tested is whether the spec ranks and spaces the cars correctly, which is the comparison a team uses in
prevalidation.

1. Physics: each car's lap on `laptime.default_autocross()` with the v1 build of its sanitized spec
   (`ledger_v2.sanitize`, then `ledger.build`). No calibration file is used.
2. Scale: each car's prediction is multiplied by the median of official / predicted over the OTHER scored cars at
   the same event (leave one out). That is one shared number per event, which carries the track length; it never
   uses the car's own time.
3. Scored runs: the best official autocross time, excluded if it includes a penalty or there is no valid time.
4. Reported: mean absolute error, median, within 5 %, and Spearman rank correlation per year. A spec-blind baseline
   is reported alongside: every car predicted at the event median. The model is only useful if it beats that
   baseline.
