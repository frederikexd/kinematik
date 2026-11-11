# Registration: other organizers, same cars

Registered 2026-11-11T04:12:55Z, before any prediction for these events was computed. The official results below had already
been downloaded and read when this was written, but no prediction had been compared with them. So this is a
hindcast with the protocol fixed in advance, not a blind test.

## Why

Every earlier ledger entry is scored against one organizer, FSG: one track, one timing system, one week. Teams
race the same car at other events in the same season. This tests whether the frozen v2 protocol carries over to
other organizers' tracks and timing, using only the car data the team published on formulastudent.de.

## Data

| Folder | Event | Results (official) |
|---|---|---|
| `fscz2025/` | Formula Student Czech Republic 2025, Most (before FSG 2025) | EV Skidpad and EV Acceleration 2025 |
| `fscz2026/` | Formula Student Czech Republic 2026, Most, July 19–24 (before FSG 2026) | EV Skidpad and EV Acceleration, official v1.0, 2026-07-28 |
| `fsita2025/` | Formula SAE Italy 2025, Varano de' Melegari, September (after FSG 2025) | Class 1EV Skidpad and Acceleration, official |
| `fsita2026/` | Formula SAE Italy 2026, Varano de' Melegari, September (after FSG 2026) | Class 1EV Skidpad and Acceleration, official |

Cars: every team in `fsg2025/` or `fsg2026/` that took a valid time at these events. Each folder's `mapping.csv`
lists the team as printed and the FSG car of the same season it is matched to. The match is by team name only;
the car data used is the FSG car data of that season, unchanged. Teams with no FSG car data for that season
can't be predicted and are not included. FS Austria's result files redirect to a status page, and FS
Netherlands doesn't link its results, so neither is included. FS East 2026 was cancelled.

## Protocol

1. Physics: `ledger_v2.physics(spec)`, the frozen v2 code (`suspension/ledger_v2.py`, sha256 `dcdfae3e9a2909c4480bb05bf4a35310250ae676243f23f19489130aa9a88d66`).
2. Calibration, fixed per season so that no factor is fitted on the year being predicted:
   - 2026 events use the factors fitted on FSG 2025 only.
   - 2025 events use the factors fitted on FSG 2026 only.
   No team record is used. A record from FSG would carry the same team's same-season result into the prediction.
3. Also reported: v1 (no calibration), and the frozen `CALIBRATION.json` (fitted on FSG 2025 and 2026 together,
   sha256 `b0bcdc04cd9bbc58098160afcf1743430fb717ca5c1df12f76db9a066eca2389`). The frozen file includes FSG results of the same cars' season, so it is labeled as
   such and is not the headline.
4. Scoring: the best official time. Excluded: a best time that includes a penalty, or no valid time. FS Czech and
   FS Italy don't print left and right skidpad laps, so the left/right split rule can't be applied. FS Czech
   prints times to 0.01 s.
5. Reported per event and pooled: mean absolute error, median, bias, within 5 %, rank correlation.

## Expectation, stated in advance

From the FSG tests: skidpad mean absolute error at or below 5 % at each organizer, and acceleration around 7 %.
If another organizer's track or timing differs systematically from FSG's, it shows up as a bias at that event,
and is reported as such.
