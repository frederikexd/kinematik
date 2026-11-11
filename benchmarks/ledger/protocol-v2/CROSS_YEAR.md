# Protocol v2 across years

Each test year is predicted with factors fitted only on the other year. The forward test (fit on 2025, predict 2026) is how the blind FSG 2027 test will run. Both years' results were public when this was made, so these are hindcasts.

## Forward: fit on FSG 2025, predict FSG 2026

Factors: skidpad R20 1.131, LC0 1.141, other 1.163 (pooled 1.141); acceleration 1.106 (challenger, fastest quarter: 1.038). Training runs: {'skidpad': 19, 'accel': 18, 'skidpad_R20': 10, 'skidpad_LC0': 3, 'skidpad_other': 6}. Teams with a record: 20.

| Protocol | Event | Cars | Mean abs. error | Median | Bias | Within 5 % | Rank corr. |
|---|---|---|---|---|---|---|---|
| v1 (defaults only) | Skidpad | 22 | 9.7 % | 7.8 % | -9.7 % | 3 of 22 | 0.45 |
| v1 (defaults only) | Acceleration | 25 | 7.7 % | 5.0 % | -5.0 % | 13 of 25 | 0.32 |
| v2 shared factors (1, 2, 3, 5) | Skidpad | 22 | 3.9 % | 2.9 % | -0.5 % | 16 of 22 | 0.50 |
| v2 shared factors (1, 2, 3, 5) | Acceleration | 25 | 6.8 % | 6.8 % | +3.5 % | 6 of 25 | 0.48 |
| v2 shared factors (1, 2, 3, 5) | Skidpad, faster lap | 21 | 4.8 % | | +1.1 % | 13 of 21 | |
| tried: team record on both events | Skidpad | 22 | 3.2 % | 2.0 % | -0.9 % | 18 of 22 | 0.60 |
| tried: team record on both events | Acceleration | 25 | 7.7 % | 6.6 % | +5.4 % | 7 of 25 | 0.64 |
| tried: team record on both events | Skidpad, faster lap | 21 | 3.6 % | | +0.7 % | 17 of 21 | |
| v2 final (1-5, record on skidpad) | Skidpad | 22 | 3.2 % | 2.0 % | -0.9 % | 18 of 22 | 0.60 |
| v2 final (1-5, record on skidpad) | Acceleration | 25 | 6.8 % | 6.8 % | +3.5 % | 6 of 25 | 0.48 |
| v2 final (1-5, record on skidpad) | Skidpad, faster lap | 21 | 3.6 % | | +0.7 % | 17 of 21 | |
| challenger: accel at fastest quarter | Skidpad | 22 | 3.2 % | 2.0 % | -0.9 % | 18 of 22 | 0.60 |
| challenger: accel at fastest quarter | Acceleration | 25 | 4.5 % | 2.9 % | -2.8 % | 18 of 25 | 0.48 |
| challenger: accel at fastest quarter | Skidpad, faster lap | 21 | 3.6 % | | +0.7 % | 17 of 21 | |

Published values set aside by rule 2: Formula Student Team Delft (Delft TU) (car 85): redline_speed_ms 23.1; UGent Racing (Ghent U) (car 51): redline_speed_ms 18.66

## Reverse: fit on FSG 2026, predict FSG 2025

Factors: skidpad R20 1.118, LC0 1.121, other 1.204 (pooled 1.121); acceleration 1.037 (challenger, fastest quarter: 1.028). Training runs: {'skidpad': 22, 'accel': 25, 'skidpad_R20': 16, 'skidpad_LC0': 2, 'skidpad_other': 4}. Teams with a record: 25.

| Protocol | Event | Cars | Mean abs. error | Median | Bias | Within 5 % | Rank corr. |
|---|---|---|---|---|---|---|---|
| v1 (defaults only) | Skidpad | 19 | 8.7 % | 9.4 % | -8.7 % | 6 of 19 | 0.67 |
| v1 (defaults only) | Acceleration | 18 | 12.2 % | 9.6 % | -11.9 % | 6 of 18 | 0.79 |
| v2 shared factors (1, 2, 3, 5) | Skidpad | 19 | 4.7 % | 4.1 % | +1.1 % | 12 of 19 | 0.76 |
| v2 shared factors (1, 2, 3, 5) | Acceleration | 18 | 10.3 % | 6.2 % | -9.0 % | 7 of 18 | 0.79 |
| v2 shared factors (1, 2, 3, 5) | Skidpad, faster lap | 19 | 5.0 % | | +2.5 % | 10 of 19 | |
| tried: team record on both events | Skidpad | 19 | 4.2 % | 4.0 % | +1.9 % | 13 of 19 | 0.82 |
| tried: team record on both events | Acceleration | 18 | 9.5 % | 7.0 % | -7.7 % | 8 of 18 | 0.68 |
| tried: team record on both events | Skidpad, faster lap | 19 | 4.9 % | | +3.3 % | 11 of 19 | |
| v2 final (1-5, record on skidpad) | Skidpad | 19 | 4.2 % | 4.0 % | +1.9 % | 13 of 19 | 0.82 |
| v2 final (1-5, record on skidpad) | Acceleration | 18 | 10.3 % | 6.2 % | -9.0 % | 7 of 18 | 0.79 |
| v2 final (1-5, record on skidpad) | Skidpad, faster lap | 19 | 4.9 % | | +3.3 % | 11 of 19 | |
| challenger: accel at fastest quarter | Skidpad | 19 | 4.2 % | 4.0 % | +1.9 % | 13 of 19 | 0.82 |
| challenger: accel at fastest quarter | Acceleration | 18 | 10.7 % | 7.0 % | -9.8 % | 7 of 18 | 0.79 |
| challenger: accel at fastest quarter | Skidpad, faster lap | 19 | 4.9 % | | +3.3 % | 11 of 19 | |

Published values set aside by rule 2: GET racing Dortmund (Dortmund TU) (car 172): wheelbase_mm 155.0, track_f_mm 124.0

