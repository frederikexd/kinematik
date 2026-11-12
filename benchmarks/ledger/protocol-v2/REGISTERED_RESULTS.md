# Results of the registered tests

Protocol: [`REGISTRATION.md`](REGISTRATION.md), sha256 `0999cc2f16666f52d991a65ac14a0c0a4cb0a101eb94d3493c2294799558d88b`. Every number below comes from `python fsae_suspension/scripts/ledger_v2.py registered`.

## Test A: frozen v2 and the challenger on FSG 2023 and 2024

| Calibration | Year | Event | Cars | Mean abs. error | Median | Bias | Within 5 % | Rank corr. |
|---|---|---|---|---|---|---|---|---|
| v2 (median) | 2023 | Skidpad | 13 | 4.3 % | 3.1 % | +0.1 % | 9 of 13 | 0.63 |
| v2 (median) | 2023 | Acceleration | 11 | 4.3 % | 3.5 % | +2.7 % | 8 of 11 | 0.79 |
| v2 (median) | 2024 | Skidpad | 13 | 5.3 % | 1.7 % | -2.3 % | 8 of 13 | 0.40 |
| v2 (median) | 2024 | Acceleration | 18 | 8.8 % | 4.8 % | -5.8 % | 9 of 18 | 0.71 |
| challenger (fastest quarter) | 2023 | Skidpad | 13 | 4.3 % | 3.1 % | +0.1 % | 9 of 13 | 0.63 |
| challenger (fastest quarter) | 2023 | Acceleration | 11 | 4.2 % | 4.2 % | -0.7 % | 8 of 11 | 0.79 |
| challenger (fastest quarter) | 2024 | Skidpad | 13 | 5.3 % | 1.7 % | -2.3 % | 8 of 13 | 0.40 |
| challenger (fastest quarter) | 2024 | Acceleration | 18 | 9.9 % | 7.0 % | -9.0 % | 6 of 18 | 0.71 |

Pooled over both years:

| Calibration | Event | Cars | Mean abs. error | Within 5 % |
|---|---|---|---|---|
| v2 (median) | Skidpad | 26 | 4.8 % | 17 of 26 |
| v2 (median) | Acceleration | 29 | 7.1 % | 17 of 29 |
| challenger (fastest quarter) | Skidpad | 26 | 4.8 % | 17 of 26 |
| challenger (fastest quarter) | Acceleration | 29 | 7.7 % | 14 of 29 |

**Decision rule (registered): v2 stands.** Pooled acceleration: challenger 7.7 %, 14 within 5 %; v2 7.1 %, 17 within 5 %.

## Test B: autocross relative pace, FSG 2023 to 2026

| Year | Cars | Model mean abs. error | Model within 5 % | Rank corr. | Spec-blind baseline | Baseline within 5 % |
|---|---|---|---|---|---|---|
| 2023 | 9 | 6.9 % | 5 of 9 | 0.72 | 7.9 % | 5 of 9 |
| 2024 | 10 | 8.1 % | 4 of 10 | -0.59 | 6.0 % | 5 of 10 |
| 2025 | 12 | 12.6 % | 0 of 12 | 0.12 | 11.1 % | 3 of 12 |
| 2026 | 15 | 10.0 % | 2 of 15 | 0.60 | 10.6 % | 3 of 15 |

## The calibration factors, fitted on each year alone

| Year | Skidpad factor (pooled) | Skidpad cars | Implied lateral g, model | Implied lateral g, official (median) | Acceleration factor | Acceleration cars |
|---|---|---|---|---|---|---|
| 2023 | 1.127 | 13 | 1.69 g | 1.30 g | 1.057 | 11 |
| 2024 | 1.157 | 13 | 1.67 g | 1.27 g | 1.108 | 18 |
| 2025 | 1.141 | 19 | 1.69 g | 1.30 g | 1.106 | 18 |
| 2026 | 1.121 | 22 | 1.69 g | 1.36 g | 1.037 | 25 |
