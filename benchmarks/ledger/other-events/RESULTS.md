# Results: other organizers, same cars

Protocol: [`REGISTRATION.md`](REGISTRATION.md), sha256 `8234b2b32b1b64bdf39bae76361373cd2acae043dfa02fd6d503d123c6e645ad`. Every number below comes from `python fsae_suspension/scripts/ledger_v2.py other-events`.

Headline: v2 with factors fitted on the other FSG year (never on the season being predicted), no team record.

| Event | Model | Event type | Cars | Mean abs. error | Median | Bias | Within 5 % | Rank corr. |
|---|---|---|---|---|---|---|---|---|
| FS Czech 2026 | v2, factors from FSG 2025 | Skidpad | 10 | 3.6 % | 2.6 % | -0.7 % | 7 of 10 | 0.87 |
| FS Czech 2026 | v2, factors from FSG 2025 | Acceleration | 10 | 4.4 % | 3.2 % | -0.4 % | 8 of 10 | 0.55 |
| FS Czech 2026 | v1, no calibration | Skidpad | 10 | 9.3 % | 9.1 % | -9.3 % | 2 of 10 | 0.86 |
| FS Czech 2026 | v1, no calibration | Acceleration | 10 | 7.6 % | 7.7 % | -7.1 % | 2 of 10 | 0.76 |
| FS Czech 2026 | v2, frozen (FSG 2025 + 2026) | Skidpad | 10 | 3.8 % | 3.0 % | -1.6 % | 7 of 10 | 0.89 |
| FS Czech 2026 | v2, frozen (FSG 2025 + 2026) | Acceleration | 10 | 4.6 % | 2.1 % | -3.9 % | 6 of 10 | 0.55 |
| FS Italy 2026 | v2, factors from FSG 2025 | Skidpad | 3 | 7.7 % | 8.9 % | +6.2 % | 1 of 3 | 1.00 |
| FS Italy 2026 | v2, factors from FSG 2025 | Acceleration | 4 | 11.7 % | 10.5 % | +11.7 % | 0 of 4 | -0.40 |
| FS Italy 2026 | v1, no calibration | Skidpad | 3 | 5.4 % | 2.4 % | -3.8 % | 2 of 3 | 1.00 |
| FS Italy 2026 | v1, no calibration | Acceleration | 4 | 4.0 % | 3.0 % | +1.1 % | 3 of 4 | -0.40 |
| FS Italy 2026 | v2, frozen (FSG 2025 + 2026) | Skidpad | 3 | 6.7 % | 7.9 % | +5.9 % | 1 of 3 | 1.00 |
| FS Italy 2026 | v2, frozen (FSG 2025 + 2026) | Acceleration | 4 | 7.8 % | 6.7 % | +7.8 % | 2 of 4 | -0.40 |
| FS Czech 2025 | v2, factors from FSG 2026 | Skidpad | 11 | 4.8 % | 2.7 % | -1.2 % | 8 of 11 | 0.53 |
| FS Czech 2025 | v2, factors from FSG 2026 | Acceleration | 10 | 11.6 % | 8.8 % | -11.6 % | 4 of 10 | 0.79 |
| FS Czech 2025 | v1, no calibration | Skidpad | 11 | 10.4 % | 8.6 % | -10.4 % | 2 of 11 | 0.11 |
| FS Czech 2025 | v1, no calibration | Acceleration | 10 | 14.8 % | 12.1 % | -14.8 % | 1 of 10 | 0.79 |
| FS Czech 2025 | v2, frozen (FSG 2025 + 2026) | Skidpad | 11 | 5.5 % | 2.8 % | -2.3 % | 8 of 11 | 0.42 |
| FS Czech 2025 | v2, frozen (FSG 2025 + 2026) | Acceleration | 10 | 9.9 % | 6.2 % | -9.1 % | 4 of 10 | 0.79 |
| FS Italy 2025 | v2, factors from FSG 2026 | Skidpad | 2 | 4.1 % | 4.1 % | +4.1 % | 1 of 2 | n/a |
| FS Italy 2025 | v2, factors from FSG 2026 | Acceleration | 2 | 19.6 % | 19.6 % | -17.1 % | 1 of 2 | n/a |
| FS Italy 2025 | v1, no calibration | Skidpad | 2 | 7.1 % | 7.1 % | -7.1 % | 1 of 2 | n/a |
| FS Italy 2025 | v1, no calibration | Acceleration | 2 | 16.2 % | 16.2 % | -16.2 % | 1 of 2 | n/a |
| FS Italy 2025 | v2, frozen (FSG 2025 + 2026) | Skidpad | 2 | 2.5 % | 2.5 % | +0.8 % | 2 of 2 | n/a |
| FS Italy 2025 | v2, frozen (FSG 2025 + 2026) | Acceleration | 2 | 20.1 % | 20.1 % | -14.7 % | 0 of 2 | n/a |

## Pooled over all four events

| Model | Event type | Cars | Mean abs. error | Bias | Within 5 % |
|---|---|---|---|---|---|
| v2 (other FSG year) | Skidpad | 26 | 4.6 % | +0.2 % | 17 of 26 |
| v2 (other FSG year) | Acceleration | 26 | 9.5 % | -4.1 % | 13 of 26 |
| v1, no calibration | Skidpad | 26 | 9.2 % | -9.0 % | 7 of 26 |
| v1, no calibration | Acceleration | 26 | 10.5 % | -9.5 % | 7 of 26 |
| v2, frozen (FSG 2025 + 2026) | Skidpad | 26 | 4.7 % | -0.9 % | 18 of 26 |
| v2, frozen (FSG 2025 + 2026) | Acceleration | 26 | 8.3 % | -4.9 % | 12 of 26 |
