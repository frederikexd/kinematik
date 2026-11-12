# Prediction ledger records

One folder per event. Each has `specs.csv` (the teams' published car data, with source URLs), `SEALED.json` (the
sealed predictions: never edit it by hand, as `verify` will catch it), `results.csv` (official times, with source
URLs), `autocross.csv` where present, and `SCORECARD.md` / `scorecard.json`.

| Folder | Kind | What |
|---|---|---|
| `fsg2026/` | hindcast | 25 cars at Formula Student Germany 2026: skidpad and acceleration |
| `fsg2012-dut12/` | hindcast | TU Delft DUT12 at FSG 2012 FSE, the validation case in Doyle et al. (2019), SAE 2019-01-0163 |
| `fsg2023/`, `fsg2024/` | data | specs (from the same team pages) and official results for the registered tests |
| `other-events/` | hindcast, registered | FS Czech 2025/2026 and FS Italy 2025/2026, the same cars predicted from their FSG car data (`REGISTRATION.md`, `RESULTS.md`) |
| `fsg2025/` | data | 21 cars at FSG 2025: specs and official results, the training year for protocol v2 |
| `protocol-v2/` | hindcast + frozen | v2 cross-year test (`CROSS_YEAR.md`, two sealed records) `CALIBRATION.json` (v2) and `CALIBRATION_challenger.json`, both frozen for FSG 2027; `REGISTRATION.md` and `REGISTERED_RESULTS.md` for the tests on unseen data |

How it works and what the results mean: [docs/PREDICTION_LEDGER.md](../../docs/PREDICTION_LEDGER.md).

Check every record at once: `python fsae_suspension/scripts/ledger_v2.py verify-all` (also run by the `reproduce`
workflow, which re-runs the registered tests and fails if any published number changes).
