# ANSYS run-log consolidation — usage

`suspension/aero/run_log.py`

Takes the spreadsheet the wings team fills in after each Fluent run, throws out the
runs that are not defensible, averages what is left per operating point, and writes
an organised workbook. Every exclusion carries a reason.

```
parse_run_log()  ->  screen()  ->  consolidate()  ->  write_workbook()
```

---

## Where to find it in the app

**Aerodynamics tab (🌬️) → "ANSYS run-log consolidation"**

1. Upload the run log (`.xlsx` / `.csv`) — the raw sheet the team keeps, banner
   row and renamed columns and all.
2. Open **Screening criteria** to adjust thresholds (air density, reference area,
   scratch-row handling, the outlier pass, sign convention). Defaults are the
   documented judgement below.
3. Press **⚡ Screen & consolidate**.

You get headline counts, the consolidated table (one row per operating point),
**every excluded run with its reason**, the full screening report, both downloads,
and a **⬇️ Load into the aero map** button that stages the consolidated points as
`CoeffResult`s for the lap sim.

The view is a thin shell over `run_log.py` — all the physics lives in the module,
so the same results come out of the CLI, the API and the UI.

---

## Quick start

```python
from suspension.aero.run_log import process, write_workbook

report = process("wings_runs.xlsx")
print(report.summary())
write_workbook(report, "aero_consolidated.xlsx")
```

```
15 run(s) parsed from wings_runs.xlsx [Sheet1]
10 accepted, 5 rejected, 3 operating point(s)
  - Front Wing @ 30 mm, 27 m/s: Cl=-0.9330 Cd=0.1990 [1/2 runs kept] — SINGLE RUN
  - Front Wing @ 40 mm, 27 m/s: Cl=-0.8858 Cd=0.1990 [4/6 runs kept] — 4 runs, tight (1.2% spread)
  - Front Wing @ 50 mm, 27 m/s: Cl=-0.8275 Cd=0.1990 [4/7 runs kept] — 4 runs, tight (0.8% spread)
```

From the command line:

```bash
python demo_run_log.py                          # built-in demo sheet
python demo_run_log.py wings_runs.xlsx -o out/  # your own export
```

No Excel? `write_csv_bundle(report, "out/")` writes the same four tables as CSV.

---

## What the input can look like

The parser is built for the sheet as it actually exists, not an idealised one:

- **a banner row above the header** — `Wings Team Simulation Results` / `Volume Mesh
  Metrics` sitting above the real column names. The header is found by scoring rows,
  not by assuming row 1.
- **units in the header** — `Ride-Height (mm)`, `Mass Imbalance (kg/s)`, `Min. Pressure (Pa)`.
- **renamed columns** — `Engineer` for `Contributor`, `Cl`, `Wall Y+`, `Freestream
  Velocity`. Roughly 120 aliases plus regex fallbacks.
- **blank filler rows** at the bottom, dropped.
- **messy cells** — `1.33E-5`, `-1,234.5`, `26,8`, `n/a`, `50 mm`.
- **columns this module does not model** — kept in `row.raw` and listed in the
  Config sheet, never silently dropped.
- **`.xlsx`, `.xlsm`, `.csv`, `.tsv`**, or bytes / a file object / a list-of-lists.

---

## The screening criteria

Every threshold lives in `ScreenConfig`, is written into the output workbook's
Config sheet, and can be overridden. `REJECT` removes a run from the average;
`WARN` keeps it and says so.

### The y+ gate — judged against the row's own turbulence model

This is the one that matters most, and the reason a single global y+ rule would be
wrong. The band comes from the closure named in the `Viscous Model` column:

| Model in the row | Treatment | Wants | Rejected outside |
|---|---|---|---|
| k-epsilon, realizable, RNG, RSM | wall function | y+ 30–300 | y+ 11.06–1000 |
| k-omega SST, Spalart-Allmaras, LES/DES | resolved | y+ ≈ 1 | y+ > 30 |
| k-omega (plain) | automatic / blended | y+ < 300 | y+ > 1000 |
| unrecognised | unknown | — | never rejects; warns on extremes |

The lower bound of 11.06 is not arbitrary — it is where the log law meets the linear
viscous-sublayer profile. Below it, a standard wall function is applying a log law to
a cell that physically sits inside the sublayer, and the wall shear it returns is not
physical. Between 11.06 and 30 you are in the buffer layer: poor practice, flagged
`YPLUS_MARGINAL`, but kept.

`YPLUS_TARGET_MISS` separately compares the achieved `Average Y+` against the
`Desired Y+` the mesh was sized for. A 40% miss means the first-layer height needs
recomputing, even when the achieved value happens to land in band.

### Mesh quality

| Flag | Rejects when | Warns when |
|---|---|---|
| `ORTHO_QUALITY` | min orthogonal quality < 0.10 (Fluent's floor) | < 0.20 |
| `SKEWNESS` | max skewness > 0.95 | > 0.90 |
| `ASPECT_RATIO` | > 100,000 | > 5,000 |
| `MESH_LENGTH_INVERTED` | min surface length > max — the values are swapped | — |
| `MESH_LENGTH_RATIO` | — | max/min > 50:1, i.e. a likely decimal slip |
| `LAYER_COUNT` | < 2 inflation layers | < 5 |
| `FIRST_LAYER_TOO_TALL` | — | first layer height > min surface length |

Aspect ratio is deliberately warn-only by default: inflation-layer cells legitimately
run into the thousands, so a high value is a smell rather than a fault.

### Pressure-field physics — the free sanity check

Assumes gauge pressures (Fluent's default), i.e. the same datum as *q* = ½ρV².

- **`CP_STAGNATION`** — max gauge pressure ÷ *q* should sit at Cp ≈ 1, because total
  pressure at a stagnation point is exactly *q* above static. If it reads 0.4 or 2.5,
  the reference velocity, density or pressure datum for that run is wrong, and **every
  coefficient in the row is off by that ratio**. Rejected outside 0.30–2.00, warned
  outside 0.80–1.30.
- **`CP_NO_SUCTION`** — a downforce surface showing Cp_min > −0.5 almost always means
  the force report summed over the wrong wall zone. That is the first failure mode the
  Fluent validation run-sheet tells you to check, and it is caught here automatically.

### Reference-area consistency

Each row's own numbers imply a reference area: `A = |L| / (q·|Cl|)`. The module takes
the median across the operating point and compares each row against it.

A contributor who normalised by a different area produces coefficients that are
internally perfectly consistent and silently incomparable to everyone else's. Nothing
else in the sheet catches this. Rows more than 25% off are rejected
(`REF_AREA_MISMATCH`); more than 5% off are warned (`REF_AREA_DRIFT`).

The same inferred area backfills blank coefficient cells — the real sheet leaves
`Drag Coefficient` empty — as `Cd = D / (q·A)`. Derived values are reported in their
own `(derived)` column and counted in the case notes; the reported column is never
overwritten. Set `reference_area_m2` in the config to use a known value instead.

### Solver setup — Scheme, Order, Pseudo Time Step, Courant Number, Initialization

These five columns describe **method**, not measurement, so most of them are
judged *comparatively* rather than against a fixed limit.

- **`FIRST_ORDER`** — first-order spatial discretisation is numerically diffusive:
  it smears the very pressure gradients a suction peak is made of, so downforce
  reads low and the wake reads wide. A **warning** by default, because it's a
  legitimate way to *start* a solve and a ramped run may not say so in the sheet.
  Set `reject_first_order=True` once your team agrees every reported run finishes
  second-order. `"First to Second Order"` is recognised as a ramp, not as first.
- **`COURANT_HIGH` / `COURANT_LOW`** — a very large pseudo-transient Courant number
  settles the residuals while the flow field is still moving: converged-*looking*
  and not converged. Very small, and the residual history flattens long before the
  forces stop moving. Warnings either way; a Courant choice doesn't invalidate a
  genuinely converged answer.
- **`SETUP_UNRECORDED`** — the run doesn't state its method, so it can't be compared
  against the rest of its operating point. Not a fault, but it weakens the check below.
- **`SETUP_MISMATCH`** — *the check these columns exist for.* Every other gate asks
  "is this run valid?"; this one asks **"are these runs the same experiment?"** Two
  runs at one ride height, one on k-epsilon and one on k-omega SST, can both pass
  every physics gate and still not be two samples of one quantity — the mean across
  them describes neither. The tool **reports the split rather than picking a winner**,
  because it can't know which model the team meant to standardise on. Set
  `reject_mixed_turbulence=True` once you've decided.

`setup_signature()` deliberately **excludes** Courant number and pseudo time step:
they change the *path* to convergence, not the converged answer, so two runs
differing only there are still comparable.

The consolidated output carries the method with the number — `Viscous Model(s)`,
`Scheme(s)`, `Discretisation`, `Initialization(s)`, `Courant Range` and
`Setup Consistent?` — because a coefficient without its setup isn't reproducible.
A mixed group is called out in the confidence string too:
`4 runs, MIXED SETUP — averaged across different solver methods`.

### Contributor

Was carrying nothing but a name. `report.contributor_stats()` (and the
**Contributors** sheet / CSV) now gives runs submitted, accepted, rejected and the
findings each person hits most. Not a leaderboard — a map of where a recurring
setup mistake lives, so it gets fixed once at the source instead of being screened
out of every batch forever.

### Solution health and bookkeeping

- `MASS_IMBALANCE` — rejects above 1e-3 kg/s, warns above 1e-4. Continuity must close.
- `NOT_CONVERGED` — an explicit `Converged = No` is always rejected. An unconverged
  force is not a force.
- `NO_RESULT` — no lift force and no lift coefficient: nothing to average.
- `LIFT_SIGN` — a positive lift value on a downforce surface. A **warning**, not a
  rejection: either the run genuinely makes lift, or Fluent's up-positive value was
  pasted in without the sign flip. Both are worth a human look.
- `TEST_ROW` — `Khalil - Test`, `scratch`, `ignore`, `wip`, `template`… matched as
  whole words, so `Testarossa` and `latest` do not trip it.

### The statistical pass, last

After every physics gate, runs that survived are compared against their peers at the
same operating point using a **modified z-score** (median/MAD, so the outliers cannot
poison the test that finds them). Beyond 3.5 scores, `STATISTICAL_OUTLIER`.

This pass is **disabled below 4 accepted runs in a group**. With three samples,
"the odd one out" is picking a favourite, not evidence.

---

## Tuning

```python
from suspension.aero.run_log import ScreenConfig, process

cfg = ScreenConfig(
    reference_area_m2=0.268,        # skip inference, use the team's known value
    rho=1.200,                      # hot day at comp
    yplus_wf_warn=(30.0, 200.0),    # tighter than default
    skewness_reject=0.85,           # stricter mesh standard
    enable_outlier_pass=False,      # physics gates only
    reject_test_rows=False,         # flag scratch rows without dropping them
)
report = process("wings_runs.xlsx", cfg)
```

`prefer_latest_per_contributor=True` additionally keeps only each contributor's last
row per operating point. It is **off by default** because it assumes row order is
chronological, which the sheet does not promise — screen on physics first.

---

## The output workbook

| Sheet | What it holds |
|---|---|
| **Consolidated** | One row per operating point — the answer. Mean/SD/min/max are **live formulas** over the Accepted Runs sheet. |
| **Accepted Runs** | The runs behind those means, grouped by case, with *q*, Cp_max, Cp_min, implied reference area and wall treatment alongside. |
| **Rejected Runs** | Every excluded run with its code and the sentence explaining it. |
| **Screening Report** | Every row, every flag, severity, measured value, limit — the full audit trail. Clean rows appear too. |
| **Contributors** | Runs submitted / accepted / rejected per person, with their most common findings. |
| **Config** | Every threshold used, the parse warnings, the unmapped columns, and a flag tally. |

The aggregates are formulas rather than baked numbers so the team can audit the
average without trusting this code, and so the file recalculates if a row is edited or
a run is manually reinstated. Rows are sorted by case, giving each one a contiguous
block, so the formulas are plain `AVERAGE(range)` — no array functions, no
version-dependent behaviour.

Colour: green = accepted clean, amber = accepted with warnings or a single-run/
wide-spread case, red = rejected.

### Reading the confidence column

- `SINGLE RUN — not a mean; no spread available` — one survivor. Treat as one data point.
- `4 runs, tight (0.8% spread)` — ≤5% peak-to-peak.
- `moderate` — ≤15%.
- `POOR AGREEMENT` — >15%. The runs disagree; do not average your way past it.
- `NO DATA — every run at this point was rejected` — the point is still listed, with
  the reasons, rather than vanishing from the output.

---

## Feeding the lap sim

Consolidated points convert into the same `CoeffResult` objects a solver backend
produces, so they flow straight into `AeroMap` and the lap sim:

```python
from suspension.aero import AeroMap, Attitude
from suspension.aero.run_log import process, to_coeff_results

report  = process("wings_runs.xlsx")
results = to_coeff_results(report)
amap    = AeroMap.from_results(results)

q = amap.query(Attitude(ride_height_mm=45, speed_ms=26.8))
print(q.c_lift, q.c_drag, q.extrapolated)
```

Each result carries `provenance.backend = "ansys-run-log"`, the turbulence models
behind it, and `n/total` plus the spread in its notes — a single-run point stays
labelled as one all the way into the map. `force_monitor_range` carries the spread
percentage, so the map's own usability checks see the run-to-run scatter.

**Sign convention:** this sheet already uses negative lift = downforce, which is
KinematiK's convention, so nothing is flipped. This differs from
`backend.read_fluent_csv()`, which *does* flip because Fluent reports lift
up-positive. Do not pre-negate values when filling in the sheet.

---

## API

| Call | Returns |
|---|---|
| `process(source, config=None, sheet=None)` | `ConsolidationReport` — the whole pipeline |
| `parse_run_log(source, sheet=None)` | `(rows, warnings, unmapped, sheet_name, label)` |
| `screen(rows, config=None)` | `list[Verdict]` — one per row, input order |
| `consolidate(verdicts, config=None)` | `list[ConsolidatedCase]` |
| `write_workbook(report, path)` | path written (needs `openpyxl`) |
| `write_csv_bundle(report, dir)` | four CSV paths, consolidated first |
| `consolidated_csv(report)` | CSV text |
| `to_coeff_results(report)` | `list[CoeffResult]` for `AeroMap` |

Useful `ConsolidationReport` members: `.summary()`, `.accepted`, `.rejected`,
`.flag_tally()`, `.cases`, `.parse_warnings`, `.unmapped_headers`, `.ok`.

Useful `Verdict` members: `.accepted`, `.reason()`, `.reject_codes`, `.warn_codes`,
`.flags`, `.derived` (q, Cp, implied area, wall treatment, outlier z).

---

## Notes and limits

- Pure standard library; `openpyxl` imports lazily and only for `.xlsx` read/write,
  so the module is importable and testable with no optional dependencies installed.
- Pressures are assumed **gauge**. If your sheet logs absolute pressure, the
  stagnation check will fire on every row — subtract the operating pressure first.
- Mass imbalance is judged in **absolute** kg/s because the sheet does not carry inlet
  mass flow. If you log it, set `mass_imbalance_warn`/`_reject` to your own fraction
  of inlet flux.
- Grouping buckets ride height to 0.5 mm and speed to 0.5 m/s
  (`ride_height_tol_mm`, `speed_tol_ms`), so 40.0 and 40.1 mm are one point.
- The thresholds are engineering judgement, not laws. They are defaults chosen to be
  defensible and are all in one place precisely so a team can argue with them.

Tests: `tests/test_run_log.py` (102) and `tests/test_run_log_ui.py` (26).
