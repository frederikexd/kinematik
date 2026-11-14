# Measured validation protocol: pack and PCB trace temperatures

Registered 2026-11-13, before any measurement. The acceptance limits below are fixed here and are scored by
`kinematik-validate` (`fsae_suspension/suspension/thermal_validation.py`). A miss is published as a miss.

Why: every benchmark so far verifies KinematiK against other tools and correlations. Cooling correlations scatter
±15-25 % against the measurements behind them, so a pack or trace temperature stays an estimate until it is compared
with a measured one. One logged test turns it into a validated figure, or shows where it is wrong.

## A. Pack or module (air-cooled cylindrical cells)

**Hardware.** A module or a representative slice of one (a few rows along the airflow and across it is enough),
in its real duct or in still air, with its real busbars.

**Instrumentation.**
- Type-K or T thermocouples bonded to the can wall (thermal epoxy or Kapton tape, mid-height), at least: the first
  row at the inlet, the last row at the outlet, and one cell in the middle. Note each as `T_<row>_<col>` in the
  module grid, counted from 0, row 0 along the inlet face's first line, columns along the flow.
- Pack current from the BMS, a shunt or a Hall sensor, at 10 Hz or faster.
- Inlet air temperature (`inlet_c`), and the airflow: duct flow in cfm from an anemometer across the inlet face, or
  "still air" with the fan off.
- If possible, a DC resistance per cell (pulse test, ΔV/ΔI at 10 A for 1 s). Otherwise the datasheet's DC value,
  where it gives one.

**Test.** Start at ambient. Run a duty cycle that is representative of an endurance lap (or a stepped constant
current) for at least 20 minutes, or until the hottest cell is within 1 K/min of steady state.

**Files.**
- `pack_log.csv`: `time_s, pack_current_a, inlet_c, T_0_0, ...`
- `pack.json`: `{"rows": 4, "cols": 6, "parallel": 6, "pitch_mm": 27, "ambient_c": 25, "cell": "Molicel INR-21700-P42A", "r_dc_ohm": 0.016, "duct_cfm": 40}`

**Command.** `kinematik-validate pack --config pack.json --log pack_log.csv --report pack_validation.md`

**Acceptance (fixed now).** For every thermocouple: at least 90 % of logged points inside the model's band at
0.8x to 1.2x cooling, and the peak rise within 15 % of the measured rise or 2 K, whichever is larger.

## B. PCB trace heating

**Hardware.** A test coupon with straight traces of several widths (e.g. 0.5, 1, 2, 4 mm) on 1 oz and, if
available, 2 oz copper, outer and inner layers, on 1.6 mm FR-4, in still air, mounted horizontally.

**Instrumentation.** A bench supply in constant-current mode; a thermocouple or an IR camera (emissivity set for
solder mask) at the middle of each trace; ambient temperature.

**Test.** For each trace, 3 to 5 currents up to the point where the rise reaches about 40 K. Wait for steady
state (under 0.2 K/min, typically 5-10 min) before reading.

**File.** `traces.csv`: `width_mm, copper_oz, external, current_a, ambient_c, measured_rise_c`

**Command.** `kinematik-validate trace --log traces.csv --report trace_validation.md`

**Acceptance (fixed now).** Every steady rise within 15 % of the measured rise or 2 K, whichever is larger.

## Reporting

Publish the logs, the config, the report and any miss in BENCHMARK.md, with the team or lab that ran the test
credited (with their consent). A pass validates the model for that hardware class only.
