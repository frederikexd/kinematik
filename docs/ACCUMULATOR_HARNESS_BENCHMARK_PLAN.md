# Accumulator and harness benchmark plan

Registered 2026-11-13, before any reference was generated. The limits below are fixed here and are not changed after
the data; a check that misses its limit is published as a miss.

Three reference tools, none of which shares code with KinematiK:

| Tool | Version | Used for |
|---|---|---|
| ngspice | 42 | harness wire resistance and voltage drop with copper's temperature coefficient |
| PyBaMM | 26.10 | pack terminal voltage, current and state of charge along a lap; lumped pack temperature with cooling |
| liionpack | 0.3.7 | a full series-parallel pack network (modified nodal analysis), cell by cell, with interconnect resistance |

## H1. Harness wire voltage drop (ngspice)

`suspension.harness.WireRun.voltage_drop_v` on straight runs with no service loop, bends or strip allowance:
22, 18, 14, 10 and 6 AWG; 0.5, 2 and 5 m; 20, 60 and 105 °C. The reference builds each run in ngspice as a resistor
whose 20 °C value comes from the AWG definition (d = 0.127 mm × 92^((36 − n)/39)) and annealed copper at 100 % IACS
(1.7241e-8 Ω·m), with tc1 = 0.00393 /K, simulated at the run's temperature with a DC current source.

Limit: every drop within 0.1 %.

## A1. Pack along a lap (PyBaMM)

`suspension.ev_electrical_check.pack_along_lap` on the lap already used against ngspice
(`benchmarks/electrical/spice_pack_lap.json`: 520 points, 0.05 s, regen included). The reference is PyBaMM's
Thevenin equivalent-circuit model with no RC pairs, power-controlled, with the same open-circuit-voltage table,
capacity and series resistance.

Limits: terminal voltage within 0.5 % everywhere; current within 1 % (or 1 A near zero); final state of charge
within 0.001.

## A2. Pack temperature with cooling (PyBaMM)

`suspension.ev_electrical_check.pack_temperature_along` on a 30-minute, 1 s current profile (repeating
acceleration, cruise and regen blocks), heat capacity 40 kJ/K, cooling 10 W/K to 298.15 K ambient, 0.1 Ω. Reference:
PyBaMM's lumped thermal model for the same equivalent circuit (cell thermal mass = heat capacity, cell-to-jig
coefficient = cooling, jig held at ambient by a very large thermal mass, entropic heat off).

Limit: temperature within 0.2 K at every point.

## A3. Series-parallel pack network (liionpack)

The lap check takes each cell's current as the pack current divided by the number of parallel strings, and the pack
resistance as n_series × r_cell / n_parallel. The reference solves the full network of 140 series × 3 parallel cells
(4.0 V, 12.8 mΩ each) at 100 A with liionpack's modified nodal analysis.

- Ideal interconnects (1 nΩ): every cell current within 0.1 % of I / n_parallel; terminal voltage within 0.1 % of
  n_series × V_cell − I × n_series × r_cell / n_parallel.
- Realistic interconnects (0.2 mΩ between cells in a string, 0.1 mΩ busbar between strings, terminals at one end):
  every cell current within 5 % of I / n_parallel; terminal voltage within 1 % of the lumped value, which leaves the
  interconnects out.

A miss here is a limit of the lumped model, published as such, not tuned away.

## Addendum 1 (2026-11-13, registered before its reference was generated)

### A4. Per-cell pack thermal network (ngspice)

`suspension.pack_thermal.PackThermalModel.simulate` on a 4 x 6 grid with two fans, 20 minutes of 1 s mixed
current, temperature coefficient of resistance and wake warming off so the network is linear. The per-cell
convection coefficients come from KinematiK's airflow proxy and are recorded as inputs: what is checked is the
network solve (Joule heat, cell-to-cell conduction, convection to air), not the airflow proxy. Reference: the same
network in ngspice by the thermal-electrical analogy (temperature as voltage, heat as current, capacitance as
thermal mass, conductances as resistors), transient with a fine maximum step.

Limit: every cell within 0.2 K at every second.

The airflow proxy itself is not benchmarked here: that needs CFD or a measured pack, and stays flagged
"synthesized" until calibrated.

## Addendum 2 (2026-11-13, registered before any airflow reference was generated)

### B1. The airflow proxy against tube-bank correlations (ht)

Case: a ducted, in-line bank of 21700 cells (D = 21 mm, 70 mm long), 27 mm pitch both ways, 6 rows along the flow and
4 across, air at 30 °C, a fan at the middle of the inlet face pushing 20, 40 and 80 cfm through the 108 x 70 mm duct.
KinematiK: `AirflowParams().h_field` with that fan. Reference: the three tube-bank correlations in the open-source
`ht` library (Zukauskas-Bejan, Grimison, ESDU 73031), air properties from CoolProp, the row-by-row coefficient taken
from the bank averages (row k = k x mean of k rows - (k - 1) x mean of k - 1 rows).

Limits: every cell within 30 % of the three correlations' mean for its row; the ratio of mean h at 80 cfm to 20 cfm
within 15 % of the correlations'.

### B2. A correlation-based duct mode, if B1 misses

If the proxy misses, a duct mode computes each row's h from the Zukauskas in-line and staggered correlations
(Incropera's tables, with the row correction) from the duct flow. Limits: within 2 % of `ht`'s Zukauskas-Bejan
(the same correlation: an implementation check), and within the spread of the three correlations at every row and
flow (agreement with the other two is not a test of the physics, which the correlations' own scatter, about
±15-25 % against the measurements they were fitted to, bounds).

Neither B1 nor B2 replaces a measured pack: absolute temperatures stay flagged "synthesized" until calibrated.

## Addendum 3 (2026-11-13)

### C1. Still-air cooling (ht, natural convection)

Every pack in the app runs with no fan placed, so every cell is cooled by the still-air coefficient `h_floor`
(8 W/m²K before this addendum). Reference: `ht`'s natural-convection correlations for a 21 mm can in 30 °C air at a
surface rise of 5, 10, 20 and 40 K (Churchill-Chu for a horizontal cylinder; ht's default method for a 70 mm
vertical one), air properties from CoolProp at the film temperature.

Disclosure: the horizontal values were computed once (4.5 to 7.3 W/m²K) before this limit was written, so C1 is not
blind. Limit: `h_floor` within 30 % of both correlations at every rise. 8 W/m²K misses at 5 and 10 K (up to 78 %
high, i.e. optimistic cooling); the default moves to the correlation's value at a 10 K rise, rounded, and is
re-scored.

### C2. How far to trust an absolute temperature

The correlations scatter by about ±15-25 % against the measurements behind them, and none of this replaces a
measured pack. The app therefore reports the hottest cell with the cooling coefficient at 0.8x and 1.2x as a band,
and labels the fan-draught proxy as unbenchmarked wherever fans are placed.
