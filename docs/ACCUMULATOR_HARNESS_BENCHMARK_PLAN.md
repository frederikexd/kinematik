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
