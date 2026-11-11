# Electrical benchmark plan

Written and committed before any comparison was run. The limits below are fixed here, not after.

## Why these three references

The request was KiCad, Altium and SPICE, or a more common tool where one exists.

| Asked for | Used | Why |
|---|---|---|
| KiCad | **KiCad 7.0.11**, through its own Python API (`pcbnew`) | The board file format KinematiK's PCB Doctor reads. KiCad's own reader is the ground truth for what is on a board. |
| SPICE | **ngspice 42** | The open SPICE3 descendant. LTspice, the most common SPICE in Formula Student, solves the same circuit equations; ngspice runs headless here, LTspice does not. |
| Altium | **atlc 4.6.1**, a 2-D finite-difference field solver | Altium can't be run here: it is Windows-only and licensed. What Altium computes that KinematiK also computes is controlled impedance, which Altium's Layer Stack Manager gets from a field solver. A field solver is therefore the reference, not Altium's closed forms. atlc is an established open one (used in amateur-radio and academic work since 1996). Altium's PCB reader is not benchmarked; that is stated as a gap. |

## A. KiCad: does KinematiK read a board the way KiCad does?

Boards: every `.kicad_pcb` that ships with KiCad 7's demos (`/usr/share/kicad/demos`, package `kicad-demos`), 14
boards, none chosen or excluded by us. A board KinematiK fails to read counts as a miss.

Per board, from `pcbnew.LoadBoard` against `pcb_doctor.parse_board`:

| Quantity | Limit |
|---|---|
| Straight track segments | exact count |
| Arc tracks | exact count |
| Vias, and the multiset of via drill diameters | exact |
| Pads | exact count |
| Footprints | exact count |
| Total routed length per net (straight + arc) | within 0.1 % or 0.01 mm, whichever is larger |
| Set of track widths per net | exact to 0.001 mm |

## B. SPICE (ngspice): the circuits KinematiK solves

**B1. Precharge** (`tractive_system.simulate_precharge`). Four circuits: 600 V / 2 kΩ / 500 µF, 400 V / 1 kΩ /
1.2 mF, 300 V / 5 kΩ / 200 µF, and the first again with the resistor shorted at 2.5 s. Compared at 50 time points
and on: time to 90 % and 95 % of pack voltage, peak current, energy in the resistor (to 10 τ).
Limit: **0.1 %** on every quantity (a linear RC has an exact solution; any larger difference is an error).

**B2. Pack along a lap** (`ev_electrical_check.pack_along_lap`). A 600 V-class pack: open-circuit voltage falling
with state of charge (four-point table), 0.25 Ω, 14 Ah. It is driven by the terminal power of a simulated lap of
KinematiK's representative autocross, regen included, sampled at 0.05 s. SPICE models the same pack as a
behavioral source with the same table and integrates the charge itself.
Limits: terminal voltage within **0.5 %** at every point, current within **1 %** at every point, final state of
charge within **0.1 percentage point**. KinematiK steps the charge forward explicitly; SPICE integrates with error
control, so this tests the stepping as well as the model.

**B3. Net resistance on real boards** (`pcb_doctor.analyze_net`, worst pad-to-pad resistance by nodal analysis).
On every net of the demo boards that KinematiK solves (no pours, at most its node limit), SPICE gets an
independent netlist built from KiCad's own geometry: one resistor per track (ρ L / (w t), same copper thickness
and temperature as KinematiK), one per via barrel, with connections found by KiCad's own hit tests (a track end
inside a pad, via or another track's copper). 1 A is injected between the pad pair KinematiK names worst, and
the resistance read back.
Limit: within **2 %** per net. This tests KinematiK's own connectivity rules (endpoint welding, via annulus, pad
reach) against KiCad's, as well as the solve.

## C. Field solver: controlled impedance (`electronics.DiffPair`)

KinematiK uses the IPC-2141 microstrip closed form and an exponential edge-coupled correction. atlc solves the
same cross-sections on a fine grid.

- Single-ended microstrip, FR-4 εr 4.3, 35 µm copper, h = 0.2 mm, w/h ∈ {0.2, 0.5, 1.0, 1.5, 2.0}.
- Edge-coupled differential, w/h ∈ {0.5, 1.0}, s/h ∈ {0.5, 1.0, 2.0}.
- Range: KinematiK's documented validity, 0.1 < w/h < 2.0.

**Checking the checker first:** atlc solves a zero-thickness centred stripline in air and in εr 4.3, whose
impedance is exact (Cohn, elliptic integrals). atlc must be within 1 % there, or its results are not used.

Limits, set from the known accuracy of the closed forms in their documented range: single-ended within **5 %**,
differential within **10 %**. If KinematiK misses, the miss is published as it is, and any fix is checked
against cases not used to make it.

## What is recorded

Every reference number goes into `fsae_suspension/benchmarks/electrical/` with the tool versions. The tests in
`tests/test_electrical_benchmark.py` re-check KinematiK against those recorded numbers on every run, so CI does
not need KiCad, ngspice or atlc installed. `scripts/electrical_reference.py` regenerates the references wherever
the tools are installed.
