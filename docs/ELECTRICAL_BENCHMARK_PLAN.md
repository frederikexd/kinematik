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

## Addendum, before round 2 (written after round 1, before any round-2 run)

Round 1 missed two limits: single-ended impedance at w/h 0.2 and 2.0, and net resistance on the demo boards.
KinematiK was changed: Hammerstad–Jensen replaces IPC-2141 for microstrip Z0, and four connectivity flaws in
`pcb_doctor.analyze_net` were fixed. Those changes were made looking at round-1 cases, so round 1 can't judge
them. Round 2 uses cases neither round has seen, with the same limits (5 % single-ended, 10 % differential):

- εr 3.66 (Rogers RO4350B) and εr 4.6 (high-Tg FR-4), h = 0.15 mm, 35 µm copper,
  w/h ∈ {0.3, 0.8, 1.3, 1.8}: 8 single-ended cases.
- εr 4.6, h = 0.15 mm, (w/h, s/h) ∈ {(0.75, 0.75), (1.25, 1.5)}: 2 differential cases.

There is no second, unseen set of KiCad boards in this round. The fixed net-resistance results are reported on the
same demo boards and labelled as not independent.

## Addendum 2, before round 3 (written before any of these boards was downloaded)

Round 2 left three things open: the net-resistance fixes were checked only on the boards that exposed the
flaws, the reference for net resistance was itself a 1-D model, and Altium was not run.

**Round 3 boards, chosen by rule, not by result.** The Formula Student and FSAE team repositories on GitHub
returned by two web searches ("github formula student kicad pcb repository BMS OR VCU OR shutdown circuit
kicad_pcb" and "github FSAE electrical kicad hardware repository team PCBs"), in order of first appearance:
UCDFS/Circuits, aditeyadc/bspd, sufst/pcb, qfsae/pcb, michaelruppe/FSAE, LSUTigerRacing/Electronics. Every
`.kicad_pcb` in them that KiCad 7 loads is used; files KiCad 7 can't load (newer formats) are listed and left out.

**Round 3 reference: a 2-D copper solve.** For each net, KiCad's own copper outlines (tracks, arcs, pads, via
rings, via and pad barrels, from `TransformShapeToPolygon`) are drawn on a grid per copper layer. The steady current
flow through that sheet (conductance σ t per square, the same ρ and t as before) is solved as a sparse linear
system. Vias and plated holes join the layers through their barrel resistance. Each pad is one terminal (its whole
copper area at one voltage, as a soldered lead makes it). 1 A goes in at one pad of KinematiK's worst pair and
out at the other. The grid is 25 µm. A sample of nets is re-solved at 12.5 µm, and the grid error must be under
1 %, or the grid is refined. Zones (pours) stay excluded, as before.

Limits are unchanged: net resistance within 2 % per net. Both KinematiK and the round-2 1-D SPICE netlist are
scored against the 2-D solve, on the demo boards and on the round-3 boards.

**Altium.** Altium can't run here. A kit (`benchmarks/electrical/altium/`) gives the author the exact steps to run
in Altium Designer: Board Information counts, net lengths and Layer Stack Manager impedances for the same
geometries as round 1 and 2. The limits are those of A and C. The returned numbers are scored by
`electrical_reference.py altium`, and the result is published whatever it shows.

### Round 3 order of work (written before any 2-D result existed)

1. The 2-D reference is computed for the demo boards first, at 12.5 µm (25 µm differed from 12.5 µm by up to 1.5 % on
   a 10-net sample, so the plan's 1 % rule moved the grid to 12.5 µm).
2. KinematiK as it stands is scored on the demo boards (round 3a).
3. Any change to KinematiK's net-resistance model is designed and checked on the demo boards only.
4. Only then is the 2-D reference computed for the 20 team boards, once. Both the round-3a KinematiK and any
   changed version are scored on them (round 3b), and both results are published.

## Addendum 3, before round 4 (written before any round-4 board was downloaded)

**The round-3 reference was biased, and is corrected.** Looking into the nets KinematiK still missed on, StickHub's
thin (0.15 mm) diagonal tracks read lower than the 2-D solve. Refining the grid moved the 2-D answer toward
KinematiK (12.5 µm → 3.1 µm: 2.58 → 2.44 mΩ, first-order), so the reference, not KinematiK, was off. Two causes:
a link between cells conducted by the *smaller* of the two cells' coverage, which narrows every edge that crosses the
grid at an angle; and a pad was the set of whole cells at least half covered, so its edge snapped to the grid. The
corrected solve conducts each link by the copper fraction of the cell-sized box centred on the face between the
cells (area fractions from 4 × 4 samples per cell), and treats pads and via rings as near-perfect conductors
(10⁴ × copper) over the part of each box they cover, so their edges are anti-aliased like a track's.

Checked before use, on copper with a known answer: two 1 mm pads joined by a 0.15 mm track, 2 mm between the pads,
turned to 0°, 15°, 30° and 45°. The copper is the same at every angle, so the answer must be too. At 6.25 µm the four
angles agree within 0.9 % and converge on the 1-D value (track outside the pads) as the grid is refined. The
reference grid is therefore **6.25 µm**, and every net is also solved at 12.5 µm; the difference is published per
net as the reference's own uncertainty.

Round 3 is re-scored against the corrected reference, both KinematiK versions, and the round-3 result as first
published stays in the record next to it.

**Round 4 boards, chosen by rule before download.** Two new web searches ("github formula student team kicad
hardware repository accumulator BMS PCB kicad_pcb" and "github FSAE racing team kicad PCB designs repository
electronics"), Formula Student / FSAE team repositories not used in round 3, in order of first appearance:
kthfspe (KTH Formula Student: KICAD_infrastructure, kicad_infrastructure.new), PUT-Motorsport (the repositories on
its GitHub page whose names name hardware: PUTM_EV_BMS_LV, PUTM_EV_DASHBOARD, PUTM_EV_BMS,
PUTM_EV_Data_Acquisition_Cards, PUTM_EV_AMS_MASTER, PUTM_EV_FRONTBOX), moyamartin/bms_hardware, iitmotorsports
(pdm, pcm, dashboard, Dashboard-2022, IIT-SAE-ECU), win2466/Hardware, WindingMotor/M130-Distribution-HUB. A
repository that turns out not to belong to a Formula Student or FSAE team is dropped, with the reason. Every
`.kicad_pcb` in them that KiCad 7 loads is used.

### Round 4 order of work

1. The corrected reference is computed for the demo boards and the round-3 boards.
2. Any change to KinematiK is designed on those boards only, then frozen (commit recorded).
3. Only then are the round-4 boards downloaded and solved, once. The frozen KinematiK is scored on them and the
   result is published whatever it shows. The limit is unchanged: 2 % per net.
