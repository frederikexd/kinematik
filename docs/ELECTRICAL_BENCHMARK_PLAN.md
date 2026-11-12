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
error scales with cell size over track width, so the reference grid is set per net: **at least 24 cells across the
net's narrowest track**, and never coarser than 12.5 µm (6.25 µm for a 0.15 mm track). One net in five, picked by a
hash of its name, is also solved at twice the cell size; the difference is published as the reference's own
uncertainty. (First written as 6.25 µm for every net, with every net also at 12.5 µm; changed before any corrected
result was scored, because that took over five hours for the demo boards alone. The 106 nets already solved at
6.25 µm meet the per-net rule and are kept.)

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

## Addendum 5, trace heating against measurements (written before any point was read off the report)

**Reference.** NBS Report 4283, D. S. Hoynes, *Characterization of Metal-Insulator Laminates* (National Bureau of
Standards, 1 May 1956), the U.S. government measurements of etched copper conductors from which the MIL-STD-275 and
IPC-2221 current charts descend. Figures 1–7 plot measured current against conductor cross-section (square mils) for
temperature rises of 10, 20, 30, 45, 60, 75 and 100 °C, with each sample's points and four curves drawn through
them: plain etched, coated, dip-soldered, and stripped and tested in free air. The report gives the data only as
these graphs.

**What is read.** On each figure, the **plain etched** curve (the solid line) is read where it crosses each
vertical grid line of cross-section (16, 36, 64, 100, 144, 196, 256, 324, 400 sq mil), and where it is drawn. The
axes are calibrated from the printed grid lines of the scan. The reading is repeated at a second zoom and the two
readings averaged; their difference is published as the reading uncertainty.

**What is compared.** KinematiK's IPC-2221 ampacity for an external conductor
(`trace_ampacity_a`: I = 0.048 ΔT^0.44 A^0.725) at the same rise and cross-section, against the NBS current.

**What this can and can't show.** IPC-2221's formula is a fit to charts derived from these measurements, after a
10 % derating. So this checks how faithfully KinematiK's formula represents the measurements behind it; it is not an
independent test. An independent one needs IPC-2152's 2009 measurements, which are not public.

**Limits, fixed now.** KinematiK must be **conservative**: its current at most 5 % above the NBS current (the
allowance for reading error) at every point. Its typical margin is reported as the median ratio, expected near 0.9
from the derating, with no pass limit on it.

## Addendum 4, nets with copper pours (written before any pour net was solved)

Until now nets with pours were left out: KinematiK gives a pour no resistance (connectivity only), so its trace-only
number is an upper bound there, not an estimate. The brown-out check keeps that bound. This adds a **best estimate
with the pour** and benchmarks it.

**Reference.** The corrected 2-D solve (addendum 3) with the pour's copper added: KiCad's own filled zone polygons
(`ZONE.GetFilledPolysList`, after clearances and thermal reliefs, spokes included) drawn on the same grid as the
tracks and pads. Pours are wide, so the grid follows the narrowest feature of the net that carries current, tracks
and thermal spokes alike: at least 12 cells across it, never finer than 12.5 µm or coarser than 50 µm. Every net is
also solved at twice the cell size and the difference published.

**KinematiK.** A sheet solve of its own reading of the file (its own pad shapes, tracks, vias and the zones'
`filled_polygon`), on its own grid: at least 6 cells across the narrowest feature, between 25 and 100 µm. It shares
the physics with the reference (current through copper sheet), not the geometry, the grid or the code.

**Cases.** Every net with a pour on KiCad's demo boards whose filled copper fits the reference's memory (at most
4 million cells per layer), between the two pads KinematiK's estimate names worst. Larger planes are listed and left
out. **Design boards:** the demo boards. **Held out:** pour nets on the round-3 and round-4 team boards under the same
rule, solved once after KinematiK's pour estimate is frozen.

**Limit:** 2 % per net, as for tracks; 5 % and 10 % are reported too.

### Addendum 3, amendment (written before any team-board net was solved with the corrected reference)

The corrected reference takes far longer per net than the old one: about 100 nets an hour on the two cores
available. So the remaining solves use random samples, drawn now with a fixed seed (`random.Random(20261111)`), not
every net:

- **Demo boards (design):** the 725 nets already solved when this was written, about three quarters of 1,000. They
  were solved in board order, so whole boards are in or out; the result says which.
- **Round-3 team boards (re-score):** 150 nets drawn from the 412.
- **Round-4 team boards (held out):** 200 nets drawn from all nets KinematiK solves on them.

With 150–200 nets, the share within 2 % is known to about ±3–4 percentage points (95 %). Everything else is
unchanged.

### Round 4 outcome, and what replaces it (written before any team-board result against the corrected reference)

The round-4 repositories were cloned as registered. Two were dropped as not Formula Student / FSAE (moyamartin's
BMS is a university project, win2466/Hardware is the Speeduino ECU). Five iitmotorsports and three PUT-Motorsport
repositories hold no `.kicad_pcb`. Of the seven boards left, the two KiCad 7 opens are empty templates, and the other
five are KiCad 8–9 files, which KiCad 7 refuses (a newer format; relabelling the version only moves the error to the
next new token). Only KiCad 7 runs here, and the reference needs KiCad's own reading of the file, so **round 4 has
no boards** and is recorded as not run.

The held-out test is therefore the **round-3 team boards against the corrected reference**, scored once with
KinematiK frozen at commit a3805c1. Every model change since round 3 was designed on the demo boards; the team
boards' results against the corrected reference have not been looked at. They were seen once against the old
reference, and nothing was changed because of them.

### Addendum 4, amendment (before any held-out pour net was solved)

The 4-million-cell cap ran out of memory on this machine (StickHub's GND, killed at 4.4 GB). The cap is
**1.5 million cells per layer** for every pour net, demo and held out. KinematiK's pour estimate is frozen at
f947524, after design on the 6 demo pour nets under the cap.

## Addendum 6, pour estimate round 2 (written before any of its held-out nets was solved)

E missed on held-out boards because KinematiK's sheet solve was coarse (6 cells across the narrowest track, links
by the mean of two cells, pads snapped to cells). Round 2 replaces it with the discretization the corrected
reference uses (face-box links, area coverage, near-perfect-conductor pads, at least 12 cells across the narrowest
feature including thermal spokes), implemented again in KinematiK on KinematiK's own reading of the file. It shares
the physics *and the numerical method* with the reference now, not the geometry reading or the code, so this
checks KinematiK's reading of the board and its implementation, not the method.

**Pairs.** KinematiK names pads by designator and number; where two pads share both (a part with two pad 1s), it
now picks only pads whose name is unique, so the reference can tell them apart.

**Held out.** The pour nets on the round-3 team boards that E could not score (27 nets: shared pad names or no
reference), now solved once after KinematiK's round-2 estimate is frozen. The 17 nets E scored are design data from
here on. Same limits, same memory cap (1.5 million cells per layer).

### Addendum 6, amendment (written after 2 held-out nets were scored, before any other round-2 result)

Only 2 of the 27 held-out nets had two uniquely named pads (+0.01 % and +0.54 %), too few to show a rate. The
held-out set is widened to **every pour net the round-2 estimate has not been scored on**: the round-2 design used
only four demo nets (both ecc83 GNDs, StickHub +5V and +1V8). That adds the 17 team-board nets round 1 scored and
the other demo pour nets under the memory cap; round 2 was never run on them. Pads that share a designator and number
are told apart by position (name@x,y), in KinematiK and in the reference alike, which brings the LED-string nets in.
Each net is solved once. With n nets all within 2 %, the 95 % lower bound on the rate within 2 % is 0.05^(1/n).

## Addendum 7, KiCad 8/9 and native Altium boards (written before any kit came back)

**KiCad 8/9.** Four Formula Student team boards saved in KiCad 8–9 (round-4 repositories: PUT Motorsport BMS LV,
BMS HV master and Frontbox; WindingMotor's FSAE MoTeC distribution hub). The author runs `export_kicad9.py` in
KiCad 9, which writes KiCad's own reading of each board (counts, routed length per net, and the copper shapes of
every net: tracks, arcs, vias, pad polygons, filled zones). Scored:
- **Reading (limit A):** KinematiK's counts equal KiCad 9's exactly; per-net routed length within 0.1 %.
- **Net resistance (B5's method, held out):** 100 nets drawn with `random.Random(20261112)` from the nets KinematiK
  solves on these boards, solved once by the corrected reference from the export (`copper2d_from_export`), scored
  against KinematiK frozen at a3805c1. Limit 2 %.
- **Pours (addendum 6, held out):** every pour net under the memory cap, scored against the round-2 estimate
  frozen at dcc80ad (shared pad names by position, eef6107 only refuses large planes earlier).

**Native Altium.** Six boards designed in Altium by Queen's Formula SAE (`qfsae/pcb`: PDP_V3, BSPD_002,
Q23_BRAKE_LIGHT_O1, Analog to CAN Rev2, PMBoard, Triangular_Brakelight). The author reads Altium's own counts
(Board Information; PCB Filter `IsTrack And OnSignal`, `IsArc And OnSignal`). Limit: KinematiK's counts equal
Altium's exactly.

### Addendum 7, amendment (written 2026-11-11, before KiCad 9 read any of the four boards)

The author cannot install KiCad 9. The KiCad 8/9 reference therefore becomes **KiCad 9.0.3** (the Ubuntu 25.10
package, `kicad 9.0.3+dfsg-1`) installed in an isolated Ubuntu 25.10 root inside the benchmark workspace and driven
by the same unchanged `export_kicad9.py`. Nothing else changes: the four boards, the reading limits, the seed
20261112, the 100-net sample, the frozen KinematiK commits and the pour rule are as registered above. The
installed version string is recorded with the results. If KiCad 9.0.3 cannot load a board, that board is reported
as not read rather than replaced.

Native Altium came back first: the author's Altium Designer 26.11.1 counts are in
`benchmarks/electrical/altium/results_native.json`.

## Addendum 8, pour estimate round 3 (written 2026-11-11, before any KiCad 9 pour net was solved by the reference)

**Round 2's held-out result (widened set, addendum 6 amendment), scored once:** 43 nets (42 team, 1 demo), 15 within
2 %. Every miss is high (worst low −0.01 %). Looking at the misses found two flaws in KinematiK's *reading* of the
file, not in the solve:
1. **Custom pads.** KiCad `custom` pads carry their copper as primitives (`gr_poly`, `gr_rect`, `gr_circle`,
   `gr_line`, `gr_arc`) around a small anchor. KinematiK kept only the anchor (0.1 × 0.1 mm on the SUFST LEDs), so
   current had to squeeze into a dot: 25 of the 28 misses (all on SUFST boards).
2. **KiCad 5 pours.** Files older than KiCad 6 (`version` before 20200811) and zones marked
   `filled_areas_thickness yes` store the pour shrunk by half its minimum width; the copper is that polygon with its
   outline stroked at `min_thickness`. KinematiK read the bare polygon, 7–9 % too little copper on the
   michaelruppe boards: the other 3 misses.

**Round 3** reads both (the primitives turned and placed like the pad; the stroke drawn along every filled edge).
The solve is unchanged. Everything round 2 was scored on is design data from here on.

**Held out:** the pour nets of the four KiCad 8/9 boards of addendum 7, under the same memory cap, solved once by the
reference from KiCad 9.0.3's export. Each net is scored against **both** round 2 (frozen at eef6107, as addendum 7
registered) and round 3 (frozen at the commit that implements this addendum, named in the results). Limit 2 %; with
n nets all within 2 %, the 95 % lower bound on the rate is 0.05^(1/n).

**The export kit** (`export_kicad9.py`) failed on KiCad 9.0.3 before writing anything (vias now take a layer for
their width; a pad's `GetParent()` returns a container without `GetReference`). It was fixed for both, with no
change to what it writes, and the fixed kit is the one shipped.

### Addendum 8, amendment (written before any KiCad 9 pour net was solved by the reference)

On the design set, the reference itself does not converge on the ten hand-solder LED nets: refining the grid moves
it 0.21 → 0.40 → 0.48 → 0.55 mΩ (cells of 83, 42, 21, 10 µm) on `Net-(D1-K)`, the signature of current meeting
copper at a point. A 2 % comparison against a number that moves 15 % per halving means nothing, so on the held-out
set a net is **scored only where the reference at twice the cell size agrees with it within 2 %**; the others are
counted and published as "reference not converged", with both KinematiK numbers next to them.

## Addendum 9, closing the pour gaps (written before any of these solves)

**Unconverged reference nets** (the 10 hand-solder nets and the 24 KiCad 9 nets set aside by addendum 8's
amendment): each is solved again at half the cell size, and at a quarter where it fits under the memory cap. A net
counts as converged when its two finest solves agree within 2 %, and it is then scored against the finest. A net
whose value keeps growing by a similar step at every halving has no finite 2-D answer: current meets copper at a
point. It is reported as singular, not scored.

**The remaining 20 KiCad 9 pour nets** (stopped by size): every one under the 1.5-million-cell cap is solved by
KinematiK round 3 (frozen 97dc029) and by the reference, under the same rules. Nets over the cap are not estimated by
KinematiK either (it returns "too large" and the brown-out check keeps the trace-only upper bound). They are listed,
since there is no estimate to score.

### Addendum 9, first result (the 24 unconverged KiCad 9 nets, refined; the large nets are still running)

At half and a quarter of the cell size, 23 of the 24 references converge. Against them round 3 is within 2 % on 17
and **low on 6** (−2.2 % to −6.5 %). So over the 42 held-out nets with a converged reference, round 3 scores 36
within 2 % (86 %), and where it misses it reads low. The cause is plain from the reference's own steps: on these
nets the answer still rises 2–8 % when the cell is halved, and KinematiK solves at the coarse cell. On the design
set, 16 of the 26 refined nets (the LED strings) still do not converge at a quarter cell and stay unscored.

## Addendum 10, pour estimate round 4: a grid check inside KinematiK (written before any round-4 number)

Round 4 does what the reference had to do. `pour_resistance` solves at its cell size, then again between the same
two pads at half the cell, and at a quarter if those two differ by more than 2 %. It returns the finest value, the
last change in percent (`grid_change_pct`) and `converged` (the last change is within 2 %). When the finer grid does
not fit the memory cap it returns the coarse value with `converged` false. Nothing else changes.

**Scoring.** The 42 held-out nets above are design data for round 4: its reason for existing came from them. They
are re-scored and published as such, against each net's finest converged reference. The claim round 4 makes is
narrower than a rate: *a pour estimate KinematiK marks converged is within 2 % of the reference*, and one it does
not mark converged is labelled unverified in the result. **Held out for that claim:** the KiCad 9 pour nets still
running under addendum 9 when this was written (none of their references had been looked at), wherever round 4
marks its estimate converged and the reference converges too.

## Addendum 11, empty pours (written when round 4's design re-score showed two nets still 5 % low)

Round 4 converged on the two isolated `HV_+5V` nets of the PUT BMS master and still read 5 % below a converged
reference, so the grid was not the cause. The file holds a pour on each net with no `filled_polygon`: KiCad filled
it and kept nothing (an island that reaches no pad). KinematiK fell back to the drawn outline and counted it as
copper. From here on, a pour without a stored fill is not copper when any other pour in the file is filled; when
no pour in the file is filled, the board was saved unfilled, the outlines stand in and the report says so. Keepouts
and no-net zones are never copper. This also stops an empty pour from joining pads in the connectivity check.

Those two nets are design data and, with their only pour empty, no longer pour nets. No other KiCad 9 pour net has
an empty pour (checked from the file, not from results), so the nets still held out for addendum 10 are unaffected.

## Addendum 12, net resistance by the sheet solve (written before any of its numbers)

B6 left the 1-D net model at 70 of 95 within 2 % on the KiCad 8/9 boards, missing on short pad-to-pad nets by up to
−9.1 %, which is 0.01 % from using up the brown-out check's 1.10 margin. The pour estimate's sheet solve works on
any net, with or without a pour, and now checks its own grid. So:

- `pcb_pour.pour_resistance(board, nid, pair=...)` solves a named pad pair. `pcb_doctor.refined_net_resistance`
  takes the 1-D model's worst pair and returns the sheet-solve value between those pads when it converges and the
  net fits a size limit (`REFINE_MAX_CELLS` per layer, so the app stays responsive), else the 1-D value, and says
  which.
- **Brown-out check:** its bound becomes the larger of the 1-D estimate × 1.10 and the converged sheet-solve value
  × 1.02, over the copper finish. It can only get more conservative than it was.
- The bring-up table reports the refined value.

**Design check:** the 95 B6 nets (seen). **Held out:** 100 nets drawn with `random.Random(20261113)` from the 352
trace-only nets of the same four boards that B6 did not sample, each solved once by the reference from KiCad 9.0.3's
export between the 1-D model's worst pair, scored against the refined value frozen at the commit that implements
this. Limit 2 %. Both the nets the refinement converges on and the ones it falls back on are scored and reported
separately; with n converged nets all within 2 %, the 95 % lower bound on that rate is 0.05^(1/n).

## Addendum 13, the pour nets still open (written before any of these runs)

1. **Nets whose reference never converges** (the 26 design nets of addendum 9's second list, 16 of them unresolved
   at a quarter cell): the current estimate is run on all 26. The check is of the label, not the number: on the 16
   KinematiK must not mark its estimate converged, or if it does, the finding is published. On the 10 with a
   converged reference the usual 2 % applies. Design data.
2. **`/MASTER_FUSE_OUT` and `+24V`** (KinematiK converged, reference not): the reference is solved at KinematiK's
   final cell size and the net scored against it. Held out for round 4, as addendum 10 registered.
3. **The 19 nets of the first batch not re-scored with round 4**, and **the 4 large nets never reached**: run under
   a time limit per net (20 and 40 minutes on the benchmark machine). A net that does not finish is reported as
   not finished, with its size.
4. **Planes over the memory cap** get no estimate by design. For them the claim is only what the brown-out check
   already does: it uses the trace-only bound, which is higher than any estimate with the pour. That is checked on
   every pour net solved so far: the trace-only 1-D value must be at or above the pour estimate.

### Addendum 13, first result and a correction to the reference (written before the corrected reference was run)

On the 15 design nets whose reference never converged, KinematiK marked 11 of its estimates converged, at about
0.22 mΩ where the reference read 0.40, 0.48, 0.55 mΩ and climbing by the same step at every halving. A constant
step per halving is a logarithm: current entering plain copper at a single cell. The cause is in the reference, not
in the boards: it puts a pad's terminal at the **mean of its polygon's vertices**. For an ordinary pad that is its
centre. For the concave custom pads of these LED footprints the mean falls in a notch outside the pad, on the pour,
so the reference injected current into the pour at a point.

**Correction (reference method, addendum 3):** a pad's terminal is the cell at the pad's own position (KiCad's
`GetPosition`, the anchor of a custom pad), falling back to the vertex mean only if that cell is not the pad's
copper. Identical for every convex pad, so no earlier result changes; this is checked by re-solving nets that had
converged. The 15 nets are then solved again with the corrected reference under addendum 9's convergence rule and
KinematiK's unchanged numbers are scored against them.

## Addendum 14, proving the pour rate (written before any of these nets was solved)

**The claim tested** is what the app does: when `pcb_doctor.pour_net_resistance` (the sheet solve with the pour,
limited to 150,000 cells, worst pair among up to 64 pads) returns a value marked `converged`, that value is within
2 % of a converged reference between the same two pads.

- **Nets:** the 598 pour nets of the 111 round-3 team boards (KiCad 5–7) that no pour benchmark has used, shuffled
  with `random.Random(20261114)` and taken in that order. KinematiK frozen at 9a84971.
- **Reference:** the corrected 2-D solve with KiCad's own filled zones (addenda 3, 4, 13), read by KiCad 7, between
  KinematiK's pair, at KinematiK's final cell size and at twice it. Converged when the two agree within 2 %; if not,
  solved again at half the cell size and scored against the finest pair that agrees. A net whose reference does not
  converge by then is reported, not scored.
- **Stopping rule:** the first 80 nets with both a converged app value and a converged reference, or the end of
  the list. Nets where the app returns no value or an unconverged one are counted and reported with their error;
  the unconverged ones are checked against the brown-out check's ×1.10.
- **Pass:** all scored nets within 2 %. With 80 of 80 the 95 % lower bound on the rate is 0.05^(1/80) = 96.3 %. Any
  miss is published with its cause.

### Addendum 14, amendment (written before any reference value for these nets existed)

488 of the 598 nets are on boards saved by KiCad 8, which KiCad 7 cannot open. For those the reference reads the
board with KiCad 9.0.3 (the environment of addendum 7's amendment), with the same solver code and rules. The
KinematiK values were already computed by the frozen commit and are not recomputed.

## Addendum 15, closing addendum 14's caveat (written before any of these nets was solved)

Addendum 14's 80 nets came from 13 boards, most of them small signal pours. This test spreads across boards.

- **Boards:** the 42 boards with pours that no electrical benchmark has used: Formula Student boards from PUT
  Motorsport (steering wheel, rear box, SWPS, AMS master, dashboard, data acquisition) and Illinois Tech, the
  Speeduino engine-management boards (`win2466/Hardware`), and the UIUC rocketry boards (`ISSUIUC/ISS-PCB`), 637
  pour nets.
- **Nets:** at most 3 per board, in the order `random.Random(20261115)` shuffles each board's pour nets, skipping
  nets the app gives no value for, until the board has 3 scored or none left. KinematiK frozen at e807fff.
- **Reference and claim:** as addendum 14 (KiCad 9.0.3 reads every board).
- **Reported:** the rate with its bound; errors by board; and the scored nets split by pour size (area of the
  largest filled layer) and by whether the net is a power or ground rail by name, so the result shows whether big
  power pours are covered.
- **Pass:** every scored net within 2 %. The combined bound with addendum 14 is reported too.

### Addendum 15, result, and addendum 16 (written before any addendum-16 net was solved)

**Addendum 15 result**, scored once: 71 nets on 26 boards with both values converged, **70 within 2 %**. The miss:
`/PYRO_D` on UIUC's Mini-MK1, +3.07 % (high). Cause: KinematiK drew every roundrect pad with KiCad's default corner
radius (a quarter of the short side); this through-hole pad's file says 0.119, so KinematiK's terminal was 4 % too
small. Fixed in the 2-D sheet solve: it now reads `roundrect_rratio`. With the fix the net reads −0.03 %. The 1-D
model keeps the default radius, since benchmarks B5 and B6 were scored with it.

**Addendum 16, held out for the fix:** on the same 42 boards, the next up to 3 pour nets of each board in the same
registered order (`random.Random(20261115)`), after those addendum 15 examined, under the same rules and reference.
KinematiK frozen at the commit that adds this text. Pass: every scored net within 2 %.
