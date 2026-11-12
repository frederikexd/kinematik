# Altium benchmark kit

Altium Designer can't run where the rest of the electrical benchmark runs (it is Windows-only and licensed), so
this part is run by hand in Altium and scored here. The cases and limits were fixed before anyone ran them
(docs/ELECTRICAL_BENCHMARK_PLAN.md, addendum 2). Send back what you get, whatever it shows; it is published either
way.

## Part 1. Does KinematiK read an Altium board the way Altium does? (about 30 minutes)

Use three of KiCad's demo boards: `pic_programmer` (2 layers), `StickHub` (arcs) and `video` (large, many vias).
They ship with KiCad (`demos/` in the KiCad install) and are in the zip that came with this kit.

For each board:

1. **Import.** File → Import Wizard → KiCad Design Files → select the `.kicad_pcb`. Accept the defaults.
2. **Count.** In the PCB editor: Reports → Board Information (on newer versions, the Properties panel with nothing
   selected, Board Information section). Copy the numbers of **Tracks, Arcs, Vias, Pads and Components** into
   `boards_template.csv`, and your Altium version.
3. **Save twice.** File → Save As:
   - once as the normal binary `.PcbDoc`, named `<board>_binary.PcbDoc`;
   - once as **PCB ASCII File (*.PcbDoc)**, named `<board>_ascii.PcbDoc`.
4. Send the filled `boards_template.csv` and the six `.PcbDoc` files.

Scoring: KinematiK's binary and ASCII Altium readers must give the same counts as Altium's Board Information,
exactly (limit A of the plan), and the same per-net routed lengths as KiCad's original within 0.1 %.

## Part 2. Controlled impedance from Altium's field solver (about 45 minutes)

Altium's Layer Stack Manager computes impedance with a 2-D field solver (Simbeor). These are the same 21 cross-
sections the open field solver (atlc) solved, round 1 and round 2.

1. New PCB → Design → Layer Stack Manager.
2. Build a 2-layer stack: top copper 0.035 mm, one dielectric of the case's **h_mm** and **er**, bottom copper.
   **Remove both solder mask layers** (they change the impedance and the reference has none). Leave the dielectric
   loss tangent at its default; it does not affect the impedance at these frequencies.
3. Impedance tab → add a profile:
   - for `single` rows, a single-ended microstrip on the top layer, referenced to the bottom layer;
   - for `differential` rows, a differential microstrip, with the row's trace width **w_mm** and gap **s_mm**.
4. Enter the width (and gap). Copy Altium's impedance into `altium_impedance_ohm`. If your version only lets you
   set a target impedance and computes the width, set a target near the expected value instead. Record the
   target in `altium_impedance_ohm` and the width Altium gives in the last column; the scoring handles both.
5. Repeat for every row of `impedance_template.csv` (rows with the same h and εr share one stack) and send the file
   back.

Scoring: KinematiK against Altium with the plan's limits, single-ended within 5 % and differential within 10 %.
Altium is also compared with atlc. Two independent field solvers agreeing checks the reference itself.

## Scoring

`python fsae_suspension/scripts/electrical_reference.py altium --dir <folder with the returned files>` writes
`benchmarks/electrical/altium_results.json` and prints the comparison.
