<!--
  KinematiK — Formula SAE / Formula EV full-car pre-validation platform
  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
  Open source. Original author: Frederik Thio, creator of KinematiK.
-->

# PCB Doctor — Altium as well as KiCad, one diagnosis

Drop these nine files over your tree. One is new
(`suspension/pcb_altium.py`); the rest are edits.

| File | Change |
|---|---|
| `suspension/pcb_altium.py` | **new** — Altium / Protel ASCII reader |
| `suspension/pcb_doctor.py` | `parse_board()` dispatcher, unit-aware patching, routed arcs |
| `streamlit_app.py` (root) | dual-format upload, binary refusal, second demo board |
| `suspension/express.py` | `.PcbDoc` ingest, `extras["board_file"]` |
| `suspension/express_jobs.py` | PCB job runs on either format |
| `suspension/__init__.py` | new submodule registered |
| `tests/test_pcb_doctor.py` | 16 new tests |
| `tests/test_express.py` | 2 new ingest tests, renamed extras key |
| `PCB_DOCTOR_USAGE.md` | the export path, the caveats |

## Why this was a reader and not a second Doctor

Half the grid routes in Altium — the student licence is free — and none of the
failure modes the Doctor exists for care which tool drew the copper. A via
still chokes a wide trace, a cap still cooks next to hot copper, and a board
still browns out under stall current.

So the change is a **front-end**, not a fork. Both readers produce the same
`PcbBoard`, and every check, the nodal IR-drop solve, the viewer, the fix
engine and the hand-off report were already written against that model. Adding
Altium added roughly 400 lines of parsing and zero lines of physics.

The proof is a test, not a claim: `demo_altium_pcb()` is the *same ECU
geometry* as `demo_kicad_pcb()` — other format, other units, other Y axis — and
`test_same_board_two_formats_one_diagnosis` asserts the two produce identical
findings and identical counts. If the front-ends ever drift, that breaks.

## Three things that would have been quietly wrong

**Units.** Altium ASCII writes lengths with the unit attached
(`WIDTH=11.811mil`). Writing a corrected width back as bare millimetres would
have been reinterpreted as mils on reopen — a 25× error, on a file heading for
a fab. Width tokens now carry their own scale and suffix, and the patch goes
back in the file's own units. `test_patch_writes_back_in_the_files_own_units`
holds that line.

**Arcs.** Altium routes arcs constantly, and an ignored arc is not a small
length error — it is a *hole in the connectivity graph*. The open-copper check
would then report a perfectly good net as dead on arrival, which is exactly the
false alarm that teaches a team to stop reading the findings. Both readers now
chord arcs (this was a latent KiCad bug too — `(arc …)` has been routed copper
since v6). Chords cut the corner, so length reads ≤0.15% short at 10° steps;
that is stated rather than hidden.

**Shared width tokens.** A chorded arc is many segments behind one `WIDTH=`
token. Two fixes on the same span would splice the file twice at overlapping
offsets and corrupt it. `apply_fixes()` now collapses edits per span, keeping
the widest requirement.

## Native binary: read, never written

A native binary `.PcbDoc` — what Altium saves by default, and therefore the file
a member actually has — is detected by OLE magic **before any decode** (decoding
first would turn it into mojibake and produce a baffling parse error) and read
by `suspension/pcb_altium_binary.py`, written against KiCad's own Altium
importer rather than a guess.

It is read and never written. A binary board carries `patchable = False` and
`apply_fixes()` refuses it outright, so the one-click re-trace still needs the
ASCII export. The asymmetry is the whole design: a bad *read* announces itself
as absurd geometry, and `_sanity_check` refuses the file rather than reporting
on it; a bad *write* silently corrupts a board a team is about to pay to
fabricate.

Reading it needs `olefile` (added to `requirements.txt`). Without it the member
gets the ASCII export instructions, not a stack trace.

## What every Altium import admits to

Two assumptions are unavoidable and both are printed, in the UI expander, in
the fix report, and in the express artefact:

* which unit bare numbers are in (decided from what the file actually wrote,
  not assumed);
* that net references resolve **0-based** against `|RECORD=Net|` file order.

The second matters most: an off-by-one means a confident diagnosis of the wrong
nets. The note tells the member to glance at a familiar net name in the current
table — a mis-resolve is visible in one look, never silent.

## Undeclared currents, and why setting one is declaring it

A net with no declared peak current now reports **MISSING** and gets no
auto-fix, rather than being assigned a default 1 A and failed against that
default. On a real board most nets are unmatched, so the old behaviour buried
the two or three real findings under twenty invented ones.

That rule creates a trap, so the type closes it: `NetAssignment` clears the
`assumed` flag on any write to `current_a`, through item assignment, `update()`
or `declare_net_current()` alike. Otherwise a caller could set the number, leave
the flag standing, and get a net that reports "current not declared" while
visibly holding the current they just supplied. Only the auto-assigner's own
`assume()` sets a current without declaring it. A plain dict built elsewhere
counts as declared, because the absence of the flag means someone stated the
number.

## Breaking change

`DataBundle.extras["kicad_pcb"]` → `extras["board_file"]`, and the `pcb_check`
job's `needs_extra` follows. The old key holding an Altium board would have
been a lie in the data model. Both tests that named the key are updated.

## Not touched

`suspension/streamlit_app.py`, `streamlit_app (1).py` and
`suspension/test_pcb_doctor.py` still carry the KiCad-only code. All three are
listed in `REMOVED_FILES.txt` as duplicates and the canonical copies are
patched — but if any of them is still live in your deployment, it needs the
same edits.

## Known limits, stated rather than discovered later

* **Copper pours are not meshed** in any format. A pour registers its net (so
  the net is never falsely called open) but contributes no geometry, which
  makes its resistance conservative rather than wrong.
* **Copper-open false alarms run 4.4% of routed nets** across the 15-board
  corpus, down from 8.0%. 49 of the 67 sit on one 51k-segment binary board
  whose Regions6 pours the binary reader does not yet read; excluding it the
  rate is 2.1%. Wrong direction for noise, right direction for safety: false
  *alarms*, never false all-clears.
* **The Streamlit panel now has coverage, but only of the element tree.**
  `tests/test_ui_pcb_doctor.py` runs the real app headless and asserts each
  board format renders its caption, diagnosis and re-trace without raising.
  It cannot see pixels, so layout and legibility are still eyeballs-only, and
  it cannot drive a second rerun on this app — so click *handlers* are verified
  by construction, everything downstream of them by the test.
* **Large boards degrade the viewer, not the diagnosis.** Above 6000 copper
  segments on the selected layers the inline SVG is suppressed with a note —
  the largest real board produces 6.6 MB of markup, which most browser tabs
  will not survive. Findings and the fix report still cover the whole board.

## Fixed after the first review pass

* **Upload ceiling 24 MB → 64 MB.** A native binary `.PcbDoc` is ~10x the size
  of the same board as text; the old limit rejected a real 27 MB board this
  tool parses in 3.6 s, under a comment claiming no real board was that big.
* **`diagnose()` and `board_svg()` are now cached** on a fingerprint of the
  board, the assigned currents and the knobs. Streamlit reruns the whole script
  on every widget interaction, so on the largest board this was ~10 s of dead
  UI per click; it is now 0.7 ms. The viewer additionally refuses to draw above
  6000 selected-layer segments — the largest board emits 6.6 MB of inline SVG,
  which most browser tabs will not survive. Findings are unaffected.
* **An unreadable file no longer hides the Trace Prescriber.** The error path
  did a bare `return`, removing the one tool that needs no file at all at the
  exact moment the member is holding a file that would not open.
* **Docstrings swept.** Three places still described the binary format as
  "refused on purpose", which stopped being true when the reader landed.
* **Coincident endpoints are welded** (`NODE_WELD_MM = 0.05`). Routers emit
  coordinates that disagree in the last micron and fixed-grid rounding then
  splits a joint across two buckets — measured 11 um apart on a real 4-layer
  board, turning one routed net into three islands. Welding is deliberately
  capped at 50 um: traces are 100-500 um wide so endpoints that close overlap
  in copper, while a genuine unrouted gap is millimetres. A test pins the
  constant so nobody widens it to silence an alarm.
* **Altium Region/Fill shapes now count as pours when they carry a net.**
  They were excluded wholesale as a workaround for the net-indexing bug —
  teardrops with `NET=0` were resolving to net index 0 and inventing a pour.
  With references resolved against each Net record's own ID that cause is gone,
  so only genuinely netted regions register; keepouts, board cutouts and
  teardrops stay excluded by name. Pour *geometry* is still not meshed, so
  resistance remains trace-only and conservative.

## Copper pour meshing — connectivity done, containment deliberately not trusted

Pours now carry **outlines** in all three readers (`PcbZone`): KiCad
`(filled_polygon (pts …))` with a fallback to the drawn `polygon`, and Altium
`VX*/VY*` vertex lists in both the ASCII and binary paths. 298 outlines parse
across the 15-board corpus.

Two safety properties shape how they are used, and both are asserted by tests.

**Pours join, they never conduct.** A pour is real copper and really does join
pads, so it contributes edges to the connectivity graph. It is kept out of the
resistance solve entirely: any sheet resistance invented for it would make the
IR drop look *smaller* than trace-only does, and small is the direction that
under-reports brown-out. `conn_edges` decides whether a net is open; `edges`
stays the trace-only mesh, so every reported resistance is still an upper bound.

**Containment can clear a false open, never create one.** It is tempting to let
geometry replace the blanket "net has a pour ⇒ never open" exemption. Measured,
that made things worse — 4.4% → 5.1% false alarms — and the reason is thermal
relief: KiCad's `filled_polygon` has a clearance hole punched around every pad,
joined by spokes, so a perfectly connected pad sits geometrically *outside* the
fill. Altium's polygon outline has the mirror problem, covering copper that may
never have been poured. So containment is used only to connect, the blanket
exemption stays, and both halves fail safe. Reading spoke geometry is what would
let the exemption go.

Net effect on the corpus: no change to the 4.4% figure. The remaining opens are
not pour-related — none of the 49 on the largest binary board is on a net that
has a pour at all.

## Via annulus attachment — the last big false-alarm source

A track does not have to hit a via's exact centre to be connected to it. A via
is an annulus of copper, and routers habitually end a track a little short:
measured on a real 4-layer board, an inner-layer track stopped **97 um from a
350 um via** — landing squarely on the via pad, but outside the 50 um endpoint
weld. The net split across layers and reported "copper open" while being
perfectly routed.

That single cause was behind **49 of the 67** remaining false alarms.

The fix uses the same physical rule the pads already use: anything within the
via's own copper radius overlaps it. That radius is a fact read from the file,
not a tolerance to tune, which is what makes it safe — a track ending a
via-radius away is *touching* the via, so this cannot quietly weld a real gap.
Tests pin both directions, including that the reach scales with via size rather
than being a constant.

| | before | after |
|---|---|---|
| artiq-kasli | 49 | **7** |
| artiq-hvsup-isol | 4 | 2 |
| condenar-mainboard | 5 | 4 |
| condenar-logger | 2 | 0 |
| demo_video | 3 | 2 |
| **corpus** | 4.4% | **1.3%** (19 / 1509 routed nets) |

No board regressed. The residue is now spread thin — no board carries more than
seven — which is the shape you want: no single systematic cause left, just a
tail.

## Fixed after the first review pass

* **Upload ceiling 24 MB → 64 MB.** A native binary `.PcbDoc` is ~10x the size
  of the same board as text; the old limit rejected a real 27 MB board this
  tool parses in 3.6 s, under a comment claiming no real board was that big.
* **`diagnose()` and `board_svg()` are now cached** on a fingerprint of the
  board, the assigned currents and the knobs. Streamlit reruns the whole script
  on every widget interaction, so on the largest board this was ~10 s of dead
  UI per click; it is now 0.7 ms. The viewer additionally refuses to draw above
  6000 selected-layer segments — the largest board emits 6.6 MB of inline SVG,
  which most browser tabs will not survive. Findings are unaffected.
* **An unreadable file no longer hides the Trace Prescriber.** The error path
  did a bare `return`, removing the one tool that needs no file at all at the
  exact moment the member is holding a file that would not open.
* **Docstrings swept.** Three places still described the binary format as
  "refused on purpose", which stopped being true when the reader landed.
* **Coincident endpoints are welded** (`NODE_WELD_MM = 0.05`). Routers emit
  coordinates that disagree in the last micron and fixed-grid rounding then
  splits a joint across two buckets — measured 11 um apart on a real 4-layer
  board, turning one routed net into three islands. Welding is deliberately
  capped at 50 um: traces are 100-500 um wide so endpoints that close overlap
  in copper, while a genuine unrouted gap is millimetres. A test pins the
  constant so nobody widens it to silence an alarm.
* **Altium Region/Fill shapes now count as pours when they carry a net.**
  They were excluded wholesale as a workaround for the net-indexing bug —
  teardrops with `NET=0` were resolving to net index 0 and inventing a pour.
  With references resolved against each Net record's own ID that cause is gone,
  so only genuinely netted regions register; keepouts, board cutouts and
  teardrops stay excluded by name. Pour *geometry* is still not meshed, so
  resistance remains trace-only and conservative.

## Copper pour meshing — connectivity done, containment deliberately not trusted

Pours now carry **outlines** in all three readers (`PcbZone`): KiCad
`(filled_polygon (pts …))` with a fallback to the drawn `polygon`, and Altium
`VX*/VY*` vertex lists in both the ASCII and binary paths. 298 outlines parse
across the 15-board corpus.

Two safety properties shape how they are used, and both are asserted by tests.

**Pours join, they never conduct.** A pour is real copper and really does join
pads, so it contributes edges to the connectivity graph. It is kept out of the
resistance solve entirely: any sheet resistance invented for it would make the
IR drop look *smaller* than trace-only does, and small is the direction that
under-reports brown-out. `conn_edges` decides whether a net is open; `edges`
stays the trace-only mesh, so every reported resistance is still an upper bound.

**Containment can clear a false open, never create one.** It is tempting to let
geometry replace the blanket "net has a pour ⇒ never open" exemption. Measured,
that made things worse — 4.4% → 5.1% false alarms — and the reason is thermal
relief: KiCad's `filled_polygon` has a clearance hole punched around every pad,
joined by spokes, so a perfectly connected pad sits geometrically *outside* the
fill. Altium's polygon outline has the mirror problem, covering copper that may
never have been poured. So containment is used only to connect, the blanket
exemption stays, and both halves fail safe. Reading spoke geometry is what would
let the exemption go.

Net effect on the corpus: no change to the 4.4% figure. The remaining opens are
not pour-related — none of the 49 on the largest binary board is on a net that
has a pour at all.

## The one substantial thing still open

**49 of the 67 remaining false opens sit on one binary board** (`artiq-kasli`),
and pours are not the cause — none of those nets has a pour. Each shows two
pads, a dozen or more segments across `F.Cu` and an inner layer, two vias, and
the two pads in separate groups. That pattern points at the binary reader's
via-to-track junction: the pads themselves sit on copper (measured 0.0 mm from
a segment), so the break is between layers, not at the pads. The via
coordinates look plausible, so the next step is checking whether track
endpoints actually land on the via centres in the binary path the way they do
in the text ones. Not yet diagnosed.

## The safety contract, made explicit and enforced

Every connectivity tolerance in `pcb_doctor.py` keeps one rule, now written
down as `SAFETY_CONTRACT`:

> false alarms are acceptable; false all-clears are not.

A wrong "copper open" wastes a member's afternoon. A missed one sends a dead
board to a fab. Every knob is therefore the smallest value that fixes a
*measured* failure, never the value that makes a complaint stop.

The risk with that rule is not that it is wrong, it is that someone later
widens a constant to silence a false alarm and flips the direction without
noticing. Three things guard against it:

* **A per-constant tripwire** whose failure message says, in the assertion
  itself, that widening is the wrong fix and points at the via-annulus rule as
  the right shape of one — that rule reaches *further* than the weld and is
  safer, because its distance comes from the via's diameter in the file rather
  than from a number someone picked. A tolerance justified by geometry can be
  as large as the geometry says; a tolerance justified by "the warnings
  stopped" cannot be any size.
* **A behavioural tripwire** (`TestSafetyAsymmetryHolds`) that ignores the
  constants entirely and asserts the property: a board with an unambiguous
  5 mm break must still come back open, whatever anyone changed to get there.
  Plus its resistance-side twin — a pour must never lower a reported
  resistance, because trace-only is an upper bound and a smaller number
  under-reports brown-out.
* **Both were verified by deliberate sabotage.** Widening `NODE_WELD_MM` to
  5 mm fails five tests; letting pours leak into the resistance solve fails
  two. A guard that has never been seen to fire is decoration.

