<!--
  KinematiK — Formula SAE / Formula EV full-car pre-validation platform
  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
  Open source. Original author: Frederik Thio, creator of KinematiK.
-->

# 🩺 PCB Doctor — usage

**Electronics (PCB) tab ▸ 🩺 PCB Doctor.** The board-check panel above it
screens the traces you *declare*; the Doctor screens the board you already
*routed*. It exists for the two reports every team files eventually:

> "The board passed DRC and simulated fine, then it failed on the car."
> "A component died even though the design was theoretically OK."

DRC checks geometry against rules. The Doctor checks copper against **physics**
and against the car's own **integration ledger** — the declared peak currents
that the LV/HV check already uses.

## Both EDAs, one diagnosis

| You route in | Drop in | Notes |
|---|---|---|
| **KiCad 5–10** | `.kicad_pcb` | Works as-is. Both net dialects: v5–9 `(net 2 "VCC")` and v10 `(net "VCC")`. |
| **Altium Designer / Protel** | `.PcbDoc` — **native binary works**, drop it straight in | Read for diagnosis only: findings, prescriptions and the viewer all work, but the one-click re-trace does not. |
| **Altium, for the re-trace** | `.PcbDoc` **saved as ASCII** (also `.pcb`) | *File ▸ Save As…* and pick the **PCB ASCII File** entry in the *Save as type* dropdown. Team-wide: *Preferences ▸ PCB Editor ▸ General ▸ Save PCB in ASCII format*. |

The format is detected from the file's **contents**, not its extension, so a
renamed export still works. Only the reader differs — every check, the viewer,
the re-trace and the report are one code path, so the two demo boards (the same
ECU in both formats) produce byte-for-byte the same findings. Altium layer
names are mapped for the physics (`MID3` → inner layer 3) and shown both ways.

**Why binary is read but never written.** A mis-parse on *read* is recoverable:
it shows up immediately as absurd geometry or garbage net names, and the reader
refuses the file rather than reporting on it. A mistake on *write* is not — it
corrupts a board you are about to pay to fabricate. So a native `.PcbDoc` gets
the full diagnosis and every numeric prescription, and the ASCII export is
needed only for the automatic re-trace. Reading it needs the `olefile` package
(in `requirements.txt`); without it you get the export instructions instead of a
stack trace.

## A net with no declared current gets no verdict

If nothing declares a net's peak current, the Doctor reports **MISSING**, not
FAIL, and offers no re-trace for it. Assigning a default and then failing the
board against that default is a guess wearing the costume of a finding — on a
real board most nets are unmatched, so it buries the two or three findings that
are real under twenty that are invented. Geometry checks (copper open, HV
clearance, diff-pair skew) don't depend on current and still run.

Declare currents in the integration ledger, or type them into the table; a
number you typed is a declaration and gets a real verdict. Setting a current
in code declares it too — `assignments[nid]["current_a"] = 8.0` and
`declare_net_current(assignments, nid, 8.0)` do the same thing, so there is no
convention to remember and no way to end up holding a number the Doctor still
calls undeclared.

Every Altium import prints **what it had to assume** (which unit bare numbers
are in, and that net references resolve 0-based against `|RECORD=Net|` order).
Glance at a familiar net name in the current table: if the names look shifted,
stop — the diagnosis is running on the wrong nets.

## The 60-second loop

1. **Drop the board file in**, or click **Demo · KiCad** / **Demo · Altium** to
   see the whole loop on a small ECU board with three planted failures — the
   same board in both formats, so you can watch your own format go through it.
2. **Skim the current table.** Every routed net is auto-assigned a current:
   name-matched nets take the owning subsystem's declared peak from the
   ledger (`FAN_PWR` → cooling's peak amps), signal nets get 50 mA, everything
   else 1 A — and every guess says so in *source*. Edit any number; the whole
   diagnosis follows it.
3. **Read the diagnosis.** Each finding says what's wrong, **why it fails on
   the car even though it simulated fine**, which **component or net** is
   implicated, and the exact numeric fix.
4. **Click re-trace.** Under-sized power segments are rewritten *in the
   original file* — only the width tokens change (KiCad's `(width …)`,
   Altium's `WIDTH=…`, **in the file's own units**, so a mil-based board stays
   imperial), everything else is byte-identical, so the patched board reopens
   in the EDA it came from with the routing intact. Download the patched file
   — same extension it arrived with — plus a hand-off fix report, and read the
   before/after FAIL count the Doctor computed by re-diagnosing the patched
   geometry.

## What it catches (that DRC doesn't)

| Failure on the car | What the Doctor computes |
|---|---|
| Trace runs hot / burns | IPC-2221 heating at the **bottleneck segment** of every net, per layer (inner copper cools ~half as well) |
| Trace opens like fuse wire | Onderdonk fusing current vs the board's fuse safety factor |
| Wide trace, dead board anyway | **Via ampacity** — the ⌀0.3 mm barrel choking a 1 mm trace at a layer change; prescribes the stitch count |
| ECU resets under load | True IR drop by **nodal analysis of the actual routed copper mesh** (segments + via barrels as a resistor network), worst pad-to-pad, vs the brown-out threshold from ⚙️ Board context |
| Board dead on arrival | **Copper opens** — pads on one net with no trace/via path between them (the rats-nest line everyone missed); nets with pours are never falsely flagged |
| Arced at the wet event | **HV clearance** vs IPC-2221 table B4 for every net you mark >60 V |
| CAN drops frames at full throttle | Diff-pair **length skew** and width steps, plus HV aggressors running parallel to the pair on real geometry |
| "A bad cap" / "the fuse just blew" | **Component derating**: electrolytics parked on hot copper (life halves per +10 °C), fuses whose marked rating is below their net's real current, connector pins asked to carry more than their family rating |

## 📏 Trace Prescriber (no board file needed)

The multi-layer routing answer sheet, for boards that don't exist yet: enter a
current, a temperature-rise budget, a run length and a via drill, and read off
the minimum width for **0.5 / 1 / 2 oz copper on outer and inner layers**, the
IR drop and heat each implies, and **how many vias every layer change needs**
so the barrel doesn't become the fuse.

## What it will never do

Analytic screening only — the same non-goal the rest of KinematiK keeps. It is
not a field solver (coupled-noise volts and eye diagrams are reported as *not
computed*, never invented), not a DRC replacement (a widened trace can newly
crowd an LV neighbour — the Doctor re-checks HV clearance on the patched
geometry, but re-run KiCad DRC before ordering), and not an autorouter that
invents new copper: it re-sizes the routes *you* made and prescribes the rest.
**Validation base:** three real KiCad boards (v9 and v10), nine real Altium
ASCII boards and four real **native binary** Altium boards (largest 51 435
segments / 3 137 pads, parsed in 3.7 s), up to a
224 x 131 mm 4-layer with 1981 segments, 303 vias and 229 components (parses
and diagnoses in 0.5 s). On all of them GND resolves as the most-padded net and
the copper pour resolves to GND — the cross-check that catches a shifted net
index, on every board in both formats.

Across the corpus the copper-open check raises **19 findings on 1509 routed
nets (1.3%)**, spread across eight boards with none carrying more than seven —
no systematic cause left, just a tail. Every residual error in the tool biases
toward false *alarm*, never false all-clear.

It reads copper, nets and components — not Gerbers, ODB++ or IPC-2581, which
carry the copper but not the net names and references every finding here is
written in terms of. Routed arcs are chorded into straight segments, so their
length reads a hair short (<0.15%); everything else about them is exact.
Diff-pair members are deliberately never auto-widened — width sets impedance —
they get a prescription instead. Validate the patched board in KiCad and your
fab's rules before manufacturing.
