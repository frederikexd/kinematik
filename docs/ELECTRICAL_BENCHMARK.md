# Electrical benchmark: KinematiK against KiCad, SPICE and a field solver

Plan and limits, fixed before any run: [ELECTRICAL_BENCHMARK_PLAN.md](ELECTRICAL_BENCHMARK_PLAN.md). Reference numbers:
`fsae_suspension/benchmarks/electrical/`. Tests: `fsae_suspension/tests/test_electrical_benchmark.py`, which re-check
KinematiK against the recorded references on every run without needing the tools. Regenerate the references with
`fsae_suspension/scripts/electrical_reference.py` wherever KiCad, ngspice and atlc are installed.

| Reference | Version | Stands in for |
|---|---|---|
| KiCad, through its own Python API (`pcbnew`) | 7.0.11 | KiCad: what is on a board |
| ngspice | 42 | SPICE (LTspice solves the same equations) |
| atlc, 2-D finite-difference field solver | 4.6.1 | Altium's impedance calculator, which is a field solver. Altium itself can't run here (Windows-only, licensed); it is run by hand from the kit in `benchmarks/electrical/altium/`. |
| 2-D copper solve on KiCad's own shapes (`electrical_reference.py`, `copper2d_resistance`) | round 3 | A board-level DC current-density solver (Altium's PDN Analyzer, Ansys SIwave DC) |

## Summary

| Test | Cases | Limit | Result |
|---|---|---|---|
| **A.** Board reading vs KiCad | KiCad's 14 demo boards: 15,000 tracks (180 arcs), 1,260 vias, 4,869 pads | exact counts; length 0.1 % | **14 of 14 exact.** Every pad position within 1 µm. Worst net length 0.009 mm (arcs). |
| **B1.** Precharge vs SPICE | 4 circuits, one with the resistor shorted mid-charge | 0.1 % | **Pass, worst 0.001 %** |
| **B2.** Pack over a lap vs SPICE | 520 points, 70 kW peak, 22.5 kW regen | 0.5 % V, 1 % I, 0.1 pp charge | **Pass: 0.0011 % V, 0.0011 % I, 0.0004 pp** |
| **B3.** Net resistance on real boards vs SPICE (1-D) | 963 nets | 2 % per net | **Miss.** Round 1: 61.7 % within 2 %, worst −98.5 %. Four flaws fixed; after: 72.5 % within 2 %, 91.8 % within 10 %, worst −24 % (same boards) |
| **B4.** Net resistance vs a 2-D copper solve, round 3 | 888 demo-board nets; **395 held-out nets on 13 Formula Student team boards** | 2 % per net | **Still misses 2 % on about one net in eight, now within 5 % on 98 %.** Held out: 87.8 % within 2 %, 98.2 % within 5 %, 99.2 % within 10 %, median −0.40 %, worst −11.2 % / +18.0 %. Before: 8.1 % within 2 %, median +14.6 % |
| **A2.** Board reading vs **Altium Designer 26.11.1** | 3 boards imported into Altium and saved as binary `.PcbDoc`: 9,429 tracks, 180 arcs, 901 vias, 2,763 pads, 346 components, 467 nets | exact counts; length 0.1 % | **Pass after two reader fixes it exposed.** Every count equals Altium's own; net lengths within 0.04 % of KiCad's original. First read: StickHub 289 pads and 90 components (Altium: 278, 94) |
| **C.** Microstrip impedance vs field solver | 5 single-ended, 6 differential | 5 % / 10 % | **Round 1 miss:** single-ended −8.3 % and −6.4 % at the range ends. Formula replaced. **Round 2, unseen geometries: 10 of 10**, worst 3.4 % |

## A. Does KinematiK read a board the way KiCad does?

KiCad's own reader (`pcbnew.LoadBoard`) against KinematiK's PCB Doctor on every board in KiCad 7's demos, none
chosen or left out. Counts of straight tracks, arc tracks, vias, pads and footprints, the via drill sizes, and each
net's track widths all match exactly on all 14 boards. Each net's routed length agrees within 0.009 mm, the worst
being StickHub's arcs, which KinematiK splits into short chords. All 4,869 pad centres agree with KiCad's within
1 µm.

## B. SPICE (ngspice)

**B1. Precharge.** Four RC precharge circuits, compared at 50 time points and on time to 90 % and 95 %, peak current
and the energy the resistor absorbs. The circuits were 600 V / 2 kΩ / 500 µF, 400 V / 1 kΩ / 1.2 mF, 300 V / 5 kΩ /
200 µF, and the first again with the resistor shorted at 2.5 s. The worst difference is 0.001 %.

**B2. The pack over a lap.** A 600 V-class pack with a falling open-circuit voltage, 0.25 Ω and 14 Ah, driven by the
terminal power of a simulated autocross lap (regen included). KinematiK steps the charge forward explicitly; SPICE
integrates it with error control. Terminal voltage and current agree within 0.0011 % at every point, and the final
state of charge within 0.0004 percentage points. (Current is compared relative to the larger of the SPICE current
and 1 A, so that the zero crossings at the switch to regen don't divide by zero.)

**B3. Net resistance on real boards.** For every net KinematiK solves on the demo boards (963, no pours), SPICE got an
independent resistor network built from KiCad's geometry and KiCad's own hit tests, then measured the resistance
between the two pads KinematiK names worst.

- **One correction to the reference, made after the first look.** The first SPICE netlists joined items only where
  an anchor point (a track end, a via centre) landed inside another item. KiCad's rule is that copper which
  *overlaps* is connected. The netlists were rebuilt to that rule before scoring.
- **Round 1: miss.** 594 of 963 nets within 2 % (61.7 %), 838 within 10 %, median 0.82 %. Where KinematiK missed, it
  was mostly *low*, the unsafe direction for a brown-out check, by up to 98.5 %.
- **Four flaws found and fixed**, all of them in the unsafe direction:
  1. **The component-body shortcut.** Any two pads of one part on the same net were bridged by a 10 mΩ link "through
     the part". That's right for a fuse, a shunt or a net-tie, and wrong for an IC: on KiCad's video board, U8 pins
     60 and 70 are joined by a 16 mm loop of track (45 mΩ) that the bridge hid. It now applies only to two-terminal
     parts.
  2. **A pad joined the middle of a via.** A surface pad touching a via joined its centre, so a top pad and a bottom
     pad on one via had nearly no resistance between them. A surface pad now joins the via on its own layer.
  3. **The via barrel shorted out.** A track ending on a via joined the via's centre directly, bypassing the barrel
     resistance on every via. It now joins the via on its own layer.
  4. **A pad's reach was a circle.** A pad caught any copper within half its longest side, ignoring its shape and
     rotation. It now uses the pad's own outline: circle, oval, or rectangle with rounded corners. On 29,214
     random points that agrees with KiCad's own pad hit test 98.7 % of the time. The rest are pads whose copper is
     offset from the drill hole, which KinematiK does not read yet.
- **After the fixes**, on the same boards, so not independent of the fixes: 699 of 964 nets within 2 % (72.5 %), 885
  within 10 % (91.8 %), median 0.21 %. The worst low reading went from −98.5 % to −24 %. That **still misses the 2 %
  limit.**
- **What's left** runs both ways, mostly on long nets. Both KinematiK and the reference model a net's copper as
  1-D resistors; real copper is 2-D. Round 3 (B4) replaces this reference with a 2-D solve of the copper.

**B4. Net resistance against a 2-D solve of the copper (round 3).** Registered in plan addendum 2 before any run.
Every copper item of a net is drawn on a 12.5 µm grid per layer from KiCad's own shapes (tracks and arcs as
capsules, pads from KiCad's polygon, vias as rings); neighbouring cells conduct in proportion to their copper
coverage; each pad is one terminal; via barrels join the layers. 1 A in at one pad, out at the other. The grid was
halved until the answer stopped moving (25 µm differed from 12.5 µm by up to 1.5 %).

The registered order, followed:

1. **Round 3a, KinematiK as it was**, on the 14 demo boards: 50 of 892 nets within 2 % (5.6 %), 395 within 10 %,
   **median +10.7 %**. Safe in direction (768 high, 74 low), but not accurate.
2. **Changes designed on the demo boards only.** Five causes, each traced to a specific net:
   1. **Track on top of a pad, via ring or wider track was counted as track.** The wider copper under it carries
      the current; this was the +10.7 %. Only the length outside such copper now counts, sampled every 25 µm, and a
      track lying wholly inside a wider one carries none.
   2. **The nodal solve lost accuracy.** numpy's pseudo-inverse of the whole network lost up to a quarter of the
      answer on long nets (coldfire demo, /PST0: 135 mΩ for a chain of tracks summing to 180 mΩ). It is now a
      grounded solve on each connected piece.
   3. **Arcs were shorted out.** An arc is stored as chords about 47 µm long, under the 50 µm endpoint weld, which
      joined each chord's own ends. A track's own two ends are no longer welded.
   4. **Pads sharing a number were one node.** A tactile switch's two pin-1 legs became a 0 Ω shortcut (coldfire,
      D102 to LV101: −27 %). They are now separate copper, joined for connectivity only.
   5. Each pad's real side (its footprint's, for a pad that doesn't name one) is used for coverage.

   On the demo boards after the changes: 798 of 888 within 2 % (89.9 %), 865 within 5 % (97.4 %), 882 within 10 %,
   median −0.14 %, worst −11.1 % / +21.7 %. These are the boards the changes were made on.
3. **Frozen** (commits 599ab39, 8c8fbf9), then the 2-D solve ran **once** on the held-out boards: the 20 boards from
   public Formula Student team repositories that KiCad 7 loads, 13 of which have nets KinematiK solves (pour-free,
   two or more pads). The 2-D solve gave no answer on 17 of 412 nets (no copper path between the pads, or a pad
   name used twice), leaving 395.

| Held-out team boards | Within 2 % | Within 5 % | Within 10 % | Median | Worst low | Worst high |
|---|---|---|---|---|---|---|
| KinematiK before (3a) | 32 of 396 (8.1 %) | 21.0 % | 34.6 % | +14.6 % | −26.0 % | +207.5 % |
| **KinematiK after** | **347 of 395 (87.8 %)** | **98.2 %** | **99.2 %** | **−0.40 %** | **−11.2 %** | **+18.0 %** |

The held-out result matches the design boards, so the changes were not fitted to them. **It still misses the
registered 2 % limit** on 48 of 395 nets, mostly short nets of a few milliohms where the current crowds into the
pad edge, which a 1-D model can't place. It now reads low as well as high, so the brown-out check multiplies the
resistance by 1.15, set on the demo boards (worst low −11.1 %). On the held-out boards that factor covers every
net: the worst, −11.2 %, needs 1.126.

**Reading the board (limit A, gap closed).** Pads whose copper is offset from the drill hole (`(drill … (offset))`)
are now read, turned with the pad, and a via without its own drill takes the net class's or board's default.
KinematiK's pad outline agrees with KiCad's own pad hit test on 99.25 % of 29,214 random points; the rest are custom
pad shapes (taken as their bounding rectangle) and points on the edge of a circle.

## A2. Does KinematiK read an Altium board the way Altium does?

The author imported pic_programmer, StickHub and video into Altium Designer 26.11.1 with Altium's KiCad Importer,
saved each as a binary `.PcbDoc`, and read Altium's own counts from Board Information and the PCB Filter
(`IsTrack And OnSignal`). KinematiK's binary reader then read the saved files
(`benchmarks/electrical/altium/results_part1.json`).

- **First read: miss.** StickHub came out at 289 pads and 90 components where Altium reports 278 and 94; video at
  2,450 pads where it has 2,238. Two reader flaws:
  1. **Phantom pads.** Pad records are found by a byte signature. Bytes inside other records matched it and were
     read as pads, some with sizes of −2,897 mm. A pad record is now accepted only if its subrecords frame within
     the stream and it has a real size (0–100 mm), hole (under 20 mm) and position, and scanning resumes after the
     whole record.
  2. **Padless components dropped.** Components with no pads (logos) were left out. They are kept.
- **After the fixes: pass.** Every count KinematiK reads equals Altium's: StickHub 1,108 copper tracks, 94
  components, 278 pads, 87 vias; video 7,951 tracks; pic_programmer 63 components, 247 pads, 6 vias. Against KiCad's
  original, the only difference is three zero-length tracks on StickHub that Altium's importer drops. The routed
  length of all 467 nets is within 0.04 % of KiCad's.

## C. Controlled impedance (atlc field solver)

**Checking the checker.** atlc solved a centred stripline whose impedance is exact (Cohn): within 0.39 % in air and
in FR-4. Its results are therefore used.

**Round 1: miss.** KinematiK used the IPC-2141 microstrip formula. On FR-4 (εr 4.3, h 0.2 mm, 35 µm copper) it
landed within 5 % in the middle of its range (−2.3 % to −3.6 %) but missed at both ends: −8.3 % at w/h 0.2 and
−6.4 % at w/h 2.0. Edge-coupled differential impedance passed, 6 of 6 within 3.7 %.

**The fix.** Hammerstad and Jensen's microstrip formula with Wheeler's thickness correction, the closed form
KiCad's own calculator uses. On the round-1 cases: single-ended within 3.3 %, differential within 2.4 %.

**Round 2, geometries registered after round 1 and never used to choose the formula.** εr 3.66 (Rogers RO4350B)
and εr 4.6, h 0.15 mm, w/h 0.3–1.8, plus two differential pairs: **10 of 10 within limits.** Single-ended worst
−3.4 %, differential worst −1.6 %. KinematiK reads slightly low at narrow traces throughout.

## Not covered

- **Altium's impedance calculator** (part 2 of the kit, 21 cross-sections in the Layer Stack Manager) has not been
  run yet. Its result is added here when it comes back, whatever it shows.
- **Trace heating (IPC-2221)** and **fuse and wire I²t** are standards KinematiK implements directly. A tool that
  computes the same equations would only repeat them. Measured heating (IPC-2152 test data) would be the
  meaningful check, and isn't here yet.
- **Net resistance on pours.** Nets with copper pours are left out of B3 and B4: KinematiK deliberately gives a pour
  no resistance (connectivity only), so its trace-only number is an upper bound there, not an estimate.
