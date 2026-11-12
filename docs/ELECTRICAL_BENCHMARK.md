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
| **B4.** Net resistance vs a 2-D copper solve, round 3 (as first published) | 888 demo-board nets; 395 held-out nets on 13 FS team boards | 2 % per net | Held out: 87.8 % within 2 %, 98.2 % within 5 %. **Superseded by B5:** the reference read thin diagonal tracks up to 8 % high |
| **B5.** Net resistance vs the **corrected** 2-D solve | 681 demo-board nets (design); **142 sampled held-out nets on FS team boards** | 2 % per net | **Still misses 2 % on about one net in eight, and errs high when it misses.** Held out: 87.3 % within 2 %, 97.9 % within 5 %, 98.6 % within 10 %, median +0.55 %, **worst low −3.1 %**, worst high +21.2 %. Before round 3: 2.8 % within 2 %, median +16.9 % |
| **A2.** Board reading vs **Altium Designer 26.11.1** | 3 boards imported into Altium and saved as binary `.PcbDoc`: 9,429 tracks, 180 arcs, 901 vias, 2,763 pads, 346 components, 467 nets | exact counts; length 0.1 % | **Pass after two reader fixes it exposed.** Every count equals Altium's own; net lengths within 0.04 % of KiCad's original. First read: StickHub 289 pads and 90 components (Altium: 278, 94) |
| **A3.** Board reading of **native Altium boards** vs Altium Designer 26.11.1 | 6 Formula SAE boards designed in Altium (Queen's FSAE), never converted: 2,226 tracks, 21 arcs, 201 vias, 1,130 pads, 328 components | exact counts | **6 of 6 boards, 30 of 30 counts identical** |
| **A4.** Board reading of **KiCad 8/9 boards** vs **KiCad 9.0.3** | 4 FS team boards saved in KiCad 8–9 (PUT Motorsport BMS LV, BMS HV master, Frontbox; WindingMotor distribution hub): 6,068 tracks, 50 arcs, 3,398 vias, 2,731 pads, 772 components, 505 routed nets | exact counts; length 0.1 % | **4 of 4 boards, every count identical.** Worst net length 0.015 % |
| **B6.** Net resistance on **KiCad 8/9 boards** (held out, KinematiK frozen at a3805c1) vs the corrected 2-D solve on KiCad 9.0.3's shapes | 100 nets drawn with seed 20261112; 95 solved by the reference | 2 % per net | **Miss.** 70 of 95 within 2 % (73.7 %), 85 within 5 % (89.5 %), **95 of 95 within 10 %**, median +0.81 %, worst −9.1 % / +6.9 %. Every miss is a short pad-to-pad net (2–9 tracks) |
| **B7.** Net resistance by the **sheet solve** between the 1-D worst pair (held out, frozen 96ca8c4) | 100 KiCad 8/9 nets B6 never sampled, seed 20261113 | 2 % per net | **92 of 92 refined nets within 2 %**, worst −1.3 % / +1.2 %; rate ≥ 96.8 % at 95 % confidence. 8 nets with repeated pad names unscored |
| **C.** Microstrip impedance vs field solver | 5 single-ended, 6 differential | 5 % / 10 % | **Round 1 miss:** single-ended −8.3 % and −6.4 % at the range ends. Formula replaced. **Round 2, unseen geometries: 10 of 10**, worst 3.4 % |
| **C2.** Microstrip impedance vs **Altium Designer 26.11.1** | the same 21 cross-sections in Altium's Layer Stack Manager, plus 5 extras | 5 % / 10 % | **21 of 21.** Single-ended worst −1.9 %, differential worst +4.0 %. Altium and atlc agree within 1.6 % on every case |
| **D.** Trace heating vs **NBS 1956 measurements** (the data behind IPC-2221) | 44 reliably read points, 10–100 °C rise (10 more at 4–16 sq mil left out) | KinematiK ≤ 1.05 × measured at every point | **Plain IPC-2221 missed on 2** (wide traces at 10 °C: +5 %, +8 %). KinematiK now uses IPC-2221 × 0.975: **44 of 44**, worst +4.96 %, median 0.89 × measured. Calibrated on these points, so a calibration check, not a validation |
| **E.** Nets with copper pours: KinematiK's with-pour estimate vs a 2-D solve of KiCad's filled copper | Round 1: 17 held-out nets. Rounds 3–4: 54 pour nets of the KiCad 8/9 boards | 2 % per net | Round 1 **miss** (35 %). Round 3 held out: **44 of 50 within 2 %, the 6 misses low** (grid, empty pours). Round 4 (grid check) held out: 5 of 5 converged nets within 2 %. Large planes not estimated |

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

## A3. Boards designed in Altium

A2's boards started in KiCad. Plan addendum 7 adds six boards Queen's Formula SAE designed in Altium (`qfsae/pcb`:
PDP_V3, BSPD_002, Q23_BRAKE_LIGHT_O1, Analog to CAN Rev2, PMBoard, Triangular_Brakelight), never converted. The
author opened each in Altium Designer 26.11.1 and read components, pads and vias from Board Information and copper
tracks and arcs with the PCB Filter (`IsTrack And OnSignal`, `IsArc And OnSignal`)
(`benchmarks/electrical/altium/results_native.json`).

| Board | Components | Pads | Vias | Tracks | Arcs |
|---|---|---|---|---|---|
| PDP_V3 | 52 | 329 | 53 | 501 | 0 |
| BSPD_002 | 75 | 242 | 39 | 824 | 1 |
| BRAKE_LIGHT_O1 | 73 | 147 | 21 | 160 | 0 |
| Analog to CAN Rev2 | 29 | 116 | 23 | 218 | 0 |
| PMBoard | 76 | 250 | 65 | 386 | 0 |
| Triangular_Brakelight | 23 | 46 | 0 | 137 | 20 |

**KinematiK reads every one of the 30 counts exactly as Altium does.** Pads that belong to no component are kept
under a placeholder that the fix report no longer counts as a component (it had read 76 components on BSPD_002).

## A4 and B6. Boards saved in KiCad 8 and 9

Plan addendum 7 and its amendment. The author could not install KiCad 9, so KiCad 9.0.3 (Ubuntu 25.10's package)
ran in an isolated Ubuntu 25.10 root in the benchmark workspace, driving the kit's `export_kicad9.py`
(`benchmarks/electrical/kicad9/`). The kit as first shipped failed on KiCad 9 before writing anything (vias take a
layer for their width; a pad's parent no longer has `GetReference`); it was fixed without changing what it writes.

- **Reading (A4): pass.** On all four boards every count equals KiCad 9's (tracks, arcs, vias, pads, components)
  and the routed length of all 505 nets is within 0.015 %.
- **Net resistance (B6, held out): miss.** 100 nets drawn with `random.Random(20261112)` from the 452 trace-only nets
  KinematiK solves on these boards, KinematiK frozen at a3805c1, the corrected 2-D solve on KiCad 9's own shapes.
  The reference returned no value on 5 (four long signal nets on the distribution hub and one high-voltage net on
  the BMS master); they are counted, not scored. On the other 95: 70 within 2 % (73.7 %), 85 within 5 %, **all 95 within
  10 %**, median +0.81 %, worst −9.1 % and +6.9 %.
- **Where it misses.** Every net off by more than 4 % is a short pad-to-pad link of 2–9 tracks (gate resistors,
  bootstrap capacitors, feedback dividers). On those the copper under and around the pads is most of the resistance,
  and KinematiK's 1-D model of how current enters a pad is off by a few percent either way. These boards have more of
  them than the round-3 team boards (87.3 % within 2 % there).
- **What it means for the checks.** The model's 1.10 margin alone falls short on the worst net by 0.01 %
  (0.909 × 1.10 = 0.9999). The brown-out check also assumes copper up to 10 % thin (× 1/0.90), so its bound is
  1.22 × the estimate and covers every net here with 11 % to spare. It is not a 2 % estimator on short
  pad-dominated nets.

## B7. Net resistance by the sheet solve (the fix for B6)

Plan addendum 12, registered before any number. The pour estimate's 2-D sheet solve works on any net and checks its
own grid, so `pcb_doctor.refined_net_resistance` runs it between the 1-D model's worst pair and uses the result when
it converges; otherwise the 1-D value stands, and the result says which was used.

- **Design check** (the 95 B6 nets): 93 refined, 93 within 2 %.
- **Held out** (`benchmarks/electrical/kicad9/net_resistance_refined.json`): 100 nets drawn with
  `random.Random(20261113)` from the 352 trace-only nets B6 never sampled, each solved once by the reference from
  KiCad 9.0.3's shapes. **92 refined, 92 of 92 within 2 %**, worst −1.28 % / +1.23 %, median 0.00 %. The 95 % lower
  bound on the rate is 0.05^(1/92) = 96.8 %. On the same 92 nets the 1-D model alone scores 68 within 2 %, worst
  +30.6 %.
- **The other 8** are connector nets on the distribution hub whose pad names repeat. The refinement declines a pair
  it cannot name uniquely, and the reference returns no value for them either, so they are unscored and keep the
  1-D value with its margin.
- **As shipped** the refinement is limited to 150,000 cells so one net takes at most a few seconds (4.6 s worst
  here, 100 s for all 100). Under that limit 78 of the 92 refine, all 78 within 2 %; the 14 larger ones keep the 1-D
  value (13 within 2 %, worst +2.2 %).
- **Brown-out check:** its bound is now the larger of 1-D × 1.10 and sheet solve × 1.02, over the copper finish. It
  cannot be less conservative than before.

## B5. Net resistance against the corrected 2-D reference

Plan addendum 3 and its amendments. While looking into the nets that still missed after round 3, refining the grid
moved the 2-D answer toward KinematiK on StickHub's thin diagonal tracks: the reference itself was off, by up to
8 % high, from two discretization flaws (links conducted by the smaller cell's coverage; pads snapped to whole
cells). The corrected solve passes a rotation test (the same pad-track-pad at 0°, 15°, 30° and 45° agrees within
0.9 % and converges on the exact value) and is solved per net with at least 24 cells across the narrowest track;
one net in five is also solved at twice the cell size, a median difference of 0.12–0.14 %
(`benchmarks/electrical/nets_round3_corrected.json`).

**Changes, designed on the demo boards only, then frozen (a3805c1):**
1. A track end whose copper only touches a pad joins it through that copper, and only if its track doesn't carry on
   to the pad anyway. Joining every such end at 0 Ω made a second path beside arcs.
2. A pad is tested across the track's whole width: a track running alongside a pad with its edge on the pad hands
   its current over there (kit-dev, C205 to U202, read 34 % high).
3. The brown-out margin is 1.10 (worst low on the demo boards −7.3 %), down from 1.15.

| | Nets | Within 2 % | Within 5 % | Within 10 % | Median | Worst low / high |
|---|---|---|---|---|---|---|
| Demo boards (design), before round 3 | 685 | 3.4 % | 16.6 % | 40.4 % | +11.8 % | −21.8 % / +193 % |
| Demo boards (design), now | 681 | 92.5 % | 99.0 % | 99.7 % | +0.41 % | −7.3 % / +16.5 % |
| **Team boards (held out), before round 3** | 142 | 2.8 % | 16.2 % | 30.3 % | +16.9 % | −5.1 % / +194 % |
| **Team boards (held out), now** | **142** | **87.3 %** | **97.9 %** | **98.6 %** | **+0.55 %** | **−3.1 % / +21.2 %** |

With 142 nets the held-out share within 2 % is known to about ±5.5 percentage points. **The 2 % limit is still
missed on about one net in eight.** The misses are short nets of a few milliohms and they read *high*: the worst
low reading on the held-out boards is −3.1 %, well inside the brown-out margin, so they can raise a needless
warning but not hide a brown-out.

**Round 4 did not run.** Its registered repositories held no board KiCad 7 can read (KiCad 8–9 files, or empty
templates), and only KiCad 7 runs here; recorded in the plan before any held-out result was seen.

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

## C2. Altium's impedance calculator

The author built each case's stack in Altium Designer 26.11.1's Layer Stack Manager (two 0.035 mm copper layers, one
dielectric of the case's h and Dk, solder masks removed, flat conductors), set a microstrip profile on the top layer
referenced to the bottom layer, entered each width (and gap), and photographed the result
(`benchmarks/electrical/altium/results_part2_impedance.json`). Where the impedance column was cut off, the value
comes from Altium's deviation from the profile's target: Z = target × (1 + deviation), checked against the cases that
displayed in full.

**21 of 21 within limits.** Single-ended (14): KinematiK −1.3 % to −1.9 % from Altium. Differential (7): −2.2 % to
+4.0 %, the worst being 0.1 mm traces 0.1 mm apart. Five extra geometries (Altium's own 50 Ω and 100 Ω solutions,
three differential pairs entered on the wrong profile) are within 1.9 %. KinematiK reads slightly low throughout,
as it does against atlc.

**The reference checked.** Altium and atlc, two independent field solvers, agree within 1.6 % on all 21 cases. Both
profiles had etch 0 (confirmed by the author afterwards), so Altium's traces are rectangular, as atlc's are.

## D. Trace heating against measurements

The reference is NBS Report 4283 (Hoynes, National Bureau of Standards, 1956): measured current against cross-section
of etched copper on 1/16 in. laminate, one figure per temperature rise (10, 20, 30, 45, 60, 75, 100 °C). The
MIL-STD-275 and IPC-2221 charts descend from it. The report gives the data only as graphs, so the plain etched curve
was read off the scan: grid lines found automatically (both axes are square-root scales), the curve read where it
crosses each grid line of cross-section, and crossings left out where the four curves crowd together or a letter
sits on the line (`benchmarks/electrical/nbs_heating.json`). A second pass at another resolution agrees within 0.2 %
(median) where it found the same line.

**This is not an independent test.** KinematiK's formula (IPC-2221, I = 0.048 ΔT^0.44 A^0.725) is a fit to charts
derived from these curves, after a 10 % derating. It checks how faithfully the formula represents the measurements
behind it.

**First scoring: 48 of 54 within the limit.** Six points were above 1.05 × measured. On a zoomed check:

- **Four were at 4 and 16 sq mil**, where the four NBS curves converge within the reading resolution; one was
  plainly misread (3.35 A read where the curve is about 2.0 A). The plan's rule leaves such crossings out, and it
  had not been applied consistently. It now is, to every point at those two sizes, after the first scoring.
- **Two are real.** Wide conductors (576 and 676 sq mil, about 10–12 mm of 1 oz copper) at a 10 °C rise: plain
  IPC-2221 allows 5 % and 8 % more current than measured. Temperature rise goes as current to the power 2.3, so
  8 % too much current is about 19 % more heating than predicted.

**The fix.** KinematiK's ampacity is IPC-2221 × 0.975. All 44 reliably read points are within the limit (worst
+4.96 %), median 0.89 × measured, at a cost of 2.5 % of rated current everywhere. A refitted power law (with and
without curvature, fitted on some figures and tested on the others) predicted the held-out temperatures no better.
The factor was set on these same points, so D is now a calibration check, not a validation.

An independent check needs IPC-2152's 2009 measurements, which are not public.

**Changes made after D.** The aim is as few false alarms as possible without reading optimistic.
- **Outer layers: unchanged.** A power law refitted to four of the seven figures and tested on the other three did
  no better. To stay at or below every design point it had to sit about 18 % below the measurements, more
  conservative than IPC-2221's 9 %. The misses are in two corners, not in the curve's shape.
- **Inner layers: rated like outer layers.** IPC-2221 halves the current an inner-layer trace may carry. IPC-2152's
  measurements found buried traces run no hotter, slightly cooler (Brooks, *Trace Currents and Temperatures
  Revisited*; Adam, SEMI-THERM 2004: about 5 %, not 50 %), so the halving only produced false alarms on inner layers.
  Inner layers now use the outer-layer rating. NBS measured outer conductors only, so this change rests on those
  sources, not on D.

## E. Nets with copper pours

Plan addendum 4. KinematiK's 1-D model does not mesh a pour, so on a poured net it covers only the pads its tracks
join. (This section first called that an upper bound. It is not: see "What the brown-out check does on a pour net"
below.) `suspension/pcb_pour.py` adds an estimate *with* the pour: a sheet
solve of KinematiK's own reading of the file (tracks, pads, vias and the zones' `filled_polygon`) on its own grid.
The reference is the corrected 2-D solve with KiCad's own filled zones added (`benchmarks/electrical/pours.json`).
Planes too large for this machine's memory (1.5 million cells per layer) are left out: 6 on the demo boards and 6 on
the team boards, mostly whole-board ground planes.

- **Demo boards (design), 6 nets:** +0.3 % to +3.5 %, all within 5 %, 4 of 6 within 2 %.
- **Team boards (held out, solved once), 17 nets: miss.** 6 within 2 % (35 %), 12 within 5 % (71 %), 13 within
  10 % (76 %), median +2.6 %, worst +47 % (a 0.65 mΩ tractive-system return).
- **Direction: never low.** On every scored net, demo and held out, the estimate is at or above the reference (worst
  +0.3 % on the high side). KinematiK's coarse grid (6 cells across the narrowest track) under-resolves thermal
  spokes and narrow necks in the pour, which can only add resistance.
- **Not scored:** on 27 held-out nets KinematiK named two pads with the same designator and number (for example a
  part with two pad 1s), which the reference cannot tell apart, or the reference found no copper path.

**Round 2** (addendum 6, the solve redone with the reference's discretization), held out on 43 nets: 15 within 2 %,
every miss high. The misses came from two reading flaws: KiCad `custom` pads read as their 0.1 mm anchor (25 nets)
and KiCad 5 pours read without their outline stroke (3 nets).

**Round 3** (addendum 8) reads both. On round 2's misses it now lands within ±0.5 % wherever the reference converges.
On ten hand-solder LED nets the reference itself keeps moving as the grid is refined (0.21 → 0.55 mΩ), so they can't be
scored at 2 %.

**Round 3 held out, KiCad 8/9 boards** (`benchmarks/electrical/kicad9/pours_round3.json`, `pours_round4.json`;
plan addenda 8 and 9). 54 of the 63 pour nets were solved; 8 planes are over the memory cap (KinematiK returns "too
large" for them and gives no estimate) and 4 just under it were not reached.
- First pass, 43 nets: 19 references agreed with themselves at twice the cell size, and round 3 was within 2 % on all
  19. The other 24 were then solved at half and a quarter cell: 23 converged, and against them round 3 was within
  2 % on 17 and **low on 6** (−2.2 % to −6.5 %).
- The 11 larger nets: 8 references converged, round 3 within 2 % on all 8 (worst +0.15 %).
- **Round 3 in all: 44 of 50 held-out nets within 2 % (88 %). All 6 misses read low.**

**Why it read low, and the two fixes.**
1. *Grid.* On 4 of the 6 the answer was still rising when the cell was halved, and KinematiK solved once.
   **Round 4** (addendum 10) solves again between the same pads at half the cell, and at a quarter if those differ by
   more than 2 %. It returns the finest value with `grid_change_pct` and `converged`; where a finer grid does not
   fit in memory the result is marked not converged.
2. *Empty pours.* The other 2 stayed 5 % low after that. Each has a pour KiCad filled and left empty (an island
   reaching no pad), and KinematiK had counted its drawn outline as copper. Such a pour is no longer copper
   (addendum 11), which also stops it from joining pads in the connectivity check.

**Round 4.**
- *Design data* (all 43 nets of the first batch re-scored): 42 where both converge, **40 within 2 %**, worst +0.80 %.
  The 2 misses (−4.9 %, −5.0 %) are the empty-pour nets, fixed by addendum 11 and no longer pour nets.
- *Held out* (the 11 larger nets, never looked at before round 4 was frozen): KinematiK marked 7 converged. On the 5
  whose reference also converges, **5 of 5 are within 2 %** (worst +0.38 %). On `/MASTER_FUSE_OUT` and `+24V` the
  reference has not converged at its own cell size, and the finer solve did not finish; they are unscored.
  4 were marked not converged and are reported as such.
- *Not finished:* `+3V3` on the BMS LV board (1.18 M cells) did not finish within 40 minutes; 3 more nets just under
  the memory cap were not reached.

**The nets "the reference could not resolve" (addendum 13).** On 15 design nets the reference kept climbing at every
grid refinement. The fault was in the reference: it put a pad's terminal at the mean of its outline points, which on
the concave custom LED pads falls outside the pad, so current entered the pour at a point. Corrected (terminal at
the pad's own position) and re-solved: where both KinematiK and the reference converge, **20 of 20 within 2 %**,
worst 0.78 % (`benchmarks/electrical/pours_unresolved_design.json`). Convex pads are unaffected.

**What the brown-out check does on a pour net.** Before this round it used the 1-D figure, which covers only pads
joined by tracks, and called it conservative. It was not: a load fed through the pour was never checked (on one net
the worst path through the pour was 40 × the tracked figure), and on 29 of 52 KiCad 9 pour nets there was no figure
at all. Now the check solves the pour within the app's size limit and uses the larger of the two (×1.02 when the
pour estimate converged, ×1.10 when not). Where the pour is too large to solve, the report says the drop through
the pour was **not checked**. On the 52 KiCad 9 pour nets, 32 solve within the limit (35 s for all of them).

**What the app runs, and what the benchmark leaves open.** In the app the pour solve is limited to 150,000 cells per
net, so it answers in seconds. The nets this benchmark could not finish (`+3V3` and three others of 1.0–1.5 million
cells) are far outside that limit; in the app they get the warning that the drop through the pour was not checked,
never a number. The 5 estimates KinematiK labels not converged are all within 3.4 % of the reference, inside the
×1.10 the brown-out check applies to them.

**What can be said.** A pour estimate KinematiK marks converged has matched a converged reference within 2 % on
every net where both exist: 5 of 5 held out, and 60 of 62 on design data (the 2 misses being the empty pours since
fixed). That is strong evidence but only 5 held-out nets, not a proven rate. Whole-board planes are not estimated,
and the report says so.

## From the file to the built board

A model that is right about the file can still be wrong about the board: finished copper comes back thinner or
thicker than ordered, and joints, connectors and crimps add resistance no layout file holds. Two things in KinematiK
account for this:

- **The brown-out check assumes copper up to 10 % thin** (`COPPER_FINISH_FACTOR`, set your fab's tolerance with
  `diagnose(copper_finish=...)`), on top of the model's 1.10 margin.
- **A bring-up measurement plan** (`bringup_measurements`, and a table in the fix report): for each declared power
  net, the two pads to probe with a 4-wire milliohm meter, the resistance expected at 22 °C and the band a good board
  falls in (the model's held-out error with the copper tolerance). A reading above the band means thin copper, a
  cracked joint or a wrong track; the check is re-run with the measured value.

Heating stays an IPC chart for an isolated trace in still air: it does not model neighbouring hot parts, planes
underneath, airflow or enclosures. Every report says so and asks for the hottest local temperature near the trace as
the ambient, which keeps the result on the safe side without pretending to model the rest.

## Not covered

- **Trace heating against independent measurements.** D checks KinematiK's formula against the 1956 data it
  descends from; IPC-2152's 2009 measurements would be the independent test, and are not public.
- **Fuse and wire I²t** are standards KinematiK implements directly; a tool computing the same equations would only
  repeat them.
- **Measurements on a real Formula Student board** (milliohms across a power net, temperature under load) need
  hardware.
- **Whole-board ground planes** are too large for the pour reference on this machine (E).
