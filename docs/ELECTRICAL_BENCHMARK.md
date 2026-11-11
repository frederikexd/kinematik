# Electrical benchmark: KinematiK against KiCad, SPICE and a field solver

Plan and limits, fixed before any run: [ELECTRICAL_BENCHMARK_PLAN.md](ELECTRICAL_BENCHMARK_PLAN.md). Reference numbers:
`fsae_suspension/benchmarks/electrical/`. Tests: `fsae_suspension/tests/test_electrical_benchmark.py`, which re-check
KinematiK against the recorded references on every run without needing the tools. Regenerate the references with
`fsae_suspension/scripts/electrical_reference.py` wherever KiCad, ngspice and atlc are installed.

| Reference | Version | Stands in for |
|---|---|---|
| KiCad, through its own Python API (`pcbnew`) | 7.0.11 | KiCad: what is on a board |
| ngspice | 42 | SPICE (LTspice solves the same equations) |
| atlc, 2-D finite-difference field solver | 4.6.1 | Altium's impedance calculator, which is a field solver. Altium itself can't run here (Windows-only, licensed). |

## Summary

| Test | Cases | Limit | Result |
|---|---|---|---|
| **A.** Board reading vs KiCad | KiCad's 14 demo boards: 15,000 tracks (180 arcs), 1,260 vias, 4,869 pads | exact counts; length 0.1 % | **14 of 14 exact.** Every pad position within 1 µm. Worst net length 0.009 mm (arcs). |
| **B1.** Precharge vs SPICE | 4 circuits, one with the resistor shorted mid-charge | 0.1 % | **Pass, worst 0.001 %** |
| **B2.** Pack over a lap vs SPICE | 520 points, 70 kW peak, 22.5 kW regen | 0.5 % V, 1 % I, 0.1 pp charge | **Pass: 0.0011 % V, 0.0011 % I, 0.0004 pp** |
| **B3.** Net resistance on real boards vs SPICE | 963 nets | 2 % per net | **Miss.** Round 1: 61.7 % within 2 %, worst −98.5 %. Four flaws fixed; after: 72.5 % within 2 %, 91.8 % within 10 %, worst −24 % (same boards) |
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
  1-D resistors; real copper is 2-D, and the two place the current entering a pad or a junction differently. Neither
  is exact. A 2-D copper solve would be the reference that settles it; this benchmark does not have one.

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

- **Altium's own PCB reader** is not benchmarked. Altium can't run here, so there is no ground truth for what an
  Altium file contains beyond the file itself.
- **Trace heating (IPC-2221)** and **fuse and wire I²t** are standards KinematiK implements directly. A tool that
  computes the same equations would only repeat them. Measured heating (IPC-2152 test data) would be the
  meaningful check, and isn't here yet.
- **Net resistance has no 2-D reference.** See B3.
