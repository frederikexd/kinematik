<!--
  KinematiK — Formula SAE / Formula EV full-car pre-validation platform
  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
  Open source. Original author: Frederik Thio, creator of KinematiK.
-->

---
title: KinematiK
emoji: 🏎️
colorFrom: yellow
colorTo: gray
sdk: streamlit
sdk_version: 1.58.0
app_file: streamlit_app.py
pinned: false
license: agpl-3.0
---

# ◢ KinematiK

**KinematiK gets you to the right question. ANSYS gives you the right answer.**

The engineering operating system for Formula SAE and Formula EV teams.

> *"Our brakes lead called it a saving grace. The numbers match ANSYS and MATLAB."*
> — CSULB SAE

---

## The problem it solves

The most expensive error in motorsport engineering isn't a bad simulation. It's garbage inputs reaching a simulation tool and producing garbage outputs that nobody catches until manufacturing.

Your team runs eight subsystems. Each one lives in its own spreadsheet, maintained by a different person, with sign conventions nobody has fully verified and assumptions that silently contradict each other across subteams. The suspension lead tunes to a weight that doesn't match what the chassis lead declared. The accumulator thermal model uses a cooling capacity the cooling subteam revised three weeks ago. Nobody is wrong. The system has no single source of truth.

KinematiK fixes the step before simulation. Every subsystem decision lives in one connected environment. When the suspension lead changes a parameter that affects brake bias, the brakes lead sees it. When powertrain output shifts weight distribution, aero and suspension know. When any number changes, KinematiK walks it through a coupling graph and shows — unprompted — which other subsystems just moved.

When you commit to an ANSYS run, you are confident the inputs are right. You are not burning simulation time finding a fault that was in the spreadsheet three steps earlier.

> ⚠️ **Always validate outputs with ANSYS, ADAMS, or MATLAB before manufacturing.** KinematiK is a pre-validation tool, not a replacement for full simulation. The tool itself will tell you this.

---

## What it covers

One environment. Every subsystem. The entire car.

| Subsystem | What KinematiK does |
|---|---|
| **Suspension / Dynamics** | 3D constraint solver, camber gain, bump steer, roll centre migration, load transfer, grip balance, compliance, setup optimiser, GGV, transient, upright mount-plate DXF |
| **Aerodynamics** | Downforce & ground effect, wing/diffuser sizing, aero map, virtual wind tunnel, wing-section DXF |
| **EV Powertrain** | Motor architecture comparison, energy budget, regen, lap time, torque vectoring, motor-flange DXF |
| **Accumulator** | Cell sizing, pack topology, FSAE-EV rules checks, thermal model, electrical feasibility gate, segment-box DXF |
| **Brakes** | Bias & lock-up, hydraulic sizing, bolt & bracket FoS, rotor thermal, fade test, rotor optimiser, rotor DXF + caliper-bracket DXF |
| **Chassis / Frame** | 3D model, team fit, weight & CG ledger, handover export, node-gusset DXF |
| **Cooling** | Thermal sizing, heatmap, cross-subsystem heat propagation, radiator-core DXF |
| **Electronics** | PCB copper survival, signal integrity, HV/LV checks, sensor/PCB-bracket DXF |
| **Data Acquisition** | Integration with car-level electrical budget, DAQ-bracket DXF |
| **Cost & BOM** | FSAE Cost event, auto-seeded from Integration ledger, CSV export |
| **Integration** | Cross-subsystem ledger, coupling graph, risk propagation, manufacturing-release gate, Verdict Center |
| **DFMEA** | Live failure mode analysis, pre-seeded rows, RPN recompute, action tracker |
| **Registry** | Component source of truth, version history, sign-off, CAD provenance parsing |

---

## The one idea

**One car, not eight tools.**

Every subsystem declares what it weighs, draws, rejects, and provides into a single Integration ledger. That one source feeds the 3D model, the lap sim, the heatmap, and the cost BOM. Declare a number once and it propagates everywhere — the eight "we're approximately 12 kg" estimates can't quietly sum to 18 kg over the car the suspension was tuned to.

Every propagated effect carries an honest confidence tag: **measured** (a solver ran), **coupled** (a modelled physical edge), or **judgement** (engineering judgement, no backing physics). A measured edge is demoted if the data behind it is still an estimate. A green board never overstates what is known.

Before a part goes to manufacture, the **manufacturing-release gate** gives a literal go/no-go. It blocks any part still resting on an estimate or an unconfirmed load. No part leaves without a clean board.

---

## Get a build-ready DXF in three clicks

Every subsystem exports the real 2-D section it takes into CAD — a wing airfoil, a mount plate with bolt holes, a radiator core face — built from your computed numbers, not redrawn from memory.

In your subsystem tab, open the **📐 mesh & DXF export** panel, pick a section, and download. In SolidWorks: **File ▸ Open ▸ DXF ▸ import as 2D sketch**, extrude, then mesh in ANSYS.

Units are embedded. Every profile is checked to import as one clean closed contour. The geometry that came out of the solver goes directly into CAD. No retyping. No redrawing. No transcription errors.

---

## Where it sits

```
Team decisions → KinematiK (pre-validation) → ANSYS / ADAMS / MATLAB (verification) → Manufacturing
```

KinematiK does not replace simulation. It makes simulation more valuable by ensuring the inputs that reach it are organised, connected, and already checked. The sim becomes a verification of a number you already trust — not the place you discover it.

---

## Three moves to start

1. **Pick your subteam.** Nothing opens until you choose who you are. You see only your subteam's tabs plus the shared spine — never all 25 at once.
2. **Declare your interface.** In Integration, fill what your subteam owns and untick *estimate* once a number is real. Everything downstream uses it.
3. **Watch it ripple.** KinematiK walks your change through the coupling graph and flags which other subsystems just moved.

---

## Pricing

**Free for students and FSAE / Formula Student teams. Always.**

The student community is not the revenue model — it is the distribution model. Every FSAE graduate who used KinematiK and joins a professional team is a warm introduction to that team, not a lost customer.

Professional teams, consultancies, and enterprises: contact for pricing.

---

## Adoption

Used across Formula SAE, Formula EV, and FSAE Baja teams. Spread organically through the SAE Discord without onboarding, documentation campaigns, or retention mechanisms.

- **567 total users** across SAE student teams
- **51% return rate** — 291 of 567 came back without being asked
- **11 seconds** to first result for a new member
- **18 days** of recorded traffic

The brakes subsystem at CSULB SAE ran KinematiK outputs against ANSYS and MATLAB in parallel and confirmed the numbers match. That was not a requested validation. They trusted it enough to check.

---

## Architecture

**Kinematics engine** — architecture-agnostic multibody solver (`suspension/topology.py`). Rigid bodies defined by points, constraint primitives (distance links, ball/pin coincidence, prismatic slider, planar, revolute, rack translation, beam-axle roll), assembled into a `Mechanism` and solved by branch-stable Levenberg–Marquardt sweep.

**Topology library** (`suspension/topologies.py`) — double wishbone, MacPherson strut, multi-link (3/4/5-link), trailing arm, semi-trailing arm, solid axle (Panhard or Watts), twist-beam, truck steer linkage, and `from_links` for experimental corners.

**Vehicle dynamics layer** — roll-centre migration, anti-dive/anti-squat, load transfer, grip balance, all topology-independent via `GenericKinematics` adapter (`suspension/adapter.py`).

**Analytics** (`suspension/analytics.py`) — privacy-respecting usage tracking. Durable identity via IP+UA fingerprint. All tracking is anonymous. No personal data stored.

---

## Database setup

Run `suspension/analytics_hardening.sql` in Supabase once. Safe to re-run (idempotent). Creates all analytics views including `v_retention` and `v_time_to_first_result`.

---

## Deploy order

1. Push `streamlit_app.py` and `suspension/analytics.py` together — matched pair.
2. Run `suspension/analytics_hardening.sql` in Supabase.
3. Confirm build stamp reads `0.12-analytics-hardened` and streamlit runtime `>= 1.58.0`.

---

## IP and attribution

KinematiK is the original work of Frederik Thio, developed independently as a personal project. Development history is timestamped in the Git commit log.

All outputs are for design direction. Always validate with full simulation before manufacturing. This is not a suggestion — it is the entire point of the tool.

---

## License

AGPL-3.0. Free to use, fork, and build on. Any modifications must be shared under the same license.

© 2026 Frederik Thio
