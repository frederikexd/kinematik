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

Open-source full-car pre-validation platform for Formula SAE and Formula EV teams.
Born as a suspension kinematics tool. Now the engineering operating system for an entire formula car.

> *"Our brakes lead called it a saving grace."*
> — CSULB SAE

---

## What it is

KinematiK is **not** ADAMS. It is **not** ANSYS. It is the hour before them.

The most expensive class of error in motorsport engineering isn't a bad simulation — it's garbage inputs reaching a simulation tool and producing garbage outputs. Teams spend days debugging solvers that were never the problem. The problem was the spreadsheet three steps earlier: disconnected, unvalidated, passed around the team for years, with sign conventions nobody has verified and assumptions that silently contradict each other across subsystems.

KinematiK fixes the step before simulation. Every subsystem decision — suspension geometry, brake bias, accumulator topology, cooling sizing, BOM cost — lives in one connected environment. When the suspension lead changes a parameter that affects brake bias, the brakes lead sees it. When the powertrain output shifts weight distribution, the aero and suspension leads know. The decisions that used to live in disconnected spreadsheets and get lost between meetings now have a single source of truth.

When you commit to an ANSYS run, you are confident the inputs are right. You are not burning a $50,000/seat simulation licence finding a fault that was in the spreadsheet three steps earlier.

> ⚠️ **Always validate outputs with ANSYS, ADAMS, or MATLAB before manufacturing.** KinematiK is a pre-validation tool, not a replacement for full simulation. Every output is a starting point, not a final answer. The tool itself will tell you this.

---

## New: 🧭 Frames & Datums — one convention, whole team, zero ambiguity

Every formula team has had this exact Discord argument:

> *"should i change my model to sae coordinates? … honestly it might be a full redo cause i have a lot of measurements that are plane-specific"*
> *"I was asking how something affected packaging in x and y and someone was like: wait, what are we defining as x and y"*
> *"we won't rly know the center of gravity until the master assembly is completely put together… the chassis changes length sometimes, so relativity to the front axle changes too"*
> *"Idk if judges prefer it"*

Four distinct failures hide in that thread — no declared convention, migration priced as a full redo, origins that drift as the design converges, and nobody able to defend the choice at design judging. **Frames & Datums** (✅ Checks & Integration, shared spine — every subteam sees it) fixes all four:

- **Team convention charter.** Declare one frame — ISO 8855, SAE J670, ISO 4130, the KinematiK internal frame, a typical SolidWorks setup, or a custom frame built from direction words (+z is derived from x × y, so declaring a left-handed frame is mathematically impossible). Saving logs a Decision in the Registry so next year's cohort inherits *why*, and exports a **judge-ready one-page charter**: axis triad, rotation senses, phrasebook, and a one-line answer for the design judge.
- **Floating datum watch.** Front axle, rear axle, mid-wheelbase and CG datums resolve **live** from the vehicle parameters. When the wheelbase stretches or the CG moves, the tab reports exactly how many millimetres each datum drifted since the charter was saved — CG-relative dimensions can't silently rot, so you *can* base designs on a datum that moves, because you're told when it moved.
- **Rosetta.** One point or free vector, shown in every convention simultaneously plus plain English ("585 mm left of centreline, 310 mm above ground"). Paste the *words* into chat, not the bare numbers. Free-vector mode shows the classic sign trap live: a +Z tyre load in SAE J670 is −Z in ISO 8855.
- **Migration wizard — the "full redo" killer.** Convert the live hardpoint set or any `name,x,y,z` CSV between frames *and* datums in one pass, with a per-point audit and a **SolidWorks Curve-Through-XYZ export** so every migrated point lands back in CAD as a sketchable reference. Days of retyping becomes two minutes.
- **Sign-convention linter.** Per-defect findings with fixes: below-ground points (a Z-down import), mirror-pair asymmetry (a Y-left/Y-right flip), metres imported as millimetres, wrong-datum envelope violations.
- **Frame tags on everything leaving the platform.** Every DXF's annotation block, the handover report, and the Integration ledger banner carry the declared convention — a section opened in CAD months later still says which way x/y/z point. If no convention is declared, the handover says **UNDECLARED** out loud instead of silently omitting it.

Rotation senses are *computed* from the frame basis via the right-hand rule, never memorised — which is how the tool knows, and shows, that SAE +yaw is nose-right while ISO +yaw is nose-left, and SAE +pitch is nose-up while ISO's is nose-down. (The hardpoint editor's own header used to mislabel its x-rear/y-right/**z-up** frame as "SAE" — SAE J670 is Z-*down*. Fixed: it's ISO 4130-style, and it says so.)

All frame maths lives in `coordinate_frames.py` — pure Python, importable without Streamlit, self-tested with exact identities (`python3 coordinate_frames.py`). See `FRAMES_DATUMS_USAGE.md`.

---

## New: 🎯 Proof Engine — certainty as a budget, pass criteria sealed before the run

No CAE, PLM, or requirements tool answers the two questions that actually govern the week before ANSYS: *which validation is worth doing first*, and *what counts as a pass, decided before the result exists*. The Proof Engine (✅ Checks & Integration, shared spine) answers both:

- **Quantified uncertainty ledger.** Every declared number carries a ± from a five-step evidence grade — guess ±40 %, estimate ±20 %, modelled ±10 %, measured ±3 %, verified ±1 % — and the band **inflates with age** (a measurement's uncertainty doubles at its staleness half-life, capped at guess). A checkbox can never claim *measured*; only a dated claim with a source can.
- **Variance attribution.** Deterministic one-at-a-time perturbation propagates every band to the objective you pick (lap time, endurance energy, pack thermal margin, mass) and shows which inputs dominate: *"±1.9 s on lap time; 61 % of it is a CG height nobody has measured."* Reproducible by hand — same ledger in, same numbers out, always.
- **The ranked proof plan.** A catalog of evidence actions — corner scales, tilt-test CG, coast-down, dyno pull, flow bench, pack thermal log, strain-gauged mount, an ANSYS study — ranked by **uncertainty retired per hour**. Corner scales can outrank ANSYS, and the arithmetic shows exactly why. The plan exports as a pinnable one-page markdown with the frame charter stamped on it. This is value-of-information planning: the literal list of questions worth asking the expensive tools.
- **🔏 Pre-registered validation contracts.** Borrowed from experimental science and never before shipped in an engineering tool: the acceptance band and criterion are fixed and **sha256-sealed before the run**. Judging fills a result block and never touches sealed fields; edit the band afterward and the seal breaks — and a broken seal refuses judgment out loud. "FoS 1.05 is probably fine" can never be decided after seeing the result.
- **The three-way verdict.** Every contract carries a plausibility envelope (prediction ± 3σ from the ledger — computed, not chosen). PASS inside the band. FAIL outside the band but plausible — a design finding, caught before the first cut. **DISCREPANT** outside the envelope — the run and the ledger disagree about reality, so neither number is acted on until units, frame, BCs, and geometry version are audited. A failed design and a garbage run finally stop looking the same.

All of it lives in `suspension/proof_engine.py` — pure Python, headless, self-tested (`python3 -m suspension.proof_engine`), with the UI in `ui/proof_planner.py` as the first tab under the new `ui/` module pattern. See `docs/BOTTLENECKS.md` for the full prevalidation bottleneck map this feature closes.

---

## New: 🧨 Saboteur — mutation testing for the input deck. Which errors would you fail to notice?

The Proof Engine's DISCREPANT verdict catches the garbage run that looks *impossible*. That leaves the deadliest class untouched: **the garbage that looks fine**. A pounds-into-kg slip on one subsystem, the kilo prefix slipping on a heat load, a Z-down hardpoint sheet in a Z-up deck, one subsystem silently missing from the mass roll-up — each can move the answer by an amount that sits comfortably inside the plausibility envelope. The run comes back, the number is believable, the sealed contract says PASS, and the team acts on it. Nobody audits a result that confirms what they expected.

Software engineering solved the mirror-image problem decades ago: **mutation testing** — deliberately inject known bug classes, see which ones the test suite fails to notice, and you know exactly where its holes are. No CAE, PLM, or requirements tool has ever pointed that idea at an engineering input deck. The Saboteur (✅ Checks & Integration, shared spine) does:

- **The sabotage sweep.** Ten catalogued corruption classes — each one a documented, real failure from the bottleneck map (unit thousandfold slips, inches-into-mm, lb-into-kg, lb·ft-into-N·m, frame Z flips, dropped and double-counted roll-up terms) — are injected one at a time into a shadow copy of the uncertainty ledger. For every (corruption, target) pair the sweep asks: *would anyone notice?* On a representative FSAE-EV deck, the answer is brutal: **only ~8 % of catalogued corruptions push the result outside its own 3σ envelope.** The other 92 % would come home from ANSYS wearing a plausible face.
- **Tripwires chosen by arithmetic, not folklore.** A tripwire is a cheap checksum recorded *alongside* the run — rolled-up mass from the mesher's printout, CG height sign, torque-per-power (implied motor base speed), implied pack voltage, heat-loss fraction. The distinction that makes them work: a tripwire compares the run against **the deck**, not against reality. However uncertain the declared numbers are about the real car, a solver that consumed the declared deck must reproduce the deck's own arithmetic to a tight consistency tolerance — so the wires stay sharp precisely when the deck is most uncertain, which is when garbage is most likely. A greedy set-cover picks the fewest wires that expose the most silent corruptions; four wires typically take detection from ~8 % to **100 %** of the catalog.
- **Sealed like a contract.** The wire set, expected values, and bands are sha256-sealed before the run. A skipped tripwire is not a passed one; an edited sheet refuses to judge.
- **The garbage names itself.** When readings come back and a wire trips, the deviation pattern is matched against every predicted corruption signature (cosine similarity on band-normalised deviations — deterministic, checkable by hand). The verdict is not "something is wrong" but *"this signature matches pounds-into-kg on the accumulator mass, magnitude 1.0× predicted."* The audit that used to eat an evening starts with a named suspect. A pattern matching *nothing* in the catalog says so honestly instead of naming a false suspect.
- **Honest blind spots.** Any corruption invisible to the result *and* to every available wire is listed out loud, with the only remaining defence named (measure that input directly) — and the coverage number charges for it. A cap-shortened sheet charges its truncation victims the same way. No unearned green boards, including this tab's own.

Why no one has built it: a tool that tells you which of a solver's answers would be undetectably wrong is a tool no solver vendor will ever ship. And it costs the team **zero new data entry** — it reuses the exact uncertainty ledger the Proof Engine already maintains.

All of it lives in `suspension/saboteur.py` — pure Python, headless, self-tested (`python3 -m suspension.saboteur`), UI in `ui/saboteur.py`. See bottleneck **#12** in `docs/BOTTLENECKS.md`.

---

## Coverage

One environment. Every subsystem. The entire car.

| Subsystem | What KinematiK does |
|---|---|
| **Suspension / Dynamics** | 3D constraint solver, camber gain, bump steer, roll centre migration, load transfer, grip balance, compliance, setup optimiser, GGV, transient, upright mount-plate DXF |
| **Aerodynamics** | Downforce & ground effect, wing/diffuser sizing, aero map, virtual wind tunnel, wing-section (airfoil) DXF |
| **EV Powertrain** | Motor architecture comparison, energy budget, regen, lap time, torque vectoring, motor-flange DXF |
| **Accumulator** | Cell sizing, pack topology, FSAE-EV rules checks, thermal model, electrical feasibility gate, segment-box DXF |
| **Brakes** | Bias & lock-up, hydraulic sizing, bolt & bracket FoS, rotor thermal, fade test, rotor optimiser + rotor DXF export, caliper-bracket DXF |
| **Chassis / Frame** | 3D model, team fit, weight & CG ledger, handover export, node-gusset DXF, **Frame Planner** (node/tube frame graph with 3D wireframe, triangulation & load-path audit with per-defect fixes, Size C→B sourcing trade study, alternative-tubing equivalency screen, panel & attachment planner for seat/harness/floor/firewall/aero panels) |
| **Cooling** | Thermal sizing, heatmap, cross-subsystem heat propagation, radiator-core DXF |
| **Electronics** | PCB copper survival, signal integrity, HV/LV checks, **PCB Doctor** (import a real `.kicad_pcb`, diagnose real-life failures with the guilty component named, one-click re-trace of under-sized copper, multi-layer Trace Prescriber), sensor/PCB-bracket DXF |
| **Data Acquisition** | Integration with car-level electrical budget, DAQ-bracket DXF |
| **Cost & BOM** | FSAE Cost event, auto-seeded from Integration ledger, CSV export |
| **Integration** | Cross-subsystem ledger, coupling graph, risk propagation, manufacturing-release gate, **Verdict Center** (per-subsystem works / look-closer / attention) |
| **Frames & Datums** | Team coordinate convention charter, live floating datums with drift watch, frame Rosetta, migration wizard with SolidWorks XYZ export, sign-convention linter, judge-ready charter export, frame tags on every DXF / handover / ledger |
| **Proof Engine** | Quantified evidence grades with staleness decay, deterministic uncertainty attribution to lap time / energy / thermal / mass, evidence actions ranked by uncertainty retired per hour, sha256-sealed pre-registered validation contracts, PASS / FAIL / DISCREPANT verdicts |
| **Saboteur** | Mutation testing for the input deck: ten catalogued corruption classes injected into a shadow ledger, silent-killer detection, tripwire checksums picked by deterministic detectability set-cover, sha256-sealed pre-flight sheets, corruption fingerprinting that names the garbage class from the tripped pattern |
| **DFMEA** | Live failure mode analysis, pre-seeded rows, RPN recompute, action tracker |
| **Registry** | Component source of truth, version history, sign-off, CAD provenance parsing |

---

## The one idea

**One car, not eight tools.**

Every subsystem declares what it weighs, draws, rejects and provides into a single **Integration ledger**. That one source feeds the 3D model, the lap sim, the heatmap and the cost BOM. Declare a number once and it propagates everywhere — the eight "we're ~12 kg" estimates can't quietly sum to 18 kg over the number suspension tuned to.

And now every declared number carries a **frame tag**: the Integration ledger banner states the team's coordinate convention (or nags until one is declared), because a number without a frame is exactly the kind of unvalidated input this whole platform exists to prevent.

When any subsystem saves an interface edit, KinematiK walks the change through a coupling graph and shows — unprompted — which other subsystems' risk just moved. Bump the motor torque and you immediately see it load the upright and heat the cooling loop. Every effect carries an honest confidence tag: **measured** (a solver ran), **coupled** (a modelled physical edge), or **judgement** (engineering judgement, no backing physics). A measured edge is demoted if the data behind it is still an estimate. A green board never overstates what is known.

---

## Four moves to start

0. **Answer the mission briefing.** The landing screen asks four one-tap questions — *what subteam(s) are you on? what are you using KinematiK for? what's the goal? are you a visual thinker?* — and compiles a personal plan: exactly which tools to open, in what order, why you need each one, and why to do it here first so ANSYS / MATLAB / OptimumK only ever **validate** your design instead of debugging your inputs. Every question has a sensible default, so a complete beginner can tap through in seconds, and everything is skippable. Visual thinkers (and anyone brand new) get a live, physically accurate concept graph or 3D render under each recommended tool; newcomers also get a plain-English line per tool. Answering also picks your subteam, so you then see only your tabs plus the shared spine (Integration, Frames & Datums, Validation, Analytics, Registry, Notes, 3D Model), grouped into five simple categories (Testing, Design, Checks, Docs, Data) — never all 25 at once. Skipped or dismissed the briefing? A one-tap **🧭 Get my mission briefing** button brings it back any time.
1. **Declare your coordinate convention.** In **Checks → 🧭 Frames & Datums**, pick the team frame and master datum (30 seconds). Every DXF, handover and ledger number is stamped with it from that moment; the migration wizard converts anything you already have.
2. **Declare your interface.** In **Integration**, fill what your subteam owns (mass, CG, torque, heat, current, downforce) and untick *estimate* once a number is real. Everything downstream uses it.
3. **Watch it ripple, then clear the cut.** KinematiK walks your change through the coupling graph and flags which other subsystems' risk just moved. Before a part goes to manufacture, run the **manufacturing-release gate** — a literal go/no-go that blocks any part still resting on an estimate or an unconfirmed load.

### Get a build-ready DXF (no CAD needed to start)

Every subsystem exports the real 2-D section it takes into CAD — a wing airfoil, a mount/flange plate with bolt holes, a radiator core face — built from *your* computed numbers. In your subsystem tab, open its own **"📐 … — mesh & DXF export"** panel (it sits just below the documentation panel, mirroring the Brakes tab's inline rotor export), pick a section, and download. In SolidWorks: **File ▸ Open ▸ DXF ▸ import as 2D sketch**, extrude, then mesh in ANSYS. Units are embedded, every profile is checked to import as one clean closed contour, and the annotation block states the team's declared coordinate convention.

---

## Positioning

KinematiK sits between your team's engineering decisions and your simulation budget.

```
Team decisions → KinematiK (pre-validation) → ANSYS / ADAMS / MATLAB (verification) → Manufacturing
```

It does not replace simulation. It makes simulation more valuable by ensuring the inputs that reach it are organised, connected, and pre-validated. The sim becomes a verification of a number you already trust — not the place you discover it.

---

## Pricing

**Free for students and FSAE / Formula Student teams. Always.**

KinematiK is free for any student or university team, permanently. The student community is not the revenue model — it is the distribution model. Every FSAE graduate who used KinematiK and joins a professional team is a warm introduction to that team, not a lost customer.

Professional teams, consultancies, and enterprises: contact for pricing.

---

## Usage stats

Usage numbers live in one place: the in-app **Analytics** tab, computed live from the database as *lifetime = pre-purge baseline snapshot + current 30-day window*. They are deliberately not hand-copied into this README — a stat printed here goes stale the moment it's written, and two documents disagreeing about the same metric is exactly the class of error this platform exists to prevent.

The one number worth stating in prose: roughly **half of all users come back** without any retention mechanism, reminder emails, or onboarding. Students are brutally honest users. If it is not useful, they close the tab and never return.

---

## Architecture

**Kinematics engine** — architecture-agnostic multibody solver (`suspension/topology.py`). Rigid bodies defined by points, constraint primitives (distance links, ball/pin coincidence, prismatic slider, planar, revolute, rack translation, beam-axle roll), assembled into a `Mechanism` and solved by branch-stable Levenberg–Marquardt sweep.

**Topology library** (`suspension/topologies.py`) — double wishbone, MacPherson strut, multi-link (3/4/5-link), trailing arm, semi-trailing arm, solid axle (Panhard or Watts), twist-beam, truck steer linkage, and `from_links` for experimental corners.

**Vehicle dynamics layer** — roll-centre migration, anti-dive/anti-squat, load transfer, grip balance, all topology-independent via `GenericKinematics` adapter (`suspension/adapter.py`).

**Coordinate frames** (`coordinate_frames.py`) — pure-Python frame registry and transform core. Every conversion routes `frame A → world → frame B` through one auditable path; all frames are proper rotations (det = +1), so points, forces, moments and angular rates share one transform and only points shift by the datum. Rotation senses are derived from the basis via the right-hand rule. Datums resolve live from the vehicle parameters (`a = L·(1 − weight_dist_front)` from static axle-load balance). Self-tested with exact identities, no fuzz: `python3 coordinate_frames.py`.

**Analytics** (`suspension/analytics.py`) — privacy-respecting usage tracking. Identity is a random per-session UUID (plus a browser cookie for return-visit counting); no IP addresses or device fingerprints are collected or stored. A member name is recorded only if the user types one in (opt-in). Telemetry never blocks the UI and a telemetry failure can never crash the app. Only three event types are written (session start, workflow complete, error); raw events are purged after 30 days.

---

## Database setup

Run `suspension/analytics_hardening.sql` in Supabase once. Safe to re-run (drop-then-create, idempotent grants). This creates all analytics views including the fixed `v_retention` and `v_time_to_first_result`.

For the per-feature funnel fix only, run `fix_feature_funnel.sql` standalone.

---

## Deploy order

1. Push `streamlit_app.py`, `project.py` and `coordinate_frames.py` together — the handover builder gained a `frame_tag` parameter that the app passes, so they are a matched set.
2. Push `suspension/analytics.py` with `streamlit_app.py` as before — still a matched pair.
3. Run `suspension/analytics_hardening.sql` in Supabase.
4. Confirm build stamp in the Usage section reads `0.24.0-saboteur` and streamlit runtime reads `>= 1.58.0`.

---

## What changed in this build (`0.24.0-saboteur`)

**🧨 Saboteur — new shared-spine tab (Checks & Integration)**
- New `suspension/saboteur.py`: mutation catalog (thousandfold/imperial unit slips, frame Z flip, kilo-prefix slips, dropped & double-counted roll-up terms, each with its real-world story), sabotage sweep over a shadow copy of the uncertainty ledger, silent-killer classification against the objective's own 3σ envelope, tripwire catalog with per-wire deck-consistency tolerances and real-world read instructions, greedy detectability set-cover (runs until nothing catchable remains; a caller-imposed cap charges its victims to the blind-spot list), sha256-sealed pre-flight sheets, cosine fingerprinting of tripped patterns against predicted corruption signatures, honest-blind-spot reporting, markdown export. Pure stdlib, deterministic end to end, self-tested.
- New `ui/saboteur.py` under the `ui/` strangulation pattern: kill board, coverage before/after, sheet sealing, judge-a-run panel with named suspects. Shares the `proof_pedigree` session map with the Proof Planner — one pedigree, two consumers, on purpose. Can optionally judge the sweep against an open validation contract's acceptance band, answering: *could garbage hand you a PASS?*
- `tests/test_saboteur.py`: 22 tests pinning determinism, per-class detection (z-flip caught by the CG wire, lb·ft by torque-per-power, dropped terms by rolled-up mass), honesty (blind spots reported and charged, unavailable wires never offered, verified pedigrees never shrink tripwire tolerances), seal tampering refused, fingerprint correctness (injected corruption identified at cosine > 0.99, magnitude 1.0×), and uncatalogued errors admitted rather than misattributed.
- `docs/BOTTLENECKS.md` gains bottleneck **#12 — the garbage that flatters the envelope**, closing the residual #8 left open.

*(Previous build `0.23.0-frames` below.)*

## What changed in build (`0.23.0-frames`)

**🧭 Frames & Datums — new shared-spine tab (Checks & Integration)**
- New `coordinate_frames.py`: frame registry (ISO 8855, SAE J670, ISO 4130, KinematiK internal, SolidWorks-typical, custom-from-words with derived +z guaranteeing right-handedness), exact point/vector/rotation-sense transforms via one `frame → world → frame` path, floating datums resolved live from wheelbase / weight split / CG height, datum-drift detection, CSV + SolidWorks Curve-Through-XYZ I/O, sign-convention linter (below-ground / mirror asymmetry / unit sniff / envelope), judge-ready charter markdown. Pure Python, streamlit only imported inside `render()`, exact-identity self-tests.
- Tab body wired with hard isolation (`try/except`) so the convention tool can never take the studio down; live hardpoint provider filters the session hardpoint dict to 3-vectors and maps keys to human labels.
- Declaring a charter logs a Decision (`team=integration`, tags `coordinates,standard`) so the convention and its rationale survive into the Registry and next season's handover.

**Frame tags on everything leaving the platform**
- `_generic_dxf_bytes` annotation block now stamps the declared convention onto every generic DXF (aero sections, mount plates, radiator faces, brackets, gussets); the brake-rotor DXF (which builds its own R12 file) stamps the same line.
- `project.build_handover_markdown` gained `frame_tag=""` and renders a **Coordinate convention** section before the weight budget; the app passes the long-form tag, which explicitly reads **UNDECLARED** when no charter exists — formal documents never silently omit the convention.
- Integration tab banner states the declared convention above the ledger, or nudges to declare one.

**Hardpoint editor mislabel fixed**
- The editor header claimed "SAE x-rear y-right z-up". SAE J670 is Z-**down**; the internal frame is ISO 4130-style. Header corrected and now points to Frames & Datums for conversion — the tool no longer commits the exact mislabel the tab was built to end.

*(Previous build `0.22.0-unified` — team CAD library ⇄ 3D model quick-assembly preview, mission briefing onboarding, `v_retention` two-phase identity rewrite, `v_time_to_first_result` anchor fix, visitor identity fixes, `session_start` deferral — see Git history for the full notes.)*

---

## IP and attribution

KinematiK is the original work of Frederik Thio, developed independently as a personal project. Development history is timestamped in the Git commit log.

All outputs are for design direction. Always validate with full simulation (ANSYS, ADAMS, MATLAB) before manufacturing. This is not a suggestion — it is the entire point of the tool.

---

## License

AGPL-3.0. Free to use, fork, and build on. Any modifications must be shared under the same license.

© 2026 Frederik Thio
