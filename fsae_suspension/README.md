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

## New: 👻 Phantom Car — the margin audit. Nobody has ever added your conservatism up.

The Proof Engine prices what the team doesn't know. The Saboteur catches a deck that lies. That leaves the third failure mode — the one that makes formula cars fat and DNFs endurance anyway: **every subsystem hedges the same uncertainty separately, in secret, and nobody adds it up.** The brakes lead quietly sizes for the car "if it comes in heavy." The structures lead takes a worst-case load *and* stacks FoS 1.5 on top — margin on margin, on a number whose evidence grade was GUESS, so the bracket is defending a 4σ statistical event and the mass bill for defending that impossible car lands on the real one. Meanwhile the energy budget — the number that actually ends an endurance — consumes the *same mass* at its optimistic target value, naked. The deck now describes at least two mutually exclusive cars, everyone believes they were prudent, and the total conservatism of the design has never once been a number. Aerospace primes run staffed margin-management processes for exactly this; no CAE, PLM, or requirements tool computes it, and nothing a student team can afford even names it. The Phantom Car (✅ Checks & Integration, shared spine) does:

- **Disclosure, not new work.** Each consumer of a deck number states the design value its sizing *actually uses* — a number that already lives in its spreadsheet — plus any factor applied on top. A built-in FSAE-EV consumption map seeds the form; seeds start at nominal on purpose, so a fresh audit says NAKED where nobody has disclosed cover instead of fabricating prudence.
- **Every hedge priced in the deck's own currency.** "Assumes 250 kg" is opinion; "hedged **+2.1σ** on an *estimate*-grade mass" is arithmetic. The σ pricing every hedge is the exact evidence-graded, staleness-inflated band the Proof Engine already maintains — one ledger, third consumer, zero new physics.
- **One sealed Margin Charter.** The team declares a single design percentile for the whole car — *we design to the 95th-percentile car* — sha256-sealed like a validation contract. Every disclosure is judged against it: **ALIGNED**, **STACKED** (the excess priced as releasable envelope in the quantity's own units), **UNDER-COVERED**, **NAKED** (naming the evidence grade it's naked on), **ANTI-HEDGED** (designing to a car the ledger says doesn't exist — called out even when a fat FoS papers over it). Edit the sealed percentile afterward and the audit refuses to judge, out loud.
- **The two-cars detector.** Assumed design values more than 1σ apart on the same quantity mean the deck provably describes more than one car — brakes stopping 250 kg while the energy budget feeds 228 — and the audit names both consumers and the width of the disagreement. The contradiction the Integration ledger kills for *values*, applied for the first time to *assumptions*.
- **β — the improbability each load case defends against.** A consumer stacking worst cases on several inputs is designing to their joint worst case; β = √(Σz²) is the same first-order reliability index (FORM) professional reliability engineering uses, computed from the σ your evidence grades already imply, odds in English: *"this bracket load case is a 1-in-2,300,000 car."*
- **The three cars.** Per objective — lap time, endurance energy, thermal margin, mass — the audit evaluates the **nominal** car, the **coherent** charter-percentile car, and the **phantom**: every channel at the most adverse value any consumer assumed, the union of everyone's private fears. The gap is the design envelope currently spent defending cars the deck's own σ says are statistically impossible — reported honestly as *envelope*, never as promised savings, because releasing it is a design decision and pricing it is this tab's job. Undisclosed consumers are listed as unaudited blind spots, never absorbed into a green board.

Why no one has built it: margin stacking is invisible to every tool in the chain *by construction* — a solver sees one load case at a time and cannot know it was hedged upstream, PLM sees files, requirements tools see targets. The information needed to add margins up (the numbers, their σ, and who consumes them) has never lived in one system before. In KinematiK it already does, so the feature is a join, not a data-entry burden.

All of it lives in `suspension/phantom_car.py` — pure Python, headless, self-tested (`python3 -m suspension.phantom_car`), UI in `ui/phantom_car.py`. See bottleneck **#13** in `docs/BOTTLENECKS.md`.

---

## New: 🎙️ Earshot — the test-day power audit. Can the session even hear the answer?

The Proof Engine says *which* test is worth doing. The Saboteur guards the deck it feeds. The Phantom Car audits the hedges around it. Earshot asks the question every one of them skips — the one clinical trials have refused to start without for fifty years and no engineering tool has ever asked of a test day: **as planned, is the answer within earshot?**

Three ways a team's scarcest resource dies quietly, all three lived by every team:

- **The deaf A-B test.** The predicted wing gain is 0.3 s; the driver's lap-to-lap σ is 0.8 s. Detecting that at 80 % power needs **112 laps per configuration** — the arithmetic is two lines, and nobody runs it. The pack holds 40 laps. The session was dead at breakfast, and the inevitable "inconclusive" gets read as *"the wing doesn't work"* — a real gain, falsely buried, which is worse than not testing. Earshot computes laps-needed, the **minimum detectable effect** of the session actually booked ("20 laps per config can hear 0.71 s, not your 0.30 s"), and the miss probability if you run it anyway — with the lap budget derived from the pack itself (usable kWh over kWh-per-lap: the EV test plan is spent in the same currency the race is). Verdicts: **RESOLVABLE / UNDERPOWERED / SWAMPED**.
- **The confounded run order.** Tires wear, the track rubbers in, the pack sags. Run all the A laps then all the B laps — one wing swap, so it's what tired teams do — and a 0.03 s/lap tire drift plants a **0.6 s bias** in the comparison: twice the hunted effect. The ordering audit computes the exact bias a linear drift injects into AABB / ABAB / ABBA (mean lap index of A minus B, times drift — checkable on a napkin) next to the swap cost each ordering pays, declares **CONFOUNDED** when drift alone rivals the effect, and reserves burn-in laps that count for nobody, because driver learning is the steepest drift of all.
- **The measurement that teaches nothing.** A tilt test at 8° with a half-degree protractor puts ±6 % on CG height from the angle term alone — the 1/(sin θ·cos θ) partial says so, and shows why 20° works where 8° can't. Instrument propagation states the band each parameter test will *actually deliver* — and therefore the evidence grade it **EARNS**: the Proof Engine's promised MEASURED ±3 % is now earned by arithmetic, never claimed by a checkbox. A plan whose delivered band can't beat the ledger's current band is called **MOOT** before anyone loads the trailer, with the dominant error term named so the plan can be fixed instead of abandoned.
- **The sealed session sheet.** δ, σ (with σ's own evidence grade stated), α, power, run order, laps, burn-in, abort criterion, and the MDE — sha256-sealed before the trailer loads, exactly like a validation contract. A shortened session judges **VOID** instead of quietly widening its own goalposts; a **NOT-DETECTED** comes back carrying the sealed probability that a real effect hid — absence of evidence, priced, never shrugged. The sheet exports as a pinnable markdown run order (`ABBAABBA…`) with the frame charter stamped on it.

Why no one has built it: the power analysis lives in statistics packages that have never heard of a car; datalogger vendors profit from sessions run, not sessions cancelled; CAE vendors sell solver hours. The a-priori question needs the predicted effect, the driver's noise floor, the pack's lap budget, and the current uncertainty bands **in one place** — in KinematiK they already are, so Earshot is a join, not a data-entry burden: one new number (driver σ, itself measurable from ten baseline laps) buys the whole audit.

All of it lives in `suspension/earshot.py` — pure Python, headless, self-tested (`python3 -m suspension.earshot`), UI in `ui/earshot.py`. See bottleneck **#14** in `docs/BOTTLENECKS.md`.

---

## New: ⛓️ Fusebox — the failure-order audit. When something must break, does the car choose what?

The Proof Engine prices what you don't know. The Saboteur catches a deck that lies. The Phantom Car totals the hedges. Earshot checks the test can hear. Fusebox asks the first question about the **physical** car that none of them — and no tool in the industry — has ever asked: a big enough hit *will* break something on every load chain; **which element goes first, and did anyone choose it?**

Electrical engineering made choosing the victim a design act 150 years ago: the fuse — cheap, sacrificial, stocked in the box — is *designed* to die so nothing expensive does. Mechanical load paths on a formula car choose by accident, and worse: under the deck's own evidence-graded σ, the order isn't even determined. A MODELLED FoS 1.35 tie rod against a GUESS-grade FoS 1.8 upright is not "tie rod first" — price both capacities with the bands the Proof Engine already maintains and **the upright loses the race roughly one curb strike in four**, silently swapping a $45 rod-end afternoon for a $900, six-week, competition-ending billet part. On an EV it escalates from lead time to safety: the accumulator container and cell restraint must be *last* in every ordering — a claim believed by construction and verified by nobody. Fusebox (✅ Checks & Integration, shared spine) makes the ordering a computed, sealed, judged design object:

- **The pecking order.** Each declared overload chain (curb strike, wing/cone strike, tow yank, side load into the accumulator bay — four seeded archetypes, fully editable) gets P(fails first) per element from the first-order statistics of the minimum of independent normal capacities: mean = the element's FoS at the load *it* sees, σ = FoS × the exact evidence-graded, staleness-inflated band law the Proof Engine assigns that grade — one pedigree law, **fifth consumer**, zero new physics. Deterministic fixed-grid quadrature; for two elements it collapses to Φ((μⱼ−μᵢ)/√(σᵢ²+σⱼ²)), checkable on a napkin.
- **Verdicts against a sealed Fuse Charter.** The team designates the intended fuse per path and one confidence for the car, sha256-sealed. Each path judges **FUSED** / **COIN-FLIP** (the ordering is undetermined at the deck's own σ — the contenders are named with their odds) / **INVERTED** (a structural part is the likely victim, priced in $ and days against the intended fuse) / **UNFUSED** (no fuse-grade element exists on the chain at all) / **BREACH-RISK** — any forbidden (S3) element carrying more first-failure probability than the sealed tolerance, and this verdict outranks every other.
- **Fix arithmetic, not fix folklore.** For every rival threatening the designated fuse, three levers solved *exactly* from the pairwise formula: soften the fuse to the printed FoS (floored at 1.10 — a fuse that pops at 1.0 pops in normal driving), stiffen the rival to the printed FoS, or **sharpen the rival's evidence grade** — the lever no redesign meeting ever tables. A strain-gauged pull test on the upright can buy the same ordering certainty as three weeks of re-machining, because half the coin flip was never mechanics — it was a GUESS-grade band doing the flipping. When a rival's band grows as fast as any FoS you add, the tool says so: *no amount of metal fixes an unknown.*
- **The overload bill.** Conditional on the hit landing: Σ P(first) × cost, and the same in days of downtime, next to the bill of the intended fuse. The gap is the price of the unmanaged pecking order — an expected value, stated as conditional on the event, never a promised saving.
- **Incident judging — the free datum.** When something actually breaks, the sealed charter judges **AS-DESIGNED / SURPRISE / BREACH**, and every verdict banks the one consolation prize of a breakage: reality just measured that element's capacity — re-grade it and the whole pecking order sharpens at zero cost. An edited charter refuses to judge, out loud.

Why no one has built it: fuse coordination is a solved discipline in electrical protection and a staffed frangibility process at aerospace primes, and it exists in no tool a student team can afford — because the ordering needs every element's capacity, the σ its evidence quality implies, its cost and lead time, and the map of which elements share a path, *joined*. A solver sees one part per run by construction. In KinematiK the σ law, the costs, and the chain already live together, so Fusebox is a join plus one honest declaration per path.

All of it lives in `suspension/fusebox.py` — pure Python, headless, self-tested (`python3 -m suspension.fusebox`), UI in `ui/fusebox.py`. See bottleneck **#15** in `docs/BOTTLENECKS.md`.

---

## New: 👻🔩 Ghost Topology — the geometry the car actually has mid-event, and whether it's sabotaging the geometry you drew

Every kinematics solver in the industry runs the links **rigid**, exports one hand-picked static load case to FEA, and stops. Nobody closes the loop where the deflected part changes the geometry, the changed geometry changes the tyre force, and the changed force changes the deflection — because closing it the honest way means co-simulating nonlinear FEA against multibody dynamics, an enterprise licence and a workstation that melts. So a student team throws a safety factor at one static case and never sees that **the geometry the tyre operates on at the load peak is not the geometry anyone designed.** KinematiK already owns every piece of that loop as a tested standalone part — the rigid corner solver, the member load-path resolver, the quasi-static compliance coupling, the 1.5-on-yield screen, and a transient integrator producing per-corner load histories at millisecond resolution. Ghost Topology (✅ Design & Sizing) is the **join**: it walks a transient overload and, at each audited instant, solves the deformed suspension state under that instant's loads and reports the three things the siloed workflow structurally cannot see.

- **Geometry drift vs rigid intent.** Camber, toe, instant-centre, roll-centre height and contact-patch, ghost minus the rigid design value *at that instant's travel*, sampled through the event. Soft enough links don't just erode the intent — they **invert** it: a designed-negative outer wheel driven positive while it's loaded, the grip-losing sign flip that a static spreadsheet cannot show. `COMPLIANCE_INVERTED` is its own headline verdict.
- **Load-path migration.** The same wheel load reacted through the rigid geometry vs the ghost geometry lands on different force lines, so the FoS FEA screened at the static case is not the FoS the part sees mid-event. The tab shows the member-by-member share shift at the worst instant — the redistribution the export-one-case workflow throws away.
- **Transient structural margin, traced.** Every member's FoS on yield in tension and yield **and** pinned-pinned Euler buckling in compression (the honest column for a spherical-jointed two-force member — same screen as the bracket audit and the tube frame), evaluated through the whole event instead of at one hand-picked case, against the team's standing 1.5 rule. The dip a static check can't see becomes the exact load case — *which instant of which event* — to hand the FEA seat, with the pass criterion attached.

And it closes the loop the siloed tools cannot: the **tyre-force feedback**. Compliance camber and compliance steer move the tyre's operating point, which moves the lateral force, which moves the deflection. Per instant that's a scalar fixed point, and its contraction ratio is **measured**, not assumed — `loop_gain = d(feedback force)/d(applied force)`. |gain| < 1 is a contraction: the closed-loop force is the geometric-series sum, and the gain *is* the reported stability margin. |gain| ≥ 1 is `FEEDBACK_DIVERGENT` — compliance-induced instability, no quasi-static equilibrium on that branch — said out loud instead of printing a fixed point that doesn't exist.

Why no supercomputer: **time-scale separation, stated and priced.** A link's structural modes live at hundreds of Hz to kHz; the chassis dynamics live at ~1–20 Hz. Across that gap the structure tracks its load quasi-statically, so the co-simulation collapses to the already-tested compliance solve evaluated *algebraically* along the load history — a few corner solves per audited instant, cached across near-identical loads. Laptop arithmetic. The same statement prices the limit: sub-5 ms load edges (a curb-strike impact) break the separation, and those instants are **flagged per instant** as a structural-dynamics question the tool refuses to answer, rather than quietly answering anyway. A member past yield voids the elastic geometry beyond that instant instead of plotting through it. Honest scope — quasi-static, one corner at a time, no plasticity — is in the module docstring and the report footer.

All of it lives in `suspension/ghost_topology.py` — pure Python, headless, self-tested (`python3 -m suspension.ghost_topology`), UI in `ui/ghost_topology.py`. See bottleneck **#16** in `docs/BOTTLENECKS.md`.

---

## New: 🎲🛡️ Stochastic Inversion — the car the welder will actually build, and the shim stack that saves it

Every tool in the chain — including this repo's own kinematics tab until now — takes hardpoints as **exact** coordinates. Tolerance lives in a drawing note that nothing downstream ever simulates. On the floor the car is a random draw from a cloud around the design: hand welds pull tabs 1–2 mm and pull them *toward the bead* — a **bias**, not just a scatter — jig errors stack, rod ends carry play. So the deck describes one car, the welder builds another, and a geometry optimised to a knife-edge peak ships fragile: the season's tuning was tuning a car that was never going to exist, and the first anyone learns of it is the tyre stickers coming back wrong at a test day nobody can afford to repeat. Stochastic Inversion (✅ Design & Sizing) makes buildability a computed number and closes the metrology loop back to a shim stack:

- **The manufacturing yield.** Declare the *asymmetric* per-point, per-axis error field the shop actually holds — three presets seed it (hand-welded tabs, jig-welded, CNC/machined, plus a weld-pull bias on any axis; welded wishbone inners get the weld class, tie-rod and outboard joints get the machining class, every bound editable) — and the kinematic acceptance bands (camber, bump steer, RC height, scrub, caster). Thousands of buildable cars sweep through the forward solver and the yield is the fraction that stay in-band. Verdicts: **ROBUST / MARGINAL / FRAGILE / SOLVER_LIMITED** — that last one names a nominal sitting near a kinematic singularity, a fragility no shim fixes.
- **The linearisation that prices itself.** The honest trick is one central-difference sensitivity matrix (40 corner solves) that propagates the whole cloud in microseconds — and every linear-mode yield ships with a verification subsample of *full nonlinear solves* whose pass/fail agreement is printed next to the number. Below 98 % agreement the result demotes itself and tells you to run full mode. Never a fast answer without its price tag.
- **The anatomy of the failures.** Per-metric fail fractions name which band kills the yield, and first-order variance attribution names which tab's tolerance drives that metric — so "build a jig for the upper rear inner" comes out of arithmetic, not a meeting.
- **The robust nudge.** An asymmetric field means E[Δmetric] = J·μ ≠ 0: the *expected* as-built car is off-intent before the first cut. The nudge solves the nominal shift that re-centres the whole cloud inside the bands — aim up-wind of the weld pull — clamped to declared per-point freedom, verified by full solves judged against the **original** intent so the goalposts cannot quietly move. A centred field gets the honest sentence (no nominal shift can raise a linear yield around a centred cloud — jig the tabs or widen the bands) instead of a fabricated optimum.
- **The Alignment Prescription.** Once the chassis *is* welded: paste the CMM / caliper as-built coordinates (a >25 mm shift is refused as a units-or-frame slip — the Saboteur lesson — never shimmed), declare the adjusters the car really has (point, axis, range, shim step), and the prescription solves, quantises to the shim step, clamps to the ranges, and then **re-solves the shimmed geometry in full** — the residual printed is the one the real car will carry. A metric in the null space of the declared adjusters is named **unreachable** instead of rounded away. Verdicts: **RESTORED / PARTIAL / UNSHIMMABLE**. Alignment day becomes arithmetic instead of folklore.

Scope, honestly: independent per-point errors (a jig shifting a whole cluster together is a nominal move, not a field), links built-to-fit (manufacturing error only — Ghost Topology owns the loaded deflection), kinematic intent only. The surviving population is exactly what Ghost Topology, Phantom Envelope and ThermicPatch should receive: a distribution instead of a point.

All of it lives in `suspension/kinematik_stochastic.py` — pure Python, headless, deterministic, self-tested (`python3 -m suspension.kinematik_stochastic`), UI in `ui/kinematik_stochastic.py`. See bottleneck **#17** in `docs/BOTTLENECKS.md`.

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
| **Phantom Car** | The margin audit: disclosed design assumptions priced in σ of the evidence-graded ledger, a sha256-sealed team design percentile, ALIGNED/STACKED/NAKED/ANTI-HEDGED verdicts with releasable envelope and exposure priced, a two-cars contradiction detector, FORM reliability index β per load case, and a nominal / coherent / phantom comparison per objective |
| **Earshot** | The test-day power audit: laps-per-config and minimum detectable effect from the two-sample power formula with the lap budget set by the pack, exact linear-drift bias per AABB/ABAB/ABBA run order with swap costs, instrument-propagated delivered bands that decide the evidence grade a test earns (SHARPENS/MOOT), sha256-sealed session sheets with DETECTED / NOT-DETECTED (miss probability priced) / VOID verdicts |
| **Fusebox** | The failure-order audit: P(fails first) per element of every declared overload chain from evidence-graded capacity bands (deterministic quadrature, napkin-checkable pairwise form), FUSED/COIN-FLIP/INVERTED/UNFUSED/BREACH-RISK verdicts against a sha256-sealed Fuse Charter, three exact fixes per rival including evidence-grade sharpening, expected overload bill in $ and days, AS-DESIGNED/SURPRISE/BREACH incident judging with the free capacity datum |
| **Stochastic Inversion** | The manufacturing-yield audit: asymmetric per-point per-axis tolerance fields (shop presets + weld-pull bias, every bound editable), Monte Carlo yield through a self-pricing sensitivity linearisation verified by full-solve subsamples, ROBUST/MARGINAL/FRAGILE/SOLVER_LIMITED verdicts, per-metric fail anatomy and variance attribution, a robust nominal nudge that re-centres the cloud (verified against the original intent), and a metrology-fed Alignment Prescription that quantises to real shim steps, re-solves the shimmed car in full, and names unreachable metrics — RESTORED/PARTIAL/UNSHIMMABLE |
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
4. Confirm build stamp in the Usage section reads `0.26.0-fusebox` and streamlit runtime reads `>= 1.58.0`.

---

## What changed in this build (`0.28.0-stochastic`)

**🎲🛡️ Stochastic Inversion — new tab (Design & Sizing)**
- New `suspension/kinematik_stochastic.py`: the manufacturing-yield audit and the metrology feedback loop. Asymmetric per-point, per-axis tolerance fields (`ToleranceSpec` lo/hi bounds, uniform or truncated-normal, closed-form mean/variance; shop presets hand-weld / jig-weld / CNC with a directional weld-pull bias applied to the welded wishbone inners only — tie-rod inner and all outboard joints are machining-class, a physics distinction, not a convenience). One central-difference sensitivity matrix (40 corner solves, refused if the nominal itself won't solve) propagates thousands of sampled cars in microseconds, and every linear-mode yield is **priced live** by a full-nonlinear verification subsample whose pass/fail agreement prints with the result — below 98 % it demotes itself with a warning. Verdicts ROBUST / MARGINAL / FRAGILE / SOLVER_LIMITED (perturbed geometries that fail to solve mean the nominal sits near a kinematic singularity). Per-metric fail fractions, expected bias E[Δ] = J·μ, p05/p95 spreads, first-order variance attribution naming the tab that drives each metric. `robust_nudge` solves the band-weighted least-squares nominal shift that re-centres an asymmetric cloud, clamps to declared per-point freedom (clamps named), honestly refuses a centred field, and verifies with full solves judged against the **original** design intent via an explicit target — so the re-centred nominal cannot quietly move the goalposts. `alignment_prescription` takes pasted as-built coordinates (a >25 mm shift is refused as a units/frame slip and never shimmed), the real adjusters (point, axis, range, shim step), solves at the as-built geometry, quantises and clamps the moves, then **re-solves the shimmed car in full** so the printed residual is the one the car will carry; metrics outside the adjusters' reach are named unreachable. RESTORED / PARTIAL / UNSHIMMABLE. Pure Python, headless, deterministic (seeded), self-tested (`python3 -m suspension.kinematik_stochastic`).
- New `ui/kinematik_stochastic.py` under the `ui/` strangulation pattern: shop-preset + weld-pull error-field editor with every per-point bound editable, acceptance-band and engine controls, verdict banner with per-verdict blurb, four metrics (yield, verification agreement, worst metric, solver failures), and four sub-tabs — per-metric spread, variance attribution, the robust nudge with freedom and verification controls plus a markdown handover, and the Alignment Prescription with a CSV as-built parser, adjuster editor, verified shim table and download.
- Registered in the lazy public-API facade (`suspension/__init__.py`): `kinematik_stochastic` submodule plus `ToleranceSpec`, `ToleranceField`, `YieldSpec`, `StochasticThresholds`, `StochasticResult`, `Sensitivity`, `sensitivity`, `stochastic_sweep`, `RobustNudge`, `robust_nudge`, `Adjuster`, `Prescription`, `alignment_prescription`, `render_stochastic_md`, `render_prescription_md` — guarded by the existing lazy-init and public-API export tests, still green.
- Wired into the shell (tab meta, the Design & Sizing category, the suspension role tabs, both description maps, the verify-goal list, and the render container) following the strangulation pattern; a broken tab is isolated in `try/except` and can never take the studio down.
- `tests/test_kinematik_stochastic.py`: 28 tests pinning the field moments and bounds, sampler determinism, preset physics (tie-rod inner is machining-class), the Jacobian against direct solves and the bump-steer↔tie-rod coupling, byte-identical reports across runs, yield monotonicity in field size, the linearisation's printed price and its agreement with full mode, attribution normalisation, every verdict boundary, the centred-field refusal, the nudge's bias re-centring and the goalpost fix (verified-vs-original ≈ predicted), freedom clamps named, the prescription restoring an injected 1.4 mm weld error to RESTORED, shim quantisation and clamping, an unreachable metric named, the metres-as-mm refusal, and adjuster axis normalisation.
- `docs/BOTTLENECKS.md` gains bottleneck **#17 — the car nobody welds**, the first audit to treat hardpoints as the distribution the shop actually delivers and to close the metrology loop back to a shim stack.

*(Previous build `0.27.0-ghost` below.)*

## What changed in this build (`0.27.0-ghost`)

**👻🔩 Ghost Topology — new tab (Design & Sizing)**
- New `suspension/ghost_topology.py`: the deformed-geometry audit — the join that closes the tyre-compliance loop the siloed rigid-kinematics → static-FEA workflow leaves open. Walks a transient overload and solves the ghost topology at each audited instant: geometry drift vs rigid intent at that instant's travel (camber, toe, instant-centre, roll-centre height, contact patch), member load-path migration (same wheel load through rigid vs ghost geometry, share shift summing to zero), transient FoS per member on yield **and** pinned-pinned Euler buckling (the honest column for a spherical-jointed two-force member), and the tyre-force feedback loop gain **measured** by contraction — |g| < 1 gives the geometric-series closed-loop force with the gain reported as the stability margin, |g| ≥ 1 is flagged divergent with open-loop values, no fabricated fixed point. Verdicts worst-first: FEEDBACK_DIVERGENT / COMPLIANCE_INVERTED / MARGIN_BREACHED / COMPLIANCE_DEGRADED / RIGID_FAITHFUL. The zero-FEA trick is time-scale separation, stated and priced: the quasi-static compliance solve evaluated algebraically along the load history, 25 N-quantized solve cache, a few solves per event. It prices its own limits — sub-5 ms load edges flagged per instant as a structural-dynamics question it refuses to answer, a member past yield voiding the elastic geometry beyond that instant. The body→corner sign mapping is applied once, documented, and a failed transient is refused rather than audited as zeros. Pure Python, headless, deterministic, self-tested (`python3 -m suspension.ghost_topology`).
- New `ui/ghost_topology.py` under the `ui/` strangulation pattern (physics stays in `suspension/`, this only orchestrates and draws): manoeuvre picker (step steer / snap oversteer / brake→throttle / curb strike), corner picker with the sign-mapping help, audited-instants slider, link tube / material / yield and chassis-tab-stiffness editor, tyre-sensitivity overrides, the verdict banner with per-verdict blurb, four metrics (worst FoS, peak Δcamber, Δtoe, loop gain with the margin-to-instability delta), the findings list, and four sub-tabs (geometry vs intent, transient margins vs the 1.5/1.0 lines, load-path shift at the worst instant, the instant table) plus a markdown handover export. Reads the live hardpoints when the Kinematics tab has set them.
- Registered in the lazy public-API facade (`suspension/__init__.py`): `ghost_topology` submodule plus `GhostCorner`, `ghost_audit`, `ghost_audit_transient`, `MemberSection`, `TireSensitivity`, `GhostThresholds`, `GhostAudit`, `GhostInstant`, `render_ghost_md`, `uniform_sections` — guarded by the existing lazy-init and public-API export tests, still green.
- Wired into the shell (tab meta, the Design & Sizing category, the render container) following the strangulation pattern; a broken tab is isolated in `try/except` and can never take the studio down.
- `tests/test_ghost_topology.py`: 24 tests pinning the closed forms (Euler Pcr, yield FoS, buckling-governed compression), the roll-centre construction, the feedback sign paths, the fixed-point self-consistency and the measured-divergence flag, stiffness monotonicity, load-path bookkeeping, the travel baseline and its clamp, every verdict boundary (stock corner RIGID_FAITHFUL, soft corner MARGIN_BREACHED at the load peak, very-soft corner COMPLIANCE_INVERTED outranking margin), the solve cache with per-instant timestamps, the fast-edge quasi-static flag, the transient sign mapping, failed-transient refusal, and the markdown report.
- `docs/BOTTLENECKS.md` gains bottleneck **#16 — the rigid lie under load**, the first audit to close the tyre-compliance feedback loop.

*(Previous build `0.26.0-fusebox` below.)*

## What changed in this build (`0.26.0-fusebox`)

**⛓️ Fusebox — new shared-spine tab (Checks & Integration)**
- New `suspension/fusebox.py`: the failure-order audit. Overload chains as ordered element lists (FoS at the element's own load share, evidence grade + staleness pricing σ via the exact Proof Engine band law — one pedigree law, fifth consumer), first-failure probabilities from deterministic fixed-grid quadrature of the minimum of independent normals with the pairwise closed-form printed for napkin checks, verdicts FUSED / COIN-FLIP / INVERTED / UNFUSED / BREACH-RISK against a sha256-sealed Fuse Charter (one confidence, one forbidden-first tolerance, designations per path), three exact fix levers per rival (soften-the-fuse floored at FoS 1.10, stiffen-the-rival, sharpen-the-grade — with honest infeasibility messages when z·r ≥ 1 means no metal can fix an unknown), the probability-weighted overload bill in $ and days, incident judging (AS-DESIGNED / SURPRISE / BREACH, every break banking the free capacity datum; a tampered charter refuses to judge), four seeded FSAE-EV archetypes whose grades tell the true story, and a pinnable markdown Fuse Map. Pure stdlib, deterministic end to end, self-tested (`python3 -m suspension.fusebox`).
- New `ui/fusebox.py` under the `ui/` strangulation pattern: pecking-order board with per-path expected bills, path & element editor (seeds resettable), fix-arithmetic panel, charter sealing + incident judging.
- Wired into all seven shell registries (tab meta, category grouping, shared spine, both description maps, the how-it-works bullets, and the render container); a broken tab is isolated in `try/except` and can never take the studio down.
- `tests/test_fusebox.py`: 27 tests pinning the quadrature against the closed form, determinism (byte-identical probabilities and markdown), the flagship 27 % coin flip, every verdict at its documented constants (BREACH-RISK outranking all, empty paths and missing severities as blind spots never passes), fix roots satisfying the pairwise equation to display precision, the fuse floor, the no-metal-fixes-an-unknown infeasibility, seal tamper-refusal, and incident verdicts with the free datum.
- `docs/BOTTLENECKS.md` gains bottleneck **#15 — the failure order nobody chose**, the first physical-consequence audit alongside the four epistemic ones.

*(Previous build `0.25.0-phantom` below.)*

## What changed in this build (`0.25.0-phantom`)

**👻 Phantom Car — new shared-spine tab (Checks & Integration)**
- New `suspension/phantom_car.py`: the margin audit. Each consumer *discloses* the design value its sizing actually uses (a built-in FSAE-EV consumption map seeds the form at nominal on purpose); every hedge is priced in σ of that quantity's own evidence-graded, staleness-inflated band — the exact Proof Engine ledger, third consumer, zero new physics. One sha256-sealed Margin Charter percentile judges the lot: **ALIGNED / STACKED** (excess priced as releasable envelope in the quantity's own units) **/ UNDER-COVERED / NAKED** (naming the evidence grade it's naked on) **/ ANTI-HEDGED**. The **two-cars detector** flags assumed values >1σ apart on the same quantity (the Integration contradiction check, applied to *assumptions* for the first time); **β = √(Σz²)** is the FORM reliability index, stating the odds each stacked load case defends against; and the **three-cars comparison** (nominal / coherent-percentile / phantom) prices, per objective, the design envelope currently spent defending cars the deck's own σ says cannot exist — reported honestly as envelope, never as promised savings. A charter with a broken seal refuses to judge, out loud. Pure stdlib, deterministic end to end, self-tested (`python3 -m suspension.phantom_car`).
- New `ui/phantom_car.py` under the `ui/` strangulation pattern: sealed-charter panel, disclosure editor (seed / demo / hand-edit), verdict board with releasable-and-exposed pricing, the two-cars detector, β per consumer, the three-cars table per objective, honest blind-spot and unresolved-key reporting, and a pinnable Margin Docket markdown export. Shares the `proof_pedigree` session map with the Proof Planner and the Saboteur — one pedigree, three consumers, on purpose.
- Wired into all seven shell registries (tab meta, category grouping, shared spine, the two description maps, the how-it-works bullets, and the render container) following the strangulation pattern; a broken tab is isolated in `try/except` and can never take the studio down.
- `tests/test_phantom_car.py`: 22 tests pinning determinism (audit and docket byte-identical), the verdict thresholds at their documented constants (ALIGNED at the charter percentile, NAKED at nominal, STACKED with releasable envelope, ANTI-HEDGED on a favourable value), the two-cars detector (contradiction named with both consumers, quiet within 1σ), β = √(Σz²) over stacked worst cases only, the phantom as the union of adverse extremes, seal tamper-evidence (an edited charter refuses to judge and computes nothing else), and honesty (unresolved keys and unaudited consumers reported, seeds never fabricating prudence).
- `docs/BOTTLENECKS.md` gains bottleneck **#13 — the margin nobody adds up**, the conservatism mirror-image of #12, closing the triangle with the Proof Engine and the Saboteur.

*(Previous build `0.24.0-saboteur` below.)*

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
