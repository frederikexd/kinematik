# The Prevalidation Bottleneck Map — EV & Formula-EV teams, 2026

*Why this document exists: KinematiK's product thesis is "the hour before
ANSYS." This is the audit of that hour — every place where garbage forms
before it reaches SolidWorks, Lotus Shark, ADAMS, or ANSYS — and where
KinematiK attacks it, deliberately does not, or now attacks it with the
Proof Engine. If a bottleneck below has no feature, that is the roadmap.*

---

## The twelve bottlenecks

### 1 · The number without a frame
A hardpoint sheet in one convention, a CAD model in another, a tire model in a
third. Migration is priced as "a full redo," so it never happens, and a sign
flip on Z survives all the way into a Shark or ADAMS input deck.
**Attacked by:** 🧭 Frames & Datums — charter, Rosetta, migration wizard,
sign-convention linter, frame tags on every export.

### 2 · The eight-spreadsheet car
Eight subsystems each holding their own copy of mass, CG, torque, airflow.
Eight "we're ~12 kg" estimates that quietly sum to 18 kg over the target
suspension tuned around. The simulated car is not the agreed car.
**Attacked by:** 🔗 Integration ledger — declare once, propagate everywhere,
diff on edit, missing ≠ zero.

### 3 · The invisible ripple
A torque bump loads an upright and heats a cooling loop, and nobody is paid to
hold that graph in their head. Cross-subsystem consequences surface at the
rig, or at competition.
**Attacked by:** risk propagation over the coupling graph, with honest
measured / coupled / judgement confidence tags; DFMEA rows move live.

### 4 · Geometry archaeology
"The current diff mount" is a Drive link in a Discord message from six weeks
ago. Which file the ANSYS study actually meshed is unrecoverable.
**Attacked by:** 🗂️ Registry — one CURRENT version per component,
content-addressed blobs, promotion history, sign-off.

### 5 · The unearned green board
Estimates rendered indistinguishably from measurements. A dashboard that is
green because nobody has looked, not because anyone has proven anything.
**Attacked before:** `is_estimate` flag, provenance tags — *binary*, so a
number was either "an estimate" or silently trusted.
**Attacked now (new):** 🎯 Proof Engine — five evidence grades
(guess/estimate/modelled/measured/verified), each with a quantified ± that
**inflates with age**. Last season's corner-weighing is not this season's.
A checkbox can never claim *measured*; only a dated claim with a source can.

### 6 · Solver hours spent by habit, not by information
The single largest waste in student-EV prevalidation. The upright gets an
8-hour FEA campaign because FEA is the skill in the room, while the lap-time
prediction the whole season tunes against is ±2 s dominated by a CG height
nobody has ever measured. No tool on the market — not ANSYS, not ADAMS, not
HyperWorks, not Shark — answers *"is this run worth doing before that test?"*
**Attacked now (new):** 🎯 Proof Planner — deterministic one-at-a-time
uncertainty propagation to the objective the team picks (lap time, endurance
energy, thermal margin, mass), variance attribution per input, and a catalog
of evidence actions (corner scales, tilt test, coast-down, dyno pull, flow
bench, pack thermal log, strain-gauged mount, an ANSYS study) ranked by
**uncertainty retired per hour**. The output is the literal, exportable list
of questions worth asking the expensive tools — the tagline as a feature.

### 7 · Goalposts decided after the run
The sim says FoS = 1.05 and 1.05 is retroactively declared fine, because the
part is already drawn and the review is tomorrow. Confirmation bias is the
default state of a tired team. Experimental science solved this with
pre-registration; no engineering tool has imported it.
**Attacked now (new):** 🔏 Validation Contracts — the acceptance band and the
criterion note are fixed and **sha256-sealed before the run**. Judging a
result never mutates sealed fields; editing them breaks the seal, and a
broken seal refuses judgment out loud. The pass criterion provably never
moved.

### 8 · Failed design vs garbage run — indistinguishable
A result slightly out of band and a result that contradicts everything
upstream get the same shrug. Acting on a garbage run — wrong units, flipped
frame, wrong geometry version, wrong BCs — is how garbage propagates
downstream wearing a solver's credibility.
**Attacked now (new):** the three-way verdict. Every contract carries a
**plausibility envelope** (prediction ± 3σ from the uncertainty ledger — not
chosen by the team, which is what keeps it honest). Inside the band: PASS.
Outside the band but inside the envelope: FAIL — a design finding, caught
before the first cut. Outside the envelope: **DISCREPANT** — the run and the
ledger disagree about reality; audit units, frame, BCs, and geometry version
before trusting either number. The Frames tab exists because frame flips top
that audit list.

### 12 · The garbage that flatters the envelope
The residual left open by #8: the plausibility envelope catches a run that
looks *impossible*, but the deadliest corruptions — pounds into a kg field,
the kilo prefix slipping, a subsystem dropped from the roll-up — move the
answer by an amount that still looks *fine*. Nobody audits a result that
confirms expectations; the sealed contract itself would smile at it. And the
checks teams do run (total mass, out of habit) are picked by folklore, so the
torque-unit slip that total mass cannot see sails through.
**Attacked now (new):** 🧨 Saboteur — mutation testing for the input deck.
Every catalogued corruption class is injected into a shadow copy of the
ledger; the ones that would come back looking plausible get **tripwires** —
cheap checksums (rolled-up mass, torque-per-power, implied pack voltage)
chosen by a deterministic detectability set-cover, sealed before the run like
a contract, and judged when the readings return. A tripped pattern is
fingerprinted against every predicted corruption signature, so the audit
starts with a named suspect ("this matches lb→kg on the accumulator mass"),
not an evening of guessing. Corruptions *no* wire can see are reported as
blind spots out loud — never absorbed into the coverage number.

### 9 · Interference found at the mill
Rework is the tax for not integrating before cutting; a richer team can
afford to cut twice.
**Attacked by:** integration clash checks, envelope-vs-chassis, Fit Forecast,
mount-point clash, master assembly.

### 10 · Rules discovered at scrutineering
EV rules (accumulator, tractive system, insulation, fusing) checked from
memory during design, verified for the first time by a scrutineer.
**Attacked by:** accumulator rules checks, tractive-system electrical gate,
manufacturing-release gate that fails on absent evidence.

### 11 · Graduation amnesia
The *why* behind every number leaves in May. Next year's cohort re-fights
settled arguments and re-makes solved mistakes.
**Attacked by:** Decision Registry, rationale fields on every interface,
charter exports, Mythbuster; and now every sealed contract and judged verdict
is a permanent, tamper-evident record of what was claimed, what was required,
and what reality said.

---

## Why the Proof Engine and the Saboteur are the ones nobody has built

CAE vendors sell *answers*. Their economics reward more solving, not less —
a tool that tells you a 2-hour scale session beats an 8-hour solve is a tool
that sells fewer solver hours, so it will never come from a solver vendor.
PLM tools track *files*, not the certainty of the numbers inside them.
Requirements tools track *targets*, not whether the evidence for meeting them
is a guess or a measurement or how stale it is.

The Proof Engine sits in the gap all three leave open: it treats **certainty
as a budgeted resource**, spends validation effort where the value-of-
information arithmetic says, and imports **pre-registration** — the strongest
anti-bias instrument experimental science has — into the engineering
validation loop, with a hash instead of an honor system.

The Saboteur closes the loop from the other side. Mutation testing is how
software engineering audits its *test suites* — inject known bug classes,
see which survive — and no one has ever pointed it at an engineering input
deck, for the same economic reason: a tool that tells you which of a
solver's answers would be undetectably wrong is a tool no solver vendor
will build. Checklist culture has the right instinct but picks its checks
by folklore; the Saboteur replaces folklore with detectability arithmetic,
reusing the exact uncertainty ledger the Proof Engine already maintains —
zero new data entry buys a sealed tripwire net and a fingerprint database.

Everything in both is deterministic and reproducible by hand with a calculator:
same ledger in, same plan out, every attribution a symmetric perturbation you
can check. That is the KinematiK ethos applied to the tool itself — it earns
trust the same way it demands the sims do.

## Deliberately out of scope

KinematiK still does not solve fields, mesh geometry, or replace a lap sim of
record. The Proof Engine's objective surrogates are documented closed forms
with named sensitivities, tagged *coupled*, and injectable — where the real
lap sim or pack-thermal solver is configured, it takes over. The surrogates
exist so the planner works on day one of a season with nothing but the
ledger, which is exactly when the team needs to be told to buy a set of
corner scales before booking cluster time.
