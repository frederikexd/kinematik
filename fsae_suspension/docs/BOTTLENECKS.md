# The Prevalidation Bottleneck Map — EV & Formula-EV teams, 2026

*Why this document exists: KinematiK's product thesis is "the hour before
ANSYS." This is the audit of that hour — every place where garbage forms
before it reaches SolidWorks, Lotus Shark, ADAMS, or ANSYS — and where
KinematiK attacks it, deliberately does not, or now attacks it with the
Proof Engine. If a bottleneck below has no feature, that is the roadmap.*

---

## The thirteen bottlenecks

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

### 13 · The margin nobody adds up
The conservatism mirror-image of #12: not garbage in the deck, but garbage in
the *assumptions about* the deck. Every subsystem hedges the same uncertainty
separately, in secret — brakes quietly sized for the heavy car, structures
taking a worst-case load and stacking FoS 1.5 on top of it, "add a bit to be
safe" applied at every hand-off — while the energy budget, the one number
that actually DNFs an endurance, consumes the same mass at its optimistic
target value, naked. The deck ends up describing several mutually exclusive
cars at once; the total conservatism of the design is a number nobody has
ever computed because it lives smeared across eight private spreadsheets;
and the mass bill for defending statistically impossible cars lands on the
real one. Aerospace primes run staffed margin-management processes for
exactly this; no CAE, PLM, or requirements tool computes it, and nothing a
student team can afford even names it.
**Attacked now (new):** 👻 Phantom Car — the margin audit. Each consumer
*discloses* the design value its sizing actually uses (disclosure, not new
work — the number already lives in the spreadsheet) plus any factor applied
on top; every hedge is priced in σ of that quantity's own evidence-graded
uncertainty (the Proof Engine ledger — one ledger, third consumer, zero new
physics); and everything is judged against a single sha256-**sealed Margin
Charter** percentile. Verdicts: ALIGNED, STACKED (with the releasable
envelope priced in the quantity's own units), UNDER-COVERED, NAKED (naming
the evidence grade it is naked on), ANTI-HEDGED. The **two-cars detector**
flags assumed values more than 1σ apart on the same quantity — the exact
contradiction the Integration ledger kills for values, applied for the first
time to assumptions. **β**, the same first-order reliability index (FORM)
professional reliability engineering uses, states the improbability each
stacked load case defends against ("this bracket case is a 1-in-2.3-million
car"). And the **three-cars comparison** — nominal, coherent-percentile,
phantom (the union of everyone's private fears) — prices the design envelope
currently spent on cars the deck's own σ says cannot exist, per objective,
reported honestly as envelope and never as promised savings. Undisclosed
consumers are listed out loud as unaudited blind spots.

### 14 · The deaf test day
The physical mirror-image of #6: solver hours get spent by habit, but TEST
DAYS get spent on faith. The Proof Engine ranks "coast-down, 5 hours,
retires X uncertainty" — silently assuming the test works. Three ways it
doesn't, and every team has lived all three: (a) the A-B session that was
statistically deaf before the trailer loaded — a predicted 0.3 s wing
against a 0.8 s driver sigma needs ~112 laps per configuration at 80 %
power, the pack holds 40, and the inevitable "inconclusive" gets read as
"the wing doesn't work", falsely burying a real gain; (b) the confounded
run order — all A laps then all B laps (one wing swap, so it's what tired
teams do) hands a 0.03 s/lap tire-wear drift a 0.6 s bias, twice the
hunted effect, while ABBA blocks would have cancelled it for free; (c) the
measurement that teaches nothing — a shallow tilt test with a coarse
protractor delivers a wider band than the ledger already carries, yet the
checkbox culture still logs "CG height: MEASURED ±3 %", an unearned grade
upgrade the whole downstream chain then trusts. Clinical trials have
refused to start without a power calculation for fifty years; DOE lives in
statistics packages that have never heard of a car; datalogger vendors
sell hindsight. Nobody runs the arithmetic BEFORE the session, because
running it needs the predicted effect (the lap model), the noise floor
(driver sigma as an evidence-graded quantity), the session budget (the
accumulator), and the current uncertainty bands, in one place.
**Attacked now (new):** 🎙️ Earshot — the test-day power audit. Laps-per-
config from the standard two-sample power formula and the minimum
detectable effect of the session actually booked, with the pack itself
setting the lap budget (usable kWh over kWh-per-lap — the plan is spent in
the same currency the race is). The ordering audit computes the exact bias
a linear drift injects into AABB / ABAB / ABBA next to the swap cost each
one pays, and declares CONFOUNDED when drift alone rivals the effect.
Instrument propagation (tilt-angle term, coast-down band separation, pad
resolution) states the band a parameter test will actually deliver — and
therefore the evidence grade it EARNS; a plan that can't beat the ledger's
current band is MOOT out loud, before the trip. The whole design — δ, σ,
α, power, ordering, burn-in laps, abort criterion, MDE — is sha256-sealed
before the trailer loads; a shortened session judges VOID instead of
quietly widening its own goalposts, and a NOT-DETECTED comes back carrying
the sealed probability that a real effect hid — absence of evidence,
priced, never shrugged.

### 15 · The failure order nobody chose
Every credible overload — the curb, the cone, the tow-truck yank — sends
one load through a chain, and something in that chain WILL fail first.
Electrical engineering made choosing that victim a design act 150 years
ago (the fuse); mechanical load paths on a formula car choose by accident:
whichever capacity happens to be lowest goes, and under the deck's own
evidence-graded sigma the order isn't even determined — a MODELLED FoS 1.35
tie rod loses the race to a GUESS-grade FoS 1.8 upright roughly one strike
in four, swapping a $45 afternoon for a $900 six-week competition-ender.
On an EV the stakes escalate from lead time to safety: the accumulator
container and cell restraint must be LAST in every ordering, and that claim
is believed by construction, verified by nobody. The ordering is invisible
to every tool in the chain — FEA reports one part's margin per run, DFMEA
ranks by RPN folklore, requirements tools see targets — so it lives between
the tools and no tool owns it.
**Attacked now (new):** ⛓️ Fusebox — the failure-order audit. P(fails
first) per element from the first-order statistics of the minimum of
independent normal capacities, sigma priced by the exact Proof Engine
grade→band law (fifth consumer), deterministic fixed-grid quadrature with
a napkin-checkable pairwise collapse. Verdicts against a sha256-sealed
Fuse Charter: FUSED / COIN-FLIP (contenders named) / INVERTED (priced in
$ and days) / UNFUSED / BREACH-RISK (any forbidden element above the
sealed tolerance outranks everything). Three exact fixes per rival —
soften the fuse (floored: a fuse that pops at 1.0 pops in normal
driving), stiffen the rival, or SHARPEN its evidence grade, because half
of most coin flips is a wide band, not weak metal, and a strain-gauge
pull test is cheaper than a re-machine. Incidents judge AS-DESIGNED /
SURPRISE / BREACH against the sealed designation, and every real break
banks the free datum: reality just measured that capacity.

### 16 · The rigid lie under load
Every kinematics solver in the chain assumes the links are rigid, hands the
motion and the loads to FEA as a static case, and stops — nobody closes the
loop where the deflected part changes the geometry, the changed geometry
changes the tyre force, and the changed force changes the deflection. The
honest way to close it is co-simulated nonlinear FEA against multibody
dynamics: an enterprise licence and a workstation that melts, so student
teams pick one hand-chosen static load case, throw a safety factor at it, and
never see that the geometry the tyre actually operates on mid-event is not
the geometry anyone drew. Three failures hide in that gap and no affordable
tool surfaces any of them: compliance camber/toe that ERODES — or under soft
enough links INVERTS — the kinematic intent while the wheel is loaded; member
load paths that MIGRATE as the deformed geometry reacts the same wheel load
through different force lines, so the FoS FEA screened at the static case is
not the FoS the part sees at the load peak; and the tyre-force FEEDBACK
itself, which can be a contraction (stable, and the gain is the margin) or a
divergence (compliance-induced instability, no quasi-static equilibrium at
all) — a distinction nobody computes because it needs the loop closed.
**Attacked now (new):** 👻🔩 Ghost Topology — the deformed-geometry audit.
Time-scale separation (link structural modes at hundreds of Hz–kHz vs chassis
dynamics at 1–20 Hz) collapses the co-simulation to the already-tested
compliance solve evaluated ALGEBRAICALLY along the transient load history —
a few Levenberg–Marquardt corner solves per audited instant, cached across
near-identical loads, laptop arithmetic instead of an FEA queue. It walks a
transient overload (step steer, snap oversteer, brake→throttle, curb strike),
solves the ghost topology at each audited instant, and reports the geometry
drift vs rigid intent, the member load-path shift, the transient FoS of every
link (yield AND pinned-pinned Euler, the honest column for a spherical-jointed
two-force member), and the tyre-force loop gain MEASURED by contraction, not
assumed. Verdicts: FEEDBACK_DIVERGENT / COMPLIANCE_INVERTED / MARGIN_BREACHED
/ COMPLIANCE_DEGRADED / RIGID_FAITHFUL. It prices its own limit: sub-5 ms load
edges break the separation and are flagged per instant as a structural-
dynamics question it refuses to answer, and a member past yield voids the
elastic geometry beyond that instant rather than plotting through it. The
output is exactly the load case and the pass criterion to hand the FEA seat —
which instant of which event — not a substitute for it. Fifth consumer of the
compliance stack, first consumer that closes the tyre loop.

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

The Phantom Car closes the triangle. The Proof Engine prices what the team
doesn't know; the Saboteur catches a deck that lies; the Phantom Car audits
what the team *pays* for not knowing — and it is only buildable where the
numbers, their σ, and the map of who consumes them already live in one place.
Margin stacking is invisible to every tool in the chain by construction: a
solver sees one load case at a time and cannot know it was hedged upstream;
PLM sees files; requirements tools see targets, never the private
conservatism between a declared number and the value a consumer's
spreadsheet actually uses. The information needed to add margins up has
never existed in one system before. In KinematiK it already does, so the
feature is a join, not a new data-entry burden.

Earshot points the same discipline at the only evidence source left: the
physical test itself. Statistics tools that could run the power analysis
have never heard of a car; datalogger vendors profit from sessions run,
not sessions cancelled; and a solver vendor has no reason to tell you the
cheap track test can't work either. The a-priori question — can this
session hear the answer — needs the predicted effect, the driver's noise
floor, the pack's lap budget, and the ledger's current bands joined in one
place, and one new number (driver sigma, itself evidence-graded and
measurable from ten baseline laps) buys the entire audit. It also closes
an honesty hole inside KinematiK itself: an evidence action's promised
grade in the Proof Engine is now EARNED by instrument arithmetic, not
claimed by a checkbox — the tool audits its own optimism the same way it
audits everyone else's.

Everything in all three is deterministic and reproducible by hand with a calculator:
same ledger in, same plan out, every attribution a symmetric perturbation you
can check. That is the KinematiK ethos applied to the tool itself — it earns
trust the same way it demands the sims do.

The Fusebox turns the same lens from knowledge to consequence. The four
audits before it ask what the team knows, whether the deck lies, what the
hedges cost, and whether a test can hear — all questions about NUMBERS. The
Fusebox asks the first question about the PHYSICAL car that no tool in the
chain can see: when the overload arrives, which element yields first, and
was that a decision or an accident? Fuse coordination is a solved
discipline in electrical protection and a staffed frangibility process at
aerospace primes, and it exists in no tool a student team can afford,
because computing an ordering needs every element's capacity, the sigma its
evidence quality implies, its replacement cost and lead time, and the map
of which elements share a path — joined. A solver sees one part per run by
construction. In KinematiK the sigma law, the costs, and the chain already
live together, so the Fusebox is a join plus one honest declaration per
path, and its sharpest lever is pure KinematiK doctrine: sometimes the
ordering is restored not by metal but by measurement, because the coin
flip was never mechanics — it was a GUESS-grade band doing the flipping.

## Deliberately out of scope

KinematiK still does not solve fields, mesh geometry, or replace a lap sim of
record. The Proof Engine's objective surrogates are documented closed forms
with named sensitivities, tagged *coupled*, and injectable — where the real
lap sim or pack-thermal solver is configured, it takes over. The surrogates
exist so the planner works on day one of a season with nothing but the
ledger, which is exactly when the team needs to be told to buy a set of
corner scales before booking cluster time.
