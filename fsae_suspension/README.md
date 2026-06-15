---
title: KinematiK
emoji: 🏎️
colorFrom: yellow
colorTo: gray
sdk: streamlit
sdk_version: 1.40.0
app_file: streamlit_app.py
pinned: false
license: mit
---

# ◢ KinematiK

**Open-source architecture-agnostic suspension studio.**
Born as a Formula SAE double-wishbone tool, now a general multibody kinematics platform: edit double-wishbone hardpoints live, or pick any topology — MacPherson strut, multi-link, trailing / semi-trailing arm, solid axle, twist-beam, or a heavy-truck steering linkage — and see the kinematics *and* the vehicle-level consequences update together, in the browser, for free. You can also drop in entirely experimental linkages that fit no textbook geometric definition.

---

## Architecture-agnostic engine

The double-wishbone assumption is no longer baked in. Under the hood is a data-driven multibody kinematics kernel (`suspension/topology.py`): rigid bodies defined by points, and a small set of constraint primitives — distance links (two-force members), ball/pin coincidence, prismatic slider (strut), planar, revolute (hinge), driving DOF, rack translation and beam-axle roll — assembled into a `Mechanism` and solved by a branch-stable Levenberg–Marquardt sweep.

A library of parameterised templates (`suspension/topologies.py`) emits ready-to-solve mechanisms for every common topology, and a `from_links` builder lets you define free-form experimental corners. An adapter (`GenericKinematics`, `suspension/adapter.py`) exposes any mechanism through the original corner-state surface, so the **same** vehicle-dynamics layer (roll-centre migration, anti-dive / anti-squat, load transfer, grip balance) drives every architecture with no changes. Instant centres and side-view swing arms are derived topology-independently from the wheel-carrier velocity field, so they're correct even for linkages that have no literal control-arm pivots.

Templates: `double_wishbone`, `macpherson_strut`, `multilink` (3/4/5-link), `trailing_arm`, `semi_trailing_arm`, `solid_axle` (Panhard or Watts), `twist_beam`, `truck_steer_linkage`, and `from_links` (experimental).

---

## The gap this fills

Every FSAE team makes the same suspension decisions: where to put ten hardpoints so the car gains camber in roll, doesn't bump-steer, and ends up neutral-to-mild-understeer at the limit. The tools that answer those questions well — OptimumK, ADAMS/Car, Lotus Shark — are either four-figure licenses or locked behind a sponsor. So most teams fall back to a kinematics spreadsheet that:

- solves one corner in isolation and stops at camber/toe curves,
- never connects geometry to **roll-centre migration, load transfer, and grip balance**, and
- can't be handed to a first-year without a 30-minute explanation.

KinematiK closes that loop. It runs a real 3D constraint solver for the linkage **and** a coupled vehicle-dynamics layer, so when you drag the lower rear pickup down 10 mm you immediately see what it does to the roll centre, the front/rear load-transfer split, and whether the car pushes or rotates at the limit. That coupling — geometry → kinematics → balance, live — is the thing the spreadsheets and the free web calculators don't do.

## What it computes

**Kinematics (3D constraint solver, not lookup tables)**
- Camber gain & bump steer (toe vs travel)
- Caster and kingpin inclination (KPI) through travel
- Scrub radius
- Front-view instant-centre location
- **Real motion ratio from the actual pushrod/rocker (bell-crank) geometry** — the
  pushrod drives the rocker, the installed spring length is read across it, and
  MR = spring travel / wheel travel is differentiated against wheel travel. Gives
  wheel rate = spring rate × MR², plus the full MR-vs-travel curve (rising/falling
  rate). Falls back to a clearly-labelled direct-acting proxy only when no rocker
  is defined.
- **Anti-dive and anti-squat percentages** from the side-view swing-arm geometry
  (the chassis pivot-axis inclination), referenced to the car's CG height and
  wheelbase and the brake/drive bias.

**Lap time & track (quasi-steady-state point mass)**
- Skidpad, 75 m acceleration (proper standing-start integration), and autocross
- Aero (downforce + drag), and a **real motor torque/speed map** (or the simpler flat
  power cap when you don't have the curve)
- **Track from GPS or cone coordinates** — drive/walk the course or drop the event-map
  cones and the sim runs your actual layout (`track_from_path`, `cones_to_centerline`,
  `latlon_to_xy`); no more manual segment entry
- **Racing-line optimisation** — uses the track width to straighten corners and reports
  the seconds gained vs the centreline (curvature-optimal line)

**Tire (Pacejka MF5.2 lateral, fitted to TTC data)**
- Load sensitivity, camber response, peak-mu and optimal-camber search
- **Combined slip** (Fx+Fy friction ellipse) and **relaxation length** — real physics,
  flagged uncalibrated until you supply drive/brake and transient data, so they never
  present an invented number as measured
- **Damper force–velocity model** (bilinear-digressive) with a damping-ratio diagnostic —
  the building block for the transient model, calibratable from your dyno curve
- **Structural tire co-simulation boundary** — a stateful `StructuralTireModel` contract
  (the FTire / CDTire integration *seam*), a Pacejka-backed reference backend that runs the
  whole co-sim today with no external binary, and vendor adapter stubs that declare exactly
  the binding they need. The reference backend returns `None` (never a faked number) for the
  carcass-deformation, contact-patch-pressure and tread/gas-temperature channels it cannot
  compute, and a staggered driver advances the tyre state once per macro-step around the
  transient solver — the same way ADAMS/Car couples FTire/CDTire to its multibody solver.
  See `docs/tire_cosim_interface.md` for the STI-style channel contract and
  `suspension/tire_cosim_ftire_example.py` for a runnable conforming-wrapper skeleton

**Electronics / custom PCB (the pre-fab electrical gate)**
- **Trace copper survival** — Onderdonk **fusing current** (does it melt), IPC-2221
  steady-state **temperature rise**, DC **resistance / IR-drop**, and an **ECU brown-out**
  check against the worst *simultaneous* load (brake light + both cooling fans at once),
  with the per-trace current rolled up from the integration ledger's declared peak currents
- **Signal integrity** — IPC-2141 edge-coupled **differential-pair impedance** vs a target
  (120 Ω CAN), and a geometric **HV-aggressor coupling** check (closest approach + coupled
  run length) so a CAN pair routed too near the switching motor-controller net is flagged,
  both owners named. Analytic *screening* only — the eye/loss/reflection that need a field
  solver are reported `None`, never faked

**Vehicle dynamics (coupled to the geometry)**
- Front/rear roll-centre heights from the solved instant centres
- **Roll stiffness derived from spring rates through the real motion ratio**
  (k_wheel = k_spring × MR², plus anti-roll-bar rate) — so a quoted spring rate
  maps to a wheel/roll rate through the actual rocker, instead of being assumed
  1:1. This is the lever the optimiser now sweeps.
- Steady-state lateral load transfer, split into geometric + elastic
- Per-tire vertical loads vs lateral g
- **Pacejka MF5.2 tire model** → load-sensitive, camber-aware grip, max lateral g,
  and an **understeer/oversteer balance index**. Ships with a sensible generic FSAE
  tire so it works out of the box, and loads a tire **fitted to your own TTC data**
  the moment you have one — see "Your tire is the edge" below.

**Flexible bodies & compliance (the rigid-link assumption, finally relaxed — NEW)**
- Every other tool here treats the control arms, pushrods and tie rods as
  infinitely stiff. They aren't: at 1.5 g the links stretch and the chassis tabs
  flex, and that shows up at the contact patch as **compliance steer** and
  **compliance camber** you never dialled in. This is the deflection a four-figure
  ADAMS Flex licence is bought for — here it's in the **◢ COMPLIANCE (FLEX)** tab.
- Resolves the **axial load in every member** (upper/lower legs, tie rod, pushrod)
  from the contact-patch wrench via a statically-determinate corner model, deflects
  each link by its **axial stiffness**, and **re-solves the kinematics** under load
  to read the toe/camber the wheel actually runs — iterated to convergence.
- Link stiffness from **tube size + material** (zero-FEA, fully defensible from
  `E·A/L`), with optional **chassis-tab stiffness in series** — usually the bigger
  real-world contributor than the tube itself.
- Or import a **real FEA mesh** of a component as a condensed flexible body: a
  beam/bar mesh KinematiK **Guyan-reduces** itself, or a **pre-reduced superelement**
  (the interface nodes + condensed stiffness an **ADAMS Flex MNF** carries). Honest
  scope: it imports the **static / constraint-mode** content that governs
  load↔deflection in a sustained corner, not the proprietary binary container or the
  dynamic normal modes — and it says so rather than faking them.
- Validated to closed form: a bar gives `E·A/L`, a cantilever `3·E·I/L³`, and a
  two-element Guyan series reduces to the exact series stiffness.

**Lap-time simulator (the number that actually wins — NEW)**
- A quasi-steady-state point-mass lap sim built **on top of the same kinematics +
  Pacejka tire + vehicle-dynamics stack** the rest of the tool uses, so every
  geometry/setup/tire change is judged in the one currency that decides events:
  **seconds**. Ships in the **◢ LAP TIME** tab.
- Runs the three timed FSAE dynamic events out of the box — skidpad (timed
  circle), 75 m acceleration, and a representative autocross/endurance lap — and
  reports per-event times plus an endurance estimate.
- Speed + lateral-g + longitudinal-g trace along the lap, and a **limit
  breakdown** (corner- vs accel- vs power- vs brake-limited %), so an underfunded
  team can see *where* time is won and aim its effort there instead of guessing.
- A **g-g-V capability envelope**: lateral/accel/braking g vs speed, showing how
  downforce raises usable grip with speed — the picture engineers use to sanity-
  check the car. (For the *full* combined envelope and the design-input sweeps,
  see the dedicated **◢ GGV DIAGRAM** tab below.)
- Point-mass layer adds power, drivetrain efficiency, traction limit, braking, and
  aero (downforce *and* the drag it costs) so wing decisions show up honestly.
- **SETUP → SECONDS** tab: re-runs the lap sim for each setup lever and ranks them
  by **lap-time gained**, not an abstract grip index — because the same 0.05 g is
  worth different time on a hairpin vs a sweeper. With one tire set, this points
  your build hours at the lever that buys the most seconds.
- Honest about method: QSS captures corner-speed limits, the accel/brake trade,
  power and downforce — the things that dominate an FSAE lap — but not transient
  yaw, combined-slip friction-circle usage, tire temperature, or the racing line.
  Trust the *ranking* firmly and the *absolute seconds* to a few percent; the UI
  says so. Robust by construction: a bad data point, a non-converging corner, or a
  pathological tire never crashes the session — the sim substitutes a safe default
  and surfaces a warning instead of raising.

**GGV diagram (the full combined-acceleration envelope — NEW)**
- The single steady-state picture an engineer reads before stepping up to full
  transient/multibody modelling: at each speed, the closed boundary in the
  (longitudinal g, lateral g) plane — cornering, braking, accelerating, and every
  combination between. Ships in the **◢ GGV DIAGRAM** tab. Where the lap sim's
  older g-g-V gave only three axis points per speed, this is the *whole* surface.
- Built **through the same load-transfer + Pacejka chain** as everything else, so
  the design inputs an FSAE team actually owns are the levers that reshape it:
  **CG height, roll-centre height, wheel/spring rate (through the real motion
  ratio), dynamic camber, weight and weight distribution, and aero ClA/CdA**.
- A built-in **design-input sweep** answers "what does changing *X* do to my
  envelope?" directly — pick CG height, weight distribution, camber, roll
  stiffness, downforce or power and see the metric move. (Camber sweeps force the
  camber lever live even with geometry attached, so a flat curve only ever means
  genuine tire insensitivity, never a silent geometry override.)
- **Combined-slip aware:** drop in a `CombinedSlipTire` and the longitudinal axis
  uses its calibrated longitudinal/lateral mu ratio while the envelope corners
  take its friction-ellipse exponents (a superellipse, not a forced circle) — the
  same object the lap sim consumes, so there's one tire model, not two.
- **Cross-checked against the lap sim:** `GGVParams.from_powertrain()` builds the
  GGV from the same `Powertrain` the lap sim uses, and `validate_against_laptime()`
  compares their axis limits directly. They agree to **under 0.1%** on lateral and
  acceleration across downforce levels; the one deliberate, *documented* exception
  (the lap sim's brake side ignores the longitudinal mu ratio) is detected and
  reported rather than hidden. The tab exposes this as a one-click check.
- Same honesty contract as the rest: envelope *shape* and its *response to setup*
  are trustworthy out of the box on the generic tire; absolute g's become real
  once you load your TTC-fitted tire. Never raises — a bad point clamps and warns,
  and inner-wheel lift at the limit is flagged as an artifact, not sold as grip.

**Transient solver (the unsteady half of the lap — NEW)**
- The thing QSS assumes away. The **◢ TRANSIENT** tab runs an explicit,
  high-frequency time-step solver that integrates the full vehicle DAE — planar
  yaw/sideslip, sprung heave/pitch/roll, four unsprung wheel-hops, and lateral
  tire relaxation — **millisecond by millisecond** (explicit RK4 @ 1 ms) on the
  *same* tire, damper and geometry the rest of the tool uses.
- It shows the behaviour a quasi-steady model structurally can't: **turn-in lag
  and yaw overshoot** (a step steer that overshoots its steady yaw then settles),
  **snap-oversteer and the countersteer that catches it** (a trailing-throttle
  slide that spins uncaught but is pulled back by a state-feedback countersteer),
  **pitch and dive** through a brake→throttle transition (the sprung mass rocking,
  the digressive damper settling it), and **kerb strikes** (the unsprung mass
  hopping at ~15–20 Hz, the contact load spiking and dropping to zero — wheel
  lift). It also contrasts the transient corner build-up against the QSS steady
  number directly (rise time, overshoot, settle).
- Honest scope, same as everywhere else: it resolves the dominant transient
  modes; longitudinal force is demanded and friction-ellipse-limited rather than
  spun up as full slip-ratio wheel states, and tire thermal state and a
  closed-loop racing line are out of scope — flagged, not faked. Use QSS for the
  lap-time number; use this for the unsteady behaviour behind it. Same
  never-crash contract: every run returns a flagged result with warnings rather
  than raising. Built on the verified `damper.py` / relaxation-length primitives
  and covered by `tests/test_transient.py` (37 checks).

**EV powertrain & energy — the architecture choice, decided in seconds (NEW)**
- KinematiK was born combustion: the lap sim carried a single `power_w` cap.
  That can't answer the expensive, hard-to-reverse question an FSAE-EV team has
  to get right *once*: **one motor + diff, two motors (axle split), or four
  hub/upright motors (full torque vectoring)?** The **EV layer**
  (`suspension/ev_powertrain.py`) runs your *live* car through all three
  architectures on the same track and ranks them in the only currencies that
  decide the EV events: **seconds and kWh**.
- It wraps the existing QSS lap sim at the one seam where architecture enters —
  corner-exit traction. The traction-limit difference is modelled from first
  principles: an **open diff** is capped by the lateral-load-unloaded inside
  driven wheel, an **axle split** shares but still has no left/right control,
  and **per-wheel torque vectoring** deploys each wheel to its own load-limit.
  That delta is the real, defensible reason TV buys lap time, and it's computed,
  not asserted.
- **Honest about what QSS can't earn:** the *yaw-moment* benefit of L/R torque
  vectoring (using a torque difference to rotate the car) is real but is a
  closed-loop control behaviour a point-mass cannot resolve — so it is reported
  as a **separate, clearly-flagged upper bound** and **never folded into the lap
  time**. Same philosophy as the CFD and FTire seams: no fabricated number to
  fill a hole.
- **Each architecture carries its own mass.** More motors + inverters weigh
  more, and that mass costs lap time, so TV must pay for its own weight before it
  shows a net gain. In the default run this is the headline: the four-motor car's
  traction edge is nearly eaten by its +16 kg, and the two-motor axle-split car
  wins on raw lap time — TV only pulls ahead once the separately-reported yaw
  benefit is counted. That trade, quantified, *is* the design-event argument.
- **Energy budget + regen + pack sizing:** integrates net tractive energy from
  the QSS speed/long-g trace through the inverter+motor efficiency, returns
  braking energy via a driven-axle-capped regen model, and tells you whether the
  pack outlasts the 22 km endurance distance — and if not, the planning-grade
  lap-time penalty of derating power to finish. Size the pack you can afford
  instead of brute-forcing it with the pack you can't.
- Make it yours by setting two dicts to your real numbers: `mass_delta_kg`
  (your motor/inverter weights) and `drive_grip_frac` (your measured/estimated
  corner-exit traction per architecture) — the comparison lives there. Same
  never-crash contract throughout; covered by `tests/test_ev_powertrain.py`
  (27 checks).

**Tire & grip (the thing that actually wins skidpad and the limit in autocross)**
- Full Magic Formula lateral model wired into the whole grip/balance stack — not a
  linear placeholder. Load sensitivity and camber response come from the curve, not
  a guess.
- A real TTC fitter: `process_ttc.py` cleans a cornering `.mat` and fits the MF5.2
  lateral coefficients, writing a private JSON you load straight into the tool.
- Grip-curve plots (μ vs load, μ vs camber) so you can read the optimal camber and
  the load-transfer cost off your actual tire.

**Lap-time simulator (the score, not the proxy)**
- Everything else reports grip at one operating point; competition is won on **lap
  time** — a transient, track-dependent integral of that grip. A funded team buys
  that integral by testing fresh rubber all year; on one tire set you predict it.
- Runs your **live** geometry, setup and tire around the **FSAE skidpad**
  (near closed-form, ~4.6–5.2 s band — sanity-check it by hand) and a
  **representative autocross**, via a quasi-steady-state point-mass model on the
  same grip envelope the rest of the tool already trusts.
- Simple, defensible longitudinal model (power/traction cap, drag, downforce,
  rolling resistance, friction-circle coupling) so straights and corner exits are
  realistic without pretending we have a motor map we don't.
- Change a hardpoint or a setup lever, re-run, read the **skidpad delta in
  seconds** — that delta is the number to defend a design decision with, and it
  pairs with the optimiser: optimise for grip, then confirm it's worth time here.
- Never crashes the session: a non-convergent linkage or a degenerate track
  returns a flagged safe default and a UI warning, not a stack trace.

**Setup optimiser (spend your one tire set wisely)**
- Sensitivity ranking: every setup knob (weight bias, CG height, roll-stiffness
  split, static camber) ranked by **grip gained per unit change** and its balance
  effect — so an underfunded team tunes the levers that matter, not the ones that
  feel important.
- A transparent coordinate search that finds the setup maximising limit grip while
  holding balance in a target window (mild understeer = fast and safe). It reports
  the trade it made and can push the result to the sidebar / decision log.

**Chassis fit & manufacturing check (load your STEP/STL)**
- Fit check: do the inboard pickups land on the frame where a bracket can mount?
- Clearance check: sweep the linkage through full travel and find the minimum
  distance from every moving link to the chassis — flags collisions before you cut tube
- 3D overlay of the swept linkage on the chassis mesh
- Export a manufacturing pickup schedule (coordinates + link lengths) for the fab team

**Multi-team integration (any subteam, any part)**
- Generic part-vs-chassis interference check: load the shared chassis once, load any
  part (caliper, radiator, battery box, wing mount, ECU tray), get collision / tight /
  clear back with the worst point highlighted
- Position parts in the shared frame with offset + rotation
- Same workflow for every Elbee subteam — aero, brakes, cooling, data-acq, electrics,
  powertrain, suspension. The idea: a team that can't out-spend its rivals wins by not
  wasting parts on rework. Catch interference in CAD before the first cut.

**Parametric mount-point clash + CG propagation (the CAD→clash→CG chain — NEW)**
- The interference checks above test a *whole loaded part* against the chassis mesh.
  This closes the faster inner loop a subteam lives in: move **one mounting hardpoint**
  a few millimetres and get, in a single call, both consequences at once —
  - **the clearance clash** against every keep-out volume another subteam's master file
    reserves (the chassis main hoop, the driver legroom box, the accumulator, a cooling
    duct), flagged as hard interference (FAIL) or inside-the-clearance-band (WARN), with
    **both subteams named as owners** so the conflict has an owner, and
  - **the updated car CG**, re-rolled through the same integration ledger the
    vehicle-dynamics model reads, so geometry and the load-transfer number can never
    drift out of sync.
- Geometry is explicit and honest: a `MountPoint` in car coordinates and a `KeepOut`
  axis-aligned box, checked with an **exact analytic point-to-box signed distance**
  (zero CAD-kernel dependency, validated to closed form). A point that legitimately
  **bolts onto** a structure is allowed to touch it; everything else it must clear.
- Provenance carries through: a point or box flagged *estimated* taints the finding as
  estimated rather than presenting placeholder geometry as final — the same
  no-false-confidence rule the rest of KinematiK keeps.
- Lives in the **◢ INTEGRATION** tab under the *Mount-point clash* view, and **persists
  with the project** (`project.json`) so the points, keep-outs and last move survive a
  restart. `update_interface_cg` opt-in shifts a subsystem's declared CG with the point
  only when that point genuinely is the part's mass location — off by default, because a
  single bracket usually isn't.

**Custom-PCB copper survival + signal integrity (the pre-fab electrical gate — NEW)**
- The same integration ledger names every subsystem's peak current and supply voltage, but
  it stops at "does the LV bus have enough headroom". This closes the inner loop the
  electrical sub-team lives in the afternoon before a board goes to fab: pick a **trace
  width** and route the **CAN differential pair**, and get, in one call, whether the board
  survives its worst moment.
- **Copper survival**, against the worst *simultaneous* load (the brake light and both
  cooling fans firing at once):
  - **Onderdonk fusing current** — the amperage at which the trace physically *melts*,
  - **IPC-2221 steady-state temperature** — does it cook past the board's derate ceiling,
  - **IR-drop / ECU brown-out** — does the voltage drop under that load pull the rail below
    the microcontroller's brown-out threshold and reset the car mid-event. A trace that
    fuses before steady state is reported as an open circuit, not merely a brown-out — the
    two findings can't physically contradict each other.
- **Signal integrity**: an **edge-coupled microstrip differential-impedance** estimate
  (IPC-2141) checked against the pair's target (120 Ω for CAN), and a **geometric coupling
  check** of every signal pair against the switching **HV motor-controller / inverter** net
  — closest approach and coupled run length — so a CAN pair routed too close to the
  high-voltage switching node is flagged with **both owners named** (electrics ↔ powertrain).
- The worst-case current per trace is **rolled up from the integration ledger's declared
  `peak_current_a`**, not retyped in the electrical tab — so "what fires at once" and what
  TEAM FIT says a subsystem draws can never drift apart.
- Honest about its limits, like the rest of KinematiK: it is an **analytic screening**
  layer, **not a PCB CAD kernel and not a field solver**. The impedance and coupling
  numbers are labelled estimates; the things that genuinely need a 2-D field solver / SPICE
  — the true eye height, insertion loss, reflection coefficient, coupled-noise waveform —
  are returned as **`None` / "not computed"** rather than invented (the same contract the
  structural-tire co-sim backend uses). The IPC temperature curve is clamped at copper's
  melting point so it can never print a non-physical number.
- Lives in the **◢ ELECTRONICS (PCB)** tab and **persists with the project** (`project.json`)
  so traces, pairs, aggressor nets and the load scenario survive a restart.

**Weight budget & handover (persistent team memory)**
- Per-team weight budget with a running total against a target mass; mass estimated
  from CAD volume + material or entered manually, with per-subteam breakdown
- TEAM FIT can push a part's CAD-estimated mass straight into the budget in one click
- Design-decision log — capture *why* a choice was made, not just what, as you go
- Interference checks auto-offer to log the problem to the decision log
- One-click handover report exported to Markdown, PDF, and JSON, bundling the
  suspension design state, weight budget, decision log, and any open cross-team items
- Everything persists to `project.json` in the project folder — commit it to the repo
  and the knowledge survives graduation instead of dying in a senior's spreadsheet

**Lead notes (cross-team comms that don't go stale)**
- Notes addressed to a specific team (or broadcast to all), with author, timestamp,
  an open/resolved status, and urgent / action-requested flags
- Open-item counts per team so a lead sees what's blocking them at a glance
- The point vs Discord: a note here is tied to the work, addressed to a team, and
  tracked until resolved — which is how you stop two finished parts not fitting

**Workflow**
- Live 3D view of the corner
- Export setup as JSON, export the travel sweep as CSV for your report plots

## Quick start

```bash
git clone <your-fork-url> kinematik && cd kinematik
pip install -r requirements.txt
streamlit run app.py
```

Then edit hardpoints in the sidebar (millimetres, SAE axes: **x** rearward, **y** to the right, **z** up). The default geometry is a representative front corner you can tune from.

### Sharing it with the team (tunnel testing)

Before deploying anywhere, you can let teammates use your local instance through a
tunnel. With the app running on port 8501:

```bash
# any one of these
cloudflared tunnel --url http://localhost:8501
ngrok http 8501
npx localtunnel --port 8501
```

Share the URL it prints. `.streamlit/config.toml` already disables XSRF/CORS and
raises the upload cap to 200 MB so the CAD file uploader works through the tunnel —
local testing won't reveal upload failures that only happen over a forwarded host,
so test an actual STEP upload through the tunnel before relying on it. Re-enable
XSRF protection before any real public deployment.

## Using the engine without the UI

The solver is a clean importable package — drop it into your own lap-sim or optimiser:

```python
from suspension import SuspensionKinematics, Hardpoints, VehicleDynamics, VehicleParams
from suspension import default_tire
from suspension.tiremodel import load_from_json
from suspension.setup import sensitivity, optimise

kin = SuspensionKinematics(Hardpoints.default())
print(kin.static.camber, kin.static.caster, kin.static.scrub_radius)

# Real motion ratio from the pushrod/rocker, wheel rate from a spring rate,
# and anti-dive / anti-squat from the side-view geometry:
print("motion ratio:", kin.motion_ratio(), "(real)" if kin.motion_ratio_is_real() else "(proxy)")
print("wheel rate @35 N/mm spring:", kin.wheel_rate(35.0), "N/mm")
print("anti-dive %:", kin.anti_dive_pct(cg_height=300, wheelbase=1550, brake_bias_front=0.65))

# Grip/balance on the generic default tire (works out of the box) ...
tire = default_tire()
# ... or on YOUR tire fitted from TTC data:
# tire = load_from_json("my_tire.json")

veh = VehicleDynamics(VehicleParams(), front_kin=kin, rear_kin=kin, tire=tire)
print("grip model:", veh.grip_model_name())          # "Pacejka MF5.2"
print("max lateral g:", veh.max_lateral_g())
print("balance index:", veh.balance_index(1.2)[0])    # + understeer, − oversteer

# Sweep the PHYSICAL levers (spring rates/ARB flow through the motion ratio into
# roll stiffness; sensitivity()/optimise() set use_spring_rates automatically):
for r in sensitivity(VehicleParams(), front_kin=kin, rear_kin=kin, tire=tire)["rankings"]:
    print(f"  {r['label']}: {r['d_maxg_per_step']:+.4f} g per {r['step']} {r['unit']}")

# Validate the model against a real skidpad run — earn trust by matching data:
from suspension import correlation
rep = correlation.correlate_skidpad(veh, measured_g=1.42)
print(rep.summary)                 # measured vs predicted, % error, trust verdict
print("within tolerance:", rep.overall_within_tol)
```

The GGV diagram is importable the same way — generate the full envelope, sweep a
design lever, and cross-check it against the lap sim, all from the live vehicle:

```python
from suspension.ggv import GGVGenerator, GGVParams, sweep_parameter, quick_ggv
from suspension import laptime

# One-call envelope from the headline numbers (no kinematics object needed):
res = quick_ggv(mass=280, cg_height=300, cl_a=2.5, power_w=60_000)
print("peak lateral g:", round(res.max_lat_g.max(), 2))
print("peak braking g:", round(res.max_brake_g.max(), 2))

# Or drive it from the live vehicle + the SAME Powertrain the lap sim uses, so
# the two share one source of truth (combined-slip tire flows through too):
pt  = laptime.Powertrain(power_kw=80, cla=2.6, cda=1.1, drive="rwd")
gp  = GGVParams.from_powertrain(pt)
res = GGVGenerator(veh, gp).generate()          # full (speed, direction) surface

# "What does changing X do to my envelope?" — X is any VehicleParams/GGVParams field
s = sweep_parameter(veh, gp, "cg_height", [250, 300, 350, 400],
                    speed=20.0, metric="max_lat_g")
print("max lat g vs CG height:", [round(m, 3) for m in s["metric"]])

# Earn trust the same way the skidpad does — prove the GGV agrees with the lap sim:
from suspension.ggv import validate_against_laptime
v = validate_against_laptime(veh, pt)
print("agrees within tol:", v["ok"], "| worst diff: %.2f%%" % (v["max_reldiff"] * 100))
```

The mount-point clash + CG chain is just as importable — move one hardpoint and get the
clearance clash and the new CG back in a single call:

```python
from suspension import (GeometryLedger, MountPoint, KeepOut, propagate_mount_move)
from suspension.interfaces import IntegrationLedger, SubsystemInterface

# the chassis engineer's master file reserves the main-hoop tube volume
geom = GeometryLedger()
geom.set_keepout(KeepOut("main-hoop", "chassis",
                         lo_mm=(1380, -180, 480), hi_mm=(1430, 180, 1050),
                         is_estimate=False))
# aero's rear-wing upper mount, currently clear
geom.set_point(MountPoint("rear-wing-upper-mount", xyz_mm=(1350, 120, 900),
                          owner_subsystem="aerodynamics", mounts_on="suspension",
                          min_clearance_mm=8.0, is_estimate=False))

# the vehicle-dynamics ledger (mass + CG) the load-transfer model reads
led = IntegrationLedger(target_mass_kg=230.0)
led.set(SubsystemInterface("aerodynamics", mass_kg=12.0,
                           cg_x_mm=1450, cg_y_mm=0, cg_z_mm=520, is_estimate=False))
led.set(SubsystemInterface("chassis", mass_kg=32.0,
                           cg_x_mm=820, cg_y_mm=0, cg_z_mm=300, is_estimate=False))

# aero drags the mount 60 mm rearward — clash + CG propagate in one call
res = propagate_mount_move(geom, led, "rear-wing-upper-mount", (1410, 120, 900))
print(res.summary())               # "...HARD CLASH flagged. CG moved ... mm in z."
for f in res.clash_findings:
    print(f"  [{f.severity.value.upper()}] {f.message}")
```

The PCB copper-survival + signal-integrity gate is importable the same way — size a
trace, route the CAN pair past the inverter, and check the board against the worst
simultaneous load in one call:

```python
from suspension import (Trace, DiffPair, Aggressor, BoardLedger, check_board)
from suspension.interfaces import IntegrationLedger, SubsystemInterface

# peak currents declared once, on the integration ledger (the single source of truth)
led = IntegrationLedger()
led.set(SubsystemInterface("cooling", peak_current_a=8.0, voltage_v=12.0, is_estimate=False))
led.set(SubsystemInterface("brakes",  peak_current_a=2.0, voltage_v=12.0, is_estimate=False))

board = BoardLedger(rail_nominal_v=5.0, ecu_brownout_v=4.5, ambient_c=40.0)
# the trace that feeds the ECU rail
board.set_trace(Trace("main_feed", net="lv_rail", owner_subsystem="electrics",
                      feeds="ecu", width_mm=0.15, copper_oz=1.0, length_mm=150.0,
                      is_estimate=False))
# the CAN pair, and the HV inverter net it must avoid
board.set_pair(DiffPair("CAN", owner_subsystem="electrics",
                        path_mm=[(0, 0), (60, 0)], target_z0_ohm=120.0))
board.set_aggressor(Aggressor("INV", owner_subsystem="powertrain", net="hv_inverter",
                              sw_voltage_v=400.0, edge_v_per_ns=8.0,
                              path_mm=[(0, 0.3), (60, 0.3)]))

# worst case: brake light + BOTH cooling fans at once -> 8 + 8 + 2 = 18 A on the feed
res = check_board(board, led, scenario={"main_feed": ["cooling", "cooling", "brakes"]})
print(res.summary())               # "FAIL: ... fail / ... warn / ... ok ..."
for f in res.findings:
    print(f"  [{f.severity.value.upper()}] {f.check}: {f.message}")
# the undersized feed FAILs on fusing, heating and ECU brown-out;
# the CAN pair FAILs on coupling to the 400 V inverter net.
```

## Flexible bodies & compliance (ADAMS Flex-style)

The rigid solver freezes every link length. The compliance layer relaxes that: it
finds the axial load in each member at a cornering case, lets the links stretch by
their stiffness, and re-solves the geometry under load. You get the **compliance
toe and camber** — the steer/camber the wheel runs that isn't in your kinematics.

The fastest way in is the **◢ COMPLIANCE (FLEX)** tab: pick a lateral g, a tube
size, optionally tick chassis-tab compliance, and read the deflected toe/camber and
the per-member force/deflection. From code:

```python
from suspension import (SuspensionKinematics, Hardpoints,
                        VehicleDynamics, VehicleParams)
from suspension import CompliantCorner, MemberStiffness, corner_wheel_load

hp  = Hardpoints.default()
kin = SuspensionKinematics(hp)
veh = VehicleDynamics(VehicleParams(), front_kin=kin, rear_kin=kin)

# Easiest: every link the same tube, optional chassis-tab stiffness in series.
corner = CompliantCorner.uniform_tube(hp, od_mm=19.05, wall_mm=0.9, k_tab=8000.0)

# Drive it straight off the real load-transfer model at the headline 1.5 g case:
res = veh.corner_compliance(1.5, corner=corner)     # front-outer wheel
print(f"compliance toe   {res.compliance_toe:+.3f} deg")   # the compliance steer
print(f"compliance camber{res.compliance_camber:+.3f} deg")
print("converged:", res.converged, "in", res.summary()['iterations'], "iters")
print("member forces (N):", res.member_forces)             # + tension, − compression
```

Want one link at a time? Pass a per-member stiffness map; members you omit stay
rigid, so you can isolate (say) the tie rod and watch only compliance steer move:

```python
stiff = {"TR": MemberStiffness(k_direct=1200.0)}   # N/mm; everything else rigid
res = CompliantCorner(hp, stiff).solve(
        corner_wheel_load(veh, "front", 1.5, outer=True))
```

A member's stiffness can come from three sources — a number you already have
(`k_direct`), an analytic tube (`material, od_mm, wall_mm` → `E·A/L` on the link's
length), or a condensed **FEA flex body**. Add `k_tab` to put a chassis-tab/bracket
stiffness in series with any of them.

### Non-linear joints: bushings, rod ends & spherical-bearing lash

Relaxing the link is only half the story — the rigid solver also treats every
*joint* as a perfect, zero-play constraint. Real cars don't have those. A
`JointCompliance` gives any connection its own **non-linear force-vs-displacement
curve** plus a **damping** coefficient, so you can model what's actually bolted in:

```python
from suspension import JointCompliance, CompliantCorner, corner_wheel_load

bushing  = JointCompliance.rubber_bushing()          # soft, progressive, high loss
# or .polyurethane_bushing() — stiffer, less progressive, lower loss
rod_end  = JointCompliance.spherical_bearing(lash_mm=0.05)   # micro-yield / clearance

cc  = CompliantCorner.with_bushings(hp, bushing=bushing, rod_end=rod_end)
res = cc.solve(corner_wheel_load(veh, "front", 1.5, outer=True))
print(res.compliance_toe, res.contact_patch_lateral_shift_mm)   # secondary steer, track compliance
print(res.member_joint_deflection["TR"])   # {'link':…, 'joint_in':…, 'joint_out':…} mm
```

Each joint is a non-linear spring **in series** with its link along the load line
(a pin-jointed two-force member transmits axial force through its end joints), so the
member's total give is `link + joint_in + joint_out` and feeds the same solver — the
**compliance steer** off a soft tie-rod bushing and the **track-compliance** lateral
patch shift off the wishbone joints fall straight out. Curve shapes available:
`linear`, `cubic` (progressive, the elastomer shape), `bilinear`, `freeplay` (the
lash dead-band of a worn rod end), and `tabular` for a measured rig curve. Build any
member by hand to mix them:

```python
stiff = {"TR": MemberStiffness(material="Steel 4130", od_mm=12.0, wall_mm=1.0,
                               joint_in=JointCompliance.rubber_bushing(),
                               joint_out=JointCompliance.spherical_bearing())}
```

**Where damping lives (honestly).** Damping is a rate term — it does no work in a
steady, zero-velocity corner, so it never touches the static compliance number.
It is surfaced two truthful ways: `corner.damping_summary(load, amplitude_mm, freq_hz)`
reports the hysteresis energy lost per cycle, and `corner.linearized_rates(load,
freq_hz)` exports each joint's tangent stiffness + equivalent viscous rate for the
transient DAE solver to consume. A bushing's off-axis (transverse) give carries no
load under the two-force idealisation, so it is modelled along the load line, not
fed back as geometry — stated, not hidden.

### Importing an FEA component (the "Flex" part)

A flexible body is a `.flex.json` in one of two schemas. Two ready samples ship in
[`examples/`](examples/) — a lower A-arm as a beam **mesh** and the same arm as a
**reduced** superelement.

**1. Mesh** — give nodes, beam/bar elements and the interface (attachment) nodes;
KinematiK assembles and **Guyan-condenses** it to the interface for you:

```json
{ "type": "mesh",
  "nodes": [ {"id": "lower_front_inner", "xyz": [-110, 200, 122.5]},
             {"id": "lower_ball",        "xyz": [  -5, 575, 110]} ],
  "elements": [ {"n1": "lower_front_inner", "n2": "lower_ball",
                 "kind": "beam", "material": "Steel 4130",
                 "od_mm": 25.4, "wall_mm": 1.65} ],
  "interface": { "lower_front_inner": "lower_front_inner",
                 "lower_ball": "lower_ball" } }
```

**2. Reduced** — a pre-condensed superelement: interface nodes + the condensed
stiffness matrix. This is the portable form an **ADAMS Flex MNF**, a Craig–Bampton
boundary reduction, or a DMIG export already carries; KinematiK uses it verbatim:

```json
{ "type": "reduced", "dofs_per_node": 6,
  "interface": [ {"name": "lower_front_inner", "xyz": [-110, 200, 122.5]},
                 {"name": "lower_ball",        "xyz": [  -5, 575, 110]} ],
  "K_condensed": [[ ... 12 x 12 ... ]] }
```

Load either and map its nodes onto a member:

```python
from suspension import load_flex_body
body = load_flex_body("examples/lower_a_arm.flex.json")
stiff = {"LF": MemberStiffness(flex_body=body,
                               node_out="lower_ball", node_in="lower_front_inner")}
```

**Honest scope.** A production `.mnf` is a proprietary binary holding the interface
data *and* the fixed-interface normal modes used for transient/NVH. KinematiK
imports the **static (constraint-mode) stiffness** — exactly what governs
load↔deflection in a *sustained* corner — and `read_mnf` raises a clear, actionable
error on a binary file instead of guessing. Export the reduced superelement (the
boundary stiffness) as JSON and the numbers are identical; only the packaging
differs. This is a steady-state, quasi-static compliance model: no damper dynamics,
no modal response, no kerb strikes.

## Your tire is the edge

You can only afford one set of tires. A funded team tests rubber all year; you
can't. So the entire equaliser is extracting maximum truth from the tire data you
*are* allowed — the FSAE Tire Test Consortium — and making every geometry and setup
decision against it before you commit the set you bought.

```bash
# Fit a full MF5.2 lateral model to your TTC cornering file (stays local/private):
python process_ttc.py path/to/your_cornering.mat my_tire.json
```

Then upload `my_tire.json` in the **TIRE & GRIP** tab. The grip, balance, and setup
optimiser instantly run on your measured tire instead of the generic default. The
`.mat` files and the fitted `.json` are TTC-confidential and are gitignored — ship
the code, never the numbers.

## Persistent storage (so handover data survives)

By default the project memory (decisions, notes, weight budget, the mount-point /
keep-out geometry ledger, and the PCB trace / differential-pair / aggressor board)
saves to a local `project.json` file. That's fine on a
laptop, but on ephemeral hosts like Streamlit
Community Cloud the filesystem is wiped on restart — so for a deployed app the team
relies on, point it at a free hosted database.

KinematiK auto-detects [Supabase](https://supabase.com) (free Postgres). To enable it:

1. Create a free Supabase project.
2. In the SQL editor, create the table:
   ```sql
   create table kinematik_project (
     id text primary key,
     data jsonb
   );
   ```
3. Copy your project URL and a service/anon key from Supabase settings.
4. In Streamlit Cloud → your app → Settings → Secrets, add:
   ```toml
   SUPABASE_URL = "https://yourproject.supabase.co"
   SUPABASE_KEY = "your-key"
   ```
   (Locally, set the same two as environment variables.)

The app picks up the credentials automatically and switches to persistent storage —
the WEIGHT & HANDOVER tab shows a green "persistent storage" badge when it's active,
or an amber "local/session" badge when it's not. No credentials → it just uses the
local JSON file, exactly as before. Nothing breaks either way.



Each corner is a rigid double-wishbone linkage. The two ball joints must lie on the spheres defined by their wishbone lengths, the upright is rigid between them, and the tie-rod outer is rigidly tied to the upright. KinematiK drives the lower ball joint through vertical travel and solves the resulting nonlinear constraint system with a damped least-squares (Levenberg–Marquardt) step at each position. The upright's rigid pose is then transported to the wheel-centre, contact patch, and spin axis, so camber/toe/caster are read from the *actual* moving wheel rather than approximated. See `suspension/kinematics.py` — it's commented for exactly this reason.

## Validate it

Sign conventions and gains are pinned by tests:

```bash
python tests/test_kinematics.py        # kinematics sign conventions & solver
python tests/test_tiremodel.py         # tire model, TTC fitter, setup optimiser
python tests/test_ggv.py               # GGV envelope, combined slip, lap-sim cross-check
python -m pytest tests/                # everything (170+ tests)
```

The tire tests pin the things the grip upgrade depends on: load sensitivity in the
right direction, the fitter recovering a known tire from noisy data, and the
optimiser never returning a setup worse than where it started.

Before you trust it for a design decision, sweep one corner against your existing
OptimumK/spreadsheet model and check the camber curve matches. If it doesn't, that's
a bug worth a GitHub issue.

## The interface that other tools don't have (SUBSYSTEM INTEGRATION tab)

OptimumK, ANSYS and SolidWorks each go deep in **one** domain. What no FSAE team has is
a place where the **interfaces between** subsystems are owned and checked — so eight
sub-teams optimise in isolation and the integration failures (the radiator that won't
fit the duct, the motor torque that exceeds the driveline, eight "~12 kg" estimates that
sum well over budget) surface at assembly or at competition, when they're expensive.

The SUBSYSTEM INTEGRATION tab (and `suspension/interfaces.py`) is a live integration
ledger. Each of the eight subsystems declares, in typed fields, what it **needs from**
the car and what it **provides to** it — mass + CG, spatial envelope, mount loads,
power draw, heat/airflow, torque, downforce. KinematiK then runs cross-subsystem
consistency checks and reports `Finding`s with a severity (`FAIL` / `WARN` / `MISSING` /
`INFO` / `OK`) that name **both** subsystems involved, so each conflict has an owner:

- mass budget vs target (net of a declared driver allowance) and combined **mass-weighted
  CG** — which is pushed straight into the vehicle model so load transfer and the lap sim
  reflect the real build, not an assumption;
- spatial **envelope fit** of each subsystem inside the chassis interior;
- **cooling airflow** required vs what the cooling package can move;
- **LV power** draw vs supply, and HV voltage match;
- **driveline torque** the powertrain delivers vs what the driveshaft/CV/upright is rated for;
- mount loads vs design loads.

Crucially it **does not simulate any subsystem** — KinematiK can't do CFD, brake-thermal,
chassis FEA or battery modelling, and faking those would be the same false-confidence trap
the rest of the codebase refuses. Each subsystem's analysis stays in the tool that does it
properly; this owns the channels between them. Every declaration carries an `is_estimate`
flag, and the board always surfaces which numbers are placeholders, so a green board never
implies more certainty than the data behind it. That coordination layer — not deeper
single-domain physics — is the edge.

**It doubles as living documentation.** Each interface carries a `rationale` ("why these
numbers"), an owner, and a last-updated stamp; every edit is auto-logged to the handover
record as it happens. `build_interface_markdown()` exports the whole contract — values,
rationale, provenance, the combined mass/CG, and the integration findings — as a
design-event-ready document, so the design justification judges ask for is captured as
the team works rather than scrambled together before the report deadline. Estimates and
checks passing on placeholder data are marked as such in the export, so the document is
honest about its own maturity.

## Correlate it against real data (the VALIDATION tab)

A sim only changes a decision if people believe it, and the honest way to earn that
is to show it predicted something you measured. The **VALIDATION** tab (and
`suspension/correlation.py`) takes data a cash-strapped team can actually collect and
reports the gap in plain, checkable numbers:

- **Skidpad** — enter your measured peak lateral g *or* timed-circle time; it reports
  the error on both channels against the live grip model. This is the cleanest case:
  steady-state and near closed-form, so a mismatch here means the grip stack is off,
  not the lap integration.
- **Acceleration (75 m)** — compares your measured run against a standing-start
  integration of the longitudinal model (`laptime.acceleration_time`).
- **Speed trace** — upload a two-column `distance, speed` CSV from GPS or a wheel-speed
  log; the sim trace is resampled onto your distance axis and compared point-for-point,
  reporting RMSE, **mean bias** (does the sim run systematically fast or slow?),
  peak-speed error, and R².

It deliberately does **not** tune the model to fit your data — it quantifies the gap
and tells you which way the model is biased, so you either trust the prediction for the
decision in front of you or go find the assumption that's wrong. Tolerances live in
`DEFAULT_TOL` in `correlation.py`; they're explicit and editable, and every report
carries the tolerance it used. A correlation can be logged straight to the handover
record so the *evidence* travels with the design decision — which is what actually
settles an argument, rather than the loudest opinion in the room.

## Roadmap / good first PRs

- **Transient response** — turn-in and pitch built on the relaxation-length and damper
  primitives now in the codebase (`tiremodel.apply_relaxation_lag`, `damper.py`). This
  is the next real step up in fidelity.
- **Calibrate the data-gated models**: fit combined-slip ellipse exponents to drive/brake
  TTC runs (`CombinedSlipTire`), relaxation length to transient runs, and the damper law
  to your dyno (`DamperCurve.from_dyno_points`). The code is in and flagged uncalibrated
  until you do — that's deliberate.
- Pull-rod and decoupled (third-spring) layouts (the pushrod/rocker module in
  `suspension/kinematics.py` is the place to extend)
- Aligning-moment (Mz) from the tire data to model steering feel and self-centering
- Full minimum-time racing line (the current one is curvature-optimal; couple the speed
  solver into the offset optimisation for the true min-time line)

Recently shipped (was on this list): real pushrod/rocker **motion ratio** and
**anti-dive / anti-squat**; **GPS/cone track import**; **racing-line optimisation**;
a real **motor map**; **combined slip**, **relaxation length** and a **damper model**
(the last three implemented honestly and gated on your data); a **validation tab**
that correlates the sim against measured skidpad / accel / datalogger traces; and
**flexible-body compliance** — link/tab deflection and FEA (ADAMS Flex-style)
import giving compliance steer/camber at the cornering limit.

### A note on honesty over a green scorecard

Several of these (combined slip, relaxation length, damper, tyre thermal) *cannot* be
made quantitatively correct without test data this project doesn't ship — Fx runs, step
inputs, dyno pulls, temperature sweeps. The code implements the real physics and exposes
an `is_calibrated`/`status()` flag that stays false, with representative magnitudes,
until you supply that data. That is intentional: a model that prints a confident number
it didn't earn is worse than an honest gap, because someone freezes a design on it. The
capability is here and turns on the moment you have the data; it will not pretend in the
meantime. A tyre **thermal** model is the one remaining red that is *not* built, for
exactly this reason — it needs temperature-swept TTC data to be anything but a guess.

## Conventions

| | |
|---|---|
| Units | millimetres, degrees, newtons, kg |
| Axes | x rearward +, y right +, z up + (SAE) |
| Camber | negative = top leaning inboard |
| Toe | positive = toe-out |
| Caster | positive = kingpin top rearward |
| Balance index | + understeer, − oversteer |

## License

MIT. Built for the FSAE community — fork it, use it on your car, send improvements back.
