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

**Open-source double-wishbone suspension studio for Formula SAE.**
Edit your hardpoints, see the kinematics *and* the vehicle-level consequences update together — in the browser, for free.

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
  check the car.
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

By default the project memory (decisions, notes, weight budget) saves to a local
`project.json` file. That's fine on a laptop, but on ephemeral hosts like Streamlit
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
python -m pytest tests/                # everything (173 tests)
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
