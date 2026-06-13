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
- Instant-centre location and motion ratio

**Vehicle dynamics (coupled to the geometry)**
- Front/rear roll-centre heights from the solved instant centres
- Steady-state lateral load transfer, split into geometric + elastic
- Per-tire vertical loads vs lateral g
- **Pacejka MF5.2 tire model** → load-sensitive, camber-aware grip, max lateral g,
  and an **understeer/oversteer balance index**. Ships with a sensible generic FSAE
  tire so it works out of the box, and loads a tire **fitted to your own TTC data**
  the moment you have one — see "Your tire is the edge" below.

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

# Grip/balance on the generic default tire (works out of the box) ...
tire = default_tire()
# ... or on YOUR tire fitted from TTC data:
# tire = load_from_json("my_tire.json")

veh = VehicleDynamics(VehicleParams(), front_kin=kin, rear_kin=kin, tire=tire)
print("grip model:", veh.grip_model_name())          # "Pacejka MF5.2"
print("max lateral g:", veh.max_lateral_g())
print("balance index:", veh.balance_index(1.2)[0])    # + understeer, − oversteer

# Which setup change buys the most grip?
for r in sensitivity(VehicleParams(), front_kin=kin, rear_kin=kin, tire=tire)["rankings"]:
    print(f"  {r['label']}: {r['d_maxg_per_step']:+.4f} g per {r['step']} {r['unit']}")
```

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
python -m pytest tests/                # everything (67 tests)
```

The tire tests pin the things the grip upgrade depends on: load sensitivity in the
right direction, the fitter recovering a known tire from noisy data, and the
optimiser never returning a setup worse than where it started.

Before you trust it for a design decision, sweep one corner against your existing
OptimumK/spreadsheet model and check the camber curve matches. If it doesn't, that's
a bug worth a GitHub issue.

## Roadmap / good first PRs

- Rear-corner **anti-squat / anti-dive** percentages from side-view geometry
- Pushrod/rocker module so motion ratio comes from real rocker geometry
- Combined-slip (longitudinal + lateral) so the tire model covers braking/traction,
  not just steady-state cornering — the lateral MF5.2 is in (`suspension/tiremodel.py`)
- Transient response: turn-in, trail-braking, and damper behaviour on top of the
  steady-state balance model
- Pull-rod and decoupled (third-spring) layouts
- Aligning-moment (Mz) from the tire data to model steering feel and self-centering

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
