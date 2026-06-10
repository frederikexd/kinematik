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
- Load-sensitive grip model → max lateral g and an **understeer/oversteer balance index**

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

kin = SuspensionKinematics(Hardpoints.default())
print(kin.static.camber, kin.static.caster, kin.static.scrub_radius)

veh = VehicleDynamics(VehicleParams(), front_kin=kin, rear_kin=kin)
print("max lateral g:", veh.max_lateral_g())
print("balance index:", veh.balance_index(1.2)[0])   # + understeer, − oversteer
```

## How the solver works

Each corner is a rigid double-wishbone linkage. The two ball joints must lie on the spheres defined by their wishbone lengths, the upright is rigid between them, and the tie-rod outer is rigidly tied to the upright. KinematiK drives the lower ball joint through vertical travel and solves the resulting nonlinear constraint system with a damped least-squares (Levenberg–Marquardt) step at each position. The upright's rigid pose is then transported to the wheel-centre, contact patch, and spin axis, so camber/toe/caster are read from the *actual* moving wheel rather than approximated. See `suspension/kinematics.py` — it's commented for exactly this reason.

## Validate it

Sign conventions and gains are pinned by tests:

```bash
python tests/test_kinematics.py        # or: python -m pytest tests/
```

Before you trust it for a design decision, sweep one corner against your existing OptimumK/spreadsheet model and check the camber curve matches. If it doesn't, that's a bug worth a GitHub issue.

## Roadmap / good first PRs

- Rear-corner **anti-squat / anti-dive** percentages from side-view geometry
- Pushrod/rocker module so motion ratio comes from real rocker geometry
- Pacejka tire model instead of the linear load-sensitivity placeholder
- Roll-centre migration plot vs travel (the IC math is already there)
- Pull-rod and decoupled (third-spring) layouts

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
