<!--
  KinematiK: Formula SAE / Formula EV pre-validation platform
  Copyright (c) 2026 Frederik Thio. AGPL-3.0.
-->

# KinematiK 

![tests](https://github.com/frederikexd/kinematik/actions/workflows/tests.yml/badge.svg)
![AI Assisted, Human Directed](https://img.shields.io/badge/AI%20assisted-human%20directed-blue)
![Not AI Slop](https://img.shields.io/badge/not%20AI%20slop-verified-brightgreen)
![1578 Tests](https://img.shields.io/badge/physics-1578%20tests%20passing-brightgreen)

Design tools for Formula SAE and Formula SAE EV teams. Suspension kinematics,
aero, brakes, accumulator, chassis, cooling, electrics and powertrain in one place, sharing
one set of numbers.

**[Open the app](https://kinematik-ijzpvutb92x7n3yzo7sdqc.streamlit.app/)** · Free for student teams · No install needed

---

## What it's for

You make design decisions before you have CFD or FEA results. KinematiK is the
step in between: fast enough to try twenty ideas, honest enough to tell you when
it can't answer.

It is not a replacement for ANSYS, ADAMS or MATLAB. It is what you run *before*
them, so the two concepts you take to a real solver are the two worth the hours.

## What it does

| | |
|---|---|
| **Suspension** | 3D constraint solver: double wishbone, MacPherson, multi-link, solid axle. Camber gain, bump steer, roll centre migration, load transfer. |
| **Aero** | Underfloor duct model, vortex lattice for wings, source-panel method for bluff bodies. Upload an STL, get a ride-height sweep. |
| **Brakes** | Bias, lock-up, hydraulic sizing, rotor thermal and fade. |
| **Accumulator** | Cell sizing, pack topology, FSAE-EV rules checks, thermal model. |
| **Chassis** | Frame graph with triangulation and load-path audit, weight and CG ledger. |
| **Electronics** | Import a real routed board (KiCad, Altium) and check copper survival and signal integrity. |
| **Integration** | Every subsystem declares mass, CG, torque, heat and load into one ledger. Change a number, see which other subsystems it moves. |

Most tabs export a build-ready DXF you can open in SolidWorks as a 2D sketch.

Full list: **[FEATURES.md](FEATURES.md)**

## Aero example

Upload an STL, pick a solver, get a ride-height sweep:

The what-if grid solves 30 combinations of throat position and ride height in
milliseconds, with the diffuser separation limit marked:

```
 ride       25%       35%       45%       55%       65%
   60      -36%      -37%      -38%      -39%      -40%
   40       +6%       +3%       +0%       -3%       -6%
   25     +102%      +92%      +84%      +76%      +68%
   20     +180%     +164%     +152%     +139%     +126%
```

Ten millimetres of ride height is worth more than any throat position available.
That's the kind of answer this is for.

## Run it locally

```bash
git clone https://github.com/<your-org>/kinematik.git
cd kinematik
pip install -r requirements.txt
streamlit run fsae_suspension/streamlit_app.py
```

Python 3.11+. The hosted app needs no install.

---

## What it doesn't do

**The physics is not validated against a measured car.** The solvers are checked
against closed-form cases and against each other: a sphere in potential flow
matches the analytic pressure distribution to 0.4%, a rectangular AR 8 wing lands
within 15% of lifting-line theory, but nothing here has been correlated with an
instrumented vehicle or a wind tunnel. Treat every number as a starting point for
your own test day.

**Absolute levels are weaker than comparisons.** On a cambered body the panel
method's absolute C_L drifts a few percent per mesh refinement while the *ratio*
between two geometries holds to 3%. Compare designs at matched settings; don't
quote a single figure.

**It tells you when it can't answer.** Diverged cases are blanked rather than
printed. Grid checks that couldn't run say so instead of reporting a reassuring
0%. Every failure message names the lever that's actually available for your
case.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md). Issues and pull requests welcome.
Deployment and database notes are in [DEPLOY.md](DEPLOY.md); release
history is in [CHANGELOG.md](CHANGELOG.md).

## License

AGPL-3.0: free to use, fork and build on. Modifications must be shared under
the same licence.

Original work of Frederik Thio. © 2026.
