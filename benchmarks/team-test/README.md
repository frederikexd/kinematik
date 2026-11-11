# Blind test on your own car

The prediction ledger tests KinematiK against what teams publish. This tests it against what matters for
prevalidation: your car, your inputs, and the decision between two setups. It runs on your next test day.

## What you need

- Your car's numbers: mass with driver, CG height, wheelbase, tracks, weight split, roll stiffness, powertrain
  (power, drive, ratio, redline, wheel radius), aero (ClA, CdA) if you have it. Every input is a field of
  `VehicleParams` or `Powertrain`; an unknown field is refused, so a typo can't silently do nothing.
- Your tire file (a Pacejka coefficient JSON), if you have one. It stays private: the sealed record holds only its
  SHA-256, never the coefficients.
- Two or more setups you'll actually run: ballast, an ARB change, a different tire, ride height, a lower CG. Each
  is a set of overrides on the base car.
- Optionally your test course as an x, y centreline in meters (`course.csv`), to predict its lap time.

## Suspension: camber and toe against your hardpoints

Add a `kinematics` block to the plan, one entry per corner you'll measure. Each entry gives the corner's
hardpoints, either inline as `hardpoints` or in a private file as `hardpoints_json` (recorded by hash only), and
the wheel travels to measure at (`travel_mm`, default −25, −10, 0, +10, +25). KinematiK predicts
`camber_<corner>_<travel>mm` and `toe_<corner>_<travel>mm` in degrees. Points use KinematiK's SAE axes (x rear,
y right, z up, mm, right-hand corner).

**Measuring it in the garage, in about two hours with no rig:**
1. Remove the spring, or fit a solid link, so the corner moves freely. Put the car on stands at ride height.
2. Jack the wheel to each travel. Measure the travel as the vertical movement of the wheel center relative to the
   chassis, + for bump.
3. At each travel, measure camber with a digital angle gauge on the rim face, negative when the top of the wheel
   leans inboard. Measure toe with strings or toe plates, positive for toe-out.
4. Enter the measured static camber and toe in the hardpoints file as `static_camber` and `static_toe`.
   KinematiK takes the static wheel angle as an input and predicts how it **changes** with travel: camber gain and
   bump steer. The 0 mm values therefore match by construction. The test is every other travel.
5. Set `tolerance` in the plan before the test, for example `{"camber": 0.3, "toe": 0.1}` degrees. Set it from
   your gauge's accuracy plus your build tolerance, not from the results.

A setup can change the geometry with per-corner overrides, for example a moved outer tie-rod point to change bump
steer: `"kinematics": {"front": {"tie_rod_outer": [90, 560, 145]}}`. Then the A vs B rows test whether KinematiK
predicts the change. A shim that only changes static camber is entered as a new `static_camber`. Its A vs B
change is the input itself, so it tests nothing about the model.

## Steps

1. Copy `example/plan.json` and fill it in. Put the measurement noise you expect per channel under `noise`
   (for example, 0.02 s for skidpad). A measured difference smaller than that counts as a tie, not a hit.
2. Seal it **before the test day**, then commit and push the sealed file, or post its SHA-256 in a public GitHub
   issue. That timestamp is what makes the test blind.

   ```
   python fsae_suspension/scripts/team_test.py seal --plan my-car/plan.json --out my-car/SEALED.json
   ```

3. Run the setups and log the channels: `skidpad_s` (one timed circle), `accel_s` (75 m from rest), `lap_s` (your
   course), `max_lat_g`, and the camber/toe values. Put them in `measured.csv` (columns setup, channel, value;
   repeat runs are averaged).
4. Score:

   ```
   python fsae_suspension/scripts/team_test.py score --record my-car/SEALED.json \
          --measured my-car/measured.csv --out my-car
   ```

The scorecard gives the error on every value: times in %, camber and toe in degrees against your tolerance. For
every pair of setups it also says whether KinematiK predicted the right direction and how far off the size of the
change was. Nothing is refitted. Publish it either way.

`verify --record my-car/SEALED.json` checks that the record is unchanged, that it was made by the same code with
the same input files, and that it reproduces.

## No team? A physical linkage

[`RC_LINKAGE.md`](RC_LINKAGE.md) runs the suspension part of this test on an RC car or a corner you build. It also
explains how `tolerance` sets the pass band from your measuring error before you measure.

## The example

`example/` shows the format. Its `measured.csv` holds **invented numbers**, not data from any car. It is there so
the commands run end to end.
