# Suspension test without a team: a physical double-wishbone corner

KinematiK's suspension solver is checked against other software (Adams, GT-SUITE). This checks it against a
physical linkage you can hold. Geometry doesn't depend on scale, so a 1:8 or 1:10 RC corner with real double
wishbones tests the same mathematics as a full-size FS corner. A skeptic can still say "it's not an FS car".
This test shows that the solver matches a real mechanism. It does not show anything about a real FS car's
compliance or loads.

## Pick a corner that is a real double wishbone

- The upper and lower arms each pivot on a hinge pin, or on two inner pivots. That gives each arm a fixed pivot
  axis, which is what KinematiK models.
- **Avoid a single upper turnbuckle ("camber link") with one ball joint at each end.** That isn't a wishbone,
  and KinematiK will not model it correctly.
- The steering (tie-rod) link must have ball ends and be lockable, so toe doesn't wander while you measure.
- Your own 3D-printed or machined corner works too, and is easier to measure: build it from your CAD points,
  then measure the built part. Don't use the CAD numbers.

## Measure the hardpoints

1. Fix the chassis to a flat reference: glass, a granite plate, or a straight aluminum bar clamped down. Choose
   axes as in KinematiK: x rearward, y outward (right side), z up, all in mm.
2. Measure every point KinematiK needs with digital calipers and a square or height gauge, from the same datum.
   Every point is the center of a pivot or ball, not its edge:
   - upper and lower inner pivots, front and rear;
   - upper and lower outer ball joints;
   - tie-rod inner and outer;
   - wheel center;
   - contact patch.
3. Measure each point three times and use the average. The spread of your three readings gives your measuring
   error (`sigma_mm`). Expect about 0.1–0.3 mm.
4. Write the points to `front_hardpoints.json` in the format of `example/front_hardpoints.json`. Also enter the
   measured static camber and toe as `static_camber` and `static_toe`.

## Fix the tolerance before you measure the curve

```
python fsae_suspension/scripts/team_test.py tolerance --plan my-rc/plan.json --sigma-mm 0.2 --gauge-deg 0.1
```

This moves every hardpoint at random by your measuring error, many times, and reports how far camber and toe can
move from that alone. It then combines that with your gauge's accuracy. Put the result in the plan as
`tolerance`. The tolerance now comes from your measuring method and is fixed before any result exists.

## Seal, measure, score

1. Set `travel_mm` to about ±60 % of the corner's travel. For a 1:10 car with ±10 mm of travel, use
   `[-6, -3, 0, 3, 6]`.
2. Seal, and push the record or post its SHA-256 publicly:
   `python fsae_suspension/scripts/team_test.py seal --plan my-rc/plan.json --out my-rc/SEALED.json`
3. Remove the spring or damper so the corner moves freely. Raise the wheel to each travel, measured at the
   wheel center relative to the chassis datum. At each travel:
   - **Camber:** a digital angle gauge flat on the rim face. A phone inclinometer works if you calibrate it on
     the plate first.
   - **Toe:** clamp a straightedge to the rim face and measure its angle to the chassis x-axis at two points
     about 100 mm apart.
4. Enter the readings in `measured.csv` as `camber_front_+3mm`, `toe_front_+3mm` and so on. Then score:
   `python fsae_suspension/scripts/team_test.py score --record my-rc/SEALED.json --measured my-rc/measured.csv --out my-rc`

Publish the scorecard whether it passes or not. A miss outside the tolerance means a measuring mistake, a linkage
that isn't the wishbone you modeled, or a solver error. Find out which, and say so.

## A second corner, A vs B

To test a design change, move one point and run it as setup B: a different tie-rod hole, or a spacer under the
outer tie-rod ball. KinematiK predicts how bump steer changes before you measure. That's the A vs B test on
geometry, and it needs no car to drive.
