# MathWorks' K&C rig: someone else's model, someone else's definitions

front_v11_final and rear_v11_final on the quarter-car kinematics-and-compliance rig of MathWorks' Simscape Vehicle
Templates (`testrig_quarter_car_doublewishbone`, MATLAB R2025b), run through the rig's own test sequence and scored by
MathWorks' own metric script (`sm_car_knc_calc_susp_metrics`). Nothing on the MathWorks side is KinematiK code:
`scripts/matlab/kk_knc_simscape.m` only writes the recorded hardpoints into the rig's vehicle data.

Re-run: in MATLAB, `kk_knc_simscape(kkroot, svtroot)`; then `python scripts/simscape_knc_benchmark.py record`.
`results.json` is the record; `tests/test_simscape_knc_benchmark.py` checks it against the rig's files.

## Result

Limits set before the first run and not changed (in the script's docstring). MathWorks' sign conventions: toe + in,
camber + top out, x forward.

| measure | front: MathWorks / KinematiK | rear: MathWorks / KinematiK | limit | result |
|---|---|---|---|---|
| static toe, camber | exact | exact | 0.01° | pass |
| bump steer (deg/m, ±10 mm) | −0.2996 / −0.2996 | 0.2131 / 0.2130 | 0.05 | pass (0.00003) |
| bump camber (deg/m) | −34.7312 / −34.7312 | −28.8550 / −28.8549 | 0.05 | pass (0.00003) |
| roll steer, roll camber (deg) | −0.0048, −1.4586 / same | −0.0080, −1.2016 / same | 0.01° | pass (< 0.00001) |
| toe and camber curves, whole post sweep | | | 0.01° | pass (< 0.00001) |
| damper ratio | 0.6880 / 0.6914 | 0.6920 / 0.6928 | 0.005 | pass (0.0034, 0.0008) |
| caster (deg) | 3.5738 / 3.6556 | 2.7759 / 2.8624 | 0.05° | **fail** |
| KPI (deg) | 7.4081 / 7.3130 | 8.9524 / 8.6008 | 0.05° | **fail** |
| caster trail (mm) | 16.37 / 13.00 | 10.61 / 12.26 | 0.5 mm | **fail** |
| scrub radius (mm, from below the wheel centre) | −0.17 / 0.33 | 6.32 / 7.83 | 0.5 mm | pass / **fail** |

Every quantity the rig measures directly passes. The four that fail are not measured: MathWorks estimates them from
the steering test. The damper-ratio gap on the front (0.0034 of a 0.005 limit) is not explained here.

## Where the fails come from

**The motion is the same.** KinematiK at the rig's own rack positions, with the wheel centre held at design height as
the rig holds it, reproduces the rig's steering test (to 12.3° of toe on the front, 9.3° on the rear): toe within
2e-7°, wheel-centre path within 4e-5 mm, camber within 2e-5°.

**MathWorks' formulas on KinematiK's motion give MathWorks' numbers.** Applying `sm_car_knc_calc_susp_metrics`' own
procedure to KinematiK's steered wheel (`mathworks_formulas`) returns the rig's caster, hub offsets and trail:

| | front: rig / formulas on KinematiK | rear: rig / formulas on KinematiK |
|---|---|---|
| caster, swing method at ±1° of toe (SAE 850219) | 3.5738 / 3.5738 | 2.7759 / 2.7759 |
| hub longitudinal offset, Pratt circle fit (mm) | 1.8036 / 1.8036 | −0.6953 / −0.6953 |
| hub lateral offset (mm) | 30.1560 / 30.1561 | 43.0562 / 43.0562 |
| caster trail as MathWorks writes it (mm) | 16.3703 / 16.3702 | 10.6134 / 10.6134 |

So the caster and trail gaps are the estimates, not the kinematics. Two of them in particular:

- **Trail.** MathWorks computes `hubLongOffset + tand(caster) * h`. With x forward, caster top rearward and the hub
  offset defined as wheel centre minus axis point, the axis meets the ground `tand(caster) * h` ahead of the axis
  point at hub height, so the geometry gives `−hubLongOffset + tand(caster) * h`. The formula appears to add the
  offset where it should subtract it. With the geometric sign it gives 12.76 mm (front) and 12.00 mm (rear), within
  0.26 mm of KinematiK's 13.00 and 12.26; the remainder is the circle fit, which treats the wheel centre's path
  around an inclined axis as a horizontal circle. Not yet reported to MathWorks.
- **Scrub.** MathWorks computes `|hubLatOffset| − tand(KPI) * h` with its own KPI estimate. With KinematiK's KPI the
  same formula gives 0.22 mm (front) and 7.78 mm (rear), within 0.11 mm of KinematiK's 0.33 and 7.83. The rear scrub
  fail follows from the rear KPI gap.
- **KPI** is a second swing-method estimate using wheel-frame angles (`aToeX`, `aCamberX`) that run 3 of the harness
  did not log, so it is not reproduced here. `kk_knc_simscape.m` now logs them for the next run.

The fails stay fails: the score is the score, and these are explanations, not adjustments.

## What it found: two definitions of camber

KinematiK prints camber as the front-view projection of the spin axis, `−atan(s_z / |s_y|)`. The rig measures the
inclination of the wheel plane, `−asin(s_z)`, as ISO 8855 and SAE J670 define it. They agree at zero toe and differ
once the wheel is steered: against the rig's steering test the projection is 0.058° (front) and 0.039° (rear) off;
the inclination is within 2e-5°. Every earlier steering benchmark used the projection on both sides, so none could see
it.

Switching KinematiK to the inclination was tried and reverted: Lotus reports the projection (KinematiK's camber in
steer matches Lotus to 0.05° with it and misses by 0.77° with the inclination, `tests/test_lotus_benchmark.py`), and
recorded designs, walls and the wheel envelope are built on it. So KinematiK keeps printing the projection and now
also gives the inclination, `suspension.kinematics.camber_inclination(camber, toe)`. When comparing camber in steer
with another tool, check which of the two it reports.

## Harness history, disclosed

Runs 1 and 2 were not of these corners as designed: the rig's `PostLoadFcn` reloads MathWorks' own FSAE corner
whenever the model loads, which replaced the rear (run 1) and both corners (run 2); in run 1 the front also sat
5.23 mm below the rig's design height. The harness now loads the model first, raises the corner rigidly to the rig's
tyre radius, and checks the assembled camber, toe and wheel-centre position. Since run 2, KinematiK's trail and scrub
are taken on the rig's own ground plane. Results here are run 3. The limits were not changed.

## Files

- `<corner>_metrics.csv`: MathWorks' metrics table, as `sm_car_knc_plot1toecamber` returns it.
- `<corner>_curves.csv`: MathWorks' toe and camber curves over the post sweep.
- `<corner>_reference.csv`: the rig's tyre radius, start position and test settings.
- `<corner>_steer.csv`: the steering test at design height (t 27.5–48.6 s), cut from the run log.
- `<corner>_timeseries.csv`: the whole run log, 3.4 MB each; not committed (`.gitignore`), regenerated by the MATLAB
  script.
- `results.json`: every comparison and both diagnostics.
