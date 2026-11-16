# Random geometry: KinematiK against Project Chrono

500 corners drawn at random around front_v11_final and rear_v11_final (seed 20261116): every pickup moved independently
by up to ±20 mm (inboard), ±15 mm (ball joints, tie rod) or ±10 mm (pushrod, rocker, damper). Camber, toe, caster,
kingpin inclination, scrub, mechanical trail, the contact-patch path and the damper length are compared over ±25 mm of
wheel-centre travel, against the limits set for Adams View before any run (0.005°, 0.05 mm).

## Run

    python scripts/random_geometry_benchmark.py draw 500
    ./chrono/bin/python scripts/random_geometry_benchmark.py chrono
    python scripts/random_geometry_benchmark.py compare

## Result

Pass on all 500. Worst: angles 0.000025°, lengths 0.00013 mm. `summary.json` is the record; the curves (about 10 MB)
are regenerated from the seed.

## What it does not show

Both sides use KinematiK's definitions from the same hardpoints, so this finds solver and implementation errors on
geometries nobody chose, not a wrong definition (see benchmarks/published_lotus for another tool's definitions). Of the
512 corners drawn, 12 did not solve over ±25 mm in KinematiK and were replaced; whether Chrono solves them has not been
checked yet.
