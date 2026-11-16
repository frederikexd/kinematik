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

## A bug it found

The first run drew 512 corners: 12 did not solve over ±25 mm in KinematiK and were set aside. Chrono solved all 12. The
cause was in KinematiK: where a warm-started sweep crosses the static rocker angle, the rocker solve was seeded with
about 1e-17 rad, and MINPACK's finite-difference step is relative to the variable, so the Jacobian was noise and the solve
failed. It never printed a wrong number (a failed solve returns "not solved" or NaN), but valid corners could show as
unsolvable. The solve now starts from an offset of exactly 0 about the seed. On those 12 corners KinematiK now agrees with
Chrono within 0.00007 mm, and `tests/test_rocker_seed_regression.py` pins one of them
(`rocker_seed_regression.json`). The rerun above drew 500 corners with none set aside.

## What it does not show

Both sides use KinematiK's definitions from the same hardpoints, so this finds solver and implementation errors on
geometries nobody chose, not a wrong definition (see benchmarks/published_lotus for another tool's definitions).
