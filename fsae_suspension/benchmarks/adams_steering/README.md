# Adams View and Project Chrono: steer over rack travel, lock and Ackermann

The kinematics benchmark sweeps wheel travel; this one sweeps the rack. Each corner (`front_v11_final`,
`front_v11_joints50`) is held at ride height (lower wishbone at its static angle) and steered by a rack on a
translational joint, -30 to +30 mm in 0.25 mm steps. Compared with KinematiK: toe and camber of the steered wheel at
every station, and, read from the toe curve as KinematiK reads its own (mirrored axle, the manifest's rack c-factor,
lock, lock basis, wheelbase and track), the steering-wheel angle at lock and the Ackermann there. Limits, set before
any result: 0.005 deg on toe and camber, 0.1 deg at the steering wheel, 0.1 percentage point of Ackermann.

## Run (Adams View)

1. File > Select Directory: the folder you want the results in (e.g. your Desktop).
2. File > Import > Adams View Command File: `steer_all.cmd`. It builds and runs both corners and writes the six
   `<corner>_steer__<measure>.tab` files (the per-corner `_steer.cmd` files do the same one corner at a time).
3. If the tables are not there, import `export_both.cmd`, which writes them from the runs in the session.
4. Copy them here and run `python scripts/adams_steering_benchmark.py`.

Project Chrono (`chrono_steering.csv`) is written by `scripts/chrono_geometry_benchmark.py`.

## Results

Project Chrono 9.0.1: pass on both corners. Toe and camber within 1e-9 deg, steering-wheel angle at lock within
0.00004 deg, Ackermann at lock within 0.0003 percentage point (front_v11_final: 118.9 deg, 75.27 %). Adams View 2026.1
(run by the author): pass on both corners; toe and camber within 0.00005 deg, lock within 0.0002 deg, Ackermann within
0.0003 point.
