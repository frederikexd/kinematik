# Adams View: anti-geometry path, scrub, trail and the rocker

Extends the kinematics benchmark (`../adams_view`) to what the reports quote and that benchmark does not reach, on
`front_v11_final` and `rear_v11_final`: the contact patch's side-view path (anti-dive / anti-squat are computed from
it), scrub radius and mechanical trail (where the kingpin axis meets the road through the contact patch), and the
rocker (pushrod spherical at its outer end, Hooke at the rocker, rocker on a revolute about its axis) with the damper
length from the rocker's spring point to the chassis mount. Limits, set before any Adams result: 0.05 mm on every path
and length, 0.1 percentage point on the anti-geometry at ride height.

## Run

1. Adams View, File > Import > Adams View Command File: `front_v11_final_geo.cmd` (then `rear_v11_final_geo.cmd`).
   Each builds the model, runs the kinematic sweep and writes `<corner>_geo__<measure>.tab` for travel, cp_dx, cp_dz,
   scrub, trail and damper_len in Adams' working directory.
2. Copy the 12 `.tab` files here, unedited.
3. `python scripts/adams_geometry_benchmark.py` prints the worst difference per measure and PASS/FAIL, for Adams and
   for Project Chrono (`chrono_geometry.csv`, written by `scripts/chrono_geometry_benchmark.py`, already run) and MATLAB Simscape Multibody (`matlab_geometry.csv`, written by
   `kk_bench_simscape(root, 'geometry')` in `scripts/matlab/kk_bench_simscape.m`, already run).

Regenerate the command files with `python scripts/adams_geometry_benchmark.py --write`.

If the tables do not appear, run both `_geo.cmd` files in one session, set File > Select Directory to the folder you want them in and import `export_both.cmd`, which writes all 12. One PostProcessor table per corner (File > Export > Table, the six measures, saved as `<corner>_geo.tab`) is read too.

## Results

Adams View 2026.1 (run by the author), Project Chrono 9.0.1 and MATLAB Simscape Multibody R2025b: pass on both corners. Worst against KinematiK: Adams 1.2e-4 mm (rear damper length; Adams' 7-digit output), Chrono 3.4e-5 mm, Simscape Multibody R2025b 2.6e-5 mm; anti-geometry within 0.0007 percentage point.
