# KinematiK against Adams View

The three corners of the paper, solved in Adams View 2026.1 (Student Edition) and compared with KinematiK at every 1 mm
of wheel-centre travel from −25 to +25 mm.

| File | What it is |
|---|---|
| `<corner>.cmd` | Adams View command file, generated from the corner's manifest by `scripts/adams_view_benchmark.py --write` |
| `<corner>.tab` | The table Adams View exported after the run (PostProcessor, File > Export > Table, spreadsheet), unedited apart from line endings (Windows to Unix) and trailing blank lines |

The model is the bare linkage: rigid wishbones on revolute inner pivots, spherical ball joints, a tie rod with a
spherical joint at the upright and a Hooke joint at the fixed rack end, and a jack that drives wheel-centre height
through an inplane primitive so the wheel centre stays free in x and y. Kinematic analysis, 50 steps, no masses. It is
built in Adams View from these files, not from the Adams/Car `_double_wishbone` template.

Limits, set before any Adams result existed: camber, caster and kingpin inclination 0.01°, toe 0.005°, wheel-centre
x and y 0.05 mm. Largest differences found:

| Corner | Camber (°) | Toe (°) | Caster (°) | KPI (°) | Wheel centre x (mm) | Wheel centre y (mm) |
|---|---|---|---|---|---|---|
| front_v11_final | 0.000026 | 0.000004 | 0.000009 | 0.000004 | 0.000025 | 0.000027 |
| rear_v11_final | 0.000002 | 0.000005 | 0.000005 | 0.000003 | 0.000013 | 0.000025 |
| front_v11_joints50 | 0.000027 | 0.000004 | 0.000010 | 0.000006 | 0.000017 | 0.000027 |

The differences are at the size of KinematiK's interpolation onto wheel-centre travel and Adams' solver tolerance.
The front table carries the model name `front_v11_final_v3`: that run used an earlier command file with the same
geometry, markers and measures; the current file adds only the two lines that select and reset the model before the
run.

What this shows: an independent commercial multibody solver gets the same camber, toe, caster, kingpin inclination
and wheel-centre path from the same hardpoints. What it does not show: that a built car matches its hardpoints. That
needs the K&C or rig measurement in the validation plan.

Re-run: `python scripts/adams_view_benchmark.py` (compare) and `pytest tests/test_adams_view_benchmark.py`. To repeat
the Adams side, import a `.cmd` file in Adams View (File > Import > Adams View Command File), then export the seven
measures from the PostProcessor as a table.
