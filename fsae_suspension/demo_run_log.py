#!/usr/bin/env python3
# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""
demo_run_log.py — consolidate the wings team's ANSYS run log.

    python demo_run_log.py                      # built-in demo sheet
    python demo_run_log.py sample.xlsx          # your own export
    python demo_run_log.py sample.xlsx -o out/  # choose the output folder

Reads the run log the aero team keeps, screens every row against the physics and
mesh-quality criteria in ScreenConfig, averages what survives per operating point,
and writes an organised workbook plus a CSV bundle.
"""

import argparse
import os
import sys

from suspension.aero.run_log import (
    ScreenConfig, process, write_workbook, write_csv_bundle,
    to_coeff_results,
)


# --------------------------------------------------------------------------- #
#  A demo sheet with the failure modes a real first-iteration log contains
# --------------------------------------------------------------------------- #
_BANNER = ["Wings Team Simulation Results"] + [None] * 8 + ["Volume Mesh Metrics"]
_HEADER = [
    "Contributor", "Front or Rear Wing?", "Ride-Height (mm)", "Velocity (m/s)",
    "Desired Y+", "Min Surface Mesh Length", "Max Surface Mesh Length",
    "First Layer Height (m)", "Number of Layers", "Min Orthogonal Quality",
    "Max Skewness", "Max Aspect Ratio", "Viscous Model", "Scheme", "Order",
    "Pseudo Time Step", "Courant Number", "Initialization", "Lift Force (N)",
    "Lift Coefficient", "Drag Force (N)", "Drag Coefficient", "Max Pressure (Pa)",
    "Min. Pressure (Pa)", "Mass Imbalance (kg/s)", "Average Y+", "Notes",
]

_Q = 0.5 * 1.225 * 26.8224 ** 2      # dynamic pressure at 26.8224 m/s (60 mph)
_AREA = 0.268                        # reference area the team normalises by


def _run(who, rh, cl, *, yplus=45.0, model="k-epsilon", ortho=0.30, skew=0.70,
         mesh=(0.006, 0.012), area=_AREA, imbalance=7.3e-6, cp_max=1.02,
         notes=None, cd=0.199):
    """One row, built so its forces and coefficients are mutually consistent."""
    return [
        who, "Front Wing", rh, 26.8224, 40, mesh[0], mesh[1], 6.267e-4, 8,
        ortho, skew, 1346.37, model, "Simple", "Second", "Disabled", "Disabled",
        "Standard",
        cl * _Q * area, cl, cd * _Q * area, cd,
        cp_max * _Q, -3.4 * _Q, imbalance, yplus, notes,
    ]


DEMO_SHEET = [
    _BANNER, _HEADER,
    # --- 50 mm ride height: the early iterations, then the good ones --------
    _run("Khalil - Test", 50, -0.815, notes="first go, ignore"),
    _run("Khalil", 50, -0.809, mesh=(0.005, 0.004)),          # min > max: typo
    _run("Khalil", 50, -0.812, yplus=6.0),                    # in the sublayer
    _run("Adriane", 50, -0.826),
    _run("Adriane", 50, -0.831),
    _run("Rohan", 50, -0.824),
    _run("Rohan", 50, -0.829),
    # --- 40 mm: a wrong reference area and a broken stagnation pressure -----
    _run("Adriane", 40, -0.885),
    _run("Rohan", 40, -0.879),
    _run("Priya", 40, -0.891),
    _run("Priya", 40, -1.770, area=_AREA / 2),                # half the area
    _run("Sam", 40, -0.883, cp_max=0.25),                     # wrong ref velocity
    _run("Sam", 40, -0.888),
    # --- 30 mm: only one run survives, and the tool says so -----------------
    _run("Priya", 30, -0.940, ortho=0.04),                    # degenerate cells
    _run("Sam", 30, -0.933),
]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("source", nargs="?", default=None,
                    help="run-log .xlsx/.csv (default: built-in demo sheet)")
    ap.add_argument("-o", "--outdir", default="out_run_log",
                    help="output folder (default: out_run_log)")
    ap.add_argument("--sheet", default=None, help="worksheet name, if not the first")
    ap.add_argument("--area", type=float, default=None,
                    help="reference area in m2 (default: infer from the rows)")
    ap.add_argument("--keep-test-rows", action="store_true",
                    help="warn about scratch rows instead of excluding them")
    args = ap.parse_args()

    cfg = ScreenConfig(reference_area_m2=args.area,
                       reject_test_rows=not args.keep_test_rows)

    source = args.source or DEMO_SHEET
    if args.source and not os.path.exists(args.source):
        print(f"no such file: {args.source}", file=sys.stderr)
        return 2

    report = process(source, cfg, sheet=args.sheet)

    print("=" * 78)
    print(report.summary())
    print("=" * 78)

    if report.parse_warnings:
        print("\nParse warnings:")
        for w in report.parse_warnings:
            print(f"  ! {w}")

    print("\nWhy runs were excluded")
    print("-" * 78)
    for v in report.rejected:
        print(f"  {v.row.label():<44} {v.reason()}")
    if not report.rejected:
        print("  (nothing was excluded)")

    print("\nConsolidated results")
    print("-" * 78)
    for c in report.cases:
        print(f"  {c.summary()}")
        if c.notes:
            print(f"      note: {'; '.join(c.notes)}")

    os.makedirs(args.outdir, exist_ok=True)
    xlsx = os.path.join(args.outdir, "aero_consolidated.xlsx")
    try:
        write_workbook(report, xlsx)
        print(f"\nWorkbook : {xlsx}")
    except ImportError as exc:
        print(f"\nWorkbook skipped: {exc}")
    for p in write_csv_bundle(report, args.outdir):
        print(f"CSV      : {p}")

    results = to_coeff_results(report)
    print(f"\n{len(results)} consolidated point(s) ready for AeroMap:")
    for r in results:
        cd = "n/a" if r.c_drag is None else f"{r.c_drag:.4f}"
        print(f"  h={r.attitude.ride_height_mm:g}mm  Cl={r.c_lift:+.4f}  Cd={cd}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
