# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""
FlexGen demo — design a titanium flexure blade to replace a lower ball joint,
headless, in well under a second.

    python3 demo_flexgen.py

Walks the full loop: synthesize the lightest blade that survives the declared
corner case, sweep it through travel, read the equivalent-spring rate the
blade contributes, downsize the coilover accordingly, and write the STEP/STL
blank plus the review report next to this script.
"""
from __future__ import annotations

from suspension.flexgen import (
    BladeLoadCase, PRBChain, coilover_downsize, equivalent_spring,
    export_step, export_stl, flexgen_lint, layup_map, render_flexgen_md,
    synthesize_blade,
)

TRAVEL_MM = 5.0            # bump/droop each way at the blade tip
AXIAL_N = -450.0           # compressive axial component at max corner
K_WHEEL_TARGET = 32.0      # N/mm wheel-rate target for the corner
MOTION_RATIO = 0.95


def main() -> None:
    print("FlexGen demo — synthesizing a Ti-6Al-4V flexure blade")
    res = synthesize_blade(
        "Titanium Ti-6Al-4V", width_mm=40.0, travel_mm=TRAVEL_MM,
        cases=[BladeLoadCase("max corner", axial_n=AXIAL_N)],
        length_range_mm=(70.0, 120.0), t_range_mm=(0.8, 3.0),
    )
    if not res.feasible:
        print("  " + res.message)
        return
    blade = res.blade
    print(f"  winner: {res.message}  (searched {res.searched} candidates)")

    spring = equivalent_spring(blade, TRAVEL_MM, axial_preload_n=AXIAL_N,
                               n_pts=9)
    p_cr = PRBChain(blade).critical_axial_load_n()
    print(f"  rate at ride: {spring.k_at_ride_n_mm:.2f} N/mm | strain energy "
          f"at full travel: {spring.energy_at_full_j * 1000:.0f} N·mm | "
          f"P_cr {p_cr:.0f} N (margin {p_cr / abs(AXIAL_N):.1f}x)")

    down = coilover_downsize(K_WHEEL_TARGET, spring.k_at_ride_n_mm,
                             MOTION_RATIO)
    print(f"  flexure supplies {down['flex_share'] * 100:.0f} % of the "
          f"{K_WHEEL_TARGET:.0f} N/mm target -> residual physical spring "
          f"{down['k_spring_residual_n_mm']:.1f} N/mm")

    findings = flexgen_lint(blade,
                            [BladeLoadCase("max corner + full bump",
                                           axial_n=AXIAL_N,
                                           travel_mm=TRAVEL_MM)])
    for f in findings:
        print(f"  [{f.level:7s}] {f.code}: {f.detail}")

    for name, content in ((f"{blade.name}.step", export_step(blade)),
                          (f"{blade.name}.stl", export_stl(blade)),
                          (f"{blade.name}_orientation.csv", layup_map(blade)),
                          (f"{blade.name}_flexgen.md",
                           render_flexgen_md(blade, spring, findings, down))):
        with open(name, "w") as fh:
            fh.write(content)
        print(f"  wrote {name}")


if __name__ == "__main__":
    main()
