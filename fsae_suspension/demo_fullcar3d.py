# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Demo: assemble and export the full-vehicle 3D view (no Streamlit needed).
# ============================================================================

"""
Build the whole-car 3D model from the default corner geometry + vehicle params
and write it to an interactive HTML file you can open in any browser.

    python demo_fullcar3d.py [output.html]

This uses exactly the same renderer the Streamlit "FULL CAR 3D" tab uses
(suspension.fullcar3d.build_full_car_figure), so what you see here is what the
app shows once every field is filled out.
"""

import sys

from suspension.kinematics import Hardpoints, SuspensionKinematics
from suspension.dynamics import VehicleParams
from suspension.fullcar3d import build_full_car_figure


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "fullcar.html"

    # Pull the same defaults the app starts with. Swap these for a loaded
    # project.json (Hardpoints.from_dict / VehicleParams(**...)) to render YOUR car.
    hp = Hardpoints.default()
    vp = VehicleParams()

    # A quick sanity solve so any bad geometry is reported before rendering.
    SuspensionKinematics(hp)

    fig = build_full_car_figure(hp, vp)
    fig.write_html(out, include_plotlyjs="cdn")
    print(f"Wrote {out}")
    print(f"  wheelbase   {vp.wheelbase:.0f} mm")
    print(f"  track F/R   {vp.track_front:.0f} / {vp.track_rear:.0f} mm")
    print(f"  CG height   {vp.cg_height:.0f} mm")
    print("Open it in a browser and drag to orbit.")


if __name__ == "__main__":
    main()
