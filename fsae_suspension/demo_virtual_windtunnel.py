# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""
Demo: the Virtual Wind Tunnel — calibrate CFD (k-omega SST) against a physical
aero map, the way an FSAE aero subteam actually uses tunnel time.

The story:
  1. Map the physical aero map in the tunnel: C_l/C_d at swept front & rear ride
     heights. Here we fabricate a small but physically-shaped measured map.
  2. Build the matching Virtual Wind Tunnel and write the CFD driver files at the
     IDENTICAL ride-height/speed points (Star-CCM+ macro shown; TS-Auto/OpenFOAM
     are one argument away).
  3. Feed back two CFD result sets — a well-resolved solve that matches, and an
     under-resolved one that over-predicts downforce — and show how the correlation
     reports CALIBRATED vs NOT CALIBRATED, point by point, with the direction of the
     error.

No license, no mesh, no solver needed: the driver files are written for real, and
the correlation runs on results you'd otherwise bring back from the cluster.

Run:  python demo_virtual_windtunnel.py
"""

import os
import tempfile

from suspension.aero import windtunnel as wt
from suspension.aero import get_backend, CFDProvenance, SolverFidelity, ride_heights_to_attitude, CoeffResult


def build_physical_map():
    """A small measured aero map: 3 front x 2 rear ride heights at one tunnel speed."""
    prov = wt.TunnelProvenance(
        facility="A2 Wind Shear (rolling road)",
        ground_state=wt.GroundState.MOVING_BELT,
        model_scale=1.0, blockage_corrected=True, reynolds=4.2e5,
        reference_area_m2=1.0, reference_length_m=1.55,
    )
    pm = wt.PhysicalAeroMap(prov, reference_area_m2=1.0, reference_length_m=1.55,
                            wheelbase_mm=1550.0)
    for front in (18.0, 25.0, 32.0):
        for rear in (40.0, 55.0):
            rh = wt.RideHeights(front, rear, speed_ms=27.0, wheelbase_mm=1550.0)
            # physically-shaped: lower & more rake => more downforce; slightly more drag
            cl = -2.95 + 0.012 * (front - 18.0) - 0.004 * (rear - 40.0)
            cd = 1.04 + 0.0015 * (front - 18.0) + 0.0010 * (rear - 40.0)
            bal = 0.43 + 0.0008 * (rear - front)
            pm.add_measurement(rh, c_lift=cl, c_drag=cd, aero_balance_front=bal)
    return pm


def synth_cfd(pm, cl_scale=1.0, cd_scale=1.0, turbulence="kOmegaSST", backend="starccm"):
    """Fabricate CFD results at the map's exact points, scaled to model solve quality."""
    prov = CFDProvenance(backend=backend, fidelity=SolverFidelity.RANS,
                         turbulence_model=turbulence,
                         notes="synthetic demo CFD result")
    out = []
    for phys in pm.measured_points():
        out.append(CoeffResult(
            attitude=phys.attitude,
            c_lift=phys.c_lift * cl_scale,
            c_drag=phys.c_drag * cd_scale,
            aero_balance_front=phys.aero_balance_front,
            converged=True, provenance=prov))
    return out


def main():
    pm = build_physical_map()
    print("=" * 78)
    print("PHYSICAL AERO MAP (the tunnel run)")
    print("=" * 78)
    print(pm.status())
    print(f"{len(pm)} measured points over front/rear ride height.\n")

    vwt = wt.VirtualWindTunnel(pm, geometry_path="car.stl", rho=1.225)
    print("=" * 78)
    print("VIRTUAL WIND TUNNEL — matched CFD run")
    print("=" * 78)
    print(vwt.plan())

    backend = get_backend("starccm")
    outdir = tempfile.mkdtemp(prefix="kinematik_vwt_demo_")
    specs = vwt.case_specs()
    written = [backend.write_case(s, outdir) for s in specs]
    print(f"\nWrote {len(written)} Star-CCM+ driver macros to {outdir}:")
    for w in written:
        print("   ", os.path.basename(w))
    print("(Run these on the team's licensed install; export one coeff CSV per case.)\n")

    print("=" * 78)
    print("CORRELATION A — a well-resolved solve (matches the tunnel)")
    print("=" * 78)
    good = synth_cfd(pm, cl_scale=1.01, cd_scale=1.01)   # ~1% off: within tolerance
    repA = vwt.correlate(good)
    print(repA.summary, "\n")

    print("=" * 78)
    print("CORRELATION B — an under-resolved solve (over-predicts downforce)")
    print("=" * 78)
    bad = synth_cfd(pm, cl_scale=1.10, cd_scale=1.04)    # 10% on C_l: out of tolerance
    repB = vwt.correlate(bad)
    print(repB.summary, "\n")

    print("Per-point C_l error, case B:")
    for p in repB.points:
        if p.paired:
            print(f"   {p.ride_heights.label():38s}  C_l {p.cl_phys:+.3f} -> {p.cl_cfd:+.3f}  "
                  f"({p.cl_err_pct:+.1f}%)")


if __name__ == "__main__":
    main()
