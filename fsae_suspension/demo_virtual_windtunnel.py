# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""
Demo: the Virtual Tunnel Solver — calibrate CFD (k-omega SST) against a physical
aero map, the way an FSAE aero subteam actually uses tunnel time.

There is no single-code choice to make. The Virtual Tunnel Solver is built ON
Star-CCM+, TS-Auto AND OpenFOAM at once: it writes every matched ride-height point
for all three codes, then fuses their converged output into one cross-code
consensus coefficient. The inter-code spread is the payoff — agreement between
independent solvers is the strongest cheap evidence a number is physical.

The story:
  1. Map the physical aero map in the tunnel: C_l/C_d at swept front & rear ride
     heights. Here we fabricate a small but physically-shaped measured map.
  2. Build the matching Virtual Tunnel Solver and write the CFD driver files at the
     IDENTICAL ride-height/speed points — all three codes, one sub-folder each.
  3. Feed back two sets of per-code CFD results and FUSE them: a case where the
     codes agree and the consensus matches the tunnel (CALIBRATED), and one where
     the codes disagree (flagged before it ever reaches the tunnel comparison).

No license, no mesh, no solver needed: the driver files are written for real, the
fusion runs on results you'd otherwise bring back from the cluster, and a code that
can't run here is an honest hole, never a fabricated number.

Run:  python demo_virtual_windtunnel.py
"""

import os
import tempfile

from suspension.aero import windtunnel as wt
from suspension.aero import (
    get_backend, CFDProvenance, SolverFidelity, ride_heights_to_attitude,
    CoeffResult, MemberOutcome, fused_results, DEFAULT_MEMBER_NAMES,
)


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


def fuse_per_code(vts, pm, scales):
    """
    Stand in for "the team ran every code and brought the CSVs back". For each
    physical point, fabricate one CoeffResult per code (scaled to model each code's
    solve quality), wrap them as MemberOutcomes, and fuse through the SAME engine the
    solver uses — so the consensus here is identical to the programmatic one. Returns
    the list of EnsembleResults.
    """
    out = []
    for phys in pm.measured_points():
        outs = []
        for code in DEFAULT_MEMBER_NAMES:
            sc = scales.get(code)
            if sc is None:                      # this code didn't run => honest hole
                outs.append(MemberOutcome(backend=code, error="not run for this demo"))
                continue
            outs.append(MemberOutcome(
                backend=code,
                result=CoeffResult(attitude=phys.attitude,
                                   c_lift=phys.c_lift * sc,
                                   c_drag=phys.c_drag * sc,
                                   aero_balance_front=phys.aero_balance_front,
                                   converged=True)))
        spec = wt.CaseSpec(attitude=phys.attitude, geometry_path="car.stl",
                           reference_area_m2=1.0, reference_length_m=1.55)
        out.append(vts._fuse(spec, outs))
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
    print("VIRTUAL TUNNEL SOLVER — matched CFD run on Star-CCM+ + TS-Auto + OpenFOAM")
    print("=" * 78)
    print(vwt.plan())

    vts = get_backend("virtual-tunnel", reduction="mean", agreement_tol=5.0,
                      min_members=2, turbulence_model="kOmegaSST")
    outdir = tempfile.mkdtemp(prefix="kinematik_vts_demo_")
    specs = vwt.case_specs()
    for s in specs:
        vts.write_case(s, outdir)              # writes all three codes per point
    print(f"\nWrote {len(specs)} matched case(s) to {outdir}, each containing input "
          f"for all three codes:")
    example = os.path.join(outdir, specs[0].case_name())
    for code in DEFAULT_MEMBER_NAMES:
        sub = os.path.join(example, code)
        files = os.listdir(sub) if os.path.isdir(sub) else []
        print(f"    {specs[0].case_name()}/{code}/  ->  {', '.join(files[:2])}")
    print("(Run each point through every code on the team's installs; export one "
          "coeff CSV per code.)\n")

    print("=" * 78)
    print("CONSENSUS A — three codes that AGREE and match the tunnel")
    print("=" * 78)
    ensA = fuse_per_code(vts, pm, {"starccm": 1.01, "tsauto": 1.02, "openfoam": 0.99})
    exA = ensA[0]
    print(f"e.g. {exA.fused.attitude.label()}: fused C_l {exA.fused.c_lift:+.3f}, "
          f"{exA.n_voted} codes voted, inter-code C_l spread {exA.cl_spread_pct:.1f}% "
          f"=> converged={exA.fused.converged}")
    repA = vwt.correlate(fused_results(ensA))
    print(repA.summary, "\n")

    print("=" * 78)
    print("CONSENSUS B — codes DISAGREE (one code over-predicts downforce)")
    print("=" * 78)
    ensB = fuse_per_code(vts, pm, {"starccm": 1.00, "tsauto": 1.12, "openfoam": 0.98})
    exB = ensB[0]
    print(f"e.g. {exB.fused.attitude.label()}: fused C_l {exB.fused.c_lift:+.3f}, "
          f"{exB.n_voted} codes voted, inter-code C_l spread {exB.cl_spread_pct:.1f}% "
          f"=> converged={exB.fused.converged}")
    n_flagged = sum(1 for er in ensB if er.n_voted >= 2 and not er.fused.converged)
    print(f"{n_flagged}/{len(ensB)} point(s) flagged: the codes disagree beyond the "
          f"5% agreement tolerance — caught BEFORE the tunnel comparison.")
    print("\nThe consensus's own verdict on each point (first three):")
    for er in ensB[:3]:
        print(f"   {er.fused.attitude.label():46s}  "
              f"C_l spread {er.cl_spread_pct:5.1f}%  "
              f"converged={er.fused.converged}")


if __name__ == "__main__":
    main()
