# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""MorphMesh demo — the bracket that grows itself, and the shop that vetoes it.

The suspension lead needs a chassis tab for the lower-front wishbone leg. A
transient event never loads that tab from one direction — the force migrates
as the car takes the curb — and the hand welder cannot hold a razor rib next
to a bead. This script runs the event, condenses the shifting force history
into a load fan, grows a topology against the whole fan, lets the declared
hand-weld limits REJECT what the floor can't build, and prints the premium of
buildability — then re-grows the same tab for a CNC shop and shows what the
jig money buys.

Run:  python demo_morphmesh.py        (about a minute on a laptop)
"""

import numpy as np

from suspension.kinematics import Hardpoints
from suspension.compliance import CompliantCorner
from suspension import ghost_topology as gt
from suspension import morphmesh as mm
from suspension import transient as tr


def main():
    hp = Hardpoints.default()

    print("=" * 72)
    print("1 · THE LOAD HISTORY  (a curb strike, audited by Ghost Topology)")
    print("=" * 72)
    res = tr.run_maneuver(None, kind="curb_strike")
    cc = CompliantCorner.uniform_tube(hp)
    params = tr.TransientParams.from_vehicle(None)
    Fz = float(tr.TransientSolver().static_corner_loads()[1])   # FR
    gc = gt.GhostCorner(
        cc, gt.uniform_sections(od_mm=19.05, wall_mm=0.9,
                                material="Steel 4130", yield_MPa=460.0),
        wheel_rate_N_per_mm=params.k_wheel_front / 1000.0,
        Fz_static_N=Fz, track_mm=params.track_front * 1000.0)
    audit = gt.ghost_audit_transient(gc, res, corner="FR", n_samples=12)
    print(f"  audited {len(audit.instants)} instants of the event "
          f"(verdict {audit.verdict})")

    print()
    print("=" * 72)
    print("2 · THE LOAD FAN  (the shifting force, compressed into arrows)")
    print("=" * 72)
    fan = mm.load_fan_from_audit(audit, hp, member="LF", n_cases=4)
    print(f"  member LF — {len(fan.cases)} case(s), span {fan.span_deg:.0f}°, "
          f"{fan.inplane_share * 100:.0f}% of exposure in the bracket plane")
    for c in fan.cases:
        print(f"    {c.angle_deg:8.1f}°   peak {c.F_N:7.0f} N   "
              f"exposure {c.weight * 100:4.0f}%   "
              f"({c.n_instants} instants, "
              f"{c.t_lo_s * 1000:.0f}–{c.t_hi_s * 1000:.0f} ms)")
    for w in fan.warnings:
        print(f"  ⚠ {w}")

    print()
    print("=" * 72)
    print("3 · GROWN FOR THE HAND SHOP  (the floor gets a veto)")
    print("=" * 72)
    dom = mm.PlateDomain.chassis_tab(h_mm=2.0)
    hand = mm.FabricationLimits.from_shop("hand_weld")
    print(f"  declared limits: min rib {hand.min_rib_mm:.1f} mm, "
          f"HAZ {hand.haz_mm:.0f} mm band at {hand.min_rib_haz_mm:.1f} mm "
          f"(= web floor {hand.web_floor_mm:.1f} + 2 × "
          f"±{hand.accuracy_mm:.1f} mm accuracy)")
    r_hand = mm.morph_component(dom, fan.cases, hand, volfrac=0.4, fan=fan,
                                max_iter=22)
    print(f"  verdict: {r_hand.verdict}   mass {r_hand.mass_g:.0f} g   "
          f"FoS {r_hand.fos:.2f} (governing case {r_hand.fos_case_deg:.0f}°)")
    for i, rd in enumerate(r_hand.rounds):
        print(f"    round {i + 1}: feature {rd.filter_mm:.1f} mm, "
              f"{rd.iterations} iters, viol "
              f"{rd.audit.get('viol_frac', 0) * 100:.1f}% bulk / "
              f"{rd.audit.get('viol_haz_frac', 0) * 100:.1f}% HAZ — "
              + ("signed ✅" if rd.accepted else "REJECTED ❌"))
    if r_hand.coarsen_premium:
        p = r_hand.coarsen_premium
        print(f"  → the premium of buildability: "
              f"{p['d_compliance_frac'] * 100:.1f}% compliance for "
              f"+{p['d_filter_mm']:.1f} mm feature size.")

    print()
    print("=" * 72)
    print("4 · THE SAME TAB, CNC SHOP  (what the jig money buys)")
    print("=" * 72)
    cnc = mm.FabricationLimits.from_shop("cnc")
    r_cnc = mm.morph_component(dom, fan.cases, cnc, volfrac=0.4, fan=fan,
                               max_iter=22)
    print(f"  verdict: {r_cnc.verdict}   mass {r_cnc.mass_g:.0f} g   "
          f"FoS {r_cnc.fos:.2f}")
    dc = (r_hand.compliance_history[-1] / r_cnc.compliance_history[-1] - 1.0
          if r_cnc.compliance_history and r_cnc.compliance_history[-1] > 0
          else float("nan"))
    if dc == dc:
        print(f"  → the hand-shop shape is {dc * 100:+.1f}% compliance vs the "
              "CNC one at the same mass budget — the number that argues for "
              "(or against) the machine time.")

    print()
    print("=" * 72)
    print("5 · THE DELIVERABLE  (the shape as data, for the CAD seat)")
    print("=" * 72)
    segs = r_hand.outline_segments(dom.h_mm)
    print(f"  cells CSV     : {len(r_hand.cells_csv(dom.h_mm).splitlines()) - 1}"
          " non-void cells")
    print(f"  outline       : {len(segs)} pixel-edge segments "
          f"(staircase at h = {dom.h_mm:.1f} mm)")
    print(f"  JSON summary  : {len(r_hand.to_json())} bytes")
    print()
    print(mm.render_morph_md(r_hand))


if __name__ == "__main__":
    main()
