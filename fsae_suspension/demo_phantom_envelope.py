# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Phantom Envelope demo — the packaging question, answered end to end.

The powertrain lead wants to bolt a motor mount near the front-right upper
control arm and needs to know it clears the arm through a hard corner — not
just at rest, but while the loaded link DEFLECTS. This script carves the rigid
motion envelope and the compliance-warped one, measures the growth, and answers
the mount clearance query with full attribution.

Run:  python demo_phantom_envelope.py
"""

import numpy as np

from suspension.kinematics import Hardpoints
from suspension.compliance import CompliantCorner
from suspension import ghost_topology as gt
from suspension import phantom_envelope as pe


def main():
    hp = Hardpoints.default()

    print("=" * 72)
    print("1 · THE RIGID MOTION ENVELOPE  (what a manual CAD check draws)")
    print("=" * 72)
    # A representative FSAE front corner. 12 mm alu links + soft tabs so the
    # compliance deflection is visible (a stock 4130 corner barely moves — which
    # the tool correctly reports too).
    cc = CompliantCorner.uniform_tube(hp, material="Aluminium 6061",
                                      od_mm=12.0, wall_mm=1.0, k_tab=1500.0)
    gc = gt.GhostCorner(
        cc, gt.uniform_sections(od_mm=12.0, wall_mm=1.0,
                                material="Aluminium 6061", yield_MPa=276.0),
        wheel_rate_N_per_mm=35.0, Fz_static_N=700.0)

    rigid = pe.carve_rigid_envelope(gc, corner_label="FR",
                                    travel_mm=(-25, 25), n_travel=9,
                                    inflate_mm=3.0)
    mn, mx = rigid.bbox
    print(f"  swept {rigid.n_instants} rigid poses over ±25 mm travel")
    print(f"  boundary cloud: {rigid.n_points} points")
    print(f"  bounding box (mm): x[{mn[0]:.0f},{mx[0]:.0f}] "
          f"y[{mn[1]:.0f},{mx[1]:.0f}] z[{mn[2]:.0f},{mx[2]:.0f}]")

    print()
    print("=" * 72)
    print("2 · THE COMPLIANCE-WARPED ENVELOPE  (what the LOADED corner claims)")
    print("=" * 72)
    # A ~2.5 g cornering pulse in corner axes (lateral pull is -y).
    t = np.linspace(0.0, 0.8, 401)
    s = np.sin(np.pi * t / 0.8)
    Fz = 700.0 + 1500.0 * s
    Fy = -2.5 * Fz * s
    Fx = np.zeros_like(t)
    audit = gt.ghost_audit(gc, t, Fx, Fy, Fz, corner_label="FR", n_samples=20)

    comp = pe.carve_ghost_envelope(audit, gc, inflate_mm=3.0)
    delta = pe.envelope_delta(rigid, comp)
    print(f"  carved {comp.n_instants} loaded instants "
          f"({comp.n_points} boundary points)")
    if comp.excluded:
        for tt, r in comp.excluded:
            print(f"  excluded t={tt*1000:.0f} ms — {r}")
    if delta.max_outward_growth_mm > 0.0:
        ld = delta.growth_load_N or {}
        print(f"  → under load the corner claims "
              f"{delta.max_outward_growth_mm:.2f} mm BEYOND the rigid sweep,")
        print(f"    governed by {delta.growth_member} at "
              f"t = {delta.growth_t_s*1000:.0f} ms "
              f"(Fy {ld.get('Fy', 0):.0f} N, Fz {ld.get('Fz', 0):.0f} N)")
        print(f"    {delta.frac_points_grown*100:.0f}% of the loaded boundary "
              "lies outside the rigid one.")

    print()
    print("=" * 72)
    print("3 · THE PACKAGING QUERY  (does the motor mount clear the arm?)")
    print("=" * 72)
    # A candidate motor-mount corner near the UCA outer ball joint, 6 mm probe.
    mount = np.array([15.0, 530.0, 300.0])
    probe_r = 6.0
    q_rigid = rigid.query(mount, probe_radius_mm=probe_r)
    q_comp = comp.query(mount, probe_radius_mm=probe_r)

    print(f"  candidate point (corner frame, mm): "
          f"({mount[0]:.0f}, {mount[1]:.0f}, {mount[2]:.0f}), "
          f"probe radius {probe_r:.0f} mm")
    print(f"  vs RIGID sweep    : "
          + ("VIOLATES by %.2f mm" % abs(q_rigid.clearance_mm) if q_rigid.violates
             else "clears by %.2f mm" % q_rigid.clearance_mm)
          + f"  (nearest {q_rigid.nearest_member})")
    print(f"  vs LOADED envelope: "
          + ("VIOLATES by %.2f mm" % abs(q_comp.clearance_mm) if q_comp.violates
             else "clears by %.2f mm" % q_comp.clearance_mm)
          + f"  (nearest {q_comp.nearest_member}"
          + (f", t = {q_comp.nearest_t_s*1000:.0f} ms)" if q_comp.nearest_t_s == q_comp.nearest_t_s else ")"))

    if (not q_rigid.violates) and q_comp.violates:
        print()
        print("  ⚠ THE CONFLICT A RIGID CHECK SHIPS:")
        print(f"    this mount clears the rigid sweep but the loaded "
              f"{q_comp.nearest_member} deflects into it "
              f"by {abs(q_comp.clearance_mm):.2f} mm mid-corner.")

    print()
    print("=" * 72)
    print("4 · THE DELIVERABLE  (the point cloud the other team queries)")
    print("=" * 72)
    blob = comp.to_point_cloud()
    print(f"  format   : {blob['format']}")
    print(f"  frame    : {blob['frame']}")
    print(f"  units    : {blob['units']}  ·  {blob['n_points']} points")
    print(f"  members  : {', '.join(blob['members'])}")
    print("  → comp.to_json() / comp.to_csv() ship this to the packaging seat.")
    print()
    print(pe.render_envelope_md(comp, delta=delta, queries=[q_comp]))


if __name__ == "__main__":
    main()
