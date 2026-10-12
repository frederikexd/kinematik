# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""
compliance_margin.py — how stiff the joints and tabs must be for a corner's
compliance walls to hold, for any manifest that declares them.

A manifest's compliance walls (compliance_toe_deg in its cornering case,
compliance_toe_braking_deg in its braking case) are evaluated on the pickup
flexibility it declares and on each link's own axial stretch. The joints at the
two ends of every link (rod ends, spherical bearings, ball joints), the tabs
they bolt to and the upright's arms are not in that model, and every one of
them adds give in series with the link. This module adds them as one declared
stiffness per link end, along the link (N/mm), and answers two questions:

    end_stiffness_sweep(manifest, ends)   the toe in each case at each end stiffness
    required_end_stiffness(manifest)      the end stiffness below which each wall breaks

and, for free play rather than stiffness, the toe deadband per 0.1 mm of
total clearance at every link (elastokinematics.lash_deadband). The required
stiffness is the number a joint, tab and arm design must exceed, and an in-situ
pull test measures it. Lengths mm, forces N, stiffness N/mm, angles deg.
"""
from __future__ import annotations

import math
from dataclasses import replace

from . import elastokinematics as ek

__all__ = ["with_end_stiffness", "toe", "end_stiffness_sweep", "required_end_stiffness", "lash_toe_per_0p1mm",
           "walls"]


def with_end_stiffness(spec: "ek.ElastoSpec", k_end_n_per_mm: float | None) -> "ek.ElastoSpec":
    """``spec`` with every link's two ends (joint, tab, arm) in series along it, each of stiffness
    ``k_end_n_per_mm`` (N/mm); None or inf leaves the ends rigid, as the manifest declares them."""
    if k_end_n_per_mm is None or not math.isfinite(k_end_n_per_mm):
        return spec
    missing = [m for m in ek.MEMBER_PICKUP if m not in spec.members]
    if missing:
        raise ValueError("compliance_margin needs every link declared as a member (material, od, wall); missing "
                         + ", ".join(missing))
    k_ends = float(k_end_n_per_mm) / 2.0                         # two ends in series
    mem = dict(spec.members)
    for m in ek.MEMBER_PICKUP:
        ms = mem[m]
        mem[m] = replace(ms, k_tab=k_ends if ms.k_tab is None else 1.0 / (1.0 / ms.k_tab + 1.0 / k_ends))
    return replace(spec, members=mem)


def toe(hp, spec: "ek.ElastoSpec") -> float:
    """Toe change (deg) of the corner under the spec's load, joints and tabs included as the spec declares."""
    r = ek.solve_elastokinematic(hp, spec, jacobian=False)
    if not r.converged:
        return float("nan")
    return float(r.change["toe"])


def walls(manifest) -> dict:
    """{case: (spec, limit in deg)} for the compliance toe walls the manifest declares."""
    vol = manifest.objects()[2]
    pb = vol.properties
    lim = {b.prop: max(abs(b.lo) if math.isfinite(b.lo) else 0.0, abs(b.hi) if math.isfinite(b.hi) else 0.0)
           for b in pb.bounds}
    out = {}
    if "compliance_toe_deg" in lim and pb.elasto is not None:
        out["corner"] = (pb.elasto, lim["compliance_toe_deg"])
    if "compliance_toe_braking_deg" in lim and getattr(pb, "elasto_braking", None) is not None:
        out["braking"] = (pb.elasto_braking, lim["compliance_toe_braking_deg"])
    return out


def _corner(manifest):
    from .steering_margin import corner
    return corner(manifest)


def end_stiffness_sweep(manifest, ends_n_per_mm=(math.inf, 100000.0, 50000.0, 20000.0, 10000.0)) -> dict:
    """{case: {"limit_deg", "toe_deg": {k_end: toe}}} for the manifest's winner; stiffness N/mm, toe deg."""
    hp = _corner(manifest)
    res = {}
    for case, (spec, limit) in walls(manifest).items():
        res[case] = {"limit_deg": limit,
                     "toe_deg": {("rigid" if not math.isfinite(k) else float(k)): toe(hp, with_end_stiffness(spec, k))
                                 for k in ends_n_per_mm}}
    return res


def required_end_stiffness(manifest, lo: float = 1000.0, hi: float = 1.0e7, rel_tol: float = 0.005) -> dict:
    """{case: (stiffness N/mm per link end at which |toe| reaches the wall, toe deg with rigid ends, limit deg)}.

    inf when the wall fails with rigid ends; ``lo`` when it holds even at ``lo``."""
    hp = _corner(manifest)
    out = {}
    for case, (spec, limit) in walls(manifest).items():
        t_rigid = toe(hp, spec)
        if abs(t_rigid) > limit:
            out[case] = (math.inf, t_rigid, limit)
            continue
        if abs(toe(hp, with_end_stiffness(spec, lo))) <= limit:
            out[case] = (lo, t_rigid, limit)
            continue
        a, b = math.log(lo), math.log(hi)                        # a fails, b holds
        while b - a > rel_tol:
            m = 0.5 * (a + b)
            if abs(toe(hp, with_end_stiffness(spec, math.exp(m)))) <= limit:
                b = m
            else:
                a = m
        out[case] = (math.exp(b), t_rigid, limit)
    return out


def lash_toe_per_0p1mm(manifest) -> float:
    """Worst-case toe deadband (deg) with 0.1 mm of total radial clearance at every link."""
    hp = _corner(manifest)
    return float(ek.lash_deadband(hp, {m: 0.1 for m in ek.MEMBER_PICKUP})["toe"])
