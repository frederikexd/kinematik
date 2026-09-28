# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Assembly sequence: what has to come off before a part can come off.

The FSAE trap is a pickup that is perfect kinematically and needs the
differential out to change its bolt. The tool-access wall checks one wrench
path; this module answers the general question for the corner and whatever
surrounds it: given the parts as simple solids, each with the directions it
can be withdrawn along, in what order can the assembly be taken apart, and
which parts must be removed to free a given one. Assembly is the reverse of a
valid disassembly (assembly by disassembly), so a sequence found here is one
the shop can build in.

MODEL
-----
Every part is a set of capsules (segment plus radius), the same solids the
frame tubes use elsewhere. A part is free along a direction when sweeping it
that far never brings it into NEW contact with a part still present: a pair
that already touches at rest (a bolt in its clevis) blocks only if the sweep
moves them closer, so a bolt sliding out along its own axis is free while one
pushed sideways into a tube is not. Parts marked ``fixed`` (the chassis) never
move. Disassembly removes any free part, repeatedly; the order and each
part's blockers are reported, and a deadlock names the parts that lock each
other.

``parts_to_free`` is the maintenance question: the fewest parts, found by
freeing blockers recursively along each part's best direction, that must come
off before the target can be withdrawn.

Scope: straight-line withdrawals along declared directions (no rotations or
compound paths), capsule solids, a sampled sweep. It does not replace a CAD
disassembly study; it runs inside the design loop, on the same coordinates.
Units mm.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


def _seg_seg(p1, q1, p2, q2) -> float:
    from .genesis_repro import seg_seg_dist
    return float(seg_seg_dist(p1, q1, p2, q2))


@dataclass
class Part:
    """A part: capsules (a, b, radius mm) and withdrawal directions."""
    name: str
    capsules: list                       # [((x,y,z), (x,y,z), r), ...]
    directions: list = field(default_factory=list)   # unit vectors
    travel_mm: float = 150.0
    fixed: bool = False

    def __post_init__(self):
        self.capsules = [(np.asarray(a, float), np.asarray(b, float), float(r))
                         for a, b, r in self.capsules]
        self.directions = [np.asarray(d, float) / np.linalg.norm(d)
                           for d in self.directions]


def _gap(p: Part, q: Part, shift=np.zeros(3)) -> float:
    return min(_seg_seg(a + shift, b + shift, c, d) - r - s
               for a, b, r in p.capsules for c, d, s in q.capsules)


def blockers(part: Part, others: list, direction, samples: int = 15,
             tol: float = 1e-6) -> list:
    """Names of parts that stop ``part`` moving along ``direction``."""
    d = np.asarray(direction, float) / np.linalg.norm(direction)
    out = []
    for q in others:
        if q is part:
            continue
        g0 = _gap(part, q)
        for t in np.linspace(0.0, part.travel_mm, samples)[1:]:
            g = _gap(part, q, d * t)
            if g < -tol and g < g0 - tol:        # new or deeper contact
                out.append(q.name)
                break
    return out


def disassembly(parts: list) -> dict:
    """Remove free parts repeatedly. Returns the order, and any deadlock."""
    present = list(parts)
    order = []
    while True:
        movable = [p for p in present if not p.fixed]
        if not movable:
            break
        freed = None
        for p in movable:
            for d in p.directions:
                if not blockers(p, present, d):
                    freed = (p, d)
                    break
            if freed:
                break
        if freed is None:
            lock = {p.name: sorted({b for d in p.directions
                                    for b in blockers(p, present, d)})
                    for p in movable}
            return {"ok": False, "order": [o[0] for o in order],
                    "deadlock": lock}
        p, d = freed
        order.append((p.name, tuple(np.round(d, 3))))
        present.remove(p)
    return {"ok": True, "order": [o[0] for o in order],
            "directions": {n: d for n, d in order},
            "assembly_order": [o[0] for o in reversed(order)]}


def parts_to_free(parts: list, target: str, _seen=None) -> list | None:
    """The parts that must come off before ``target`` can be withdrawn.

    For each withdrawal direction of the target, its blockers must be freed
    first (recursively); the direction needing the fewest removals wins.
    Returns the removal list in order, or None if the target cannot be freed
    (a fixed part blocks every direction, or the blockers lock each other).
    """
    by = {p.name: p for p in parts}
    seen = set(_seen or ()) | {target}
    t = by[target]
    best = None
    for d in t.directions:
        need = blockers(t, parts, d)
        if any(by[n].fixed for n in need):
            continue
        plan, ok = [], True
        for n in need:
            if n in seen:
                ok = False
                break
            sub = parts_to_free(parts, n, seen)
            if sub is None:
                ok = False
                break
            for s in sub + [n]:
                if s not in plan:
                    plan.append(s)
        if ok and (best is None or len(plan) < len(best)):
            best = plan
    return best
