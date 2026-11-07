# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Ripple: what each edit did to the car, with undo.

Why: a new member changes a spring rate and has no idea what it did, or whether it broke something two tabs away.
KinematiK already re-solves the whole car on every edit, so the consequences are known; they just were not shown.
Ripple compares the car's inputs run to run, and when they change it records one entry: what changed (before ->
after) and a few headline results before and after. Each entry can be undone on its own: only the inputs that entry
changed go back, so later edits stay.

Pure Python, no Streamlit import: the app snapshots its state with `snapshot`, keeps the feed in session state, and
renders it. Values are stored in SI (mm, kg, N/mm, N·m/°, fraction); the app converts them for display.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

MERGE_WINDOW_S = 30.0     # edits to the same inputs within this many seconds merge into one entry (slider drags)
MAX_ENTRIES = 12


def _norm(v):
    """Comparable, hashable form: lists/tuples of numbers -> tuple of floats; numbers -> float; others as-is."""
    if isinstance(v, (list, tuple)):
        return tuple(float(x) for x in v)
    if isinstance(v, bool) or v is None or isinstance(v, str):
        return v
    try:
        return float(v)
    except (TypeError, ValueError):
        return v


def snapshot(vp: dict, hp: dict, topology: str, preset: str, topo_points: dict | None = None) -> dict:
    """Flatten the car's inputs into {path: value}. Paths: ("vp", key), ("hp", key), ("ss", "topology"),
    ("ss", "kk_preset"), ("topo", "<topology>/<point>") for a non-wishbone topology's points. Values in SI (mm, kg, N/mm),
    unitless where they have no unit. Non-numeric vp entries other than flags and strings are skipped."""
    out = {}
    for k, v in (vp or {}).items():
        nv = _norm(v)
        if isinstance(nv, (float, bool, str)) or nv is None:
            if isinstance(nv, float) and not math.isfinite(nv):
                continue
            out[("vp", k)] = nv
    for k, v in (hp or {}).items():
        out[("hp", k)] = _norm(v)
    out[("ss", "topology")] = topology
    out[("ss", "kk_preset")] = preset
    for k, v in (topo_points or {}).items():
        out[("topo", f"{topology}/{k}")] = _norm(v)
    return out


def _same(a, b) -> bool:
    if isinstance(a, float) and isinstance(b, float):
        return abs(a - b) <= 1e-9 * max(1.0, abs(a), abs(b))
    if isinstance(a, tuple) and isinstance(b, tuple) and len(a) == len(b):
        return all(_same(x, y) for x, y in zip(a, b))
    return a == b


def diff(prev: dict, cur: dict) -> list:
    """[(path, before, after)] for every path whose value changed (a missing path counts as None). Sorted by path so
    the order is stable. Unitless list of changes."""
    out = []
    for p in sorted(set(prev) | set(cur), key=repr):
        a, b = prev.get(p), cur.get(p)
        if not _same(a, b):
            out.append((p, a, b))
    return out


@dataclass
class Entry:
    id: int
    t: float                                  # time of the latest edit in this entry, seconds since the epoch
    changes: list                             # [(path, before, after)]
    m_before: dict | None = None              # headline results before the edit (None if unavailable)
    m_after: dict | None = None
    undone: bool = False
    t0: float = 0.0                           # time of the first edit merged into this entry
    paths: frozenset = field(default_factory=frozenset)


def record(feed: list, changes: list, m_before, m_after, now: float, next_id: int) -> int:
    """Add one edit to `feed` (newest first) and return the next free entry id. An edit to exactly the same inputs as
    the newest, not-undone entry, within MERGE_WINDOW_S of it, updates that entry instead (so a slider drag is one
    entry): its "before" values and results stay, its "after" ones move on. An edit that returns those inputs to
    where the entry started removes the entry. Times in seconds (s); ids unitless."""
    if not changes:
        return next_id
    paths = frozenset(p for p, _a, _b in changes)
    top = feed[0] if feed else None
    if top is not None and not top.undone and top.paths == paths and now - top.t <= MERGE_WINDOW_S:
        before = {p: a for p, a, _b in top.changes}
        merged = [(p, before[p], b) for p, _a, b in changes]
        if all(_same(a, b) for _p, a, b in merged):
            feed.pop(0)                         # dragged back to where it started: nothing happened
        else:
            top.changes, top.m_after, top.t = merged, m_after, now
        return next_id
    feed.insert(0, Entry(next_id, now, list(changes), m_before, m_after, t0=now, paths=paths))
    del feed[MAX_ENTRIES:]
    return next_id + 1


def undo_targets(entry: Entry) -> list:
    """[(path, value)] to write back to undo `entry`: each input it changed, set to its value before. A non-wishbone
    point that did not exist before (it appeared with a topology switch) is left alone; switching the topology back
    is enough. Values in SI (mm, kg, N/mm, N·m/°)."""
    return [(p, a) for p, a, _b in entry.changes if not (p[0] == "topo" and a is None)]
