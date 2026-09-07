# ============================================================================
#  KinematiK — suspension/perf.py
#  Per-tab timing harness. Answers ONE question: on a rerun, where does the
#  time actually go?
#
#  Why this exists. A static read of streamlit_app.py says 17,097 of 33,164
#  lines sit at module scope, so every one of the 40 tab bodies executes on
#  every script-run while the member can see exactly one. That is a line count,
#  not a millisecond count, and optimising against it would be guessing — a
#  1,800-line tab that only formats strings is cheap, and an 80-line tab that
#  solves a linkage 40 times is not. This measures the milliseconds.
#
#  Design constraints, in priority order:
#    1. Off by default, and nearly free when off — one dict lookup per tab.
#       A profiler that costs something is a profiler people disable and then
#       forget to re-enable when they actually need it.
#    2. Never raise. This wraps every tab in the app; a bug here would take
#       the whole tool down, which is a catastrophic trade for telemetry.
#    3. No Streamlit import. The store is injected, so the logic is testable
#       without booting the app.
#    4. Self time, not inclusive time. See _Frame below — inclusive time
#       double-counts nested blocks and makes an outer tab look like the
#       culprit when the cost is really in something it contains.
# ============================================================================
"""Measure per-tab execution cost across reruns.

Usage from the app (see _TabOpenProxy)::

    perf.begin_run(store)                       # once per script-run
    tok = perf.enter(store, "brakes", active=False)
    ...tab body...
    perf.exit(store, tok)
    perf.end_run(store)

Then :func:`summary` turns the ring buffer into a table ranked by total cost,
split by whether the member was actually looking at the tab.
"""

from __future__ import annotations

import time

#: How many reruns to keep. Enough to see a median and a p95 without letting
#: the buffer become a memory problem in a long session.
MAX_RUNS = 40

_STORE_KEY = "_kk_perf"
_ENABLED_KEY = "_kk_perf_enabled"


# --------------------------------------------------------------------------- #
#  State
# --------------------------------------------------------------------------- #
def _state(store, create=True):
    """The perf record inside the caller's store (normally st.session_state).

    ``create=False`` for read paths. A reader that allocates is how a harness
    that is switched off still leaves a key in session state — harmless here,
    but the whole point of this module is that OFF means off, and a reader with
    a side effect is the kind of thing that quietly stops being true.
    """
    st = store.get(_STORE_KEY)
    if not isinstance(st, dict):
        if not create:
            return None
        st = {"runs": [], "current": None, "stack": []}
        store[_STORE_KEY] = st
    return st


def enabled(store) -> bool:
    return bool(store.get(_ENABLED_KEY, False))


def set_enabled(store, on: bool) -> None:
    store[_ENABLED_KEY] = bool(on)
    if not on:
        store.pop(_STORE_KEY, None)     # drop the buffer so it can't go stale


def reset(store) -> None:
    store.pop(_STORE_KEY, None)


# --------------------------------------------------------------------------- #
#  Recording
# --------------------------------------------------------------------------- #
def begin_run(store) -> None:
    """Start a new script-run. Safe to call when disabled (does nothing)."""
    if not enabled(store):
        return
    try:
        s = _state(store)
        s["current"] = {"t0": time.perf_counter(), "tabs": {}}
        s["stack"] = []
    except Exception:
        pass


def enter(store, feature: str, active: bool = False):
    """Open a timing frame. Returns an opaque token, or None when disabled.

    The token is what :func:`exit` needs; passing None back is a no-op, so the
    caller needs no ``if enabled`` branch of its own.
    """
    if not enabled(store):
        return None
    try:
        s = _state(store)
        if s.get("current") is None:
            begin_run(store)
            s = _state(store)
        frame = {"feature": str(feature), "active": bool(active),
                 "start": time.perf_counter(), "child": 0.0}
        s["stack"].append(frame)
        return frame
    except Exception:
        return None


def exit(store, token) -> None:                      # noqa: A001 (mirrors enter)
    """Close a timing frame opened by :func:`enter`."""
    if token is None:
        return
    try:
        s = _state(store, create=False)
        if s is None:
            return
        stack = s.get("stack") or []
        # Unwind to this frame. A tab body that raised can leave frames open;
        # dropping them is better than mis-attributing their time to a parent.
        while stack and stack[-1] is not token:
            stack.pop()
        if not stack:
            return
        stack.pop()
        elapsed = time.perf_counter() - token["start"]
        # SELF time: what this block cost minus what its children cost.
        # Inclusive time would make an outer tab look expensive when the cost
        # is really in a nested block, which is the classic way a profiler
        # sends you to optimise the wrong function.
        self_ms = max(0.0, (elapsed - token["child"]) * 1000.0)
        if stack:
            stack[-1]["child"] += elapsed
        cur = s.get("current")
        if cur is None:
            return
        row = cur["tabs"].setdefault(
            token["feature"], {"ms": 0.0, "blocks": 0, "active": False})
        # A feature can own several `with` blocks in one run (model3d has
        # three); they sum into one row, which is how a reader thinks of it.
        row["ms"] += self_ms
        row["blocks"] += 1
        row["active"] = row["active"] or token["active"]
    except Exception:
        pass


def end_run(store) -> None:
    """Close the script-run and push it into the ring buffer."""
    if not enabled(store):
        return
    try:
        s = _state(store)
        cur = s.get("current")
        if not cur:
            return
        cur["total_ms"] = (time.perf_counter() - cur["t0"]) * 1000.0
        cur.pop("t0", None)
        runs = s.setdefault("runs", [])
        runs.append(cur)
        del runs[:-MAX_RUNS]
        s["current"] = None
        s["stack"] = []
    except Exception:
        pass


# --------------------------------------------------------------------------- #
#  Reporting
# --------------------------------------------------------------------------- #
def _percentile(vals, q):
    if not vals:
        return 0.0
    xs = sorted(vals)
    if len(xs) == 1:
        return xs[0]
    i = min(len(xs) - 1, max(0, int(round(q * (len(xs) - 1)))))
    return xs[i]


def summary(store) -> dict:
    """Aggregate the buffer into something rankable.

    Returns ``{"runs": n, "median_run_ms": .., "rows": [...],
    "background_ms": .., "background_pct": ..}``.

    ``background_ms`` is the headline: time spent in tabs the member was NOT
    looking at. That is the number that justifies (or doesn't) the work of
    making tab bodies lazy — a big line count with a small background cost
    means the refactor buys nothing, and it is better to find that out here
    than three weeks into it.
    """
    out = {"runs": 0, "median_run_ms": 0.0, "rows": [],
           "background_ms": 0.0, "background_pct": 0.0, "measured_ms": 0.0}
    try:
        s = _state(store, create=False)
        runs = (s.get("runs") if s else None) or []
        if not runs:
            return out
        out["runs"] = len(runs)
        out["median_run_ms"] = _percentile([r.get("total_ms", 0.0)
                                            for r in runs], 0.5)
        per = {}
        for r in runs:
            for feat, row in (r.get("tabs") or {}).items():
                acc = per.setdefault(feat, {"samples": [], "active": 0,
                                            "background": 0, "blocks": 0})
                acc["samples"].append(row["ms"])
                acc["blocks"] = max(acc["blocks"], row["blocks"])
                if row["active"]:
                    acc["active"] += 1
                else:
                    acc["background"] += 1
        rows = []
        for feat, acc in per.items():
            xs = acc["samples"]
            rows.append({
                "feature": feat,
                "median_ms": _percentile(xs, 0.5),
                "p95_ms": _percentile(xs, 0.95),
                "max_ms": max(xs),
                "runs": len(xs),
                "blocks": acc["blocks"],
                # How often this tab ran while the member was elsewhere. 100%
                # means every millisecond it costs is pure waste.
                "background_pct": (100.0 * acc["background"] / len(xs))
                if xs else 0.0,
            })
        rows.sort(key=lambda r: r["median_ms"], reverse=True)
        out["rows"] = rows
        out["measured_ms"] = sum(r["median_ms"] for r in rows)
        out["background_ms"] = sum(
            r["median_ms"] * r["background_pct"] / 100.0 for r in rows)
        if out["measured_ms"] > 0:
            out["background_pct"] = (100.0 * out["background_ms"]
                                     / out["measured_ms"])
        return out
    except Exception:
        return out


def format_report(store) -> str:
    """Plain-text summary, for a log line or a copy-paste into an issue."""
    s = summary(store)
    if not s["runs"]:
        return "No timing data yet — interact with a few tabs."
    L = [f"KinematiK tab timing — {s['runs']} rerun(s), "
         f"median script-run {s['median_run_ms']:.0f} ms",
         f"measured in tab bodies: {s['measured_ms']:.0f} ms  |  "
         f"spent in tabs the member wasn't looking at: "
         f"{s['background_ms']:.0f} ms ({s['background_pct']:.0f}%)",
         "",
         f"{'feature':18s} {'median':>9s} {'p95':>9s} {'max':>9s} "
         f"{'bg%':>6s} {'blocks':>7s}"]
    for r in s["rows"]:
        L.append(f"{r['feature']:18s} {r['median_ms']:8.1f}m "
                 f"{r['p95_ms']:8.1f}m {r['max_ms']:8.1f}m "
                 f"{r['background_pct']:5.0f}% {r['blocks']:7d}")
    return "\n".join(L)
