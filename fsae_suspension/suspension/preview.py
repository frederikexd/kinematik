# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Preview mode: five minutes of the full app without an account, then the sign-up screen.

Why: with accounts switched on, a visitor meets a sign-in form before seeing anything, so the first question
("is this worth an account?") cannot be answered. Preview lets them explore for PREVIEW_SECONDS, then asks.

What preview may touch: nothing persistent. Its project store is an in-memory backend held in the visitor's own
session, so a preview can never read or write a team's workspace, the shared legacy project row, or another
visitor's preview. Leaving the page discards it.

One preview per browser session: once it has run out, the sign-up screen replaces it until the visitor signs in.
Opening a fresh session starts a new one; this is a nudge, not a paywall.

Pure Python, no Streamlit import: `ss` is any mutable mapping (st.session_state in the app, a dict in tests), and
`now` is seconds since the epoch (time.time() when omitted).
"""
from __future__ import annotations

import copy
import time

PREVIEW_SECONDS = 300            # 5 minutes
WARN_AT_SECONDS = 60             # one toast when this much is left

KEY_START = "_kx_preview_started"
KEY_WARNED = "_kx_preview_warned"
KEY_BACKEND = "_kx_preview_backend"


def _now(now):
    return time.time() if now is None else float(now)


def start(ss, now: float | None = None) -> bool:
    """Start the preview clock; False if this session already had its preview (the clock is never reset)."""
    if ss.get(KEY_START) is not None:
        return False
    ss[KEY_START] = _now(now)
    return True


def started(ss) -> bool:
    """Whether this session has used its preview, running or over."""
    return ss.get(KEY_START) is not None


def remaining_s(ss, now: float | None = None) -> float:
    """Seconds of preview left (0 s when over or never started)."""
    t0 = ss.get(KEY_START)
    if t0 is None:
        return 0.0
    return max(0.0, PREVIEW_SECONDS - (_now(now) - float(t0)))


def active(ss, now: float | None = None) -> bool:
    """A preview is running in this session."""
    return started(ss) and remaining_s(ss, now) > 0.0


def expired(ss, now: float | None = None) -> bool:
    """This session's preview has run out."""
    return started(ss) and remaining_s(ss, now) <= 0.0


def end_now(ss, now: float | None = None) -> None:
    """End the preview early (the visitor chose to sign up)."""
    if started(ss):
        ss[KEY_START] = _now(now) - PREVIEW_SECONDS


def should_warn(ss, now: float | None = None) -> bool:
    """True once, the first time a check finds WARN_AT_SECONDS (60 s) or less left."""
    if active(ss, now) and remaining_s(ss, now) <= WARN_AT_SECONDS and not ss.get(KEY_WARNED):
        ss[KEY_WARNED] = True
        return True
    return False


def clock(seconds: float) -> str:
    """m:ss for a number of seconds, e.g. 4:05."""
    s = max(0, int(seconds + 0.999))
    return f"{s // 60}:{s % 60:02d}"


class MemoryBackend:
    """A project backend that lives only in this object: nothing touches disk or the network. Same interface as
    project.JSONFileBackend (read, read_version, write)."""

    def __init__(self):
        self._data: dict = {}
        self._version = 0
        self.degraded_reason = None

    def read(self) -> dict:
        return copy.deepcopy(self._data)

    def read_version(self):
        return str(self._version)

    def write(self, payload: dict, expected_version=None):
        self._data = copy.deepcopy(payload)
        self._version += 1


def backend(ss) -> MemoryBackend:
    """This session's preview backend, created once and reused so every store built during the preview sees the
    same in-memory project."""
    b = ss.get(KEY_BACKEND)
    if b is None:
        b = MemoryBackend()
        ss[KEY_BACKEND] = b
    return b
