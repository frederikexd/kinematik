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

One preview per browser: the moment it starts, its start time is written to the browser twice, a cookie and
localStorage (COOKIE_KEY), and every later visit from that browser picks the same clock back up (restore): a reload, a
new tab or a new visit continues the one five minutes, and once they are over the sign-up screen stays. A browser that
lost its cookie but kept localStorage gets the cookie back and reloads once (recover_script). Only clearing the site's
cookies AND its stored data, or a private window, starts a new preview. No IP or fingerprint is used: a whole team
on one network keeps one preview per browser each. This is a nudge, not a paywall.

Pure Python, no Streamlit import: `ss` is any mutable mapping (st.session_state in the app, a dict in tests), and
`now` is seconds since the epoch (time.time() when omitted).
"""
from __future__ import annotations

import copy
import math
import time

PREVIEW_SECONDS = 300            # 5 minutes
WARN_AT_SECONDS = 60             # one toast when this much is left

KEY_START = "_kx_preview_started"
KEY_WARNED = "_kx_preview_warned"
KEY_BACKEND = "_kx_preview_backend"
COOKIE_KEY = "kinematik_preview"        # cookie and localStorage key in the browser: the epoch second it started
COOKIE_MAX_AGE = 63072000               # 2 years, in seconds


def _now(now):
    return time.time() if now is None else float(now)


def start(ss, now: float | None = None) -> bool:
    """Start the preview clock; False if this session already had its preview (the clock is never reset)."""
    if ss.get(KEY_START) is not None:
        return False
    ss[KEY_START] = _now(now)
    return True


def restore(ss, cookies, now: float | None = None) -> bool:
    """Carry this browser's preview into a new session from its cookie (cookies: the request's cookie mapping).
    A cookie that is not a number, or lies in the future, counts as a preview already spent. False when there is
    no cookie or the session already has its clock."""
    if ss.get(KEY_START) is not None:
        return False
    try:
        raw = (cookies or {}).get(COOKIE_KEY)
    except Exception:
        raw = None
    if raw is None or not str(raw).strip():
        return False
    n = _now(now)
    try:
        t0 = float(str(raw).strip())
    except ValueError:
        t0 = math.nan
    ss[KEY_START] = t0 if math.isfinite(t0) and t0 <= n else n - PREVIEW_SECONDS
    return True


def persist_script(start: float, max_age: int = COOKIE_MAX_AGE) -> str:
    """A zero-height snippet that writes the preview's start (epoch seconds) to the parent page's cookie and
    localStorage, so the next session from this browser restores it. Never reloads."""
    return f"""<script>(function() {{
  try {{
    var w = window.parent || window, K = "{COOKIE_KEY}", v = "{int(start)}";
    try {{ w.localStorage.setItem(K, v); }} catch (e) {{}}
    w.document.cookie = K + "=" + v + "; max-age={int(max_age)}; path=/; SameSite=Lax" +
        (w.location.protocol === "https:" ? "; Secure" : "");
  }} catch (e) {{}}
}})();</script>"""


def recover_script() -> str:
    """A zero-height snippet for a session with no preview cookie: if localStorage still remembers a preview (only
    the cookies were cleared), it puts the cookie back and reloads once, so the server sees the preview as used.
    A browser that never had a preview has nothing in localStorage and is left alone; sessionStorage guards against
    a reload loop where cookies are blocked."""
    return f"""<script>(function() {{
  try {{
    var w = window.parent || window, K = "{COOKIE_KEY}", v = null;
    try {{ v = w.localStorage.getItem(K); }} catch (e) {{}}
    if (!v || w.document.cookie.indexOf(K + "=") !== -1) return;
    try {{ if (w.sessionStorage.getItem(K + "_recovered")) return; w.sessionStorage.setItem(K + "_recovered", "1"); }}
    catch (e) {{ return; }}
    w.document.cookie = K + "=" + v + "; max-age={COOKIE_MAX_AGE}; path=/; SameSite=Lax" +
        (w.location.protocol === "https:" ? "; Secure" : "");
    w.location.reload();
  }} catch (e) {{}}
}})();</script>"""


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
