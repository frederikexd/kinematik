# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  suspension/preview_lock.py — uploads and downloads locked during the preview
# ============================================================================
"""
During the five-minute preview every file upload and download in the app is still drawn, so a visitor sees what
KinematiK can take in and hand back, but disabled, with a lock on its label and the reason on hover
(preview.lock_widget). The app sets preview.LOCK_KEY in the session on every run; signed-in users never see a lock.

Patched once per process, on the DeltaGenerator class (container calls: `col.download_button`) and on the `st`
module (its widgets are bound to the root container at import time), the same way streamlit_app.py patches
widgets for engagement tracking. The lock is read per call from the caller's own session, so one visitor's preview
never locks another user's page.
"""

from __future__ import annotations

import functools

from . import preview as _pv

_PATCHED = "_kx_preview_lock_patched"


def _locked(st) -> bool:
    try:
        return bool(st.session_state.get(_pv.LOCK_KEY))
    except Exception:                       # noqa: BLE001 — outside a script run nothing is locked
        return False


def _wrap(st, orig, note: str):
    @functools.wraps(orig)
    def wrapped(*args, **kwargs):
        if not _locked(st):
            return orig(*args, **kwargs)
        # Class attribute: args[0] is the container; module attribute: already bound.
        head, rest = (args[:1], args[1:]) if args and not isinstance(args[0], str) else ((), args)
        rest, kwargs = _pv.lock_widget(rest, kwargs, note)
        return orig(*head, *rest, **kwargs)
    return wrapped


def install(st) -> None:
    """Patch st.file_uploader and st.download_button (class and module) once per process (unitless)."""
    if getattr(st, _PATCHED, False):
        return
    from streamlit.delta_generator import DeltaGenerator as DG
    for name, note in (("file_uploader", _pv.LOCKED_UPLOAD), ("download_button", _pv.LOCKED_DOWNLOAD)):
        if hasattr(DG, name):
            setattr(DG, name, _wrap(st, getattr(DG, name), note))
        if hasattr(st, name):
            setattr(st, name, _wrap(st, getattr(st, name), note))
    setattr(st, _PATCHED, True)
