# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The one place an input becomes a float array, for every module that needs it."""
from __future__ import annotations

import numpy as np


def finite(name: str, values, min_len: int = 1, max_abs: float | None = None) -> np.ndarray:
    """``values`` as a 1-D float array in their own units (points in mm stay in mm), or ValueError naming ``name``: every entry must be a finite
    number (no nan, no inf), within ±``max_abs`` when that is given, and there must be at least ``min_len`` of them. For data that came from a file or a
    logger, where one bad sample would otherwise turn every result downstream into nan without a word."""
    try:
        a = np.asarray(values, float).ravel()
    except (TypeError, ValueError):
        raise ValueError(f"{name}: not a list of numbers") from None
    if a.size < min_len:
        raise ValueError(f"{name}: needs at least {min_len} value(s), got {a.size}")
    bad = int(np.count_nonzero(~np.isfinite(a)))
    if bad:
        raise ValueError(f"{name}: {bad} of {a.size} value(s) are not finite numbers (nan or inf)")
    if max_abs is not None and a.size and float(np.max(np.abs(a))) > max_abs:
        raise ValueError(f"{name}: a value of {float(a[np.argmax(np.abs(a))]):g} is beyond ±{max_abs:g}")
    return a


def finite_scalar(name: str, value, positive: bool = False) -> float:
    """``value`` as a float in its own units (a mass in kg stays in kg), or ValueError naming ``name`` when it is
    not a finite number (or, with ``positive``, not above zero)."""
    try:
        v = float(value)
    except (TypeError, ValueError):
        raise ValueError(f"{name}: not a number") from None
    if not np.isfinite(v):
        raise ValueError(f"{name}: {v} is not a finite number")
    if positive and v <= 0:
        raise ValueError(f"{name}: must be above zero, got {v:g}")
    return v


def all_finite(obj) -> bool:
    """True when no float in ``obj`` (numbers, arrays, and lists, tuples, dicts or dataclasses of them, to any
    depth) is nan or inf; unitless, the values are only tested. For the last line of a function that must not hand back a poisoned result."""
    if isinstance(obj, (bool, str, bytes, int)) or obj is None:
        return True
    if isinstance(obj, (float, np.floating)):
        return bool(np.isfinite(obj))
    if isinstance(obj, np.ndarray):
        return obj.dtype.kind != "f" or bool(np.isfinite(obj).all())
    if isinstance(obj, dict):
        return all(all_finite(v) for v in obj.values())
    if isinstance(obj, (list, tuple, set)):
        return all(all_finite(v) for v in obj)
    if hasattr(obj, "__dict__"):
        return all(all_finite(v) for v in vars(obj).values())
    return True


def vec(v) -> np.ndarray:
    """``v`` as a float array (no copy when it already is one); units are unchanged, so a point in mm stays in mm."""
    return np.asarray(v, float)
