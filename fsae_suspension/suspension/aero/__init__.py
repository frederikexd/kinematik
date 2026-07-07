"""suspension.aero — lazy subpackage facade (PEP 562). Same contract as the
parent package: submodules and re-exports resolve on first touch only."""
import importlib

_SUBMODULES = {"windtunnel", "reference", "scaling"}

_SYMBOL_HOME = {
    "ReferenceAeroModel": "reference",
    "Attitude":           "reference",
    "ScaleSpec":          "scaling",
    "SimilitudePlan":     "scaling",
    "ToleranceBudget":    "scaling",
    "MountAlignment":     "scaling",
    "ScaledRunPlan":      "scaling",
}
_FALLBACK_SCAN = ("reference", "scaling", "windtunnel")

__all__ = sorted(_SUBMODULES | set(_SYMBOL_HOME))


def __getattr__(name):
    if name in _SUBMODULES:
        mod = importlib.import_module(f".{name}", __name__)
        globals()[name] = mod
        return mod
    home = _SYMBOL_HOME.get(name)
    candidates = ([home] if home else []) + [m for m in _FALLBACK_SCAN
                                             if m != home]
    for cand in candidates:
        try:
            mod = importlib.import_module(f".{cand}", __name__)
        except ImportError:
            continue
        if hasattr(mod, name):
            obj = getattr(mod, name)
            globals()[name] = obj
            return obj
    raise AttributeError(
        f"module {__name__!r} has no attribute {name!r} — add it to "
        f"suspension.aero.__init__._SYMBOL_HOME")


def __dir__():
    return __all__
