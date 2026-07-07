"""suspension — lazy package facade (PEP 562).

Importing this package executes NOTHING heavy. Submodules load on first
attribute access (`suspension.tubeframe` → imports tubeframe only), and the
re-exported symbols (`suspension.Hardpoints`, …) resolve through _SYMBOL_HOME
on first touch. Result: `import suspension` costs microseconds; each subsystem
pays its own numpy/scipy/pandas bill only when actually used.

NOTE: the distributed archive contained only `tubeframe.py`; _SYMBOL_HOME maps
each public symbol to its conventional home module. If a symbol lives
elsewhere in your tree, either correct the entry or rely on the _FALLBACK_SCAN
(tries the listed candidates in order — slower on the first miss only, never a
crash-on-rename).
"""
import importlib

# Public submodules — anything importable as suspension.<name>.
_SUBMODULES = {
    "topologies", "fullcar3d", "compliance", "flex", "chassis", "integration",
    "project", "tiremodel", "setup", "laptime", "correlation", "ev_powertrain",
    "lapsim", "ev_electrical_check", "ev_excel_roundtrip", "pack_thermal",
    "damper", "interfaces", "transient", "ggv", "tire_thermal", "units",
    "bracket_fos", "bolted_joint", "tubeframe", "tractive_system",
    "pcm_cooling", "dfmea", "risk_propagation", "process_library",
    "pt_integration", "registry", "cad_ingest", "analytics", "mem_utils",
    "aero", "pcb_doctor", "kinematics", "dynamics", "mythbuster",
    "status_dashboard",
}

# symbol -> home submodule for the package-level re-exports.
_SYMBOL_HOME = {
    "SuspensionKinematics": "kinematics",
    "Hardpoints":           "kinematics",
    "GenericKinematics":    "topologies",
    "list_templates":       "topologies",
    "example":              "topologies",
    "VehicleDynamics":      "dynamics",
    "VehicleParams":        "dynamics",
    "MATERIALS":            "compliance",
    "MemberStiffness":      "compliance",
    "CompliantCorner":      "compliance",
    "corner_wheel_load":    "compliance",
    "WheelLoad":            "compliance",
    "load_flex_body":       "flex",
}

# Ordered candidates scanned only when a _SYMBOL_HOME entry is wrong/missing.
_FALLBACK_SCAN = ("kinematics", "dynamics", "topologies", "compliance", "flex")

__all__ = sorted(_SUBMODULES | set(_SYMBOL_HOME))


def __getattr__(name):
    if name in _SUBMODULES:
        mod = importlib.import_module(f".{name}", __name__)
        globals()[name] = mod              # cache: __getattr__ never re-fires
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
        f"module {__name__!r} has no attribute {name!r} — if this is a real "
        f"symbol, add it to suspension.__init__._SYMBOL_HOME")


def __dir__():
    return __all__
