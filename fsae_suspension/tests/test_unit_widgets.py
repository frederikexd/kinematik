# ============================================================================
#  KinematiK — tests for the unit-aware widget wrappers in suspension/units.py
#  Locks in the metric<->US round-trip contract used by every feature's inputs.
# ============================================================================
import sys, os, types
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pytest
from suspension import units as u


class _Col:
    """Minimal container that echoes the displayed value back (like a widget
    left at its default), so we can assert the metric value round-trips."""
    def __init__(self, ss): self.ss = ss
    def number_input(self, label, *a, **k):
        val = a[2] if len(a) > 2 else k.get("value")
        if val is None:
            val = a[0] if a and a[0] is not None else 0.0
        if k.get("key"): self.ss[k["key"]] = val
        return val
    def slider(self, label, *a, **k):
        val = a[2] if len(a) > 2 else k.get("value")
        if val is None:
            val = a[0] if a and a[0] is not None else 0.0
        return val
    def select_slider(self, label, options=None, value=None, **k):
        return value if value is not None else (list(options)[0] if options else None)


@pytest.fixture
def stub_st(monkeypatch):
    st = types.ModuleType("streamlit")
    class SS(dict):
        def get(self, k, d=None): return dict.get(self, k, d)
    st.session_state = SS()
    monkeypatch.setattr(u, "st", st, raising=False)
    return st


def _metric(stub_st): stub_st.session_state["unit_system"] = "metric"
def _us(stub_st): stub_st.session_state["unit_system"] = "us"


def test_metric_is_identity(stub_st):
    _metric(stub_st)
    c = _Col(stub_st.session_state)
    # value passed in metric comes back in metric unchanged
    assert u.unum(c, "Width (mm)", 30.0, 200.0, 60.0, "mm", step=5.0) == pytest.approx(60.0)
    assert u.uslider(c, "Travel (mm)", 0.0, 60.0, 25.0, "mm") == pytest.approx(25.0)


def test_us_absolute_length_round_trip(stub_st):
    _us(stub_st)
    c = _Col(stub_st.session_state)
    # 60 mm displays as 2.3622 in; echoed back and converted to metric == 60 mm
    got = u.unum(c, "Width (mm)", 30.0, 200.0, 60.0, "mm", step=5.0)
    assert got == pytest.approx(60.0, abs=1e-9)


def test_us_temperature_absolute_has_offset(stub_st):
    _us(stub_st)
    # 82 C -> 179.6 F on display, and the wrapper converts the echoed F back to C
    assert u.from_metric(82.0, "°C") == pytest.approx(179.6)
    c = _Col(stub_st.session_state)
    assert u.unum(c, "Core temp (°C)", 40.0, 120.0, 82.0, "°C") == pytest.approx(82.0)


def test_us_temperature_delta_has_no_offset(stub_st):
    _us(stub_st)
    # a 35 C span is a 63 F-degree span (scale only, no +32)
    assert u.from_metric_delta(35.0, "°C") == pytest.approx(63.0)
    c = _Col(stub_st.session_state)
    got = u.unum(c, "Window half-width (°C)", 15.0, 55.0, 35.0, "°C", is_delta=True)
    assert got == pytest.approx(35.0, abs=1e-9)
    # and the delta path must differ from the absolute path in US mode
    assert u.from_metric(35.0, "°C") != pytest.approx(u.from_metric_delta(35.0, "°C"))


def test_none_bounds_unbounded_input(stub_st):
    _us(stub_st)
    c = _Col(stub_st.session_state)
    # unbounded number_input (min/max None) must not crash and must round-trip
    got = u.unum(c, "HV accumulator x (mm)", None, None, -150.0, "mm", step=5.0, key="t_hv")
    assert got == pytest.approx(-150.0, abs=1e-9)


@pytest.mark.parametrize("val,unit,us_label", [
    (2.6, "m²", "ft²"), (22.0, "km", "mi"), (300.0, "cc", "in³"),
    (12000.0, "N·m/rad", "lbf·ft/rad"), (1.1, "m/s", "mph"),
    (460.0, "MPa", "ksi"), (19.05, "mm", "in"),
])
def test_new_units_round_trip_and_label(stub_st, val, unit, us_label):
    _us(stub_st)
    assert u.label(unit) == us_label
    disp = u.from_metric(val, unit)
    back = u.to_metric(disp, unit)
    assert back == pytest.approx(val, rel=1e-9)


def test_uselect_slider_returns_metric_option(stub_st):
    _us(stub_st)
    c = _Col(stub_st.session_state)
    # options are metric; the metric choice is returned regardless of display
    got = u.uselect_slider(c, "Mesh cell (mm)", [1.0, 1.5, 2.0, 2.5], 1.5, "mm")
    assert got == 1.5


def test_ulabel_switches_units_in_label(stub_st):
    _us(stub_st)
    assert u.ulabel("Width (mm)") == "Width (in)"
    assert u.ulabel("Drag CdA (m²)") == "Drag CdA (ft²)"
    assert u.ulabel("Yield (MPa)") == "Yield (ksi)"
