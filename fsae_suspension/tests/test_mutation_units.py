"""Characterization suite for suspension/units.py (mutation gate); see tests/_golden.py. Every conversion both ways
and as a difference in both systems, the string, compound, label and sentence rewriters on the docstring examples
and their edge cases, and the three unit-aware widgets against a stand-in container and session state: clamping,
a unit-system switch under a keyed widget, an empty widget value, deltas and the select-slider's labels."""
from types import SimpleNamespace

import pytest

from suspension import units as U
from tests._golden_lazy import check_lazy


class _Box:
    """Records each widget call and answers with a chosen value (the widget's own default when None)."""

    def __init__(self, answer="default"):
        self.answer, self.calls = answer, []

    def _w(self, kind, *a, **kw):
        fmt = kw.get("format_func")
        rec = [kind, list(a), {k: v for k, v in kw.items() if k != "format_func"}]
        if fmt is not None:
            rec.append([fmt(o) for o in kw.get("options", [])])
        self.calls.append(rec)
        if self.answer == "default":
            return kw.get("value", a[3] if len(a) > 3 else None)
        return self.answer

    def number_input(self, *a, **kw):
        return self._w("number_input", *a, **kw)

    def slider(self, *a, **kw):
        return self._w("slider", *a, **kw)

    def select_slider(self, *a, **kw):
        return self._w("select_slider", *a, **kw)


@pytest.fixture
def state(monkeypatch):
    s = SimpleNamespace(session_state={})
    monkeypatch.setattr(U, "st", s)
    return s.session_state


STRINGS = ["12.5", "+12.50", "-3", "0", "abc", "", " 7.25 extra", "1e3", "+0.0"]
LABELS = ["XYmm, mmZ, Akg, kgB, (mm)", "2mmx", "rotor mass (g)", "single-stop peak temp (°C)", "travel (mm, + bump)", "downforce (N) @ 80 km/h",
          "ride height (mm)  ← lower", "rate (N/mm) and (N·m/°)", "cm and mm and m", "kg/s vs kg", "mmHg", "", "kPa·s"]
SENTENCES = ["5 Xmm, 6 mmZ, 7 Akg", "mass 42.00 kg → 41.00 kg", "target mass: 230 kg → 228 kg", "shifted 1530.0 mm", "+5 °C rise", "-3.5 N·m",
             "no units 12", "", "10 km/h and 2 m/s", "12 mmHg", "1.5 m³/h"]


def _cases(state):
    state.clear()
    yield "sys.unset", [U.current_system(), U.is_us()]
    for system in (U.METRIC, U.US, "other"):
        U._PAIR_RES = None          # the sentence patterns are cached on first use: build them under every mutant
        state.clear()
        state["unit_system"] = system
        yield f"{system}.sys", [U.current_system(), U.is_us()]
        for unit in sorted(set(U._CONVERSIONS) | U._PASSTHROUGH | {"zz"}):
            yield f"{system}.{unit}", [U.label(unit), U.from_metric(12.5, unit), U.from_metric_delta(12.5, unit),
                                       U.to_metric(12.5, unit), list(U.display(-4.0, unit)),
                                       [U.convert_value_str(s, unit) for s in STRINGS],
                                       list(U.convert_compound("12.5", unit))]
        yield f"{system}.compound", [list(U.convert_compound(v, u)) for v in ("2.5", "+0.40", "x", "-1")
                                     for u in ("°/10mm", "N/mm @35", "N/mm", "N/mm @ 12.5 front", "N/mm @x",
                                               "kg", "°")]
        yield f"{system}.ulabel", [U.ulabel(t) for t in LABELS]
        yield f"{system}.usentence", [U.usentence(t) for t in SENTENCES]
        yield f"{system}.infer", [U._infer_decimals(t) for t in ("12", "12.50", "+0.125", ".5")]
        for name, kw in (("plain", dict(lo=0.0, hi=100.0, val=25.0, unit="mm")),
                         ("temp", dict(lo=-40.0, hi=150.0, val=20.0, unit="°C", step=5.0, fmt="%.1f")),
                         ("delta", dict(lo=0.0, hi=50.0, val=35.0, unit="°C", is_delta=True, step=1.0)),
                         ("open", dict(lo=None, hi=None, val=None, unit="kg")),
                         ("clamp_lo", dict(lo=10.0, hi=20.0, val=5.0, unit="N")),
                         ("clamp_hi", dict(lo=10.0, hi=20.0, val=50.0, unit="N")),
                         ("unknown", dict(lo=0.0, hi=1.0, val=0.5, unit="zz"))):
            box = _Box()
            r = U.unum(box, "Length (mm)", kw.pop("lo"), kw.pop("hi"), kw.pop("val"), kw.pop("unit"), **kw)
            yield f"{system}.unum.{name}", [r, box.calls]
        for answer in (None, 3.0):
            for lo, hi, val in ((None, None, None), (1.0, 9.0, None), (1.0, 9.0, 4.0)):
                box = _Box(answer)
                yield f"{system}.unum.answer.{answer}.{lo}.{hi}.{val}", [
                    U.unum(box, "x (kg)", lo, hi, val, "kg"), box.calls]
        box = _Box()
        yield f"{system}.uslider", [U.uslider(box, "Rate (N/mm)", 10.0, 80.0, 35.0, "N/mm", step=5.0, help="h"),
                                    U.uslider(box, "T (°C)", 0.0, 120.0, 90.0, "°C"),
                                    U.uslider(box, "dT (°C)", 0.0, 30.0, 45.0, "°C", is_delta=True),
                                    U.uslider(box, "z", 0.0, 1.0, -1.0, "zz"), box.calls]
        box = _Box()
        yield f"{system}.select", [U.uselect_slider(box, "Mesh (mm)", [0.5, 1.0, 2.5, 25.4, 254.0, 1e-5], 1.0, "mm",
                                                    key="m", help="h"),
                                   U.uselect_slider(box, "Count", [1, 2, 3], 2, "zz"), box.calls]
    # a keyed widget whose stored value was entered under the other system
    for widget in ("unum", "uslider"):
        state.clear()
        state["unit_system"] = U.METRIC
        f = getattr(U, widget)
        box = _Box()
        first = f(box, "Len (mm)", 0.0, 1000.0, 254.0, "mm", key="k")
        state["unit_system"] = U.US
        second = f(box, "Len (mm)", 0.0, 1000.0, 254.0, "mm", key="k")
        state["k"] = 5.0
        third = f(box, "Len (mm)", 0.0, 1000.0, 254.0, "mm", key="k")
        state["k"] = "junk"
        fourth = f(box, "Len (mm)", 0.0, 1000.0, 254.0, "mm", key="k")
        state["k"] = 99999.0
        fifth = f(box, "Len (mm)", 0.0, 1000.0, 254.0, "mm", key="k")
        yield f"switch.{widget}", [first, second, third, fourth, fifth, box.calls, sorted(state.items(), key=str)]
    for system in (U.METRIC, U.US):
        state.clear()
        state["unit_system"] = system
        state["_u_None"] = (5.0, U.METRIC if system == U.US else U.US)
        box = _Box()
        r = [U.unum(box, "a (mm)", 0.0, 100.0, 20.0, "mm", help="h"), U.uslider(box, "b (mm)", 0.0, 100.0, 20.0, "mm")]
        yield f"keyless.{system}", [r, box.calls, sorted(state.items(), key=str)]
        for lo, hi, stored in ((None, 10.0, 50.0), (20.0, None, 5.0), (None, None, 7.0), (0.0, 10.0, "junk")):
            state["q"] = stored
            box = _Box()
            yield f"stored.{system}.{lo}.{hi}.{stored}", [U.unum(box, "q (kg)", lo, hi, 4.0, "kg", key="q"),
                                                           box.calls, state.get("q")]
        state.pop("q", None)
        state.pop("_u_q", None)
        box = _Box(None)
        yield f"nofallback.{system}", [U.unum(box, "q (kg)", None, None, 4.0, "kg", key="q"), box.calls]
    state.clear()
    state["unit_system"] = U.METRIC
    box = _Box()
    U.unum(box, "s (mm)", 0.0, 1000.0, 254.0, "mm", key="s")
    U.uslider(box, "t (mm)", 0.0, 1000.0, 254.0, "mm", key="t")
    state["unit_system"] = U.US
    U.unum(box, "s (mm)", 0.0, 1000.0, 254.0, "mm", key="s")
    U.uslider(box, "t (mm)", 0.0, 1000.0, 254.0, "mm", key="t")
    yield "switched_state", sorted(state.items(), key=str)
    state.clear()
    state["unit_system"] = U.US
    state["_u_e"] = (12.0, U.US)
    box = _Box(None)
    yield "shadow_fallback", [U.unum(box, "x (mm)", None, None, None, "mm", key="e"), box.calls]
    state["_u_f"] = ("bad",)
    with pytest.raises(ValueError):
        U.unum(_Box(None), "x (mm)", None, None, None, "mm", key="f")
    state["_u_g"] = ("bad", U.US)
    yield "shadow_unreadable", [U.unum(_Box(None), "x (mm)", None, None, None, "mm", key="g")]


def test_every_conversion_and_widget(state):
    check_lazy("units", _cases(state))


def test_no_streamlit_means_metric(monkeypatch):
    monkeypatch.setattr(U, "st", None)
    assert U.current_system() == U.METRIC and U.from_metric(25.4, "mm") == 25.4

    class Broken:
        @property
        def session_state(self):
            raise RuntimeError("no session")
    monkeypatch.setattr(U, "st", Broken())
    assert U.current_system() == U.METRIC
