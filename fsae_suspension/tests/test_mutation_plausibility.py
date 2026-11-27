"""Characterization suite for suspension/plausibility.py (mutation gate); see tests/_golden.py. Every range of both
classes at, inside and either side of each bound, the optional terms switched off, lists, None, NaN and negatives,
the specific power a mass and power imply, the warning (text and category), unknown
classes, and whole EV and ICE decks with and without curves, refusals and partial sections."""
import math
import warnings

from suspension import plausibility as P
from tests._golden_lazy import check_lazy


def _run(fn, *a, **kw):
    with warnings.catch_warnings(record=True) as w:
        warnings.simplefilter("always")
        try:
            out = fn(*a, **kw)
        except ValueError as e:
            out = {"raised": str(e)}
    return [out, [[str(x.message), x.category.__name__] for x in w]]


EV = {"vehicle": {"mass_kg": 280.0, "cda_m2": 1.1, "cla_m2": 2.5, "crr": 0.015, "tire_radius_m": 0.23, "mu": 1.6,
                  "cg_height_m": 0.28, "wheelbase_m": 1.55},
      "ev": {"inverter_eff": 0.97, "motor_eff": 0.94, "driveline_eff": 0.95, "final_drive": 12.0,
             "motor_max_rpm": 20000.0, "motor_peak_torque_nm": 30.0, "pack_power_kw": 80.0,
             "pack": {"series": 140, "parallel": 4, "cell_v": 3.6, "cell_ah": 4.2, "cell_ohm": 0.016}}}
ICE = {"vehicle": {"mass_kg": 300.0, "tire_radius_m": 0.25},
       "ice": {"bte": 0.3, "final_drive": 3.0, "redline_rpm": 13000.0, "primary": 2.0, "gears": [2.5, 1.9, 1.5, 1.2],
               "launch_rpm": 6000.0, "driveline_eff": 0.9}}


def _cases():
    for cls, table in (("generic", P.GENERIC), ("fsae", P.FSAE)):
        for name, (lo, hi, *_rest) in sorted(table.items()):
            for x in dict.fromkeys((lo, hi, lo * 0.999 if lo > 0 else lo - 0.001, hi * 1.001, (lo + hi) / 2, 0.0, -1.0)):
                yield f"{cls}.{name}.{x!r}", P.check({name: x}, cls, warn=False)
    yield "none_class", [P.check({"mass_kg": 5e5}, None, warn=False), P.check({"mass_kg": 600.0}, None, warn=False)]
    yield "bad_class", _run(P.check, {"mass_kg": 1.0}, "kart")
    yield "skips", [P.check({"mass_kg": None, "unknown": 1e9, "mu": float("nan"), "crr": 0.0, "cda_m2": 0.0,
                             "cla_m2": 0.0, "aux_power_w": 0.0, "wheel_radius_m": 0.0}, "fsae", warn=False)]
    yield "lists", [P.check({"drivetrain_eff": [0.9, 1.2, 0.2, float("inf")]}, warn=False),
                    P.check({"drivetrain_eff": (0.9, 0.95)}, warn=False),
                    P.check({"mu": [-1.0, 5.0, 0.1]}, "fsae", warn=False), P.check({"efficiency": []}, warn=False)]
    yield "specific", [P.check({"peak_power_w": 80000.0, "mass_kg": 280.0}, "fsae", warn=False),
                       P.check({"peak_power_w": 1e6, "mass_kg": 300.0}, warn=False),
                       P.check({"peak_power_w": 1e6, "mass_kg": 0.0}, warn=False),
                       P.check({"peak_power_w": None, "mass_kg": 300.0}, warn=False),
                       P.check({"peak_power_w": 0.0, "mass_kg": 300.0}, warn=False)]
    yield "where", [P.check({"mass_kg": 608.0}, "fsae", where="deck", warn=False),
                    P.check({"crr": 1.5}, where="", warn=False)]
    yield "warn", _run(P.check, {"mass_kg": 608.0, "mu": 160.0}, "fsae", where="w")
    yield "warn.off", _run(P.check, {"mass_kg": 608.0}, "fsae", warn=False)
    yield "agree", [P._agree("x", d, i, w) for d, i, w in ((None, 10.0, ""), (10.0, None, ""), (10.0, 0.0, ""),
                                                           (float("nan"), 10.0, ""), (10.5, 10.0, ""),
                                                           (10.51, 10.0, "here"), (9.49, 10.0, ""), (9.5, 10.0, ""),
                                                           (20.0, 10.0, "w"))] + [P._agree("x", 10.4, 10.0, "", tol=0.03)]
    curve = ([0.0, 5000.0, 20000.0], [30.0, 30.0, 10.0])
    yield "deck.ev", _run(P.check_deck, EV)
    yield "deck.ev.curve", _run(P.check_deck, EV, motor_curve=curve)
    yield "deck.ev.badcurve", _run(P.check_deck, EV, motor_curve=([0.0, 9000.0], [60.0, 50.0]))
    slip = {"vehicle": dict(EV["vehicle"], mass_kg=617.0, tire_radius_m=230.0),
            "ev": dict(EV["ev"], final_drive=0.08, pack_power_kw=80000.0,
                       pack={"series": 1, "parallel": 1, "cell_v": 3.6, "cell_ah": 4200.0, "cell_ohm": 16.0})}
    yield "deck.ev.slips", _run(P.check_deck, slip, "fsae", "slip.json")
    yield "deck.ev.generic", _run(P.check_deck, slip, "generic")
    yield "deck.ev.nopack", _run(P.check_deck, {"vehicle": EV["vehicle"], "ev": {"pack": {"series": 10}}})
    yield "deck.ev.nokw", _run(P.check_deck, {"vehicle": EV["vehicle"], "ev": {"final_drive": 12.0}})
    for bad in ({"mass_kg": 0.0}, {"mass_kg": -5.0}):
        yield f"deck.mass.{bad['mass_kg']}", _run(P.check_deck, {"vehicle": bad})
    yield "deck.nomass", _run(P.check_deck, {"vehicle": {"mass_kg": None}})
    for sec, k, v in (("ev", "inverter_eff", 97.0), ("ev", "motor_eff", 0.0), ("ev", "driveline_eff", 1.0),
                      ("ice", "driveline_eff", 1.01), ("ice", "bte", 30.0), ("ice", "bte", 0.0), ("ice", "bte", 1.0),
                      ("ice", "bte", None)):
        yield f"deck.eff.{sec}.{k}.{v}", _run(P.check_deck, {"vehicle": {"mass_kg": 280.0}, sec: {k: v}})
    yield "deck.ice", _run(P.check_deck, ICE)
    yield "deck.ice.curve", _run(P.check_deck, ICE, engine_curve=([4000.0, 9000.0, 13000.0], [40.0, 60.0, 45.0]))
    yield "deck.ice.lowpower", _run(P.check_deck, ICE, engine_curve=([2000.0, 5100.0 * 30.0 / math.pi / 20.0],
                                                                     [24.0, 20.0]))
    yield "check.mixed_bad", _run(P.check, {"crr": [1e-5, 0.01, 0.5]}, "generic", warn=False)
    yield "deck.light", _run(P.check_deck, {"vehicle": dict(EV["vehicle"], mass_kg=0.5)})
    for b in (1.0, 0.0, 1.5, 1e-9):
        yield f"deck.bte.{b}", _run(P.check_deck, dict(ICE, ice=dict(ICE["ice"], bte=b)))
    yield "deck.ice.curve.nomass", _run(P.check_deck, {"vehicle": {"tire_radius_m": 0.25}, "ice": ICE["ice"]},
                                        engine_curve=([4000.0, 16000.0], [200.0, 10.0]))
    yield "deck.ice.slips", _run(P.check_deck, {"vehicle": ICE["vehicle"],
                                                "ice": dict(ICE["ice"], primary=0.4, launch_rpm=600.0, bte=0.6)})
    for drop in ("redline_rpm", "gears", "primary", "final_drive", "launch_rpm"):
        ic = {k: v for k, v in ICE["ice"].items() if k != drop}
        yield f"deck.ice.no_{drop}", _run(P.check_deck, {"vehicle": ICE["vehicle"], "ice": ic})
    yield "deck.ice.noradius", _run(P.check_deck, {"vehicle": {"mass_kg": 300.0}, "ice": ICE["ice"]})
    yield "deck.ice.launch0", _run(P.check_deck, {"vehicle": ICE["vehicle"], "ice": dict(ICE["ice"], launch_rpm=0.0)})
    yield "deck.both", _run(P.check_deck, {"vehicle": EV["vehicle"], "ev": EV["ev"], "ice": ICE["ice"]}, "fsae", "")
    yield "tables", [sorted(P.CLASSES, key=str), P.OPTIONAL_TERMS, math.isfinite(P.GENERIC["mass_kg"][1])]


def test_every_range_and_deck():
    check_lazy("plausibility", _cases())
