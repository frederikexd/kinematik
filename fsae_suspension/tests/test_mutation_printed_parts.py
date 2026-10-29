"""suspension/printed_parts.py: each declared knockdown at both sides of its breakpoints, with its stated reason, the
allowable as their product, the substitution verdict bands and actions, and the thin-wall manifold (hoop p d / 2t,
axial half of it, minimum wall p d n / 2 sigma) printed upright and lying down."""
import math

import pytest

from suspension import printed_parts as pp

ONYX, PAHT, PETG, ASA = (pp.MATERIALS[k] for k in ("onyx", "paht_cf", "petg_cf", "asa"))


def test_the_datasheet_library():
    want = {"onyx": (40.0, 0.45, 2.4, 145.0, 1.2, True, True), "paht_cf": (95.0, 0.40, 5.3, 190.0, 1.16, True, False),
            "pa12_cf": (70.0, 0.60, 4.0, 160.0, 1.05, True, False), "petg_cf": (45.0, 0.50, 3.5, 78.0, 1.30, False, False),
            "asa": (42.0, 0.45, 2.0, 98.0, 1.07, False, False)}
    for k, v in want.items():
        m = pp.MATERIALS[k]
        assert (m.key, m.tensile_xy_mpa, m.z_factor, m.modulus_gpa, m.hdt_c, m.density_g_cm3, m.hygroscopic,
                m.continuous_fiber) == (k, *v) and m.source and m.note
    assert ONYX.tensile_z_mpa == pytest.approx(18.0)


def test_material_lookup():
    assert pp.material("onyx") is ONYX and pp.material(" PAHT-CF ") is PAHT and pp.material("paht cf") is PAHT
    assert pp.material("markforged") is ONYX and pp.material("PETG") is PETG
    for bad in ("kevlar", "", None):
        with pytest.raises(KeyError) as e:
            pp.material(bad)
        assert str(e.value) == f"\"unknown material '{bad}' — have asa, onyx, pa12_cf, paht_cf, petg_cf\""


def test_the_duty_defaults():
    d = pp.Duty()
    assert (d.service_temp_c, d.load_is_sustained, d.across_layers, d.dried_and_sealed, d.design_fos) == \
        (80.0, True, True, False, 3.0)


def test_the_temperature_knockdown():
    assert pp._temp_factor(ONYX, 85.0) == (1.0, "85 °C is below the 85 °C knee — no knockdown")
    f, why = pp._temp_factor(ONYX, 115.0)
    assert f == pytest.approx(1 - 0.8 * 30 / 60) and why == "115 °C sits 30 °C below HDT — retention 60% on the declared linear curve"
    assert pp._temp_factor(ONYX, 145.0) == (0.0, "145 °C is at or above the 145 °C HDT — this material is not a "
                                                  "candidate at this temperature")
    assert pp._temp_factor(ONYX, 144.9)[0] == pytest.approx(1 - 0.8 * 59.9 / 60)


def test_orientation_creep_and_moisture():
    assert pp._orientation_factor(ONYX, False) == (1.0, "principal stress runs in-plane — full datasheet strength")
    assert pp._orientation_factor(ONYX, True) == (0.45, "principal stress crosses the layers — 45% of the in-plane "
                                                        "number, because that is what the layer bond carries")
    assert pp._creep_factor(False, 200.0, ONYX) == (1.0, "transient load — no sustained-load knockdown")
    assert pp._creep_factor(True, 55.0, ONYX) == (0.75, "sustained load — 75% to cover creep, which a static FEA does "
                                                        "not see")
    assert pp._creep_factor(True, 55.1, ONYX) == (0.55, "sustained load at elevated temperature — 55% to cover creep, "
                                                        "which a static FEA does not see")
    assert pp._moisture_factor(PETG, False) == (1.0, "not hygroscopic")
    assert pp._moisture_factor(ONYX, True)[0] == 0.95 and pp._moisture_factor(ONYX, True)[1].startswith("dried and seal")
    f, why = pp._moisture_factor(ONYX, False)
    assert f == 0.80 and why.startswith("nylon-based and not stated as dried/sealed — 20 % off.")


def test_the_allowable_is_the_product_of_the_knockdowns():
    a = pp.derate("onyx")
    assert [n for n, *_ in a.factors] == ["orientation", "temperature", "creep", "moisture"]
    assert [v for _, v, _ in a.factors] == pytest.approx([0.45, 1.0, 0.55, 0.80])
    assert a.total_knockdown == pytest.approx(0.45 * 0.55 * 0.80)
    assert a.allowable_mpa == pytest.approx(40 * 0.45 * 0.55 * 0.8) and a.viable is True and a.base_mpa == 40.0
    assert a.working_stress_mpa() == pytest.approx(a.allowable_mpa / 3.0)
    assert a.duty == pp.Duty() and a.material is ONYX
    hot = pp.derate("petg_cf", pp.Duty(service_temp_c=95.0))
    assert hot.allowable_mpa == 0.0 and hot.viable is False
    z = pp.Allowable(ONYX, pp.Duty(design_fos=0.0), 40.0, allowable_mpa=2.0)
    assert z.working_stress_mpa() == pytest.approx(2.0 / 1e-6) and z.total_knockdown == 1.0
    cold = pp.derate("asa", pp.Duty(service_temp_c=20.0, load_is_sustained=False, across_layers=False,
                                    dried_and_sealed=True))
    assert cold.total_knockdown == 1.0 and cold.allowable_mpa == 42.0


def test_the_substitution_bands():
    d = pp.Duty(service_temp_c=60.0)
    s = pp.substitute("paht_cf", "onyx", d)
    r = pp.derate("onyx", d).allowable_mpa / pp.derate("paht_cf", d).allowable_mpa
    assert s.ratio == pytest.approx(r) and r < 0.5
    assert s.verdict == (f"MATERIAL CHANGE, NOT A SUBSTITUTION. The replacement carries {r:.0%} of the allowable stress "
                         f"at this duty. Any FEA run against the original material is void.")
    assert s.actions[0] == f"Re-run the FEA with {s.got.allowable_mpa:.1f} MPa allowable, not the datasheet number."
    assert s.actions[1] == (f"Wall thickness scales roughly as 1/allowable for pressure parts — expect about "
                            f"{1 / r:.1f}× the wall.")
    assert s.actions[2].startswith("The replacement accepts continuous-fibre reinforcement.")
    assert s.actions[3] == "Dry the filament and seal the part. It is the cheapest 20 % on this list."
    assert len(s.actions) == 4 and s.wanted.material is PAHT and s.got.material is ONYX
    same = pp.substitute("onyx", "onyx", pp.Duty(dried_and_sealed=True))
    assert same.ratio == pytest.approx(1.0) and same.actions == []
    assert same.verdict == ("CLOSE ENOUGH TO PROCEED — 100% of the original allowable at this duty. Still re-run the "
                            "check; close is not the same as covered.")
    mid = pp.substitute("asa", "petg_cf", pp.Duty(service_temp_c=40.0, load_is_sustained=False))
    rm = mid.ratio
    assert 0.5 <= rm < 0.85 or mid.verdict.startswith("CLOSE")
    if 0.5 <= rm < 0.85:
        assert mid.verdict == f"WORKABLE WITH REWORK — {rm:.0%} of the original allowable. Not a drop-in."
    dead = pp.substitute("paht_cf", "petg_cf", pp.Duty(service_temp_c=100.0))
    assert dead.verdict == "NOT A SUBSTITUTION. The replacement has no strength left at this service temperature."
    assert dead.actions == ["Change the material or lower the service temperature — no amount of wall thickness fixes "
                            "a part above its HDT."]
    nan = pp.substitute("petg_cf", "onyx", pp.Duty(service_temp_c=100.0))
    assert math.isnan(nan.ratio)


def test_a_rework_substitution_with_and_without_fibre():
    d = pp.Duty(service_temp_c=15.0, load_is_sustained=False, across_layers=False, dried_and_sealed=True)
    s = pp.substitute("asa", "petg_cf", d)                  # 45 vs 42 MPa, both dry: close enough
    assert s.ratio == pytest.approx(45.0 / 42.0) and s.verdict.startswith("CLOSE ENOUGH") and s.actions == []
    w = pp.substitute("pa12_cf", "onyx", d)                 # 40 * 0.95 / (70 * 0.95)
    assert w.ratio == pytest.approx(40 / 70)
    assert w.verdict == "WORKABLE WITH REWORK — 57% of the original allowable. Not a drop-in."
    assert w.actions[0] == "Re-check the governing sections; a part sized with margin may survive, one sized to a target will not."
    assert w.actions[1].startswith("The replacement accepts continuous-fibre") and len(w.actions) == 2


def test_the_printed_manifold():
    up = pp.manifold_check("onyx", inner_dia_mm=25.0, wall_mm=3.0, pressure_bar=1.5)
    a = pp.derate("onyx", pp.Duty(across_layers=True))
    hoop = 0.15 * 25 / 6
    assert (up.hoop_mpa, up.axial_mpa, up.fos) == pytest.approx((hoop, hoop / 2, a.allowable_mpa / hoop))
    assert up.min_wall_mm == pytest.approx(0.15 * 25 * 3 / (2 * a.allowable_mpa)) and up.ok == (up.fos >= 3.0)
    assert up.notes[0] == (f"Hoop {up.hoop_mpa:.2f} MPa, axial {up.axial_mpa:.2f} MPa, allowable {a.allowable_mpa:.2f} MPa "
                           "after knockdowns.")
    assert up.notes[1].startswith("Printed upright, so the hoop stress") and up.notes[2].startswith("Nylon-based")
    assert up.notes[3].startswith("Thin-wall formula, no stress concentrations.") and len(up.notes) == 4
    flat = pp.manifold_check("onyx", inner_dia_mm=25.0, wall_mm=3.0, pressure_bar=1.5, printed_upright=False)
    assert flat.allowable.duty.across_layers is False and flat.fos == pytest.approx(up.fos / 0.45)
    assert flat.notes[1].startswith("Printed lying down: the hoop direction is in-plane.")
    dry = pp.manifold_check("asa", inner_dia_mm=20.0, wall_mm=2.0, pressure_bar=1.0,
                            duty=pp.Duty(service_temp_c=30.0, load_is_sustained=False, dried_and_sealed=True,
                                         design_fos=2.0, across_layers=False))
    assert len(dry.notes) == 3 and dry.allowable.duty.across_layers is True and dry.allowable.duty.design_fos == 2.0
    assert dry.allowable.duty.service_temp_c == 30.0 and dry.allowable.duty.load_is_sustained is False
    assert dry.allowable.duty.dried_and_sealed is True
    assert dry.ok == (dry.fos >= 2.0)
    assert dry.min_wall_mm == pytest.approx(0.1 * 20 * 2 / (2 * dry.allowable.allowable_mpa))
    none = pp.manifold_check("onyx", inner_dia_mm=25.0, wall_mm=3.0, pressure_bar=0.0)
    assert none.fos == math.inf and none.ok is True
    zero_wall = pp.manifold_check("onyx", inner_dia_mm=25.0, wall_mm=0.0, pressure_bar=1.0)
    assert zero_wall.hoop_mpa == pytest.approx(0.1 * 25 / 2e-6)
    hot = pp.manifold_check("petg_cf", inner_dia_mm=25.0, wall_mm=3.0, pressure_bar=1.0)
    assert hot.min_wall_mm == math.inf and hot.fos == 0.0 and hot.ok is False


def test_the_advice_text_is_exact():
    from suspension import printed_parts as pp
    up = pp.manifold_check("paht_cf", inner_dia_mm=20.0, wall_mm=3.0, pressure_bar=2.0)
    assert up.notes[1:] == [
        "Printed upright, so the hoop stress — the bigger of the two — pulls straight across the layer bond. Printing it "
        "lying down puts the hoop direction in-plane and leaves only the axial stress on the weak axis, which is a free "
        "factor of two.",
        "Nylon-based and in permanent contact with hot glycol. Fluid compatibility and long-term swelling are not "
        "modelled here and are a real failure mode for printed wet parts — soak a coupon for a fortnight before you "
        "trust a season on it.",
        "Thin-wall formula, no stress concentrations. Every port, boss, flange fillet and printed thread is a "
        "concentration this does not see, and printed parts are notch-sensitive. Treat the wall as the easy half of the "
        "problem."]
    flat = pp.manifold_check("asa", inner_dia_mm=20.0, wall_mm=3.0, pressure_bar=2.0, printed_upright=False)
    assert flat.notes[1:3] == ["Printed lying down: the hoop direction is in-plane. Check that the flanges and any "
                               "bosses are not now the across-layer features instead.", up.notes[-1]]
    assert pp._moisture_factor(pp.MATERIALS["onyx"], True) == (
        0.95, "dried and sealed — 5 % held back, because sealed is a plan and not a measurement")
    assert pp._moisture_factor(pp.MATERIALS["onyx"], False) == (
        0.80, "nylon-based and not stated as dried/sealed — 20 % off. A part printed from wet filament and left in a "
              "humid garage is measurably weaker than its datasheet")
    s = pp.substitute("paht_cf", "onyx", pp.Duty(service_temp_c=80.0))
    assert ("The replacement accepts continuous-fibre reinforcement. That buys in-plane strength only, so it helps if "
            "— and only if — you also change the print orientation so the principal stress runs along the fibres."
            ) in s.actions


def test_the_dried_flag_reaches_the_moisture_factor():
    from suspension import printed_parts as pp
    wet = pp.derate("onyx", pp.Duty(dried_and_sealed=False))
    dry = pp.derate("onyx", pp.Duty(dried_and_sealed=True))
    assert dict((n, f) for n, f, _ in dry.factors)["moisture"] == 0.95
    assert dry.allowable_mpa == pytest.approx(wet.allowable_mpa * 0.95 / 0.80)


def test_an_allowable_under_one_megapascal_still_counts(monkeypatch):
    from dataclasses import replace
    from suspension import printed_parts as pp
    weak = replace(pp.MATERIALS["asa"], tensile_xy_mpa=1.0)
    monkeypatch.setitem(pp.MATERIALS, "weak", weak)
    a = pp.derate("weak")
    assert 0.0 < a.allowable_mpa < 1.0 and a.viable is True
    m = pp.manifold_check("weak", inner_dia_mm=20.0, wall_mm=3.0, pressure_bar=0.1)
    assert m.min_wall_mm == pytest.approx(0.01 * 20.0 * 3.0 / (2.0 * m.allowable.allowable_mpa))
    s = pp.substitute("weak", "asa")
    assert s.ratio == pytest.approx(pp.derate("asa").allowable_mpa / a.allowable_mpa)
