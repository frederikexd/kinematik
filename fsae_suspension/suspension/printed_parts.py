# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  suspension/printed_parts.py — printed polymer, derated honestly
# ============================================================================
"""
What a printed part is actually allowed to carry, and what a substitution costs.

WHY THIS EXISTS

A team plans a printed coolant manifold in PAHT-CF, the print bureau does not
stock it, and the part gets made in Onyx instead because Onyx is "pretty
similar". Then the manifold is "verified through FEA" — with an isotropic
material card, at room temperature, against a yield strength taken from the
in-plane column of a datasheet.

Every one of those steps is where printed parts actually fail:

  ANISOTROPY. A printed part is a laminate. Strength across the layers is
  typically 40–60 % of strength along them, and the number on the front of a
  datasheet is the in-plane one. A tube printed upright has its layer lines
  running around the hoop — which is exactly the direction hoop stress pulls.
  Print orientation is a structural decision, not a shop-floor one.

  TEMPERATURE. Heat-deflection temperature is not a service limit; it is the
  temperature at which a specified beam has already deflected a specified
  amount under a specified load. Useful polymers lose most of their stiffness
  well below it. Coolant at 80 °C against an HDT of 145 °C sounds like a wide
  margin and is not.

  CREEP. Polymers under sustained load at elevated temperature keep moving.
  A bolted joint through a printed flange loses preload over a season, and the
  static FEA that passed says nothing about it.

  MOISTURE. Nylon-based filaments — Onyx and PAHT-CF both — absorb water, and
  a wet part is weaker and tougher than a dry one. Whether the part was dried
  and whether it stays sealed is a strength input.

So this module does not tell you a part is fine. It applies four declared
knockdowns to a published datasheet number, shows each one separately, and
gives back an allowable with the reasoning attached. Where a substitution is
involved it prints the delta, because "pretty similar" is a judgement that
should survive being written down as a ratio.

Every material figure here is a PUBLISHED DATASHEET VALUE, not a measured one,
and datasheet values come from moulded or ideally-printed coupons on someone
else's printer. Print your own coupons. Until you do, treat everything here as
a screening number with a factor of safety chosen accordingly.

Pure Python + NumPy. Self-test: python3 -m suspension.printed_parts
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field as _dcfield
from typing import Dict, List, Optional, Tuple

import numpy as np


__all__ = ["Material", "MATERIALS", "Duty", "Allowable", "derate",
           "substitute", "manifold_check", "ManifoldResult", "material"]


# =========================================================================== #
#  Materials — published datasheet values, named as such
# =========================================================================== #
@dataclass(frozen=True)
class Material:
    key: str
    name: str
    tensile_xy_mpa: float        # in-plane, the number on the datasheet front
    z_factor: float              # across-layer strength / in-plane strength
    modulus_gpa: float
    hdt_c: float                 # heat deflection temperature, 0.45 MPa
    density_g_cm3: float
    hygroscopic: bool
    continuous_fiber: bool       # can it take continuous-fibre reinforcement?
    source: str
    note: str = ""

    @property
    def tensile_z_mpa(self) -> float:
        return self.tensile_xy_mpa * self.z_factor


MATERIALS: Dict[str, Material] = {
    "onyx": Material(
        "onyx", "Markforged Onyx (micro-carbon-filled nylon)",
        tensile_xy_mpa=40.0, z_factor=0.45, modulus_gpa=2.4, hdt_c=145.0,
        density_g_cm3=1.2, hygroscopic=True, continuous_fiber=True,
        source="Markforged published datasheet",
        note="Takes continuous carbon/glass/Kevlar reinforcement, which is "
             "the only route to a large strength gain — and it reinforces "
             "IN-PLANE only, so it does nothing for the across-layer "
             "direction that usually governs."),
    "paht_cf": Material(
        "paht_cf", "PAHT-CF (high-temperature carbon-filled nylon)",
        tensile_xy_mpa=95.0, z_factor=0.40, modulus_gpa=5.3, hdt_c=190.0,
        density_g_cm3=1.16, hygroscopic=True, continuous_fiber=False,
        source="Bambu Lab published datasheet",
        note="Annealing recovers much of the high-temperature performance; "
             "un-annealed parts sit well below the datasheet."),
    "pa12_cf": Material(
        "pa12_cf", "PA12-CF (SLS or filament)",
        tensile_xy_mpa=70.0, z_factor=0.60, modulus_gpa=4.0, hdt_c=160.0,
        density_g_cm3=1.05, hygroscopic=True, continuous_fiber=False,
        source="typical published range",
        note="SLS parts are far less anisotropic than filament parts — the "
             "z_factor here assumes SLS."),
    "petg_cf": Material(
        "petg_cf", "PETG-CF",
        tensile_xy_mpa=45.0, z_factor=0.50, modulus_gpa=3.5, hdt_c=78.0,
        density_g_cm3=1.30, hygroscopic=False, continuous_fiber=False,
        source="typical published range",
        note="HDT below coolant temperature — not a candidate for anything "
             "downstream of a radiator."),
    "asa": Material(
        "asa", "ASA",
        tensile_xy_mpa=42.0, z_factor=0.45, modulus_gpa=2.0, hdt_c=98.0,
        density_g_cm3=1.07, hygroscopic=False, continuous_fiber=False,
        source="typical published range",
        note="UV stable, which matters for anything living outside the "
             "bodywork."),
}


def material(key: str) -> Material:
    k = (key or "").strip().lower().replace("-", "_").replace(" ", "_")
    if k in MATERIALS:
        return MATERIALS[k]
    for m in MATERIALS.values():
        if k and k in m.name.lower():
            return m
    raise KeyError(f"unknown material '{key}' — have "
                   f"{', '.join(sorted(MATERIALS))}")


# =========================================================================== #
#  Duty — what the part is actually asked to do
# =========================================================================== #
@dataclass
class Duty:
    """The service condition. Defaults are a coolant-side printed part."""
    service_temp_c: float = 80.0
    load_is_sustained: bool = True      # held for hours, not a transient peak
    across_layers: bool = True          # does the principal stress cross them?
    dried_and_sealed: bool = False      # was the filament dried, part sealed?
    design_fos: float = 3.0             # required factor of safety


#  --- the four knockdowns, each declared and each reported separately ------ #
def _temp_factor(mat: Material, t_c: float) -> Tuple[float, str]:
    """Strength retention against temperature.

    HDT is not a service limit. This uses a declared linear retention curve:
    full strength up to (HDT − 60) °C, falling to 20 % at HDT, and refusing
    to answer above it. It is deliberately conservative and deliberately
    simple — the honest alternative is a DMA curve for your own printer,
    which nobody has and everybody should.
    """
    knee = mat.hdt_c - 60.0
    if t_c <= knee:
        return 1.0, f"{t_c:g} °C is below the {knee:g} °C knee — no knockdown"
    if t_c >= mat.hdt_c:
        return 0.0, (f"{t_c:g} °C is at or above the {mat.hdt_c:g} °C HDT — "
                     f"this material is not a candidate at this temperature")
    f = 1.0 - 0.8 * (t_c - knee) / (mat.hdt_c - knee)
    return f, (f"{t_c:g} °C sits {mat.hdt_c - t_c:g} °C below HDT — "
               f"retention {f:.0%} on the declared linear curve")


def _orientation_factor(mat: Material, across: bool) -> Tuple[float, str]:
    if not across:
        return 1.0, "principal stress runs in-plane — full datasheet strength"
    return mat.z_factor, (
        f"principal stress crosses the layers — {mat.z_factor:.0%} of the "
        f"in-plane number, because that is what the layer bond carries")


def _creep_factor(sustained: bool, t_c: float, mat: Material
                  ) -> Tuple[float, str]:
    if not sustained:
        return 1.0, "transient load — no sustained-load knockdown"
    hot = t_c > (mat.hdt_c - 90.0)
    f = 0.55 if hot else 0.75
    return f, (f"sustained load{' at elevated temperature' if hot else ''} — "
               f"{f:.0%} to cover creep, which a static FEA does not see")


def _moisture_factor(mat: Material, dried: bool) -> Tuple[float, str]:
    if not mat.hygroscopic:
        return 1.0, "not hygroscopic"
    if dried:
        return 0.95, ("dried and sealed — 5 % held back, because sealed is a "
                      "plan and not a measurement")
    return 0.80, ("nylon-based and not stated as dried/sealed — 20 % off. A "
                  "part printed from wet filament and left in a humid garage "
                  "is measurably weaker than its datasheet")


@dataclass
class Allowable:
    material: Material
    duty: Duty
    base_mpa: float
    factors: List[Tuple[str, float, str]] = _dcfield(default_factory=list)
    allowable_mpa: float = 0.0
    viable: bool = True

    @property
    def total_knockdown(self) -> float:
        f = 1.0
        for _n, v, _w in self.factors:
            f *= v
        return f

    def working_stress_mpa(self) -> float:
        """What you may actually load it to, after the design FoS."""
        return self.allowable_mpa / max(self.duty.design_fos, 1e-6)


def derate(mat_key: str, duty: Optional[Duty] = None) -> Allowable:
    """Datasheet number → allowable stress, with every step shown."""
    mat = material(mat_key)
    d = duty or Duty()
    a = Allowable(material=mat, duty=d, base_mpa=mat.tensile_xy_mpa)
    for name, (f, why) in (
            ("orientation", _orientation_factor(mat, d.across_layers)),
            ("temperature", _temp_factor(mat, d.service_temp_c)),
            ("creep", _creep_factor(d.load_is_sustained, d.service_temp_c,
                                    mat)),
            ("moisture", _moisture_factor(mat, d.dried_and_sealed))):
        a.factors.append((name, f, why))
    a.allowable_mpa = a.base_mpa * a.total_knockdown
    a.viable = a.allowable_mpa > 0.0
    return a


# =========================================================================== #
#  Substitution — "pretty similar", written down as a ratio
# =========================================================================== #
@dataclass
class Substitution:
    wanted: Allowable
    got: Allowable
    ratio: float
    verdict: str
    actions: List[str] = _dcfield(default_factory=list)


def substitute(wanted_key: str, got_key: str,
               duty: Optional[Duty] = None) -> Substitution:
    """What a forced material swap actually costs, at THIS duty.

    The comparison is made after derating both, because two materials with a
    30 % gap on the datasheet can have a 3× gap in service if one of them is
    running much closer to its HDT.
    """
    d = duty or Duty()
    a, b = derate(wanted_key, d), derate(got_key, d)
    ratio = (b.allowable_mpa / a.allowable_mpa) if a.allowable_mpa > 0 \
        else float("nan")
    actions: List[str] = []
    if not b.viable:
        verdict = ("NOT A SUBSTITUTION. The replacement has no strength left "
                   "at this service temperature.")
        actions.append("Change the material or lower the service temperature "
                       "— no amount of wall thickness fixes a part above its "
                       "HDT.")
    elif ratio < 0.5:
        verdict = (f"MATERIAL CHANGE, NOT A SUBSTITUTION. The replacement "
                   f"carries {ratio:.0%} of the allowable stress at this duty. "
                   f"Any FEA run against the original material is void.")
        actions.append(f"Re-run the FEA with "
                       f"{b.allowable_mpa:.1f} MPa allowable, not the "
                       f"datasheet number.")
        actions.append(f"Wall thickness scales roughly as 1/allowable for "
                       f"pressure parts — expect about "
                       f"{1.0/max(ratio,1e-6):.1f}× the wall.")
    elif ratio < 0.85:
        verdict = (f"WORKABLE WITH REWORK — {ratio:.0%} of the original "
                   f"allowable. Not a drop-in.")
        actions.append("Re-check the governing sections; a part sized with "
                       "margin may survive, one sized to a target will not.")
    else:
        verdict = (f"CLOSE ENOUGH TO PROCEED — {ratio:.0%} of the original "
                   f"allowable at this duty. Still re-run the check; close "
                   f"is not the same as covered.")
    if b.material.continuous_fiber and ratio < 0.85:
        actions.append("The replacement accepts continuous-fibre "
                       "reinforcement. That buys in-plane strength only, so "
                       "it helps if — and only if — you also change the print "
                       "orientation so the principal stress runs along the "
                       "fibres.")
    if b.material.hygroscopic and not d.dried_and_sealed:
        actions.append("Dry the filament and seal the part. It is the "
                       "cheapest 20 % on this list.")
    return Substitution(a, b, ratio, verdict, actions)


# =========================================================================== #
#  A printed pressure part
# =========================================================================== #
@dataclass
class ManifoldResult:
    allowable: Allowable
    hoop_mpa: float
    axial_mpa: float
    fos: float
    min_wall_mm: float
    ok: bool
    notes: List[str] = _dcfield(default_factory=list)


def manifold_check(mat_key: str, *, inner_dia_mm: float, wall_mm: float,
                   pressure_bar: float, duty: Optional[Duty] = None,
                   printed_upright: bool = True) -> ManifoldResult:
    """A printed coolant manifold as the pressure vessel it is.

    Thin-wall hoop stress `σ = p·d / (2t)`, axial half that. The orientation
    question is the whole game: a tube printed UPRIGHT has its layer lines
    running around the circumference, so hoop stress — the larger of the two —
    pulls directly across the layer bond. Printed lying down, the hoop
    direction is in-plane and the weak axis carries only the axial stress.

    Cooling systems are also not steady: a 1.0 bar cap sees well over that on
    a heat-soak after shutdown, and this uses the stated pressure as given, so
    state the peak rather than the nominal.
    """
    d = duty or Duty()
    d = Duty(service_temp_c=d.service_temp_c,
             load_is_sustained=d.load_is_sustained,
             across_layers=printed_upright,
             dried_and_sealed=d.dried_and_sealed,
             design_fos=d.design_fos)
    a = derate(mat_key, d)
    p_mpa = float(pressure_bar) * 0.1
    hoop = p_mpa * float(inner_dia_mm) / (2.0 * max(float(wall_mm), 1e-6))
    axial = hoop / 2.0
    governing = hoop
    fos = (a.allowable_mpa / governing) if governing > 0 else float("inf")
    min_wall = (p_mpa * float(inner_dia_mm) * d.design_fos
                / (2.0 * a.allowable_mpa)) if a.allowable_mpa > 0 \
        else float("inf")
    notes = [
        f"Hoop {hoop:.2f} MPa, axial {axial:.2f} MPa, allowable "
        f"{a.allowable_mpa:.2f} MPa after knockdowns.",
    ]
    if printed_upright:
        notes.append(
            "Printed upright, so the hoop stress — the bigger of the two — "
            "pulls straight across the layer bond. Printing it lying down "
            "puts the hoop direction in-plane and leaves only the axial "
            "stress on the weak axis, which is a free factor of two.")
    else:
        notes.append(
            "Printed lying down: the hoop direction is in-plane. Check that "
            "the flanges and any bosses are not now the across-layer "
            "features instead.")
    if a.material.hygroscopic:
        notes.append(
            "Nylon-based and in permanent contact with hot glycol. Fluid "
            "compatibility and long-term swelling are not modelled here and "
            "are a real failure mode for printed wet parts — soak a coupon "
            "for a fortnight before you trust a season on it.")
    notes.append(
        "Thin-wall formula, no stress concentrations. Every port, boss, "
        "flange fillet and printed thread is a concentration this does not "
        "see, and printed parts are notch-sensitive. Treat the wall as the "
        "easy half of the problem.")
    return ManifoldResult(a, hoop, axial, fos, min_wall,
                          bool(fos >= d.design_fos), notes)


# =========================================================================== #
#  Self-test
# =========================================================================== #
def _selftest() -> int:
    fails = 0

    def chk(name, cond, detail=""):
        nonlocal fails
        print(f"  {'PASS' if cond else 'FAIL'}  {name}"
              + (f"   {detail}" if detail and not cond else ""))
        if not cond:
            fails += 1

    print("· derating")
    a = derate("onyx", Duty(service_temp_c=80.0))
    chk("knockdowns applied", a.total_knockdown < 0.5, str(a.total_knockdown))
    chk("allowable well under datasheet",
        a.allowable_mpa < a.base_mpa * 0.5, str(a.allowable_mpa))
    chk("every factor is explained",
        all(w for _n, _v, w in a.factors))

    cold = derate("onyx", Duty(service_temp_c=20.0, load_is_sustained=False,
                               across_layers=False, dried_and_sealed=True))
    chk("benign duty keeps most strength",
        cold.total_knockdown > 0.9, str(cold.total_knockdown))

    over = derate("petg_cf", Duty(service_temp_c=95.0))
    chk("above HDT is not viable", not over.viable)

    print("· substitution")
    s = substitute("paht_cf", "onyx", Duty(service_temp_c=80.0))
    chk("onyx is materially weaker than paht-cf", s.ratio < 0.6, f"{s.ratio}")
    chk("verdict is not reassuring", "NOT A SUBSTITUTION" in s.verdict
        or "MATERIAL CHANGE" in s.verdict, s.verdict)
    chk("actions are concrete", any("FEA" in x for x in s.actions))
    same = substitute("onyx", "onyx", Duty())
    chk("identity substitution is 100 %", abs(same.ratio - 1.0) < 1e-9)

    print("· manifold")
    up = manifold_check("onyx", inner_dia_mm=25.0, wall_mm=3.0,
                        pressure_bar=1.5, printed_upright=True)
    flat = manifold_check("onyx", inner_dia_mm=25.0, wall_mm=3.0,
                          pressure_bar=1.5, printed_upright=False)
    chk("orientation changes the answer", flat.fos > up.fos * 1.5,
        f"{up.fos} vs {flat.fos}")
    chk("min wall is reported", np.isfinite(up.min_wall_mm))
    thin = manifold_check("onyx", inner_dia_mm=40.0, wall_mm=1.0,
                          pressure_bar=2.0, printed_upright=True)
    chk("a thin hot wall fails", not thin.ok, str(thin.fos))

    print(f"\n{'ALL PASS' if not fails else f'{fails} FAILURES'}")
    return 1 if fails else 0


if __name__ == "__main__":
    import sys
    sys.exit(_selftest())
