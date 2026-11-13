# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Cells from their manufacturers' datasheets, so a pack run starts from sourced numbers instead of a generic cell.

Only what a datasheet states is filled in, with the source on the entry; anything a datasheet leaves out stays None
and the pack falls back to KinematiK's generic 21700 value for it, which `defaults_used` names so the report can
say so. Two things datasheets rarely give and the thermal model needs:

* DC internal resistance (the I²R heat). Most give AC impedance at 1 kHz, which reads well below the DC value that
  heats the cell (the P42A: 10 mΩ AC, 16 mΩ DC at 10 A for 1 s). An AC-only figure is NOT used as the heating
  resistance; enter a measured DC value or accept the flagged default.
* Specific heat. No datasheet here gives it; the generic 1000 J/(kg·K) is a literature-typical value and is flagged.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace

from .pack_thermal import CellParams, default_cell_params


@dataclass(frozen=True)
class CellSpec:
    name: str
    diameter_mm: float
    length_mm: float
    mass_g: float                        # datasheet maximum where only a maximum is given
    capacity_ah: float                   # typical, or the minimum where only that is given
    nominal_v: float
    max_continuous_a: float | None       # None: the datasheet gives no continuous rating
    r_dc_ohm: float | None               # DC internal resistance; None: the datasheet gives none
    r_ac_1khz_ohm: float | None          # AC impedance at 1 kHz, for reference only (not the heating resistance)
    source: str

    @property
    def surface_area_m2(self) -> float:
        """Can surface for convection, m²: the side plus both ends."""
        d, length = self.diameter_mm * 1e-3, self.length_mm * 1e-3
        return math.pi * d * length + 2.0 * math.pi * d * d / 4.0


CELL_LIBRARY: dict[str, CellSpec] = {s.name: s for s in (
    CellSpec("Molicel INR-21700-P42A", 21.7, 70.2, 70.0, 4.2, 3.6, 45.0, 0.016, 0.010,
             "Molicel P42A datasheet V4 (80092): 4200 mAh typical, 45 A continuous, 70 g max, 21.7 x 70.2 mm max, "
             "AC 1 kHz 10 mΩ, DC 10 A/1 s 16 mΩ. https://www.rcscomponents.kiev.ua/datasheets/INR21700P42A-V4-80092.pdf"),
    CellSpec("Molicel INR-21700-P45B", 21.4, 70.0, 70.0, 4.5, 3.6, 45.0, None, 0.008,
             "Molicel P45B datasheet: 4.5 Ah typical, 45 A continuous, 70.0 g max, 21.40 x 70.00 mm, AC 1 kHz "
             "<= 8 mΩ (no DC value given). https://www.kronium.cz/uploads/p45b-datasheet.pdf"),
    CellSpec("Samsung INR21700-40T", 21.22, 70.3, 70.0, 4.0, 3.6, 35.0, None, None,
             "Samsung SDI INR21700-40T datasheet: 4000 mAh min standard, 35 A continuous without temperature cut "
             "(45 A with an 80 °C cut), 70.0 g max, 21.22 x 70.30 mm max; no resistance given. "
             "https://cdn.simpower.co.nz/wp-content/uploads/2021/09/INR21700-40T.pdf"),
    CellSpec("Sony/Murata US18650VTC6", 18.5, 65.2, 46.5, 3.13, 3.6, None, None, None,
             "Sony US18650VTC6 datasheet: 3130 mAh at 0.2C, 46.5 g average, 18.50 x 65.20 mm max; no continuous "
             "rating and no resistance value given. https://www.kronium.cz/uploads/SONY_US18650VTC6.pdf"),
)}


def cell_params(name: str, *, r_dc_ohm: float | None = None, pitch_mm: float | None = None,
                cp_j_kgk: float | None = None) -> tuple[CellParams, list[str]]:
    """CellParams for a library cell, and the fields left at KinematiK's generic default (unitless list of names).

    `r_dc_ohm` (Ω, a measured DC resistance) overrides the datasheet's; `pitch_mm` (mm) sets the grid spacing;
    `cp_j_kgk` (J/(kg·K)) a measured specific heat. Raises KeyError for an unknown name."""
    spec = CELL_LIBRARY[name]
    base = default_cell_params()
    used = []
    r = r_dc_ohm if r_dc_ohm is not None else spec.r_dc_ohm
    if r is None:
        r = base.r_internal_ohm
        used.append("DC internal resistance")
    cp = cp_j_kgk if cp_j_kgk is not None else base.cp
    if cp_j_kgk is None:
        used.append("specific heat")
    p = replace(base, mass_kg=spec.mass_g * 1e-3, cp=cp, r_internal_ohm=r, nominal_v=spec.nominal_v,
                surface_area_m2=spec.surface_area_m2,
                pitch_mm=pitch_mm if pitch_mm is not None else base.pitch_mm)
    return p, used
