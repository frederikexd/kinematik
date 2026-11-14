# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The cell library carries only datasheet numbers, and says which pack inputs still fall back to a default."""
import math

import pytest

from suspension import cell_library as cl
from suspension.pack_thermal import default_cell_params


def test_every_entry_cites_its_datasheet():
    for spec in cl.CELL_LIBRARY.values():
        assert "http" in spec.source and spec.mass_g > 0 and spec.capacity_ah > 0


def test_p42a_uses_the_dc_resistance_not_the_ac_impedance():
    p, used = cl.cell_params("Molicel INR-21700-P42A")
    assert p.r_internal_ohm == 0.016 and cl.CELL_LIBRARY["Molicel INR-21700-P42A"].r_ac_1khz_ohm == 0.010
    assert p.mass_kg == pytest.approx(0.070) and used == ["specific heat"]
    d, length = 0.0217, 0.0702
    assert p.surface_area_m2 == pytest.approx(math.pi * d * length + math.pi * d * d / 2.0, rel=1e-12)


def test_ac_only_and_missing_resistance_fall_back_and_say_so():
    base = default_cell_params()
    for name in ("Molicel INR-21700-P45B", "Samsung INR21700-40T", "Sony/Murata US18650VTC6"):
        p, used = cl.cell_params(name)
        assert p.r_internal_ohm == base.r_internal_ohm and used == ["DC internal resistance", "specific heat"]


def test_measured_values_override():
    p, used = cl.cell_params("Samsung INR21700-40T", r_dc_ohm=0.012, pitch_mm=27.0, cp_j_kgk=950.0)
    assert (p.r_internal_ohm, p.pitch_mm, p.cp, used) == (0.012, 27.0, 950.0, [])
    assert cl.cell_params("Samsung INR21700-40T")[0].pitch_mm == default_cell_params().pitch_mm


def test_unknown_cell_raises():
    with pytest.raises(KeyError):
        cl.cell_params("not a cell")
