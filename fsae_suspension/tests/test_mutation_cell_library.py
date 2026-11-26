"""Characterization suite for suspension/cell_library.py (mutation gate); see tests/_golden.py. Every library cell's
datasheet fields and can area, and its pack parameters with and without a measured DC resistance (including zero),
specific heat and pitch, plus an unknown name."""
import dataclasses

from suspension import cell_library as C
from tests._golden import plain
from tests._golden_lazy import check_lazy


def _cases():
    for name, spec in C.CELL_LIBRARY.items():
        yield f"spec.{name}", [plain(spec), spec.surface_area_m2]
        for kw in ({}, {"r_dc_ohm": 0.021}, {"r_dc_ohm": 0.0}, {"cp_j_kgk": 950.0}, {"cp_j_kgk": 0.0},
                   {"pitch_mm": 23.5}, {"pitch_mm": 0.0}, {"r_dc_ohm": 0.012, "cp_j_kgk": 1100.0, "pitch_mm": 25.0}):
            p, used = C.cell_params(name, **kw)
            yield f"params.{name}.{sorted(kw.items())}", [dataclasses.asdict(p), used]
    try:
        C.cell_params("Nope 99")
    except KeyError as e:
        yield "unknown", str(e)
    yield "names", sorted(C.CELL_LIBRARY)
    # every shipped cell is 3.6 V, the generic value: a 3.7 V one shows the datasheet voltage is carried
    C.CELL_LIBRARY["test 3.7 V"] = C.CellSpec("test 3.7 V", 18.0, 65.0, 45.0, 2.5, 3.7, 20.0, 0.03, None, "test")
    try:
        p, used = C.cell_params("test 3.7 V")
        yield "params.3v7", [dataclasses.asdict(p), used]
    finally:
        del C.CELL_LIBRARY["test 3.7 V"]


def test_every_cell_and_override():
    check_lazy("cell_library", _cases())
