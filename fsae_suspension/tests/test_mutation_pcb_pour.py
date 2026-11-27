"""Characterization suite for suspension/pcb_pour.py (mutation gate); see tests/_golden.py. The KiCad and Altium demo
boards and a hand-built board with pours, round, rectangular and rounded pads, through pads, vias, tracks and arcs:
every net's grid and worst pad-to-pad resistance (with and without a named pair, few and many pads, a cell budget
that forces the refinement to stop), plus each cell rasterizer on its own."""
import numpy as np

from suspension import pcb_doctor as pd
from suspension import pcb_pour as P
from suspension.pcb_altium import demo_altium_pcb, parse_altium_ascii
from tests._golden import plain
from tests._golden_lazy import check_lazy

HAND = """|RECORD=Board|THICKNESS=1.6mm|
|RECORD=Net|ID=1|NAME=VBAT|
|RECORD=Net|ID=2|NAME=GND|
|RECORD=Component|DESIGNATOR=J1|LAYER=TOP|X=0mm|Y=0mm|
|RECORD=Pad|NAME=1|COMPONENT=0|LAYER=TOP|NET=1|X=1mm|Y=1mm|XSIZE=2mm|YSIZE=1.2mm|SHAPE=RECTANGLE|
|RECORD=Pad|NAME=2|COMPONENT=0|LAYER=MULTILAYER|NET=1|X=19mm|Y=1mm|XSIZE=1.8mm|YSIZE=1.8mm|HOLESIZE=1mm|SHAPE=ROUND|
|RECORD=Pad|NAME=3|COMPONENT=0|LAYER=TOP|NET=1|X=10mm|Y=9mm|XSIZE=1.5mm|YSIZE=0.8mm|SHAPE=ROUNDEDRECTANGLE|ROTATION=90|
|RECORD=Pad|NAME=4|COMPONENT=0|LAYER=BOTTOM|NET=2|X=2mm|Y=12mm|XSIZE=1mm|YSIZE=1mm|SHAPE=ROUND|
|RECORD=Pad|NAME=5|COMPONENT=0|LAYER=BOTTOM|NET=2|X=18mm|Y=12mm|XSIZE=1mm|YSIZE=1mm|SHAPE=ROUND|
|RECORD=Track|LAYER=TOP|NET=1|X1=1mm|Y1=1mm|X2=10mm|Y2=9mm|WIDTH=0.5mm|
|RECORD=Arc|LAYER=TOP|NET=1|X=15mm|Y=5mm|RADIUS=3mm|WIDTH=0.4mm|STARTANGLE=0|ENDANGLE=180|
|RECORD=Via|X=12mm|Y=4mm|DIAMETER=0.8mm|HOLESIZE=0.4mm|NET=1|
|RECORD=Polygon|LAYER=TOP|NET=1|VX0=0mm|VY0=0mm|VX1=20mm|VY1=0mm|VX2=20mm|VY2=3mm|VX3=0mm|VY3=3mm|
|RECORD=Polygon|LAYER=BOTTOM|NET=2|VX0=0mm|VY0=10mm|VX1=20mm|VY1=10mm|VX2=20mm|VY2=14mm|VX3=0mm|VY3=14mm|
|RECORD=Track|LAYER=BOTTOM|NET=2|X1=2mm|Y1=12mm|X2=18mm|Y2=12mm|WIDTH=0.2mm|
"""


def _boards():
    return {"kicad": pd.parse_board(pd.demo_kicad_pcb(), "d.kicad_pcb") if hasattr(pd, "parse_board")
            else pd.parse_kicad_pcb(pd.demo_kicad_pcb()),
            "altium": parse_altium_ascii(demo_altium_pcb()), "hand": parse_altium_ascii(HAND)}


def _pr(d):
    """A solve's result; its grid change is a difference of two near-equal resistances, so it is kept to 1e-4 %."""
    d = plain(d)
    if isinstance(d, dict) and isinstance(d.get("grid_change_pct"), float):
        d["grid_change_pct"] = round(d["grid_change_pct"], 4)
    return d


def _cases():
    for name, b in _boards().items():
        cap = 400_000 if name == "hand" else 60_000
        nets = sorted(n for n in b.nets if n)
        for nid in nets[:6]:
            yield f"{name}.{nid}.grid", P.pour_grid_mm(b, nid)
            yield f"{name}.{nid}.r", _pr(P.pour_resistance(b, nid, max_cells=cap))
            yield f"{name}.{nid}.r2", _pr(P.pour_resistance(b, nid, max_pads=2, max_cells=cap))
            yield f"{name}.{nid}.tiny", _pr(P.pour_resistance(b, nid, max_cells=200))
            yield f"{name}.{nid}.at", plain(P._pour_at(b, nid, 0.5, max_cells=cap))
        yield f"{name}.none", _pr(P.pour_resistance(b, 999, max_cells=60_000))
    hand = _boards()["hand"]
    pads = [p for fp in hand.footprints for p in fp.pads]
    names = [p.number for p in pads]
    yield "hand.pair", _pr(P.pour_resistance(hand, 1, pair=(names[0], names[1]), max_cells=400_000))
    yield "hand.pair_bad", _pr(P.pour_resistance(hand, 1, pair=("1", "nope"), max_cells=60_000))
    h = 0.25
    yield "samples", plain(P._samples(0, 3, 1, 2, h))
    inside = np.zeros((8, 8), bool)
    inside[2:6, 1:5] = True
    yield "reduce", plain(P._reduce(inside, 0, 0))
    for seg in hand.segments[:3]:
        yield f"cells.track.{seg.start}.{seg.end}", plain(P._cells_track(seg, h))
    yield "cells.capsule", [plain(P._cells_capsule(0.0, 0.0, 2.0, 1.0, 0.3, h)),
                            plain(P._cells_capsule(1.0, 1.0, 1.0, 1.0, 0.5, h))]
    yield "cells.disc", [plain(P._cells_disc(1.0, 1.0, 0.6, h)), plain(P._cells_disc(0.0, 0.0, 0.01, h))]
    for p in pads:
        yield f"cells.pad.{p.number}", plain(P._cells_pad(p, h))
    yield "cells.polygon", [plain(P._cells_polygon([(0, 0), (2, 0), (2, 1), (0, 1)], h)),
                            plain(P._cells_polygon([(0, 0), (2, 0), (1, 2)], h)),
                            plain(P._cells_polygon([(0, 0), (1, 1)], h))]
    yield "consts", [P.POUR_TEMP_C, P.MAX_CELLS_PER_LAYER, P.GRID_TOL_PCT, P.K_TERM]


def test_every_net_through_the_sheet_solve():
    check_lazy("pcb_pour", _cases(), rel=1e-6)   # the sheet solve is iterative: repeatable to ~1e-9
