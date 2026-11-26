"""suspension/pcb_altium.py on hand-written ASCII exports against tests/golden: every unit the reader accepts and the
default-unit vote, every field alias, nets by ID (from 0, from 5, duplicated), by name and by file order, unknown
references, inner and plane layers in stack order, pours and keep-outs with outlines, and the helpers one by one.
Regenerate with KINEMATIK_REGOLD=1 only after reading the diff."""
import dataclasses
import json
import math
import os
from pathlib import Path

import pytest

from suspension import pcb_altium as pa

GOLD = Path(__file__).parent / "golden" / "pcb_altium.json"

ALIASES = """|RECORD=Board|THICKNESS=1.2mm|
|record=net|NETNAME=A|
|RECORD= Net |NAME=B|
|RECORD=Net|
|RECORD=Component|DESIGNATOR=U1|SOURCECOMMENT=chip|LAYER=BOTTOM|LOCATION.X=10mm|LOCATION.Y=5mm|
|RECORD=Component|NAME=J1|VALUE=conn|LAYER=TOP|X=1mm|Y=1mm|
|RECORD=Component|SOURCEDESCRIPTION=desc|LAYER=MID1|
|RECORD=Component|COMMENT=R=10k|SOURCEDESIGNATOR=R9|X=3mm|Y=4mm|
|RECORD=Pad|DESIGNATOR=3|COMPONENT=0|LAYER=BOTTOM|NET=B|LOCATION.X=10mm|LOCATION.Y=5mm|TOPXSIZE=1mm|TOPYSIZE=2mm|HOLE=0mm|TOPSHAPE=OCTAGONAL|
|RECORD=Pad|NAME=1|COMPONENT=7|LAYER=MULTILAYER|NET=|X=2mm|Y=2mm|SIZEX=1.5mm|SIZEY=1.5mm|DRILL=0.8mm|SHAPE=ROUND|ROTATION=45|
|RECORD=Pad|NAME=2|LAYER=TOP|NET=Z|X=4mm|Y=4mm|
|RECORD=Track|LAYER=MIDLAYER10|NET=1|X1=0mm|Y1=0mm|X2=5mm|Y2=0mm|WIDTH=0.3mm|
|RECORD=Track|LAYER=MID LAYER 2|NET=A|X1=0mm|Y1=1mm|X2=5mm|Y2=1mm|WIDTH=12mil|
|RECORD=Track|LAYER=TOPOVERLAY|NET=1|X1=0mm|Y1=2mm|X2=5mm|Y2=2mm|WIDTH=0.2mm|
|RECORD=Track|LAYER=TOP|NET=9|X1=0mm|Y1=3mm|X2=5mm|Y2=3mm|WIDTH=8|
|RECORD=Arc|LAYER=BOTTOM|NET=2|LOCATION.X=6mm|LOCATION.Y=6mm|RADIUS=2mm|WIDTH=0.25mm|STARTANGLE=0|ENDANGLE=90|
|RECORD=Arc|LAYER=TOP|NET=2|X=6mm|Y=6mm|RADIUS=1mm|WIDTH=0.25mm|
|RECORD=Via|LOCATION.X=3mm|LOCATION.Y=3mm|VIADIAMETER=0.8mm|DRILL=0.4mm|NET=2|STARTLAYER=TOP|ENDLAYER=MID10|
|RECORD=Via|X=4mm|Y=3mm|NET=1|
|RECORD=Polygon|LAYER=PLANE1|NET=1|VX0=0mm|VY0=0mm|VX1=10mm|VY1=0mm|VX2=10mm|VY2=10mm|
|RECORD=SplitPlane|LAYER=Internal Plane 3|NET=2|VX0=0mm|VY0=0mm|VX1=4mm|VY1=0mm|VX2=4mm|VY2=bad|
|RECORD=Region|LAYER=TOP|NET=2|KEEPOUT=YES|VX0=0mm|VY0=0mm|VX1=1mm|VY1=0mm|VX2=1mm|VY2=1mm|
|RECORD=Fill|LAYER=TOP|NET=2|TEARDROP=1|VX0=0mm|VY0=0mm|VX1=1mm|VY1=0mm|VX2=1mm|VY2=1mm|
|RECORD=Region|LAYER=BOTTOM|NET=2|ISBOARDCUTOUT=true|VX0=0mm|VY0=0mm|VX1=2mm|VY1=0mm|VX2=2mm|VY2=2mm|
|RECORD=Region|LAYER=BOTTOM|NET=1|KEEPOUT=FALSE|VX0=0mm|VY0=0mm|VX1=3mm|VY1=0mm|VX2=3mm|VY2=3mm|VX3=0mm|
|RECORD=Net|ID=9|NAME=FOX"""

IDS_FROM_5 = """|RECORD=Net|ID=5|NAME=P|
|RECORD=Net|ID=6|NAME=Q|
|RECORD=Track|LAYER=TOP|NET=6|X1=0|Y1=0|X2=100|Y2=0|WIDTH=10|
|RECORD=Track|LAYER=TOP|NET=0|X1=0|Y1=10|X2=100|Y2=10|WIDTH=10mil|
|RECORD=Track|LAYER=BOTTOM|NET=P|X1=0|Y1=20|X2=100|Y2=20|WIDTH=1mm|
|RECORD=Track|LAYER=BOTTOM|X1=0|Y1=30|X2=100|Y2=30|WIDTH=10|
|RECORD=Region|LAYER=BOTTOM|NET=5|VX0=0|VY0=0|VX1=100|VY1=0|VX2=100|VY2=100|
"""

DUPLICATE_IDS = """|RECORD=Net|ID=1|NAME=P|\r
|RECORD=Net|ID=1|NAME=Q|\r
|RECORD=Track|LAYER=TOP|NET=1|X1=0mm|Y1=0mm|X2=1mm|Y2=0mm|WIDTH=0.2mm|\r
"""

TIE = """|RECORD=Board|BOARDTHICKNESS=62mil|
|RECORD=Track|LAYER=TOP|NET=0|X1=1mm|Y1=0|X2=100|Y2=0|WIDTH=10|
"""

NO_COPPER = """|RECORD=Board|BOARDTHICKNESS=1.6mm|
|RECORD=Net|ID=0|NAME=GND|
|RECORD=Component|SOURCEDESIGNATOR=U1|LAYER=TOP|X=1mm|Y=1mm|
"""

UNITLESS = """|RECORD=Board|LAYERSTACKTHICKNESS=47|
|RECORD=Net|ID=0|NAME=GND|
|RECORD=Net|ID=1|NAME=VBAT|
|RECORD=Component|SOURCEDESIGNATOR=U7|SOURCEDESCRIPTION=regulator|LAYER=BOTTOM|
|RECORD=Component|SOURCEDESIGNATOR=U8|VALUE=v|LAYER=MID1|X=100|Y=200|
|RECORD=Pad|NAME=1|COMPONENT=0|LAYER=BOTTOM|NET=1|X=100|Y=100|XSIZE=40|YSIZE=30|HOLESIZE=12|SHAPE=ROUNDED|ROTATION=30|
|RECORD=Pad|NAME=2|COMPONENT=0|LAYER=TOP|NET=1|X=200|Y=100|TOPXSIZE=50|TOPYSIZE=20|HOLE=13|TOPSHAPE=RECTANGLE|ROTATION=x|
|RECORD=Pad|NAME=3|COMPONENT=1|LAYER=MULTILAYER|NET=0|X=300|Y=100|SIZEX=60|SIZEY=60|DRILL=35|SHAPE=OCTAGONAL|
|RECORD=Pad|NAME=4|COMPONENT=1|LAYER=MID1|NET=0|X=400|Y=100|
|RECORD=Pad|NAME=5|LAYER=TOP|NET=1|X=500|Y=100|XSIZE=20|YSIZE=20|
|RECORD=Line|LAYER=TOP|NET=1|X1=0|Y1=0|X2=400|Y2=0|WIDTH=15|
|RECORD=Track|LAYER=TOP|NET=1|X1=0|Y1=0|WIDTH=15|
|RECORD=Track|LAYER=TOP|NET=1|X1=0|Y1=0|X2=10|Y2=10|
|RECORD=Track|LAYER=TOP|NET=0|X1=0|Y1=50|X2=400|Y2=50|WIDTH=20|
|RECORD=Arc|LAYER=TOP|NET=1|X=200|Y=200|RADIUS=50|WIDTH=10|STARTANGLE=45|ENDANGLE=135|
|RECORD=Arc|LAYER=TOP|NET=1|X=200|Y=200|RADIUS=0|WIDTH=10|
|RECORD=Arc|LAYER=TOP|NET=1|X=200|Y=200|RADIUS=20|
|RECORD=Arc|LAYER=TOP|NET=1|X=200|Y=200|RADIUS=20|WIDTH=8|STARTANGLE=bad|
|RECORD=Arc|LAYER=TOP|NET=0|X=250|Y=250|RADIUS=30|WIDTH=8|
|RECORD=Via|X=100|Y=300|DIAMETER=30|HOLESIZE=14|NET=1|STARTLAYER=MULTILAYER|ENDLAYER=MULTILAYER|
|RECORD=Via|X=200|Y=300|SIZE=28|HOLE=15|NET=1|STARTLAYER=MID2|ENDLAYER=BOTTOM|
|RECORD=Via|X=300|Y=300|VIADIAMETER=26|DRILL=11|NET=0|
|RECORD=Via|NET=0|
|RECORD=Plane|LAYER=PLANE2|NET=1|
|RECORD=Polygon|LAYER=TOP|NET=1|VX0=0|VY0=0|VX1=100|VY1=0|VX2=100|VY2=100|
|RECORD=Polygon|LAYER=PLANE3|NET=0|
|RECORD=Polygon|LAYER=TOPOVERLAY|NET=1|
|RECORD=Fill|LAYER=MULTILAYER|NET=1|VX0=0|VY0=0|VX1=10|VY1=0|VX2=10|VY2=10|
|RECORD=Region|LAYER=MECHANICAL1|NET=1|
|RECORD=Region|LAYER=TOP|NET=0|VX0=0|VY0=0|VX1=10|VY1=0|VX2=10|VY2=10|
"""

THICK = """|RECORD=Board|THICKNESS=12mm|
|RECORD=Track|LAYER=TOP|NET=0|X1=0|Y1=0|X2=1mm|Y2=0|WIDTH=0.2mm|
"""

HUGE = """|RECORD=Track|LAYER=TOP|NET=0|X1=0|Y1=0|X2=80000000|Y2=0|WIDTH=10|
"""

TALL = """|RECORD=Track|LAYER=TOP|NET=0|X1=0|Y1=0|X2=1|Y2=80000000|WIDTH=10|
"""

EDGES = """|RECORD=Board|THICKNESS=1mm|
|RECORD=Net|ID=1|NAME=P|
|RECORD=Arc|LAYER=TOPOVERLAY|NET=1|X=5mm|Y=5mm|RADIUS=1mm|WIDTH=0.2mm|
|RECORD=Track|LAYER=TOP|NET=1|X1=0mm|Y1=0mm|X2=5mm|Y2=0mm|WIDTH=0.3mm|
|RECORD=Arc|LAYER=TOP|NET=1|X=5mm|Y=5mm|RADIUS=1mm|WIDTH=10|STARTANGLE=0|ENDANGLE=45|
|RECORD=Via|Y=3mm|NET=1|
|RECORD=Via|X=4mm|Y=3mm|NET=1|
|RECORD=Pad|X=1mm|NET=1|
|RECORD=Pad|LAYER=TOP|NET=1|X=2mm|Y=2mm|DRILL=1mm|
|RECORD=Plane|LAYER=PLANE2|NET=1|VX0=0mm|VY0=0mm|VX1=3mm|VY1=0mm|VX2=3mm|VY2=3mm|
|RECORD=Polygon|LAYER=TOP|VX0=0mm|VY0=0mm|VX1=3mm|VY1=0mm|VX2=3mm|VY2=3mm|
|RECORD=Polygon|LAYER=TOPOVERLAY|NET=1|VX0=0mm|VY0=0mm|VX1=3mm|VY1=0mm|VX2=3mm|VY2=3mm|
|RECORD=Region|LAYER=MID5|NET=1|VX0=0mm|VY0=0mm|VX1=2mm|VY1=0mm|VX2=2mm|VY2=2mm|
"""

OFFSET = """|RECORD=Track|LAYER=TOP|NET=0|X1=3000mm|Y1=0mm|X2=3010mm|Y2=0mm|WIDTH=0.3mm|
"""

TINY = """|RECORD=Track|LAYER=TOP|NET=0|X1=0mm|Y1=0mm|X2=0.5mm|Y2=0mm|WIDTH=0.1mm|
"""

PADS = """|RECORD=Pad|NAME=1|LAYER=TOP|X=1mm|Y=1mm|
|RECORD=Pad|NAME=2|LAYER=BOTTOM|X=3mm|Y=1mm|
"""

CASES = {"unitless": UNITLESS, "thick": THICK, "aliases": ALIASES, "ids_from_5": IDS_FROM_5, "duplicate_ids": DUPLICATE_IDS, "tie": TIE,
         "no_copper": NO_COPPER, "demo": pa.demo_altium_pcb(), "edges": EDGES, "offset": OFFSET, "tiny": TINY,
         "pads": PADS}


def _plain(x):
    if dataclasses.is_dataclass(x) and not isinstance(x, type):
        return {f.name: _plain(getattr(x, f.name)) for f in dataclasses.fields(x) if f.name != "text"}
    if isinstance(x, dict):
        return {str(k): _plain(v) for k, v in x.items()}
    if isinstance(x, (list, tuple, set)):
        return [_plain(v) for v in (sorted(x, key=repr) if isinstance(x, set) else x)]
    if isinstance(x, float):
        return None if math.isnan(x) else float(f"{x:.9g}")
    return x


def test_every_export_word_for_word():
    got = json.loads(json.dumps({k: _plain(pa.parse_altium_ascii(t)) for k, t in CASES.items()}))
    if os.environ.get("KINEMATIK_REGOLD"):
        GOLD.write_text(json.dumps(got, indent=0) + "\n", encoding="utf-8")
    want = json.loads(GOLD.read_text(encoding="utf-8"))
    for case in want:
        for part in want[case]:
            assert got[case][part] == want[case][part], (case, part)


def test_a_board_in_internal_units_is_refused():
    for text in (HUGE, TALL):
        with pytest.raises(ValueError, match=r"^this board measures 2,032,000 mm across, which is not a PCB\. Its "
                                             r"lengths have no unit suffix, so they were read as mils; they are most "
                                             r"likely Altium \*internal\* units \(1/10000 mil\), which would make "
                                             r"the real board 203\.2 mm\."):
            pa.parse_altium_ascii(text)
    tiny = pa.parse_altium_ascii(HUGE.replace("80000000", "39000"))
    assert any("990.6" in n or "mm" in n for n in tiny.notes)
    assert pa.is_altium_binary(pa.OLE_MAGIC + b"rest") and not pa.is_altium_binary("\ud800 text")


def test_the_helpers():
    assert pa._default_unit("|A=1mm|B=2mil|") == "mil" and pa._default_unit("|A=1mm|B=2mm|C=3mil|") == "mm"
    assert pa._len_mm("5", "mil") == pytest.approx(0.127) and pa._len_mm("5", "mm") == 5.0
    assert pa._len_mm(None, "mm") is None
    assert pa._len_mm("2in", "mm") == 50.8 and pa._len_mm('1"', "mm") == 25.4 and pa._len_mm("x", "mm") is None
    assert [pa.altium_layer_to_kicad(n) for n in ("MultiLayer", "multi", "Top Layer", "bottom_layer", "MID-3",
                                                  "Signal12", "GND2", "Mechanical1", "", None)] == [
        "MULTI", "MULTI", "F.Cu", "B.Cu", "In3.Cu", "In12.Cu", "In2.Cu", None, None, None]
    assert pa._len_unit("10mil") == (0.0254, "mil") and pa._len_unit("1.5MM") == (1.0, "mm")
    assert pa._len_unit("7") == (0.0254, "") and pa._len_unit("7", "mm") == (1.0, "") and pa._len_unit(None) == (
        0.0254, "") and pa._len_unit("", "mm") == (1.0, "")
    assert [pa._truthy(v) for v in ("TRUE", " yes ", "1", "0", "no", None, "")] == [True, True, True, False, False,
                                                                                   False, False]
    assert pa._is_plane("Internal Plane 2") and pa._is_plane("gnd_1") and not pa._is_plane(None)
    assert not pa._is_plane("MID1")
    assert pa._order_layers({"B.Cu", "In10.Cu", "In2.Cu", "F.Cu"}) == ["F.Cu", "In2.Cu", "In10.Cu", "B.Cu"]
    assert pa._order_layers(set()) == ["F.Cu", "B.Cu"] and pa._order_layers({"In3.Cu"}) == ["In3.Cu"]
    assert pa._fields("|K=a=b|", 10)["K"] == ("a=b", 13, 16)
    assert [pa._mil(v) for v in (0.0254, 1.0, 0.254, 0.0)] == ["1mil", "39.3701mil", "10mil", "0mil"]
    assert pa.is_altium_ascii("|record=Net|") and not pa.is_altium_ascii("no records")
    pad = "x" * 19993
    assert not pa.is_altium_ascii(pad + "|RECORD=Net|") and pa.is_altium_ascii("x" * 19992 + "|RECORD=Net|")
    with pytest.raises(ValueError, match=r"^no \|RECORD= entries found — this is not an Altium/Protel ASCII PCB\. If "
                                         r"it is a native \(binary\) \.PcbDoc, re-save it as ASCII; see the export "
                                         r"note in the panel\.$"):
        pa.parse_altium_ascii("(kicad_pcb)")


def test_the_thickness_window_and_the_size_limit():
    track = "|RECORD=Track|LAYER=TOP|NET=0|X1=0mm|Y1=0mm|X2=5mm|Y2=0mm|WIDTH=0.3mm|\n"
    default = pa.parse_altium_ascii(track).board_thickness_mm
    for th, want in (("0.1", default), ("1.0", 1.0), ("10.0", default), ("10.5", default), ("9.5", 9.5)):
        assert pa.parse_altium_ascii(f"|RECORD=Board|THICKNESS={th}mm|\n" + track).board_thickness_mm == want, th
    long = "|RECORD=Track|LAYER=TOP|NET=0|X1=0mm|Y1=0mm|X2={}mm|Y2=0mm|WIDTH=0.3mm|\n"
    assert pa.parse_altium_ascii(long.format("2000")).segments
    with pytest.raises(ValueError, match="2,000 mm across"):
        pa.parse_altium_ascii(long.format("2000.5"))
    e = pa.parse_altium_ascii(EDGES)
    assert e.board_thickness_mm == 1.0 and "In5.Cu" in e.copper_layers and len(e.vias) == 1
    assert {p.number for fp in e.footprints for p in fp.pads} == {"?"} and [z.layer for z in e.zones] == ["In2.Cu", "In5.Cu"]
    assert pa.parse_altium_ascii(PADS).copper_layers == ["F.Cu", "B.Cu"]
    assert pa.altium_layer_to_kicad(None) is None and pa.altium_layer_to_kicad("") is None
