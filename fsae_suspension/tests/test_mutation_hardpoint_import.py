"""Characterization suite for suspension/hardpoint_import.py (mutation gate); see tests/_golden.py. OptimumK-style
CSV and XLSX in every unit, frame, mirror and re-origin choice; headerless, semicolon and odd-column files; four
corners in one file; MacPherson, multi-link and trailing-arm vocabularies; duplicates and ambiguous names; and
garbage, which is refused with a reason."""
import io
import sys
from pathlib import Path

import openpyxl

from suspension import hardpoint_import as hi
from tests._golden import plain
from tests._golden_lazy import check_lazy

sys.path.insert(0, str(Path(__file__).parent))
from test_hardpoint_import import OPTIMUMK_CSV  # noqa: E402

FOUR = b"""Name,X [mm],Y [mm],Z [mm]
UCA front inboard FL,100,-240,280
UCA rear inboard FL,-130,-240,299
LCA front inboard FL,110,-200,122
LCA rear inboard FL,-140,-200,117
UCA outboard FL,-12,-540,300
LCA outboard FL,5,-575,110
Tie rod inner FL,-100,-230,160
Tie rod outer FL,-90,-560,150
Wheel centre FL,0,-600,228
Contact patch FL,0,-605,0
UCA front inboard FR,100,240,280
LCA outboard FR,5,575,110
Wheel centre FR,0,600,228
Front Right lower ball joint,5,575,111
RL upper front pivot,1650,-230,290
RR wheel center,1650,590,230
Rear Left LCA outer,1655,-570,105
"""
MACPHERSON = b"""point,x,y,z
Strut top mount,0.02,0.48,0.62
Lower control arm front pivot,0.15,0.22,0.12
Lower control arm rear pivot,-0.18,0.24,0.11
Lower ball joint,0.0,0.58,0.10
Steering tie rod inner,-0.12,0.25,0.15
Steering tie rod outer,-0.11,0.57,0.14
Wheel centre,0.0,0.62,0.23
Tyre contact patch,0.0,0.63,0.0
"""
MULTI = b"""name;x;y;z
link1 inboard;10;250;150
link1 outboard;12;560;140
link 2 inner;-150;250;160
link 2 outer;-140;560;150
third link chassis;50;270;300
third link upright;40;540;310
link_4 inboard;-120;270;290
link_4 outboard;-115;545;300
control arm 5 in;0;300;200
control arm 5 out;0;520;205
link7 inboard;1;2;3
wheel center;0;600;230
"""
TRAILING = b"""Point,X,Y,Z
Trailing arm pivot,12.0,9.0,9.5
Trailing arm hub,0.0,22.0,9.0
Hub carrier,0.0,23.0,9.0
Spring lower mount,-2.0,18.0,8.0
Spring top,-2.0,18.0,20.0
Wheel center,0.0,24.0,9.0
"""
MESSY = b"""Point,Long (in),Lat (in),Vert (in)
upper front inner,4,9,11
upper front inner,4.1,9,11
upper,1,2,3
,1,2,3
lower outer,0.2,22.6,4.3
pushrod inboard,1,10,15
rocker pivot,1.5,10,16
bellcrank axis,1.5,10.5,16
spring chassis,2,8,18
damper rocker end,2,9,17
1,2,3,4
nan,inf,-inf,1
"""
HEADERLESS = b"""Upper Ball Joint;-12;540;300
Lower Ball Joint;5;575;110
Wheel Center;0;600;228
"""


def _xlsx(csv_bytes, two_sheets=False):
    wb = openpyxl.Workbook()
    ws = wb.active
    for line in csv_bytes.decode().strip().split("\n"):
        ws.append([c if not c.replace(".", "", 1).replace("-", "", 1).isdigit() else float(c)
                   for c in line.split(",")])
    if two_sheets:
        ws2 = wb.create_sheet("Rear")
        ws2.append(["Point", "X", "Y", "Z"])
        ws2.append(["RL wheel center", 1600, -590, 230])
    b = io.BytesIO()
    wb.save(b)
    return b.getvalue()


def _xlsx_odd():
    """A workbook with a formula cell, a boolean cell, a blank row between two tables and a second sheet."""
    wb = openpyxl.Workbook()
    ws = wb.active
    ws.append(["Name", "X", "Y", "Z"])
    ws.append(["Upper ball joint", "=1+2", 540, 300])
    ws.append(["Lower ball joint", True, 575, 110])
    ws.append([None, None, None, None])
    ws.append(["Point", "X [m]", "Y [m]", "Z [m]"])
    ws.append(["Wheel centre", 0.0, 0.6, 0.228])
    ws2 = wb.create_sheet("Rear")
    ws2.append(["Name", "X", "Y", "Z"])
    ws2.append(["RL wheel center", 1600, -590, 230])
    b = io.BytesIO()
    wb.save(b)
    return b.getvalue()


def _xlsx_sheets():
    """A first sheet whose header puts the name last, with a blank row and then a headerless row; a second sheet
    of headerless rows only: neither the blank row's table nor the first sheet's columns may carry over."""
    wb = openpyxl.Workbook()
    ws = wb.active
    ws.append(["X", "Y", "Z", "Name"])
    ws.append([1, 2, 3, "Upper ball joint"])
    ws.append([None, None, None, None])
    ws.append(["Lower ball joint", 4, 5, 6])
    ws2 = wb.create_sheet("Mid")
    ws2.append(["X", "Y", "Z", "Name"])
    ws2.append([1, 2, 3, "Tie rod outer"])
    ws3 = wb.create_sheet("Rear")
    ws3.append(["Wheel center", 7, 8, 9])
    b = io.BytesIO()
    wb.save(b)
    return b.getvalue()


UNIT_WORDS = ["mm", "millimeter", "millimetre", "millimeters", "millimetres", "m", "meter", "metre", "meters",
              "metres", "in", "inch", "inches", '"']
EXTRA = {f"unit_{i}": (f"Point,X ({u}),Y ({u}),Z ({u})\nUpper ball joint,1.5,2.5,3.5\nWheel center,0.5,6.0,2.3\n"
                       .encode(), "u.csv") for i, u in enumerate(UNIT_WORDS)}
EXTRA.update({
    "bom": (b"\xef\xbb\xbfName,X,Y,Z\nUpper ball joint,1,2,3\n", "bom.csv"),
    "badbytes": (b"Name,X,Y,Z\nUpper ball joint\xff,1,2,3\n", "bad.csv"),
    "late_delim": (b"# note\n" * 10 + b"Name;X;Y;Z\nUpper ball joint;1;2;3\n", "late.txt"),
    "tabs": (b"Name\tX (in)\tY (in)\tZ (in)\nUpper ball joint\t1\t22\t12\n", "tabs.tsv"),
    "comma_dec": (b"Name;X [m];Y [m];Z [m]\nUpper ball joint;0,012;0,54;0,3\nLower ball joint;0,005;0,575;0,11\n",
                  "comma.csv"),
    "two_tables": (b"Name,X,Y,Z\nUpper ball joint,1,2,3\n\nPoint,X,Y,Z\nLower ball joint,4,5,6\n,,,\n"
                   b"Name,X,Y,Z\nWheel center,7,8,9\n", "two.csv"),
    "noname_hdr": (b"X,Y,Z,Label\n1,2,3,upper ball joint\n4,5,6,lower ball joint\n", "nn.csv"),
    "numeric_hdr": (b"Name,1,2,3\nUpper ball joint,1,2,3\n", "num.csv"),
    "pk_csv": (b"PK not really a zip\n", "fake.csv"), "csv_as_xlsm": (b"Name,X,Y,Z\nA,1,2,3\n", "a.xlsm"),
    "csv_as_xltx": (b"Name,X,Y,Z\nA,1,2,3\n", "a.xltx"), "noext": (b"Name,X,Y,Z\nUpper ball joint,1,2,3\n", ""),
    "xlsx_noext": (_xlsx(b"Name,X,Y,Z\nUpper ball joint,1,2,3"), "blob.bin"), "xlsx_odd": (_xlsx_odd(), "odd.xlsx"),
    "comment_first": (b"Comment,Name,X,Y,Z\nfront,Upper ball joint,1,2,3\nrear,Lower ball joint,4,5,6\n", "cf.csv"),
    "fused": (b"Name,X,Y,Z\nTieRod outer,1,2,3\nTrackrod inner,2,3,4\nToeLink outer,3,4,5\nWheelCenter,4,5,6\n"
              b"WheelCentre RL,5,6,7\nBellcrank pivot,6,7,8\nPushRod outer,7,8,9\nPullrod inner,8,9,1\n"
              b"BallJoint upper,9,1,2\nContactPatch,1,3,5\n", "fused.csv"),
    "partial_tags": (b"Name,X,Y,Z\nfront upper ball joint,1,600,3\nleft wheel center,2,-600,4\n"
                     b"right tie rod outer,3,560,5\nrear lower ball joint,4,575,6\nFront Left UCA outboard,5,-540,300\n"
                     b"Rear Right LCA outboard,6,575,110\n", "pt.csv"),
    "ordinals": (b"Name,X,Y,Z\nlink one inboard,1,2,3\n2nd link outboard,2,3,4\nthird link inner,3,4,5\n"
                 b"fourth link outer,4,5,6\nfifth link chassis,5,6,7\nlink #3 upright,6,7,8\nfirst link outer,7,8,9\n"
                 b"link_6 inboard,8,9,1\nsecond link inboard,9,1,2\nrod 3 outer,1,1,1\nstrut-4-top,2,2,2\n"
                 b"link iii inner,3,3,3\nl4 outboard,4,4,4\narm2 inner,5,5,5\n", "ord.csv"),
    "csv_error": (b"Name,X,Y,Z\n" + b"a" * 140000 + b",1,2,3\n", "big.csv"),
    "xlsx_blank_name": (_xlsx(b"Name,X,Y,Z\nUpper ball joint,1,2,3"), ""),
    "sheets": (_xlsx_sheets(), "s.xlsx"),
    "layouts": (b"Vertical stop,1,2,3\nLongitudinal link,4,5,6\nUpper ball joint,1,2,3,note\n,Lower ball joint,4,5,6\n"
                b"Upper front inner left,1,2,3\nlink 3 middle,1,2,3\nY,X,Z,,Part\n1,2,3,,Wheel center\nShort,1\n"
                b"4,5,6,,Contact patch\n", "lay.csv"),
    "dup_z": (b"Name,X,Y,Z,Vertical (mm)\nUpper ball joint,1,2,3,99\n", "dz.csv"),
    "fused_across": (b"Name,X,Y,Z\nxtie rod inner,1,2,3\nxtoe link outer,1,2,3\nxwheel center,1,2,3\n"
                     b"xwheel centre,1,2,3\nxbell crankx pivot,1,2,3\nxpush rod outer,1,2,3\nxpull rod rocker,1,2,3\n"
                     b"xcontact patchx,1,2,3\n", "fa.csv"),
    "inf_cells": (b"Name,X,Y,Z\nUpper ball joint,inf,2,3\nLower ball joint,1,nan,3\nWheel center,1,2,true\n", "i.csv"),
})


FILES = {"optimumk": (OPTIMUMK_CSV, "car.csv"), "four": (FOUR, "four.csv"), "macpherson": (MACPHERSON, "m.csv"),
         "multi": (MULTI, "multi.txt"), "trailing": (TRAILING, "t.csv"), "messy": (MESSY, "messy.csv"),
         "headerless": (HEADERLESS, "h.csv"), "xlsx": (_xlsx(OPTIMUMK_CSV), "car.xlsx"),
         "xlsx2": (_xlsx(FOUR, True), "four.xlsx"), "empty": (b"", "e.csv"), "garbage": (b"\x00\xff\xfe junk", "g.xlsx"),
         "text": (b"just some words\nno numbers here\n", "t.csv"), **EXTRA}


def _res(r):
    return {"r": plain(r), "summary": r.summary() if hasattr(r, "summary") else None}


def _everything():
    for name, (data, fname) in FILES.items():
        try:
            pts, hint = hi.parse_tabular(data, fname)
        except ValueError as e:
            yield f"{name}.parse", {"raised": "ValueError", "message": str(e)}
            continue
        yield f"{name}.parse", [plain(pts), hint]
        groups = hi.group_corners(pts)
        yield f"{name}.groups", plain(groups)
        yield f"{name}.map", plain(hi.map_names(pts))
        yield f"{name}.units", plain(hi.infer_units(pts, hint))
        yield f"{name}.units_m", plain(hi.infer_units(pts, "m"))
        for corner in sorted(groups) or [""]:
            cp = hi.points_for_corner(groups, corner)
            yield f"{name}.{corner}.points", plain(cp)
            yield f"{name}.{corner}.default", _res(hi.build_result(cp, header_hint=hint, corner=corner))
        yield f"{name}.all.default", _res(hi.build_result(pts, header_hint=hint))
    pts, hint = hi.parse_tabular(OPTIMUMK_CSV, "car.csv")
    for frame in ("iso8855", "sae_j670", "iso4130", "kinematik", "sw_front"):
        for mirror in (None, True, False):
            for reorigin in (True, False):
                yield f"frame.{frame}.{mirror}.{reorigin}", _res(hi.build_result(
                    pts, frame_key=frame, header_hint=hint, mirror=mirror, reorigin=reorigin, corner="FL"))
    for unit in ("mm", "m", "in", None):
        for topo in ("auto", "double_wishbone", "macpherson", "multilink", "trailing_arm", "twist_beam"):
            yield f"unit.{unit}.{topo}", _res(hi.build_result(pts, unit=unit, topology=topo))
    for scale in (1.0, 0.001, 0.0393701, 10.0, 1e-6):
        sp = [hi.RawPoint(p.name, p.x * scale, p.y * scale, p.z * scale) if hasattr(p, "x") else p for p in pts]
        yield f"magnitude.{scale}", plain(hi.infer_units(sp))
    nowc, h2 = hi.parse_tabular(MACPHERSON[:MACPHERSON.index(b"Wheel centre")], "m.csv")
    for reorigin in (True, False):
        yield f"nowc.{reorigin}", _res(hi.build_result(nowc, header_hint=h2, reorigin=reorigin))
    yield "topo.default", _res(hi.build_result(pts, header_hint=hint))
    yield "topo.unknown_missing", _res(hi.build_result(nowc, header_hint=h2, topology="nope"))
    for big in (60.0, 60.5, 3.0, 2.99):
        yield f"edge.{big}", plain(hi.infer_units([hi.RawPoint("a", big, 0.0, 0.0)]))
    yield "topo.unknown", _res(hi.build_result(pts, header_hint=hint, topology="nope"))
    for n in (1, 2, 3, 4):
        yield f"few.{n}", [plain(hi.infer_units(pts[:n])), plain(hi.infer_units(pts[-n:], "in"))]
    try:                                  # no filename reads as CSV, whatever the bytes are
        nofile = plain(hi.parse_tabular(_xlsx(b"Name,X,Y,Z\nUpper ball joint,1,2,3")))
    except ValueError as e:
        nofile = str(e)
    yield "parse.nofilename", nofile
    yield "corners.none", plain(hi.points_for_corner({}, "FL"))
    yield "consts", [hi.MULTILINK_MAX, plain(hi._TOPOLOGY_LABEL)]


def test_every_file_through_the_import():
    check_lazy("hardpoint_import", _everything())
