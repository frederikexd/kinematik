# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Addendum 16's reference: pad-to-pad gaps on different nets from KiCad 7's own pad polygons (pcbnew).

    python3 scripts/bridge_gap_reference.py <board.kicad_pcb> ...
    python3 scripts/bridge_gap_reference.py --out kicad7_altium_pad_gaps.json <label>=<board.PcbDoc> ...

A .PcbDoc is imported with KiCad's own Altium importer (addendum 17).

Writes benchmarks/electrical/bridge_gaps/kicad7_pad_gaps.json: for each board, every different-net pad pair on a
shared copper side closer than 0.5 mm, measured edge to edge on KiCad's polygons (0.001 mm error).
"""
import json
import math
import sys
from pathlib import Path

import pcbnew

OUT = Path(__file__).resolve().parents[1] / "benchmarks" / "electrical" / "bridge_gaps" / "kicad7_pad_gaps.json"
REACH_MM = 0.5


def _seg(p, q, r, s):
    def pt(a, b, c):
        dx, dy = c[0] - b[0], c[1] - b[1]
        L = dx * dx + dy * dy
        t = 0.0 if L == 0 else max(0.0, min(1.0, ((a[0] - b[0]) * dx + (a[1] - b[1]) * dy) / L))
        return math.hypot(a[0] - b[0] - t * dx, a[1] - b[1] - t * dy)
    return min(pt(p, r, s), pt(q, r, s), pt(r, p, q), pt(s, p, q))


def _poly(pad, layer):
    ps = pcbnew.SHAPE_POLY_SET()
    pad.TransformShapeToPolygon(ps, layer, 0, pcbnew.FromMM(0.001))
    out = []
    for i in range(ps.OutlineCount()):
        ch = ps.Outline(i)
        out += [(pcbnew.ToMM(ch.CPoint(k).x), pcbnew.ToMM(ch.CPoint(k).y)) for k in range(ch.PointCount())]
    return ps, out


def board_gaps(path):
    if str(path).lower().endswith(".pcbdoc"):
        b = pcbnew.IO_MGR.Load(pcbnew.IO_MGR.ALTIUM_DESIGNER, str(path))
    else:
        b = pcbnew.LoadBoard(path)
    rows = []
    pads = []
    for fp in b.GetFootprints():
        for p in fp.Pads():
            name = p.GetNetname()
            if not p.GetNetCode() or not name or name.lower().startswith("unconnected"):
                continue
            for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
                if p.IsOnLayer(layer):
                    ps, pts = _poly(p, layer)
                    if pts:
                        pads.append((f"{fp.GetReference()}.{p.GetNumber()}", p.GetNetCode(), name, layer, ps, pts))
    for i, a in enumerate(pads):
        for c in pads[i + 1:]:
            if a[1] == c[1] or a[3] != c[3] or a[0] == c[0]:
                continue
            ax = [q[0] for q in a[5]]; ay = [q[1] for q in a[5]]
            cx = [q[0] for q in c[5]]; cy = [q[1] for q in c[5]]
            if min(cx) > max(ax) + REACH_MM or max(cx) < min(ax) - REACH_MM or min(cy) > max(ay) + REACH_MM \
                    or max(cy) < min(ay) - REACH_MM:
                continue
            if a[4].Collide(pcbnew.VECTOR2I(pcbnew.FromMM(c[5][0][0]), pcbnew.FromMM(c[5][0][1]))) or \
                    c[4].Collide(pcbnew.VECTOR2I(pcbnew.FromMM(a[5][0][0]), pcbnew.FromMM(a[5][0][1]))):
                gap = 0.0
            else:
                ea, ec = list(zip(a[5], a[5][1:] + a[5][:1])), list(zip(c[5], c[5][1:] + c[5][:1]))
                gap = min(_seg(p, q, r, s) for p, q in ea for r, s in ec)
            if gap < REACH_MM:
                rows.append({"pins": sorted((a[0], c[0])), "gap_mm": round(gap, 5),
                             "side": "F.Cu" if a[3] == pcbnew.F_Cu else "B.Cu"})
    return rows


if __name__ == "__main__":  # pragma: no cover
    args = sys.argv[1:]
    dest = OUT
    if args[:1] == ["--out"]:
        dest, args = OUT.with_name(args[1]), args[2:]
    out = {"tool": f"KiCad {pcbnew.Version()} pcbnew pad polygons", "boards": {}}
    for a in args:
        label, _, f = a.rpartition("=") if "=" in a else (Path(a).stem, "", a)
        out["boards"][label] = board_gaps(f)
        print(label, len(out["boards"][label]))
    dest.write_text(json.dumps(out, indent=0), encoding="utf-8")
