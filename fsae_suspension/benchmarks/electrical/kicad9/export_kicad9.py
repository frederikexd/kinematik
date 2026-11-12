"""KinematiK benchmark kit: export KiCad 8/9's own reading of a board (counts, routed lengths, copper shapes).

Run in KiCad 8 or 9: open the PCB Editor, Tools > Scripting Console, then type
    exec(open(r"C:\\path\\to\\export_kicad9.py").read())
It exports every .kicad_pcb in the "boards" folder next to this script, and writes one .json per board there.
Send back the .json files. Nothing on your PC is changed."""
import json, os, glob, pcbnew

HERE = os.path.dirname(os.path.abspath(__file__)) if "__file__" in globals() else os.getcwd()
nm = 1e-6


def poly_points(ps):
    out = []
    try:
        ps = pcbnew.SHAPE_POLY_SET(ps)
    except Exception:
        pass
    for frac in ("Fracture",):
        try:
            getattr(ps, frac)()
        except TypeError:
            try:
                getattr(ps, frac)(pcbnew.SHAPE_POLY_SET.PM_FAST)
            except Exception:
                pass
        except Exception:
            pass
    for k in range(ps.OutlineCount()):
        o = ps.Outline(k)
        out.append([[o.CPoint(j).x * nm, o.CPoint(j).y * nm] for j in range(o.PointCount())])
    return out


def pad_polys(p, ly):
    ps = pcbnew.SHAPE_POLY_SET()
    loc = getattr(pcbnew, "ERROR_INSIDE", None)
    try:
        if loc is None:
            raise TypeError
        p.TransformShapeToPolygon(ps, ly, 0, 500, loc)
    except TypeError:
        p.TransformShapeToPolygon(ps, ly, 0, 500)
    return poly_points(ps)


def via_width(t, ly=None):
    """KiCad 9 vias have a padstack: ask for the width on a layer (KiCad 7-8 take no argument)."""
    try:
        return t.GetWidth(ly if ly is not None else pcbnew.F_Cu)
    except TypeError:
        return t.GetWidth()


def owner_ref(p):
    """The footprint reference of a pad (KiCad 9 returns a container from GetParent)."""
    fp = p.GetParentFootprint() if hasattr(p, "GetParentFootprint") else p.GetParent()
    return fp.GetReference()


def export(path):
    b = pcbnew.LoadBoard(path)
    stack = list(b.GetEnabledLayers().CuStack())
    names = {ly: b.GetLayerName(ly) for ly in stack}
    tracks = list(b.GetTracks())
    d = {"kicad_version": pcbnew.Version(), "file": os.path.basename(path),
         "copper_layers": [names[ly] for ly in stack],
         "counts": {"straight_tracks": sum(1 for t in tracks if t.Type() == pcbnew.PCB_TRACE_T),
                    "arc_tracks": sum(1 for t in tracks if t.Type() == pcbnew.PCB_ARC_T),
                    "vias": sum(1 for t in tracks if t.Type() == pcbnew.PCB_VIA_T),
                    "pads": len(b.GetPads()), "footprints": len(b.GetFootprints())},
         "nets": {}}
    for t in tracks:
        n = t.GetNetname()
        e = d["nets"].setdefault(n, {"length_mm": 0.0, "tracks": [], "vias": [], "pads": [], "zones": []})
        if t.Type() == pcbnew.PCB_VIA_T:
            e["vias"].append({"at": [t.GetPosition().x * nm, t.GetPosition().y * nm], "size": via_width(t) * nm,
                              "drill": t.GetDrillValue() * nm,
                              "layers": [names[ly] for ly in stack if t.IsOnLayer(ly)]})
            continue
        e["length_mm"] += t.GetLength() * nm
        tr = {"layer": names.get(t.GetLayer(), b.GetLayerName(t.GetLayer())), "width": t.GetWidth() * nm,
              "start": [t.GetStart().x * nm, t.GetStart().y * nm], "end": [t.GetEnd().x * nm, t.GetEnd().y * nm]}
        if t.Type() == pcbnew.PCB_ARC_T:
            tr["mid"] = [t.GetMid().x * nm, t.GetMid().y * nm]
        e["tracks"].append(tr)
    for p in b.GetPads():
        n = p.GetNetname()
        e = d["nets"].setdefault(n, {"length_mm": 0.0, "tracks": [], "vias": [], "pads": [], "zones": []})
        e["pads"].append({"name": f"{owner_ref(p)}.{p.GetNumber()}",
                          "at": [p.GetPosition().x * nm, p.GetPosition().y * nm],
                          "polys": {names[ly]: pad_polys(p, ly) for ly in stack if p.IsOnLayer(ly)}})
    for z in b.Zones():
        if z.GetIsRuleArea():
            continue
        n = z.GetNetname()
        e = d["nets"].setdefault(n, {"length_mm": 0.0, "tracks": [], "vias": [], "pads": [], "zones": []})
        for ly in stack:
            if z.IsOnLayer(ly):
                e["zones"].append({"layer": names[ly], "spoke": z.GetThermalReliefSpokeWidth() * nm,
                                   "polys": poly_points(z.GetFilledPolysList(ly))})
    out = os.path.splitext(path)[0] + ".kicad9.json"
    with open(out, "w") as f:
        json.dump(d, f)
    print("wrote", out, d["counts"])


for f in sorted(glob.glob(os.path.join(HERE, "boards", "*.kicad_pcb"))):
    try:
        export(f)
    except Exception as ex:
        print("FAILED", f, ex)
