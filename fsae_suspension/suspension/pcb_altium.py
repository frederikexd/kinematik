# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""
Altium front-end for PCB Doctor — read a real Altium board, diagnose it with
exactly the same physics as a KiCad one, and mend it in place.

Half the FSAE grid routes in Altium (the student licence is free) and the other
half in KiCad, and the failure modes the Doctor exists for — the via that
chokes a wide trace, the brown-out nobody could reproduce on the bench, the
electrolytic parked on hot copper — do not care which tool drew the copper.
This module is a *front-end only*: it produces the same `PcbBoard` the KiCad
reader produces, so every check, the viewer, the fix engine and the report are
written once and run identically on both.

What that costs, and what it buys:

  * **Layer names are normalised on import.** `TOP` → `F.Cu`, `MID3` →
    `In3.Cu`, `BOTTOM` → `B.Cu`. The physics needs to know *outer vs inner*
    (buried copper cools about half as well), and it should not have to learn
    two vocabularies to find out. The file's own names are kept in
    `board.native_layers` so the UI can still say `MID3` to an Altium user.
  * **The Y axis is flipped.** Altium's Y points up, KiCad's points down. Every
    distance, length and clearance is invariant under the flip, so only the
    viewer would have cared — and it would have drawn every Altium board
    upside down. Coordinates are never written back, so the flip cannot leak
    into a patched file.
  * **Units are preserved per token.** Altium ASCII writes lengths with their
    unit attached (`WIDTH=11.811mil`). The patcher puts the corrected width
    back in that same unit, so mending a mil-based board does not silently
    convert it to millimetres.
  * **Arcs are chorded, not dropped.** Altium routes arcs constantly. An
    ignored arc is a *hole in the connectivity graph*, and the open-copper
    check would then report a perfectly good net as dead on arrival — the one
    kind of false alarm that would teach a team to stop reading the findings.

Honesty rules, kept:

  * **Net indices are resolved 0-based** against `|RECORD=Net|` file order,
    which is what Altium's ASCII writer emits. An off-by-one here would mean a
    diagnosis run on the wrong currents, so the import records the assumption
    in `board.notes`, the UI prints it, and the per-net current table shows the
    resolved names — a mis-resolve is visible in one glance, never silent.
  * **Non-copper layers are skipped, not guessed at.** Silkscreen, keep-out and
    mechanical lines are not copper and never enter the ampacity mesh.
  * **The native binary `.PcbDoc` is read by `pcb_altium_binary`, not here.**
    That module diagnoses it but refuses to patch it, so `ALTIUM_BINARY_HELP`
    below is no longer a refusal — it is the path to the *re-trace*, and to a
    readable file when the binary reader cannot make sense of one.
"""

from __future__ import annotations

import math
import re

from .pcb_doctor import (PcbBoard, PcbSegment, PcbVia, PcbPad, PcbFootprint,
                         PcbZone,
                         DEFAULT_BOARD_THICKNESS_MM,
                         _arc_points_center, _chord_segments)

# --------------------------------------------------------------------------- #
#  Units
# --------------------------------------------------------------------------- #
#  Altium's internal unit is 1/10000 mil; its ASCII writer emits human units
#  with the suffix attached. Bare numbers are mils unless the file is clearly
#  metric, which `_default_unit` decides by looking at what the file actually
#  wrote rather than by assuming.
UNIT_MM = {"mil": 0.0254, "thou": 0.0254, "mm": 1.0, "cm": 10.0,
           "um": 0.001, "in": 25.4, "inch": 25.4, '"': 25.4}

_LEN_RE = re.compile(
    r"""^\s*([-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)\s*"""
    r"""(mil|thou|mm|cm|um|inch|in|")?\s*$""", re.I)

_MIL_HINT = re.compile(r"=\s*[-+]?[\d.]+\s*mil\b", re.I)
_MM_HINT = re.compile(r"=\s*[-+]?[\d.]+\s*mm\b", re.I)


def _default_unit(text: str) -> str:
    """Which unit bare numbers are in, decided from what the file wrote."""
    mils = len(_MIL_HINT.findall(text))
    mms = len(_MM_HINT.findall(text))
    return "mm" if mms > mils else "mil"


def _len_mm(raw, default_unit: str = "mil"):
    """`'11.811mil'` → 0.3 (mm). None if it isn't a length."""
    if raw is None:
        return None
    m = _LEN_RE.match(str(raw))
    if not m:
        return None
    unit = (m.group(2) or default_unit).lower()
    return float(m.group(1)) * UNIT_MM.get(unit, 0.0254)


def _len_unit(raw, default_unit: str = "mil"):
    """`'11.811mil'` → (0.0254, 'mil') — the scale and suffix to write back."""
    m = _LEN_RE.match(str(raw or ""))
    unit = (m.group(2) if m and m.group(2) else "").lower()
    if not unit:
        return UNIT_MM.get(default_unit, 0.0254), ""
    return UNIT_MM.get(unit, 0.0254), unit


# --------------------------------------------------------------------------- #
#  Layers
# --------------------------------------------------------------------------- #
_MID_RE = re.compile(r"^(?:MID|MIDLAYER|SIGNAL)(\d+)$")
_PLANE_RE = re.compile(r"^(?:INTERNALPLANE|PLANE|POWER|GND)(\d+)$")

_TOP_NAMES = {"TOP", "TOPLAYER", "TOPCOPPER", "COMPONENTSIDE", "L1"}
_BOT_NAMES = {"BOTTOM", "BOTTOMLAYER", "BOTTOMCOPPER", "SOLDERSIDE"}
MULTILAYER = "MULTI"


def altium_layer_to_kicad(name):
    """Canonical KiCad copper name, `MULTI` for through-everything, or None if
    the layer is not copper at all (silkscreen, mask, keep-out, mechanical).
    Returning None is what keeps a silkscreen outline out of the ampacity mesh.
    """
    n = re.sub(r"[\s_\-]", "", str(name or "")).upper()
    if not n:
        return None
    if n in _TOP_NAMES:
        return "F.Cu"
    if n in _BOT_NAMES:
        return "B.Cu"
    if n in ("MULTILAYER", "MULTI"):
        return MULTILAYER
    m = _MID_RE.match(n)
    if m:
        return f"In{int(m.group(1))}.Cu"
    m = _PLANE_RE.match(n)
    if m:
        # An internal plane is copper and it is an inner layer; the plane's own
        # net is registered as a pour so the net is never called "open".
        return f"In{int(m.group(1))}.Cu"
    return None


def _is_plane(name) -> bool:
    return bool(_PLANE_RE.match(re.sub(r"[\s_\-]", "", str(name or "")).upper()))


def _order_layers(found: set) -> list:
    """F.Cu, then inner layers in numeric order, then B.Cu — the stackup order
    a person expects, not set order."""
    inner = sorted((l for l in found if l.startswith("In")),
                   key=lambda l: int(re.sub(r"\D", "", l) or 0))
    out = (["F.Cu"] if "F.Cu" in found else []) + inner
    if "B.Cu" in found:
        out.append("B.Cu")
    return out or ["F.Cu", "B.Cu"]


# --------------------------------------------------------------------------- #
#  Record reader (span-preserving, like the KiCad side)
# --------------------------------------------------------------------------- #
_RECORD_RE = re.compile(r"^\s*\|RECORD=", re.I)


def _fields(line: str, base: int) -> dict:
    """`|RECORD=Track|WIDTH=10mil|` → {'WIDTH': ('10mil', start, end), …} with
    absolute character spans, so a width can later be rewritten in place."""
    out, pos = {}, 0
    for part in line.split("|"):
        start = base + pos
        pos += len(part) + 1
        if "=" not in part:
            continue
        key, _, val = part.partition("=")
        vstart = start + len(key) + 1
        out[key.strip().upper()] = (val, vstart, vstart + len(val))
    return out


def _g(f: dict, *keys):
    for k in keys:
        if k in f:
            return f[k][0]
    return None


def _span(f: dict, *keys):
    for k in keys:
        if k in f:
            return (f[k][1], f[k][2])
    return None


def outline_from_fields(f: dict, unit: str) -> list:
    """Pull a pour outline out of an Altium record's `VX0/VY0 … VXn/VYn` keys.

    Y is negated to match the rest of the import. Arc-bulged edges (`CX*/CY*`
    centres with `EA*` angles) are read as straight chords: a pour outline is
    used only to answer "is this pad inside", and a chord moves that boundary
    by microns on a shape measured in millimetres.
    """
    pts, i = [], 0
    while True:
        vx = _g(f, f"VX{i}")
        vy = _g(f, f"VY{i}")
        if vx is None or vy is None:
            break
        x, y = _len_mm(vx, unit), _len_mm(vy, unit)
        if x is None or y is None:
            break
        pts.append((x, -y))
        i += 1
    return pts


def _truthy(raw) -> bool:
    return str(raw or "").strip().upper() in ("TRUE", "1", "YES")


def _int(raw, default=None):
    try:
        return int(float(str(raw).strip()))
    except (TypeError, ValueError):
        return default


def is_altium_ascii(text: str) -> bool:
    """True if this text looks like an Altium/Protel ASCII PCB."""
    head = text[:20000]
    return bool(re.search(r"\|RECORD=", head, re.I))


# --------------------------------------------------------------------------- #
#  The parser
# --------------------------------------------------------------------------- #
def parse_altium_ascii(text: str) -> PcbBoard:
    """Parse an Altium / Protel **ASCII** PCB into the same `PcbBoard` the
    KiCad reader returns. Tolerant by construction: an unknown record or a
    malformed field is skipped, never fatal — a board the Doctor can read 95%
    of is worth far more than an exception."""
    if not is_altium_ascii(text):
        raise ValueError(
            "no |RECORD= entries found — this is not an Altium/Protel ASCII "
            "PCB. If it is a native (binary) .PcbDoc, re-save it as ASCII; "
            "see the export note in the panel.")

    unit = _default_unit(text)
    board = PcbBoard(text=text, fmt="altium", length_unit=unit)

    # --- pass 1: index the records, keeping file order for Net/Component ----- #
    records = []          # (kind, fields)
    pos = 0
    for line in text.splitlines(keepends=True):
        if _RECORD_RE.match(line):
            f = _fields(line.rstrip("\r\n"), pos)
            kind = re.sub(r"\W", "", (_g(f, "RECORD") or "")).upper()
            records.append((kind, f))
        pos += len(line)

    # --- nets: file order is the index space every other record references --- #
    #  Net id 0 is reserved for "no net" to match the KiCad convention the rest
    #  of the Doctor assumes, so Altium index i becomes id i+1.
    #  Every other record points at a net by *index*. Two conventions exist in
    #  the wild and the difference is silent: some exports number nets from 0,
    #  others from 1. Guessing wrong shifts every net by one — the diagnosis
    #  then runs on the wrong currents and names the wrong nets, while looking
    #  entirely plausible. Real boards showed both: one file's Net records
    #  started at ID=0, another's at ID=1.
    #
    #  So the `ID` field on the Net record is authoritative when present, and
    #  file order is only the fallback for exports that omit it.
    net_order, ids = [], []
    for kind, f in records:
        if kind == "NET":
            net_order.append((_g(f, "NAME", "NETNAME") or "").strip())
            ids.append(_int(_g(f, "ID")))
    have_ids = bool(ids) and all(i is not None for i in ids) and \
        len(set(ids)) == len(ids)
    if have_ids:
        index_of = dict(zip(ids, range(len(ids))))
        board.notes.append(
            f"Net references resolved against the ID field on each "
            f"|RECORD=Net| (this export numbers nets from {min(ids)}).")
    else:
        index_of = {i: i for i in range(len(net_order))}
        board.notes.append(
            "This export's Net records carry no usable ID, so net references "
            "were resolved 0-based against file order — check a familiar net "
            "name in the current table; if the names look shifted, do not "
            "trust the diagnosis.")
    board.nets = {0: ""}
    for i, nm in enumerate(net_order):
        board.nets[i + 1] = nm
    by_name = {nm: i + 1 for i, nm in enumerate(net_order) if nm}

    def net_id(raw):
        """Resolve a NET= field: a net ID, or a literal name."""
        if raw is None:
            return 0
        s = str(raw).strip()
        if not s:
            return 0
        n = _int(s)
        if n is not None:
            i = index_of.get(n)
            return (i + 1) if i is not None else 0     # unknown = unconnected
        return by_name.get(s, 0)

    # --- components: file order is the index pads reference ------------------ #
    comps = []
    for kind, f in records:
        if kind != "COMPONENT":
            continue
        x = _len_mm(_g(f, "X", "LOCATION.X"), unit) or 0.0
        y = _len_mm(_g(f, "Y", "LOCATION.Y"), unit) or 0.0
        layer = altium_layer_to_kicad(_g(f, "LAYER"))
        comps.append(PcbFootprint(
            ref=(_g(f, "SOURCEDESIGNATOR", "DESIGNATOR", "NAME") or "?").strip(),
            value=(_g(f, "COMMENT", "SOURCECOMMENT", "VALUE",
                      "SOURCEDESCRIPTION") or "").strip(),
            layer=layer if layer in ("F.Cu", "B.Cu") else "F.Cu",
            at=(x, -y)))
    # Free pads — Altium allows pads that belong to no component (castellations,
    # antenna feeds, test points), and a real board is full of them. They are
    # real copper and real net endpoints, so they must reach the connectivity
    # graph; each carries its own layer, which is why PcbPad.layer exists.
    loose = PcbFootprint(ref="?", value="free pads (no component)", layer="F.Cu",
                         at=(0.0, 0.0))

    found_layers, plane_nets = set(), set()
    n_region = 0

    # --- pass 2: geometry ---------------------------------------------------- #
    for kind, f in records:
        if kind in ("TRACK", "LINE"):
            layer = altium_layer_to_kicad(_g(f, "LAYER"))
            if layer is None or layer == MULTILAYER:
                continue                  # not copper — silkscreen, mech, mask
            x1 = _len_mm(_g(f, "X1"), unit); y1 = _len_mm(_g(f, "Y1"), unit)
            x2 = _len_mm(_g(f, "X2"), unit); y2 = _len_mm(_g(f, "Y2"), unit)
            wraw = _g(f, "WIDTH")
            w = _len_mm(wraw, unit)
            if None in (x1, y1, x2, y2) or not w:
                continue
            scale, suffix = _len_unit(wraw, unit)
            found_layers.add(layer)
            board.segments.append(PcbSegment(
                net=net_id(_g(f, "NET")), layer=layer, width_mm=w,
                start=(x1, -y1), end=(x2, -y2),
                width_span=_span(f, "WIDTH") or (0, 0),
                width_scale_mm=scale, width_unit=suffix))

        elif kind == "ARC":
            layer = altium_layer_to_kicad(_g(f, "LAYER"))
            if layer is None or layer == MULTILAYER:
                continue
            cx = _len_mm(_g(f, "X", "LOCATION.X"), unit)
            cy = _len_mm(_g(f, "Y", "LOCATION.Y"), unit)
            r = _len_mm(_g(f, "RADIUS"), unit)
            wraw = _g(f, "WIDTH")
            w = _len_mm(wraw, unit)
            if None in (cx, cy, r) or not w or r <= 0:
                continue
            try:
                a0 = float(_g(f, "STARTANGLE") or 0.0)
                a1 = float(_g(f, "ENDANGLE") or 360.0)
            except ValueError:
                continue
            pts = _arc_points_center(cx, cy, r, a0, a1)
            scale, suffix = _len_unit(wraw, unit)
            found_layers.add(layer)
            board.segments.extend(_chord_segments(
                [(px, -py) for px, py in pts],
                net=net_id(_g(f, "NET")), layer=layer, width_mm=w,
                width_span=_span(f, "WIDTH") or (0, 0),
                width_scale_mm=scale, width_unit=suffix))

        elif kind == "VIA":
            x = _len_mm(_g(f, "X", "LOCATION.X"), unit)
            y = _len_mm(_g(f, "Y", "LOCATION.Y"), unit)
            if None in (x, y):
                continue
            dia = _len_mm(_g(f, "DIAMETER", "VIADIAMETER", "SIZE"), unit) or 0.6
            hole = _len_mm(_g(f, "HOLESIZE", "HOLE", "DRILL"), unit) or 0.3
            top = altium_layer_to_kicad(_g(f, "STARTLAYER")) or "F.Cu"
            bot = altium_layer_to_kicad(_g(f, "ENDLAYER")) or "B.Cu"
            if top == MULTILAYER:
                top = "F.Cu"
            if bot == MULTILAYER:
                bot = "B.Cu"
            found_layers.update({top, bot})
            board.vias.append(PcbVia(net=net_id(_g(f, "NET")), at=(x, -y),
                                     size_mm=dia, drill_mm=hole,
                                     layers=(top, bot)))

        elif kind == "PAD":
            x = _len_mm(_g(f, "X", "LOCATION.X"), unit)
            y = _len_mm(_g(f, "Y", "LOCATION.Y"), unit)
            if None in (x, y):
                continue
            raw_layer = _g(f, "LAYER")
            layer = altium_layer_to_kicad(raw_layer)
            hole = _len_mm(_g(f, "HOLESIZE", "HOLE", "DRILL"), unit) or 0.0
            through = (layer == MULTILAYER) or hole > 0.0
            sx = _len_mm(_g(f, "XSIZE", "TOPXSIZE", "SIZEX"), unit) or 1.0
            sy = _len_mm(_g(f, "YSIZE", "TOPYSIZE", "SIZEY"), unit) or 1.0
            nid = net_id(_g(f, "NET"))
            pad = PcbPad(number=(_g(f, "NAME", "DESIGNATOR") or "?").strip(),
                         net=nid, net_name=board.nets.get(nid, ""),
                         at=(x, -y), size=(sx, sy), through=through,
                         layer=(layer if layer in ("F.Cu", "B.Cu")
                                else ("" if through else "F.Cu")))
            ci = _int(_g(f, "COMPONENT"), -1)
            (comps[ci] if (ci is not None and 0 <= ci < len(comps))
             else loose).pads.append(pad)
            if layer in ("F.Cu", "B.Cu"):
                found_layers.add(layer)

        elif kind in ("POLYGON", "PLANE", "SPLITPLANE"):
            # Copper the Doctor deliberately does not mesh. Registering the net
            # stops the open-copper check flagging a net that is in fact joined
            # by a pour, and keeps its resistance labelled conservative rather
            # than wrong.
            #
            # Only *Polygon* and plane records count. `Region` and `Fill` are
            # deliberately excluded: on real boards the overwhelming majority of
            # Region records are teardrops and board cutouts carrying NET=0,
            # which read as net index 0 and would invent a pour on whichever net
            # happens to be first in the file. A phantom pour SUPPRESSES a real
            # copper-open finding — a false all-clear — whereas omitting a real
            # pour merely produces a false alarm. Given the ambiguity is not
            # resolvable from the file, the tool errs toward the alarm.
            nid = net_id(_g(f, "NET"))
            if nid:
                board.zone_nets.add(nid)
            layer = altium_layer_to_kicad(_g(f, "LAYER"))
            if layer and layer != MULTILAYER:
                found_layers.add(layer)
            if _is_plane(_g(f, "LAYER")) and nid:
                plane_nets.add(nid)
            if nid and layer and layer != MULTILAYER:
                _o = outline_from_fields(f, unit)
                if len(_o) >= 3:
                    board.zones.append(PcbZone(net=nid, layer=layer, outline=_o))

        elif kind in ("REGION", "FILL"):
            #  Regions were once excluded wholesale, because 1300 teardrops
            #  carrying NET=0 were resolving to net index 0 and inventing a pour
            #  on whichever net happened to be first — and a phantom pour hides
            #  a real copper open. That was a symptom of the net-indexing bug,
            #  not of regions: now that references resolve against the Net
            #  record's own ID, NET=0 on a file whose ids start at 1 correctly
            #  means "no net", and only regions genuinely attached to a net
            #  register. Keepouts, board cutouts and teardrops are still
            #  excluded by name — none of them is a pour, whatever net they
            #  claim.
            layer = altium_layer_to_kicad(_g(f, "LAYER"))
            if layer and layer != MULTILAYER:
                found_layers.add(layer)
            n_region += 1
            if _truthy(_g(f, "KEEPOUT")) or _truthy(_g(f, "ISBOARDCUTOUT")) \
                    or _truthy(_g(f, "TEARDROP")):
                continue
            nid = net_id(_g(f, "NET"))
            if nid:
                board.zone_nets.add(nid)
                if layer and layer != MULTILAYER:
                    _o = outline_from_fields(f, unit)
                    if len(_o) >= 3:
                        board.zones.append(
                            PcbZone(net=nid, layer=layer, outline=_o))

        elif kind == "BOARD":
            th = _len_mm(_g(f, "BOARDTHICKNESS", "THICKNESS",
                            "LAYERSTACKTHICKNESS"), unit)
            if th and 0.1 < th < 10.0:
                board.board_thickness_mm = th

    board.footprints = [c for c in comps if c.pads or c.ref != "?"]
    if loose.pads:
        board.footprints.append(loose)
    board.zone_nets |= plane_nets

    # --- layer bookkeeping: canonical for the physics, native for the human -- #
    board.copper_layers = _order_layers(found_layers)
    board.native_layers = {"F.Cu": "Top Layer", "B.Cu": "Bottom Layer"}
    for l in board.copper_layers:
        if l.startswith("In") and l.endswith(".Cu"):
            board.native_layers[l] = f"Mid Layer {l[2:-3]}"
    if board.board_thickness_mm == DEFAULT_BOARD_THICKNESS_MM:
        board.notes.append(
            f"Board thickness not stated in the export — assumed "
            f"{DEFAULT_BOARD_THICKNESS_MM:g} mm, which only affects via-barrel "
            f"length. Set your real stackup thickness if it differs.")
    board.notes.append(
        f"Lengths without a unit suffix read as {unit}; widths are written "
        f"back in the unit the file used, so the patched board stays "
        f"{'metric' if unit == 'mm' else 'imperial'}.")
    if n_region:
        board.notes.append(
            f"{n_region} Region/Fill shapes read. Those attached to a net count "
            f"as pours (so the net is not falsely called open); keepouts, board "
            f"cutouts and teardrops are excluded. Pour *geometry* is still not "
            f"meshed, so resistance stays trace-only and conservative.")
    if any(s.from_arc for s in board.segments):
        board.notes.append(
            "Routed arcs were chorded into straight segments so connectivity "
            "and length are right; each arc still has one width token, "
            "patched once.")

    # A file whose lengths carry no unit at all could be in Altium's internal
    # unit (1/10000 mil) rather than mils. Reading those as mils inflates the
    # board 10,000× — and an inflated board reads as *all traces enormous*,
    # i.e. a silent ALL-CLEAR on a board that was never checked. That is the
    # one direction this tool must never fail in, so an implausible board is
    # refused outright rather than diagnosed.
    if board.segments or board.vias:
        x0, y0, x1, y1 = board.bbox()
        span = max(x1 - x0, y1 - y0)
        if span > 2000.0:                       # 2 m — no FSAE PCB is this big
            scaled = span * 1e-4
            raise ValueError(
                f"this board measures {span:,.0f} mm across, which is not a "
                f"PCB. Its lengths have no unit suffix, so they were read as "
                f"mils; they are most likely Altium *internal* units "
                f"(1/10000 mil), which would make the real board "
                f"{scaled:,.1f} mm. Re-export from Altium with File ▸ Save As "
                f"▸ 'PCB ASCII File' rather than converting by hand — the "
                f"Doctor refuses this file instead of reporting every trace "
                f"as comfortably oversized, which is what a 10,000× error "
                f"looks like from the inside.")
        if span < 1.0:
            board.notes.append(
                f"This board measures only {span:.2f} mm across — check the "
                f"units are right before trusting any width verdict.")
    return board


# --------------------------------------------------------------------------- #
#  Native binary .PcbDoc — detection, and the ASCII route to the re-trace
# --------------------------------------------------------------------------- #
OLE_MAGIC = b"\xd0\xcf\x11\xe0\xa1\xb1\x1a\xe1"

#  Shown when a binary board cannot be read at all, and when a readable one is
#  diagnosed but the member wants the automatic re-trace, which only the text
#  formats can have applied safely.
ALTIUM_BINARY_HELP = (
    "For the one-click **re-trace**, the Doctor needs the **ASCII** form of "
    "this board — a native (binary) `.PcbDoc` is diagnosed but never rewritten. "
    "Altium writes and reads the ASCII form losslessly:\n\n"
    "1. Open the board in Altium Designer.\n"
    "2. **File ▸ Save As…**, and in the *Save as type* dropdown pick the "
    "**PCB ASCII File (`*.PcbDoc`)** entry.\n"
    "3. Drop that file here.\n\n"
    "(Team-wide: *Preferences ▸ PCB Editor ▸ General ▸ Save PCB in ASCII "
    "format* makes it the default. Protel 99SE ASCII `.pcb` files work too.)\n\n"
    "Why the asymmetry: a bad *read* announces itself as absurd geometry and "
    "gets refused; a bad *write* silently corrupts a board you are about to pay "
    "to fabricate. So binary is read, and only text is patched."
)


def is_altium_binary(data) -> bool:
    """True for a native binary .PcbDoc (OLE compound document)."""
    if isinstance(data, str):
        data = data.encode("utf-8", errors="replace")[:8]
    return bytes(data)[:8] == OLE_MAGIC


# --------------------------------------------------------------------------- #
#  Demo board — the same ECU, the same three planted failures, in Altium ASCII
# --------------------------------------------------------------------------- #
#  Deliberately the *same geometry* as `demo_kicad_pcb()`, written in mils with
#  Altium's Y-up axis. That makes the demo a live cross-format proof: both files
#  describe one board, so both must produce the same diagnosis. The test suite
#  asserts exactly that.
_DEMO_Y0_MM = 60.0        # Altium Y origin, so the flip lands on the same board


def _mil(mm: float) -> str:
    return f"{mm / 0.0254:.4f}".rstrip("0").rstrip(".") + "mil"


def _xy(x_mm: float, y_mm: float, px="X", py="Y") -> str:
    return f"{px}={_mil(x_mm)}|{py}={_mil(_DEMO_Y0_MM - y_mm)}"


def demo_altium_pcb() -> str:
    """The demo ECU board as an Altium ASCII PCB: fan feed under-sized and
    via-choked, CAN pair hugging the 400 V inverter sense net, bulk cap on the
    hot copper."""
    nets = ["GND", "FAN_PWR", "CAN_H", "CAN_L", "HV_INV_SENSE", "LV_5V"]
    n = {name: i for i, name in enumerate(nets)}          # 0-based, as Altium
    comps = [  # ref, comment, x_mm, y_mm
        ("J1", "FanConn", 5, 10), ("U1", "VNH7070", 70, 10),
        ("C1", "470uF 16V", 40, 11.5), ("F1", "5A blade", 20, 10),
        ("U2", "STM32F4", 70, 40), ("J2", "CAN out", 5, 40)]
    ci = {ref: i for i, (ref, *_) in enumerate(comps)}
    pads = [  # comp, name, x_mm, y_mm, net, hole_mm
        ("J1", "1", 5, 10, "FAN_PWR", 1.0), ("J1", "2", 5, 12.54, "GND", 1.0),
        ("U1", "1", 70, 10, "FAN_PWR", 0.0), ("U1", "2", 70, 13, "LV_5V", 0.0),
        ("C1", "1", 40, 11.5, "FAN_PWR", 0.0), ("C1", "2", 40, 13.5, "GND", 0.0),
        ("F1", "1", 18, 10, "FAN_PWR", 0.0), ("F1", "2", 22, 10, "FAN_PWR", 0.0),
        ("U2", "1", 70, 40, "CAN_H", 0.0), ("U2", "2", 70, 41.5, "CAN_L", 0.0),
        ("J2", "1", 5, 40, "CAN_H", 1.0), ("J2", "2", 5, 41.5, "CAN_L", 1.0)]
    tracks = [  # layer, net, x1, y1, x2, y2, width_mm
        ("TOP", "FAN_PWR", 5, 10, 18, 10, 0.3),
        ("TOP", "FAN_PWR", 22, 10, 40, 10, 0.3),
        ("TOP", "FAN_PWR", 40, 10, 40, 11.5, 0.3),
        ("TOP", "FAN_PWR", 40, 10, 45, 10, 0.3),
        ("BOTTOM", "FAN_PWR", 45, 10, 70, 10, 0.3),
        ("TOP", "CAN_H", 5, 40, 70, 40, 0.2),
        ("TOP", "CAN_L", 5, 41.5, 55, 41.5, 0.2),
        ("TOP", "CAN_L", 55, 41.5, 60, 46, 0.2),
        ("TOP", "CAN_L", 60, 46, 68, 46, 0.2),
        ("TOP", "CAN_L", 68, 46, 70, 41.5, 0.2),
        ("TOP", "HV_INV_SENSE", 5, 39.2, 70, 39.2, 0.25),
        ("TOP", "LV_5V", 70, 10, 70, 25, 0.5)]
    vias = [("FAN_PWR", 45, 10), ("FAN_PWR", 70, 10)]

    L = ["|RECORD=Board|FILENAME=demo_ecu_board.PcbDoc|"
         f"BOARDTHICKNESS={_mil(1.6)}|KIND=0|"]
    L += [f"|RECORD=Net|ID={i}|NAME={nm}|" for i, nm in enumerate(nets)]
    for ref, comment, x, y in comps:
        L.append(f"|RECORD=Component|SOURCEDESIGNATOR={ref}|COMMENT={comment}|"
                 f"PATTERN={ref}_FP|LAYER=TOP|{_xy(x, y)}|ROTATION=0|")
    for ref, name, x, y, net, hole in pads:
        layer = "MULTILAYER" if hole else "TOP"
        L.append(f"|RECORD=Pad|NAME={name}|COMPONENT={ci[ref]}|LAYER={layer}|"
                 f"NET={n[net]}|{_xy(x, y)}|XSIZE={_mil(1.7 if hole else 1.5)}|"
                 f"YSIZE={_mil(1.7 if hole else 1.5)}|"
                 f"HOLESIZE={_mil(hole)}|SHAPE=ROUND|PLATED=TRUE|")
    for layer, net, x1, y1, x2, y2, w in tracks:
        L.append(f"|RECORD=Track|LAYER={layer}|NET={n[net]}|COMPONENT=-1|"
                 f"{_xy(x1, y1, 'X1', 'Y1')}|{_xy(x2, y2, 'X2', 'Y2')}|"
                 f"WIDTH={_mil(w)}|")
    for net, x, y in vias:
        L.append(f"|RECORD=Via|{_xy(x, y)}|DIAMETER={_mil(0.6)}|"
                 f"HOLESIZE={_mil(0.3)}|STARTLAYER=TOP|ENDLAYER=BOTTOM|"
                 f"NET={n[net]}|")
    # a silkscreen line and a mechanical outline: both must be ignored as copper
    L.append("|RECORD=Track|LAYER=TOPOVERLAY|NET=-1|X1=0mil|Y1=0mil|"
             "X2=3000mil|Y2=0mil|WIDTH=8mil|")
    L.append("|RECORD=Track|LAYER=MECHANICAL1|NET=-1|X1=0mil|Y1=0mil|"
             "X2=0mil|Y2=2400mil|WIDTH=4mil|")
    return "\n".join(L) + "\n"
