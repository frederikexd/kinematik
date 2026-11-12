# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""
PCB Doctor — import the *real* board file, find why it fails in real life,
name the component, prescribe the fix, and re-trace it in place.

The board-ledger in `electronics.py` checks the traces the team *declares*.
This module closes the other half of the loop the electrical members actually
live in: the board already exists as a routed file, it passed DRC, it "works
theoretically" — and then it browns out on track, a via desolders itself, or
the CAN bus drops frames next to the inverter. The schematic was never the
problem; the *copper* was.

**Both EDAs, one Doctor.** Half the grid routes in KiCad and half in Altium,
and none of these failure modes care which tool drew the copper. Two readers
feed one `PcbBoard` model — KiCad `.kicad_pcb` (v5–9) and Altium/Protel
**ASCII** PCB (`.PcbDoc` saved as ASCII, `.pcb`) — so every check, the viewer,
the fix engine and the report exist once and behave identically on both. Layer
names are normalised to the KiCad vocabulary on import (Altium `MID3` →
`In3.Cu`) because the physics only needs to know outer from inner, and the
file's own names are kept for display. `parse_board()` is the one door;
`sniff_format()` identifies a file from its content, not its extension.

The native *binary* `.PcbDoc` — what Altium saves by default — is **read** too,
by `pcb_altium_binary`, but never written: a binary board carries
`patchable = False` and `apply_fixes()` refuses it outright. A mis-parse on read
is recoverable (it shows up as absurd geometry, and the reader refuses the file
rather than reporting on it); a mistake on write corrupts a board a team is
about to pay to fabricate. The one-click re-trace therefore still needs the
ASCII export, and `pcb_altium.ALTIUM_BINARY_HELP` explains how to get it.

PCB Doctor does four things, all analytic, all dependency-free beyond numpy:

  1. **Parse the real board.** Span-preserving readers extract every copper
     segment (width, layer, net), every routed arc (chorded, so connectivity
     is never broken), every via (drill, size, layers), every footprint
     (reference, value, pads) and every pour — recording the exact character
     span of each width token, so the file can later be patched surgically
     instead of regenerated.

  2. **Diagnose real-life failure modes DRC never sees.** DRC checks geometry
     against *rules*; the Doctor checks copper against *physics* and against
     the car's own integration ledger (the declared peak currents):
       * per-net ampacity at the bottleneck segment (IPC-2221 heating),
       * Onderdonk fusing margin under the fuse safety factor,
       * via ampacity — the classic "1 mm trace choked by one 0.3 mm via",
       * true IR-drop → ECU brown-out, computed by **nodal analysis of the
         actual routed copper network** (segments + vias as a resistor mesh),
         not a guessed length,
       * copper opens — pads on the same net not joined by any trace/via
         (the rats-nest line everyone missed; the board is dead on arrival),
       * HV creepage/clearance vs IPC-2221 table B4 — the "worked on the
         bench, arced at 400 V in the rain" failure,
       * differential-pair skew & width asymmetry (CAN_H/CAN_L),
       * HV aggressor running parallel to a signal pair (coupled noise),
       * **component-level real-life derating**: the electrolytic cap parked
         on hot copper (lifetime halves every +10 °C), the fuse whose marked
         rating is below the net's real current, the connector pin asked to
         carry more than its family rating.
     Every finding names the net *and* the component(s) implicated, explains
     in plain language why it fails on the car even though it simulated fine,
     and carries a concrete numeric fix.

  3. **Fix the traces on the existing board.** For every under-sized power
     segment the Doctor computes the exact IPC-2221 width the assigned current
     needs and rewrites that segment's width token *in the original file* —
     KiCad's `(width …)`, Altium's `WIDTH=…`, in the file's own units — and
     nothing else in the file is touched, so it re-opens in the EDA it came
     from with the routing intact. Differential-pair members are **not**
     auto-widened (width sets impedance); they get a prescription instead.
     After patching, the HV clearance check re-runs on the patched geometry so
     a widened trace that now crowds a neighbour is reported, not hidden.

  4. **Prescribe traces before they exist.** The multi-layer Trace Prescriber
     answers the question that actually confuses people routing multi-layer
     boards — "how wide, on which layer, how many vias?" — as a single table:
     required width per copper weight per layer class for a given current,
     plus the via count for every layer transition.

Honesty rules the rest of KinematiK keeps, kept here:
  * Copper pours (zones) are not in the 1-D mesh. A net with a pour is never declared "open", and its voltage
    drop is checked through the pour by a 2-D sheet solve (pcb_pour) where the net is small enough; where it is
    not, the report says the drop through the pour was not checked. The 1-D figure on such a net covers only the
    pads its tracks join, which is NOT an upper bound for a pad fed through the pour.
  * Anything needing a field solver (eye diagrams, true coupled noise volts)
    is reported as *not computed*, never invented.
  * Currents come from the integration ledger where a name match exists; every
    auto-assignment is labelled and editable, and a check run on an assumed
    current says so in the finding.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, field

import numpy as np

from .interfaces import Finding, Severity
from .electronics import (OZ_TO_UM, RHO_CU_20C, ALPHA_CU, Trace, IPC_K,
                          min_parallel_distance_mm, parallel_run_length_mm)

MM2_TO_MIL2 = 39.3701 ** 2          # mm² → mil²
DEFAULT_VIA_PLATING_MM = 0.025      # ≈ 1 oz barrel plating, IPC class 2
DEFAULT_BOARD_THICKNESS_MM = 1.6


# --------------------------------------------------------------------------- #
#  S-expression reader (span-preserving, for surgical patching)
# --------------------------------------------------------------------------- #
@dataclass
class Atom:
    """One bare or quoted token with its character span in the source text."""
    value: str
    start: int
    end: int
    quoted: bool = False


def _tokenize(text: str):
    """Yield ('(', i) / (')', i) / Atom for a KiCad s-expression file."""
    i, n = 0, len(text)
    while i < n:
        c = text[i]
        if c in " \t\r\n":
            i += 1
        elif c == "(":
            yield ("(", i); i += 1
        elif c == ")":
            yield (")", i); i += 1
        elif c == '"':
            j = i + 1
            buf = []
            while j < n:
                if text[j] == "\\" and j + 1 < n:
                    buf.append(text[j + 1]); j += 2
                elif text[j] == '"':
                    break
                else:
                    buf.append(text[j]); j += 1
            yield Atom("".join(buf), i, j + 1, quoted=True)
            i = j + 1
        else:
            j = i
            while j < n and text[j] not in ' \t\r\n()"':
                j += 1
            yield Atom(text[i:j], i, j)
            i = j


def parse_sexpr(text: str):
    """Parse into nested lists. Each list's first element is usually the node
    name Atom; children are Atoms or sub-lists. Raises ValueError on imbalance."""
    stack = [[]]
    for tok in _tokenize(text):
        if isinstance(tok, Atom):
            stack[-1].append(tok)
        elif tok[0] == "(":
            new = []
            stack[-1].append(new)
            stack.append(new)
        else:  # ')'
            if len(stack) == 1:
                raise ValueError("unbalanced ')' in board file")
            stack.pop()
    if len(stack) != 1:
        raise ValueError("unbalanced '(' in board file")
    root = stack[0]
    return root[0] if len(root) == 1 else root


def _name(node) -> str:
    return node[0].value if node and isinstance(node[0], Atom) else ""


def _children(node, name: str):
    return [c for c in node[1:] if isinstance(c, list) and _name(c) == name]


def _child(node, name: str):
    cs = _children(node, name)
    return cs[0] if cs else None


def _floats(node, count=None):
    vals = []
    for c in node[1:]:
        if isinstance(c, Atom):
            try:
                v = float(c.value)
            except ValueError:
                continue
            if math.isfinite(v):        # "nan", "inf", "1e999": a name or a corrupt value, never a dimension
                vals.append(v)
    return vals if count is None else (vals + [0.0] * count)[:count]


def _positive(raw) -> float:
    """A width or size (mm) from the file: finite and above zero, else ValueError (the item is then not copper)."""
    v = float(raw)
    if not (math.isfinite(v) and v > 0):
        raise ValueError(f"not a usable dimension: {raw!r}")
    return v


def _atoms(node):
    return [c for c in node[1:] if isinstance(c, Atom)]


# --------------------------------------------------------------------------- #
#  Parsed board model
# --------------------------------------------------------------------------- #
@dataclass
class PcbSegment:
    net: int
    layer: str
    width_mm: float
    start: tuple          # (x, y) mm
    end: tuple
    width_span: tuple = (0, 0)   # (char_start, char_end) of the width token
    # How the width token is *written* in the source file, so a patch can put
    # the number back in the file's own units instead of silently converting
    # the board to millimetres. KiCad stores bare mm → scale 1, no suffix.
    # Altium ASCII stores e.g. `WIDTH=11.811mil` → scale 0.0254, suffix "mil".
    width_scale_mm: float = 1.0
    width_unit: str = ""
    # Arcs are tessellated into several PcbSegments that all share one width
    # token in the file; the patcher de-duplicates on the span so the same
    # token is never rewritten twice.
    from_arc: bool = False
    # Which arc of the file the chord came from (-1: not an arc). A binary
    # file has no width token to tell its arcs apart by.
    arc_id: int = -1

    @property
    def length_mm(self) -> float:
        return math.hypot(self.end[0] - self.start[0], self.end[1] - self.start[1])

    def width_token(self, mm: float) -> str:
        """Render `mm` the way this segment's width is written in the file."""
        val = mm / (self.width_scale_mm or 1.0)
        return f"{val:g}{self.width_unit}"


@dataclass
class PcbVia:
    net: int
    at: tuple
    size_mm: float
    drill_mm: float
    layers: tuple = ("F.Cu", "B.Cu")


@dataclass
class PcbPad:
    number: str
    net: int
    net_name: str
    at: tuple             # absolute board coords, mm
    size: tuple = (1.0, 1.0)
    through: bool = False
    # Which copper side this pad is actually on. KiCad pads inherit their
    # footprint's side, so its reader leaves this empty and the footprint
    # decides. Altium states the layer on the pad record itself — including
    # for free pads that belong to no component at all, where there is no
    # footprint to inherit from. Getting this wrong makes a bottom-side pad
    # invisible to the top-side copper graph and fakes a "copper open".
    layer: str = ""
    # "round" covers KiCad's circle/oval and Altium's ROUND (a circle when the
    # two sizes are equal, otherwise an oval); "rect" and "roundrect" as named;
    # "" when the reader could not tell. The angle is the pad's absolute
    # orientation on the board in degrees, counter-clockwise as the EDA displays
    # it. Both are used to draw the pad and to decide what copper touches it.
    shape: str = ""
    angle_deg: float = 0.0
    # Where the copper's centre sits relative to `at` (board frame, mm). KiCad
    # pads may offset their copper from the drill (a TO-92's staggered legs):
    # `at` is the hole, the copper is shifted by this much.
    copper_offset: tuple = (0.0, 0.0)
    # A KiCad `custom` pad's copper beyond its anchor, in board mm:
    # ("poly", [(x, y), ...]) and ("capsule", (x0, y0, x1, y1, half_width)).
    primitives: list = field(default_factory=list)
    #  A roundrect's corner radius as a fraction of its shorter side (KiCad's `roundrect_rratio`; 0.25 when absent).
    rratio: float = 0.25


@dataclass
class PcbFootprint:
    ref: str
    value: str
    layer: str
    at: tuple             # (x, y) mm
    pads: list = field(default_factory=list)


@dataclass
class PcbZone:
    """A copper pour: its net, its layer, and its outline in board mm.

    Only the *outline* is kept. The pour is used to answer "is this pad joined
    to the rest of its net", never "how much resistance does it add" — see
    `analyze_net` for why that asymmetry is deliberate.
    """
    net: int
    layer: str
    outline: list = field(default_factory=list)     # [(x, y), ...]
    spoke_mm: float = 0.5                             # thermal relief spoke width, mm (KiCad's thermal_bridge_width)
    # KiCad 5 fills (and `filled_areas_thickness yes`): the copper is the polygon with its outline stroked this
    # wide (the zone's min_thickness); 0 for KiCad 6+ fills, where the polygon is the copper.
    stroke_mm: float = 0.0

    def contains(self, pt, tol: float = 0.0) -> bool:
        """Ray-cast point-in-polygon, with an optional outward tolerance so a
        pad sitting exactly on the pour edge still counts as inside."""
        pts = self.outline
        n = len(pts)
        if n < 3:
            return False
        x, y = pt
        inside = False
        j = n - 1
        for i in range(n):
            xi, yi = pts[i]
            xj, yj = pts[j]
            if (yi > y) != (yj > y):
                xc = xi + (y - yi) * (xj - xi) / ((yj - yi) or 1e-12)
                if x < xc:
                    inside = not inside
            j = i
        if inside or tol <= 0:
            return inside
        j = n - 1
        for i in range(n):
            if _pt_seg_dist(pt, pts[i], pts[j]) <= tol:
                return True
            j = i
        return False


@dataclass
class PcbBoard:
    """Everything the Doctor needs from a routed board file, plus the raw text
    for surgical width patches. One model, two front-ends: KiCad s-expression
    and Altium/Protel ASCII both land here, so every physics check, the viewer
    and the patcher are written once and are EDA-agnostic. Layer names are
    normalised to the KiCad vocabulary (F.Cu / In*.Cu / B.Cu) on import —
    `native_layers` keeps the file's own names for round-trip display."""
    text: str = ""
    nets: dict = field(default_factory=dict)            # id -> name
    segments: list = field(default_factory=list)        # [PcbSegment]
    vias: list = field(default_factory=list)            # [PcbVia]
    footprints: list = field(default_factory=list)      # [PcbFootprint]
    zone_nets: set = field(default_factory=set)         # net ids with pours
    zones: list = field(default_factory=list)           # [PcbZone] with outlines
    copper_layers: list = field(default_factory=list)   # ["F.Cu", "In1.Cu", ...]
    board_thickness_mm: float = DEFAULT_BOARD_THICKNESS_MM
    copper_oz: float = 1.0                              # finished outer weight
    # ---- provenance: which EDA this came out of, and what we had to assume -- #
    fmt: str = "kicad"            # "kicad" | "altium" | "altium_binary"
    #  Can the source file be surgically re-written? True for the text formats,
    #  where a width is one token at a known character span. False for a native
    #  binary .PcbDoc: it is read for diagnosis only, because a mis-placed byte
    #  in a binary board corrupts a file that is about to go to a fab.
    patchable: bool = True
    length_unit: str = "mm"                             # unit the file writes in
    native_layers: dict = field(default_factory=dict)   # canonical -> file name
    notes: list = field(default_factory=list)           # import caveats, shown

    @property
    def fmt_label(self) -> str:
        return {"kicad": "KiCad", "altium": "Altium / Protel ASCII",
                "altium_binary": "Altium (native binary)"}.get(
            self.fmt, self.fmt)

    @property
    def file_suffix(self) -> str:
        """Extension a patched copy of this board should carry."""
        return {"kicad": ".kicad_pcb", "altium": ".PcbDoc",
                "altium_binary": ".PcbDoc"}.get(self.fmt, ".txt")

    def native_layer(self, canonical: str) -> str:
        return self.native_layers.get(canonical, canonical)

    # ---- convenience -------------------------------------------------------- #
    def net_id(self, name: str):
        for i, n in self.nets.items():
            if n == name:
                return i
        return None

    def net_name(self, nid: int) -> str:
        return self.nets.get(nid, f"net#{nid}")

    def segments_of(self, nid: int):
        return [s for s in self.segments if s.net == nid]

    def vias_of(self, nid: int):
        return [v for v in self.vias if v.net == nid]

    def pads_of(self, nid: int):
        out = []
        for fp in self.footprints:
            for p in fp.pads:
                if p.net == nid:
                    out.append((fp, p))
        return out

    def routed_net_ids(self):
        ids = sorted({s.net for s in self.segments} | {v.net for v in self.vias})
        return [i for i in ids if i in self.nets and self.nets[i]]

    def bbox(self):
        xs, ys = [], []
        for s in self.segments:
            xs += [s.start[0], s.end[0]]; ys += [s.start[1], s.end[1]]
        for v in self.vias:
            xs.append(v.at[0]); ys.append(v.at[1])
        for fp in self.footprints:
            xs.append(fp.at[0]); ys.append(fp.at[1])
        if not xs:
            return (0.0, 0.0, 100.0, 100.0)
        return (min(xs), min(ys), max(xs), max(ys))


# --------------------------------------------------------------------------- #
#  Which EDA drew this? — one door for both formats
# --------------------------------------------------------------------------- #
SUPPORTED_SUFFIXES = (".kicad_pcb", ".pcbdoc", ".pcb", ".txt")


def sniff_format(data, filename: str = "") -> str:
    """Identify a dropped board file from its *content* first and its name only
    as a tie-break — teams rename exports constantly, and an Altium ASCII board
    saved as `board.txt` should still just work.

    Returns "kicad", "altium", "altium_binary", or "unknown".
    """
    from . import pcb_altium as _alt
    if isinstance(data, (bytes, bytearray, memoryview)):
        if _alt.is_altium_binary(data):
            return "altium_binary"
        head = bytes(data)[:20000].decode("utf-8", errors="replace")
    else:
        head = data[:20000]
        if _alt.is_altium_binary(head):
            return "altium_binary"
    if re.search(r"\|RECORD=", head, re.I):
        return "altium"
    if "kicad_pcb" in head or re.match(r"\s*\(\s*(kicad_pcb|pcb)\b", head):
        return "kicad"
    low = (filename or "").lower()
    if low.endswith(".kicad_pcb"):
        return "kicad"
    if low.endswith((".pcbdoc", ".pcb")):
        return "altium"
    return "unknown"


def parse_board(data, filename: str = "") -> PcbBoard:
    """Parse a routed board from **either** EDA into the one `PcbBoard` model.

    Everything downstream — the ampacity mesh, the nodal IR-drop solve, the
    clearance table, the viewer, the patcher, the report — is written against
    that model, so adding Altium added a reader, not a second Doctor. Accepts
    bytes or text; raises ValueError with something actionable in it, since
    "couldn't parse" is useless to someone holding a board file.
    """
    from . import pcb_altium as _alt
    fmt = sniff_format(data, filename)
    if fmt == "altium_binary":
        #  Altium saves binary by default, so this is the file a member
        #  actually has. Read it if we can; the ASCII instructions are the
        #  fallback, not the first answer.
        from . import pcb_altium_binary as _bin
        if isinstance(data, str):
            data = data.encode("latin-1", errors="replace")
        try:
            return _bin.parse_altium_binary(bytes(data))
        except _bin.AltiumBinaryUnavailable as exc:
            raise ValueError(f"{exc}\n\n{_alt.ALTIUM_BINARY_HELP}") from exc
        except ValueError as exc:
            # The reader's own sanity failures already carry the way out; a
            # ValueError from the OLE layer does not, and "bytes length not a
            # multiple of item size" helps nobody holding a board file.
            msg = str(exc)
            if "ASCII" not in msg:
                msg = f"{msg}\n\n{_alt.ALTIUM_BINARY_HELP}"
            raise ValueError(msg) from exc
        except Exception as exc:                    # noqa: BLE001
            raise ValueError(
                f"this native .PcbDoc could not be read ({type(exc).__name__}: "
                f"{exc}).\n\n{_alt.ALTIUM_BINARY_HELP}") from exc
    text = (bytes(data).decode("utf-8", errors="replace")
            if isinstance(data, (bytes, bytearray, memoryview)) else data)
    if fmt == "altium":
        return _alt.parse_altium_ascii(text)
    if fmt == "kicad":
        return parse_kicad_pcb(text)
    raise ValueError(
        "unrecognised board file — the Doctor reads KiCad `.kicad_pcb` "
        "(v5–9) and Altium/Protel **ASCII** PCB (`.PcbDoc` saved as ASCII). "
        "A native binary Altium `.PcbDoc` needs File ▸ Save As ▸ *PCB ASCII "
        "File* first. Gerbers, ODB++ and IPC-2581 are not read: they carry "
        "copper but not the net names and component references every finding "
        "here is written in terms of.")


# --------------------------------------------------------------------------- #
#  Arc tessellation (shared by both readers)
# --------------------------------------------------------------------------- #
#  Routed arcs are copper. Dropping them does not merely lose a little length —
#  it breaks the connectivity graph, and the "copper open / rats-nest missed"
#  check would then flag a perfectly connected net as dead on arrival. Both
#  front-ends therefore chord every arc into short straight segments that share
#  one width token in the file.
#
#  Chords cut the corner, so a tessellated arc reads very slightly *shorter*
#  than the copper really is — i.e. slightly optimistic on resistance, the
#  wrong direction for a screening tool. 10° keeps that under 0.15% of the
#  arc's length, far inside the error already carried by the assigned current.
_ARC_MAX_STEP_DEG = 10.0


def _arc_points_center(cx: float, cy: float, r: float,
                       a0_deg: float, a1_deg: float) -> list:
    """Points along an arc given centre/radius/angles (degrees, CCW)."""
    sweep = a1_deg - a0_deg
    while sweep <= 0:
        sweep += 360.0
    while sweep > 360.0:
        sweep -= 360.0
    n = max(2, int(math.ceil(sweep / _ARC_MAX_STEP_DEG)) + 1)
    return [(cx + r * math.cos(math.radians(a0_deg + sweep * i / (n - 1))),
             cy + r * math.sin(math.radians(a0_deg + sweep * i / (n - 1))))
            for i in range(n)]


def _arc_points_3pt(p0, pm, p1) -> list:
    """Points along the arc start→mid→end (KiCad's representation). Falls back
    to the straight chord if the three points are collinear."""
    (x0, y0), (xm, ym), (x1, y1) = p0, pm, p1
    d = 2.0 * (x0 * (ym - y1) + xm * (y1 - y0) + x1 * (y0 - ym))
    if abs(d) < 1e-12:
        return [p0, p1]
    ux = ((x0 ** 2 + y0 ** 2) * (ym - y1) + (xm ** 2 + ym ** 2) * (y1 - y0)
          + (x1 ** 2 + y1 ** 2) * (y0 - ym)) / d
    uy = ((x0 ** 2 + y0 ** 2) * (x1 - xm) + (xm ** 2 + ym ** 2) * (x0 - x1)
          + (x1 ** 2 + y1 ** 2) * (xm - x0)) / d
    r = math.hypot(x0 - ux, y0 - uy)
    if not (r > 0) or r > 1e6:
        return [p0, p1]
    a0 = math.degrees(math.atan2(y0 - uy, x0 - ux))
    am = math.degrees(math.atan2(ym - uy, xm - ux))
    a1 = math.degrees(math.atan2(y1 - uy, x1 - ux))
    ccw = ((am - a0) % 360.0) < ((a1 - a0) % 360.0)
    pts = (_arc_points_center(ux, uy, r, a0, a1) if ccw
           else list(reversed(_arc_points_center(ux, uy, r, a1, a0))))
    # pin the endpoints exactly, so the node keys match the touching segments
    pts[0], pts[-1] = p0, p1
    return pts


def _chord_segments(pts: list, **seg_kw) -> list:
    """Turn a polyline into PcbSegments, all sharing the arc's width token."""
    out = []
    for a, b in zip(pts, pts[1:]):
        if math.hypot(b[0] - a[0], b[1] - a[1]) < 1e-9:
            continue
        out.append(PcbSegment(start=a, end=b, from_arc=True, **seg_kw))
    return out


def parse_kicad_pcb(text: str) -> PcbBoard:
    """Parse a KiCad 5–9 .kicad_pcb file into a PcbBoard. Tolerant: anything
    it does not recognise is skipped, never fatal."""
    root = parse_sexpr(text)
    if not isinstance(root, list) or _name(root) not in ("kicad_pcb", "pcb"):
        raise ValueError("not a KiCad board file (expected a kicad_pcb node)")
    board = PcbBoard(text=text)

    # copper layers from the (layers …) block
    lay = _child(root, "layers")
    if lay:
        for entry in lay[1:]:
            if isinstance(entry, list):
                ats = [c for c in entry if isinstance(c, Atom)]
                cu = next((a.value for a in ats if a.value.endswith(".Cu")), None)
                if cu:
                    board.copper_layers.append(cu)
    if not board.copper_layers:
        board.copper_layers = ["F.Cu", "B.Cu"]

    # board thickness from (general (thickness x))
    gen = _child(root, "general")
    if gen:
        th = _child(gen, "thickness")
        if th:
            v = _floats(th)
            if v:
                board.board_thickness_mm = v[0]

    # KiCad 10 dropped numeric net IDs: where v5-9 wrote `(net 2 "VCC")` and
    # declared every net up front, v10 writes `(net "VCC")` on each object and
    # declares nothing. Parsing only the numeric form does not fail loudly on a
    # v10 board — it drops EVERY segment and reports a board with no copper and
    # no nets, which looks like an empty file rather than a bug. So a net
    # reference is resolved either way, and names met for the first time are
    # interned into the same id space the rest of the Doctor already uses.
    _by_name = {}

    def _net_ref(nt) -> int:
        """Net id from `(net 2 "VCC")` (v5-9) or `(net "VCC")` (v10)."""
        if not nt:
            return 0
        ats = _atoms(nt)
        if not ats:
            return 0
        raw = ats[0].value
        try:
            return int(float(raw))
        except (TypeError, ValueError, OverflowError):
            pass
        name = str(raw).strip().strip('"')
        if not name:
            return 0
        if name not in _by_name:
            nid = (max(board.nets) + 1) if board.nets else 1
            board.nets[nid] = name
            _by_name[name] = nid
        return _by_name[name]

    #  A KiCad 5 via may omit its drill: it then has its net class's via drill,
    #  else the board default in (setup (via_drill …)). Assuming a fixed 0.3 mm
    #  instead misread every such via on a real FSAE precharge board (0.4 mm in
    #  KiCad), found by the electrical benchmark's held-out boards.
    _drill_default = 0.3
    _drill_by_net: dict = {}
    _clear_by_name: dict = {}
    #  KiCad before 6 (file version before 20200811) stored a pour shrunk by half its minimum width and stroked the
    #  outline back on when drawing; reading the bare polygon loses 7-9 % of the copper (benchmark addendum 8).
    _ver = _child(root, "version")
    _legacy_fill = bool(_ver and _floats(_ver) and _floats(_ver)[0] < 20200811)
    for node in root[1:]:
        if not isinstance(node, list):
            continue
        if _name(node) == "setup":
            vd = _child(node, "via_drill")
            if vd and _floats(vd):
                _drill_default = _floats(vd)[0]
        elif _name(node) == "net_class":
            vd = _child(node, "via_drill")
            if vd and _floats(vd):
                for an in _children(node, "add_net"):
                    if _atoms(an):
                        _drill_by_net[str(_atoms(an)[0].value).strip('"')] = _floats(vd)[0]
            cl = _child(node, "clearance")              # the class's copper clearance (mm), KiCad 5-7 boards
            if cl and _floats(cl) and math.isfinite(_floats(cl)[0]):
                for an in _children(node, "add_net"):
                    if _atoms(an):
                        _clear_by_name[str(_atoms(an)[0].value).strip('"')] = _floats(cl)[0]

    _unreadable_tracks = 0
    _unfilled_zones: list = []          # (net id, first and one-past-last index of its outlines in board.zones)
    _any_fill = False
    for node in root[1:]:
        if not isinstance(node, list):
            continue
        nm = _name(node)
        if nm == "net":
            ats = _atoms(node)
            if ats:
                try:
                    nid = int(float(ats[0].value))
                except (ValueError, OverflowError):
                    continue
                board.nets[nid] = ats[1].value if len(ats) > 1 else ""
        elif nm == "segment":
            st_, en_, wd, ly, nt = (_child(node, k) for k in
                                    ("start", "end", "width", "layer", "net"))
            if not (st_ and en_ and wd and nt):
                continue
            w_atom = next((a for a in wd[1:] if isinstance(a, Atom)), None)
            try:
                seg = PcbSegment(
                    net=_net_ref(nt),
                    layer=_atoms(ly)[0].value if ly and _atoms(ly) else "F.Cu",
                    width_mm=_positive(w_atom.value),
                    start=tuple(_floats(st_, 2)), end=tuple(_floats(en_, 2)),
                    width_span=(w_atom.start, w_atom.end))
                board.segments.append(seg)
            except (ValueError, IndexError, AttributeError):
                _unreadable_tracks += 1
                continue
        elif nm == "arc":
            # A *routed* arc (KiCad ≥6). `gr_arc` is graphics and is not copper,
            # so the exact name match above matters.
            st_, md_, en_, wd, ly, nt = (_child(node, k) for k in
                                         ("start", "mid", "end", "width",
                                          "layer", "net"))
            if not (st_ and md_ and en_ and wd and nt):
                continue
            w_atom = next((a for a in wd[1:] if isinstance(a, Atom)), None)
            try:
                pts = _arc_points_3pt(tuple(_floats(st_, 2)),
                                      tuple(_floats(md_, 2)),
                                      tuple(_floats(en_, 2)))
                board.segments.extend(_chord_segments(
                    pts,
                    net=_net_ref(nt),
                    layer=_atoms(ly)[0].value if ly and _atoms(ly) else "F.Cu",
                    width_mm=_positive(w_atom.value),
                    width_span=(w_atom.start, w_atom.end)))
            except (ValueError, IndexError, AttributeError):
                _unreadable_tracks += 1
                continue
        elif nm == "via":
            at, sz, dr, ly, nt = (_child(node, k) for k in
                                  ("at", "size", "drill", "layers", "net"))
            if not (at and nt):
                continue
            try:
                layers = tuple(a.value for a in _atoms(ly)) if ly else ("F.Cu", "B.Cu")
                vnet = _net_ref(nt)
                board.vias.append(PcbVia(
                    net=vnet,
                    at=tuple(_floats(at, 2)),
                    size_mm=_floats(sz)[0] if sz and _floats(sz) else 0.6,
                    drill_mm=(_floats(dr)[0] if dr and _floats(dr) else
                              _drill_by_net.get(board.nets.get(vnet, ""), _drill_default)),
                    layers=layers or ("F.Cu", "B.Cu")))
            except (ValueError, IndexError):
                continue
        elif nm in ("footprint", "module"):
            board.footprints.append(_parse_footprint(node, _net_ref))
        elif nm == "zone":
            nt = _child(node, "net")
            if not (nt and _atoms(nt)):
                continue
            try:
                znid = _net_ref(nt)
            except ValueError:
                continue
            board.zone_nets.add(znid)
            # Outlines, so the pour can join pads that nothing else reaches.
            # `filled_polygon` is the copper KiCad actually poured (it respects
            # clearances and islands); the `polygon` outline is only the region
            # the user drew, so prefer the former and fall back to the latter
            # for a zone that has never been filled.
            zl = _child(node, "layer") or _child(node, "layers")
            zlayers = [str(a.value).strip('"') for a in _atoms(zl)] if zl else []
            zlayers = [l for l in zlayers if l.endswith(".Cu")] or ["F.Cu"]
            filled = _children(node, "filled_polygon")
            spoke = 0.5
            fl = _child(node, "fill")
            tb = _child(fl, "thermal_bridge_width") if fl else None
            if tb and _atoms(tb):
                try:
                    spoke = _positive(_atoms(tb)[0].value)
                except (TypeError, ValueError):
                    pass
            stroke = 0.0
            fat = _child(node, "filled_areas_thickness")
            if filled and ((fat and _atoms(fat) and str(_atoms(fat)[0].value) == "yes") or (_legacy_fill and not fat)):
                mt = _child(node, "min_thickness")
                stroke = _floats(mt)[0] if mt and _floats(mt) else 0.254
            _z0 = len(board.zones)
            for poly in (filled or _children(node, "polygon")):
                pts_node = _child(poly, "pts")
                if not pts_node:
                    continue
                pts = [tuple(_floats(xy, 2)) for xy in _children(pts_node, "xy")]
                pts = [q for q in pts if len(q) == 2]
                if len(pts) < 3:
                    continue
                players = zlayers
                pl = _child(poly, "layer")
                if pl and _atoms(pl):
                    nm_ = str(_atoms(pl)[0].value).strip('"')
                    if nm_.endswith(".Cu"):
                        players = [nm_]
                for lay in players:
                    board.zones.append(
                        PcbZone(net=znid, layer=lay, outline=pts, spoke_mm=spoke, stroke_mm=stroke))
            if filled:
                _any_fill = True
            elif znid and not _child(node, "keepout"):       # a keepout or a no-net zone was never going to be copper
                _unfilled_zones.append((znid, _z0, len(board.zones)))
    #  A pour with no stored fill. If other pours in the file are filled, KiCad filled this one too and found nothing
    #  to keep (an island joined to no pad): it is not copper, and reading its outline as copper joined pads and
    #  lowered resistances that KiCad 9 does not (benchmark addendum 11). If no pour in the file is filled, the
    #  board was saved without filling: the outlines stand in, and the report says so.
    if _unfilled_zones and _any_fill:
        drop = {i for _nid, i0, i1 in _unfilled_zones for i in range(i0, i1)}
        nets_ = sorted({board.nets.get(nid_, str(nid_)) for nid_, _i0, _i1 in _unfilled_zones})
        board.zones = [z for i, z in enumerate(board.zones) if i not in drop]
        board.zone_nets &= {z.net for z in board.zones}      # a net whose only pour is empty is a trace-only net
        board.notes.append(
            f"{len(_unfilled_zones)} pour(s) have no filled copper in the file (nets: {', '.join(nets_[:6])}"
            f"{', ...' if len(nets_) > 6 else ''}). KiCad left them empty, so they are not counted as copper.")
    elif _unfilled_zones:
        board.notes.append(
            "The pours in this file were saved without being filled. Their drawn outlines are used as the copper; "
            "fill all zones in KiCad (B) and save again for the real shapes.")
    board.clearance_mm = {nid: _clear_by_name[nm] for nid, nm in board.nets.items() if nm in _clear_by_name}
    if _unreadable_tracks:
        board.notes.append(
            f"{_unreadable_tracks} track(s) in the file have no usable width, position or net and were left out. "
            "Nets they belong to may read as open; re-save the board from KiCad and upload it again.")
    return board


def _pad_primitives(pd, centre, angle_deg) -> list:
    """A custom pad's copper primitives in board mm. They are written in the pad's own frame (turned by its angle,
    KiCad's y down), like a pad's drill offset."""
    pr_ = _child(pd, "primitives")
    if not pr_:
        return []
    t = math.radians(angle_deg)
    ct, st = math.cos(t), math.sin(t)

    def _tf(u, v):
        return (centre[0] + u * ct + v * st, centre[1] - u * st + v * ct)

    def _width(n):
        w = _child(n, "width") or _child(_child(n, "stroke") or [], "width")
        return _floats(w)[0] if w and _floats(w) else 0.0

    def _filled(n):
        f = _child(n, "fill")
        return not f or not _atoms(f) or str(_atoms(f)[0].value) in ("yes", "solid")

    out: list = []
    for n in pr_[1:]:
        if not isinstance(n, list):
            continue
        k, w = _name(n), _width(n)
        try:
            if k == "gr_poly":
                pts = [_tf(*_floats(xy, 2)) for xy in _children(_child(n, "pts") or [], "xy")]
                if len(pts) >= 3 and _filled(n):
                    out.append(("poly", pts))
                if w > 1e-3:
                    out += [("capsule", (*a, *b, w / 2.0)) for a, b in zip(pts, pts[1:] + pts[:1])]
            elif k in ("gr_rect", "gr_circle", "gr_line", "gr_arc"):
                a = _floats(_child(n, "start") or _child(n, "center"), 2)
                b = _floats(_child(n, "end"), 2)
                if k == "gr_rect":
                    c = [_tf(a[0], a[1]), _tf(b[0], a[1]), _tf(b[0], b[1]), _tf(a[0], b[1])]
                    if _filled(n):
                        out.append(("poly", c))
                    if w > 1e-3:
                        out += [("capsule", (*p, *q, w / 2.0)) for p, q in zip(c, c[1:] + c[:1])]
                elif k == "gr_circle":
                    r = math.hypot(b[0] - a[0], b[1] - a[1])
                    cx, cy = _tf(*a)
                    if _filled(n) or w > 2 * r:
                        out.append(("capsule", (cx, cy, cx, cy, r + w / 2.0)))
                    else:                                           # an outline only: a ring of chords
                        ring = [(cx + r * math.cos(2 * math.pi * i / 48), cy + r * math.sin(2 * math.pi * i / 48))
                                for i in range(49)]
                        out += [("capsule", (*p, *q, w / 2.0)) for p, q in zip(ring, ring[1:])]
                elif k == "gr_line":
                    out.append(("capsule", (*_tf(*a), *_tf(*b), max(w, 1e-3) / 2.0)))
                else:                                               # gr_arc: start, mid, end (KiCad 6+)
                    m = _floats(_child(n, "mid"), 2)
                    pts = _arc_points_3pt(_tf(*a), _tf(*m), _tf(*b)) if len(m) == 2 else [_tf(*a), _tf(*b)]
                    out += [("capsule", (*p, *q, max(w, 1e-3) / 2.0)) for p, q in zip(pts, pts[1:])]
        except (TypeError, ValueError, IndexError):
            continue
    return out


def _parse_footprint(node, net_ref=None) -> PcbFootprint:
    at = _child(node, "at")
    fx, fy, frot = (_floats(at, 3) if at else [0.0, 0.0, 0.0])
    ly = _child(node, "layer")
    layer = _atoms(ly)[0].value if ly and _atoms(ly) else "F.Cu"
    ref, val = "?", ""
    # KiCad ≥7: (property "Reference" "C1" …) — KiCad ≤6: (fp_text reference C1 …)
    for pr in _children(node, "property"):
        ats = _atoms(pr)
        if len(ats) >= 2:
            key = ats[0].value.lower()
            if key == "reference":
                ref = ats[1].value
            elif key == "value":
                val = ats[1].value
    for ft in _children(node, "fp_text"):
        ats = _atoms(ft)
        if len(ats) >= 2:
            if ats[0].value == "reference":
                ref = ats[1].value
            elif ats[0].value == "value":
                val = ats[1].value
    fp = PcbFootprint(ref=ref, value=val, layer=layer, at=(fx, fy))
    rot = math.radians(frot or 0.0)
    cr, sr = math.cos(rot), math.sin(rot)
    for pd in _children(node, "pad"):
        ats = _atoms(pd)
        number = ats[0].value if ats else "?"
        through = any(a.value in ("thru_hole", "np_thru_hole") for a in ats)
        # Which copper side is this pad on? NOT necessarily its footprint's.
        # A PCI edge connector is one footprint placed on the top whose fingers
        # sit on both sides; plenty of designs also put an SMD pad on the far
        # side of a top-side part. Inheriting the footprint's layer puts those
        # pads on the wrong copper, the trace that reaches them is invisible to
        # the connectivity graph, and a perfectly routed net is reported as
        # "copper open" — this was the single largest false-alarm source on a
        # real 4-layer board. KiCad states it on the pad: (layers "B.Cu" ...).
        pad_cu = [str(a.value).strip('"') for a in _atoms(_child(pd, "layers") or [])
                  if str(a.value).strip('"').endswith(".Cu")
                  or str(a.value).strip('"') in ("*.Cu",)]
        if any(l == "*.Cu" for l in pad_cu) or len(pad_cu) > 1:
            through = True          # spans sides: reachable from either
            pad_layer = ""
        else:
            pad_layer = pad_cu[0] if pad_cu else ""
        pat = _child(pd, "at")
        # the third number is the pad's absolute board orientation — KiCad
        # writes it with the footprint's own rotation already folded in
        px, py, pang = (_floats(pat, 3) if pat else [0.0, 0.0, 0.0])
        kinds = {str(a.value) for a in ats[1:3]}
        shape = ("round" if kinds & {"circle", "oval"} else
                 "roundrect" if "roundrect" in kinds else
                 "rect" if kinds & {"rect", "trapezoid"} else "")
        # pad offset is in footprint frame; rotate into board frame.
        # KiCad's y axis points down; fp rotation is CCW in its own convention —
        # the standard transform below matches KiCad's file coordinates.
        ax = fx + px * cr + py * sr
        ay = fy - px * sr + py * cr
        psz = _child(pd, "size")
        size = tuple(_floats(psz, 2)) if psz else (1.0, 1.0)
        rr_ = _child(pd, "roundrect_rratio")
        rratio = min(max(_floats(rr_)[0], 0.0), 0.5) if rr_ and _floats(rr_) else 0.25
        # (drill … (offset dx dy)): the copper sits off the hole, in the pad's
        # own frame; rotate it by the pad's board orientation like `at` above.
        off = (0.0, 0.0)
        pdr = _child(pd, "drill")
        poff = _child(pdr, "offset") if pdr else None
        if poff and len(_floats(poff, 2)) == 2:
            ox, oy = _floats(poff, 2)
            pr = math.radians(float(pang or 0.0))
            off = (ox * math.cos(pr) + oy * math.sin(pr), -ox * math.sin(pr) + oy * math.cos(pr))
        pnet, pname = 0, ""
        pnt = _child(pd, "net")
        if pnt and _atoms(pnt):
            a2 = _atoms(pnt)
            if net_ref is not None:
                pnet = net_ref(pnt)          # handles both KiCad dialects
            else:
                try:
                    pnet = int(float(a2[0].value))
                except (TypeError, ValueError, OverflowError):
                    pnet = 0
            # v5-9: (net 2 "VCC"); v10: (net "VCC")
            pname = (a2[1].value if len(a2) > 1
                     else str(a2[0].value).strip().strip('"'))
            if pname.lstrip("-").isdigit():
                pname = ""
        prims = _pad_primitives(pd, (ax + off[0], ay + off[1]), float(pang or 0.0)) if "custom" in kinds else []
        fp.pads.append(PcbPad(number=number, net=pnet, net_name=pname,
                              at=(ax, ay), size=size, through=through,
                              layer=pad_layer, shape=shape,
                              angle_deg=float(pang or 0.0), copper_offset=off, primitives=prims,
                              rratio=rratio))
    return fp


# --------------------------------------------------------------------------- #
#  IPC-2221 sizing primitives (the Trace Prescriber core)
# --------------------------------------------------------------------------- #
def required_area_mil2(current_a: float, dT_c: float, external: bool) -> float:
    """Invert the IPC-2221 heating chart: copper cross-section (mil²) that keeps
    the steady-state rise at `dT_c` for `current_a`."""
    if current_a <= 0 or dT_c <= 0:
        return 0.0
    k = IPC_K       # internal as external: IPC-2152 found buried traces no hotter (see electronics.Trace)
    return float((current_a / (k * dT_c ** 0.44)) ** (1.0 / 0.725))


def required_width_mm(current_a: float, dT_c: float, copper_oz: float,
                      external: bool) -> float:
    """Minimum finished trace width for a current at a temperature-rise budget."""
    a_mm2 = required_area_mil2(current_a, dT_c, external) / MM2_TO_MIL2
    t_mm = copper_oz * OZ_TO_UM / 1000.0
    return a_mm2 / t_mm if t_mm > 0 else math.inf


def trace_ampacity_a(width_mm: float, dT_c: float, copper_oz: float,
                     external: bool) -> float:
    """The forward IPC-2221 question: what current does *this* copper carry at
    a given rise? `required_width_mm` answers the reverse. Both directions are
    needed, because when no current is declared the only honest thing to report
    is the capacity the trace already has — a fact about the board — rather
    than a verdict against a current nobody stated."""
    t_mm = copper_oz * OZ_TO_UM / 1000.0
    a_mil2 = max(width_mm, 0.0) * t_mm * MM2_TO_MIL2
    if a_mil2 <= 0 or dT_c <= 0:
        return 0.0
    k = IPC_K       # internal as external: IPC-2152 found buried traces no hotter (see electronics.Trace)
    return k * (dT_c ** 0.44) * (a_mil2 ** 0.725)


def via_ampacity_a(drill_mm: float, dT_c: float,
                   plating_mm: float = DEFAULT_VIA_PLATING_MM) -> float:
    """Steady-state current one plated via barrel carries at a given rise.
    Barrel cross-section = π · drill · plating; treated as internal copper."""
    a_mil2 = math.pi * drill_mm * plating_mm * MM2_TO_MIL2
    return 0.024 * (dT_c ** 0.44) * (a_mil2 ** 0.725)


def vias_needed(current_a: float, drill_mm: float, dT_c: float,
                plating_mm: float = DEFAULT_VIA_PLATING_MM) -> int:
    cap = via_ampacity_a(drill_mm, dT_c, plating_mm)
    return max(1, math.ceil(current_a / cap)) if cap > 0 else 1


def via_resistance_ohm(drill_mm: float, length_mm: float,
                       plating_mm: float = DEFAULT_VIA_PLATING_MM,
                       temp_c: float = 40.0) -> float:
    rho = RHO_CU_20C * (1.0 + ALPHA_CU * (temp_c - 20.0))
    area_m2 = math.pi * (drill_mm * 1e-3) * (plating_mm * 1e-3)
    return rho * (length_mm * 1e-3) / max(area_m2, 1e-12)


def prescribe_trace(current_a: float, dT_c: float = 20.0,
                    length_mm: float = 100.0, rail_v: float = 5.0,
                    via_drill_mm: float = 0.3,
                    board_thickness_mm: float = DEFAULT_BOARD_THICKNESS_MM) -> dict:
    """The one-shot answer to "how do I trace this?": required width for each
    copper weight on outer and inner layers, via count per layer transition, and
    the IR drop / dissipation the chosen width implies. Pure IPC-2221 analytics."""
    rows = []
    for oz in (0.5, 1.0, 2.0):
        for ext, cls in ((True, "outer (F.Cu / B.Cu)"), (False, "inner (In*.Cu)")):
            w = required_width_mm(current_a, dT_c, oz, ext)
            tr = Trace(name="rx", net="rx", owner_subsystem="electrics",
                       width_mm=max(w, 1e-4), copper_oz=oz,
                       length_mm=length_mm, is_external=ext)
            rows.append({
                "copper_oz": oz, "layer_class": cls, "width_mm": w,
                "ir_drop_v": tr.voltage_drop_v(current_a, temp_c=40.0 + dT_c),
                "dissipation_w": tr.power_dissipated_w(current_a, temp_c=40.0 + dT_c),
            })
    n_vias = vias_needed(current_a, via_drill_mm, dT_c)
    return {
        "current_a": current_a, "dT_c": dT_c, "length_mm": length_mm,
        "rows": rows,
        "vias_per_transition": n_vias,
        "via_drill_mm": via_drill_mm,
        "via_note": (f"every time this net changes layer it must cross on "
                     f"{n_vias}× ⌀{via_drill_mm:g} mm vias (barrel plating "
                     f"{DEFAULT_VIA_PLATING_MM*1000:.0f} µm) to keep the barrel "
                     f"rise ≤ {dT_c:g} °C — one via is the classic hidden "
                     f"bottleneck on a multi-layer power net"),
        "rail_v": rail_v,
    }


# --------------------------------------------------------------------------- #
#  Net analysis — connectivity + true resistance by nodal analysis
# --------------------------------------------------------------------------- #
_MAX_NODAL_NODES = 900   # pinv is O(n³); above this, honesty > guessing


#  Two copper endpoints this close are the same electrical point. Real boards
#  are full of them: routers emit coordinates that disagree in the last micron,
#  and rounding to a fixed grid then drops the two sides of a joint into
#  different buckets — measured 11 um apart on a real 4-layer board, splitting
#  a fully routed net into three islands and reporting it "copper open".
#
#  50 um is chosen to be unambiguous rather than generous: routed traces are
#  100-500 um wide, so endpoints 50 um apart physically overlap in copper and
#  cannot be an intentional break, while a genuine unrouted connection is a
#  rats-nest line measured in millimetres. Widening this further would start
#  welding real gaps shut, which is the one error direction this tool must not
#  have — so it stays at the smallest value that fixes the observed failure.
NODE_WELD_MM = 0.05

#: The one-way contract every connectivity tolerance in this module keeps.
#:
#: PCB Doctor is allowed to cry wolf. It is not allowed to wave a bad board
#: through. Concretely: an error that reports a *connected* net as open wastes
#: a member's afternoon, while an error that reports an *open* net as connected
#: sends a dead board to a fab. Every knob below is therefore set to the
#: smallest value that fixes an observed, measured failure — never to the value
#: that makes a complaint go away.
#:
#: If you are reading this because someone reported a false "copper open" and
#: you are about to widen NODE_WELD_MM: don't. Find the physical reason two
#: pieces of copper touch, the way the via-annulus rule did — that rule reaches
#: much further than 50 um, and is *safer*, because the distance comes from the
#: via's own diameter in the file rather than from a number someone picked.
#: A tolerance justified by geometry can be as large as the geometry says.
#: A tolerance justified by "it made the warnings stop" cannot be any size.
#:
#: `tests/test_pcb_doctor.py` pins this. If one of those tests is failing,
#: the assertion is not the thing that is wrong.
SAFETY_CONTRACT = "false alarms are acceptable; false all-clears are not"

#  worst_r_ohm is a best estimate, checked against a 2-D solve of the copper
#  (docs/ELECTRICAL_BENCHMARK.md, round 3); it can read low as well as high.
#  The brown-out check must not under-read, so it uses the estimate times this
#  factor (unitless): on the boards the model was designed on, against the
#  corrected 2-D reference (addendum 3), the 2-D answer was never more than
#  7.3 % above it (1/(1-0.073) = 1.079), rounded up to 1.10.
NET_R_UPPER_FACTOR = 1.10

#  The 1-D estimate misses by up to 9 % on short pad-to-pad nets (benchmark B6). Where a net is small enough, a 2-D
#  sheet solve of the same copper between the same two pads replaces it (plan addendum 12). This many cells in all
#  keeps one net to a few seconds and a few hundred MB, so the hosted app stays responsive; a larger net keeps the
#  1-D value and its margin.
REFINE_MAX_CELLS = 150_000
#  The brown-out check holds a converged sheet-solve value to this factor (unitless): the solve's own grid tolerance.
SHEET_UPPER_FACTOR = 1.02


def refined_net_resistance(board: PcbBoard, nid: int, nr: dict = None) -> dict:
    """The best estimate of a trace-only net's worst pad-to-pad resistance (Ω): the 2-D sheet solve between the 1-D
    model's worst pair when it converges on its own grid check and fits REFINE_MAX_CELLS, else the 1-D value.
    Returns {"r_ohm", "pair", "source" ("sheet solve" or "1-D"), "r_1d_ohm", "grid_change_pct"}."""
    nr = nr if nr is not None else analyze_net(board, nid)
    r1, pair = nr.get("worst_r_ohm"), nr.get("worst_pair")
    out = {"r_ohm": r1, "pair": pair, "source": "1-D", "r_1d_ohm": r1, "grid_change_pct": None}
    if r1 is None or not pair or pair[0] == pair[1] or nid in board.zone_nets:
        return out
    try:
        from . import pcb_pour
        sh = pcb_pour.pour_resistance(board, nid, pair=tuple(pair), max_cells=REFINE_MAX_CELLS)
    except (ImportError, MemoryError):                     # no scipy, or no room: the 1-D value stands
        return out
    if sh.get("r_ohm") and sh.get("converged"):
        out.update(r_ohm=sh["r_ohm"], source="sheet solve", grid_change_pct=sh["grid_change_pct"])
    return out


#  A pour estimate whose grid check did not converge is held to this factor (unitless) in the brown-out check. The
#  worst an unconverged estimate read low on the held-out pour nets was 6.5 %.
POUR_UNVERIFIED_FACTOR = 1.10
POUR_MAX_PADS = 64                 # pads of a pour net searched for the worst pair
#  The opt-in "solve large pours" limit (cells in all): planes up to about 1.5 million cells per layer, the size the
#  held-out large-plane benchmark (pours_round4.json, 7 of 7 converged nets within 2 %) was solved at. Minutes per
#  plane, sometimes much longer; a plane larger than this is still not estimated.
LARGE_POUR_MAX_CELLS = 3_000_000


def pour_net_resistance(board: PcbBoard, nid: int, max_cells: int | None = None) -> dict | None:
    """Worst pad-to-pad resistance (Ω) of a net through its pour, from the sheet solve limited to `max_cells` cells
    (default REFINE_MAX_CELLS, seconds per net; LARGE_POUR_MAX_CELLS solves large planes, minutes per net):
    {"r_ohm", "pair", "converged", ...}, or None when the net has no pour, is too large to solve here, or its pads
    are not joined by copper."""
    if nid not in board.zone_nets:
        return None
    try:
        from . import pcb_pour
        out = pcb_pour.pour_resistance(board, nid, max_pads=POUR_MAX_PADS, max_cells=max_cells or REFINE_MAX_CELLS)
    except (ImportError, MemoryError):
        return None
    return out if out.get("r_ohm") else None


#  The board that comes back from the fab is not the file. Finished copper can
#  be thinner than ordered (plating and etch tolerance), so the brown-out check
#  also assumes the copper may be this fraction of its nominal thickness. 0.90
#  (10 % thin) is a cautious default, not a standard: set your fab's finished
#  copper tolerance through diagnose(copper_finish=...).
COPPER_FINISH_FACTOR = 0.90


def _node_key(layer: str, pt) -> tuple:
    return (layer, round(pt[0], 2), round(pt[1], 2))


def _weld_nodes(node_fn, keyed: dict, tol: float = NODE_WELD_MM, joined: set = frozenset()) -> list:
    """Return zero-ohm edges joining copper nodes that are within `tol` of each
    other on the same layer. Pairs in `joined` (the two ends of one track) are
    left alone: an arc is stored as chords about 47 um long, under `tol`, and
    welding a chord's own ends shorted the arc out.

    A spatial hash keyed on `tol`-sized cells keeps this linear: each node only
    compares against the nine cells around it, so a 51k-segment board costs the
    same per node as a ten-segment one.
    """
    cells: dict = {}
    for key, pt in keyed.items():
        layer = key[0]
        cx, cy = int(pt[0] // tol), int(pt[1] // tol)
        cells.setdefault((layer, cx, cy), []).append((key, pt))
    out, seen = [], set()
    for (layer, cx, cy), items in cells.items():
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for okey, opt in cells.get((layer, cx + dx, cy + dy), ()):
                    for key, pt in items:
                        if key == okey:
                            continue
                        pair = (key, okey) if key < okey else (okey, key)
                        if pair in seen or pair in joined:
                            continue
                        if math.hypot(pt[0] - opt[0], pt[1] - opt[1]) <= tol:
                            seen.add(pair)
                            out.append((node_fn(key), node_fn(okey), 1e-9))
    return out


def analyze_net(board: PcbBoard, nid: int, ambient_c: float = 40.0) -> dict:
    """Build the resistor mesh of a net's real copper (segments + via barrels),
    then report: bottleneck segment, per-layer routed length, connectivity of
    the pads, and worst pad-to-pad effective resistance (nodal analysis).
    Zones are NOT meshed here: the resistance (Ω) covers the pads joined by tracks only, and nets with pours are
    never declared open. For a pad fed through a pour see `pour_net_resistance`."""
    segs = board.segments_of(nid)
    vias = board.vias_of(nid)
    pads = board.pads_of(nid)
    #  A net with a pour is exempted from the open check. Pour outlines are now
    #  parsed and DO join pads (see the conn_edges block below), but they are
    #  deliberately not trusted to prove the negative — that a pad the outline
    #  misses is really unreached.
    #
    #  The reason is thermal relief. KiCad's `filled_polygon` is the copper
    #  actually poured, so it has a clearance hole punched around every pad and
    #  the pad is joined across it by spokes; the pad centre therefore sits
    #  geometrically OUTSIDE the fill while being perfectly connected. Altium's
    #  polygon outline has the mirror problem — it is the region the user drew,
    #  which may cover copper that was never poured. Measured on the corpus,
    #  trusting containment to declare a net open pushed false alarms from 4.4%
    #  to 5.1%: precisely the wrong direction, and for a reason that is a
    #  property of pour rendering rather than of the boards.
    #
    #  So containment is used only to CONNECT (it can clear a false open, never
    #  create one), and the blanket exemption stays until spoke geometry is read
    #  too. Both halves fail safe.
    has_zone = nid in board.zone_nets
    has_zone_no_geom = has_zone

    # --- adjacency ----------------------------------------------------------- #
    nodes: dict = {}
    def node(key):
        if key not in nodes:
            nodes[key] = len(nodes)
        return nodes[key]

    edges = []   # (i, j, ohm)
    keyed: dict = {}          # node key -> exact point, for welding below
    via_hubs: list = []       # (hub node, via, layers) for annulus attachment
    temp = ambient_c + 20.0
    #  Track that lies on top of a pad or a via ring carries almost none of the
    #  current: the far wider pad copper under it does, so the current really
    #  enters the track at the pad's edge. Counting that stretch as track made
    #  every short net read high (median +11 % against a 2-D solve of the
    #  copper on KiCad's demo boards), so only the length outside pads and via
    #  rings counts.
    _joined: set = set()      # (node key, node key) of each track's two ends, never welded
    _pads_net = [(p, p.layer or ("B.Cu" if fp.layer == "B.Cu" else "F.Cu")) for fp, p in pads]
    #  A track lying wholly inside a wider track of the net (a re-route left on
    #  top of the old one) adds no copper: counted as a parallel path it halved
    #  that stretch (StickHub, J1 to JP1: -43 %). It keeps its connectivity but
    #  carries no current, provided both its ends meet other copper of the net.
    _ends: dict = {}
    for s in segs:
        for e_ in (s.start, s.end):
            _ends[_node_key(s.layer, e_)] = _ends.get(_node_key(s.layer, e_), 0) + 1
    _vkeys = {_node_key(ly, v.at) for v in vias for ly in board.copper_layers}
    _conn_only: list = []
    _adj: dict = {}           # node -> nodes one track away
    for s in segs:
        ka, kb = _node_key(s.layer, s.start), _node_key(s.layer, s.end)
        keyed.setdefault(ka, s.start)
        keyed.setdefault(kb, s.end)
        _joined.add((ka, kb) if ka < kb else (kb, ka))
        a, b = node(ka), node(kb)
        _adj.setdefault(a, set()).add(b)
        _adj.setdefault(b, set()).add(a)
        if _within_wider(s, segs) and all(_ends.get(k, 0) >= 2 or k in _vkeys for k in (ka, kb)):
            _conn_only.append((a, b, 0.0))
            continue
        covered = _covered_fraction(s, _pads_net, vias, segs=segs)
        tr = Trace(name="s", net="s", owner_subsystem="e",
                   width_mm=max(s.width_mm, 1e-4), copper_oz=board.copper_oz,
                   length_mm=max(s.length_mm * (1.0 - covered), 1e-3),
                   is_external=s.layer in ("F.Cu", "B.Cu"))
        edges.append((a, b, tr.resistance_ohm(temp_c=temp)))
    for v in vias:
        r = via_resistance_ohm(v.drill_mm, board.board_thickness_mm, temp_c=temp)
        hub = node(("via", round(v.at[0], 2), round(v.at[1], 2)))
        span = v.layers if len(v.layers) >= 2 else ("F.Cu", "B.Cu")
        layers = board.copper_layers if set(span) >= {"F.Cu", "B.Cu"} else span
        for ly in layers:
            kv = _node_key(ly, v.at)
            keyed.setdefault(kv, v.at)
            edges.append((hub, node(kv), r / 2.0))
        via_hubs.append((hub, v, set(layers)))
    # Weld coincident-but-not-identical endpoints before the pads are attached,
    # so a pad reaching either side of a joint reaches the whole net.
    edges.extend(_weld_nodes(node, keyed, joined=_joined))

    #  A track does not have to hit a via's exact centre to be connected to it.
    #  A via is an annulus of copper, and routers habitually end a track a little
    #  short of the middle: measured on a real 4-layer board, an inner-layer
    #  track stopped 97 um from a 350 um via — landing squarely ON the via pad,
    #  yet outside the 50 um endpoint weld, so the net split across layers and
    #  reported "copper open" while being perfectly routed. This was the single
    #  cause behind 49 of the corpus's remaining false alarms.
    #
    #  The rule is the same physical one the pads already use: anything within
    #  the via's own copper radius overlaps it. That radius is a fact from the
    #  file, not a tolerance to tune, so this cannot quietly weld a real gap —
    #  a track ending a via-radius away is touching the via.
    #
    #  The touching track joins the via's own node ON ITS LAYER, never the hub:
    #  the hub is the middle of the barrel, and joining a layer's copper straight
    #  to it shorted out the barrel resistance on every via (found by the SPICE
    #  benchmark: a top pad and a bottom pad on one via read 0.04 mΩ, not ~1 mΩ).
    for hub, v, vlayers in via_hubs:
        reach_v = max(v.size_mm, v.drill_mm) / 2.0 + NODE_WELD_MM
        for k, idx in list(nodes.items()):
            if k[0] in ("via", "pad", "zone") or k[0] not in vlayers:
                continue
            if math.hypot(k[1] - v.at[0], k[2] - v.at[1]) <= reach_v:
                own = node(_node_key(k[0], v.at))
                if own != idx:
                    edges.append((own, idx, 1e-5))

    # pads: connect to copper that overlaps the pad's own shape. A node is a
    # track end or a via's landing on one layer; its copper is a disc (half the
    # track width, or the via's ring), and it touches the pad where that disc
    # overlaps the pad outline: KiCad's own connectivity rule. An SMD pad joins
    # a via on its own layer, so a top pad and a bottom pad on one via still
    # pass through the barrel; only a through-hole pad joins the via's hub.
    pad_nodes = []
    node_pts = {idx: key for key, idx in nodes.items()}
    half_w: dict = {}
    for s_ in segs:
        for e_ in (s_.start, s_.end):
            kk_ = _node_key(s_.layer, e_)
            half_w[kk_] = max(half_w.get(kk_, 0.0), s_.width_mm / 2.0)
    for _hub, v_, vl_ in via_hubs:
        for ly in vl_:
            kk_ = _node_key(ly, v_.at)
            half_w[kk_] = max(half_w.get(kk_, 0.0), v_.size_mm / 2.0)
    #  Each pad is its own node, even when a footprint repeats a pad number (a
    #  tactile switch's two pin-1 legs, a connector's shield tabs). Merging them
    #  made the part's internal link a 0 Ω copper shortcut: KiCad's coldfire
    #  demo, D102 to LV101 through RST_SW101's two pin 1s, read 27 % low against
    #  a 2-D solve. The internal link still counts for connectivity (below).
    def _r_per_mm(width_mm):
        return Trace(name="s", net="s", owner_subsystem="e", width_mm=max(width_mm, 1e-4),
                     copper_oz=board.copper_oz, length_mm=1.0).resistance_ohm(temp_c=temp)
    for _pi, (fp, p) in enumerate(pads):
        key = ("pad", fp.ref, p.number, _pi)
        pn = node(key)
        reach = max(p.size) / 2.0 + 0.06
        pad_layer = p.layer or ("B.Cu" if fp.layer == "B.Cu" else "F.Cu")
        hit = False
        cand: list = []
        for k, idx in list(nodes.items()):
            if k[0] in ("pad", "zone"):
                continue
            if k[0] == "via":
                if not p.through:
                    continue
                margin = 0.0
            elif p.through or k[0] == pad_layer:
                margin = half_w.get(k, 0.0)
            else:
                continue
            pt_ = keyed.get(k, (k[1], k[2]))
            if _pad_overlaps(p, pt_, margin + 0.01):
                cand.append((idx, pt_, margin, k[0] == "via" or margin <= 0 or _pad_overlaps(p, pt_, 0.0)))
        #  Copper whose centre is on the pad joins it at 0 Ω. A track end whose
        #  centre is off the pad but whose copper touches it (KiCad's overlap
        #  rule) joins through the copper between its centre and the pad edge,
        #  and only if its track doesn't carry on to the pad anyway (no track
        #  from it reaches another node that touches this pad). Joining every
        #  such end at 0 Ω made a second path beside the track: an arc's chord
        #  ends sit every 40 um, several fall in that band, and the arc read
        #  low (StickHub, /LED6: -8 % against a 2-D solve).
        _cidx = {c[0] for c in cand}
        for idx, pt_, margin, inside in cand:
            if not inside and _adj.get(idx, set()) & _cidx:
                continue
            r_att = 1e-5
            if not inside:
                lo, hi = 0.0, margin + 0.01
                for _ in range(16):
                    mid = 0.5 * (lo + hi)
                    lo, hi = (lo, mid) if _pad_overlaps(p, pt_, mid) else (mid, hi)
                r_att = max(1e-5, hi * _r_per_mm(2.0 * margin))
            edges.append((pn, idx, r_att))
            hit = True
        # A pad does not have to land on a trace *endpoint*. Routing very often
        # runs a track straight across a pad and carries on — the pad sits
        # mid-segment, touching real copper, with no node anywhere near it. The
        # endpoint-only test above misses exactly that case and then reports the
        # net as "copper open", which on real boards was the single largest
        # source of false alarms. So: if no endpoint was in reach, test the pad
        # against each segment *body* on its own layer, and join it to that
        # segment's nearer end — topologically the correct connection, since the
        # trace either side of the pad is one continuous piece of copper.
        if not hit:
            for sg in segs:
                if not (p.through or sg.layer == pad_layer):
                    continue
                if _pt_seg_dist(p.at, sg.start, sg.end) > reach:
                    continue
                near = (sg.start
                        if math.dist(p.at, sg.start) <= math.dist(p.at, sg.end)
                        else sg.end)
                edges.append((pn, node(_node_key(sg.layer, near)), 1e-5))
                hit = True
        pad_nodes.append((fp.ref, p.number, pn, hit))

    # Two pads of the SAME footprint on the SAME net (a fuse, shunt, net-tie or
    # 0 Ω link sitting in the copper path) are bridged by the component body,
    # not by copper — join them with a nominal 10 mΩ so the net isn't declared
    # open and the IR-drop mesh sees the series element the current really
    # flows through.
    #  Only a two-terminal part is bridged this way. Two pins of an IC or a
    #  connector on one net are joined by the board's copper, not through the
    #  part: bridging them hid a 16 mm track loop behind a 10 mΩ link and
    #  under-read its resistance by 80 % (KiCad's video demo, U8 pins 60 and 70,
    #  in the SPICE benchmark).
    _two_pad = {fp_.ref for fp_ in board.footprints if len(fp_.pads) == 2}
    _by_fp = {}
    for ref, num, pn, hit in pad_nodes:
        if ref in _two_pad:
            _by_fp.setdefault(ref, []).append(pn)
    for _pns in _by_fp.values():
        for _a, _b in zip(_pns, _pns[1:]):
            edges.append((_a, _b, 0.010))

    # --- copper pours: connectivity ONLY, never resistance --------------------- #
    #  A pour is real copper and really does join pads — ignoring it is why a
    #  net poured to ground reads as a fistful of islands. But a pour is also a
    #  sheet, and any resistance the Doctor invented for it would be a guess
    #  that makes the IR drop look *smaller* than trace-only does. Small is the
    #  dangerous direction: it under-reports brown-out.
    #
    #  So pour edges live in their own list. `conn_edges` decides whether a net
    #  is open; `edges` — untouched here — remains the trace-only mesh the nodal
    #  solve uses, so every resistance the Doctor reports is still an
    #  upper bound on the real thing.
    conn_edges = list(edges) + _conn_only
    _first_pin: dict = {}
    for ref, num, pn, _hit in pad_nodes:          # same-numbered pads: joined inside the part
        _first_pin.setdefault((ref, num), pn)
        if _first_pin[(ref, num)] != pn:
            conn_edges.append((_first_pin[(ref, num)], pn, 0.0))
    for zi, z in enumerate(getattr(board, "zones", ()) or ()):
        if z.net != nid or len(z.outline) < 3:
            continue
        zhub = node(("zone", nid, zi))
        x0 = min(p_[0] for p_ in z.outline); x1 = max(p_[0] for p_ in z.outline)
        y0 = min(p_[1] for p_ in z.outline); y1 = max(p_[1] for p_ in z.outline)
        for k, idx in list(nodes.items()):
            if k[0] == "zone":
                continue
            if k[0] == "pad":
                fp_, p_ = next(((f2, q) for f2, q in pads
                                if f2.ref == k[1] and q.number == k[2]),
                               (None, None))
                if p_ is None:
                    continue
                pt = p_.at
                on_layer = p_.through or (p_.layer or
                    ("B.Cu" if fp_.layer == "B.Cu" else "F.Cu")) == z.layer
            else:
                pt = (k[1], k[2])
                on_layer = (k[0] == "via") or (k[0] == z.layer)
            if not on_layer or not (x0 <= pt[0] <= x1 and y0 <= pt[1] <= y1):
                continue
            if z.contains(pt, tol=NODE_WELD_MM):
                conn_edges.append((zhub, idx, 0.0))

    # --- connected components ------------------------------------------------- #
    parent = list(range(len(nodes)))
    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]; x = parent[x]
        return x
    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb
    for a, b, _ in conn_edges:
        union(a, b)
    pad_comps = {}
    for ref, num, pn, hit in pad_nodes:
        pad_comps.setdefault(find(pn), []).append(f"{ref}.{num}")
    open_groups = list(pad_comps.values()) if len(pad_comps) > 1 else []

    # --- worst pad-to-pad resistance (nodal analysis) -------------------------- #
    worst_r, worst_pair = None, None
    attached = [(ref, num, pn) for ref, num, pn, hit in pad_nodes if hit]
    if len(attached) >= 2 and len(nodes) <= _MAX_NODAL_NODES and not open_groups:
        n = len(nodes)
        L = np.zeros((n, n))
        for a, b, r in edges:
            g = 1.0 / max(r, 1e-9)
            L[a, a] += g; L[b, b] += g
            L[a, b] -= g; L[b, a] -= g
        #  Effective resistance by a grounded solve on each connected piece of the
        #  trace mesh, not a pseudo-inverse of the whole Laplacian: the mesh mixes
        #  1e-9 Ω welds with 50 mΩ tracks, and numpy's pinv lost a quarter of the
        #  answer on such meshes (KiCad's coldfire demo, /PST0: 135 mΩ for a
        #  plain chain of tracks summing to 180 mΩ; found against a 2-D solve).
        #  Two pads joined only through a pour have no trace path: that pair is
        #  skipped, since the trace-only resistance between them is undefined.
        try:
            comp = list(range(n))
            def _root(x):
                while comp[x] != x:
                    comp[x] = comp[comp[x]]; x = comp[x]
                return x
            for a, b, _ in edges:
                ra, rb = _root(a), _root(b)
                if ra != rb:
                    comp[ra] = rb
            sample = attached[:10]
            Xs: dict = {}
            for c in {_root(pn) for _r, _n, pn in sample}:
                idx = [k for k in range(n) if _root(k) == c]
                if len(idx) < 2:
                    continue
                pos = {k: m for m, k in enumerate(idx)}
                sub = L[np.ix_(idx, idx)][1:, 1:]          # ground the piece's first node
                X = np.zeros((len(idx), len(idx)))
                X[1:, 1:] = np.linalg.solve(sub, np.eye(len(idx) - 1))
                Xs[c] = (pos, X)
            for i in range(len(sample)):
                for j in range(i + 1, len(sample)):
                    a, b = sample[i][2], sample[j][2]
                    if _root(a) != _root(b) or _root(a) not in Xs:
                        continue
                    pos, X = Xs[_root(a)]
                    pa, pb = pos[a], pos[b]
                    r = float(X[pa, pa] + X[pb, pb] - 2 * X[pa, pb])
                    if worst_r is None or r > worst_r:
                        worst_r = r
                        worst_pair = (f"{sample[i][0]}.{sample[i][1]}",
                                      f"{sample[j][0]}.{sample[j][1]}")
        except np.linalg.LinAlgError:
            worst_r = None

    # --- bottleneck + per-layer stats ----------------------------------------- #
    bottleneck = min(segs, key=lambda s: s.width_mm) if segs else None
    per_layer = {}
    for s in segs:
        d = per_layer.setdefault(s.layer, {"len": 0.0, "min_w": s.width_mm})
        d["len"] += s.length_mm
        d["min_w"] = min(d["min_w"], s.width_mm)
    layer_transitions = len({s.layer for s in segs}) - 1 if segs else 0

    return {
        "net": board.net_name(nid), "nid": nid,
        "segments": len(segs), "vias": len(vias), "pads": len(pads),
        "has_zone": has_zone, "per_layer": per_layer,
        "bottleneck": bottleneck, "layer_transitions": max(layer_transitions, 0),
        "open_groups": open_groups if not has_zone_no_geom else [],
        "worst_r_ohm": worst_r, "worst_pair": worst_pair,
        "nodal_skipped": len(nodes) > _MAX_NODAL_NODES,
    }


# --------------------------------------------------------------------------- #
#  Current / voltage auto-assignment from the integration ledger
# --------------------------------------------------------------------------- #
_NET_HINTS = [
    (r"fan|pump|rad|cool", "cooling"),
    (r"brake|bspd|bl_", "brakes"),
    (r"inv|motor|hv|ts[_+-]|batt|accu|400", "powertrain"),
    (r"daq|sensor|log|imu|gps", "data-acquisition"),
    (r"ecu|vcu|lv|5v|3v3|12v|main|rail|pwr|vcc", "electrics"),
]
_HV_NAME = re.compile(r"hv|inv|ts[_+-]|batt|accu|400|motor", re.I)
_SIGNAL_NAME = re.compile(
    r"can|lin[_-]|spi|i2c|uart|usb|tx|rx|sda|scl|clk|sense|sig|adc|pwm|gpio|en[_-]|cs[_-]", re.I)
_SKIP_NAME = re.compile(r"^(gnd|agnd|dgnd|pgnd|earth|chassis)|unconnected|no_?connect|n\$?c$", re.I)


#  The subsystems a board's power nets are matched to, in the order the electrics module lists them.
LOAD_SUBSYSTEMS = tuple(dict.fromkeys(sub for _pat, sub in _NET_HINTS))


def subsystem_loads(ledger_dict: dict) -> list:
    """The electrical loads the integration ledger holds for each board subsystem: rows of {"subsystem",
    "peak_current_a" (A), "voltage_v" (V), "power_draw_w" (W)}, None where nothing is declared."""
    ifaces = (ledger_dict or {}).get("interfaces", {}) or {}
    names = list(LOAD_SUBSYSTEMS) + sorted(n for n, d in ifaces.items() if n not in LOAD_SUBSYSTEMS
                                           and any((d or {}).get(k) is not None
                                                   for k in ("peak_current_a", "voltage_v", "power_draw_w")))
    return [{"subsystem": n, **{k: (ifaces.get(n) or {}).get(k)
                                for k in ("peak_current_a", "voltage_v", "power_draw_w")}} for n in names]


def write_loads_to_ledger(ledger_dict: dict, rows: list, source: str = "electrics module") -> list:
    """Write the loads typed in the electrics module into the integration ledger (in place) and return the names of
    the subsystems that changed. Each row: {"subsystem", "peak_current_a" (A), "voltage_v" (V), "power_draw_w" (W)};
    a blank cell clears that field. When power is blank but current and voltage are both given, power is filled as
    V × I and the ledger's note says it was derived. A value that is not a finite number of zero or more is refused
    with ValueError naming the subsystem and the field, and nothing is written."""
    clean = []
    for r in rows:
        sub = str(r.get("subsystem") or "").strip()
        if not sub:
            continue
        vals = {}
        for k, unit in (("peak_current_a", "A"), ("voltage_v", "V"), ("power_draw_w", "W")):
            v = r.get(k)
            if v is None or (isinstance(v, str) and not v.strip()) or (isinstance(v, float) and math.isnan(v)):
                vals[k] = None
                continue
            try:
                v = float(v)
            except (TypeError, ValueError):
                raise ValueError(f"{sub}: {k} is not a number ({v!r})") from None
            if not math.isfinite(v) or v < 0:
                raise ValueError(f"{sub}: {k} must be a finite number of zero or more {unit}, got {v:g}")
            vals[k] = v
        derived = vals["power_draw_w"] is None and vals["peak_current_a"] is not None and vals["voltage_v"] is not None
        if derived:
            vals["power_draw_w"] = vals["peak_current_a"] * vals["voltage_v"]
        clean.append((sub, vals, derived))
    ifaces = ledger_dict.setdefault("interfaces", {})
    changed = []
    for sub, vals, derived in clean:
        d = dict(ifaces.get(sub) or {"name": sub, "is_estimate": False})   # typed by the engineer: declared
        before = {k: d.get(k) for k in vals}
        d.update(vals)
        if before != vals:
            note = (d.get("notes") or "").split(" [electrics:")[0].rstrip()
            d["notes"] = (note + f" [electrics: current, voltage{', power = V x I' if derived else ', power'} set in "
                          f"the {source}]").strip()
            ifaces[sub] = d
            changed.append(sub)
    return changed


def ledger_fingerprint(ledger=None) -> tuple:
    """
    A hashable summary of everything `auto_assign_net_currents` reads off the
    integration ledger. The UI caches the derived assignments for a session; this
    is what tells it the cache is stale, so a peak current bumped in the
    Integration tab reaches the diagnosis on the very next rerun instead of the
    session quietly finishing on the amps it started with.

    Only the declared peaks matter here — adding a note or a mass to the ledger
    must NOT throw away the electrical member's hand-edited net table.
    """
    if ledger is None:
        return ()
    out = []
    for name, iface in (getattr(ledger, "interfaces", {}) or {}).items():
        peak = getattr(iface, "peak_current_a", None)
        volt = getattr(iface, "voltage_v", None)
        if peak is None and volt is None:
            # Undeclared contributes nothing to the assignment, so its absence
            # must not appear as a change — otherwise declaring a mass in an
            # unrelated subsystem would throw away the net table.
            continue
        out.append((str(name), None if peak is None else float(peak), None if volt is None else float(volt)))
    return tuple(sorted(out, key=lambda t: t[0]))


def auto_assign_net_currents(board: PcbBoard, ledger=None) -> dict:
    """Map each routed net to (current_a, voltage_v, source_note) using name
    heuristics against the integration ledger's declared peak currents. Every
    guess is labelled; the UI lets the user overwrite all of it."""
    peaks, volts = {}, {}
    if ledger is not None:
        for sub_name, iface in getattr(ledger, "interfaces", {}).items():
            i = getattr(iface, "peak_current_a", None)
            if i:
                peaks[sub_name] = float(i)
            v = getattr(iface, "voltage_v", None)
            if v:
                volts[sub_name] = float(v)
    out = {}
    for nid in board.routed_net_ids():
        name = board.net_name(nid)
        if _SKIP_NAME.search(name or ""):
            continue
        if _SIGNAL_NAME.search(name):
            cur, volt = 0.05, 5.0
            src = "signal net by name — assumed 50 mA (edit me)"
            if _HV_NAME.search(name):
                volt = 400.0
            out[nid] = NetAssignment(net=name, voltage_v=volt).assume(cur, src)
            continue
        cur, volt, src = 1.0, 5.0, "assumed 1 A (no ledger match — edit me)"
        assumed = True
        matched = None
        for pat, sub in _NET_HINTS:
            if re.search(pat, name, re.I):
                matched = sub
                if sub in peaks:
                    cur = peaks[sub]
                    src = f"ledger: {sub} declared peak {cur:g} A"
                    assumed = False          # a real declared number
                else:
                    src = f"name matches {sub} but no ledger peak declared — assumed 1 A"
                break
        if _HV_NAME.search(name):
            volt = 400.0
        if matched in volts:                 # the subsystem's declared voltage beats the name guess
            volt = volts[matched]
        a = NetAssignment(net=name, voltage_v=volt)
        if assumed:
            a.assume(cur, src)               # a guess, and labelled as one
        else:
            a["current_a"], a["source"] = cur, src
        out[nid] = a
    return out


class NetAssignment(dict):
    """One net's electrical inputs, which knows whether its current was
    *declared* or merely *assumed*.

    This is a dict subclass rather than a plain dict for one reason: the
    distinction has to survive being written to carelessly. The Doctor reports
    MISSING (and offers no re-trace) for a net whose current it guessed, so if
    a caller sets `a["current_a"] = 8.0` and the `assumed` flag stays True, the
    net keeps reporting "current not declared" while visibly holding the number
    the user just gave it. That is a confusing bug that no amount of
    documentation reliably prevents, so setting a current here *is* declaring
    it — the two cannot drift apart.

    A plain dict built by hand elsewhere still works and counts as declared,
    because `assumed` is absent and absence means "someone stated this".
    """

    _CURRENT_KEYS = ("current_a",)

    def __setitem__(self, key, value):
        if key in self._CURRENT_KEYS:
            super().__setitem__("assumed", False)
        super().__setitem__(key, value)

    def update(self, *args, **kw):           # dict.update bypasses __setitem__
        for k, v in dict(*args, **kw).items():
            self[k] = v

    def setdefault(self, key, default=None):
        if key not in self:
            self[key] = default
        return self[key]

    @property
    def declared(self) -> bool:
        return not self.get("assumed", False)

    def assume(self, current_a: float, source: str):
        """Record a *guessed* current — the only way to set one without
        promoting it to a declaration. Used by the auto-assigner."""
        dict.__setitem__(self, "current_a", float(current_a))
        dict.__setitem__(self, "source", source)
        dict.__setitem__(self, "assumed", True)
        return self


def declare_net_current(assignments: dict, nid: int, current_a: float,
                        voltage_v=None, source: str = "declared by hand"):
    """State a net's peak current, turning a guess into an input.

    The explicit form, and the one to reach for when the intent matters. It is
    no longer the *only* safe form: `assignments[nid]["current_a"] = 8.0` now
    declares the net too, because `NetAssignment` clears `assumed` on any write
    to the current. Both roads lead to the same place, which is the point —
    correctness here should not depend on remembering which helper to call."""
    a = assignments.get(nid)
    if a is None:
        a = assignments[nid] = NetAssignment(net="", voltage_v=5.0)
    a["current_a"] = float(current_a)        # clears `assumed` by construction
    if voltage_v is not None:
        a["voltage_v"] = float(voltage_v)
    a["source"] = source
    return a


# --------------------------------------------------------------------------- #
#  IPC-2221 table B4 clearance (external, uncoated, ≤3050 m)
# --------------------------------------------------------------------------- #
_B4_EXTERNAL = [(15, 0.1), (30, 0.1), (50, 0.6), (100, 0.6), (150, 0.6),
                (170, 1.25), (250, 1.25), (300, 1.25), (500, 2.5)]


def clearance_required_mm(voltage_v: float) -> float:
    for vmax, c in _B4_EXTERNAL:
        if voltage_v <= vmax:
            return c
    return 2.5 + 0.005 * (voltage_v - 500.0)


def _within_wider(seg, segs, samples: int = 24) -> bool:
    """Does the track lie inside wider tracks of its net on its layer: both ends and at least 90 % of its
    centreline (sampled) within a wider track's copper?"""
    wide = [w for w in segs if w is not seg and w.layer == seg.layer and w.width_mm > seg.width_mm + 1e-6]
    if not wide:
        return False
    def _in(pt):
        return any(_pt_seg_dist(pt, w.start, w.end) <= w.width_mm / 2.0 for w in wide)
    if not (_in(seg.start) and _in(seg.end)):
        return False
    hit = sum(_in((seg.start[0] + (k + 0.5) / samples * (seg.end[0] - seg.start[0]),
                   seg.start[1] + (k + 0.5) / samples * (seg.end[1] - seg.start[1]))) for k in range(samples))
    return hit >= 0.9 * samples


def _covered_fraction(seg, pads, vias, samples: int = 24, segs=()) -> float:
    """Fraction (0–1, unitless) of a track's length whose centreline lies on a pad's copper (a pad on the track's
    layer, or a through-hole pad), inside a via's ring on that layer, or inside a wider track of the same net on
    that layer (a thin track running into a wide one), sampled at `samples` points. `pads` are (pad, copper
    layer) pairs, the layer being the pad's own or its footprint's side."""
    if seg.length_mm <= 0:
        return 0.0
    near = [p for p, ly in pads if (p.through or ly == seg.layer)
            and _pt_seg_dist(p.at, seg.start, seg.end) <= max(p.size) + 0.1]
    ring = [v for v in vias if _pt_seg_dist(v.at, seg.start, seg.end) <= v.size_mm / 2.0]
    wide = [w for w in segs if w is not seg and w.layer == seg.layer and w.width_mm > seg.width_mm + 1e-6
            and min(_pt_seg_dist(w.start, seg.start, seg.end), _pt_seg_dist(w.end, seg.start, seg.end),
                    _pt_seg_dist(seg.start, w.start, w.end), _pt_seg_dist(seg.end, w.start, w.end))
            <= w.width_mm / 2.0]
    if not near and not ring and not wide:
        return 0.0
    samples = min(max(samples, math.ceil(seg.length_mm / 0.025)), 4000)   # at least one sample per 25 um
    hit = 0
    #  A pad is tested across the track's whole width, not only at its centreline: a track running alongside a pad
    #  with its edge on the pad's copper hands its current to the pad there (kit-dev demo, C205 to U202: the track's
    #  edge overlaps U202's pad for 0.5 mm, and the centreline rule read it 34 % high against the 2-D solve). For a
    #  track entering a pad end-on, the cross-section first touches the pad where the centreline does, as before.
    #  Crediting only the share of the cross-section on the pad, or only its inner half, did worse on the demo
    #  boards (83 % and 92 % of nets within 2 %, against 93 %).
    ux, uy = seg.end[0] - seg.start[0], seg.end[1] - seg.start[1]
    ln = math.hypot(ux, uy) or 1.0
    nx, ny = -uy / ln * seg.width_mm / 2.0, ux / ln * seg.width_mm / 2.0
    across = (-1.0, -0.5, 0.0, 0.5, 1.0)
    for k in range(samples):
        f = (k + 0.5) / samples
        pt = (seg.start[0] + f * ux, seg.start[1] + f * uy)
        if any(math.hypot(pt[0] - v.at[0], pt[1] - v.at[1]) <= v.size_mm / 2.0 for v in ring) or \
                any(_pt_seg_dist(pt, w.start, w.end) <= w.width_mm / 2.0 for w in wide):
            hit += 1
            continue
        if near and any(_pad_overlaps(p, (pt[0] + a_ * nx, pt[1] + a_ * ny), 0.0) for p in near for a_ in across):
            hit += 1
    return hit / samples


def _pad_overlaps(pad, pt, margin_mm: float) -> bool:
    """Does a disc of radius margin_mm (mm) at pt touch the pad's copper? The pad is taken in its own frame
    (rotated by its angle in degrees): a circle, a stadium for an oval, otherwise its bounding rectangle."""
    w, h = (float(x) for x in pad.size)
    ox, oy = getattr(pad, "copper_offset", (0.0, 0.0)) or (0.0, 0.0)
    dx, dy = pt[0] - pad.at[0] - ox, pt[1] - pad.at[1] - oy
    th = math.radians(pad.angle_deg or 0.0)
    lx = dx * math.cos(th) - dy * math.sin(th)
    ly = dx * math.sin(th) + dy * math.cos(th)
    shape = (pad.shape or "").lower()
    if shape in ("circle", "round") and abs(w - h) < 1e-9:
        return math.hypot(lx, ly) <= w / 2.0 + margin_mm
    if shape in ("oval", "round"):
        if w >= h:
            seg = (max(abs(lx) - (w - h) / 2.0, 0.0), ly)
            return math.hypot(*seg) <= h / 2.0 + margin_mm
        seg = (lx, max(abs(ly) - (h - w) / 2.0, 0.0))
        return math.hypot(*seg) <= w / 2.0 + margin_mm
    # a rectangle; a roundrect's corners are rounded with KiCad's default radius, a quarter of the shorter side. The
    # 1-D model keeps this (its benchmarks B5-B6 were scored with it); the 2-D sheet solve uses the file's own ratio.
    rad = 0.25 * min(w, h) if shape == "roundrect" else 0.0
    ex = max(abs(lx) - (w / 2.0 - rad), 0.0)
    ey = max(abs(ly) - (h / 2.0 - rad), 0.0)
    return math.hypot(ex, ey) <= rad + margin_mm


def _pt_seg_dist(p, a, b) -> float:
    """Distance from a point to a line segment (not to its endpoints)."""
    ax, ay = a; bx, by = b; px, py = p
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 <= 0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _seg_seg_dist_2d(p1, p2, q1, q2) -> float:
    """Min distance between two 2-D segments (numpy-free fast path)."""
    def pt_seg(p, a, b):
        ax, ay = a; bx, by = b; px, py = p
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        if L2 <= 0:
            return math.hypot(px - ax, py - ay)
        t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
        return math.hypot(px - (ax + t * dx), py - (ay + t * dy))
    # cheap exact-enough: endpoints against opposite segments
    return min(pt_seg(p1, q1, q2), pt_seg(p2, q1, q2),
               pt_seg(q1, p1, p2), pt_seg(q2, p1, p2))


# --------------------------------------------------------------------------- #
#  Differential pair detection
# --------------------------------------------------------------------------- #
_PAIR_SUFFIX = [("_H", "_L"), ("_P", "_N"), ("+", "-"), ("H", "L")]


def find_diff_pairs(board: PcbBoard):
    """Detect H/L and P/N pairs by net-name convention (CAN_H/CAN_L, USB_P/…)."""
    byname = {board.net_name(n): n for n in board.routed_net_ids()}
    pairs, used = [], set()
    for name, nid in byname.items():
        for hi, lo in _PAIR_SUFFIX:
            if name.upper().endswith(hi) and len(name) > len(hi):
                base = name[:-len(hi)]
                mate = next((m for m in byname
                             if m.upper() == (base + lo).upper() and m != name), None)
                if mate and name not in used and mate not in used:
                    pairs.append((base.rstrip("_") or name, nid, byname[mate]))
                    used.add(name); used.add(mate)
                break
    return pairs


# --------------------------------------------------------------------------- #
#  The diagnosis — every real-life failure mode, with the fix attached
# --------------------------------------------------------------------------- #
@dataclass
class TraceFix:
    """One executable fix: widen a specific segment to a specific width."""
    net: str
    nid: int
    seg_index: int          # index into board.segments
    old_width_mm: float
    new_width_mm: float
    layer: str
    auto: bool = True       # False for diff-pair members (prescription only)
    note: str = ""


@dataclass
class ViaFix:
    """A recommended stitching via (never applied unless the user picks it): a via of the net's own size beside an
    existing one, joined to it by a short track on every layer it spans. Positions in board mm."""
    net: str
    nid: int
    at: tuple
    size_mm: float
    drill_mm: float
    layers: tuple
    joins: tuple            # the existing via's position it is stitched to (mm)
    stub_width_mm: float
    note: str = ""


@dataclass
class DoctorReport:
    findings: list = field(default_factory=list)     # [Finding]
    fixes: list = field(default_factory=list)        # [TraceFix]
    #  Recommended only. The Doctor never writes these on its own: the engineer picks which go in (plan of record).
    via_recommendations: list = field(default_factory=list)   # [ViaFix]
    net_reports: dict = field(default_factory=dict)  # nid -> analyze_net dict

    def counts(self):
        c = {"fail": 0, "warning": 0, "missing": 0, "info": 0, "ok": 0}
        for f in self.findings:
            c[f.severity.value] = c.get(f.severity.value, 0) + 1
        return c

    def summary(self) -> str:
        c = self.counts()
        return (f"{c['fail']} FAIL · {c['warning']} WARN · {c['missing']} MISSING "
                f"· {c['info']} INFO · {c['ok']} OK — "
                f"{sum(1 for x in self.fixes if x.auto)} segment width fixes ready to apply")


def diagnose(board: PcbBoard, assignments: dict,
             rail_v: float = 5.0, brownout_v: float = 4.5,
             ambient_c: float = 40.0, max_temp_c: float = 105.0,
             fuse_sf: float = 2.0, dT_budget_c: float = 20.0,
             copper_finish: float = COPPER_FINISH_FACTOR, pour_max_cells: int | None = None) -> DoctorReport:
    """Run every check against the parsed board + the per-net current/voltage
    assignments. Returns findings (plain-language WHY + numeric FIX) and the
    executable width-fix list."""
    rep = DoctorReport()
    F = rep.findings
    #  The voltages the user declared, kept on the board so a widening is checked against the clearance an HV net
    #  needs (IPC-2221 B4), not only the LV default.
    board.net_voltage_v = {nid: float(a.get("voltage_v", 0) or 0) for nid, a in assignments.items()}

    def hint(nid):
        a = assignments.get(nid, {})
        return a.get("source", "")

    # ---------- per-net copper physics ---------------------------------------- #
    hot_nets = {}   # nid -> (temp_c, worst_segment) for component proximity later
    for nid, a in assignments.items():
        cur = float(a.get("current_a", 0.0) or 0.0)
        name = a.get("net", board.net_name(nid))
        segs = board.segments_of(nid)
        if not segs:
            continue
        nr = analyze_net(board, nid, ambient_c=ambient_c)
        rep.net_reports[nid] = nr

        # -- copper OPEN: the board is dead on arrival --------------------------- #
        if nr["open_groups"]:
            groups = " | ".join(", ".join(g[:6]) for g in nr["open_groups"][:4])
            F.append(Finding(
                check=f"copper open — {name}", severity=Severity.FAIL,
                subsystems=["electrics"],
                message=(f"Net '{name}' is routed but its pads are NOT all joined by "
                         f"copper — pad groups [{groups}] have no trace/via path "
                         f"between them and there is no pour on this net. DRC and "
                         f"the schematic both look fine; in real life the circuit "
                         f"is simply open and the board does nothing. FIX: route "
                         f"the missing connection between those pad groups (the "
                         f"rats-nest line your EDA is still showing).")))

        if cur <= 0:
            continue

        # -- no declared current? then there is no verdict to give ---------------- #
        #  Assigning a default and then failing the net against that default is a
        #  guess wearing the costume of a finding. On a real board most nets are
        #  unmatched, so it buries the three findings that are real under twenty
        #  that are invented — and a member who sees a working board come back
        #  covered in FAILs learns to close the tab. MISSING says the true thing:
        #  the board may be fine, the *input* is absent. Geometry-only checks
        #  (copper open, clearance, diff-pair) ran above and still stand, because
        #  they do not depend on the current at all.
        if a.get("assumed"):
            bn_a = min(segs, key=lambda sg: sg.width_mm)
            F.append(Finding(
                check=f"current not declared — {name}", severity=Severity.MISSING,
                subsystems=["electrics"],
                message=(f"No peak current is declared for '{name}', so its copper "
                         f"cannot be judged: its narrowest segment is "
                         f"{bn_a.width_mm:.2f} mm on {bn_a.layer}, which is fine "
                         f"for {trace_ampacity_a(bn_a.width_mm, dT_budget_c, board.copper_oz, bn_a.layer in ('F.Cu', 'B.Cu')):.2f} A "
                         f"and not fine above it. FIX: declare this net's peak in "
                         f"the integration ledger, or type its current into the "
                         f"table above — the Doctor will not guess a number and "
                         f"then fail your board against its own guess."))) 
            continue

        # -- ampacity at the bottleneck segment ---------------------------------- #
        bn = nr["bottleneck"]

        def _req_w(external: bool, oz: float) -> float:
            """Width that satisfies BOTH the IPC-2221 heating budget and the
            Onderdonk fusing safety factor (fusing is pure cross-section)."""
            w_heat = required_width_mm(cur, dT_budget_c, oz, external)
            c = math.sqrt(math.log10((1083.0 - ambient_c) / (234.0 + ambient_c)
                                     + 1.0) / (33.0 * 10.0))
            a_fuse_mil2 = (fuse_sf * cur) / c
            t_mm = oz * OZ_TO_UM / 1000.0
            w_fuse = (a_fuse_mil2 / MM2_TO_MIL2) / t_mm
            return max(w_heat, w_fuse)

        req_w = _req_w(bn.layer in ("F.Cu", "B.Cu"), board.copper_oz)
        eq = Trace(name=name, net=name, owner_subsystem="electrics",
                   width_mm=bn.width_mm, copper_oz=board.copper_oz,
                   length_mm=bn.length_mm,
                   is_external=bn.layer in ("F.Cu", "B.Cu"))
        rise = eq.temp_rise_c(cur)
        t_run = ambient_c + rise
        fuse_i = eq.fusing_current_a(ambient_c=ambient_c)
        seg_idx = board.segments.index(bn)
        is_pair_member = any(nid in (h, l) for _, h, l in find_diff_pairs(board))

        if t_run > max_temp_c or bn.width_mm < req_w * 0.999:
            sev = Severity.FAIL if t_run > max_temp_c else Severity.WARN
            F.append(Finding(
                check=f"trace ampacity — {name}", severity=sev,
                subsystems=["electrics"],
                message=(f"Bottleneck segment on '{name}' ({bn.layer}) is "
                         f"{bn.width_mm:.2f} mm wide; at the assigned {cur:g} A "
                         f"({hint(nid)}) it runs ≈{t_run:.0f} °C "
                         f"(rise {rise:.0f} °C over {ambient_c:g} °C ambient) — "
                         f"{'past' if t_run > max_temp_c else 'eating into'} the "
                         f"{max_temp_c:g} °C derate ceiling. This is the failure "
                         f"that passes every simulation: the schematic has no "
                         f"widths, DRC has no amps. FIX: widen to "
                         f"≥{req_w:.2f} mm (IPC-2221 heating + Onderdonk fusing "
                         f"at {fuse_sf:g}×, {board.copper_oz:g} oz) or move the "
                         f"net to 2 oz copper "
                         f"(≥{_req_w(bn.layer in ('F.Cu','B.Cu'), 2.0):.2f} mm)."),
                detail={"required_width_mm": req_w, "run_temp_c": t_run}))
            # widen ALL under-width segments of this net, not just the bottleneck
            for i, s in enumerate(board.segments):
                if s.net != nid:
                    continue
                r = _req_w(s.layer in ("F.Cu", "B.Cu"), board.copper_oz)
                if s.width_mm < r * 0.999:
                    w_new = math.ceil(r * 20) / 20.0                  # round up to 0.05
                    clash = None if is_pair_member else widening_conflict(board, i, w_new)
                    rep.fixes.append(TraceFix(
                        net=name, nid=nid, seg_index=i, old_width_mm=s.width_mm,
                        new_width_mm=w_new,
                        layer=s.layer, auto=not is_pair_member and clash is None,
                        note=("diff-pair member — widening changes impedance; "
                              "prescribed only, use copper weight / paralleling"
                              if is_pair_member else
                              f"widening to {w_new:g} mm would leave {max(clash[1], 0):.2f} mm to "
                              f"'{clash[0]}' (needs {clash[2]:g} mm): not applied — reroute "
                              f"this segment, or move the net to 2 oz copper" if clash else "")))

        # -- fusing margin -------------------------------------------------------- #
        if fuse_i < fuse_sf * cur:
            F.append(Finding(
                check=f"fusing margin — {name}", severity=Severity.FAIL,
                subsystems=["electrics"],
                message=(f"'{name}' bottleneck fuses (physically melts, Onderdonk "
                         f"10 s) at {fuse_i:.1f} A — under the required "
                         f"{fuse_sf:g}× safety factor on {cur:g} A "
                         f"({fuse_sf*cur:.1f} A). A stalled fan or a short below "
                         f"the fuse rating opens this trace like a fuse wire, and "
                         f"the 'random component failure' the team chases later is "
                         f"actually vaporised copper. FIX covered by the ampacity "
                         f"width above; verify the upstream fuse trips below "
                         f"{fuse_i:.1f} A.")))

        # -- via bottleneck --------------------------------------------------------- #
        if nr["layer_transitions"] > 0:
            vlist = board.vias_of(nid)
            if not vlist:
                F.append(Finding(
                    check=f"via bottleneck — {name}", severity=Severity.WARN,
                    subsystems=["electrics"],
                    message=(f"'{name}' changes layer but no vias were found on the "
                             f"net — the transition is probably through a pad "
                             f"barrel; confirm its plating carries {cur:g} A.")))
            else:
                d = min(v.drill_mm for v in vlist)
                cap1 = via_ampacity_a(d, dT_budget_c)
                need = vias_needed(cur, d, dT_budget_c)
                if len(vlist) < need:
                    F.append(Finding(
                        check=f"via bottleneck — {name}", severity=Severity.FAIL,
                        subsystems=["electrics"],
                        message=(f"'{name}' crosses layers on {len(vlist)} via(s) "
                                 f"(⌀{d:g} mm drill ≈ {cap1:.1f} A each at "
                                 f"ΔT {dT_budget_c:g} °C) but carries {cur:g} A. "
                                 f"The wide trace is fine — the barrel is the "
                                 f"choke point; it overheats, the solder joint "
                                 f"fatigues, and the board dies weeks later with "
                                 f"no visible damage. FIX: stitch the transition "
                                 f"with ≥{need} vias (add "
                                 f"{need - len(vlist)} more) or use a "
                                 f"⌀0.6 mm drill.")))
                stub = max(min(sg.width_mm for sg in segs), 0.25)
                rep.via_recommendations += recommend_stitching_vias(board, nid, need - len(vlist), stub)

        # -- true IR drop → brown-out ------------------------------------------------ #
        volt = float(a.get("voltage_v", rail_v) or rail_v)
        #  On a net with a pour the 1-D model sees only the pads its tracks join: a pad fed through the pour is
        #  not in its worst pair, and that pair can be many times the tracked one (40 x on one benchmark net,
        #  plan addendum 13). So the pour is solved too, and where it cannot be the report says what was left out.
        pour = pour_net_resistance(board, nid, pour_max_cells) if (nr["has_zone"] and volt <= 60.0) else None
        r_est, pr, r_up = nr["worst_r_ohm"], nr["worst_pair"] or ("?", "?"), 0.0
        if r_est is not None:
            ref_ = refined_net_resistance(board, nid, nr)
            r_up = r_est * NET_R_UPPER_FACTOR                      # never below the bound the 1-D model gave
            if ref_["source"] == "sheet solve":
                r_up = max(r_up, ref_["r_ohm"] * SHEET_UPPER_FACTOR)
        if pour is not None:
            p_up = pour["r_ohm"] * (SHEET_UPPER_FACTOR if pour["converged"] else POUR_UNVERIFIED_FACTOR)
            if p_up > r_up:
                r_est, pr, r_up = pour["r_ohm"], tuple(pour["pair"]), p_up
        if nr["has_zone"] and volt <= 60.0 and pour is None:
            F.append(Finding(
                check=f"IR drop — {name}", severity=Severity.WARN,
                subsystems=["electrics"],
                message=(f"'{name}' has a copper pour too large to solve "
                         f"{'even with large pours switched on' if pour_max_cells else 'in the quick check'}, so the "
                         f"voltage drop to pads fed through the pour is NOT checked"
                         + (f"; the figure below covers only the pads its tracks join." if r_est is not None
                            else " and none of its pads are joined by tracks alone.")
                         + (" Check the drop to the farthest load in your PCB tool's DC analysis." if pour_max_cells
                            else " Switch on 'Solve large pours' to check it here (minutes per pour), or check the "
                                 "drop to the farthest load in your PCB tool's DC analysis."))))
        if r_est is not None and volt <= 60.0:
            r_up /= max(copper_finish, 1e-3)
            drop = cur * r_up
            #  Each net is judged at its own voltage: a 12 V fan feed against 12 V, not against the 5 V logic rail.
            #  The board's brown-out is kept as the same fraction of nominal (4.5 V on 5 V is 90 %, so 10.8 V on 12 V).
            net_rail = volt if volt > 0 else rail_v
            net_brownout = brownout_v * net_rail / rail_v if rail_v > 0 else brownout_v
            scaled = (f" ({brownout_v:g} V on {rail_v:g} V, the board's brown-out fraction, applied to this "
                      f"{net_rail:g} V net)" if net_rail != rail_v else "")
            delivered = net_rail - drop
            zone_note = ("" if not nr["has_zone"] else " (through the pour)" if pour is not None and
                         r_est == pour["r_ohm"] else " (pads joined by tracks; the pour was not solved)"
                         if pour is None else " (the tracked pads; the pour's worst pair is lower)")
            if delivered < net_brownout:
                F.append(Finding(
                    check=f"IR drop / brown-out — {name}", severity=Severity.FAIL,
                    subsystems=["electrics"],
                    message=(f"Nodal analysis of the routed copper on '{name}' "
                             f"gives {r_est*1000:.0f} mΩ between "
                             f"{pr[0]} and {pr[1]}{zone_note}, taken as up to "
                             f"{r_up*1000:.0f} mΩ with the model's margin and copper up to "
                             f"{100*(1-copper_finish):.0f} % thin from the fab. At {cur:g} A that "
                             f"drops {drop:.2f} V — the {net_rail:g} V rail arrives "
                             f"at {delivered:.2f} V, below the {net_brownout:.3g} V "
                             f"brown-out{scaled}. On track this is the ECU resetting "
                             f"exactly when the fans and brake light fire "
                             f"together — invisible on the bench at idle current. "
                             f"FIX: apply the width fixes below, add a pour, or "
                             f"feed {pr[1]} directly from the rail.")))
            elif drop > 0.25 * (net_rail - net_brownout):
                F.append(Finding(
                    check=f"IR drop — {name}", severity=Severity.WARN,
                    subsystems=["electrics"],
                    message=(f"'{name}' drops {drop:.2f} V at {cur:g} A over its "
                             f"real routed mesh (up to {r_up*1000:.0f} mΩ, "
                             f"{pr[0]}→{pr[1]}){zone_note} — "
                             f"{int(drop / (net_rail - net_brownout) * 100)}% of the brown-out "  # rounded down: 100 % only when it is all gone
                             f"margin gone in copper before any connector or "
                             f"harness drop is counted.{scaled}")))
        elif nr.get("nodal_skipped"):
            F.append(Finding(
                check=f"IR drop — {name}", severity=Severity.INFO,
                subsystems=["electrics"],
                message=(f"'{name}' has too much copper to mesh here "
                         f"(> {_MAX_NODAL_NODES} nodes) — IR drop not computed "
                         f"rather than guessed.")))

        if rise > 20.0:
            hot_nets[nid] = (t_run, bn)

    # ---------- HV clearance (IPC-2221 B4) — the wet-track arc ------------------ #
    hv_ids = [nid for nid, a in assignments.items()
              if float(a.get("voltage_v", 0) or 0) > 60.0]
    seg_cap = 6000
    for hv in hv_ids:
        v = float(assignments[hv]["voltage_v"])
        req = clearance_required_mm(v)
        hv_segs = board.segments_of(hv)[:seg_cap]
        worst = None
        for s in board.segments[:seg_cap]:
            if s.net == hv or s.net not in board.nets:
                continue
            for h in hv_segs:
                if h.layer != s.layer:
                    continue
                d = _seg_seg_dist_2d(h.start, h.end, s.start, s.end)
                gap = d - (h.width_mm + s.width_mm) / 2.0
                if worst is None or gap < worst[0]:
                    worst = (gap, s, h)
        if worst and worst[0] < req:
            gap, s, h = worst
            F.append(Finding(
                check=f"HV clearance — {board.net_name(hv)}",
                severity=Severity.FAIL if gap < req * 0.6 else Severity.WARN,
                subsystems=["electrics", "powertrain"],
                message=(f"'{board.net_name(hv)}' ({v:g} V) runs "
                         f"{max(gap,0):.2f} mm edge-to-edge from "
                         f"'{board.net_name(s.net)}' on {s.layer} — IPC-2221 B4 "
                         f"external/uncoated wants ≥{req:.2f} mm at {v:g} V. It "
                         f"passes a default DRC (set for LV) and works dry; add "
                         f"humidity, flux residue or dust and it tracks/arcs — "
                         f"the classic 'the board just died at the wet event'. "
                         f"FIX: open the gap to ≥{req:.2f} mm, slot the board, "
                         f"or conformal-coat and re-rate to the coated column.")))

    # ---------- differential pairs ---------------------------------------------- #
    pairs = find_diff_pairs(board)
    for base, hid, lid in pairs:
        hs, ls = board.segments_of(hid), board.segments_of(lid)
        if not hs or not ls:
            continue
        lh = sum(s.length_mm for s in hs)
        ll = sum(s.length_mm for s in ls)
        skew = abs(lh - ll)
        wh = {round(s.width_mm, 3) for s in hs}
        wl = {round(s.width_mm, 3) for s in ls}
        if skew > max(2.0, 0.05 * max(lh, ll)):
            F.append(Finding(
                check=f"diff pair skew — {base}", severity=Severity.WARN,
                subsystems=["electrics", "data-acquisition"],
                message=(f"Pair '{base}': H routed {lh:.1f} mm, L routed "
                         f"{ll:.1f} mm — {skew:.1f} mm skew "
                         f"({skew/max(lh,ll)*100:.0f}%). The pair's noise "
                         f"immunity is common-mode rejection; unequal lengths "
                         f"convert inverter noise to differential error and CAN "
                         f"starts dropping frames under load only. FIX: "
                         f"length-match to within 2 mm (serpentine the short "
                         f"side).")))
        if len(wh | wl) > 1:
            F.append(Finding(
                check=f"diff pair width — {base}", severity=Severity.WARN,
                subsystems=["electrics"],
                message=(f"Pair '{base}' mixes conductor widths "
                         f"{sorted(wh | wl)} mm — impedance steps at every width "
                         f"change reflect edges; keep one width on both "
                         f"conductors.")))
        # -- aggressor proximity on real geometry -------------------------------- #
        for hv in hv_ids:
            for hseg in board.segments_of(hv)[:400]:
                for pseg in (hs + ls)[:400]:
                    if hseg.layer != pseg.layer:
                        continue
                    a = np.array([hseg.start, hseg.end], float)
                    b = np.array([pseg.start, pseg.end], float)
                    dmin = min_parallel_distance_mm(a, b)
                    if dmin is not None and dmin < 2.0:
                        run = parallel_run_length_mm(a, b, within_mm=2.0)
                        F.append(Finding(
                            check=f"HV coupling — {base}",
                            severity=Severity.WARN,
                            subsystems=["electrics", "powertrain"],
                            message=(f"Pair '{base}' runs {dmin:.2f} mm from HV "
                                     f"net '{board.net_name(hv)}' on "
                                     f"{pseg.layer} (≈{run:.0f} mm exposed). The "
                                     f"coupled voltage needs a field solver — "
                                     f"not computed, not invented — but at this "
                                     f"gap the screening budget is blown. FIX: "
                                     f"reroute ≥2 mm away or drop a grounded "
                                     f"guard trace between them."),
                        ))
                        break
                else:
                    continue
                break

    # ---------- component-level real-life derating ------------------------------- #
    fuse_re = re.compile(r"([\d.]+)\s*A", re.I)
    for fp in board.footprints:
        r0 = fp.ref[:1].upper() if fp.ref else "?"
        # fuses: marked rating vs the net current through them
        if r0 == "F":
            m = fuse_re.search(fp.value or "")
            if m:
                rating = float(m.group(1))
                through = max((float(assignments.get(p.net, {}).get("current_a", 0) or 0)
                               for p in fp.pads), default=0.0)
                if through > rating:
                    F.append(Finding(
                        check=f"component — {fp.ref}", severity=Severity.FAIL,
                        subsystems=["electrics"],
                        message=(f"Fuse {fp.ref} is marked {rating:g} A but its "
                                 f"net carries {through:g} A — it blows in normal "
                                 f"operation, the exact 'component failed even "
                                 f"though the design was fine' report. FIX: fit "
                                 f"≥{math.ceil(through*1.25)} A (25% headroom) "
                                 f"and re-check the trace fusing margin above.")))
        # anything parked on hot copper — distance to the *whole* hot net
        for nid, (t_run, bn) in hot_nets.items():
            d = min((_seg_seg_dist_2d(sg.start, sg.end, fp.at, fp.at)
                     for sg in board.segments_of(nid)), default=1e9)
            if d < 3.0:
                if r0 == "C":
                    F.append(Finding(
                        check=f"component — {fp.ref}", severity=Severity.WARN,
                        subsystems=["electrics"],
                        message=(f"Capacitor {fp.ref} ({fp.value or 'value ?'}) "
                                 f"sits {d:.1f} mm from the '{board.net_name(nid)}' "
                                 f"hot spot (≈{t_run:.0f} °C copper). If it's an "
                                 f"electrolytic, life halves every +10 °C — a cap "
                                 f"rated 2000 h @ 105 °C dies mid-season, and the "
                                 f"post-mortem blames 'a bad cap'. FIX: move it "
                                 f">5 mm away, or fix the trace width so the "
                                 f"copper never gets hot.")))
                elif r0 in ("U", "Q", "D"):
                    F.append(Finding(
                        check=f"component — {fp.ref}", severity=Severity.INFO,
                        subsystems=["electrics"],
                        message=(f"{fp.ref} ({fp.value or ''}) is {d:.1f} mm from "
                                 f"the ≈{t_run:.0f} °C copper of "
                                 f"'{board.net_name(nid)}' — add that rise to its "
                                 f"junction-temperature budget before trusting "
                                 f"the datasheet derating curve.")))
        # connectors on high-current nets: pin rating is the silent limit
        if r0 in ("J", "P") or fp.ref[:2].upper() == "CN":
            worst = max((float(assignments.get(p.net, {}).get("current_a", 0) or 0)
                         for p in fp.pads), default=0.0)
            if worst >= 3.0:
                F.append(Finding(
                    check=f"component — {fp.ref}", severity=Severity.INFO,
                    subsystems=["electrics"],
                    message=(f"Connector {fp.ref} carries a {worst:g} A net; a "
                             f"typical 2.54 mm header pin is rated ~3 A and "
                             f"derates hot. The pin, not the trace, becomes the "
                             f"fuse. Verify the contact rating or split the "
                             f"current across paralleled pins.")))

    if not F:
        F.append(Finding(check="board", severity=Severity.OK,
                         subsystems=["electrics"],
                         message=("Every routed net clears ampacity, fusing, via, "
                                  "IR-drop, clearance and pairing screens at the "
                                  "assigned currents — commit the currents in the "
                                  "table (they drive everything) and go to fab.")))
    return rep


# --------------------------------------------------------------------------- #
#  The auto-fix: rewrite widths in the original file, verify, report
# --------------------------------------------------------------------------- #
#  Copper-to-copper spacing (mm) a widened trace must keep to every other net on its layer. KiCad's own default
#  clearance; a board's rules may be tighter, never looser for a fix this tool makes on its own.
FIX_MIN_CLEARANCE_MM = 0.2


def _seg_seg_mm(a0, a1, b0, b1) -> float:
    """Shortest distance (mm) between two line segments in the plane."""
    def _pt(p, q0, q1):
        dx, dy = q1[0] - q0[0], q1[1] - q0[1]
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 <= 0 else max(0.0, min(1.0, ((p[0] - q0[0]) * dx + (p[1] - q0[1]) * dy) / L2))
        return math.hypot(p[0] - q0[0] - t * dx, p[1] - q0[1] - t * dy)

    def _cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    d1, d2, d3, d4 = _cross(b0, b1, a0), _cross(b0, b1, a1), _cross(a0, a1, b0), _cross(a0, a1, b1)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)):
        return 0.0
    return min(_pt(a0, b0, b1), _pt(a1, b0, b1), _pt(b0, a0, a1), _pt(b1, a0, a1))


def copper_conflict(board: PcbBoard, nid: int, a0, a1, half_width_mm: float, layer: str,
                    min_clear_mm: float = FIX_MIN_CLEARANCE_MM, skip_segment: int = -1):
    """The first piece of another net's copper on `layer` that a capsule from a0 to a1 (mm) of half width
    `half_width_mm` (mm) would come closer to than it may, as (net name, gap mm, clearance needed mm), or None."""
    volts = getattr(board, "net_voltage_v", {}) or {}
    rules = getattr(board, "clearance_mm", {}) or {}

    def _need(other_net):
        v = max(abs(volts.get(nid, 0.0)), abs(volts.get(other_net, 0.0)))
        lv = max(min_clear_mm, rules.get(nid, 0.0), rules.get(other_net, 0.0))
        return max(lv, clearance_required_mm(v)) if v > 60.0 else lv

    for i, o in enumerate(board.segments):
        if i == skip_segment or o.net == nid or o.layer != layer:
            continue
        gap = _seg_seg_mm(a0, a1, o.start, o.end) - half_width_mm - o.width_mm / 2.0
        if gap < _need(o.net) - 1e-6:
            return board.net_name(o.net), gap, _need(o.net)
    for fp in board.footprints:
        for p in fp.pads:
            if p.net == nid or not (p.through or (p.layer or ("B.Cu" if fp.layer == "B.Cu" else "F.Cu")) == layer):
                continue
            gap = _seg_seg_mm(a0, a1, p.at, p.at) - half_width_mm - math.hypot(p.size[0], p.size[1]) / 2.0
            if gap < _need(p.net) - 1e-6:
                return (board.net_name(p.net) if p.net else f"{fp.ref}.{p.number}"), gap, _need(p.net)
    for v in board.vias:
        if v.net == nid or layer not in v.layers and not (v.layers == ("F.Cu", "B.Cu")):
            continue
        gap = _seg_seg_mm(a0, a1, v.at, v.at) - half_width_mm - v.size_mm / 2.0
        if gap < _need(v.net) - 1e-6:
            return board.net_name(v.net), gap, _need(v.net)
    for z in board.zones:                                  # another net's pour: its outline edges
        if z.net == nid or z.layer != layer or len(z.outline) < 3:
            continue
        pts = z.outline
        for q0, q1 in zip(pts, pts[1:] + pts[:1]):
            gap = _seg_seg_mm(a0, a1, q0, q1) - half_width_mm
            if gap < _need(z.net) - 1e-6:
                return board.net_name(z.net), gap, _need(z.net)
    return None


def recommend_stitching_vias(board: PcbBoard, nid: int, n_more: int, stub_width_mm: float) -> list:
    """Up to `n_more` stitching vias for net `nid`, each beside one of its vias, joined by a stub on every layer the
    via spans and clear of all other copper (`copper_conflict`). Fewer come back when there is no room."""
    vias = board.vias_of(nid)
    if not vias or n_more <= 0:
        return []
    placed: list = []
    for v in vias:
        pitch = v.size_mm + max(FIX_MIN_CLEARANCE_MM, 0.2)
        for k in range(8):
            if len(placed) >= n_more:
                return placed
            ang = 2 * math.pi * k / 8
            at = (v.at[0] + pitch * math.cos(ang), v.at[1] + pitch * math.sin(ang))
            if any(math.hypot(at[0] - q.at[0], at[1] - q.at[1]) < pitch - 1e-9 for q in list(vias) + placed):
                continue
            layers = tuple(v.layers) if v.layers else ("F.Cu", "B.Cu")
            if any(copper_conflict(board, nid, at, at, v.size_mm / 2.0, ly) for ly in layers) or \
                    any(copper_conflict(board, nid, v.at, at, stub_width_mm / 2.0, ly) for ly in layers):
                continue
            placed.append(ViaFix(net=board.net_name(nid), nid=nid, at=at, size_mm=v.size_mm, drill_mm=v.drill_mm,
                                 layers=layers, joins=tuple(v.at), stub_width_mm=stub_width_mm,
                                 note=f"beside the via at ({v.at[0]:.2f}, {v.at[1]:.2f}) mm"))
    return placed


def widening_conflict(board: PcbBoard, seg_index: int, new_width_mm: float,
                      min_clear_mm: float = FIX_MIN_CLEARANCE_MM):
    """The first piece of another net's copper on the same layer that a segment widened to `new_width_mm` (mm)
    would come closer to than it may, as (net name, gap mm, clearance needed mm), or None when the widening is
    clear. The clearance needed is the larger of `min_clear_mm` and either net's net-class clearance in the file,
    or IPC-2221 B4 when either net is declared above 60 V. Checks tracks, pads (as their bounding circle,
    conservatively), vias and other nets' pours."""
    sg = board.segments[seg_index]
    return copper_conflict(board, sg.net, sg.start, sg.end, new_width_mm / 2.0, sg.layer, min_clear_mm,
                           skip_segment=seg_index)


def apply_fixes(board: PcbBoard, fixes: list, vias: list = ()) -> tuple:
    """Return (patched_text, applied_fixes). Only auto=True fixes are applied;
    each rewrites exactly the one width token of its segment — KiCad's
    `(width x)` or Altium's `WIDTH=x` — and nothing else in the file moves, so
    the board reopens in its own EDA with the routing intact.

    Two things the naive version gets wrong and this one does not:
      * **Units.** The number goes back in the file's own units (mm for KiCad,
        whatever the Altium export used, mil or mm), so a patched Altium board
        does not quietly become a millimetre board.
      * **Shared tokens.** A tessellated arc is many segments behind one
        `WIDTH=` token; two fixes on the same span would splice the file twice
        at overlapping offsets and corrupt it. Edits are collapsed per span,
        keeping the widest requirement.
    """
    if not getattr(board, "patchable", True):
        raise ValueError(
            f"this {board.fmt_label} board is read for diagnosis only — the "
            f"Doctor will not rewrite widths inside a binary file it might "
            f"corrupt. Re-save from Altium as 'PCB ASCII File' and re-run to "
            f"get the one-click re-trace; every finding above still applies.")
    per_span = {}          # span -> (width_mm, segment)
    applied = []
    #  Each widening is checked against the board as it will be, other accepted widenings included, so two nets
    #  widened side by side cannot close the gap between them. One that would is left out and turned manual.
    old_w = {}
    try:
        for fx in fixes:
            if not fx.auto:
                continue
            seg = board.segments[fx.seg_index]
            span = tuple(seg.width_span)
            if span == (0, 0):
                continue       # nothing to patch (synthetic segment)
            clash = widening_conflict(board, fx.seg_index, fx.new_width_mm)
            if clash is not None:
                fx.auto = False
                fx.note = (f"widening to {fx.new_width_mm:g} mm would leave {max(clash[1], 0):.2f} mm to "
                           f"'{clash[0]}' once the other fixes are in (needs {clash[2]:g} mm): not "
                           f"applied — reroute this segment, or move the net to 2 oz copper")
                continue
            old_w.setdefault(fx.seg_index, seg.width_mm)
            seg.width_mm = max(seg.width_mm, fx.new_width_mm)
            prev = per_span.get(span)
            if prev is None or fx.new_width_mm > prev[0]:
                per_span[span] = (fx.new_width_mm, seg)
            applied.append(fx)
    finally:
        for i, w in old_w.items():                      # the board object is the diagnosis, not the patch
            board.segments[i].width_mm = w
    edits = sorted(((s[0], s[1], seg.width_token(w))
                    for s, (w, seg) in per_span.items()),
                   key=lambda e: e[0], reverse=True)
    text = board.text
    for a, b, rep in edits:
        text = text[:a] + rep + text[b:]
    if vias:                                       # only the ones the engineer picked; KiCad files only
        if board.fmt != "kicad":
            raise ValueError("stitching vias can be written into KiCad boards only; add them in Altium by hand")
        add = []
        for vf in vias:
            ly = " ".join(f'"{x}"' for x in vf.layers[:2]) if len(vf.layers) >= 2 else '"F.Cu" "B.Cu"'
            add.append(f'  (via (at {vf.at[0]:.4f} {vf.at[1]:.4f}) (size {vf.size_mm:g}) (drill {vf.drill_mm:g}) '
                       f'(layers {ly}) (net {vf.nid}))')
            for L in vf.layers:
                add.append(f'  (segment (start {vf.joins[0]:.4f} {vf.joins[1]:.4f}) (end {vf.at[0]:.4f} '
                           f'{vf.at[1]:.4f}) (width {vf.stub_width_mm:g}) (layer "{L}") (net {vf.nid}))')
        end = text.rstrip().rfind(")")
        text = text[:end] + "\n".join(add) + "\n" + text[end:]
        applied = applied + list(vias)
    return text, applied


_SEV_RANK = {"ok": 0, "info": 1, "missing": 2, "warning": 3, "fail": 4}


def _worse_findings(before: DoctorReport, after: DoctorReport) -> list:
    """Findings in `after` that are new, or more severe than the same check in `before` (by check name)."""
    was = {}
    for f in before.findings:
        was[f.check] = max(was.get(f.check, 0), _SEV_RANK.get(f.severity.value, 0))
    return [f for f in after.findings
            if _SEV_RANK.get(f.severity.value, 0) >= _SEV_RANK["warning"]
            and _SEV_RANK.get(f.severity.value, 0) > was.get(f.check, 0)]


def fix_board(board: PcbBoard, assignments: dict, report: DoctorReport = None, max_rounds: int = 4,
              vias: list = (), **diag_kw) -> dict:
    """Apply the automatic width fixes and prove the result: the patched file is parsed and diagnosed again, and
    any fix whose result shows a finding the original did not have (or a worse one) is withdrawn, turned manual
    with the reason, and the rest re-applied, until the patched board is no worse anywhere. Fixes that the first
    patch leaves still wanting (heating moves with width) are applied in further rounds.

    Returns {"text" (the patched file, or None when nothing could be applied safely), "applied", "manual",
    "report" (the patched board's diagnosis), "remaining" (FAIL and WARN findings still on the patched board;
    these need rerouting, vias or copper weight, which a width rewrite cannot give), "rounds"}.
    diag_kw go to `diagnose` (rail_v, brownout_v, ambient_c in °C, max_temp_c in °C, fuse_sf, ...)."""
    rep0 = report if report is not None else diagnose(board, assignments, **diag_kw)
    vias = list(vias)
    work, text, rep, applied, rnd = board, None, rep0, [], 0
    manual = [fx for fx in rep0.fixes if not fx.auto]
    name = getattr(board, "filename", "") or ("x" + board.file_suffix)
    for rnd in range(1, max_rounds + 1):
        cand = list(rep.fixes)
        if not [fx for fx in cand if fx.auto]:
            break
        progressed = False
        while [fx for fx in cand if fx.auto]:
            t2, app2 = apply_fixes(work, cand)
            if not app2:
                break
            b2 = parse_board(t2, name)
            b2.copper_oz = board.copper_oz
            rep2 = diagnose(b2, assignments, **diag_kw)
            worse = _worse_findings(rep0, rep2)
            if not worse:
                work, text, rep, progressed = b2, t2, rep2, True
                applied += app2
                break
            #  Withdraw the fixes on the nets the new findings name; with no net to blame, withdraw them all.
            bad = {fx.net for fx in app2 if any(fx.net in f.check or f"'{fx.net}'" in f.message for f in worse)}
            for fx in app2:
                if not bad or fx.net in bad:
                    fx.auto = False
                    fx.note = (f"withdrawn: with this widening the patched board shows "
                               f"'{worse[0].check}' ({worse[0].severity.value}), which the original did not")
        manual += [fx for fx in cand if not fx.auto and fx not in manual]
        if not progressed:
            break
    #  The stitching vias the engineer picked go in last, on top of the verified widths, and are verified the same way.
    if vias and board.fmt == "kicad":
        t3, app3 = apply_fixes(work, [], vias)
        b3 = parse_board(t3, name)
        b3.copper_oz = board.copper_oz
        rep3 = diagnose(b3, assignments, **diag_kw)
        worse = _worse_findings(rep0, rep3)
        if worse:
            for vf in vias:
                vf.note += (f"; withdrawn: with the picked vias the board shows '{worse[0].check}' "
                            f"({worse[0].severity.value}), which the original did not")
            manual += vias
        else:
            work, text, rep = b3, t3, rep3
            applied += app3
    elif vias:
        manual += vias                                 # Altium: written by hand from the recommendation
    #  A fix still wanted on the final board but not made is manual, whatever round it came from.
    manual = [fx for fx in manual if isinstance(fx, ViaFix) or not any(
        getattr(a, "seg_index", None) == fx.seg_index and a.net == fx.net for a in applied)]
    remaining = [f for f in rep.findings if f.severity.value in ("fail", "warning")]
    return {"text": text, "applied": applied, "manual": manual, "report": rep, "remaining": remaining,
            "rounds": rnd}


def bringup_measurements(board: PcbBoard, nets=None, room_c: float = 22.0,
                         copper_finish: float = COPPER_FINISH_FACTOR) -> list:
    """The bring-up checklist: for each net (default: every net KinematiK can solve), the two pads to measure
    between, the resistance (mΩ) expected at room temperature `room_c` (°C), and the band (mΩ) a good board falls
    in. For the nets named in `nets` the expected value is the refined one (`refined_net_resistance`); with no list
    it is the quick 1-D estimate, so a whole board stays fast. Measure with a 4-wire (Kelvin) milliohm meter, probes on the pads, the board unpowered.

    The band combines the model's measured error on held-out boards (2-D reference, docs/ELECTRICAL_BENCHMARK.md
    B5: worst −3.1 % / +21.2 %) with the fab's copper tolerance (±(1 − copper_finish)). Above the band: copper
    thinner than assumed, a cracked joint or a wrong track; re-run the brown-out check with the measured value.
    Below it: thicker copper or a pour or solder the model doesn't count, which is harmless."""
    from .electronics import ALPHA_CU as _A
    ids = nets if nets is not None else sorted({s.net for s in board.segments if s.net})
    tol = 1.0 - copper_finish
    rows = []
    for nid in ids:
        r = analyze_net(board, nid)
        if r.get("worst_r_ohm") is None:
            continue
        r_best = refined_net_resistance(board, nid, r)["r_ohm"] if nets is not None else r["worst_r_ohm"]
        est = r_best * (1 + _A * (room_c - 20.0)) / (1 + _A * 40.0)             # the solves run at 60 °C
        rows.append({"net": board.net_name(nid), "pads": r["worst_pair"],
                     "expected_mohm": est * 1e3,
                     "band_mohm": (est / 1.212 / (1 + tol) * 1e3, est / 0.969 / (1 - tol) * 1e3)})
    return rows


def fix_report_md(board: PcbBoard, report: DoctorReport, applied: list,
                  assignments: dict) -> str:
    """A hand-off document: what was wrong, why it fails in real life, what was
    changed automatically, what still needs a human."""
    order = {"fail": 0, "warning": 1, "missing": 2, "info": 3, "ok": 4}
    eda = board.fmt_label
    lines = ["# PCB Doctor — fix report", "",
             f"Source board: **{eda}** · "
             f"{len(board.segments)} copper segments · {len(board.vias)} vias · "
             f"{sum(1 for f in board.footprints if f.ref != '?')} components", "",
             f"Nets diagnosed: {len(assignments)} · {report.summary()}", ""]
    if board.notes:
        lines += ["## What the import had to assume"]
        lines += [f"- {n}" for n in board.notes]
        lines.append("")
    lines.append("## Findings (why it fails in real life)")
    for f in sorted(report.findings, key=lambda x: order[x.severity.value]):
        lines.append(f"- **{f.severity.value.upper()} — {f.check}**: {f.message}")
    lines += ["", "## Widths rewritten in the patched file"]
    if applied:
        for fx in applied:
            lines.append(f"- `{fx.net}` on {fx.layer}: "
                         f"{fx.old_width_mm:g} mm → **{fx.new_width_mm:g} mm** "
                         f"(segment #{fx.seg_index})")
        lines.append("")
        lines.append("Open the patched file in KiCad and re-run DRC: a widened "
                     "trace can newly crowd a neighbour — the Doctor re-checks HV "
                     "clearance on the patched geometry, but LV clearance is "
                     "DRC's job.")
    else:
        lines.append("- none required / all remaining fixes need a human "
                     "(diff pairs, via stitching, reroutes).")
    manual = [fx for fx in report.fixes if not fx.auto]
    if manual:
        lines += ["", "## Prescribed but NOT auto-applied"]
        for fx in manual:
            lines.append(f"- `{fx.net}` ({fx.layer}) needs {fx.new_width_mm:g} mm "
                         f"equivalent — {fx.note}")
    ids = [board.net_id(n) for n in assignments if board.net_id(n)]
    plan = bringup_measurements(board, ids) if ids else []
    if plan:
        lines += ["", "## Bring-up measurements (on the built board)",
                  "The file is not the board: copper comes back thinner or thicker, joints vary. Before powering "
                  "the car, measure each declared power net with a 4-wire (Kelvin) milliohm meter, board unpowered, "
                  "probes on the two pads named. Above the band: thin copper, a cracked joint or a wrong track; "
                  "re-run the brown-out check with the measured value.", "",
                  "| Net | Measure between | Expected at 22 °C | Good board |", "|---|---|---|---|"]
        for r_ in plan:
            lo, hi = r_["band_mohm"]
            lines.append(f"| `{r_['net']}` | {r_['pads'][0]} – {r_['pads'][1]} | {r_['expected_mohm']:.1f} mΩ | "
                         f"{lo:.1f}–{hi:.1f} mΩ |")
    lines += ["", "---",
              "Analytic screening (IPC-2221 heating, Onderdonk fusing, nodal "
              "IR-drop, B4 clearance). Heating assumes an isolated trace in still air on a board at the ambient you "
              "set: enter the hottest local temperature near the trace (enclosure, nearby hot parts), not the "
              "outside air. Not a field solver, not a DRC replacement. "
              "Validate the patched board in KiCad DRC + your fab's rules before "
              "ordering."]
    return "\n".join(lines)


# --------------------------------------------------------------------------- #
#  Demo board — one click, three planted real-life failures
# --------------------------------------------------------------------------- #
#  The demo's planted faults are *physics* faults — the kind DRC passes. The
#  rest of the board has to be copper a real fab would accept, because the
#  viewer draws every pad: two connector pins that overlap, a sense trace that
#  runs straight through a connector pad, or a rail that starts on another
#  net's pad are shorts anyone can see, and they make the whole demo read as
#  broken. Every trace below ends on its own net's pad or via, no two pads of
#  different nets touch, and no trace crosses a pad of another net
#  (`TestDemoBoardIsBuildable` holds that line).
#
#  The one failure the Doctor can only find with a *declared* current is the
#  fan feed, and a fresh session's ledger declares nothing — so the demo ships
#  its own declarations (`DEMO_NET_DECLARATIONS`). Without them the demo shows
#  "0 width fixes", which is the Doctor correctly refusing to judge a guess and
#  the member concluding the feature does nothing.
DEMO_NET_DECLARATIONS = {
    # the fan's peak: the number the planted 0.30 mm feed cannot carry
    "FAN_PWR": {"current_a": 8.0},
    # the inverter sense line sits at pack voltage — the clearance fault
    "HV_INV_SENSE": {"voltage_v": 400.0},
}
DEMO_DECLARATION_SOURCE = "demo board — declared with the demo (edit me)"


def apply_demo_declarations(board: PcbBoard, assignments: dict,
                            overrides: dict = None) -> dict:
    """Declare the demo board's peak currents / voltages onto `assignments`
    (in place, and returned). Keyed by net *name*, so the KiCad and Altium
    demos — whose net ids differ — receive the same inputs. `overrides`
    ({net: {"current_a": …}}) lets a test ask "what if the fan drew 0.5 A"
    through the same path the UI uses."""
    decl = {k: dict(v) for k, v in DEMO_NET_DECLARATIONS.items()}
    for k, v in (overrides or {}).items():
        decl.setdefault(k, {}).update(v)
    for name, spec in decl.items():
        nid = board.net_id(name)
        if nid is None:
            continue
        if "current_a" in spec:
            declare_net_current(assignments, nid, spec["current_a"],
                                voltage_v=spec.get("voltage_v"),
                                source=DEMO_DECLARATION_SOURCE)
            assignments[nid]["net"] = assignments[nid].get("net") or name
        elif nid in assignments and "voltage_v" in spec:
            # a voltage alone is not a current: the net stays MISSING-current,
            # which is exactly what a real sense line with no declared amps is
            assignments[nid]["voltage_v"] = float(spec["voltage_v"])
    return assignments


def demo_kicad_pcb() -> str:
    """A small synthetic .kicad_pcb: an ECU board whose fan feed is under-sized
    and via-choked, whose CAN pair hugs the 400 V inverter sense net, and whose
    bulk cap sits on the hot copper — the three failures teams actually hit.
    Declare its currents with `apply_demo_declarations`."""
    return """(kicad_pcb (version 20240108) (generator "kinematik-demo")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (1 "In1.Cu" signal) (2 "In2.Cu" signal) (31 "B.Cu" signal))
  (net 0 "") (net 1 "GND") (net 2 "FAN_PWR") (net 3 "CAN_H") (net 4 "CAN_L")
  (net 5 "HV_INV_SENSE") (net 6 "LV_5V")
  (footprint "Connector:J1" (layer "F.Cu") (at 5 10)
    (property "Reference" "J1") (property "Value" "FanConn")
    (pad "1" thru_hole circle (at 0 0) (size 1.7 1.7) (drill 1.0) (net 2 "FAN_PWR"))
    (pad "2" thru_hole circle (at 0 2.54) (size 1.7 1.7) (drill 1.0) (net 1 "GND")))
  (footprint "Regulator:U1" (layer "F.Cu") (at 70 10)
    (property "Reference" "U1") (property "Value" "VNH7070")
    (pad "1" smd rect (at 0 0) (size 2 2) (net 2 "FAN_PWR"))
    (pad "2" smd rect (at 0 3) (size 2 2) (net 6 "LV_5V")))
  (footprint "Capacitor:C1" (layer "F.Cu") (at 40 11.5)
    (property "Reference" "C1") (property "Value" "470uF 16V")
    (pad "1" smd rect (at 0 0) (size 1.5 1.5) (net 2 "FAN_PWR"))
    (pad "2" smd rect (at 0 2) (size 1.5 1.5) (net 1 "GND")))
  (footprint "Capacitor:C2" (layer "F.Cu") (at 70 25)
    (property "Reference" "C2") (property "Value" "10uF 0805")
    (pad "1" smd rect (at 0 0) (size 1.2 1.2) (net 6 "LV_5V"))
    (pad "2" smd rect (at 0 2) (size 1.2 1.2) (net 1 "GND")))
  (footprint "Fuse:F1" (layer "F.Cu") (at 20 10)
    (property "Reference" "F1") (property "Value" "5A blade")
    (pad "1" smd rect (at -2 0) (size 2 2) (net 2 "FAN_PWR"))
    (pad "2" smd rect (at 2 0) (size 2 2) (net 2 "FAN_PWR")))
  (footprint "MCU:U2" (layer "F.Cu") (at 70 40)
    (property "Reference" "U2") (property "Value" "STM32F4")
    (pad "1" smd rect (at 0 0) (size 1 1) (net 3 "CAN_H"))
    (pad "2" smd rect (at 0 1.5) (size 1 1) (net 4 "CAN_L")))
  (footprint "Connector:J2" (layer "F.Cu") (at 5 40)
    (property "Reference" "J2") (property "Value" "CAN out")
    (pad "1" thru_hole circle (at 0 0) (size 1.1 1.1) (drill 0.7) (net 3 "CAN_H"))
    (pad "2" thru_hole circle (at 0 1.5) (size 1.1 1.1) (drill 0.7) (net 4 "CAN_L")))
  (footprint "Connector:J3" (layer "F.Cu") (at 12 33)
    (property "Reference" "J3") (property "Value" "HV sense in")
    (pad "1" thru_hole circle (at 0 0) (size 2 2) (drill 1.2) (net 5 "HV_INV_SENSE")))
  (footprint "Resistor:R1" (layer "F.Cu") (at 60 33)
    (property "Reference" "R1") (property "Value" "1M HV divider 2512")
    (pad "1" smd rect (at 0 0) (size 1.6 3.2) (net 5 "HV_INV_SENSE"))
    (pad "2" smd rect (at 5.5 0) (size 1.6 3.2) (net 1 "GND")))
  (segment (start 5 10) (end 18 10) (width 0.3) (layer "F.Cu") (net 2))
  (segment (start 22 10) (end 40 10) (width 0.3) (layer "F.Cu") (net 2))
  (segment (start 40 10) (end 40 11.5) (width 0.3) (layer "F.Cu") (net 2))
  (segment (start 40 10) (end 45 10) (width 0.3) (layer "F.Cu") (net 2))
  (via (at 45 10) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net 2))
  (segment (start 45 10) (end 70 10) (width 0.3) (layer "B.Cu") (net 2))
  (via (at 70 10) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net 2))
  (segment (start 5 40) (end 70 40) (width 0.2) (layer "F.Cu") (net 3))
  (segment (start 5 41.5) (end 55 41.5) (width 0.2) (layer "F.Cu") (net 4))
  (segment (start 55 41.5) (end 60 46) (width 0.2) (layer "F.Cu") (net 4))
  (segment (start 60 46) (end 68 46) (width 0.2) (layer "F.Cu") (net 4))
  (segment (start 68 46) (end 70 41.5) (width 0.2) (layer "F.Cu") (net 4))
  (segment (start 12 33) (end 12 39.2) (width 0.25) (layer "F.Cu") (net 5))
  (segment (start 12 39.2) (end 60 39.2) (width 0.25) (layer "F.Cu") (net 5))
  (segment (start 60 39.2) (end 60 33) (width 0.25) (layer "F.Cu") (net 5))
  (segment (start 70 13) (end 70 25) (width 0.5) (layer "F.Cu") (net 6))
)
"""


# --------------------------------------------------------------------------- #
#  SVG board viewer — the copper a fix will widen glows, everything else dims
# --------------------------------------------------------------------------- #
#  KiCad's own palette, so a KiCad user reads the layers without a legend. That
#  palette puts F.Cu in red, which is exactly why "failing copper glows red"
#  could never work: on a single-layer view every trace was already red, so a
#  board with nothing to fix looked like a board on fire. The fix highlight is
#  now a white glow on full-strength copper with the rest of the board dimmed —
#  a signal no layer colour can imitate.
_LAYER_COLORS = {"F.Cu": "#d94f4f", "B.Cu": "#4a7bd9", "In1.Cu": "#3fae7a",
                 "In2.Cu": "#c9a03a", "In3.Cu": "#9a6ad1", "In4.Cu": "#48b8b8"}
_FIX_GLOW = "#ffffff"          # the halo on copper a pending fix will widen
_TH_PAD_FILL = "#d7b46a"       # plated through-hole pads: both sides at once
_VIEW_BG = "#0e1419"


def _pad_extent(p) -> float:
    """Half the diagonal-safe span of a pad — enough to keep a rotated pad
    inside the frame without computing its rotated box."""
    w, h = (p.size or (1.0, 1.0))[:2]
    return 0.5 * max(float(w), float(h))


def _view_bbox(board: PcbBoard, layers, pads):
    """Frame what is actually drawn: copper *edges*, pads and vias — not just
    centre-lines and footprint origins, which clipped every pad on the board
    edge and left a parts-only footprint off-frame entirely."""
    xs0, ys0, xs1, ys1 = [], [], [], []
    for sg in board.segments:
        if sg.layer not in layers:
            continue
        r = sg.width_mm / 2.0
        for x, y in (sg.start, sg.end):
            xs0.append(x - r); xs1.append(x + r)
            ys0.append(y - r); ys1.append(y + r)
    for v in board.vias:
        r = v.size_mm / 2.0
        xs0.append(v.at[0] - r); xs1.append(v.at[0] + r)
        ys0.append(v.at[1] - r); ys1.append(v.at[1] + r)
    for _fp, p in pads:
        r = _pad_extent(p)
        xs0.append(p.at[0] - r); xs1.append(p.at[0] + r)
        ys0.append(p.at[1] - r); ys1.append(p.at[1] + r)
    if not xs0:
        return board.bbox()
    return min(xs0), min(ys0), max(xs1), max(ys1)


def board_svg(board: PcbBoard, report=None, show_layers=None,
              width_px: int = 760, height_px: int = 440) -> str:
    """Inline SVG of the parsed copper: segments per layer, the real pads of
    every footprint (so each trace can be seen landing on its pin), vias and
    component refs. When the report carries fixes, the segments they touch
    glow and the rest of the copper dims."""
    from html import escape as _esc
    layers = list(show_layers or board.copper_layers)
    lset = set(layers)

    def _pad_layer(fp, p):
        return p.layer or fp.layer or "F.Cu"

    pads = [(fp, p) for fp in board.footprints for p in fp.pads
            if p.through or _pad_layer(fp, p) in lset]
    x0, y0, x1, y1 = _view_bbox(board, lset, pads)
    w = max(x1 - x0, 1.0); h = max(y1 - y0, 1.0)
    side, top, bottom = 18, 44, 30      # legend strip + label room / scale bar
    s = min((width_px - 2 * side) / w, (height_px - top - bottom) / h)
    # centre the board in whatever room the aspect ratio leaves over
    ox = side + ((width_px - 2 * side) - w * s) / 2.0
    oy = top + ((height_px - top - bottom) - h * s) / 2.0

    def tx(pt):
        return ox + (pt[0] - x0) * s, oy + (pt[1] - y0) * s

    fixes_by_seg = {}
    if report is not None:
        for fx in report.fixes:
            fixes_by_seg.setdefault(fx.seg_index, fx)
    dim = bool(fixes_by_seg)
    parts = [f'<svg viewBox="0 0 {width_px} {height_px}" '
             f'xmlns="http://www.w3.org/2000/svg" '
             f'style="width:100%;height:auto;background:{_VIEW_BG};'
             f'border:1px solid var(--line);border-radius:8px;">']

    # ---- copper ------------------------------------------------------------ #
    glow = []
    for i, seg in enumerate(board.segments):
        if seg.layer not in lset:
            continue
        (ax, ay), (bx, by) = tx(seg.start), tx(seg.end)
        col = _LAYER_COLORS.get(seg.layer, "#7d8c99")
        sw = max(seg.width_mm * s, 1.2)
        fx = fixes_by_seg.get(i)
        line = (f'x1="{ax:.1f}" y1="{ay:.1f}" x2="{bx:.1f}" y2="{by:.1f}" '
                f'stroke-linecap="round"')
        if fx is not None:
            tip = _esc(f"{fx.net} on {fx.layer}: {fx.old_width_mm:g} → "
                       f"{fx.new_width_mm:g} mm"
                       + ("" if fx.auto else " (prescription — not auto)"))
            glow.append(f'<g><title>{tip}</title>'
                        f'<line {line} stroke="{_FIX_GLOW}" '
                        f'stroke-width="{sw + 7:.1f}" opacity="0.28"/>'
                        f'<line {line} stroke="{_FIX_GLOW}" '
                        f'stroke-width="{sw + 3:.1f}" opacity="0.55"/>'
                        f'<line {line} stroke="{col}" stroke-width="{sw:.1f}"/>'
                        f'</g>')
        else:
            parts.append(f'<line {line} stroke="{col}" stroke-width="{sw:.1f}" '
                         f'opacity="{0.35 if dim else 0.9}"/>')
    parts.extend(glow)                   # on top, so nothing hides a fix

    # ---- pads: the copper every trace is supposed to land on --------------- #
    for fp, p in pads:
        cx, cy = tx(p.at)
        pw, ph = (float(v) * s for v in (p.size or (1.0, 1.0))[:2])
        pw, ph = max(pw, 2.0), max(ph, 2.0)
        shape = p.shape or ("round" if p.through else "rect")
        rx = (min(pw, ph) / 2.0 if shape == "round"
              else min(pw, ph) * 0.25 if shape == "roundrect" else 0.0)
        fill = (_TH_PAD_FILL if p.through
                else _LAYER_COLORS.get(_pad_layer(fp, p), "#7d8c99"))
        rot = (f' transform="rotate({-p.angle_deg:g} {cx:.1f} {cy:.1f})"'
               if p.angle_deg else "")
        tip = _esc(f"{fp.ref}.{p.number} · {p.net_name or 'no net'}")
        parts.append(f'<g{rot}><title>{tip}</title>'
                     f'<rect x="{cx - pw / 2:.1f}" y="{cy - ph / 2:.1f}" '
                     f'width="{pw:.1f}" height="{ph:.1f}" rx="{rx:.1f}" '
                     f'fill="{fill}" opacity="{0.7 if dim else 0.95}"/>')
        if p.through:
            parts.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" '
                         f'r="{max(min(pw, ph) * 0.22, 0.8):.1f}" '
                         f'fill="{_VIEW_BG}"/>')
        parts.append('</g>')

    for v in board.vias:
        vx, vy = tx(v.at)
        parts.append(f'<circle cx="{vx:.1f}" cy="{vy:.1f}" '
                     f'r="{max(v.size_mm * s / 2, 2):.1f}" fill="#e8edf2" '
                     f'stroke="{_VIEW_BG}" stroke-width="1"/>')

    # ---- reference labels, placed where no copper is ----------------------- #
    #  First choice is just above the part's pads; when a trace arrives from
    #  that side (a cap fed from above, a connector under a bus) the label
    #  moves right, left or below instead of being struck through. A coarse
    #  grid keeps the collision test local, so a 6000-segment board still draws
    #  in well under a second.
    CELL = 40.0
    grid = {}

    def _cells(bx0, by0, bx1, by1):
        for gx in range(int(bx0 // CELL), int(bx1 // CELL) + 1):
            for gy in range(int(by0 // CELL), int(by1 // CELL) + 1):
                yield gx, gy

    def _add(ob, box):
        for c in _cells(*box):
            grid.setdefault(c, []).append(ob)

    for seg in board.segments:
        if seg.layer not in lset:
            continue
        (ax, ay), (bx, by) = tx(seg.start), tx(seg.end)
        r = max(seg.width_mm * s, 1.2) / 2.0 + 1.0
        _add(("seg", ax, ay, bx, by, r),
             (min(ax, bx) - r, min(ay, by) - r, max(ax, bx) + r, max(ay, by) + r))
    for _fp, p in pads:
        cx, cy = tx(p.at)
        r = _pad_extent(p) * s + 1.0
        _add(("box", cx - r, cy - r, cx + r, cy + r), (cx - r, cy - r, cx + r, cy + r))

    def _hits(bx0, by0, bx1, by1):
        seen = set()
        for c in _cells(bx0, by0, bx1, by1):
            for ob in grid.get(c, ()):
                if id(ob) in seen:
                    continue
                seen.add(id(ob))
                if ob[0] == "box":
                    if not (ob[3] < bx0 or ob[1] > bx1 or ob[4] < by0 or ob[2] > by1):
                        return True
                else:
                    _, ax, ay, bx, by, r = ob
                    # distance from the box centre-line to the segment, cheap
                    # and conservative: sample the segment at ~4 px steps
                    n = max(1, int(math.hypot(bx - ax, by - ay) // 4))
                    for k in range(n + 1):
                        px = ax + (bx - ax) * k / n
                        py = ay + (by - ay) * k / n
                        if bx0 - r <= px <= bx1 + r and by0 - r <= py <= by1 + r:
                            return True
        return False

    for fp in board.footprints:
        fpads = [p for p in fp.pads if p.through or _pad_layer(fp, p) in lset]
        tw, th = 6.6 * len(fp.ref or "?") + 2, 10.0
        if fpads:
            ext = [(tx(p.at), _pad_extent(p) * s) for p in fpads]
            px0 = min(c[0] - r for c, r in ext); px1 = max(c[0] + r for c, r in ext)
            py0 = min(c[1] - r for c, r in ext); py1 = max(c[1] + r for c, r in ext)
        else:
            if (fp.layer or "F.Cu") not in lset:
                continue
            fx_, fy_ = tx(fp.at)
            parts.append(f'<rect x="{fx_ - 3:.1f}" y="{fy_ - 3:.1f}" width="6" '
                         f'height="6" fill="none" stroke="#e8edf2" '
                         f'stroke-width="1" rx="1.5"/>')
            px0, py0, px1, py1 = fx_ - 3, fy_ - 3, fx_ + 3, fy_ + 3
        ymid = (py0 + py1) / 2.0
        # (left, baseline) candidates: above, right, left, below
        cands = [(px0, py0 - 4), (px1 + 4, ymid + th / 2 - 1),
                 (px0 - 4 - tw, ymid + th / 2 - 1), (px0, py1 + th + 3)]
        lx, ly = cands[0]
        for cx_, cy_ in cands:
            bx0, by0, bx1, by1 = cx_, cy_ - th + 1, cx_ + tw, cy_ + 1
            if bx0 < 0 or bx1 > width_px or by0 < top - 14 or by1 > height_px - bottom + 8:
                continue
            if not _hits(bx0, by0, bx1, by1):
                lx, ly = cx_, cy_
                break
        #  Two passes (outline, then fill) rather than `paint-order`, which
        #  some SVG renderers ignore and then paint the outline over the text.
        #  The outline keeps a label legible even where no free spot existed.
        ref = _esc(fp.ref)
        txt = (f'x="{lx:.1f}" y="{ly:.1f}" font-size="10" font-weight="700"')
        parts.append(f'<text {txt} fill="none" stroke="{_VIEW_BG}" '
                     f'stroke-width="3" stroke-linejoin="round">{ref}</text>'
                     f'<text {txt} fill="#e8edf2">{ref}</text>')

    # ---- legend strip (above the board, so it never sits on copper) --------- #
    lx = side
    for ly in layers:
        parts.append(f'<line x1="{lx}" y1="14" x2="{lx + 16}" y2="14" '
                     f'stroke="{_LAYER_COLORS.get(ly, "#7d8c99")}" '
                     f'stroke-width="4"/><text x="{lx + 21}" y="18" '
                     f'fill="#8d99a6" font-size="10">'
                     f'{_esc(board.native_layer(ly))}</text>')
        lx += 30 + 6 * len(board.native_layer(ly))
    if dim:
        parts.append(f'<line x1="{lx}" y1="14" x2="{lx + 16}" y2="14" '
                     f'stroke="{_FIX_GLOW}" stroke-width="7" opacity="0.5"/>'
                     f'<text x="{lx + 21}" y="18" fill="#e8edf2" '
                     f'font-size="10">glow = segment a fix will widen '
                     f'({len(fixes_by_seg)})</text>')

    # ---- scale bar ---------------------------------------------------------- #
    bar_mm = next((b for b in (100, 50, 20, 10, 5, 2, 1, 0.5)
                   if b * s <= width_px * 0.35), 0.5)
    bar = bar_mm * s
    parts.append(f'<line x1="{side}" y1="{height_px - 10}" '
                 f'x2="{side + bar:.1f}" y2="{height_px - 10}" '
                 f'stroke="#8d99a6" stroke-width="2"/>'
                 f'<text x="{side}" y="{height_px - 15}" fill="#8d99a6" '
                 f'font-size="10">{bar_mm:g} mm</text>')
    parts.append("</svg>")
    return "".join(parts)

