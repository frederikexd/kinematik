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
  * Copper pours (zones) are not meshed — nets with pours get *conservative*
    trace-only resistance labelled as such, and are never declared "open".
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
from .electronics import (OZ_TO_UM, RHO_CU_20C, ALPHA_CU, Trace,
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
                vals.append(float(c.value))
            except ValueError:
                pass
    return vals if count is None else (vals + [0.0] * count)[:count]


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
    if _name(root) not in ("kicad_pcb", "pcb"):
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
        except (TypeError, ValueError):
            pass
        name = str(raw).strip().strip('"')
        if not name:
            return 0
        if name not in _by_name:
            nid = (max(board.nets) + 1) if board.nets else 1
            board.nets[nid] = name
            _by_name[name] = nid
        return _by_name[name]

    for node in root[1:]:
        if not isinstance(node, list):
            continue
        nm = _name(node)
        if nm == "net":
            ats = _atoms(node)
            if ats:
                try:
                    nid = int(float(ats[0].value))
                except ValueError:
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
                    width_mm=float(w_atom.value),
                    start=tuple(_floats(st_, 2)), end=tuple(_floats(en_, 2)),
                    width_span=(w_atom.start, w_atom.end))
                board.segments.append(seg)
            except (ValueError, IndexError, AttributeError):
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
                    width_mm=float(w_atom.value),
                    width_span=(w_atom.start, w_atom.end)))
            except (ValueError, IndexError, AttributeError):
                continue
        elif nm == "via":
            at, sz, dr, ly, nt = (_child(node, k) for k in
                                  ("at", "size", "drill", "layers", "net"))
            if not (at and nt):
                continue
            try:
                layers = tuple(a.value for a in _atoms(ly)) if ly else ("F.Cu", "B.Cu")
                board.vias.append(PcbVia(
                    net=_net_ref(nt),
                    at=tuple(_floats(at, 2)),
                    size_mm=_floats(sz)[0] if sz and _floats(sz) else 0.6,
                    drill_mm=_floats(dr)[0] if dr and _floats(dr) else 0.3,
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
                        PcbZone(net=znid, layer=lay, outline=pts))
    return board


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
        px, py = (_floats(pat, 2) if pat else [0.0, 0.0])
        # pad offset is in footprint frame; rotate into board frame.
        # KiCad's y axis points down; fp rotation is CCW in its own convention —
        # the standard transform below matches KiCad's file coordinates.
        ax = fx + px * cr + py * sr
        ay = fy - px * sr + py * cr
        psz = _child(pd, "size")
        size = tuple(_floats(psz, 2)) if psz else (1.0, 1.0)
        pnet, pname = 0, ""
        pnt = _child(pd, "net")
        if pnt and _atoms(pnt):
            a2 = _atoms(pnt)
            if net_ref is not None:
                pnet = net_ref(pnt)          # handles both KiCad dialects
            else:
                try:
                    pnet = int(float(a2[0].value))
                except (TypeError, ValueError):
                    pnet = 0
            # v5-9: (net 2 "VCC"); v10: (net "VCC")
            pname = (a2[1].value if len(a2) > 1
                     else str(a2[0].value).strip().strip('"'))
            if pname.lstrip("-").isdigit():
                pname = ""
        fp.pads.append(PcbPad(number=number, net=pnet, net_name=pname,
                              at=(ax, ay), size=size, through=through,
                              layer=pad_layer))
    return fp


# --------------------------------------------------------------------------- #
#  IPC-2221 sizing primitives (the Trace Prescriber core)
# --------------------------------------------------------------------------- #
def required_area_mil2(current_a: float, dT_c: float, external: bool) -> float:
    """Invert the IPC-2221 heating chart: copper cross-section (mil²) that keeps
    the steady-state rise at `dT_c` for `current_a`."""
    if current_a <= 0 or dT_c <= 0:
        return 0.0
    k = 0.048 if external else 0.024
    return float((current_a / (k * dT_c ** 0.44)) ** (1.0 / 0.725))


def required_width_mm(current_a: float, dT_c: float, copper_oz: float,
                      external: bool) -> float:
    """Minimum finished trace width for a current at a temperature-rise budget."""
    a_mm2 = required_area_mil2(current_a, dT_c, external) / MM2_TO_MIL2
    t_mm = copper_oz * OZ_TO_UM / 1000.0
    return a_mm2 / t_mm if t_mm > 0 else float("inf")


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
    k = 0.048 if external else 0.024
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


def _node_key(layer: str, pt) -> tuple:
    return (layer, round(pt[0], 2), round(pt[1], 2))


def _weld_nodes(node_fn, keyed: dict, tol: float = NODE_WELD_MM) -> list:
    """Return zero-ohm edges joining copper nodes that are within `tol` of each
    other on the same layer.

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
                        if pair in seen:
                            continue
                        if math.hypot(pt[0] - opt[0], pt[1] - opt[1]) <= tol:
                            seen.add(pair)
                            out.append((node_fn(key), node_fn(okey), 1e-9))
    return out


def analyze_net(board: PcbBoard, nid: int, ambient_c: float = 40.0) -> dict:
    """Build the resistor mesh of a net's real copper (segments + via barrels),
    then report: bottleneck segment, per-layer routed length, connectivity of
    the pads, and worst pad-to-pad effective resistance (nodal analysis).
    Zones are NOT meshed: resistance is trace-only (conservative) and nets with
    pours are never declared open."""
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
    for s in segs:
        ka, kb = _node_key(s.layer, s.start), _node_key(s.layer, s.end)
        keyed.setdefault(ka, s.start)
        keyed.setdefault(kb, s.end)
        a, b = node(ka), node(kb)
        tr = Trace(name="s", net="s", owner_subsystem="e",
                   width_mm=max(s.width_mm, 1e-4), copper_oz=board.copper_oz,
                   length_mm=max(s.length_mm, 1e-3),
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
    edges.extend(_weld_nodes(node, keyed))

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
    for hub, v, vlayers in via_hubs:
        reach_v = max(v.size_mm, v.drill_mm) / 2.0 + NODE_WELD_MM
        for k, idx in list(nodes.items()):
            if k[0] in ("via", "pad", "zone") or k[0] not in vlayers:
                continue
            if math.hypot(k[1] - v.at[0], k[2] - v.at[1]) <= reach_v:
                edges.append((hub, idx, 1e-5))

    # pads: connect to any node on the pad's copper within its own footprint
    pad_nodes = []
    node_pts = {idx: key for key, idx in nodes.items()}
    for fp, p in pads:
        key = ("pad", fp.ref, p.number)
        pn = node(key)
        reach = max(p.size) / 2.0 + 0.06
        pad_layer = p.layer or ("B.Cu" if fp.layer == "B.Cu" else "F.Cu")
        hit = False
        for k, idx in list(nodes.items()):
            if k[0] in ("pad",):
                continue
            lx, ly_ = k[1], k[2]
            if math.hypot(lx - p.at[0], ly_ - p.at[1]) <= reach:
                # via hubs and through-hole barrels join every layer; an SMD pad
                # only touches copper on its own side of the board.
                if k[0] == "via" or p.through or k[0] == pad_layer:
                    edges.append((pn, idx, 1e-5))
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
    _by_fp = {}
    for ref, num, pn, hit in pad_nodes:
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
    conn_edges = list(edges)
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
        try:
            Lp = np.linalg.pinv(L)
            sample = attached[:10]
            for i in range(len(sample)):
                for j in range(i + 1, len(sample)):
                    a, b = sample[i][2], sample[j][2]
                    r = float(Lp[a, a] + Lp[b, b] - 2 * Lp[a, b])
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
        if peak is None:
            # Undeclared contributes nothing to the assignment, so its absence
            # must not appear as a change — otherwise declaring a mass in an
            # unrelated subsystem would throw away the net table.
            continue
        out.append((str(name), float(peak)))
    return tuple(sorted(out))


def auto_assign_net_currents(board: PcbBoard, ledger=None) -> dict:
    """Map each routed net to (current_a, voltage_v, source_note) using name
    heuristics against the integration ledger's declared peak currents. Every
    guess is labelled; the UI lets the user overwrite all of it."""
    peaks = {}
    if ledger is not None:
        for sub_name, iface in getattr(ledger, "interfaces", {}).items():
            i = getattr(iface, "peak_current_a", None)
            if i:
                peaks[sub_name] = float(i)
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
        for pat, sub in _NET_HINTS:
            if re.search(pat, name, re.I):
                if sub in peaks:
                    cur = peaks[sub]
                    src = f"ledger: {sub} declared peak {cur:g} A"
                    assumed = False          # a real declared number
                else:
                    src = f"name matches {sub} but no ledger peak declared — assumed 1 A"
                break
        if _HV_NAME.search(name):
            volt = 400.0
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
class DoctorReport:
    findings: list = field(default_factory=list)     # [Finding]
    fixes: list = field(default_factory=list)        # [TraceFix]
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
             fuse_sf: float = 2.0, dT_budget_c: float = 20.0) -> DoctorReport:
    """Run every check against the parsed board + the per-net current/voltage
    assignments. Returns findings (plain-language WHY + numeric FIX) and the
    executable width-fix list."""
    rep = DoctorReport()
    F = rep.findings

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
                    rep.fixes.append(TraceFix(
                        net=name, nid=nid, seg_index=i, old_width_mm=s.width_mm,
                        new_width_mm=math.ceil(r * 20) / 20.0,   # round up to 0.05
                        layer=s.layer, auto=not is_pair_member,
                        note=("diff-pair member — widening changes impedance; "
                              "prescribed only, use copper weight / paralleling"
                              if is_pair_member else "")))

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

        # -- true IR drop → brown-out ------------------------------------------------ #
        volt = float(a.get("voltage_v", rail_v) or rail_v)
        if nr["worst_r_ohm"] is not None and volt <= 60.0:
            drop = cur * nr["worst_r_ohm"]
            delivered = rail_v - drop
            pr = nr["worst_pair"] or ("?", "?")
            zone_note = (" (trace-only, pour on this net will lower it — "
                         "conservative)" if nr["has_zone"] else "")
            if delivered < brownout_v:
                F.append(Finding(
                    check=f"IR drop / brown-out — {name}", severity=Severity.FAIL,
                    subsystems=["electrics"],
                    message=(f"Nodal analysis of the routed copper on '{name}' "
                             f"gives {nr['worst_r_ohm']*1000:.0f} mΩ between "
                             f"{pr[0]} and {pr[1]}{zone_note}. At {cur:g} A that "
                             f"drops {drop:.2f} V — the {rail_v:g} V rail arrives "
                             f"at {delivered:.2f} V, below the {brownout_v:g} V "
                             f"brown-out. On track this is the ECU resetting "
                             f"exactly when the fans and brake light fire "
                             f"together — invisible on the bench at idle current. "
                             f"FIX: apply the width fixes below, add a pour, or "
                             f"feed {pr[1]} directly from the rail.")))
            elif drop > 0.25 * (rail_v - brownout_v):
                F.append(Finding(
                    check=f"IR drop — {name}", severity=Severity.WARN,
                    subsystems=["electrics"],
                    message=(f"'{name}' drops {drop:.2f} V at {cur:g} A over its "
                             f"real routed mesh ({nr['worst_r_ohm']*1000:.0f} mΩ, "
                             f"{pr[0]}→{pr[1]}){zone_note} — "
                             f"{drop/(rail_v-brownout_v)*100:.0f}% of the brown-out "
                             f"margin gone in copper before any connector or "
                             f"harness drop is counted.")))
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
def apply_fixes(board: PcbBoard, fixes: list) -> tuple:
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
    for fx in fixes:
        if not fx.auto:
            continue
        seg = board.segments[fx.seg_index]
        span = tuple(seg.width_span)
        if span == (0, 0):
            continue       # nothing to patch (synthetic segment)
        prev = per_span.get(span)
        if prev is None or fx.new_width_mm > prev[0]:
            per_span[span] = (fx.new_width_mm, seg)
        applied.append(fx)
    edits = sorted(((s[0], s[1], seg.width_token(w))
                    for s, (w, seg) in per_span.items()),
                   key=lambda e: e[0], reverse=True)
    text = board.text
    for a, b, rep in edits:
        text = text[:a] + rep + text[b:]
    return text, applied


def fix_report_md(board: PcbBoard, report: DoctorReport, applied: list,
                  assignments: dict) -> str:
    """A hand-off document: what was wrong, why it fails in real life, what was
    changed automatically, what still needs a human."""
    order = {"fail": 0, "warning": 1, "missing": 2, "info": 3, "ok": 4}
    eda = board.fmt_label
    lines = ["# PCB Doctor — fix report", "",
             f"Source board: **{eda}** · "
             f"{len(board.segments)} copper segments · {len(board.vias)} vias · "
             f"{len(board.footprints)} components", "",
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
    lines += ["", "---",
              "Analytic screening (IPC-2221 heating, Onderdonk fusing, nodal "
              "IR-drop, B4 clearance). Not a field solver, not a DRC replacement. "
              "Validate the patched board in KiCad DRC + your fab's rules before "
              "ordering."]
    return "\n".join(lines)


# --------------------------------------------------------------------------- #
#  Demo board — one click, three planted real-life failures
# --------------------------------------------------------------------------- #
def demo_kicad_pcb() -> str:
    """A small synthetic .kicad_pcb: an ECU board whose fan feed is under-sized
    and via-choked, whose CAN pair hugs the 400 V inverter sense net, and whose
    bulk cap sits on the hot copper — the three failures teams actually hit."""
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
    (pad "1" thru_hole circle (at 0 0) (size 1.7 1.7) (drill 1.0) (net 3 "CAN_H"))
    (pad "2" thru_hole circle (at 0 1.5) (size 1.7 1.7) (drill 1.0) (net 4 "CAN_L")))
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
  (segment (start 5 39.2) (end 70 39.2) (width 0.25) (layer "F.Cu") (net 5))
  (segment (start 70 10) (end 70 25) (width 0.5) (layer "F.Cu") (net 6))
)
"""


# --------------------------------------------------------------------------- #
#  SVG board viewer — failing copper glows red
# --------------------------------------------------------------------------- #
_LAYER_COLORS = {"F.Cu": "#d94f4f", "B.Cu": "#4a7bd9", "In1.Cu": "#3fae7a",
                 "In2.Cu": "#c9a03a", "In3.Cu": "#9a6ad1", "In4.Cu": "#48b8b8"}


def board_svg(board: PcbBoard, report=None, show_layers=None,
              width_px: int = 760, height_px: int = 440) -> str:
    """Inline SVG of the parsed copper: segments per layer, vias, component refs.
    Segments referenced by a pending fix are haloed red."""
    x0, y0, x1, y1 = board.bbox()
    w = max(x1 - x0, 1.0); h = max(y1 - y0, 1.0)
    pad = 26
    s = min((width_px - 2 * pad) / w, (height_px - 2 * pad) / h)

    def tx(p):
        return pad + (p[0] - x0) * s, pad + (p[1] - y0) * s

    bad = set()
    if report is not None:
        bad = {fx.seg_index for fx in report.fixes}
    layers = show_layers or board.copper_layers
    parts = [f'<svg viewBox="0 0 {width_px} {height_px}" '
             f'style="width:100%;height:auto;background:#0e1419;'
             f'border:1px solid var(--line);border-radius:8px;">']
    bar = min(20.0 * s, width_px * 0.4)
    parts.append(f'<line x1="{pad}" y1="{height_px-12}" x2="{pad+bar}" '
                 f'y2="{height_px-12}" stroke="#8d99a6" stroke-width="2"/>'
                 f'<text x="{pad}" y="{height_px-16}" fill="#8d99a6" '
                 f'font-size="10">{bar/s:.0f} mm</text>')
    for i, seg in enumerate(board.segments):
        if seg.layer not in layers:
            continue
        (ax, ay), (bx, by) = tx(seg.start), tx(seg.end)
        col = _LAYER_COLORS.get(seg.layer, "#7d8c99")
        sw = max(seg.width_mm * s, 1.2)
        if i in bad:
            parts.append(f'<line x1="{ax:.1f}" y1="{ay:.1f}" x2="{bx:.1f}" '
                         f'y2="{by:.1f}" stroke="#ff3333" '
                         f'stroke-width="{sw+5:.1f}" stroke-linecap="round" '
                         f'opacity="0.35"/>')
        parts.append(f'<line x1="{ax:.1f}" y1="{ay:.1f}" x2="{bx:.1f}" '
                     f'y2="{by:.1f}" stroke="{col}" stroke-width="{sw:.1f}" '
                     f'stroke-linecap="round" opacity="0.9"/>')
    for v in board.vias:
        vx, vy = tx(v.at)
        parts.append(f'<circle cx="{vx:.1f}" cy="{vy:.1f}" r="{max(v.size_mm*s/2,2):.1f}" '
                     f'fill="#e8edf2" stroke="#0e1419" stroke-width="1"/>')
    for fp in board.footprints:
        fx_, fy_ = tx(fp.at)
        parts.append(f'<rect x="{fx_-4:.1f}" y="{fy_-4:.1f}" width="8" height="8" '
                     f'fill="none" stroke="#e8edf2" stroke-width="1" rx="1.5"/>'
                     f'<text x="{fx_+6:.1f}" y="{fy_-5:.1f}" fill="#e8edf2" '
                     f'font-size="10" font-weight="700">{fp.ref}</text>')
    # legend
    lx = width_px - 120
    for i, ly in enumerate(layers):
        parts.append(f'<line x1="{lx}" y1="{16+i*14}" x2="{lx+18}" y2="{16+i*14}" '
                     f'stroke="{_LAYER_COLORS.get(ly, "#7d8c99")}" stroke-width="4"/>'
                     f'<text x="{lx+24}" y="{20+i*14}" fill="#8d99a6" '
                     f'font-size="10">{ly}</text>')
    parts.append("</svg>")
    return "".join(parts)
