# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""
Native (binary) Altium `.PcbDoc` reader — read-only, on purpose.

Altium saves binary by default, so this is the file an FSAE electrical member
actually has on disk. Requiring them to re-export as ASCII before the Doctor
would say anything was the single biggest reason for a team to bounce off the
tool, so this module reads the native file directly.

**It reads. It does not write.** Diagnosis is recoverable if it goes wrong: a
bad parse shows up immediately as absurd geometry or garbage net names, and
`_sanity_check` below refuses the file rather than reporting on it. Patching is
not recoverable — a mistake corrupts a board a team is about to pay to
fabricate. So a binary board carries `patchable = False`, the UI offers the
prescriptions but not the one-click re-trace, and mending still routes through
the ASCII export. That asymmetry is deliberate and is the whole design.

Format notes, all verified against KiCad's own Altium importer
(`pcbnew/pcb_io/altium/`, GPL-2.0-or-later — compatible with this project's
AGPL-3.0; the layouts below were written from that reference, not copied):

  * The file is an OLE compound document. Each object class lives in a storage
    (`Tracks6`, `Vias6`, `Pads6`, `Arcs6`, `Nets6`, `Components6`, `Polygons6`)
    holding one `Data` stream.
  * Geometry streams are packed binary records: a 1-byte record type, a 4-byte
    subrecord length, then a fixed field layout.
  * `Nets6`, `Components6` and `Polygons6` are *not* binary — they are blocks of
    the very same `|KEY=VALUE|` text the ASCII exporter writes, length-prefixed.
    So those three reuse the ASCII field reader verbatim.
  * The internal unit is 1/10000 mil = 2.54 nm exactly.
  * Y is negated on import, matching the ASCII path and the KiCad convention
    the rest of the Doctor assumes.
  * Net indices here are plain 0-based array indices into `Nets6` — the ASCII
    format's ID-vs-file-order ambiguity does not exist in the binary.
"""

from __future__ import annotations

import math
import struct

from .pcb_doctor import (PcbBoard, PcbSegment, PcbVia, PcbPad, PcbFootprint,
                         PcbZone,
                         DEFAULT_BOARD_THICKNESS_MM,
                         _arc_points_center, _chord_segments)
from . import pcb_altium as _ascii

#: Altium's internal unit: 1/10000 mil, i.e. 2.54 nm, exactly.
INTERNAL_MM = 2.54e-6

#: Storage names holding the object classes the Doctor needs.
STREAMS = ("Board6", "Nets6", "Components6", "Polygons6",
           "Tracks6", "Arcs6", "Vias6", "Pads6")


class AltiumBinaryUnavailable(RuntimeError):
    """Raised when `olefile` is missing, so the caller can fall back to the
    ASCII instructions instead of showing a stack trace."""


# --------------------------------------------------------------------------- #
#  Layer enum -> the canonical KiCad names the physics speaks
# --------------------------------------------------------------------------- #
#  1 = Top, 2..31 = Mid 1..30, 32 = Bottom, 39.. = Internal Plane 1..16,
#  74 = Multi-layer. Anything else is not copper and must never reach the
#  ampacity mesh.
#
#  Only the v6 layer byte is used. Records also carry a 32-bit `layer_v7`, but
#  on real files that field holds a packed value (0x01000001, 0x0102000F, ...)
#  rather than a plain enum, and trusting it silently discarded every copper
#  track on the first board tested. v6 covers codes 1-74, which is every layer
#  this tool reasons about, and an unrecognised code is skipped rather than
#  guessed at.
MULTILAYER = _ascii.MULTILAYER


def layer_from_code(code: int):
    if code == 1:
        return "F.Cu"
    if 2 <= code <= 31:
        return f"In{code - 1}.Cu"
    if code == 32:
        return "B.Cu"
    if 39 <= code <= 54:
        return f"In{code - 38}.Cu"          # internal planes
    if code == 74:
        return MULTILAYER
    return None


def _is_plane_code(code: int) -> bool:
    return 39 <= code <= 54


# --------------------------------------------------------------------------- #
#  Record cursor
# --------------------------------------------------------------------------- #
class _Cursor:
    """Byte cursor over one `Data` stream, with Altium's subrecord framing.

    Every record is self-framing: type byte, 4-byte length, body. Reading by
    frame rather than by field offset means a record longer than expected (a
    newer Altium writing extra trailing fields) is skipped cleanly instead of
    desynchronising the whole stream — which is how a parser built on fixed
    offsets turns one unknown field into thousands of garbage objects.
    """

    __slots__ = ("buf", "pos", "end")

    def __init__(self, buf: bytes):
        self.buf, self.pos, self.end = buf, 0, len(buf)

    def remaining(self) -> int:
        return self.end - self.pos

    def u8(self) -> int:
        v = self.buf[self.pos]
        self.pos += 1
        return v

    def _unpack(self, fmt: str, n: int):
        v = struct.unpack_from(fmt, self.buf, self.pos)[0]
        self.pos += n
        return v

    def u16(self):
        return self._unpack("<H", 2)

    def u32(self):
        return self._unpack("<I", 4)

    def i32(self):
        return self._unpack("<i", 4)

    def f64(self):
        return self._unpack("<d", 8)

    def mm(self) -> float:
        return self.i32() * INTERNAL_MM

    def pos_xy(self) -> tuple:
        """A coordinate pair, with Y negated into the KiCad convention."""
        x = self.mm()
        y = self.mm()
        return (x, -y)

    def size_xy(self) -> tuple:
        return (self.mm(), self.mm())

    def skip(self, n: int):
        self.pos += n

    def subrecord(self) -> tuple:
        """Read a subrecord header. Returns (body_start, body_end)."""
        n = self.u32()
        start = self.pos
        return start, min(start + n, self.end)


def _iter_records(buf: bytes, want_type: int):
    """Yield (cursor, body_start, body_end) for each record of `want_type`.

    A record whose type byte is wrong means the stream is not what we think it
    is, so we stop rather than reinterpret the rest of the file as garbage.
    """
    c = _Cursor(buf)
    while c.remaining() >= 5:
        rtype = c.u8()
        if rtype != want_type:
            return
        start, end = c.subrecord()
        if end <= start or end > c.end:
            return
        yield c, start, end
        c.pos = end


def _iter_property_blocks(buf: bytes):
    """Yield `|KEY=VALUE|` text for each length-prefixed property block.

    Nets6/Components6/Polygons6 store the exact text the ASCII exporter writes,
    so these feed straight into the ASCII field reader and the two front-ends
    stay one implementation rather than two that can disagree.
    """
    pos, n = 0, len(buf)
    while pos + 4 <= n:
        length = struct.unpack_from("<I", buf, pos)[0]
        pos += 4
        if length & 0xFF000000:                     # binary block: skip it
            length &= 0x00FFFFFF
            pos += length
            continue
        if length == 0 or pos + length > n:
            return
        raw = buf[pos:pos + length]
        pos += length
        yield raw.rstrip(b"\x00").decode("latin-1", errors="replace")


# --------------------------------------------------------------------------- #
#  The reader
# --------------------------------------------------------------------------- #
def is_available() -> bool:
    try:
        import olefile           # noqa: F401
        return True
    except ImportError:
        return False


def parse_altium_binary(data: bytes) -> PcbBoard:
    """Parse a native binary `.PcbDoc` into the same `PcbBoard` both other
    readers produce. Read-only: the result has `patchable = False`."""
    try:
        import olefile
    except ImportError as exc:                      # pragma: no cover
        raise AltiumBinaryUnavailable(
            "reading native binary .PcbDoc needs the `olefile` package "
            "(pip install olefile). Until it is installed, re-save the board "
            "from Altium as 'PCB ASCII File' and drop that in instead.") from exc

    import io
    if not olefile.isOleFile(io.BytesIO(data)):
        raise ValueError("not an OLE compound document")
    ole = olefile.OleFileIO(io.BytesIO(data))

    def stream(name: str) -> bytes:
        path = f"{name}/Data"
        try:
            if not ole.exists(path):
                return b""
            with ole.openstream(path) as fh:
                return fh.read()
        except Exception:                           # noqa: BLE001
            return b""

    board = PcbBoard(text="", fmt="altium_binary", length_unit="mm")
    board.patchable = False

    # ---- nets: property blocks, plain 0-based array order -------------------- #
    net_names = []
    for txt in _iter_property_blocks(stream("Nets6")):
        f = _ascii._fields(txt, 0)
        net_names.append((_ascii._g(f, "NAME", "NETNAME") or "").strip())
    board.nets = {0: ""}
    for i, nm in enumerate(net_names):
        board.nets[i + 1] = nm

    def net_id(idx) -> int:
        # 0xFFFF / -1 is Altium's "no net"
        return (idx + 1) if (idx is not None and 0 <= idx < len(net_names)) else 0

    # ---- components: property blocks ---------------------------------------- #
    comps = []
    for txt in _iter_property_blocks(stream("Components6")):
        f = _ascii._fields(txt, 0)
        x = _ascii._len_mm(_ascii._g(f, "X"), "mil") or 0.0
        y = _ascii._len_mm(_ascii._g(f, "Y"), "mil") or 0.0
        lay = _ascii.altium_layer_to_kicad(_ascii._g(f, "LAYER"))
        comps.append(PcbFootprint(
            ref=(_ascii._g(f, "SOURCEDESIGNATOR", "DESIGNATOR",
                           "NAME") or "?").strip(),
            value=(_ascii._g(f, "COMMENT", "SOURCECOMMENT", "VALUE") or "").strip(),
            layer=lay if lay in ("F.Cu", "B.Cu") else "F.Cu",
            at=(x, -y)))
    loose = PcbFootprint(ref="?", value="free pads (no component)",
                         layer="F.Cu", at=(0.0, 0.0))

    found, plane_nets = set(), set()

    # ---- tracks (record type 4) --------------------------------------------- #
    for c, start, end in _iter_records(stream("Tracks6"), 4):
        if end - start < 33:
            continue
        c.pos = start
        code = c.u8()
        c.skip(2)                                   # flags1, flags2
        nid = net_id(c.u16())
        c.skip(2)                                   # polygon
        c.skip(2)                                   # component
        c.skip(4)
        p1 = c.pos_xy()
        p2 = c.pos_xy()
        w = c.mm()
        layer = layer_from_code(code)
        if layer is None or layer == MULTILAYER or w <= 0:
            continue
        found.add(layer)
        board.segments.append(PcbSegment(
            net=nid, layer=layer, width_mm=w, start=p1, end=p2))

    # ---- arcs (record type 1) ------------------------------------------------ #
    for c, start, end in _iter_records(stream("Arcs6"), 1):
        if end - start < 45:
            continue
        c.pos = start
        code = c.u8()
        c.skip(2)
        nid = net_id(c.u16())
        c.skip(2 + 2 + 4)
        cx, cy = c.pos_xy()                         # note: cy already negated
        r = c.mm()
        a0, a1 = c.f64(), c.f64()
        w = c.mm()
        layer = layer_from_code(code)
        if layer is None or layer == MULTILAYER or w <= 0 or r <= 0:
            continue
        found.add(layer)
        # angles are CCW in Altium's Y-up frame; generate there, then flip Y
        pts = [(px, -py) for px, py in
               _arc_points_center(cx, -cy, r, a0, a1)]
        board.segments.extend(_chord_segments(
            pts, net=nid, layer=layer, width_mm=w))

    # ---- vias (record type 3) ------------------------------------------------ #
    for c, start, end in _iter_records(stream("Vias6"), 3):
        if end - start < 31:
            continue
        c.pos = start
        c.skip(1 + 2)                               # unknown, flags1, flags2
        nid = net_id(c.u16())
        c.skip(8)
        at = c.pos_xy()
        dia = c.mm()
        hole = c.mm()
        top = layer_from_code(c.u8()) or "F.Cu"
        bot = layer_from_code(c.u8()) or "B.Cu"
        if top == MULTILAYER:
            top = "F.Cu"
        if bot == MULTILAYER:
            bot = "B.Cu"
        found.update({top, bot})
        board.vias.append(PcbVia(net=nid, at=at, size_mm=dia or 0.6,
                                 drill_mm=hole or 0.3, layers=(top, bot)))

    # ---- pads (record type 2): name in subrecord 1, geometry in subrecord 5 -- #
    #  A pad record does NOT end after subrecord 5 — real files carry further
    #  subrecords for pad modes, and how many varies. Walking "five subrecords
    #  then next record" therefore lands mid-stream and the loop dies after the
    #  first pad, which is not an error anyone sees: it silently yields a board
    #  with one pad, every net looks padless, and the open-copper check then
    #  passes trivially. So record starts are *found*, not assumed.
    #
    #  The signature is strong: type byte 0x02, a subrecord-1 length L (a short
    #  designator, 1..64 bytes), and the string-length prefix inside it that
    #  must equal L-1. False positives are essentially impossible.
    pbuf = stream("Pads6")

    def _pad_starts(buf: bytes):
        out, i, n = [], 0, len(buf)
        while i < n - 6:
            if buf[i] == 2:
                L = struct.unpack_from("<I", buf, i + 1)[0]
                if 1 <= L <= 64 and i + 5 + L <= n and buf[i + 5] == L - 1:
                    out.append(i)
                    i += 5 + L
                    continue
            i += 1
        return out

    c = _Cursor(pbuf)
    for _start in _pad_starts(pbuf):
        c.pos = _start
        c.u8()                                      # record type, already known
        s1, e1 = c.subrecord()                      # name (length-prefixed str)
        c.pos = s1
        name = "?"
        if e1 > s1:
            n = c.u8()
            name = c.buf[c.pos:c.pos + n].decode("latin-1", "replace") or "?"
        c.pos = e1
        ok = True
        for _ in range(3):                          # subrecords 2, 3, 4
            if c.remaining() < 4:
                ok = False
                break
            s, e = c.subrecord()
            c.pos = e
        if not ok or c.remaining() < 4:
            continue
        s5, e5 = c.subrecord()
        if e5 - s5 < 49:
            continue
        c.pos = s5
        code = c.u8()
        c.skip(2)
        nid = net_id(c.u16())
        c.skip(2)
        comp_i = c.u16()
        c.skip(4)
        at = c.pos_xy()
        topsize = c.size_xy()
        c.size_xy()                                 # midsize
        c.size_xy()                                 # botsize
        hole = c.mm()
        layer = layer_from_code(code)
        through = (layer == MULTILAYER) or hole > 0.0
        pad = PcbPad(number=name, net=nid, net_name=board.nets.get(nid, ""),
                     at=at, size=topsize or (1.0, 1.0), through=through,
                     layer=(layer if layer in ("F.Cu", "B.Cu") else ""))
        (comps[comp_i] if comp_i < len(comps) else loose).pads.append(pad)
        if layer in ("F.Cu", "B.Cu"):
            found.add(layer)

    # ---- polygons: property blocks, pours only (see the ASCII reader) -------- #
    for txt in _iter_property_blocks(stream("Polygons6")):
        f = _ascii._fields(txt, 0)
        raw = _ascii._g(f, "NET")
        idx = _ascii._int(raw)
        nid = net_id(idx) if idx is not None else 0
        if nid:
            board.zone_nets.add(nid)
        lay = _ascii.altium_layer_to_kicad(_ascii._g(f, "LAYER"))
        if lay and lay != MULTILAYER:
            found.add(lay)
            if _ascii._is_plane(_ascii._g(f, "LAYER")):
                plane_nets.add(nid)
            if nid:
                # The binary Polygons6 blocks are the same |KEY=VALUE| text the
                # ASCII exporter writes, so the outline reader is shared.
                o = _ascii.outline_from_fields(f, "mil")
                if len(o) >= 3:
                    board.zones.append(PcbZone(net=nid, layer=lay, outline=o))
    board.zone_nets |= {n for n in plane_nets if n}

    # ---- board thickness ------------------------------------------------------ #
    for txt in _iter_property_blocks(stream("Board6")):
        th = _ascii._len_mm(_ascii._g(_ascii._fields(txt, 0),
                                      "BOARDTHICKNESS", "THICKNESS"), "mil")
        if th and 0.1 < th < 10.0:
            board.board_thickness_mm = th
        break

    board.footprints = [c_ for c_ in comps if c_.pads or c_.ref != "?"]
    if loose.pads:
        board.footprints.append(loose)
    board.copper_layers = _ascii._order_layers(found)
    board.native_layers = {"F.Cu": "Top Layer", "B.Cu": "Bottom Layer"}
    for l in board.copper_layers:
        if l.startswith("In") and l.endswith(".Cu"):
            board.native_layers[l] = f"Mid Layer {l[2:-3]}"

    board.notes.append(
        "Read directly from the native binary .PcbDoc. The Doctor will "
        "diagnose it but will NOT patch it: rewriting widths inside a binary "
        "board risks corrupting a file you are about to send to a fab. For the "
        "one-click re-trace, re-save from Altium as 'PCB ASCII File'.")
    if board.board_thickness_mm == DEFAULT_BOARD_THICKNESS_MM:
        board.notes.append(
            f"Board thickness not found in the file — assumed "
            f"{DEFAULT_BOARD_THICKNESS_MM:g} mm, which only affects via-barrel "
            f"length.")
    _sanity_check(board)
    return board


def _sanity_check(board: PcbBoard):
    """Refuse a board whose numbers cannot be true.

    A binary parser that drifts by a byte does not crash — it yields confident
    nonsense. These are the cheap invariants that catch that, and refusing is
    the right response because the ASCII export is always available as a
    correct second opinion.
    """
    if not board.segments and not board.vias and not board.footprints:
        raise ValueError(
            "no copper, vias or components found in this .PcbDoc. It may be a "
            "library or an Altium version this reader does not understand — "
            "re-save it as 'PCB ASCII File' and drop that in instead.")
    if board.segments or board.vias:
        x0, y0, x1, y1 = board.bbox()
        span = max(x1 - x0, y1 - y0)
        if span > 2000.0 or (span and span < 0.5):
            raise ValueError(
                f"this board measures {span:,.1f} mm across, which is not a "
                f"PCB — the binary reader has mis-parsed it. Nothing here is "
                f"trustworthy, so it is refused rather than reported on. "
                f"Re-save from Altium as 'PCB ASCII File' and drop that in.")
    widths = [s.width_mm for s in board.segments]
    if widths and (min(widths) <= 0 or max(widths) > 100.0):
        raise ValueError(
            f"trace widths run from {min(widths):.3f} to {max(widths):.1f} mm, "
            f"which is not real copper — the binary reader has mis-parsed this "
            f"file. Re-save from Altium as 'PCB ASCII File' instead.")
