# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""
ECU pin check: a PE3 tune's channel names against the harness that wires them.

A team names its outputs and inputs in PE3Monitor ("Fan" on Digital Output #1,
"MPH" on Digital Input #2) and draws the harness in a separate wiring table.
Nothing ties the two together, so a renamed output, a speed sensor on a pin the
loom never wires, or a table that says DO9 where the pinout puts DO10 survives
until the car is on the bench. This module reads the names stored in a `.ped`
tune, maps each channel to its connector pin with Performance Electronics'
published PE3-84XX pinout, finds every ECU pin a wiring table references, and
reports where the three disagree.

What it reads and what it does not:
  * Channel names are stored as plain 8-character text in the `.ped` file and
    are read exactly. They are labels the tuner chose: the manual's own example
    is naming Digital Output #10 "Fuel" when it drives the fuel pump relay.
  * The settings behind a name (fan on/off temperatures, pump prime, input
    modes) are binary fields this module does not decode. A matching name is
    evidence of intent, not proof of behaviour.
  * A channel still carrying its default name ("DO #3") is treated as unused.

Pinout source: Performance Electronics, "PE3-84XX Pin Out",
https://pe-ltd.com/assets/PE3-84XX_Pin_Out.pdf. Frequency inputs #1-#4 are a
mode of Digital Inputs #1-#4 (PE3 Series User Manual), so they share those pins.
"""
from __future__ import annotations

import csv
import hashlib
import io
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

MAIN, COMM = "MAIN", "COMM"

# (connector, pin) -> function, verbatim from the PE3-84XX pinout.
PE3_84XX_PINOUT: dict[tuple[str, int], str] = {
    (MAIN, 1): "Coil #1", (MAIN, 2): "Digital Output #2", (MAIN, 3): "Injector #1",
    (MAIN, 4): "Coil #2", (MAIN, 5): "Digital Output #5", (MAIN, 6): "Digital Output #10",
    (MAIN, 7): "Digital Output #6", (MAIN, 8): "Analog Input #5", (MAIN, 9): "GND",
    (MAIN, 10): "Injector #2", (MAIN, 11): "Digital Output #1", (MAIN, 12): "MAP Signal",
    (MAIN, 13): "Digital Input #2", (MAIN, 14): "Digital Input #1", (MAIN, 15): "Analog Input #8",
    (MAIN, 16): "+5V Supply", (MAIN, 17): "Tach Out", (MAIN, 18): "GND",
    (MAIN, 19): "Digital Output #3", (MAIN, 20): "GND", (MAIN, 21): "Digital Output #4",
    (MAIN, 22): "Digital Output #7", (MAIN, 23): "Analog Input #6", (MAIN, 24): "Digital Output #8",
    (MAIN, 25): "Digital Output #9", (MAIN, 26): "5V GND", (MAIN, 27): "Digital Input #6",
    (MAIN, 28): "Air Temp Signal", (MAIN, 29): "Coolant Temp Signal", (MAIN, 30): "TPS Signal",
    (MAIN, 31): "Analog Input #7", (MAIN, 32): "Crank Signal", (MAIN, 33): "Cam Signal",
    (MAIN, 34): "Battery V+",
    (COMM, 1): "Ethernet 01", (COMM, 2): "Ethernet 02", (COMM, 3): "Digital Input #4",
    (COMM, 4): "Digital Input #5", (COMM, 5): "Analog Input #1", (COMM, 6): "Analog Input #2",
    (COMM, 7): "CAN Bus Lo", (COMM, 8): "CAN Bus Hi", (COMM, 9): "Stepper Motor",
    (COMM, 10): "Injector #7", (COMM, 11): "Injector #5", (COMM, 12): "Injector #3",
    (COMM, 13): "Coil #3", (COMM, 14): "Ethernet 14", (COMM, 15): "Ethernet 15",
    (COMM, 16): "Digital Input #3", (COMM, 17): "Digital Input #7", (COMM, 18): "Analog Input #3",
    (COMM, 19): "Analog Input #4", (COMM, 20): "Stepper Motor", (COMM, 21): "Stepper Motor",
    (COMM, 22): "Stepper Motor", (COMM, 23): "Injector #8", (COMM, 24): "Injector #6",
    (COMM, 25): "Injector #4", (COMM, 26): "Coil #4",
}

PED_MAGIC = b"Performance Electronics File"

# channel family -> (prefix in a channel id, default-name stem, count, pinout function stem)
_FAMILIES = {
    "DI": ("DI", "di#", 7, "Digital Input #"),
    "DO": ("DO", "do#", 10, "Digital Output #"),
    "AN": ("AN", "an#", 8, "Analog Input #"),
    "FREQ": ("FREQ", "freq#", 4, "Digital Input #"),   # frequency mode of DI #1-#4
}
_NAME_LEN = 8
_STRIDES = range(_NAME_LEN, 21)


def pin_of(channel: str) -> tuple[str, int] | None:
    """Connector pin of a channel id such as 'DO10', 'DI2', 'AN5' or 'FREQ2' (unitless: connector and pin number)."""
    m = re.fullmatch(r"(DI|DO|AN|FREQ)(\d+)", channel)
    if not m:
        return None
    _, _, _, stem = _FAMILIES[m.group(1)]
    want = f"{stem}{int(m.group(2))}"
    for pin, func in PE3_84XX_PINOUT.items():
        if func == want:
            return pin
    return None


def _norm(raw: bytes) -> str:
    return raw.decode("latin-1").replace(" ", "").lower()  # pragma: no mutate (codec names ignore case)


def _printable(raw: bytes) -> bool:
    return all(32 <= c < 127 for c in raw)


def _find_block(data: bytes, stem: str, count: int) -> tuple[int, int] | None:
    """Offset and stride of the name block whose default names fit best (at least 3)."""
    best, best_hits = None, 2
    first = f"{stem}".encode()
    starts = {m.start() for m in re.finditer(re.escape(first).replace(b"\\#", b"\\s*#"), data, re.I)}
    for hit in starts:
        for stride in _STRIDES:
            for k in range(count):
                for back in range(_NAME_LEN):
                    off = hit - back - k * stride
                    if off < 0 or off + (count - 1) * stride + _NAME_LEN > len(data):  # pragma: no mutate (a window past either end holds fewer default names, never more)
                        continue  # pragma: no mutate (the same guard)
                    fields = [data[off + i * stride: off + i * stride + _NAME_LEN] for i in range(count)]
                    if not all(_printable(f) for f in fields):
                        continue
                    hits = sum(_norm(f) == f"{stem}{i + 1}" for i, f in enumerate(fields))
                    if hits > best_hits:
                        best, best_hits = (off, stride), hits
    return best


def read_ped_names(data: bytes) -> dict[str, str]:
    """Custom channel names in a PE3 `.ped` tune: {'DO1': 'Fan', 'DI2': 'MPH', ...}.

    Channels still carrying their default name, or a blank one, are left out. Names only, so unitless.
    Raises ValueError for a file that is not a Performance Electronics tune.
    """
    if not data.startswith(PED_MAGIC):
        raise ValueError("not a Performance Electronics .ped file")
    names: dict[str, str] = {}
    for fam, (prefix, stem, count, _) in _FAMILIES.items():
        block = _find_block(data, stem, count)
        if block is None:
            continue
        off, stride = block
        for i in range(count):
            raw = data[off + i * stride: off + i * stride + _NAME_LEN]
            name = raw.decode("latin-1").strip()  # pragma: no mutate (codec names ignore case)
            if name and _norm(raw) != f"{stem}{i + 1}":
                names[f"{prefix}{i + 1}"] = name
    return names


_PIN_REFS = [
    (re.compile(r"ECU[_ ]?MAIN\.(\d+)", re.I), MAIN),
    (re.compile(r"ECU[_ ]?COMM\.(\d+)", re.I), COMM),
    (re.compile(r"\bPin\s*M\s*-\s*(\d+)", re.I), MAIN),
    # Connector named by its size, as harness tools export it: 'ECU 34-Pin.M06' (MAIN), 'ECU 26-Pin.C14' (COMM).
    (re.compile(r"\b34[-\s]?Pin\s*\.?\s*M?0*(\d+)", re.I), MAIN),
    (re.compile(r"\b26[-\s]?Pin\s*\.?\s*C?0*(\d+)", re.I), COMM),
]
_CLAIMS = [
    (re.compile(r"\b(?:DO|Digital\s+Output)\s*#?\s*(\d+)\b", re.I), "Digital Output #"),
    # Not 'COIL1.2' / 'INJ1.2': that is a pin of the part's own connector (COIL1, pin 2), as in
    # 'Coil #2.COIL1.2', not a claim about which coil it is.
    (re.compile(r"\b(?:INJ|Injector)\s*#?\s*(\d+)\b(?!\.\d)", re.I), "Injector #"),
    (re.compile(r"\b(?:COIL|Coil)\s*#?\s*(\d+)\b(?!\.\d)", re.I), "Coil #"),
]


def harness_pins(rows) -> dict[tuple[str, int], list[str]]:
    """Every ECU pin a wiring table references, with the text of the rows that mention it.

    `rows` is any iterable of rows (lists of cells). A pin is recognised as
    'ECU_MAIN.6', 'ECU_COMM.14', 'Pin M-6' (spaces around the dash allowed), or by connector size as
    'ECU 34-Pin.M06' (MAIN) and 'ECU 26-Pin.C14' (COMM). A row naming the same pin twice is listed once.
    """
    found: dict[tuple[str, int], list[str]] = {}
    for row in rows:
        text = " | ".join(str(c) for c in row if c not in (None, ""))
        pins = {(conn, int(m.group(1))) for rx, conn in _PIN_REFS for m in rx.finditer(text)}
        for pin in sorted(pins):
            found.setdefault(pin, []).append(text)
    return found


@dataclass
class Finding:
    level: str          # "mismatch", "not wired", "check" or "ok"
    pin: tuple[str, int] | None
    message: str


@dataclass
class EcuPinReport:
    names: dict[str, str]
    wired: dict[tuple[str, int], list[str]]
    findings: list[Finding] = field(default_factory=list)
    inputs: list[tuple[str, str]] = field(default_factory=list)    # (file name, SHA-256)
    version: str = ""

    @property
    def problems(self) -> list[Finding]:
        """Every finding that is not 'ok' (unitless)."""
        return [f for f in self.findings if f.level != "ok"]


def _words(text: str) -> set[str]:
    return {w for w in re.findall(r"[a-z]+", text.lower()) if len(w) >= 3}


def _shares_word(name: str, texts: list[str]) -> bool:
    mine, theirs = _words(name), set().union(*(_words(t) for t in texts))
    return any(a.startswith(b) or b.startswith(a) for a in mine for b in theirs)


def _label(pin: tuple[str, int]) -> str:
    return f"{pin[0]}-{pin[1]} ({PE3_84XX_PINOUT[pin]})"           # only ever called for a pin the pinout has


def check(names: dict[str, str], wired: dict[tuple[str, int], list[str]]) -> EcuPinReport:
    """Compare tune names, the pinout and the wiring tables; one finding per disagreement (unitless: names and pins)."""
    rep = EcuPinReport(names, wired)
    for pin, texts in sorted(wired.items()):
        func = PE3_84XX_PINOUT.get(pin)
        if func is None:
            rep.findings.append(Finding("mismatch", pin, f"{pin[0]}-{pin[1]} is wired but the PE3-84XX has no such pin"))
            continue
        for text in texts:
            if len(set(harness_pins([[text]]))) != 1:
                continue                      # a row naming two ECU pins cannot say which one it describes
            for rx, stem in _CLAIMS:
                for m in rx.finditer(text):
                    claimed = f"{stem}{int(m.group(1))}"
                    if claimed != func:
                        rep.findings.append(Finding(
                            "mismatch", pin,
                            f"wiring table calls {_label(pin)} '{m.group(0)}', but the pinout puts "
                            f"{claimed} on {next((f'{c}-{n}' for (c, n), f in PE3_84XX_PINOUT.items() if f == claimed), 'no pin')}"))
    seen: set[tuple[tuple[str, int], str]] = set()
    for channel, name in sorted(names.items(), key=lambda kv: (re.match(r'\D+', kv[0]).group(), int(re.sub(r'\D', '', kv[0])))):
        pin = pin_of(channel)
        if pin is None:  # pragma: no mutate (every channel read from a tune has a pin)
            continue  # pragma: no mutate (the same guard)
        if (pin, name) in seen:               # FREQ n is a mode of DI n: one pin, reported once
            continue  # pragma: no mutate (FREQ sorts last and carries one custom name at most, so nothing follows)
        seen.add((pin, name))
        if channel.startswith("DI") and names.get("FREQ" + channel[2:]) == name:
            channel = f"{channel}/FREQ{channel[2:]}"
        if pin not in wired:
            rep.findings.append(Finding(
                "not wired", pin,
                f"tune names {channel} '{name}' ({_label(pin)}), but no wiring table wires that pin: a missing "
                f"wire, or an unused name left over from a base tune?"))
        elif _shares_word(name, wired[pin]):
            rep.findings.append(Finding("ok", pin, f"{channel} '{name}' on {_label(pin)} matches the wiring table"))
        else:
            rep.findings.append(Finding(
                "check", pin,
                f"tune names {channel} '{name}' on {_label(pin)}, but the wiring table rows for that pin share no word with it"))
    # The other direction: an output the harness drives that the tune never set up. A pump relay wired to DO10 while
    # DO10 still carries its default name is the case where the pump function was never assigned in PE3Monitor.
    # Names are what can be read, so this is a prompt to confirm the assignment, not proof it is missing.
    for pin in sorted(wired):
        m = re.fullmatch(r"Digital Output #(\d+)", PE3_84XX_PINOUT.get(pin, ""))  # pragma: no mutate (any default that is no output name)
        if m and f"DO{m.group(1)}" not in names:
            rep.findings.append(Finding(
                "check", pin,
                f"the wiring tables drive {_label(pin)}, but the tune has not named DO{m.group(1)}: confirm in "
                f"PE3Monitor that a function is assigned to that output"))
    return rep


def ped_diff(a: bytes, b: bytes) -> list[tuple[int, int, int]]:
    """(offset, byte in a, byte in b) for every byte that differs between two tunes of the same length (unitless).

    The assignment of a function (fuel pump, fan) to an output is stored in an undocumented binary field. Two
    copies of one tune that differ in exactly one setting, such as the pump moved from DO10 to DO9, locate it:
    the field is wherever they differ. The trailer's save date is excluded, since it changes on every save.
    """
    if len(a) != len(b):
        raise ValueError(f"tunes differ in length ({len(a)} vs {len(b)} bytes): not two saves of one tune layout")
    stamp = a.rfind(b"Date:")
    end = len(a) if stamp < 0 else stamp
    return [(i, a[i], b[i]) for i in range(end) if a[i] != b[i]]


_SIGNAL_RE = re.compile(r"Signal:\s*([^;|]+)", re.I)
_PART_RE = re.compile(r"\b(K\d+\s*-\s*[A-Za-z][A-Za-z &/]*?)\.\d+")
_GENERIC = {"signal", "digital", "output", "input", "analog", "out"}
_SYNONYMS = {"gnd": "ground"}


def wiring_label(texts: list[str]) -> str:
    """What the wiring tables say is on a pin: their 'Signal:' notes and relay names, else the first row."""
    seen: list[str] = []
    for t in texts:
        for m in [*_SIGNAL_RE.finditer(t), *_PART_RE.finditer(t)]:
            label = " ".join(m.group(1).split())
            if label and label not in seen:
                seen.append(label)
    return " / ".join(seen) if seen else " ".join(texts[0].split())[:80]


def _function_words(text: str) -> set[str]:
    return {_SYNONYMS.get(w, w) for w in _words(text)} - _GENERIC


@dataclass
class PinRow:
    pin: str            # 'MAIN-6'
    function: str       # the PE3-84XX pinout's name for the pin
    wiring: str         # what the wiring tables say is on it
    result: str         # 'matches', 'matches tune name', 'mismatch', 'check' or 'nothing to compare'


PIN_LEGEND = ("'matches' means the wiring tables' label for the pin uses the pinout's word for it (Coil, Injector, "
              "MAP...); 'matches tune name' means it uses the tune's name for that output; 'nothing to compare' "
              "means the tables give no label to compare.")


def pin_rows(rep: EcuPinReport) -> list[PinRow]:
    """Every pin the wiring tables wire, with the pinout's function, the tables' label and how they compare (unitless)."""
    rows = []
    for pin in sorted(rep.wired):
        func = PE3_84XX_PINOUT.get(pin, "not in the PE3-84XX pinout")
        label = wiring_label(rep.wired[pin])
        levels = {f.level for f in rep.findings if f.pin == pin}
        if "mismatch" in levels:
            result = "mismatch"
        elif "check" in levels:               # a pin with a "not wired" finding is, by definition, not here
            result = "check"
        elif "ok" in levels:
            result = "matches tune name"
        elif _function_words(func) & _function_words(label):
            result = "matches"
        else:
            result = "nothing to compare"
        rows.append(PinRow(f"{pin[0]}-{pin[1]}", func, label, result))
    return rows


#: Performance Electronics' own advice (support reply, 2026-11-23): names in the tune don't prove a relay is wired
#: to that output, so switch each output from the software and watch the relay. Shown after the per-output lines.
OUTPUT_DIAGNOSTICS = ("Then, per Performance Electronics: name every digital output under Engine > Setup Digital Names "
                      "(that is what this check reads from the tune), and switch each one from the Output Diagnostics "
                      "window to confirm the relay wired to it actually clicks. A name in the tune does not prove the "
                      "physical wiring; switching the output does.")


def confirm_list(rep: EcuPinReport) -> list[str]:
    """One line per wired digital output: what to confirm in PE3Monitor before first power-up (unitless).

    Which function an output is set to switch is not readable from the tune (see CAVEAT), so a relay wired to an
    output nobody assigned (the pump that never primed) is caught by looking, and this says exactly where.
    """
    out = []
    for pin in sorted(rep.wired):
        m = re.fullmatch(r"Digital Output #(\d+)", PE3_84XX_PINOUT.get(pin, ""))  # pragma: no mutate (any default that is no output name)
        if m:
            n = m.group(1)
            named = f" (named '{rep.names[f'DO{n}']}' in the tune)" if f"DO{n}" in rep.names else ""
            out.append(f"PE3Monitor > Tuning > Digital Outputs: DO{n}{named} on {pin[0]}-{pin[1]} is set to "
                       f"switch what the wiring tables put on it: {wiring_label(rep.wired[pin])}")
    return out


def format_report(rep: EcuPinReport) -> str:
    out = ["ECU pin check (PE3-84XX)" + (f", KinematiK {rep.version}" if rep.version else ""), ""]
    if rep.inputs:
        out.append("Inputs (SHA-256):")
        out += [f"  {name}  {digest}" for name, digest in rep.inputs]
        out.append("")
    out.append("Tune channel names: " + (", ".join(f"{k} '{v}'" for k, v in sorted(rep.names.items())) or "none found"))
    out.append(f"ECU pins referenced by the wiring tables: {len(rep.wired)}")
    out.append("")
    for f in rep.findings:
        out.append(f"[{f.level}] {f.message}")
    if not rep.problems:
        out.append("No disagreement between tune names, pinout and wiring tables.")
    rows = pin_rows(rep)
    if rows:
        out += ["", "Every wired pin:", f"  {'pin':<9} {'PE3-84XX function':<20} {'result':<18} wiring tables say"]
        out += [f"  {r.pin:<9} {r.function:<20} {r.result:<18} {r.wiring}" for r in rows]
        out.append(f"  ({PIN_LEGEND})")
    todo = confirm_list(rep)
    if todo:
        out += ["", "Confirm in PE3Monitor before first power-up (not readable from the tune):"]
        out += [f"  [ ] {t}" for t in todo]
        out.append(f"  [ ] {OUTPUT_DIAGNOSTICS}")
    out += ["", CAVEAT]
    return "\n".join(out)


CAVEAT = ("Only the channel names in the tune were read, not the settings behind them (thresholds, priming, "
          "input modes). A match means the names and the wiring agree, not that the outputs behave correctly.")


def report_pdf(rep: EcuPinReport) -> bytes:
    """The same report as format_report, as a one-page A4 PDF (reportlab)."""
    from xml.sax.saxutils import escape

    from reportlab.lib import colors
    from reportlab.lib.pagesizes import A4
    from reportlab.lib.styles import getSampleStyleSheet
    from reportlab.lib.units import mm
    from reportlab.platypus import Paragraph, SimpleDocTemplate, Spacer, Table, TableStyle

    ss = getSampleStyleSheet()
    body, small = ss["BodyText"], ss["BodyText"].clone("small", fontSize=7.5, leading=9)  # pragma: no mutate (paragraph style and page setup: the default style, sheet-name case, A4 and a one-page margin read the same; the text and layout are pinned)
    mark = {"mismatch": "MISMATCH", "not wired": "NOT WIRED", "check": "CHECK", "ok": "OK"}
    # pragma: no mutate start (PDF layout: shading, rules and spacing; the text is checked word for word)
    tint = {"mismatch": colors.HexColor("#f8d7da"), "not wired": colors.HexColor("#fff3cd"),
            "check": colors.HexColor("#fff3cd"), "ok": colors.HexColor("#d1e7dd")}
    gap = Spacer(1, 4 * mm)
    # pragma: no mutate end
    story = [Paragraph("ECU pin check (PE3-84XX)", ss["Title"]),  # pragma: no mutate (paragraph style and page setup: the default style, sheet-name case, A4 and a one-page margin read the same; the text and layout are pinned)
             Paragraph(escape(f"KinematiK {rep.version}" if rep.version else "KinematiK"), body), gap]
    if rep.inputs:
        story.append(Paragraph("Inputs (SHA-256)", ss["Heading3"]))
        story.append(Table([[Paragraph(escape(n), small), Paragraph(d, small)] for n, d in rep.inputs],
                           colWidths=[62 * mm, 108 * mm]))
    story += [Paragraph("Tune channel names", ss["Heading3"]),  # pragma: no mutate (paragraph style and page setup: the default style, sheet-name case, A4 and a one-page margin read the same; the text and layout are pinned)
              Paragraph(escape(", ".join(f"{k} '{v}'" for k, v in sorted(rep.names.items())) or "none found"), body),
              Paragraph(f"ECU pins referenced by the wiring tables: {len(rep.wired)}", body),
              Paragraph("Findings", ss["Heading3"])]
    if rep.findings:
        t = Table([[Paragraph(mark[f.level], small), Paragraph(escape(f.message), body)] for f in rep.findings],  # pragma: no mutate (paragraph style and page setup: the default style, sheet-name case, A4 and a one-page margin read the same; the text and layout are pinned)
                  colWidths=[22 * mm, 148 * mm])
        # pragma: no mutate start (PDF layout: shading, rules and spacing; the text is checked word for word)
        t.setStyle(TableStyle([("BACKGROUND", (0, i), (0, i), tint[f.level]) for i, f in enumerate(rep.findings)]
                              + [("VALIGN", (0, 0), (-1, -1), "TOP"),
                                 ("LINEBELOW", (0, 0), (-1, -1), 0.25, colors.lightgrey)]))
        # pragma: no mutate end
        story.append(t)
    if not rep.problems:
        story.append(Paragraph("No disagreement between tune names, pinout and wiring tables.", body))
    rows = pin_rows(rep)
    if rows:
        story.append(Paragraph("Every wired pin", ss["Heading3"]))
        # pragma: no mutate start (PDF layout: shading, rules and spacing; the text is checked word for word)
        shade = {"mismatch": tint["mismatch"], "check": tint["check"], "matches": tint["ok"],
                 "matches tune name": tint["ok"]}
        # pragma: no mutate end
        data = [[Paragraph(f"<b>{h}</b>", small) for h in ("Pin", "PE3-84XX function", "Wiring tables say", "Result")]]
        data += [[Paragraph(r.pin, small), Paragraph(escape(r.function), small), Paragraph(escape(r.wiring), small),
                  Paragraph(r.result, small)] for r in rows]
        t = Table(data, colWidths=[18 * mm, 36 * mm, 86 * mm, 30 * mm], repeatRows=1)
        # pragma: no mutate start (PDF layout: shading, rules and spacing; the text is checked word for word)
        t.setStyle(TableStyle([("BACKGROUND", (3, i + 1), (3, i + 1), shade[r.result])
                               for i, r in enumerate(rows) if r.result in shade]
                              + [("VALIGN", (0, 0), (-1, -1), "TOP"),
                                 ("LINEBELOW", (0, 0), (-1, -1), 0.25, colors.lightgrey)]))
        # pragma: no mutate end
        story.append(t)
        story.append(Paragraph(escape(PIN_LEGEND), small))
    todo = confirm_list(rep)
    if todo:
        story.append(Paragraph("Confirm in PE3Monitor before first power-up", ss["Heading3"]))
        story.append(Paragraph("Which function an output switches is not readable from the tune, so check these by "
                               "eye. A relay wired to an output nobody assigned is caught here.", small))
        story += [Paragraph("[&nbsp;&nbsp;] " + escape(t), body) for t in todo]
        story.append(Paragraph("[&nbsp;&nbsp;] " + escape(OUTPUT_DIAGNOSTICS), body))
    story += [gap, Paragraph(CAVEAT, body),
              Paragraph("Pinout: Performance Electronics, PE3-84XX Pin Out.", small)]
    buf = io.BytesIO()
    SimpleDocTemplate(buf, pagesize=A4, leftMargin=20 * mm, rightMargin=20 * mm, topMargin=18 * mm,  # pragma: no mutate (paragraph style and page setup: the default style, sheet-name case, A4 and a one-page margin read the same; the text and layout are pinned)
                      bottomMargin=18 * mm, title="ECU pin check", author="KinematiK").build(story)
    return buf.getvalue()


def table_rows(name: str, data: bytes):
    """Rows of a wiring table given as file bytes: every sheet of an .xlsx/.xlsm, or a CSV (cell text, unitless)."""
    if name.lower().endswith((".xlsx", ".xlsm")):
        import openpyxl
        wb = openpyxl.load_workbook(io.BytesIO(data), data_only=True, read_only=True)  # pragma: no mutate (read_only is speed only; data_only is pinned by a formula cell in the tests)
        for ws in wb.worksheets:
            yield from ws.iter_rows(values_only=True)
    else:
        yield from csv.reader(io.StringIO(data.decode(
            "utf-8-sig",  # pragma: no mutate (codec names ignore case)
            errors="replace")))


_RELAY_PIN = re.compile(r"^\s*(K\d+)\s*-\s*(.+?)\.(30|85|86|87a?)\s*$", re.I)
_HEADER_WORDS = {"id", "color", "gauge", "from", "to", "destination", "cavity", "net", "part number", "ident"}
_RELAY_PIN_NAMED = re.compile(r"^\s*(.*?Relay)\s*#?\s*(\d+)\.(30|85|86|87a?)\s*$", re.I)


def _relay_pin(node: str):
    """(relay id 'K5', its name, pin '86') for a relay pin node: 'K5 - Fan.86' or 'Radiator Fan Relay #5.86'
    (unitless)."""
    m = _RELAY_PIN.match(node)
    if m:
        return m.group(1).upper(), m.group(2).strip(), m.group(3).lower()
    m = _RELAY_PIN_NAMED.match(node)
    if m:
        return f"K{int(m.group(2))}", m.group(1).strip(), m.group(3).lower()
    return None


def relay_chains(rows) -> list[Finding]:
    """Relay coils fed from another relay's switched output, read from a wiring table's From/To columns (unitless).

    A coil (85/86) on a net that a different relay's 87/87a switches can only energise while that other relay is
    on: K5 (fan) fed from S13, the net K2 (injectors) switches. It may work, as it did on Calvin's car, but it is a
    dependency nobody drew on purpose more often than not, so it is reported as a check, not a fault. Nets are built
    by joining every row's From and To; a splice (S13) joins the rows that meet at it.
    """
    parent: dict[str, str] = {}

    def _find(x):
        while parent.setdefault(x, x) != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    # Each sheet has its own header row. A workbook's later sheets (per-connector cavity lists, per-splice lists)
    # use other columns, so From/To are taken from the latest header and ignored under a header without them.
    fi = ti = None
    for r in rows:
        cells = [str(c).strip().lower() if c is not None else "" for c in r]  # pragma: no mutate (an empty cell need only be no header word)
        if len(_HEADER_WORDS & set(cells)) >= 3:
            fi, ti = (cells.index("from"), cells.index("to")) if {"from", "to"} <= set(cells) else (None, None)  # pragma: no mutate (a header row has three header words, so the subset is never equal)
            continue
        if fi is None:
            continue
        a = str(r[fi]).strip() if fi < len(r) and r[fi] is not None else ""
        b = str(r[ti]).strip() if ti < len(r) and r[ti] is not None else ""
        if a and b:
            parent[_find(a)] = _find(b)
    nets: dict[str, list[str]] = {}
    for node in list(parent):
        nets.setdefault(_find(node), []).append(node)
    feeds: dict[str, list[tuple[str, str, str, str]]] = {}      # coil relay -> (coil pin, source relay, its pin, name)
    names: dict[str, str] = {}
    for members in nets.values():
        pins = [(m, _relay_pin(m)) for m in members]
        for m, x in pins:
            if x:
                names[x[0]] = x[1]
        outputs = [(m, y) for m, y in pins if y and y[2] in ("87", "87a")]
        for m, x in pins:
            if x and x[2] in ("85", "86"):
                for o, y in outputs:
                    if y[0] != x[0]:
                        feeds.setdefault(x[0], []).append((x[2], y[0], y[2], y[1]))
    # A coil fed from the main relay is ordinary design. Flag only a coil fed from a relay that is itself switched by
    # another relay (K5 <- K2 <- K1): the device then depends on a relay that has nothing to do with it.
    out = []
    for k, srcs in sorted(feeds.items()):
        for coil_pin, src, src_pin, src_name in srcs:
            if src in feeds:
                up = feeds[src][0][1]
                out.append(Finding(
                    "check", None,
                    f"relay {k} ({names[k]}) coil {coil_pin} is fed from {src} ({src_name}) switched output "
                    f"{src_pin}, and {src} is itself switched by {up}: {k} can only switch while {src} is on. "
                    f"Intended, or feed it from {up}'s net like the others?"))
    return out


def run(tune: tuple[str, bytes], tables: list[tuple[str, bytes]]) -> EcuPinReport:
    """The whole check from file names and bytes, with each input's SHA-256 and the tool version recorded (unitless)."""
    names = read_ped_names(tune[1])
    wired: dict[tuple[str, int], list[str]] = {}
    chains: list[Finding] = []
    for name, data in tables:
        rows = list(table_rows(name, data))
        for pin, texts in harness_pins(rows).items():
            wired.setdefault(pin, []).extend(texts)
        chains += relay_chains(rows)
    rep = check(names, wired)
    rep.findings += chains
    rep.inputs = [(n, hashlib.sha256(d).hexdigest()) for n, d in [tune, *tables]]
    from suspension.consistency_check import tool_version
    rep.version = tool_version()
    return rep


def main(argv: list[str] | None = None) -> int:
    """Command line: the check, or --diff of two tunes. Exit code 1 when the check finds a problem (unitless)."""
    args = sys.argv[1:] if argv is None else argv
    if len(args) == 3 and args[0] == "--diff":
        rows = ped_diff(Path(args[1]).read_bytes(), Path(args[2]).read_bytes())
        for off, x, y in rows:
            print(f"0x{off:05x}  {x:3d} -> {y:3d}")
        print(f"{len(rows)} byte(s) differ")
        return 0
    if len(args) < 2:
        print("usage: python -m suspension.ecu_pins TUNE.ped WIRING_TABLE.(xlsx|csv) [MORE_TABLES ...]\n"
              "       python -m suspension.ecu_pins --diff TUNE_A.ped TUNE_B.ped")
        return 2
    files = [(Path(a).name, Path(a).read_bytes()) for a in args]
    rep = run(files[0], files[1:])
    print(format_report(rep))
    return 1 if rep.problems else 0


if __name__ == "__main__":
    raise SystemExit(main())
