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
    return raw.decode("latin-1").replace(" ", "").lower()


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
                    if off < 0 or off + (count - 1) * stride + _NAME_LEN > len(data):
                        continue
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
            name = raw.decode("latin-1").strip()
            if name and _norm(raw) != f"{stem}{i + 1}":
                names[f"{prefix}{i + 1}"] = name
    return names


_PIN_REFS = [
    (re.compile(r"ECU[_ ]?MAIN\.(\d+)", re.I), MAIN),
    (re.compile(r"ECU[_ ]?COMM\.(\d+)", re.I), COMM),
    (re.compile(r"\bPin\s*M\s*-\s*(\d+)", re.I), MAIN),
]
_CLAIMS = [
    (re.compile(r"\b(?:DO|Digital\s+Output)\s*#?\s*(\d+)\b", re.I), "Digital Output #"),
    (re.compile(r"\b(?:INJ|Injector)\s*#?\s*(\d+)\b", re.I), "Injector #"),
    (re.compile(r"\b(?:COIL|Coil)\s*#?\s*(\d+)\b", re.I), "Coil #"),
]


def harness_pins(rows) -> dict[tuple[str, int], list[str]]:
    """Every ECU pin a wiring table references, with the text of the rows that mention it.

    `rows` is any iterable of rows (lists of cells). A pin is recognised as
    'ECU_MAIN.6', 'ECU_COMM.14' or 'Pin M-6' (spaces around the dash allowed).
    """
    found: dict[tuple[str, int], list[str]] = {}
    for row in rows:
        text = " | ".join(str(c) for c in row if c not in (None, ""))
        for rx, conn in _PIN_REFS:
            for m in rx.finditer(text):
                found.setdefault((conn, int(m.group(1))), []).append(text)
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
    return f"{pin[0]}-{pin[1]} ({PE3_84XX_PINOUT.get(pin, 'not in the PE3-84XX pinout')})"


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
    seen: dict[tuple[tuple[str, int], str], str] = {}
    for channel, name in sorted(names.items(), key=lambda kv: (kv[0].rstrip('0123456789'), int(re.sub(r'\D', '', kv[0])))):
        pin = pin_of(channel)
        if pin is None:
            continue
        if (pin, name) in seen:               # FREQ n is a mode of DI n: one pin, reported once
            continue
        seen[(pin, name)] = channel
        if channel.startswith("DI") and names.get("FREQ" + channel[2:]) == name:
            channel = f"{channel}/FREQ{channel[2:]}"
        if pin not in wired:
            rep.findings.append(Finding(
                "not wired", pin,
                f"tune names {channel} '{name}' ({_label(pin)}), but no wiring table wires that pin"))
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
        m = re.fullmatch(r"Digital Output #(\d+)", PE3_84XX_PINOUT.get(pin, ""))
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
        elif levels & {"check", "not wired"}:
            result = "check"
        elif "ok" in levels:
            result = "matches tune name"
        elif _function_words(func) & set().union(*(_function_words(x) for x in label.split(" / "))):
            result = "matches"
        else:
            result = "nothing to compare"
        rows.append(PinRow(f"{pin[0]}-{pin[1]}", func, label, result))
    return rows


def confirm_list(rep: EcuPinReport) -> list[str]:
    """One line per wired digital output: what to confirm in PE3Monitor before first power-up (unitless).

    Which function an output is set to switch is not readable from the tune (see CAVEAT), so a relay wired to an
    output nobody assigned (the pump that never primed) is caught by looking, and this says exactly where.
    """
    out = []
    for pin in sorted(rep.wired):
        m = re.fullmatch(r"Digital Output #(\d+)", PE3_84XX_PINOUT.get(pin, ""))
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
    body, small = ss["BodyText"], ss["BodyText"].clone("small", fontSize=7.5, leading=9)
    mark = {"mismatch": "MISMATCH", "not wired": "NOT WIRED", "check": "CHECK", "ok": "OK"}
    tint = {"mismatch": colors.HexColor("#f8d7da"), "not wired": colors.HexColor("#fff3cd"),
            "check": colors.HexColor("#fff3cd"), "ok": colors.HexColor("#d1e7dd")}
    story = [Paragraph("ECU pin check (PE3-84XX)", ss["Title"]),
             Paragraph(escape(f"KinematiK {rep.version}" if rep.version else "KinematiK"), body), Spacer(1, 4 * mm)]
    if rep.inputs:
        story.append(Paragraph("Inputs (SHA-256)", ss["Heading3"]))
        story.append(Table([[Paragraph(escape(n), small), Paragraph(d, small)] for n, d in rep.inputs],
                           colWidths=[62 * mm, 108 * mm]))
    story += [Paragraph("Tune channel names", ss["Heading3"]),
              Paragraph(escape(", ".join(f"{k} '{v}'" for k, v in sorted(rep.names.items())) or "none found"), body),
              Paragraph(f"ECU pins referenced by the wiring tables: {len(rep.wired)}", body),
              Paragraph("Findings", ss["Heading3"])]
    if rep.findings:
        t = Table([[Paragraph(mark[f.level], small), Paragraph(escape(f.message), body)] for f in rep.findings],
                  colWidths=[22 * mm, 148 * mm])
        t.setStyle(TableStyle([("BACKGROUND", (0, i), (0, i), tint[f.level]) for i, f in enumerate(rep.findings)]
                              + [("VALIGN", (0, 0), (-1, -1), "TOP"),
                                 ("LINEBELOW", (0, 0), (-1, -1), 0.25, colors.lightgrey)]))
        story.append(t)
    if not rep.problems:
        story.append(Paragraph("No disagreement between tune names, pinout and wiring tables.", body))
    rows = pin_rows(rep)
    if rows:
        story.append(Paragraph("Every wired pin", ss["Heading3"]))
        shade = {"mismatch": tint["mismatch"], "check": tint["check"], "matches": tint["ok"],
                 "matches tune name": tint["ok"]}
        data = [[Paragraph(f"<b>{h}</b>", small) for h in ("Pin", "PE3-84XX function", "Wiring tables say", "Result")]]
        data += [[Paragraph(r.pin, small), Paragraph(escape(r.function), small), Paragraph(escape(r.wiring), small),
                  Paragraph(r.result, small)] for r in rows]
        t = Table(data, colWidths=[18 * mm, 36 * mm, 86 * mm, 30 * mm], repeatRows=1)
        t.setStyle(TableStyle([("BACKGROUND", (3, i + 1), (3, i + 1), shade[r.result])
                               for i, r in enumerate(rows) if r.result in shade]
                              + [("VALIGN", (0, 0), (-1, -1), "TOP"),
                                 ("LINEBELOW", (0, 0), (-1, -1), 0.25, colors.lightgrey)]))
        story.append(t)
        story.append(Paragraph(escape(PIN_LEGEND), small))
    todo = confirm_list(rep)
    if todo:
        story.append(Paragraph("Confirm in PE3Monitor before first power-up", ss["Heading3"]))
        story.append(Paragraph("Which function an output switches is not readable from the tune, so check these by "
                               "eye. A relay wired to an output nobody assigned is caught here.", small))
        story += [Paragraph("[&nbsp;&nbsp;] " + escape(t), body) for t in todo]
    story += [Spacer(1, 4 * mm), Paragraph(CAVEAT, body),
              Paragraph("Pinout: Performance Electronics, PE3-84XX Pin Out.", small)]
    buf = io.BytesIO()
    SimpleDocTemplate(buf, pagesize=A4, leftMargin=20 * mm, rightMargin=20 * mm, topMargin=18 * mm,
                      bottomMargin=18 * mm, title="ECU pin check", author="KinematiK").build(story)
    return buf.getvalue()


def table_rows(name: str, data: bytes):
    """Rows of a wiring table given as file bytes: every sheet of an .xlsx/.xlsm, or a CSV (cell text, unitless)."""
    if name.lower().endswith((".xlsx", ".xlsm")):
        import openpyxl
        wb = openpyxl.load_workbook(io.BytesIO(data), read_only=True, data_only=True)
        for ws in wb.worksheets:
            yield from ws.iter_rows(values_only=True)
    else:
        yield from csv.reader(io.StringIO(data.decode("utf-8-sig", errors="replace")))


def run(tune: tuple[str, bytes], tables: list[tuple[str, bytes]]) -> EcuPinReport:
    """The whole check from file names and bytes, with each input's SHA-256 and the tool version recorded (unitless)."""
    names = read_ped_names(tune[1])
    wired: dict[tuple[str, int], list[str]] = {}
    for name, data in tables:
        for pin, texts in harness_pins(table_rows(name, data)).items():
            wired.setdefault(pin, []).extend(texts)
    rep = check(names, wired)
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
