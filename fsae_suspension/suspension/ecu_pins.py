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
    """Connector pin of a channel id such as 'DO10', 'DI2', 'AN5' or 'FREQ2'."""
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

    Channels still carrying their default name, or a blank one, are left out.
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

    @property
    def problems(self) -> list[Finding]:
        return [f for f in self.findings if f.level != "ok"]


def _words(text: str) -> set[str]:
    return {w for w in re.findall(r"[a-z]+", text.lower()) if len(w) >= 3}


def _shares_word(name: str, texts: list[str]) -> bool:
    mine, theirs = _words(name), set().union(*(_words(t) for t in texts))
    return any(a.startswith(b) or b.startswith(a) for a in mine for b in theirs)


def _label(pin: tuple[str, int]) -> str:
    return f"{pin[0]}-{pin[1]} ({PE3_84XX_PINOUT.get(pin, 'not in the PE3-84XX pinout')})"


def check(names: dict[str, str], wired: dict[tuple[str, int], list[str]]) -> EcuPinReport:
    """Compare tune names, the pinout and the wiring tables; one finding per disagreement."""
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
    return rep


def format_report(rep: EcuPinReport) -> str:
    out = ["ECU pin check (PE3-84XX)", ""]
    out.append("Tune channel names: " + (", ".join(f"{k} '{v}'" for k, v in sorted(rep.names.items())) or "none found"))
    out.append(f"ECU pins referenced by the wiring tables: {len(rep.wired)}")
    out.append("")
    for f in rep.findings:
        out.append(f"[{f.level}] {f.message}")
    if not rep.problems:
        out.append("No disagreement between tune names, pinout and wiring tables.")
    out += ["", "Names are labels set in PE3Monitor; the output settings behind them (thresholds, priming, input "
            "modes) are not read, so a match shows consistent naming and wiring, not behaviour."]
    return "\n".join(out)


def _rows_from(path: Path):
    if path.suffix.lower() in (".xlsx", ".xlsm"):
        import openpyxl
        wb = openpyxl.load_workbook(path, read_only=True, data_only=True)
        for ws in wb.worksheets:
            yield from ws.iter_rows(values_only=True)
    else:
        with open(path, newline="", encoding="utf-8-sig") as fh:
            yield from csv.reader(fh)


def main(argv: list[str] | None = None) -> int:
    args = sys.argv[1:] if argv is None else argv
    if len(args) < 2:
        print("usage: python -m suspension.ecu_pins TUNE.ped WIRING_TABLE.(xlsx|csv) [MORE_TABLES ...]")
        return 2
    names = read_ped_names(Path(args[0]).read_bytes())
    wired: dict[tuple[str, int], list[str]] = {}
    for table in args[1:]:
        for pin, texts in harness_pins(_rows_from(Path(table))).items():
            wired.setdefault(pin, []).extend(texts)
    rep = check(names, wired)
    print(format_report(rep))
    return 1 if rep.problems else 0


if __name__ == "__main__":
    raise SystemExit(main())
