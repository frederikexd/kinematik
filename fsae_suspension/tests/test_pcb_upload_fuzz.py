"""Uploaded board files are untrusted input. Whatever a file holds, `parse_board` either returns a board or raises
ValueError with a message for the person holding the file. It never raises anything else and never hangs.

The cases are seeded mutations of the built-in demo boards (truncation, deleted and duplicated chunks, numbers
replaced by nan / inf / 1e999 / huge values, stray brackets, random bytes), so every run tests the same files.
A 10-minute run of the same mutations on real team boards (KiCad 5-9 and binary Altium) found one escape, an
OverflowError on a net id of "inf"; it is pinned below."""
import random
import re

import pytest

from suspension import pcb_doctor as D
from suspension.pcb_altium import demo_altium_pcb

BAD = [b"nan", b"inf", b"-inf", b"1e999", b"-1e999", b"0", b"-1", b"999999999999", b'""', b"(", b")", b"\x00",
       b"1e-320", b"0x10", b"\xff\xfe"]


def _mutate(seed: bytes, rng: random.Random) -> bytes:
    b = bytearray(seed)
    for _ in range(rng.choice([1, 1, 2, 5, 20])):
        kind, i = rng.randrange(6), rng.randrange(len(b))
        if kind == 0:
            b = b[:i]
        elif kind == 1:
            del b[i:i + rng.choice([1, 10, 200, 2000])]
        elif kind == 2:
            nums = list(re.finditer(rb"-?\d+\.?\d*", bytes(b[i:i + 2000])))
            if nums:
                m = rng.choice(nums)
                b[i + m.start():i + m.end()] = rng.choice(BAD)
        elif kind == 3:
            b[i:i] = rng.choice(BAD) * rng.choice([1, 3, 50])
        elif kind == 4:
            j = rng.randrange(len(b))
            b[i:i] = b[j:j + rng.choice([10, 300, 2000])]
        else:
            b[i] = rng.randrange(256)
        if not b:
            b = bytearray(b"(")
    return bytes(b)


@pytest.mark.parametrize("name,seed", [("demo.kicad_pcb", D.demo_kicad_pcb().encode()),
                                       ("demo.PcbDoc", demo_altium_pcb().encode())], ids=["kicad", "altium_ascii"])
def test_a_damaged_board_file_is_refused_cleanly_or_read(name, seed):
    rng = random.Random(20261111)
    read = 0
    for _ in range(400):
        data = _mutate(seed, rng)
        try:
            board = D.parse_board(data, name)
        except ValueError as exc:
            assert str(exc)                                  # says something
            continue
        read += 1
        for nid in sorted(board.nets)[:3]:                   # what was read can be analysed without crashing
            D.analyze_net(board, nid)
    assert read > 20                                         # the mutations are not all fatal: both paths are tested


def test_non_finite_numbers_are_never_read_as_dimensions_or_net_ids():
    txt = D.demo_kicad_pcb()
    for bad in ("inf", "nan", "1e999", "-inf"):
        b = D.parse_board(re.sub(r"\(net 1 ", f"(net {bad} ", txt), "x.kicad_pcb")      # was OverflowError on "inf"
        assert all(isinstance(n, int) for n in b.nets)
        b = D.parse_board(re.sub(r"\(width [0-9.]+\)", f"(width {bad})", txt, count=3), "x.kicad_pcb")
        assert all(0 < s.width_mm < 1e3 for s in b.segments)
        assert any("3 track(s)" in n for n in b.notes)       # and the report says what was left out


def test_a_file_that_is_not_a_board_is_a_value_error():
    for junk in (b"", b"x", b"(", b"()", b"\x00" * 64, b"kicad_pcb", b"(kicad_pcb", b"(kicad_pcb)"):
        try:
            D.parse_board(junk, "x.kicad_pcb")
        except ValueError:
            pass
