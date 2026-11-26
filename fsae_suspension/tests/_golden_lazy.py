"""Fail-fast companion to tests/_golden.py: `check_lazy(name, pairs)` takes an iterable of (case, value) pairs and
compares each with tests/golden/<name>.json as soon as it is produced, so a broken case fails without computing the
rest. Same file format, same float tolerance and same KINEMATIK_GOLDEN_RECORD=1 re-recording as `check`."""
import json
import os

from tests._golden import GOLDEN_DIR, _same, check, plain


def check_lazy(name, pairs, rel=1e-9, abs_=1e-12):
    path = GOLDEN_DIR / f"{name}.json"
    if os.environ.get("KINEMATIK_GOLDEN_RECORD") == "1" or not path.exists():
        return check(name, dict(pairs), rel, abs_)
    want = json.loads(path.read_text())
    seen = set()
    for k, v in pairs:
        assert k in want, f"case {k} not recorded"
        assert k not in seen, f"case {k} produced twice"
        _same(plain(v), want[k], k, rel, abs_)
        seen.add(k)
    missing = set(want) - seen
    assert not missing, f"cases {sorted(missing)[:5]} missing"
