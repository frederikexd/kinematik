"""Characterization (golden-master) helper for the mutation suites.

A case list is evaluated, every result is flattened to plain JSON (dataclasses, enums, numpy arrays, findings), and
compared field by field with a recorded file under tests/golden/. Floats compare at rel 1e-9 (NaN equals NaN, inf
equals inf). Record or refresh a file with KINEMATIK_GOLDEN_RECORD=1 after a deliberate behaviour change, and review
the diff: the file IS the specification of every message, branch and number the cases reach."""
import dataclasses
import enum
import json
import math
import os
from pathlib import Path

import numpy as np

GOLDEN_DIR = Path(__file__).parent / "golden"


def plain(x):
    if dataclasses.is_dataclass(x) and not isinstance(x, type):
        return {f.name: plain(getattr(x, f.name)) for f in dataclasses.fields(x)}
    if isinstance(x, enum.Enum):
        return plain(x.value)
    if isinstance(x, dict):
        return {str(k): plain(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [plain(v) for v in x]
    if isinstance(x, np.ndarray):
        return plain(x.tolist())
    if isinstance(x, (np.bool_, bool)):
        return bool(x)
    if isinstance(x, (np.integer,)):
        return int(x)
    if isinstance(x, (float, np.floating)):
        f = float(x)
        if math.isnan(f):
            return "NaN"
        if math.isinf(f):
            return "inf" if f > 0 else "-inf"
        return f
    if x is None or isinstance(x, (int, str)):
        return x
    return repr(x)


def _same(a, b, path, rel=1e-9, abs_=1e-12):
    if isinstance(a, bool) or isinstance(b, bool):
        if type(a) is not type(b) or a != b:
            raise AssertionError(f"{path}: {a!r} != {b!r}")
        return
    if isinstance(a, float) or isinstance(b, float):
        if not (isinstance(a, (int, float)) and isinstance(b, (int, float))):
            raise AssertionError(f"{path}: {a!r} != {b!r}")
        if not math.isclose(a, b, rel_tol=rel, abs_tol=abs_):
            raise AssertionError(f"{path}: {a!r} != {b!r}")
        return
    if isinstance(a, dict) and isinstance(b, dict):
        if set(a) != set(b):
            raise AssertionError(f"{path}: keys {sorted(set(a) ^ set(b))} differ")
        for k in a:
            _same(a[k], b[k], f"{path}.{k}", rel, abs_)
        return
    if isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            raise AssertionError(f"{path}: length {len(a)} != {len(b)}")
        for i, (u, v) in enumerate(zip(a, b)):
            _same(u, v, f"{path}[{i}]", rel, abs_)
        return
    if a != b or type(a) is not type(b):
        raise AssertionError(f"{path}: {a!r} != {b!r}")


def check(name, results, rel=1e-9, abs_=1e-12):
    """`results` maps a case label to any value; compares with tests/golden/<name>.json (floats at `rel`/`abs_`)."""
    got = plain(results)
    path = GOLDEN_DIR / f"{name}.json"
    if os.environ.get("KINEMATIK_GOLDEN_RECORD") == "1" or not path.exists():
        if os.environ.get("KINEMATIK_GOLDEN_RECORD") != "1":
            raise AssertionError(f"no golden file {path}; record it with KINEMATIK_GOLDEN_RECORD=1")
        GOLDEN_DIR.mkdir(exist_ok=True)
        path.write_text(json.dumps(got, indent=1, sort_keys=True, ensure_ascii=False))
        return
    want = json.loads(path.read_text())
    for k in want:
        assert k in got, f"case {k} missing"
    for k in got:
        assert k in want, f"case {k} not recorded"
        _same(got[k], want[k], k, rel, abs_)
