"""suspension/_num.py: the shared float conversion."""
import numpy as np

from suspension._num import vec


def test_vec_makes_float_arrays_without_copying_one():
    a = vec([1, 2, 3])
    assert a.dtype == np.float64 and list(a) == [1.0, 2.0, 3.0]
    b = np.array([1.0, 2.0])
    assert vec(b) is b


# ---- finite / finite_scalar / all_finite / json_finite: every guard, and the exact words it uses ----
import dataclasses
import io

import pytest

from suspension._num import all_finite, finite, finite_scalar, json_finite


def _says(exc_info, text):
    assert str(exc_info.value) == text


def test_finite_takes_one_value_by_default_and_refuses_none():
    assert list(finite("x", [5.0])) == [5.0]
    with pytest.raises(ValueError) as e:
        finite("x", [])
    _says(e, "x: needs at least 1 value(s), got 0")


def test_finite_messages():
    with pytest.raises(ValueError) as e:
        finite("x", ["a"])
    _says(e, "x: not a list of numbers")
    with pytest.raises(ValueError) as e:
        finite("x", [1.0, float("nan"), float("inf")])
    _says(e, "x: 2 of 3 value(s) are not finite numbers (nan or inf)")
    with pytest.raises(ValueError) as e:
        finite("x", [1.0, -7.5], max_abs=5.0)
    _says(e, "x: a value of -7.5 is beyond ±5")
    assert list(finite("x", [5.0, -5.0], max_abs=5.0)) == [5.0, -5.0]
    with pytest.raises(ValueError) as e:
        finite("x", [1.0], min_len=2)
    _says(e, "x: needs at least 2 value(s), got 1")


def test_finite_scalar_messages():
    assert finite_scalar("m", "2.5") == 2.5
    assert finite_scalar("m", 0.0) == 0.0
    with pytest.raises(ValueError) as e:
        finite_scalar("m", "abc")
    _says(e, "m: not a number")
    with pytest.raises(ValueError) as e:
        finite_scalar("m", float("inf"))
    _says(e, "m: inf is not a finite number")
    with pytest.raises(ValueError) as e:
        finite_scalar("m", 0.0, positive=True)
    _says(e, "m: must be above zero, got 0")
    assert finite_scalar("m", 1e-9, positive=True) == 1e-9


@dataclasses.dataclass
class _Res:
    a: float
    b: list


def test_all_finite_every_kind():
    for ok in (None, 3, True, "nan", b"inf", 1.5, np.float32(2), 1 + 2j, np.array([1, 2]), np.array([1.0]),
               {"a": [1.0, (2.0,)]}, {1.0, 2.0}, _Res(1.0, [2.0]), object()):
        assert all_finite(ok) is True, ok
    for bad in (float("nan"), np.float64("inf"), np.array([1.0, np.nan]), {"a": [1.0, (float("inf"),)]},
                [1.0, float("nan")], (float("nan"),), {float("nan")}, _Res(1.0, [float("nan")])):
        assert all_finite(bad) is False, bad


def test_json_finite_reads_text_bytes_files_and_refuses_nan():
    assert json_finite('{"a": 1}') == {"a": 1}
    assert json_finite(b'\xef\xbb\xbf{"a": 2}') == {"a": 2}
    assert json_finite(bytearray(b'[3]')) == [3]
    assert json_finite(io.StringIO('{"a": 4}')) == {"a": 4}
    for tok in ("NaN", "Infinity", "-Infinity"):
        with pytest.raises(ValueError) as e:
            json_finite(f'{{"a": {tok}}}')
        _says(e, f"the file holds {tok}, which is not a finite number")
