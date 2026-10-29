"""suspension/_num.py: the shared float conversion."""
import numpy as np

from suspension._num import vec


def test_vec_makes_float_arrays_without_copying_one():
    a = vec([1, 2, 3])
    assert a.dtype == np.float64 and list(a) == [1.0, 2.0, 3.0]
    b = np.array([1.0, 2.0])
    assert vec(b) is b
