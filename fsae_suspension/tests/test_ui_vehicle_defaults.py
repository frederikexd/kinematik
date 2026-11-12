"""A workspace saved with a blank vehicle field (None) once crashed every page through the car-basics summary.
Blank or non-numeric fields now take the defaults, and the summary shows a dash instead of raising."""
import os

import pytest

pytest.importorskip("streamlit")
from streamlit.testing.v1 import AppTest  # noqa: E402

_APP = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "streamlit_app.py")


def test_blank_vehicle_fields_take_the_defaults():
    at = AppTest.from_file(_APP, default_timeout=600)
    at.run()
    vp = dict(at.session_state.vp)
    vp.update(mass=None, cg_height="abc", weight_dist_front=None)
    at.session_state.vp = vp
    at.run()
    assert not at.exception, [str(e.value) for e in at.exception]
    assert isinstance(at.session_state.vp["mass"], float)
    assert isinstance(at.session_state.vp["cg_height"], float)
    assert isinstance(at.session_state.vp["weight_dist_front"], float)
