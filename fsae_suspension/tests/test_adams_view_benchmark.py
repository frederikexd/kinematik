# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Supplement S6: KinematiK against Adams View 2026.1 on the paper's three corners. The Adams tables are the exports of
the runs; KinematiK must stay inside the limits set before those runs, and the command files must still be the ones
the manifests generate."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import adams_view_benchmark as avb                                        # noqa: E402


@pytest.mark.parametrize("name", avb.CORNERS)
def test_command_file_is_generated_from_the_manifest(name):
    assert (avb.BENCH / f"{name}.cmd").read_text() == avb.command_file(name)


@pytest.mark.parametrize("name", avb.CORNERS)
def test_kinematik_agrees_with_adams_view_within_the_preregistered_limits(name):
    r = avb.compare(name)
    for q in avb.QUANTITIES:
        assert r["worst"][q] <= avb.LIMITS[q], (name, q, r["worst"][q])


def test_limits_are_the_preregistered_ones():
    assert avb.LIMITS == {"camber": 0.01, "toe_in": 0.005, "caster": 0.01, "kpi": 0.01, "wc_dx": 0.05, "wc_dy": 0.05}


def test_a_table_exported_from_the_wrong_model_is_refused(tmp_path, monkeypatch):
    src = (avb.BENCH / "rear_v11_final.tab").read_text()
    (tmp_path / "front_v11_joints50.tab").write_text(src)
    monkeypatch.setattr(avb, "BENCH", tmp_path)
    with pytest.raises(ValueError, match="rear_v11_final"):
        avb.compare("front_v11_joints50")
