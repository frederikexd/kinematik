# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""Tests for what the project store actually persists.

Two regressions, both silent, both data loss:

1. `integration_document` was set as an attribute by _persist_doc_ledger() but
   never listed in _payload(). save() returned True, so the app told the member
   "committed to the Integration Document — everyone in the workspace can see
   it" for a write that never happened. The season-long cross-team deliverable
   reset on every restart.

2. `as_json()` was a second, hand-maintained field list that drifted from
   _payload(). Since apply_project_bundle() feeds as_json()'s output back
   through _apply(), restoring your own backup deleted every stamped report.
"""
import dataclasses
import json
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from suspension import project as pj                      # noqa: E402


def _store():
    fd, path = tempfile.mkstemp(suffix=".json")
    os.close(fd)
    os.unlink(path)
    return pj.ProjectStore(path), path


def _a_report():
    from suspension.report_store import ReportRecord
    return ReportRecord(**{
        f.name: (f.default if f.default is not dataclasses.MISSING else "x")
        for f in dataclasses.fields(ReportRecord)})


LEDGER = {"kinematics": {"label": "Kinematics", "subsystem": "suspension",
                         "committed_on": "2026-08-05 09:00",
                         "md": "# Kinematics\n- camber gain -0.20"}}


# --- 1. the Integration Document must survive a restart --------------------
def test_integration_document_is_persisted():
    s, path = _store()
    s.integration_document = LEDGER
    assert s.save() is True
    assert pj.ProjectStore(path).integration_document == LEDGER


def test_integration_document_is_in_the_payload():
    """Pinned by name: the bug was an attribute nothing serialized."""
    s, _ = _store()
    assert "integration_document" in s._payload()


def test_commit_then_reload_keeps_every_feature():
    s, path = _store()
    s.integration_document = dict(LEDGER)
    s.save()
    s2 = pj.ProjectStore(path)
    s2.integration_document["brakes"] = {"label": "Brakes",
                                         "subsystem": "brakes",
                                         "committed_on": "x", "md": "# b"}
    s2.save()
    assert sorted(pj.ProjectStore(path).integration_document) == \
        ["brakes", "kinematics"]


# --- 2. as_json must not drift from _payload ------------------------------
def test_as_json_matches_the_persisted_shape():
    """The two field lists are now one. A field added to persistence is in the
    export by construction, which is the only durable fix for this class."""
    s, _ = _store()
    exported = set(json.loads(s.as_json()))
    persisted = set(s._payload()) - {"updated"}
    assert exported == persisted


def test_as_json_omits_the_locking_baseline():
    s, _ = _store()
    assert "updated" not in json.loads(s.as_json())


def test_restoring_your_own_backup_keeps_reports():
    s, _ = _store()
    s.reports = [_a_report()]
    s._apply(json.loads(s.as_json()))          # what apply_project_bundle does
    assert len(s.reports) == 1


def test_restoring_your_own_backup_keeps_the_integration_document():
    s, _ = _store()
    s.integration_document = LEDGER
    s._apply(json.loads(s.as_json()))
    assert s.integration_document == LEDGER


# --- 3. absent key != empty value -----------------------------------------
def test_bundle_without_reports_key_does_not_wipe_them():
    """An older bundle, or one from an external tool, carries no "reports".
    That means "not carried", not "there are none"."""
    s, _ = _store()
    s.reports = [_a_report()]
    s._apply({"team_name": "Elbee Racing", "weights": []})
    assert len(s.reports) == 1


def test_explicit_empty_list_still_clears():
    """The guard must not make a genuine clear impossible."""
    s, _ = _store()
    s.reports = [_a_report()]
    s._apply({"reports": []})
    assert s.reports == []


def test_bundle_without_integration_document_does_not_wipe_it():
    s, _ = _store()
    s.integration_document = LEDGER
    s._apply({"team_name": "Elbee Racing"})
    assert s.integration_document == LEDGER


# --- 4. nothing else regressed --------------------------------------------
def test_ordinary_fields_still_round_trip():
    s, path = _store()
    s.team_name = "Elbee Racing"
    s.target_mass_kg = 210.0
    s.add_weight(pj.WeightItem("suspension", "upright", mass_g=850, qty=4))
    s.ledger = {"interfaces": {"susp->chassis": "M8 shear"}}
    s.save()
    s2 = pj.ProjectStore(path)
    assert s2.team_name == "Elbee Racing"
    assert s2.target_mass_kg == 210.0
    assert len(s2.weights) == 1
    assert s2.ledger["interfaces"]["susp->chassis"] == "M8 shear"
