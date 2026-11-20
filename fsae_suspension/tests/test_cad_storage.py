# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""CAD library bytes move to Storage on save, so the project document (and every
history copy of it) carries only a pointer; locally they stay embedded."""
import base64
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from suspension import project as pm  # noqa: E402

WS = "0b1c2d3e-4f50-6172-8394-a5b6c7d8e9f0"


class _Bucket:
    def __init__(self, fail=False):
        self.objects, self.fail, self.downloads = {}, fail, 0

    def upload(self, path, file, file_options=None):
        if self.fail:
            raise RuntimeError("policy denied")
        self.objects[path] = bytes(file)

    def download(self, path):
        self.downloads += 1
        return self.objects[path]


class _Backend:
    def __init__(self):
        self.written = None
        self.ctx = type("C", (), {"workspace_id": WS, "access_token": "t"})()
        self.project_id = "default"

    def read(self):
        return {}

    def write(self, payload, expected_version=None):
        self.written = payload


@pytest.fixture(autouse=True)
def _clean_cache():
    pm._CAD_BYTES_CACHE.clear()
    yield
    pm._CAD_BYTES_CACHE.clear()


def _store(bucket):
    s = pm.ProjectStore(backend=_Backend())
    s._cad_bucket = lambda: bucket
    return s


def _cad(raw=b"solid part\x00\x01", name="front upright.STEP"):
    return pm.CADFile(name=name, kind="file", data_b64=base64.b64encode(raw).decode(),
                      size_bytes=len(raw))


def test_save_moves_bytes_to_storage_and_keeps_only_a_pointer():
    b = _Bucket()
    s = _store(b)
    c = _cad()
    s.add_cad_file(c)
    assert s.save()
    row = s.backend.written["cad_files"][0]
    assert row["data_b64"] == "" and row["storage_path"].startswith(WS + "/")
    assert row["storage_path"].endswith("front-upright.STEP")
    assert b.objects[row["storage_path"]] == b"solid part\x00\x01"


def test_bytes_read_back_from_storage_once_then_cached():
    b = _Bucket()
    s = _store(b)
    s.add_cad_file(_cad())
    s.save()
    pm._CAD_BYTES_CACHE.clear()
    c = s.cad_files[0]
    assert s.cad_bytes(c) == b"solid part\x00\x01" and s.cad_bytes(c) == b"solid part\x00\x01"
    assert b.downloads == 1


def test_failed_upload_keeps_the_file_embedded():
    s = _store(_Bucket(fail=True))
    s.add_cad_file(_cad())
    assert s.save()
    row = s.backend.written["cad_files"][0]
    assert row["data_b64"] and row["storage_path"] == ""
    assert "policy denied" in s.cad_offload_error


def test_local_backend_keeps_bytes_embedded(tmp_path):
    s = pm.ProjectStore(path=str(tmp_path / "p.json"))
    s.add_cad_file(_cad())
    assert s.save() and s.cad_files[0].data_b64 and s.cad_files[0].storage_path == ""
    assert s.cad_bytes(s.cad_files[0]) == b"solid part\x00\x01"


def test_links_and_missing_files_have_no_bytes():
    s = _store(None)
    assert s.cad_bytes(pm.CADFile(name="asm", kind="link", link="https://x")) is None
    assert s.cad_bytes(pm.CADFile(name="a.step", kind="file", storage_path=WS + "/x")) is None


def test_old_documents_without_the_pointer_still_load(tmp_path):
    s = pm.ProjectStore(path=str(tmp_path / "p.json"))
    s._apply({"cad_files": [{"name": "a.stl", "kind": "file", "data_b64": "AA==", "id": "1"}]})
    assert s.cad_files[0].storage_path == "" and s.cad_bytes(s.cad_files[0]) == b"\x00"
