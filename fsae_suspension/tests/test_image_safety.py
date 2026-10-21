# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Holds true what .github/workflows/security.yml relies on to ignore the Pillow advisories:
KinematiK opens no image itself, and every mesh load first limits Pillow to PNG and JPEG."""
import io
import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SOURCES = [p for p in ROOT.rglob("*.py")
           if "tests" not in p.parts and "__pycache__" not in p.parts]


def test_no_module_imports_pillow_except_the_guard():
    hits = [str(p.relative_to(ROOT)) for p in SOURCES
            if p.name != "image_safety.py"
            and re.search(r"^\s*(from PIL\b|import PIL\b)", p.read_text(encoding="utf-8"), re.M)]
    assert hits == []


def test_every_mesh_load_in_the_package_is_guarded():
    unguarded = []
    for p in (ROOT / "suspension").rglob("*.py"):
        lines = p.read_text(encoding="utf-8").splitlines()
        for i, line in enumerate(lines):
            if re.search(r"\btrimesh\.load\(", line) and "restrict_image_decoders()" not in lines[i - 1]:
                unguarded.append(f"{p.relative_to(ROOT)}:{i + 1}")
    assert unguarded == []


# The team CAD library stores a published file as bytes and hands the same bytes back on download; it
# never decodes one, so images are allowed there and nowhere else.
STORE_ONLY_UPLOADERS = {"cad_pub_file"}


def test_only_the_store_only_library_accepts_image_types():
    image = {"png", "jpg", "jpeg", "gif", "tif", "tiff", "bmp", "webp", "eps", "ppm", "pgm", "pdf"}
    for p in SOURCES:
        for m in re.finditer(r"file_uploader\((.*?)\)", p.read_text(encoding="utf-8"), re.S):
            key = re.search(r'key="(\w+)"', m.group(1))
            if key and key.group(1) in STORE_ONLY_UPLOADERS:
                continue
            types = set(re.findall(r'"(\w+)"', m.group(1).split("type=", 1)[-1]))
            assert not (types & image), f"{p.relative_to(ROOT)} accepts {types & image}"


def test_the_app_never_renders_an_image():
    hits = [str(p.relative_to(ROOT)) for p in SOURCES if re.search(r"\bst\.image\(", p.read_text(encoding="utf-8"))]
    assert hits == []


def test_guard_refuses_other_decoders_and_keeps_png():
    pil = pytest.importorskip("PIL.Image")
    from PIL import TiffImagePlugin  # noqa: F401  (make sure the TIFF writer exists before the guard runs)

    tiff, png = io.BytesIO(), io.BytesIO()
    pil.new("RGB", (4, 4)).save(tiff, format="TIFF")
    pil.new("RGB", (4, 4)).save(png, format="PNG")

    from suspension.image_safety import restrict_image_decoders
    restrict_image_decoders()
    pil.init()                                           # a later init() must not bring decoders back
    assert set(pil.OPEN) <= {"PNG", "JPEG"}
    assert "PDF" not in pil.SAVE
    tiff.seek(0)
    with pytest.raises(pil.UnidentifiedImageError):
        pil.open(tiff)
    png.seek(0)
    assert pil.open(png).size == (4, 4)
