# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Limit Pillow to the image formats KinematiK needs.

KinematiK never opens an image itself, but an uploaded GLB or OBJ can carry
embedded textures, and trimesh hands those to Pillow, which picks a decoder
from the file's bytes rather than its name. Pillow's TIFF, GIF, PPM/PGM and
EPS decoders and its PDF append path have open advisories fixed only in
Pillow 13, which Streamlit does not yet allow. Textures in mesh files are PNG
or JPEG, so every other decoder is removed before a mesh is loaded, and a
texture in another format is refused as unreadable instead of being decoded.

Writers are left alone (matplotlib and Streamlit save PNG and JPEG), except
the PDF writer, whose append mode parses an existing file.
"""
from __future__ import annotations

ALLOWED_DECODERS = frozenset({"PNG", "JPEG"})

_done = False


def restrict_image_decoders() -> None:
    """Idempotent. Safe to call when Pillow is not installed."""
    global _done
    if _done:
        return
    try:
        from PIL import Image
    except ImportError:          # no Pillow, no decoders to restrict
        _done = True
        return
    Image.init()                 # load every plugin now so a later init() cannot re-add one
    for fmt in list(Image.OPEN):
        if fmt not in ALLOWED_DECODERS:
            del Image.OPEN[fmt]
    Image.ID[:] = [fmt for fmt in Image.ID if fmt in ALLOWED_DECODERS]
    for registry in (Image.SAVE, Image.SAVE_ALL):
        registry.pop("PDF", None)
    _done = True
