# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The one place an input becomes a float array, for every module that needs it."""
from __future__ import annotations

import numpy as np


def vec(v) -> np.ndarray:
    """``v`` as a float array (no copy when it already is one); units are unchanged, so a point in mm stays in mm."""
    return np.asarray(v, float)
