# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Extract addendum 4's reference from the Stanford parallel-module dataset (doi:10.17632/zh58byr53c.1, CC BY 4.0).

    pip install mat-io
    python scripts/parallel_module_reference.py <Processed_data dir> <1_Single_cell_characterisation dir>

Writes benchmarks/electrical/parallel_modules/d_stanford_modules.json: per test, the cells and their HPPC 10 s
resistance, the measured branch currents at 10 s and averaged over the 0.75C discharge step, and every cell's
current and temperature on that step at 10 s intervals. Nothing here is compared with KinematiK; the tests do that.
"""
from __future__ import annotations

import json
import re
import sys
from pathlib import Path

import numpy as np
import scipy.io as sio

OUT = Path(__file__).resolve().parents[1] / "benchmarks" / "electrical" / "parallel_modules"


def hppc_r10(path: Path) -> float:
    """10 s discharge-pulse resistance (ohm) at the highest state of charge pulsed: the first pulse step."""
    m = sio.loadmat(path, squeeze_me=True)
    cur, volt, step = m["CurrentData"], m["VoltageData"], m["StepIndex"]
    edges = np.r_[0, np.where(np.diff(step) != 0)[0] + 1, step.size]
    for a, b in zip(edges[:-1], edges[1:]):
        if b - a > 50 and -3.5 < np.mean(cur[a:b]) < -1.5 and 9.0 < (b - a) / 10.0 < 11.0:
            return float((volt[a - 1] - volt[b - 1]) / -np.mean(cur[a:b]))
    raise ValueError(f"no 10 s discharge pulse in {path.name}")


def main(processed: str, single: str) -> None:
    from matio import load_from_mat
    r10 = {p.stem.split("_")[-1]: hppc_r10(p) for p in sorted(Path(single).rglob("HPPC_MultiSine_*.mat"))
           if "MACOSX" not in str(p)}
    tests = []
    for p in sorted(Path(processed).rglob("M*_*.mat")):
        if "MACOSX" in str(p):
            continue
        mix, age, r_ic, temp = re.match(r"M\d_(\w+?)_(Unaged|Aged)_R(\d)_T(\d+)", p.stem).groups()
        d = load_from_mat(p)["Data_processed"]
        df, names = d["Data"][0, 0], [str(x) for x in d["Cells_name"][0, 0].iloc[0].tolist()]
        s = df["Step_Index"].to_numpy()
        tot = df["CurrentA"].to_numpy()
        on = np.where((s == 5) & (tot < -1.0))[0]
        st = df["Step_Times"].to_numpy()[on]
        cells = np.array([df[f"CurrentA_Cell_{k}"].to_numpy()[on] for k in range(1, 5)])
        sign = -1.0 if np.mean(cells) < 0 else 1.0
        cells = cells * sign                                           # discharge positive
        temps = np.array([df[f"TemperatureC_Cell_{k}"].to_numpy()[on] for k in range(1, 5)])
        amb = df["Ambient_TemperatureC"].to_numpy()[on]
        at10 = (st >= 9.0) & (st <= 11.0)
        grid = np.arange(0.0, st[-1], 10.0)
        tests.append({
            "file": p.stem, "mix": mix, "aged": age == "Aged", "r_interconnect_mohm": int(r_ic),
            "chamber_c": int(temp), "cells": names, "r10_ohm": [r10[c] for c in names],
            "i_total_a": float(-np.mean(tot[on])), "i_cell_10s_a": cells[:, at10].mean(axis=1).tolist(),
            "i_cell_mean_a": cells.mean(axis=1).tolist(), "t_s": grid.tolist(),
            "i_cell_a": [np.interp(grid, st, c).round(4).tolist() for c in cells],
            "temp_c": [np.interp(grid, st, c).round(3).tolist() for c in temps],
            "ambient_c": np.interp(grid, st, amb).round(3).tolist(),
        })
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "d_stanford_modules.json").write_text(json.dumps({
        "source": "Piombo et al., Data in Brief 2024; Mendeley Data doi:10.17632/zh58byr53c.1 (CC BY 4.0), "
                  "Stanford Energy Control Lab. Extracted by scripts/parallel_module_reference.py.",
        "hppc_r10_ohm": r10, "tests": tests}, separators=(",", ":")), encoding="utf-8")
    print(f"{len(tests)} tests, {len(r10)} cells")


if __name__ == "__main__":  # pragma: no cover
    main(*sys.argv[1:3])
