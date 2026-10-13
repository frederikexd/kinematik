# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Re-run published manifests on the current code and report whether the outputs match the recorded ones (every
coordinate to 1e-6 mm, every fit and yield to 1e-9; supplement S13.3), with the grade and build yield the re-run
returned. usage: python scripts/verify_run.py manifests/<name>.recorded.genesis.json [...]"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from suspension.genesis_repro import GenesisManifest                                    # noqa: E402

if __name__ == "__main__":
    for p in sys.argv[1:]:
        m = GenesisManifest.from_json(Path(p).read_text())
        t = time.time()
        res = m.run()
        ok, diff = m.verify(res)
        w = res.winner
        grade = f"{w.verdict}, build yield {100 * w.yield_frac:.2f}%" if w is not None and w.yield_frac is not None \
            else (w.verdict if w is not None else "NO_FIT")
        print(f"{m.name} (inputs {m.inputs_sha256[:12]}): {grade}; recorded "
              f"{m.recorded.get('winner_verdict') or 'NO_FIT'}; matches the recorded outputs: {ok}; "
              f"{round(time.time() - t)} s", flush=True)
        for d in diff:
            print("   ", d)
